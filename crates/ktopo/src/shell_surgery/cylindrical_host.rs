//! Discovery-only adapter for convex planar hosts with circular product sweeps.
//!
//! This module inspects live geometry and topology and proposes possible role
//! assignments. It does not call proof predicates and cannot construct a shell
//! verdict; the parent theorem independently re-verifies every proposed field.

use super::*;
use crate::entity::FinId;

pub(super) fn discover(store: &Store, shell_id: ShellId) -> Result<Vec<ShellSurgeryEvidence>> {
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(Vec::new());
    }
    let mut planar = Vec::new();
    let mut cylinders = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(Vec::new());
        }
        match store.get(face.surface)? {
            SurfaceGeom::Plane(_) => planar.push(face_id),
            SurfaceGeom::Cylinder(cylinder) => cylinders.push((face_id, *cylinder)),
            _ => return Ok(Vec::new()),
        }
    }
    if cylinders.is_empty() {
        return Ok(Vec::new());
    }

    let mut facets = Vec::new();
    for face_id in planar {
        let face = store.get(face_id)?;
        let mut outer_candidates = Vec::new();
        for &loop_id in &face.loops {
            let loop_ = store.get(loop_id)?;
            if loop_.fins.len() < 3 {
                continue;
            }
            let mut vertices = Vec::with_capacity(loop_.fins.len());
            let mut complete = true;
            for &fin in &loop_.fins {
                match store.fin_tail(fin)? {
                    Some(vertex) => vertices.push(vertex),
                    None => {
                        complete = false;
                        break;
                    }
                }
            }
            if complete {
                outer_candidates.push((loop_id, vertices));
            }
        }
        if let [(outer_loop, vertices)] = outer_candidates.as_slice() {
            facets.push(PlanarFacetEvidence {
                face: face_id,
                outer_loop: *outer_loop,
                vertices: vertices.clone(),
            });
        }
    }
    if facets.len() < 4 {
        return Ok(Vec::new());
    }
    let base = PlanarBaseEvidence { facets };

    let mut features = Vec::with_capacity(cylinders.len());
    for (side_face, cylinder) in cylinders {
        let face = store.get(side_face)?;
        let [first_loop, second_loop] = face.loops.as_slice() else {
            return Ok(Vec::new());
        };
        let Some(first) = discover_attachment(store, &base, *first_loop)? else {
            return Ok(Vec::new());
        };
        let Some(second) = discover_attachment(store, &base, *second_loop)? else {
            return Ok(Vec::new());
        };
        let profiles = [first, second];
        let translation = second.profile.frame().origin() - first.profile.frame().origin();
        let interval = profiles.map(|profile| {
            (profile.profile.frame().origin() - cylinder.frame().origin()).dot(cylinder.frame().z())
        });
        let supports = base
            .facets
            .iter()
            .map(|facet| {
                let side = profiles
                    .iter()
                    .enumerate()
                    .find_map(|(index, endpoint)| {
                        (endpoint.role == EndpointRole::Port && endpoint.planar_face == facet.face)
                            .then_some(SupportSide::IncidentAt(index))
                    })
                    .unwrap_or(SupportSide::StrictInterior);
                SupportReference {
                    face: facet.face,
                    side,
                }
            })
            .collect();
        features.push(ProductSweepEvidence {
            side_face,
            cylinder,
            profiles,
            translation,
            interval,
            supports,
            role: FeatureRole::Through,
        });
    }

    let mut relations = Vec::new();
    for first in 0..features.len() {
        for second in first + 1..features.len() {
            let direction = features[first].cylinder.frame().z();
            let origin = features[first].profiles[0].profile.frame().origin();
            let range = |feature: &ProductSweepEvidence| {
                let values = feature
                    .profiles
                    .map(|profile| (profile.profile.frame().origin() - origin).dot(direction));
                [values[0].min(values[1]), values[0].max(values[1])]
            };
            relations.push(PairwiseRelationEvidence::Strict(StrictSeparationEvidence {
                first,
                second,
                direction,
                origin,
                first_range: range(&features[first]),
                second_range: range(&features[second]),
            }));
        }
    }

    let mut proposals = Vec::new();
    expand_feature_roles(
        shell_id,
        &base,
        &features,
        &relations,
        0,
        &mut Vec::new(),
        &mut proposals,
    );
    Ok(proposals)
}

fn expand_feature_roles(
    shell: ShellId,
    base: &PlanarBaseEvidence,
    features: &[ProductSweepEvidence],
    relations: &[PairwiseRelationEvidence],
    index: usize,
    selected: &mut Vec<ProductSweepEvidence>,
    proposals: &mut Vec<ShellSurgeryEvidence>,
) {
    if index == features.len() {
        proposals.push(ShellSurgeryEvidence {
            shell,
            base: base.clone(),
            features: selected.clone(),
            relations: relations.to_vec(),
        });
        return;
    }
    for role in [FeatureRole::Through, FeatureRole::Boss, FeatureRole::Pocket] {
        let mut feature = features[index].clone();
        feature.role = role;
        selected.push(feature);
        expand_feature_roles(
            shell,
            base,
            features,
            relations,
            index + 1,
            selected,
            proposals,
        );
        selected.pop();
    }
}

fn discover_attachment(
    store: &Store,
    base: &PlanarBaseEvidence,
    side_loop: LoopId,
) -> Result<Option<AttachmentLoopEvidence>> {
    let loop_ = store.get(side_loop)?;
    let [side_fin] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let fin = store.get(*side_fin)?;
    let edge = store.get(fin.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let peer: FinId = if first == side_fin {
        *second
    } else if second == side_fin {
        *first
    } else {
        return Ok(None);
    };
    let peer_fin = store.get(peer)?;
    let planar_loop = peer_fin.parent;
    let planar_face = store.get(planar_loop)?.face;
    let Some(curve) = edge.curve else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store.get(curve)? else {
        return Ok(None);
    };
    let role = if base.facets.iter().any(|facet| facet.face == planar_face) {
        EndpointRole::Port
    } else {
        EndpointRole::Cap
    };
    Ok(Some(AttachmentLoopEvidence {
        side_loop,
        planar_face,
        planar_loop,
        edge: fin.edge,
        profile: *circle,
        role,
    }))
}
