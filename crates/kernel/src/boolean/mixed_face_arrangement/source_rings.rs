//! Assign uncut source holes to certified cells of the new arrangement.

use super::*;
use kcore::interval::Interval;
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};

pub(super) fn polygon_loop(
    store: &Store,
    face: RawFaceId,
) -> Result<RawLoopId, MixedFaceArrangementError> {
    let face = store
        .get(face)
        .map_err(|_| MixedFaceArrangementError::MissingSourceFace)?;
    let mut polygon = None;
    for &loop_id in face.loops() {
        let ring = store
            .get(loop_id)
            .map_err(|_| MixedFaceArrangementError::MissingSourceLoop)?;
        if ring.fins().len() >= 3 {
            if polygon.replace(loop_id).is_some() {
                return Err(MixedFaceArrangementError::MultipleSourceLoops);
            }
        } else if ring.fins().len() != 1 {
            return Err(MixedFaceArrangementError::MultipleSourceLoops);
        }
    }
    polygon.ok_or(MixedFaceArrangementError::EmptySourceLoop)
}

pub(super) struct UncutRing {
    loop_id: RawLoopId,
    containers: BTreeSet<usize>,
}

/// Uncut rings may lie outside a cut disk or strictly inside it. Rings with
/// certified Section roots are delegated to the split-boundary arrangement;
/// tangency and unresolved interval comparisons remain unsupported.
pub(super) fn admit_rings(
    store: &Store,
    face_id: RawFaceId,
    cuts: &[FaceCutEvidence],
    roots: &[BoundaryRootEvidence],
) -> Result<Vec<UncutRing>, MixedFaceArrangementError> {
    let fail = || MixedFaceArrangementError::MultipleSourceLoops;
    let polygon = polygon_loop(store, face_id)?;
    let face = store.get(face_id).map_err(|_| fail())?;
    let mut rings = Vec::new();
    for &loop_id in face.loops() {
        if loop_id == polygon {
            continue;
        }
        let ring = store.get(loop_id).map_err(|_| fail())?;
        let [fin_id] = ring.fins() else {
            return Err(fail());
        };
        let fin = store.get(*fin_id).map_err(|_| fail())?;
        let use_ = fin.pcurve().ok_or_else(fail)?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        let Curve2dGeom::Circle(circle) = store.pcurve(use_.curve()).map_err(|_| fail())? else {
            return Err(fail());
        };
        if fin.sense().times(use_.sense()) != face.sense().flipped()
            || edge.vertices() != [None, None]
            || edge.bounds().is_some()
            || edge.tolerance().is_some()
            || use_.seam().is_some()
            || !use_.chart().is_identity()
            || use_.range().width() != core::f64::consts::TAU
            || use_.edge_to_pcurve().scale().abs() != 1.0
            || use_.closure_winding() != Some([0, 0])
            || certify_whole_fin_incidence(
                store,
                face_id,
                loop_id,
                *fin_id,
                kcore::tolerance::LINEAR_RESOLUTION,
            ) != WholeFinIncidence::Certified
        {
            return Err(fail());
        }
        if roots.iter().any(|root| root.loop_id == loop_id) {
            continue;
        }
        let mut containers = BTreeSet::new();
        for cut in cuts {
            let (CutEmbedding::WholeCircle {
                center,
                radius,
                x_direction,
                ..
            }
            | CutEmbedding::Circle {
                center,
                radius,
                x_direction,
                ..
            }) = cut.embedding
            else {
                return Err(fail());
            };
            let x = Interval::point(center[0]) - Interval::point(circle.center().x);
            let y = Interval::point(center[1]) - Interval::point(circle.center().y);
            let effective_radius = |radius: f64, x: [f64; 2]| {
                (Interval::point(x[0]).square() + Interval::point(x[1]).square())
                    .sqrt()
                    .map(|gram| Interval::point(radius) * gram)
                    .ok_or_else(fail)
            };
            let cut_radius = effective_radius(radius, x_direction)?;
            let source_radius =
                effective_radius(circle.radius(), [circle.x_dir().x, circle.x_dir().y])?;
            let distance = x.square() + y.square();
            let sum = cut_radius + source_radius;
            if distance.lo() > sum.square().hi() {
                continue;
            }
            let margin = cut_radius - source_radius;
            if margin.lo() > 0.0 && distance.hi() < margin.square().lo() {
                containers.insert(cut.key.branch());
            } else {
                return Err(fail());
            }
        }
        rings.push(UncutRing {
            loop_id,
            containers,
        });
    }
    Ok(rings)
}

/// Label the certified dual from its polygon-boundary cell. Crossing a whole
/// cut toggles that cut's membership; contradictory or disconnected labels
/// refuse. A source ring belongs to the unique cell with its proven membership.
pub(super) fn assign_to_cells(
    rings: Vec<UncutRing>,
    arrangement: &MixedPlanarFaceArrangement,
) -> Result<Vec<(RawLoopId, usize)>, MixedFaceArrangementError> {
    if rings.is_empty() {
        return Ok(Vec::new());
    }
    let fail = || MixedFaceArrangementError::MultipleSourceLoops;
    let polygon_anchor = arrangement.source_spans().first().ok_or_else(fail)?.key();
    let mut exterior = arrangement.cells().iter().filter(|cell| {
        cell.boundaries().iter().any(|boundary| {
            boundary
                .uses()
                .iter()
                .any(|use_| matches!(use_.edge(), ArrangementEdgeKey::Source(key) if key == polygon_anchor))
        })
    });
    let exterior = exterior
        .next()
        .filter(|_| exterior.next().is_none())
        .ok_or_else(fail)?
        .key();
    let mut labels = BTreeMap::from([(exterior, BTreeSet::new())]);
    let mut pending = std::collections::VecDeque::from([exterior]);
    while let Some(cell) = pending.pop_front() {
        for edge in arrangement.adjacency() {
            let peer = if edge.forward_cell() == cell {
                edge.reverse_cell()
            } else if edge.reverse_cell() == cell {
                edge.forward_cell()
            } else {
                continue;
            };
            let mut label = labels[&cell].clone();
            if !label.remove(&edge.cut().branch()) {
                label.insert(edge.cut().branch());
            }
            if let Some(existing) = labels.get(&peer) {
                if existing != &label {
                    return Err(fail());
                }
            } else {
                labels.insert(peer, label);
                pending.push_back(peer);
            }
        }
    }
    if labels.len() != arrangement.cells().len() {
        return Err(fail());
    }
    rings
        .into_iter()
        .map(|ring| {
            let mut owners = labels
                .iter()
                .filter(|(_, label)| **label == ring.containers);
            let (&cell, _) = owners.next().ok_or_else(fail)?;
            if owners.next().is_some() {
                return Err(fail());
            }
            Ok((ring.loop_id, cell))
        })
        .collect()
}
