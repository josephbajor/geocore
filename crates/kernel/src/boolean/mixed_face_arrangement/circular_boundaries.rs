//! Split topology-owned inner circles and embed their transverse cut graph.

use super::super::face_arrangement::preview_bounded_surface_cycles;
use super::*;

pub(super) fn append_split_rings(
    store: &Store,
    face: RawFaceId,
    roots: &[BoundaryRootEvidence],
    split: &mut SplitSourceBoundary,
) -> Result<usize, MixedFaceArrangementError> {
    let fail = || MixedFaceArrangementError::MultipleSourceLoops;
    let polygon = source_rings::polygon_loop(store, face)?;
    let raw_face = store.get(face).map_err(|_| fail())?;
    let mut fin_ordinal = store.get(polygon).map_err(|_| fail())?.fins().len();
    let mut count = 0;
    for &loop_id in raw_face.loops() {
        if loop_id == polygon {
            continue;
        }
        let ring = store.get(loop_id).map_err(|_| fail())?;
        let [fin_id] = ring.fins() else {
            return Err(fail());
        };
        let fin = store.get(*fin_id).map_err(|_| fail())?;
        let mut ordered = roots
            .iter()
            .filter(|root| root.loop_id == loop_id)
            .collect::<Vec<_>>();
        if ordered.is_empty() {
            fin_ordinal += 1;
            continue;
        }
        ordered.sort_by_key(|root| root.key.ordinal);
        if ordered.len() < 2
            || ordered.iter().enumerate().any(|(ordinal, root)| {
                root.key.edge != fin.edge()
                    || root.fin != *fin_id
                    || root.key.ordinal != ordinal
                    || root.interval.lo <= 0.0
                    || root.interval.hi >= core::f64::consts::TAU
            })
            || ordered
                .windows(2)
                .any(|pair| pair[0].interval.hi >= pair[1].interval.lo)
        {
            return Err(fail());
        }
        if fin.sense() == Sense::Reversed {
            ordered.reverse();
        }
        for traversal_ordinal in 0..ordered.len() {
            let pair = [
                ordered[traversal_ordinal],
                ordered[(traversal_ordinal + 1) % ordered.len()],
            ];
            let shifts = if fin.sense() == Sense::Forward {
                [0, i32::from(pair[1].interval.hi < pair[0].interval.lo)]
            } else {
                [i32::from(pair[0].interval.hi < pair[1].interval.lo), 0]
            };
            let key = MixedSourceSpanKey {
                fin_loop_ordinal: fin_ordinal,
                traversal_ordinal,
            };
            split.spans.push(DirectedSourceSpan::new(
                key.clone(),
                MixedArrangementVertex::SectionEndpoint(pair[0].endpoint),
                MixedArrangementVertex::SectionEndpoint(pair[1].endpoint),
            ));
            split.lineage.push(MixedSourceSpanLineage {
                key,
                loop_id,
                fin: *fin_id,
                edge: fin.edge(),
                range: core::array::from_fn(|end| MixedSourceParameterEvidence::SectionRoot {
                    endpoint: pair[end].endpoint,
                    root_ordinal: pair[end].key.ordinal,
                    enclosure_bits: [
                        pair[end].interval.lo.to_bits(),
                        pair[end].interval.hi.to_bits(),
                    ],
                    period_shift: shifts[end],
                }),
            });
        }
        count += 1;
        fin_ordinal += 1;
    }
    if roots
        .iter()
        .any(|root| !raw_face.loops().contains(&root.loop_id))
    {
        return Err(fail());
    }
    Ok(count)
}

/// A complete circular Section on a planar face with disjoint inner source
/// circles supplies simple, noncrossing arcs. If it never meets the outer
/// polygon, its exterior material remains connected. The carrier orientation
/// labels every cut side; each interior cycle bounds a distinct disk cell,
/// while exterior cycles share the polygon's cell. The common surface core
/// independently checks source sides, cut pairing, connectivity, and Euler
/// characteristic. No ring or fragment count chooses a topology template.
pub(super) fn arrange(
    store: &Store,
    face: RawFaceId,
    cuts: &[FaceCutEvidence],
    polygon_anchor: MixedSourceSpanKey,
    input: FaceArrangementInput<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>,
    split_rings: usize,
) -> Result<MixedPlanarFaceArrangement, MixedFaceArrangementError> {
    let fail = || MixedFaceArrangementError::MultipleSourceLoops;
    let polygon = source_rings::polygon_loop(store, face)?;
    let first = cuts.first().ok_or_else(fail)?;
    let CutEmbedding::Circle {
        branch,
        orientation,
        ..
    } = first.embedding
    else {
        return Err(fail());
    };
    if cuts.iter().any(|cut| !matches!(cut.embedding, CutEmbedding::Circle { branch: b, orientation: o, .. } if b == branch && o == orientation)
        || cut.endpoints.iter().any(|end| end.boundary_root.as_ref().is_none_or(|root| root.loop_id == polygon))) {
        return Err(fail());
    }
    let forward_inside = orientation == source_boundary_orientation(store, face)?;
    let cycles =
        preview_bounded_surface_cycles(&input).map_err(MixedFaceArrangementError::Arrangement)?;
    let mut assignments = Vec::new();
    let mut cells = Vec::new();
    let mut outside_cycles = 0_i64;
    let mut polygon_found = false;
    for cycle in &cycles {
        let mut side = None;
        let mut exterior = false;
        let mut polygon_side = false;
        for use_ in cycle.uses() {
            match use_.edge() {
                ArrangementEdgeKey::Source(source) => {
                    exterior |= use_.direction() == ArrangementDirection::Reverse;
                    polygon_side |= *source == polygon_anchor
                        && use_.direction() == ArrangementDirection::Forward;
                }
                ArrangementEdgeKey::Cut(_) => {
                    let inside =
                        (use_.direction() == ArrangementDirection::Forward) == forward_inside;
                    if side
                        .replace(inside)
                        .is_some_and(|previous| previous != inside)
                    {
                        return Err(fail());
                    }
                }
            }
        }
        let owner = if exterior {
            if side.is_some() || polygon_side {
                return Err(fail());
            }
            CertifiedCycleSide::Exterior
        } else if side == Some(true) {
            if polygon_side {
                return Err(fail());
            }
            let key = cells.len() + 1;
            cells.push(CertifiedCellTopology::new(key, 1));
            CertifiedCycleSide::Cell(key)
        } else {
            if side.is_none() && !polygon_side {
                return Err(fail());
            }
            polygon_found |= polygon_side;
            outside_cycles += 1;
            CertifiedCycleSide::Cell(0)
        };
        assignments.push(CertifiedCycleAssignment::new(
            cycle.uses().first().ok_or_else(fail)?.clone(),
            owner,
        ));
    }
    if !polygon_found {
        return Err(fail());
    }
    cells.push(CertifiedCellTopology::new(0, 2 - outside_cycles));
    let chi = 1_i64
        .checked_sub(i64::try_from(split_rings).map_err(|_| fail())?)
        .ok_or_else(fail)?;
    let arranged = arrange_bounded_surface(
        input,
        CertifiedSurfaceEmbedding::new(assignments, cells, chi),
    )
    .map_err(MixedFaceArrangementError::EmbeddedArrangement)?;
    Ok(normalize_embedded_arrangement(arranged))
}
