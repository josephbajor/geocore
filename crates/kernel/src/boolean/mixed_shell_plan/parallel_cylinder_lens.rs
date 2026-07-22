//! Mixed-shell plan for strict cylinder lenses with coincident cap cells.
//!
//! The global Section graph remains indeterminate because coincident planes
//! are two-dimensional contact.  The operation-local relation accounts for
//! those gaps, while sealed periodic arrangements select both side patches.
//! Each planar cap loop is then the reversed chain of the two exact physical
//! end uses already present on the selected sides.  This preserves one edge,
//! one endpoint pair, and one period lift across the side/cap pair.

use std::collections::BTreeMap;

use super::*;
use crate::SectionCompletion;
use crate::boolean::parallel_cylinder_boundary::{
    CoincidentCapBoundaryPiece, ParallelCoincidentBoundaryKey, ParallelCoincidentBoundaryPayload,
    SelectedCoincidentCapBoundaryUse, SelectedCoincidentCapPlan,
    SelectedParallelCylinderCoincidentBoundary,
};
use crate::boolean::parallel_cylinder_relation::CertifiedParallelCylinderCoincidentCapRelation;
use crate::boolean::parallel_cylinder_relation::ParallelCylinderSourceRootWitness;

struct ReversedBoundarySegment {
    use_: MixedShellEdgeUse,
    start: MixedShellVertexKey,
    end: MixedShellVertexKey,
}

/// Build one regularized Boolean plan under the exact coincident-cap relation.
pub(crate) fn plan_parallel_cylinder_coincident_boolean<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: SelectedParallelCylinderCoincidentBoundary,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    linear: f64,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    if graph.completion() != SectionCompletion::Indeterminate || graph.gaps().is_empty() {
        return Err(MixedShellPlanError::SectionIncomplete);
    }
    let (selected, cap_plans) = selected.into_parts();
    let mut arranged = Vec::new();
    let mut caps = BTreeMap::new();
    for selected in selected {
        let (key, operand, payload, orientation) = selected.into_parts();
        match (key, payload) {
            (
                ParallelCoincidentBoundaryKey::Arranged(cell),
                ParallelCoincidentBoundaryPayload::Arranged,
            ) if operand == operand_side(cell.source().operand()) => {
                arranged.push((cell, operand, orientation));
            }
            _ => return Err(MixedShellPlanError::CoincidentCapSelectionMismatch),
        }
    }
    for cap in cap_plans {
        let physical_end = cap.target().physical_end();
        if caps.insert(physical_end, cap).is_some() {
            return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
        }
    }
    if arranged
        .iter()
        .filter(|(cell, _, _)| matches!(cell.cell(), MixedShellCellKind::Periodic(_)))
        .count()
        != 2
        || caps.len() != relation.overlap_ends().len()
    {
        return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
    }
    for (physical_end, end) in relation.overlap_ends().iter().enumerate() {
        let cap = caps
            .get(&physical_end)
            .ok_or(MixedShellPlanError::CoincidentCapSelectionMismatch)?;
        validate_cap_selection(store, graph, cap, physical_end, end)?;
    }

    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::CoincidentCaps(relation),
        bindings,
        arranged,
        |faces, spans| append_cap_faces(store, graph, &caps, faces, spans, linear),
    )
}

fn validate_cap_selection(
    store: &Store,
    graph: &BodySectionGraph,
    plan: &SelectedCoincidentCapPlan,
    physical_end: usize,
    end: &crate::boolean::parallel_cylinder_relation::ParallelCylinderCoincidentCapEndWitness,
) -> Result<(), MixedShellPlanError> {
    let cap = plan.target();
    let source = end
        .source(cap.target_operand())
        .ok_or(MixedShellPlanError::CoincidentCapSelectionMismatch)?;
    if !matches!(
        plan.owner_key(),
        ParallelCoincidentBoundaryKey::CapEnd(value)
            | ParallelCoincidentBoundaryKey::CapRemainder(value)
            if value == physical_end
    ) || cap.physical_end() != physical_end
        || cap.target_boundary() != source.boundary()
        || cap.target_face().raw() != source.cap_face()
        || source_face_key(store, graph, cap.target_face(), cap.target_operand())?
            != cap.target_source()
    {
        return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
    }
    let mut expected = end
        .sources()
        .iter()
        .flatten()
        .map(|source| CoincidentCapBoundaryPiece::SourceArc {
            operand: source.operand(),
            edge: source.edge(),
            roots: source.roots(),
        })
        .collect::<Vec<_>>();
    if let Some(arc) = end.cap_arc() {
        expected.push(CoincidentCapBoundaryPiece::SectionArc {
            fragment: arc.fragment(),
            endpoints: arc.endpoints(),
        });
    }
    let actual = plan.boundary().map(|use_| match use_ {
        SelectedCoincidentCapBoundaryUse::SourceSpan {
            operand,
            edge,
            roots,
            ..
        } => CoincidentCapBoundaryPiece::SourceArc {
            operand,
            edge,
            roots,
        },
        SelectedCoincidentCapBoundaryUse::SectionArc {
            fragment,
            endpoints,
            ..
        } => CoincidentCapBoundaryPiece::SectionArc {
            fragment,
            endpoints,
        },
    });
    if expected.as_slice() != cap.boundary() || actual != *cap.boundary() {
        return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
    }
    Ok(())
}

fn append_cap_faces(
    store: &Store,
    graph: &BodySectionGraph,
    caps: &BTreeMap<usize, SelectedCoincidentCapPlan>,
    faces: &mut Vec<MixedShellFacePlan>,
    spans: &mut Vec<MixedBoundedSourceSpanPlan>,
    linear: f64,
) -> Result<(), MixedShellPlanError> {
    for (&physical_end, plan) in caps {
        let cap = plan.target();
        if faces
            .iter()
            .any(|face| face.source() == cap.target_source())
        {
            return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
        }
        let mut segments = Vec::with_capacity(cap.boundary().len());
        for use_ in plan.boundary() {
            segments.push(match *use_ {
                SelectedCoincidentCapBoundaryUse::SourceSpan {
                    operand,
                    edge,
                    roots,
                    side_cell,
                    span,
                    side_orientation,
                } => reversed_source_segment(
                    store,
                    cap,
                    physical_end,
                    operand,
                    edge,
                    roots,
                    side_cell,
                    span,
                    side_orientation,
                    faces,
                    spans,
                    linear,
                )?,
                SelectedCoincidentCapBoundaryUse::SectionArc {
                    fragment,
                    endpoints,
                    side_cell,
                    side_orientation,
                } => reversed_section_segment(
                    graph,
                    cap,
                    physical_end,
                    fragment,
                    endpoints,
                    side_cell,
                    side_orientation,
                    faces,
                )?,
            });
        }
        let loop_ = close_reversed_segments(physical_end, segments)?;
        faces.push(MixedShellFacePlan {
            source: cap.target_source(),
            source_face: cap.target_face().clone(),
            selected_orientation: plan.orientation(),
            loops: vec![loop_],
        });
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn reversed_source_segment(
    store: &Store,
    cap: &crate::boolean::parallel_cylinder_boundary::PreparedCoincidentCapCell,
    physical_end: usize,
    operand: usize,
    edge: RawEdgeId,
    roots: [ParallelCylinderSourceRootWitness; 2],
    side_cell: MixedShellCellKey,
    periodic_span: PeriodicSourceLoopKey,
    side_orientation: SelectedOrientation,
    faces: &[MixedShellFacePlan],
    spans: &[MixedBoundedSourceSpanPlan],
    linear: f64,
) -> Result<ReversedBoundarySegment, MixedShellPlanError> {
    let span_ordinal = periodic_span.cyclic_span_ordinal().ok_or(
        MixedShellPlanError::CoincidentCapBoundaryChain(physical_end),
    )?;
    let local_span = MixedSourceSpanKey {
        fin_loop_ordinal: periodic_span.topology_ordinal(),
        traversal_ordinal: span_ordinal,
    };
    let mut matches = Vec::new();
    for span in spans.iter().filter(|span| {
        matches!(side_cell.cell(), MixedShellCellKind::Periodic(_))
            && side_cell.source().operand() == operand
            && span.source() == side_cell.source()
            && span.span() == &local_span
            && span.edge() == edge
            && same_source_roots(span.roots(), &roots)
    }) {
        for (face_index, face) in faces.iter().enumerate() {
            if face.source() != side_cell.source()
                || face.selected_orientation() != side_orientation
            {
                continue;
            }
            for (loop_index, loop_) in face.loops().iter().enumerate() {
                for (use_index, use_) in loop_.uses().iter().enumerate() {
                    if use_.edge()
                        == &(MixedShellEdgeKey::PlanarSource {
                            source: span.source(),
                            span: span.span().clone(),
                        })
                    {
                        matches.push((span, face_index, loop_index, use_index));
                    }
                }
            }
        }
    }
    let [(span, face_index, loop_index, use_index)] = matches.as_slice() else {
        return Err(MixedShellPlanError::CoincidentCapBoundaryUseCount {
            physical_end,
            actual: matches.len(),
        });
    };
    let face = &faces[*face_index];
    let loop_ = &face.loops()[*loop_index];
    let use_ = &loop_.uses()[*use_index];
    let [tail, head] = adjacent_vertices(loop_, *use_index, physical_end)?;
    if face.source() != span.source()
        || !matches!(use_.pcurve(), MixedPcurveLineage::SourceTopology)
        || !section_vertices_match([tail, head], roots.map(|root| root.endpoint()))
    {
        return Err(MixedShellPlanError::CoincidentCapBoundaryChain(
            physical_end,
        ));
    }
    let proof = ProjectedSourceCircleOnPlane::certify(
        store,
        face.source_face(),
        span,
        cap.target_source(),
        cap.target_face(),
        linear,
    )
    .map_err(MixedShellPlanError::ProjectedSourceCircle)?;
    Ok(ReversedBoundarySegment {
        use_: MixedShellEdgeUse {
            edge: use_.edge().clone(),
            direction: opposite(use_.direction()),
            pcurve: MixedPcurveLineage::ProjectedSourceCircleOnPlane(proof),
        },
        start: head.clone(),
        end: tail.clone(),
    })
}

fn same_source_roots(
    actual: &[MixedBoundedSourceRoot; 2],
    expected: &[ParallelCylinderSourceRootWitness; 2],
) -> bool {
    let matches = |actual: MixedBoundedSourceRoot, expected: ParallelCylinderSourceRootWitness| {
        actual.endpoint() == expected.endpoint()
            && actual.root_ordinal() == expected.root_ordinal()
            && actual.parameter().to_bits() == expected.parameter().to_bits()
            && actual.enclosure().map(f64::to_bits) == expected.enclosure().map(f64::to_bits)
    };
    matches(actual[0], expected[0]) && matches(actual[1], expected[1])
        || matches(actual[0], expected[1]) && matches(actual[1], expected[0])
}

fn reversed_section_segment(
    graph: &BodySectionGraph,
    cap: &crate::boolean::parallel_cylinder_boundary::PreparedCoincidentCapCell,
    physical_end: usize,
    fragment: usize,
    endpoints: [usize; 2],
    side_cell: MixedShellCellKey,
    side_orientation: SelectedOrientation,
    faces: &[MixedShellFacePlan],
) -> Result<ReversedBoundarySegment, MixedShellPlanError> {
    let mut matches = Vec::new();
    for face in faces {
        if !matches!(side_cell.cell(), MixedShellCellKind::Periodic(_))
            || face.source() != side_cell.source()
            || face.selected_orientation() != side_orientation
        {
            continue;
        }
        for loop_ in face.loops() {
            for (use_index, use_) in loop_.uses().iter().enumerate() {
                if use_.edge() == &MixedShellEdgeKey::SectionFragment(fragment) {
                    matches.push((loop_, use_index, use_));
                }
            }
        }
    }
    let [(loop_, use_index, use_)] = matches.as_slice() else {
        return Err(MixedShellPlanError::CoincidentCapBoundaryUseCount {
            physical_end,
            actual: matches.len(),
        });
    };
    let section = graph
        .curve_fragments()
        .get(fragment)
        .ok_or(MixedShellPlanError::UnknownSectionFragment(fragment))?;
    let branch = graph.branches().get(section.branch()).ok_or(
        MixedShellPlanError::UnknownSectionBranch {
            fragment,
            branch: section.branch(),
        },
    )?;
    let [tail, head] = adjacent_vertices(loop_, *use_index, physical_end)?;
    if branch.faces()[cap.target_operand()] != *cap.target_face()
        || !matches!(use_.pcurve(), MixedPcurveLineage::Section { .. })
        || !section_vertices_match([tail, head], endpoints)
    {
        return Err(MixedShellPlanError::CoincidentCapBoundaryChain(
            physical_end,
        ));
    }
    Ok(ReversedBoundarySegment {
        use_: MixedShellEdgeUse {
            edge: MixedShellEdgeKey::SectionFragment(fragment),
            direction: opposite(use_.direction()),
            pcurve: MixedPcurveLineage::Section {
                branch: section.branch(),
                operand: cap.target_operand(),
                cylinder_period_shift: 0,
            },
        },
        start: head.clone(),
        end: tail.clone(),
    })
}

fn adjacent_vertices(
    loop_: &MixedShellLoopPlan,
    use_index: usize,
    physical_end: usize,
) -> Result<[&MixedShellVertexKey; 2], MixedShellPlanError> {
    loop_
        .vertices()
        .get(use_index..=use_index + 1)
        .and_then(|vertices| <&[_; 2]>::try_from(vertices).ok())
        .map(|vertices| [&vertices[0], &vertices[1]])
        .ok_or(MixedShellPlanError::CoincidentCapBoundaryChain(
            physical_end,
        ))
}

fn section_vertices_match(vertices: [&MixedShellVertexKey; 2], endpoints: [usize; 2]) -> bool {
    let [
        MixedShellVertexKey::SectionEndpoint(first),
        MixedShellVertexKey::SectionEndpoint(second),
    ] = vertices
    else {
        return false;
    };
    same_endpoint_pair([*first, *second], endpoints)
}

fn same_endpoint_pair(first: [usize; 2], second: [usize; 2]) -> bool {
    first == second || first == [second[1], second[0]]
}

fn close_reversed_segments(
    physical_end: usize,
    mut segments: Vec<ReversedBoundarySegment>,
) -> Result<MixedShellLoopPlan, MixedShellPlanError> {
    if segments.len() != 2 {
        return Err(MixedShellPlanError::CoincidentCapBoundaryUseCount {
            physical_end,
            actual: segments.len(),
        });
    }
    segments.sort_by(|left, right| left.use_.edge.cmp(&right.use_.edge));
    let first = segments.remove(0);
    let mut ordered = vec![first];
    while let Some(current) = ordered.last().map(|segment| segment.end.clone()) {
        let Some(index) = segments.iter().position(|segment| segment.start == current) else {
            break;
        };
        ordered.push(segments.remove(index));
    }
    if !segments.is_empty() || ordered.last().map(|segment| &segment.end) != Some(&ordered[0].start)
    {
        return Err(MixedShellPlanError::CoincidentCapBoundaryChain(
            physical_end,
        ));
    }
    let mut vertices = vec![ordered[0].start.clone()];
    vertices.extend(ordered.iter().map(|segment| segment.end.clone()));
    Ok(MixedShellLoopPlan {
        uses: ordered.into_iter().map(|segment| segment.use_).collect(),
        vertices,
    })
}
