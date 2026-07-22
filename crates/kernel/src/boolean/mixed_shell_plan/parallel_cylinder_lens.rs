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
    PreparedCoincidentCapCell,
};
use crate::boolean::parallel_cylinder_relation::CertifiedParallelCylinderCoincidentCapRelation;
use crate::boolean::parallel_cylinder_relation::ParallelCylinderSourceRootWitness;

struct ReversedBoundarySegment {
    use_: MixedShellEdgeUse,
    start: MixedShellVertexKey,
    end: MixedShellVertexKey,
}

/// Build the Intersect lens plan under the exact coincident-cap relation.
pub(crate) fn plan_parallel_cylinder_coincident_intersection<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<
        Item = SelectedBoundaryFragment<
            ParallelCoincidentBoundaryKey,
            ParallelCoincidentBoundaryPayload,
        >,
    >,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    linear: f64,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    if graph.completion() != SectionCompletion::Indeterminate || graph.gaps().is_empty() {
        return Err(MixedShellPlanError::SectionIncomplete);
    }
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
            (
                ParallelCoincidentBoundaryKey::CapEnd(physical_end),
                ParallelCoincidentBoundaryPayload::Cap(cap),
            ) if physical_end == cap.physical_end()
                && operand == operand_side(cap.target_operand())
                && orientation == SelectedOrientation::Preserved =>
            {
                if caps.insert(physical_end, cap).is_some() {
                    return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
                }
            }
            _ => return Err(MixedShellPlanError::CoincidentCapSelectionMismatch),
        }
    }
    if arranged.len() != 2 || caps.len() != relation.overlap_ends().len() {
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
    cap: &PreparedCoincidentCapCell,
    physical_end: usize,
    end: &crate::boolean::parallel_cylinder_relation::ParallelCylinderCoincidentCapEndWitness,
) -> Result<(), MixedShellPlanError> {
    let source = end
        .source(cap.target_operand())
        .ok_or(MixedShellPlanError::CoincidentCapSelectionMismatch)?;
    if cap.physical_end() != physical_end
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
    if expected.as_slice() != cap.boundary() {
        return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
    }
    Ok(())
}

fn append_cap_faces(
    store: &Store,
    graph: &BodySectionGraph,
    caps: &BTreeMap<usize, PreparedCoincidentCapCell>,
    faces: &mut Vec<MixedShellFacePlan>,
    spans: &mut Vec<MixedBoundedSourceSpanPlan>,
    linear: f64,
) -> Result<(), MixedShellPlanError> {
    for (&physical_end, cap) in caps {
        if faces
            .iter()
            .any(|face| face.source() == cap.target_source())
        {
            return Err(MixedShellPlanError::CoincidentCapSelectionMismatch);
        }
        let mut segments = Vec::with_capacity(cap.boundary().len());
        for piece in cap.boundary() {
            segments.push(match *piece {
                CoincidentCapBoundaryPiece::SourceArc {
                    operand,
                    edge,
                    roots,
                } => reversed_source_segment(
                    store,
                    cap,
                    physical_end,
                    operand,
                    edge,
                    roots,
                    faces,
                    spans,
                    linear,
                )?,
                CoincidentCapBoundaryPiece::SectionArc {
                    fragment,
                    endpoints,
                } => {
                    reversed_section_segment(graph, cap, physical_end, fragment, endpoints, faces)?
                }
            });
        }
        let loop_ = close_reversed_segments(physical_end, segments)?;
        faces.push(MixedShellFacePlan {
            source: cap.target_source(),
            source_face: cap.target_face().clone(),
            selected_orientation: SelectedOrientation::Preserved,
            loops: vec![loop_],
        });
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn reversed_source_segment(
    store: &Store,
    cap: &PreparedCoincidentCapCell,
    physical_end: usize,
    operand: usize,
    edge: RawEdgeId,
    roots: [ParallelCylinderSourceRootWitness; 2],
    faces: &[MixedShellFacePlan],
    spans: &[MixedBoundedSourceSpanPlan],
    linear: f64,
) -> Result<ReversedBoundarySegment, MixedShellPlanError> {
    let mut matches = Vec::new();
    for span in spans.iter().filter(|span| {
        span.source().operand() == operand
            && span.edge() == edge
            && same_source_roots(span.roots(), &roots)
    }) {
        for (face_index, face) in faces.iter().enumerate() {
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
    cap: &PreparedCoincidentCapCell,
    physical_end: usize,
    fragment: usize,
    endpoints: [usize; 2],
    faces: &[MixedShellFacePlan],
) -> Result<ReversedBoundarySegment, MixedShellPlanError> {
    let mut matches = Vec::new();
    for face in faces {
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
