//! Exact-dual classification of mixed Plane/Cylinder face arrangements.
//!
//! Each bounded planar face is anchored by one topology-owned source span.
//! A numeric point is chosen strictly inside that already-certified span and
//! classified against the other complete source body.  The point never
//! chooses a cell: exact source-span ownership does.  Occupancy then toggles
//! across every Section cut in the certified connected dual graph.
//!
//! A cylinder-side annulus uses its topology-owned source ring as the same
//! kind of anchor. Complete Section evidence proves that uncut cylinder caps
//! have constant classification. Exterior caps enter the same generic truth
//! selector as every other source fragment and retain the periodic side's
//! endpoint-free whole-loop identity without inventing a seam vertex.

use std::collections::BTreeMap;

use kcore::operation::OperationScope;
use kgeom::curve::Curve;
use kgeom::vec::Point3;
use ktopo::geom::CurveGeom;
use ktopo::store::Store;

use super::boundary_select::{
    BoundaryFragmentClassification, ClassifiedBoundaryFragment, OperandSide,
    RegularizedBooleanOperation,
};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::face_arrangement::ArrangementEdgeKey;
use super::mixed_cap_boundary::{
    MixedCylinderCapRing, bind_cylinder_cap_rings, classified_exterior_cap,
};
use super::mixed_face_arrangement::{
    MixedFaceArrangementError, MixedPlanarFaceOutput, MixedSourceSpanLineage,
    arrange_mixed_planar_face_with_lineage,
};
use super::mixed_periodic_arrangement::{
    MixedPeriodicArrangementError, MixedPeriodicFaceArrangement, PeriodicArrangementCellKey,
    arrange_mixed_periodic_face,
};
use super::mixed_shell_plan::{
    MixedArrangementBinding, MixedShellCellKey, MixedSourceFaceKey, source_face_key,
};
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use crate::classify::{PointBodyVerdict, classify_point_in_body_in_scope};
use crate::error::Error;
use crate::{BodyId, BodySectionGraph, FaceId, Part, SectionPeriodicFaceEmbeddingEvidence};

/// Failure before truth selection or topology allocation.
#[derive(Debug)]
pub(crate) enum MixedBoundaryError {
    Execution(Error),
    IncompleteSection,
    PlanarArrangement(MixedFaceArrangementError),
    PeriodicArrangement(MixedPeriodicArrangementError),
    MissingPeriodicFaceEvidence,
    SourceTopology,
    AnchorUnavailable,
    AnchorBoundaryContact,
    AnchorIndeterminate(&'static str),
    ContradictoryDual,
    DisconnectedDual,
    CylinderCapNotExterior,
    CylinderCapSelectionRequired,
}

impl From<Error> for MixedBoundaryError {
    fn from(error: Error) -> Self {
        Self::Execution(error)
    }
}

struct PreparedPlanarFace {
    face: FaceId,
    operand: usize,
    source: MixedSourceFaceKey,
    output: MixedPlanarFaceOutput,
}

struct PreparedPeriodicFace {
    face: FaceId,
    operand: usize,
    source: MixedSourceFaceKey,
    arrangement: MixedPeriodicFaceArrangement,
}

/// Owned arrangements plus their complete open-cell classifications.
pub(crate) struct PreparedMixedBoundary {
    planar: Vec<PreparedPlanarFace>,
    periodic: Vec<PreparedPeriodicFace>,
    caps: Vec<MixedCylinderCapRing>,
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

impl PreparedMixedBoundary {
    pub(crate) fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.planar
            .iter()
            .map(|face| MixedArrangementBinding::Planar {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: face.output.arrangement(),
                lineage: face.output.lineage(),
            })
            .chain(
                self.periodic
                    .iter()
                    .map(|face| MixedArrangementBinding::Periodic {
                        face: face.face.clone(),
                        operand: face.operand,
                        arrangement: &face.arrangement,
                    }),
            )
            .chain(
                self.caps
                    .iter()
                    .map(|ring| MixedArrangementBinding::CylinderCap { ring }),
            )
            .collect()
    }

    pub(crate) fn classified(&self) -> Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>> {
        self.classified.clone()
    }
}

/// Arrange and classify every planar-host face and the finite cylinder side.
///
/// Each uncut cylinder cap is classified as one endpoint-free source cell.
/// The proof plan retains its exact shared source-ring identity. Realization
/// remains fail-closed until the analytic-shell adapter accepts endpoint-free
/// edges; this boundary layer never changes set truth to hide that seam.
#[allow(clippy::too_many_arguments)]
pub(crate) fn prepare_mixed_bounded_arc_boundary(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    planar_operand: usize,
    cylinder_operand: usize,
    _operation: RegularizedBooleanOperation,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedMixedBoundary, MixedBoundaryError> {
    if !graph.gaps().is_empty()
        || graph.completion() != crate::SectionCompletion::Complete
        || planar_operand > 1
        || cylinder_operand > 1
        || planar_operand == cylinder_operand
    {
        return Err(MixedBoundaryError::IncompleteSection);
    }
    let work = mixed_boundary_work(
        planar.faces().len(),
        graph.curve_fragments().len(),
        graph.curve_endpoints().len(),
        graph.curve_components().len(),
    )
    .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;
    let store = &part.state.store;
    let mut prepared_planar = Vec::with_capacity(planar.faces().len());
    let mut classified = Vec::new();
    for source_face in planar.faces() {
        let face = source_face.clone();
        let output =
            arrange_mixed_planar_face_with_lineage(store, graph, face.clone(), planar_operand)
                .map_err(MixedBoundaryError::PlanarArrangement)?;
        let source = source_face_key(store, graph, &face, planar_operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let classes = classify_planar_face(
            part,
            &bodies[cylinder_operand],
            output.arrangement(),
            output.lineage(),
            linear,
            scope,
        )?;
        classified.extend(output.arrangement().cells().iter().map(|cell| {
            ClassifiedBoundaryFragment::new(
                MixedShellCellKey::planar(source, cell.key()),
                operand_side(planar_operand),
                (),
                as_boundary_classification(classes[&cell.key()]),
            )
        }));
        prepared_planar.push(PreparedPlanarFace {
            face,
            operand: planar_operand,
            source,
            output,
        });
    }

    let periodic_face = graph
        .periodic_face_embeddings()
        .iter()
        .find_map(|evidence| match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(certified)
                if certified.operand() == cylinder_operand
                    && certified.face().raw() == cylinder.side_face() =>
            {
                Some(certified.face())
            }
            _ => None,
        })
        .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
    let periodic_arrangement =
        arrange_mixed_periodic_face(graph, periodic_face.clone(), cylinder_operand)
            .map_err(MixedBoundaryError::PeriodicArrangement)?;
    let periodic_source = source_face_key(store, graph, &periodic_face, cylinder_operand)
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let cap_rings = bind_cylinder_cap_rings(
        store,
        graph,
        cylinder,
        cylinder_operand,
        &periodic_face,
        &periodic_arrangement,
    )
    .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let periodic_classes = classify_periodic_face(
        part,
        graph,
        &bodies[planar_operand],
        &periodic_face,
        cylinder_operand,
        &periodic_arrangement,
        linear,
        scope,
    )?;
    classified.extend(periodic_arrangement.cells().iter().map(|cell| {
        ClassifiedBoundaryFragment::new(
            MixedShellCellKey::periodic(periodic_source, *cell.key()),
            operand_side(cylinder_operand),
            (),
            as_boundary_classification(periodic_classes[cell.key()]),
        )
    }));

    certify_exterior_cylinder_caps(
        part,
        graph,
        &bodies[planar_operand],
        cylinder,
        cylinder_operand,
        linear,
        scope,
    )?;
    classified.extend(cap_rings.iter().map(|ring| {
        classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(ring.cap_source(), ring.boundary()),
            cylinder_operand,
        )
    }));

    Ok(PreparedMixedBoundary {
        planar: prepared_planar,
        periodic: vec![PreparedPeriodicFace {
            face: periodic_face,
            operand: cylinder_operand,
            source: periodic_source,
            arrangement: periodic_arrangement,
        }],
        caps: cap_rings.into_iter().collect(),
        classified,
    })
}

/// Geometry-independent ceiling charged before the first arrangement exit.
///
/// One unit owns each planar/periodic face, each directed use of every
/// Section fragment on both source surfaces, each endpoint rotation, each
/// component embedding, and one final classification record per potential
/// cut side. Checked arithmetic makes overflow an explicit refusal.
fn mixed_boundary_work(
    planar_faces: usize,
    fragments: usize,
    endpoints: usize,
    components: usize,
) -> Option<u64> {
    let faces = u64::try_from(planar_faces).ok()?.checked_add(3)?;
    let fragments = u64::try_from(fragments).ok()?;
    let endpoints = u64::try_from(endpoints).ok()?;
    let components = u64::try_from(components).ok()?;
    faces
        .checked_add(fragments.checked_mul(4)?)?
        .checked_add(endpoints.checked_mul(2)?)?
        .checked_add(components)
}

fn classify_planar_face(
    part: &Part<'_>,
    other: &BodyId,
    arrangement: &super::mixed_face_arrangement::MixedPlanarFaceArrangement,
    lineage: &super::mixed_face_arrangement::MixedPlanarSourceLineage,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<BTreeMap<usize, bool>, MixedBoundaryError> {
    let span = arrangement
        .source_spans()
        .first()
        .ok_or(MixedBoundaryError::AnchorUnavailable)?;
    let source = lineage
        .spans()
        .iter()
        .find(|candidate| candidate.key() == span.key())
        .ok_or(MixedBoundaryError::AnchorUnavailable)?;
    let owner = arrangement
        .cells()
        .iter()
        .find(|cell| {
            cell.boundary().uses().iter().any(
                |use_| matches!(use_.edge(), ArrangementEdgeKey::Source(key) if key == span.key()),
            )
        })
        .map(|cell| cell.key())
        .ok_or(MixedBoundaryError::AnchorUnavailable)?;
    let point = source_span_point(&part.state.store, source)?;
    let anchor = classify_anchor(part, other, point, linear, scope)?;
    propagate_planar(arrangement, owner, anchor)
}

#[allow(clippy::too_many_arguments)]
fn classify_periodic_face(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    other: &BodyId,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPeriodicFaceArrangement,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<BTreeMap<PeriodicArrangementCellKey, bool>, MixedBoundaryError> {
    let certified = graph
        .periodic_face_embeddings()
        .iter()
        .find_map(|evidence| match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(value)
                if value.operand() == operand && value.face() == *face =>
            {
                Some(value)
            }
            _ => None,
        })
        .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
    let loop_id = certified.source_loops()[0].raw();
    let loop_ = part
        .state
        .store
        .get(loop_id)
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let [fin_id] = loop_.fins() else {
        return Err(MixedBoundaryError::SourceTopology);
    };
    let fin = part
        .state
        .store
        .get(*fin_id)
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let edge = part
        .state
        .store
        .get(fin.edge())
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let curve_id = edge.curve().ok_or(MixedBoundaryError::SourceTopology)?;
    let CurveGeom::Circle(circle) = part
        .state
        .store
        .get(curve_id)
        .map_err(|_| MixedBoundaryError::SourceTopology)?
    else {
        return Err(MixedBoundaryError::SourceTopology);
    };
    let point = circle.eval(circle.param_range().lo);
    let anchor = classify_anchor(part, other, point, linear, scope)?;
    propagate_periodic(
        arrangement,
        PeriodicArrangementCellKey::AnnularRemainder,
        anchor,
    )
}

fn source_span_point(
    store: &Store,
    source: &MixedSourceSpanLineage,
) -> Result<Point3, MixedBoundaryError> {
    let intervals = source
        .range()
        .each_ref()
        .map(|value| value.parameter_interval());
    let open = if intervals[0][1] < intervals[1][0] {
        [intervals[0][1], intervals[1][0]]
    } else if intervals[1][1] < intervals[0][0] {
        [intervals[1][1], intervals[0][0]]
    } else {
        return Err(MixedBoundaryError::AnchorUnavailable);
    };
    let parameter = open[0] * 0.5 + open[1] * 0.5;
    if !parameter.is_finite() || parameter <= open[0] || parameter >= open[1] {
        return Err(MixedBoundaryError::AnchorUnavailable);
    }
    let edge = store
        .get(source.edge())
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let (lo, hi) = edge.bounds().ok_or(MixedBoundaryError::SourceTopology)?;
    if parameter <= lo || parameter >= hi {
        return Err(MixedBoundaryError::AnchorUnavailable);
    }
    let curve = edge.curve().ok_or(MixedBoundaryError::SourceTopology)?;
    Ok(store
        .get(curve)
        .map_err(|_| MixedBoundaryError::SourceTopology)?
        .as_curve()
        .eval(parameter))
}

fn classify_anchor(
    part: &Part<'_>,
    other: &BodyId,
    point: Point3,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<bool, MixedBoundaryError> {
    let classification = classify_point_in_body_in_scope(part, other, point, linear, scope)?;
    match classification.verdict() {
        PointBodyVerdict::Interior => Ok(true),
        PointBodyVerdict::Exterior => Ok(false),
        PointBodyVerdict::Boundary { .. } => Err(MixedBoundaryError::AnchorBoundaryContact),
        PointBodyVerdict::Indeterminate { reason } => {
            Err(MixedBoundaryError::AnchorIndeterminate(reason))
        }
    }
}

fn propagate_planar(
    arrangement: &super::mixed_face_arrangement::MixedPlanarFaceArrangement,
    anchor: usize,
    value: bool,
) -> Result<BTreeMap<usize, bool>, MixedBoundaryError> {
    let mut classes = BTreeMap::from([(anchor, value)]);
    loop {
        let before = classes.len();
        for edge in arrangement.adjacency() {
            propagate_pair(&mut classes, edge.forward_cell(), edge.reverse_cell())?;
        }
        if classes.len() == before {
            break;
        }
    }
    if classes.len() != arrangement.cells().len() {
        return Err(MixedBoundaryError::DisconnectedDual);
    }
    Ok(classes)
}

fn propagate_periodic(
    arrangement: &MixedPeriodicFaceArrangement,
    anchor: PeriodicArrangementCellKey,
    value: bool,
) -> Result<BTreeMap<PeriodicArrangementCellKey, bool>, MixedBoundaryError> {
    let mut classes = BTreeMap::from([(anchor, value)]);
    loop {
        let before = classes.len();
        for edge in arrangement.adjacency() {
            propagate_pair(&mut classes, *edge.forward_cell(), *edge.reverse_cell())?;
        }
        if classes.len() == before {
            break;
        }
    }
    if classes.len() != arrangement.cells().len() {
        return Err(MixedBoundaryError::DisconnectedDual);
    }
    Ok(classes)
}

fn propagate_pair<K: Copy + Ord>(
    classes: &mut BTreeMap<K, bool>,
    first: K,
    second: K,
) -> Result<(), MixedBoundaryError> {
    match (classes.get(&first).copied(), classes.get(&second).copied()) {
        (Some(left), Some(right)) if left == right => Err(MixedBoundaryError::ContradictoryDual),
        (Some(left), None) => {
            classes.insert(second, !left);
            Ok(())
        }
        (None, Some(right)) => {
            classes.insert(first, !right);
            Ok(())
        }
        _ => Ok(()),
    }
}

fn certify_exterior_cylinder_caps(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    other: &BodyId,
    cylinder: &CertifiedCylinderSource,
    cylinder_operand: usize,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<(), MixedBoundaryError> {
    for boundary in cylinder.boundaries() {
        if graph
            .branches()
            .iter()
            .any(|branch| branch.faces()[cylinder_operand].raw() == boundary.cap_face())
        {
            return Err(MixedBoundaryError::CylinderCapNotExterior);
        }
        if classify_anchor(part, other, boundary.center(), linear, scope)? {
            return Err(MixedBoundaryError::CylinderCapNotExterior);
        }
    }
    Ok(())
}

/// Whether generic set truth omits an exterior source cap.
///
/// Retained for a focused truth-table proof. Production cap fragments now go
/// through `boundary_select` rather than using this predicate for control.
const fn caps_are_omitted_by_truth(
    operation: RegularizedBooleanOperation,
    cylinder_operand: usize,
) -> bool {
    match operation {
        RegularizedBooleanOperation::Intersect => true,
        RegularizedBooleanOperation::Subtract => cylinder_operand == 1,
        RegularizedBooleanOperation::Unite => false,
    }
}

const fn operand_side(operand: usize) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

const fn as_boundary_classification(inside: bool) -> BoundaryFragmentClassification {
    if inside {
        BoundaryFragmentClassification::Interior
    } else {
        BoundaryFragmentClassification::Exterior
    }
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;

    use super::super::boundary_select::select_boundary_fragments;
    use super::super::curved_source::{CylinderSourceOutcome, extract_cylinder_source};
    use super::super::extract::extract_planar_source_body;
    use super::super::mixed_shell_plan::{
        MixedShellCellKind, MixedShellEdgeKey, MixedShellVertexKey,
        materialize::prepare_mixed_shell_materialization, plan_mixed_shell,
    };
    use super::*;
    use crate::{BlockRequest, CylinderRequest, Kernel, SectionBodiesRequest};

    #[test]
    fn cap_omission_matches_regularized_truth() {
        assert!(caps_are_omitted_by_truth(
            RegularizedBooleanOperation::Intersect,
            0
        ));
        assert!(caps_are_omitted_by_truth(
            RegularizedBooleanOperation::Intersect,
            1
        ));
        assert!(caps_are_omitted_by_truth(
            RegularizedBooleanOperation::Subtract,
            1
        ));
        assert!(!caps_are_omitted_by_truth(
            RegularizedBooleanOperation::Subtract,
            0
        ));
        assert!(!caps_are_omitted_by_truth(
            RegularizedBooleanOperation::Unite,
            0
        ));
        assert!(!caps_are_omitted_by_truth(
            RegularizedBooleanOperation::Unite,
            1
        ));
    }

    #[test]
    fn selected_caps_retain_only_topology_owned_whole_ring_identity() {
        for (swapped, operation, expected_caps) in [
            (false, RegularizedBooleanOperation::Unite, 2),
            (true, RegularizedBooleanOperation::Unite, 2),
            (true, RegularizedBooleanOperation::Subtract, 2),
            (false, RegularizedBooleanOperation::Subtract, 0),
        ] {
            let mut session = Kernel::new().create_session();
            let part_id = session.create_part();
            let (block, cylinder) = {
                let mut edit = session.edit_part(part_id.clone()).unwrap();
                let block = edit
                    .create_block(BlockRequest::new(
                        Frame::world().with_origin(kgeom::vec::Point3::new(0.0, 0.0, 1.0)),
                        [2.0, 5.0, 1.0],
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                let cylinder = edit
                    .create_cylinder(CylinderRequest::new(Frame::world(), 1.5, 2.0))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                (block, cylinder)
            };
            let (left, right, planar_operand, cylinder_operand) = if swapped {
                (cylinder.clone(), block.clone(), 1, 0)
            } else {
                (block.clone(), cylinder.clone(), 0, 1)
            };
            let graph = session
                .part(part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(left.clone(), right.clone()))
                .unwrap()
                .into_result()
                .unwrap();
            let part = session.part(part_id).unwrap();
            let context = OperationContext::new(part.policy(), Tolerances::default())
                .unwrap()
                .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
            let mut scope = OperationScope::new(&context);
            let planar = extract_planar_source_body(
                &part,
                block,
                u8::try_from(planar_operand).unwrap(),
                &mut scope,
            )
            .unwrap();
            let cylinder_source =
                match extract_cylinder_source(&part.state.store, cylinder.raw(), &mut scope)
                    .unwrap()
                {
                    CylinderSourceOutcome::Ready(source) => source,
                    other => panic!("unexpected cylinder extraction: {other:?}"),
                };
            let prepared = prepare_mixed_bounded_arc_boundary(
                &part,
                &graph,
                &[left, right],
                &planar,
                &cylinder_source,
                planar_operand,
                cylinder_operand,
                operation,
                context.tolerances().linear(),
                &mut scope,
            )
            .unwrap();
            let selected = select_boundary_fragments(operation, prepared.classified()).unwrap();
            let cap_count = selected
                .iter()
                .filter(|fragment| {
                    matches!(fragment.key().cell(), MixedShellCellKind::CylinderCap(_))
                })
                .count();
            assert_eq!(cap_count, expected_caps, "swapped={swapped} {operation:?}");
            if expected_caps == 0 {
                continue;
            }

            let plan =
                plan_mixed_shell(&part.state.store, &graph, prepared.bindings(), selected).unwrap();
            assert_eq!(plan.cap_rings().len(), 2);
            for ring in plan.cap_rings() {
                let face = plan
                    .faces()
                    .iter()
                    .find(|face| face.source() == ring.cap_source())
                    .unwrap();
                let [loop_] = face.loops() else {
                    panic!("cap must retain one complete source loop")
                };
                let [use_] = loop_.uses() else {
                    panic!("cap must retain one endpoint-free ring use")
                };
                assert_eq!(
                    use_.edge(),
                    &MixedShellEdgeKey::PeriodicSource {
                        source: ring.side_source(),
                        loop_key: ring.side_loop_key(),
                    }
                );
                let side_uses = plan
                    .faces()
                    .iter()
                    .filter(|face| face.source() == ring.side_source())
                    .flat_map(|face| face.loops())
                    .flat_map(|loop_| loop_.uses())
                    .filter(|candidate| candidate.edge() == use_.edge())
                    .collect::<Vec<_>>();
                let [side_use] = side_uses.as_slice() else {
                    panic!("whole ring must retain exactly one selected side use")
                };
                assert_ne!(
                    use_.direction(),
                    side_use.direction(),
                    "cap direction must be derived from and oppose its selected side use"
                );
                let [first, second] = loop_.vertices() else {
                    panic!("whole ring must retain one repeated proof seam")
                };
                assert_eq!(first, second);
                assert!(matches!(first, MixedShellVertexKey::ProofSeam { .. }));
                let edge = part.state.store.get(ring.edge()).unwrap();
                assert_eq!(edge.vertices(), [None, None]);
                assert!(edge.bounds().is_none());
            }
            let blueprint = prepare_mixed_shell_materialization(&plan, &part.state.store).unwrap();
            assert_eq!(
                blueprint
                    .edges()
                    .iter()
                    .filter(|edge| edge.endpoints().is_none())
                    .count(),
                2,
                "materialization must retain both endpoint-free rings without seam vertices"
            );
        }
    }
}
