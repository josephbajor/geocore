//! Proof-bearing adoption of mixed planar/periodic face arrangements.
#![allow(clippy::result_large_err)]

use std::collections::{BTreeMap, BTreeSet};

use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::vec::Point3;
use ktopo::analytic_shell::AnalyticFaceSplitPiece;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, LoopId as RawLoopId, Sense,
};
use ktopo::store::Store;

#[path = "mixed_shell_components.rs"]
pub(crate) mod components;
#[path = "mixed_shell_plan/cylinder_pair.rs"]
pub(crate) mod cylinder_pair;
#[path = "mixed_shell_materialize.rs"]
pub(crate) mod materialize;
#[path = "mixed_shell_plan/parallel_cylinder_lens.rs"]
mod parallel_cylinder_lens;
#[path = "mixed_shell_plan/projected_source_circle.rs"]
mod projected_source_circle;

pub(crate) use parallel_cylinder_lens::plan_parallel_cylinder_coincident_boolean;
pub(crate) use projected_source_circle::{
    ProjectedEndpointFreeSourceCircle, ProjectedSourceCircleOnPlane,
    ProjectedSourceCircleOnPlaneError,
};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::disk_face_arrangement::{ArrangedDiskFace, DiskChordKey, DiskSourceArcKey};
use super::face_arrangement::{ArrangementCycle, ArrangementDirection, ArrangementEdgeKey};
use super::mixed_cap_boundary::MixedCylinderCapRing;
use super::mixed_face_arrangement::{
    MixedArrangementVertex, MixedCutFragmentKey, MixedPlanarFaceArrangement,
    MixedPlanarSourceLineage, MixedSourceParameterEvidence, MixedSourceSpanKey,
};
use super::mixed_periodic_arrangement::{
    MixedPeriodicFaceArrangement, PeriodicArrangementCellKey, PeriodicArrangementVertexKey,
    PeriodicCutFragmentKey, PeriodicSourceLoopKey,
};
use crate::section::{SectionSkewCylinderPersistenceInput, bounded_skew_persistence_input};
use crate::{
    BodySectionGraph, FaceId, SectionBranch, SectionCompletion, SectionCurveEndpointTopology,
    SectionCurveFragment, SectionCurveFragmentSpan, SectionPeriodicFaceEmbeddingEvidence,
};

type PeriodicArrangementCycle =
    ArrangementCycle<PeriodicSourceLoopKey, PeriodicCutFragmentKey, PeriodicArrangementVertexKey>;
type DiskArrangementCycle = ArrangementCycle<DiskSourceArcKey, DiskChordKey, usize>;
type OrientedCycleParts<S, C, V> = (
    Vec<(ArrangementEdgeKey<S, C>, ArrangementDirection)>,
    Vec<V>,
);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct MixedSourceFaceKey {
    operand: usize,
    topology_ordinal: usize,
}

impl MixedSourceFaceKey {
    pub(crate) const fn operand(self) -> usize {
        self.operand
    }

    pub(crate) const fn topology_ordinal(self) -> usize {
        self.topology_ordinal
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellCellKind {
    Planar(usize),
    Disk(usize),
    Periodic(PeriodicArrangementCellKey),
    CylinderCap(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct MixedShellCellKey {
    source: MixedSourceFaceKey,
    cell: MixedShellCellKind,
}

impl MixedShellCellKey {
    pub(crate) const fn planar(source: MixedSourceFaceKey, cell: usize) -> Self {
        Self {
            source,
            cell: MixedShellCellKind::Planar(cell),
        }
    }

    pub(crate) const fn periodic(
        source: MixedSourceFaceKey,
        cell: PeriodicArrangementCellKey,
    ) -> Self {
        Self {
            source,
            cell: MixedShellCellKind::Periodic(cell),
        }
    }

    pub(crate) const fn disk(source: MixedSourceFaceKey, cell: usize) -> Self {
        Self {
            source,
            cell: MixedShellCellKind::Disk(cell),
        }
    }

    pub(crate) const fn cylinder_cap(source: MixedSourceFaceKey, boundary: usize) -> Self {
        Self {
            source,
            cell: MixedShellCellKind::CylinderCap(boundary),
        }
    }

    pub(crate) const fn source(self) -> MixedSourceFaceKey {
        self.source
    }

    pub(crate) const fn cell(self) -> MixedShellCellKind {
        self.cell
    }
}

pub(crate) enum MixedArrangementBinding<'a> {
    Planar {
        face: FaceId,
        operand: usize,
        arrangement: &'a MixedPlanarFaceArrangement,
        lineage: &'a MixedPlanarSourceLineage,
    },
    Disk {
        face: FaceId,
        operand: usize,
        arranged: &'a ArrangedDiskFace,
    },
    Periodic {
        face: FaceId,
        operand: usize,
        arrangement: &'a MixedPeriodicFaceArrangement,
        embedding: Option<&'a crate::CertifiedSectionPeriodicFaceEmbedding>,
    },
    CylinderCap {
        ring: &'a MixedCylinderCapRing,
    },
}

impl MixedArrangementBinding<'_> {
    fn face(&self) -> &FaceId {
        match self {
            Self::Planar { face, .. } | Self::Disk { face, .. } | Self::Periodic { face, .. } => {
                face
            }
            Self::CylinderCap { ring } => ring.cap_face(),
        }
    }

    const fn operand(&self) -> usize {
        match self {
            Self::Planar { operand, .. }
            | Self::Disk { operand, .. }
            | Self::Periodic { operand, .. } => *operand,
            Self::CylinderCap { ring } => ring.operand(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellVertexKey {
    SectionEndpoint(usize),
    PlanarSourceVertex {
        source: MixedSourceFaceKey,
        topology_ordinal: usize,
    },
    ProofSeam {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
    },
    DerivedRingSeam(usize),
    Tangency(usize),
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellEdgeKey {
    PlanarSource {
        source: MixedSourceFaceKey,
        span: MixedSourceSpanKey,
    },
    PeriodicSource {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
    },
    SectionFragment(usize),
    DerivedRing(usize),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedPcurveLineage {
    SourceTopology,
    ProjectedSourceCircleOnPlane(ProjectedSourceCircleOnPlane),
    ProjectedEndpointFreeSourceCircle(ProjectedEndpointFreeSourceCircle),
    Section {
        branch: usize,
        operand: usize,
        cylinder_period_shift: i64,
    },
    DerivedRing {
        cylinder_parameter_bits: Option<u64>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MixedDerivedRingLineage {
    Source(RawEdgeId),
    Derived([RawFaceId; 2]),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MixedDerivedRingPlan {
    circle: Circle,
    tangency: Option<(usize, Point3)>,
    lineage: MixedDerivedRingLineage,
}

impl MixedDerivedRingPlan {
    const fn endpoint_free(circle: Circle, lineage: MixedDerivedRingLineage) -> Self {
        Self {
            circle,
            tangency: None,
            lineage,
        }
    }

    const fn tangent(
        circle: Circle,
        vertex: usize,
        point: Point3,
        lineage: MixedDerivedRingLineage,
    ) -> Self {
        Self {
            circle,
            tangency: Some((vertex, point)),
            lineage,
        }
    }

    pub(crate) const fn circle(&self) -> Circle {
        self.circle
    }

    pub(crate) const fn tangency(&self) -> Option<(usize, Point3)> {
        self.tangency
    }

    pub(crate) const fn lineage(&self) -> MixedDerivedRingLineage {
        self.lineage
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellEdgeUse {
    edge: MixedShellEdgeKey,
    direction: ArrangementDirection,
    pcurve: MixedPcurveLineage,
}

impl MixedShellEdgeUse {
    pub(crate) const fn edge(&self) -> &MixedShellEdgeKey {
        &self.edge
    }

    pub(crate) const fn direction(&self) -> ArrangementDirection {
        self.direction
    }

    pub(crate) const fn pcurve(&self) -> &MixedPcurveLineage {
        &self.pcurve
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellLoopPlan {
    uses: Vec<MixedShellEdgeUse>,
    vertices: Vec<MixedShellVertexKey>,
}

impl MixedShellLoopPlan {
    pub(crate) fn uses(&self) -> &[MixedShellEdgeUse] {
        &self.uses
    }

    pub(crate) fn vertices(&self) -> &[MixedShellVertexKey] {
        &self.vertices
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellFacePlan {
    source: MixedSourceFaceKey,
    source_face: FaceId,
    selected_orientation: SelectedOrientation,
    loops: Vec<MixedShellLoopPlan>,
    merge_sources: Option<[FaceId; 2]>,
    split_lineage: Option<AnalyticFaceSplitPiece>,
}

impl MixedShellFacePlan {
    pub(crate) const fn source(&self) -> MixedSourceFaceKey {
        self.source
    }

    pub(crate) const fn source_face(&self) -> &FaceId {
        &self.source_face
    }

    pub(crate) const fn selected_orientation(&self) -> SelectedOrientation {
        self.selected_orientation
    }

    pub(crate) fn loops(&self) -> &[MixedShellLoopPlan] {
        &self.loops
    }

    pub(crate) const fn merge_sources(&self) -> Option<&[FaceId; 2]> {
        self.merge_sources.as_ref()
    }

    pub(crate) const fn split_lineage(&self) -> Option<AnalyticFaceSplitPiece> {
        self.split_lineage
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MixedSectionEdgePlan {
    fragment_index: usize,
    fragment: SectionCurveFragment,
    branch: SectionBranch,
    endpoints: [usize; 2],
    carrier_faces: [MixedSourceFaceKey; 2],
    skew_persistence: Option<SectionSkewCylinderPersistenceInput>,
}

impl MixedSectionEdgePlan {
    pub(crate) const fn fragment_index(&self) -> usize {
        self.fragment_index
    }

    pub(crate) const fn fragment(&self) -> &SectionCurveFragment {
        &self.fragment
    }

    pub(crate) const fn branch(&self) -> &SectionBranch {
        &self.branch
    }

    pub(crate) const fn endpoints(&self) -> [usize; 2] {
        self.endpoints
    }

    pub(crate) const fn carrier_faces(&self) -> [MixedSourceFaceKey; 2] {
        self.carrier_faces
    }

    pub(crate) const fn skew_persistence(&self) -> Option<SectionSkewCylinderPersistenceInput> {
        self.skew_persistence
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MixedBoundedSourceRoot {
    endpoint: usize,
    root_ordinal: usize,
    parameter_bits: u64,
    enclosure_bits: [u64; 2],
    period_shift: i32,
}

impl MixedBoundedSourceRoot {
    pub(crate) const fn endpoint(self) -> usize {
        self.endpoint
    }

    pub(crate) const fn root_ordinal(self) -> usize {
        self.root_ordinal
    }

    pub(crate) const fn parameter(self) -> f64 {
        f64::from_bits(self.parameter_bits)
    }

    pub(crate) const fn enclosure(self) -> [f64; 2] {
        [
            f64::from_bits(self.enclosure_bits[0]),
            f64::from_bits(self.enclosure_bits[1]),
        ]
    }

    pub(crate) const fn period_shift(self) -> i32 {
        self.period_shift
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedBoundedSourceSpanPlan {
    source: MixedSourceFaceKey,
    span: MixedSourceSpanKey,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    roots: [MixedBoundedSourceRoot; 2],
}

impl MixedBoundedSourceSpanPlan {
    pub(crate) const fn source(&self) -> MixedSourceFaceKey {
        self.source
    }

    pub(crate) const fn span(&self) -> &MixedSourceSpanKey {
        &self.span
    }

    pub(crate) const fn loop_id(&self) -> RawLoopId {
        self.loop_id
    }

    pub(crate) const fn fin(&self) -> RawFinId {
        self.fin
    }

    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn roots(&self) -> &[MixedBoundedSourceRoot; 2] {
        &self.roots
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellMaterializationGap {
    ExactSourceRootParameterRequired {
        source: MixedSourceFaceKey,
        span: MixedSourceSpanKey,
        endpoint: usize,
    },
    ExactTrimParameterRequired {
        fragment: usize,
        endpoint: usize,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MixedShellProofPlan {
    faces: Vec<MixedShellFacePlan>,
    section_edges: Vec<MixedSectionEdgePlan>,
    bounded_source_spans: Vec<MixedBoundedSourceSpanPlan>,
    cap_rings: Vec<MixedCylinderCapRing>,
    derived_rings: Vec<MixedDerivedRingPlan>,
    materialization: materialize::RetainedMaterializationEvidence,
    materialization_gaps: Vec<MixedShellMaterializationGap>,
}

impl MixedShellProofPlan {
    pub(crate) fn faces(&self) -> &[MixedShellFacePlan] {
        &self.faces
    }

    pub(crate) fn section_edges(&self) -> &[MixedSectionEdgePlan] {
        &self.section_edges
    }

    pub(crate) fn bounded_source_spans(&self) -> &[MixedBoundedSourceSpanPlan] {
        &self.bounded_source_spans
    }

    pub(crate) fn cap_rings(&self) -> &[MixedCylinderCapRing] {
        &self.cap_rings
    }

    pub(crate) fn derived_rings(&self) -> &[MixedDerivedRingPlan] {
        &self.derived_rings
    }

    pub(crate) fn materialization_gaps(&self) -> &[MixedShellMaterializationGap] {
        &self.materialization_gaps
    }

    #[cfg(test)]
    pub(crate) fn clear_skew_persistence_for_test(&mut self, fragment: usize) {
        if let Some(edge) = self
            .section_edges
            .iter_mut()
            .find(|edge| edge.fragment_index == fragment)
        {
            edge.skew_persistence = None;
        }
    }

    #[cfg(test)]
    pub(crate) fn swap_skew_carrier_faces_for_test(&mut self, fragment: usize) {
        if let Some(edge) = self
            .section_edges
            .iter_mut()
            .find(|edge| edge.fragment_index == fragment)
        {
            edge.carrier_faces.swap(0, 1);
        }
    }

    #[cfg(test)]
    pub(crate) fn perturb_skew_endpoint_bound_for_test(&mut self, fragment: usize) {
        let Some(edge) = self
            .section_edges
            .iter_mut()
            .find(|edge| edge.fragment_index == fragment)
        else {
            return;
        };
        edge.fragment.perturb_bounded_procedural_bound_for_test();
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedShellPlanError {
    SectionIncomplete,
    EmptySelection,
    InvalidOperand(usize),
    FacePartMismatch,
    SourceBodyUnavailable(usize),
    FaceNotOwnedByOperand {
        operand: usize,
        face: FaceId,
    },
    DuplicateArrangement(MixedSourceFaceKey),
    SelectionOperandMismatch(MixedShellCellKey),
    DuplicateSelectedCell(MixedShellCellKey),
    MissingArrangement(MixedSourceFaceKey),
    ArrangementKindMismatch(MixedShellCellKey),
    MissingPlanarCell(MixedShellCellKey),
    MissingDiskCell(MixedShellCellKey),
    MissingPeriodicCell(MixedShellCellKey),
    CylinderCapBindingMismatch(MixedShellCellKey),
    MalformedArrangementCycle(MixedShellCellKey),
    PlanarCutEndpointIdentityUnavailable(MixedSourceFaceKey),
    MissingPlanarCutLineage(MixedSourceFaceKey),
    AmbiguousPlanarCutLineage(MixedSourceFaceKey),
    UnknownSectionFragment(usize),
    UnknownSectionBranch {
        fragment: usize,
        branch: usize,
    },
    InvalidSkewPersistence {
        fragment: usize,
    },
    SectionFragmentLeavesFace {
        fragment: usize,
        source: MixedSourceFaceKey,
    },
    PeriodicComponentMismatch(PeriodicCutFragmentKey),
    PeriodicFragmentEndpointMismatch(PeriodicCutFragmentKey),
    MissingPeriodicEmbedding {
        source: MixedSourceFaceKey,
        fragment: usize,
    },
    PhysicalUseContainsProofSeam(MixedShellCellKey),
    SectionUseCount {
        fragment: usize,
        actual: usize,
    },
    SectionUseDirectionMismatch(usize),
    EndpointFreeRingUseCount {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
        actual: usize,
    },
    EndpointFreeRingUseDirectionMismatch {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
    },
    EndpointFreeRingBindingMismatch {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
    },
    BoundedSourceSpanUseCount {
        source: MixedSourceFaceKey,
        span: MixedSourceSpanKey,
        actual: usize,
    },
    BoundedSourceSpanDirectionMismatch {
        source: MixedSourceFaceKey,
        span: MixedSourceSpanKey,
    },
    PlanarLineageMismatch(MixedSourceFaceKey),
    DiskLineageMismatch(MixedSourceFaceKey),
    AxialContactBoundaryMismatch,
    CommonSupportBoundaryMismatch,
    InternalTangencyBoundaryMismatch,
    CoincidentCapSelectionMismatch,
    CoincidentCapBoundaryUseCount {
        physical_end: usize,
        actual: usize,
    },
    CoincidentCapBoundaryChain(usize),
    ProjectedSourceCircle(ProjectedSourceCircleOnPlaneError),
}

#[derive(Clone, Copy)]
struct SectionUseLineage {
    fragment: usize,
    arrangement_to_section: ArrangementDirection,
    cylinder_period_shift: i64,
}

enum SectionPlanningAdmission<'a> {
    Complete,
    AxialContact(&'a super::parallel_cylinder_relation::CertifiedParallelCylinderAxialContact),
    CommonSupport(&'a super::parallel_cylinder_relation::CertifiedParallelCylinderCommonSupport),
    InternalTangency(
        &'a super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    ),
    CoincidentCaps(
        &'a super::parallel_cylinder_relation::CertifiedParallelCylinderCoincidentCapRelation,
    ),
}

impl SectionPlanningAdmission<'_> {
    fn validate(&self, graph: &BodySectionGraph) -> Result<(), MixedShellPlanError> {
        match self {
            Self::Complete
                if graph.completion() == SectionCompletion::Complete && graph.gaps().is_empty() =>
            {
                Ok(())
            }
            Self::AxialContact(relation)
                if graph.completion() == SectionCompletion::Indeterminate
                    && !graph.gaps().is_empty()
                    && relation.contact_boundaries()[0].operand()
                        != relation.contact_boundaries()[1].operand() =>
            {
                Ok(())
            }
            Self::CommonSupport(relation)
                if graph.completion() == SectionCompletion::Indeterminate
                    && !graph.gaps().is_empty()
                    && relation.boundaries().len() == 4 =>
            {
                Ok(())
            }
            Self::InternalTangency(relation)
                if graph.completion() == SectionCompletion::Indeterminate
                    && !graph.gaps().is_empty()
                    && relation.boundaries().len() == 4 =>
            {
                Ok(())
            }
            Self::CoincidentCaps(relation)
                if graph.completion() == SectionCompletion::Indeterminate
                    && !graph.gaps().is_empty()
                    && relation.overlap_ends().len() == 2
                    && relation.rulings().len() == 2 =>
            {
                Ok(())
            }
            _ => Err(MixedShellPlanError::SectionIncomplete),
        }
    }
}

pub(crate) fn plan_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    if graph.completion() != SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Err(MixedShellPlanError::SectionIncomplete);
    }

    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::Complete,
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |_, _, _, _, _| Ok(()),
    )
}

pub(crate) fn plan_axial_contact_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderAxialContact,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
    tolerance: f64,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::AxialContact(relation),
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |arrangements, faces, bounded_source_spans, _, _| {
            rebase_axial_contact_boundary_arcs(
                store,
                graph,
                arrangements,
                faces,
                bounded_source_spans,
                tolerance,
            )
        },
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn plan_internal_axial_contact_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderAxialContact,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
    outer_contact: &MixedCylinderCapRing,
    inner_contact: &MixedCylinderCapRing,
    tolerance: f64,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::AxialContact(relation),
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |_, faces, _, cap_rings, _| {
            append_internal_contact_hole(
                store,
                faces,
                cap_rings,
                outer_contact,
                inner_contact,
                tolerance,
            )
        },
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn plan_coincident_axial_contact_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderAxialContact,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
    far_rings: [&MixedCylinderCapRing; 2],
    tolerance: f64,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::AxialContact(relation),
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |_, faces, _, _, _| merge_coincident_side_faces(store, faces, far_rings, tolerance),
    )
}

pub(crate) fn plan_common_support_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderCommonSupport,
    interval: &super::axial_interval_sweep::AxialIntervalPlan,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
    tolerance: f64,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::CommonSupport(relation),
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |_, faces, _, rings, _| {
            graft_common_support_spans(
                store,
                faces,
                rings,
                interval,
                relation.preorder(),
                tolerance,
            )
        },
    )
}

pub(crate) fn plan_internal_tangency_bands_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    interval: &super::axial_interval_sweep::AxialIntervalPlan,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::InternalTangency(relation),
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |_, faces, _, rings, derived| {
            graft_internal_tangency_bands(faces, rings, derived, cylinders, relation, interval)
        },
    )
}

pub(crate) fn plan_internal_tangency_union_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    tails: &[super::axial_interval_sweep::PlannedAxialSpan],
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    plan_mixed_shell_with_augmentation(
        store,
        graph,
        SectionPlanningAdmission::InternalTangency(relation),
        bindings,
        selected.into_iter().map(|fragment| {
            let (key, operand, (), orientation) = fragment.into_parts();
            (key, operand, orientation)
        }),
        |_, faces, _, rings, derived| {
            graft_internal_tangency_union(faces, rings, derived, cylinders, relation, tails)
        },
    )
}

#[derive(Debug, Clone, Copy)]
struct InternalTangencyBoundary {
    operand: usize,
    boundary: usize,
    axial_parameter: f64,
}

#[derive(Debug, Clone, Copy)]
struct InternalTangencyBand {
    operand: usize,
    low: InternalTangencyBoundary,
    high: InternalTangencyBoundary,
    split: Option<AnalyticFaceSplitPiece>,
}

fn graft_internal_tangency_bands(
    faces: &mut Vec<MixedShellFacePlan>,
    rings: &mut Vec<MixedCylinderCapRing>,
    derived: &mut Vec<MixedDerivedRingPlan>,
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    interval: &super::axial_interval_sweep::AxialIntervalPlan,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    bind_internal_tangency_boundaries(cylinders, relation)?;
    if interval.spans().len() > 2 {
        return Err(fail());
    }
    let source_faces = faces.clone();
    let source_rings = rings.clone();
    let contained = relation.contained_operand();
    let source = cylinders.get(contained).ok_or_else(fail)?;
    faces.clear();
    rings.clear();
    for (span_index, span) in interval.spans().iter().enumerate() {
        let endpoints = [span.low(), span.high()]
            .map(|contributors| bind_internal_boundary_class(relation, contributors))
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?;
        let [low, high] = endpoints.as_slice() else {
            return Err(fail());
        };
        let centers = [
            internal_axis_endpoint(cylinders, source, contained, low)?,
            internal_axis_endpoint(cylinders, source, contained, high)?,
        ];
        if centers[0].1 == centers[1].1 {
            return Err(fail());
        }
        let mut ring_indices = [0_usize; 2];
        for end in 0..2 {
            let circle = Circle::new(
                source.cylinder().frame().with_origin(centers[end].0),
                source.cylinder().radius(),
            )
            .map_err(|_| fail())?;
            let lineage =
                internal_ring_lineage(cylinders, source.side_face(), contained, &endpoints[end])?;
            ring_indices[end] = derived.len();
            derived.push(MixedDerivedRingPlan::endpoint_free(circle, lineage));
        }
        let side_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == contained)
            .ok_or_else(fail)?;
        let mut side = source_face(
            &source_faces,
            side_ring.side_source(),
            side_ring.side_face(),
        )?;
        let desired = if centers[0].1 < centers[1].1 {
            [ArrangementDirection::Forward, ArrangementDirection::Reverse]
        } else {
            [ArrangementDirection::Reverse, ArrangementDirection::Forward]
        };
        let directions: [ArrangementDirection; 2] = core::array::from_fn(|end| {
            if derived_ring_cylinder_scale(
                derived[ring_indices[end]].circle(),
                *source.cylinder().frame(),
            ) > 0.0
            {
                desired[end]
            } else {
                opposite(desired[end])
            }
        });
        side.loops = (0..2)
            .map(|end| {
                derived_ring_loop(
                    ring_indices[end],
                    directions[end],
                    Some(centers[end].1),
                    derived,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        side.split_lineage = (interval.spans().len() == 2).then_some(if span_index == 0 {
            AnalyticFaceSplitPiece::First
        } else {
            AnalyticFaceSplitPiece::Second
        });
        faces.push(side);
        for end in 0..2 {
            let boundary = endpoints[end]
                .iter()
                .find(|boundary| boundary.operand == contained)
                .or_else(|| endpoints[end].first())
                .ok_or_else(fail)?;
            let source_ring = source_rings
                .iter()
                .find(|ring| {
                    ring.operand() == boundary.operand && ring.boundary() == boundary.boundary
                })
                .ok_or_else(fail)?;
            let mut cap = source_face(
                &source_faces,
                source_ring.cap_source(),
                source_ring.cap_face(),
            )?;
            let sibling = InternalTangencyBoundary {
                operand: boundary.operand,
                boundary: 1 - boundary.boundary,
                axial_parameter: relation
                    .axial_parameter(boundary.operand, 1 - boundary.boundary)
                    .ok_or_else(fail)?,
            };
            let source_low = compare_internal_boundaries(relation, *boundary, sibling)
                == core::cmp::Ordering::Less;
            if source_low != (end == 0) {
                cap.selected_orientation = reverse_selected_orientation(cap.selected_orientation);
            }
            cap.loops = vec![derived_ring_loop(
                ring_indices[end],
                opposite(directions[end]),
                None,
                derived,
            )?];
            faces.push(cap);
        }
    }
    Ok(())
}

fn graft_internal_tangency_union(
    faces: &mut Vec<MixedShellFacePlan>,
    rings: &mut Vec<MixedCylinderCapRing>,
    derived: &mut Vec<MixedDerivedRingPlan>,
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    tails: &[super::axial_interval_sweep::PlannedAxialSpan],
) -> Result<(), MixedShellPlanError> {
    use core::cmp::Ordering;

    use super::axial_interval_sweep::AxialIntervalOperand;
    use super::face_arrangement::certify_tangency_vertex;

    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    bind_internal_tangency_boundaries(cylinders, relation)?;
    if !(1..=2).contains(&tails.len()) {
        return Err(fail());
    }
    let contained = relation.contained_operand();
    let containing = relation.containing_operand();
    let source_faces = faces.clone();
    let source_rings = rings.clone();
    let [outer_low, outer_high] = ordered_internal_boundaries(relation, containing)?;
    let mut bands = Vec::with_capacity(tails.len() + 1);
    for (tail_index, tail) in tails.iter().enumerate() {
        let operand = if contained == 0 {
            AxialIntervalOperand::Left
        } else {
            AxialIntervalOperand::Right
        };
        if !tail.side_operands().contains(operand) {
            return Err(fail());
        }
        let low = single_internal_boundary(bind_internal_boundary_class(relation, tail.low())?)?;
        let high = single_internal_boundary(bind_internal_boundary_class(relation, tail.high())?)?;
        bands.push(InternalTangencyBand {
            operand: contained,
            low,
            high,
            split: (tails.len() == 2).then_some(if tail_index == 0 {
                AnalyticFaceSplitPiece::First
            } else {
                AnalyticFaceSplitPiece::Second
            }),
        });
    }
    bands.push(InternalTangencyBand {
        operand: containing,
        low: outer_low,
        high: outer_high,
        split: None,
    });
    bands.sort_by(|first, second| compare_internal_boundaries(relation, first.low, second.low));
    if bands.windows(2).any(|pair| {
        pair[0].operand == pair[1].operand
            || !same_internal_boundary(pair[0].high, pair[1].low)
            || pair[0].high.operand != containing
    }) {
        return Err(fail());
    }

    let outer = cylinders[containing];
    let inner = cylinders[contained];
    let outer_source_frame = *outer.cylinder().frame();
    let outer_low_center = internal_boundary_center(cylinders, outer_low)?;
    let trial_frame = Frame::new(
        outer_low_center,
        outer_source_frame.z(),
        outer_source_frame.x(),
    )
    .map_err(|_| fail())?;
    let (_, trial_height) = exact_internal_axial_projection(
        trial_frame,
        internal_boundary_center(cylinders, outer_high)?,
    )
    .ok_or_else(fail)?;
    let axis = match trial_height.total_cmp(&0.0) {
        Ordering::Greater => outer_source_frame.z(),
        Ordering::Less => -outer_source_frame.z(),
        Ordering::Equal => return Err(fail()),
    };
    let outer_origin = outer_low_center;
    let inner_origin = exact_internal_axial_projection(*inner.cylinder().frame(), outer_origin)
        .map(|projection| projection.0)
        .ok_or_else(fail)?;
    let radial = inner_origin - outer_origin;
    let outer_frame = Frame::new(outer_origin, axis, radial).map_err(|_| fail())?;
    let inner_frame = outer_frame.with_origin(inner_origin);
    if outer.cylinder().radius() <= inner.cylinder().radius() {
        return Err(fail());
    }

    let prepared = bands
        .iter()
        .map(|band| {
            let source = cylinders[band.operand];
            let frame = if band.operand == containing {
                outer_frame
            } else {
                inner_frame
            };
            let radius = source.cylinder().radius();
            let low_center = internal_boundary_center(cylinders, band.low)?;
            let high_center = internal_boundary_center(cylinders, band.high)?;
            let low = exact_internal_axial_projection(frame, low_center).ok_or_else(fail)?;
            let high = exact_internal_axial_projection(frame, high_center).ok_or_else(fail)?;
            (low.1 < high.1)
                .then_some((*band, frame, radius, low.0, high.0))
                .ok_or_else(fail)
        })
        .collect::<Result<Vec<_>, _>>()?;

    derived.clear();
    let far_boundaries = [prepared[0].0.low, prepared.last().ok_or_else(fail)?.0.high];
    let far_centers = [prepared[0].3, prepared.last().ok_or_else(fail)?.4];
    let far_frames = [prepared[0].1, prepared.last().ok_or_else(fail)?.1];
    let far_radii = [prepared[0].2, prepared.last().ok_or_else(fail)?.2];
    let mut far_ring_indices = [0_usize; 2];
    for end in 0..2 {
        far_ring_indices[end] = derived.len();
        derived.push(MixedDerivedRingPlan::endpoint_free(
            Circle::new(
                far_frames[end].with_origin(far_centers[end]),
                far_radii[end],
            )
            .map_err(|_| fail())?,
            MixedDerivedRingLineage::Source(
                cylinders[far_boundaries[end].operand].boundaries()[far_boundaries[end].boundary]
                    .edge(),
            ),
        ));
    }

    struct Contact {
        vertex: usize,
        outer_ring: usize,
        inner_ring: usize,
        boundary: InternalTangencyBoundary,
    }
    let mut contacts = Vec::with_capacity(tails.len());
    for contact_index in 0..tails.len() {
        let left = prepared[contact_index];
        let right = prepared[contact_index + 1];
        let boundary = left.0.high;
        if boundary.operand != containing || !same_internal_boundary(boundary, right.0.low) {
            return Err(fail());
        }
        let outer_center = if left.0.operand == containing {
            left.4
        } else {
            right.3
        };
        let inner_center = if left.0.operand == contained {
            left.4
        } else {
            right.3
        };
        let outer_circle = Circle::new(
            outer_frame.with_origin(outer_center),
            outer.cylinder().radius(),
        )
        .map_err(|_| fail())?;
        let inner_circle = Circle::new(
            inner_frame.with_origin(inner_center),
            inner.cylinder().radius(),
        )
        .map_err(|_| fail())?;
        let outer_ring = derived.len();
        let inner_ring = outer_ring + 1;
        certify_tangency_vertex(contact_index, [outer_ring, inner_ring]).map_err(|_| fail())?;
        let point = outer_circle.eval(0.0);
        derived.push(MixedDerivedRingPlan::tangent(
            outer_circle,
            contact_index,
            point,
            MixedDerivedRingLineage::Source(
                cylinders[containing].boundaries()[boundary.boundary].edge(),
            ),
        ));
        derived.push(MixedDerivedRingPlan::tangent(
            inner_circle,
            contact_index,
            point,
            MixedDerivedRingLineage::Derived([
                inner.side_face(),
                cylinders[containing].boundaries()[boundary.boundary].cap_face(),
            ]),
        ));
        contacts.push(Contact {
            vertex: contact_index,
            outer_ring,
            inner_ring,
            boundary,
        });
    }

    faces.clear();
    rings.clear();
    let mut side_directions = Vec::with_capacity(prepared.len());
    for (band_index, (band, _, _, low_center, high_center)) in prepared.iter().enumerate() {
        let source = cylinders[band.operand];
        let native = *source.cylinder().frame();
        let low_parameter = exact_internal_axial_projection(native, *low_center)
            .map(|projection| projection.1)
            .ok_or_else(fail)?;
        let high_parameter = exact_internal_axial_projection(native, *high_center)
            .map(|projection| projection.1)
            .ok_or_else(fail)?;
        let desired = if low_parameter < high_parameter {
            [ArrangementDirection::Forward, ArrangementDirection::Reverse]
        } else if low_parameter > high_parameter {
            [ArrangementDirection::Reverse, ArrangementDirection::Forward]
        } else {
            return Err(fail());
        };
        let ring_indices = [
            if band_index == 0 {
                far_ring_indices[0]
            } else if band.operand == contained {
                contacts[band_index - 1].inner_ring
            } else {
                contacts[band_index - 1].outer_ring
            },
            if band_index + 1 == prepared.len() {
                far_ring_indices[1]
            } else if band.operand == contained {
                contacts[band_index].inner_ring
            } else {
                contacts[band_index].outer_ring
            },
        ];
        let directions: [ArrangementDirection; 2] = core::array::from_fn(|end| {
            if derived_ring_cylinder_scale(
                derived[ring_indices[end]].circle(),
                *source.cylinder().frame(),
            ) > 0.0
            {
                desired[end]
            } else {
                opposite(desired[end])
            }
        });
        let source_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == band.operand)
            .ok_or_else(fail)?;
        let mut face = source_face(
            &source_faces,
            source_ring.side_source(),
            source_ring.side_face(),
        )?;
        face.loops = vec![
            derived_ring_loop(ring_indices[0], directions[0], Some(low_parameter), derived)?,
            derived_ring_loop(
                ring_indices[1],
                directions[1],
                Some(high_parameter),
                derived,
            )?,
        ];
        face.split_lineage = band.split;
        faces.push(face);
        side_directions.push(directions);
    }

    for (end, band_index) in [0, prepared.len() - 1].into_iter().enumerate() {
        let boundary = far_boundaries[end];
        let source_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == boundary.operand && ring.boundary() == boundary.boundary)
            .ok_or_else(fail)?;
        let mut cap = source_face(
            &source_faces,
            source_ring.cap_source(),
            source_ring.cap_face(),
        )?;
        cap.loops = vec![derived_ring_loop(
            far_ring_indices[end],
            opposite(side_directions[band_index][end]),
            None,
            derived,
        )?];
        faces.push(cap);
    }
    for (index, contact) in contacts.iter().enumerate() {
        let left = prepared[index].0;
        let outer_side = if left.operand == containing {
            side_directions[index][1]
        } else {
            side_directions[index + 1][0]
        };
        let inner_side = if left.operand == contained {
            side_directions[index][1]
        } else {
            side_directions[index + 1][0]
        };
        if outer_side == inner_side {
            return Err(fail());
        }
        let source_ring = source_rings
            .iter()
            .find(|ring| {
                ring.operand() == containing && ring.boundary() == contact.boundary.boundary
            })
            .ok_or_else(fail)?;
        let mut shoulder = source_face(
            &source_faces,
            source_ring.cap_source(),
            source_ring.cap_face(),
        )?;
        shoulder.loops = vec![MixedShellLoopPlan {
            uses: vec![
                derived_ring_use(contact.outer_ring, opposite(outer_side), None),
                derived_ring_use(contact.inner_ring, opposite(inner_side), None),
            ],
            vertices: vec![
                MixedShellVertexKey::Tangency(contact.vertex),
                MixedShellVertexKey::Tangency(contact.vertex),
                MixedShellVertexKey::Tangency(contact.vertex),
            ],
        }];
        faces.push(shoulder);
    }
    Ok(())
}

fn source_face(
    faces: &[MixedShellFacePlan],
    source: MixedSourceFaceKey,
    face: &FaceId,
) -> Result<MixedShellFacePlan, MixedShellPlanError> {
    let matching = faces
        .iter()
        .filter(|candidate| candidate.source == source && candidate.source_face == *face)
        .cloned()
        .collect::<Vec<_>>();
    let [face] = matching.as_slice() else {
        return Err(MixedShellPlanError::InternalTangencyBoundaryMismatch);
    };
    let mut face = face.clone();
    face.merge_sources = None;
    face.split_lineage = None;
    Ok(face)
}

fn derived_ring_use(
    ring: usize,
    direction: ArrangementDirection,
    cylinder_parameter: Option<f64>,
) -> MixedShellEdgeUse {
    MixedShellEdgeUse {
        edge: MixedShellEdgeKey::DerivedRing(ring),
        direction,
        pcurve: MixedPcurveLineage::DerivedRing {
            cylinder_parameter_bits: cylinder_parameter.map(f64::to_bits),
        },
    }
}

fn derived_ring_cylinder_scale(circle: Circle, cylinder: Frame) -> f64 {
    let local_x = [
        circle.frame().x().dot(cylinder.x()),
        circle.frame().x().dot(cylinder.y()),
    ];
    let local_y = [
        circle.frame().y().dot(cylinder.x()),
        circle.frame().y().dot(cylinder.y()),
    ];
    if local_x[0] * local_y[1] - local_x[1] * local_y[0] > 0.0 {
        1.0
    } else {
        -1.0
    }
}

const fn reverse_selected_orientation(orientation: SelectedOrientation) -> SelectedOrientation {
    match orientation {
        SelectedOrientation::Preserved => SelectedOrientation::Reversed,
        SelectedOrientation::Reversed => SelectedOrientation::Preserved,
    }
}

fn derived_ring_loop(
    ring: usize,
    direction: ArrangementDirection,
    cylinder_parameter: Option<f64>,
    rings: &[MixedDerivedRingPlan],
) -> Result<MixedShellLoopPlan, MixedShellPlanError> {
    let planned = rings
        .get(ring)
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)?;
    let vertex = if let Some((vertex, _)) = planned.tangency() {
        MixedShellVertexKey::Tangency(vertex)
    } else {
        MixedShellVertexKey::DerivedRingSeam(ring)
    };
    Ok(MixedShellLoopPlan {
        uses: vec![derived_ring_use(ring, direction, cylinder_parameter)],
        vertices: vec![vertex.clone(), vertex],
    })
}

fn bind_internal_tangency_boundaries(
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    let mut seen = [[false; 2]; 2];
    for witness in relation.boundaries() {
        let boundary = cylinders
            .get(witness.operand())
            .and_then(|source| source.boundaries().get(witness.boundary()))
            .ok_or_else(fail)?;
        if seen[witness.operand()][witness.boundary()]
            || boundary.cap_face() != witness.cap_face()
            || boundary.edge() != witness.edge()
        {
            return Err(fail());
        }
        seen[witness.operand()][witness.boundary()] = true;
    }
    (seen == [[true; 2]; 2]).then_some(()).ok_or_else(fail)
}

fn bind_internal_boundary_class(
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    contributors: super::axial_interval_sweep::AxialEndpointContributors,
) -> Result<Vec<InternalTangencyBoundary>, MixedShellPlanError> {
    use super::axial_interval_sweep::{AuthoredAxialEndpoint, AxialIntervalOperand};

    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    let mut boundaries = Vec::with_capacity(2);
    for contributor in contributors.iter() {
        let operand = match contributor.operand() {
            AxialIntervalOperand::Left => 0,
            AxialIntervalOperand::Right => 1,
        };
        let boundary = match contributor.endpoint() {
            AuthoredAxialEndpoint::Start => 0,
            AuthoredAxialEndpoint::End => 1,
        };
        if boundaries
            .iter()
            .any(|candidate: &InternalTangencyBoundary| candidate.operand == operand)
        {
            return Err(fail());
        }
        boundaries.push(InternalTangencyBoundary {
            operand,
            boundary,
            axial_parameter: relation
                .axial_parameter(operand, boundary)
                .ok_or_else(fail)?,
        });
    }
    (!boundaries.is_empty() && boundaries.len() <= 2)
        .then_some(boundaries)
        .ok_or_else(fail)
}

fn internal_axis_endpoint(
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    source: &super::curved_source::CertifiedCylinderSource,
    operand: usize,
    boundaries: &[InternalTangencyBoundary],
) -> Result<(Point3, f64), MixedShellPlanError> {
    if let Some(boundary) = boundaries
        .iter()
        .find(|boundary| boundary.operand == operand)
    {
        return Ok((
            source.boundaries()[boundary.boundary].center(),
            boundary.axial_parameter,
        ));
    }
    let boundary = boundaries
        .first()
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)?;
    exact_internal_axial_projection(
        *source.cylinder().frame(),
        internal_boundary_center(cylinders, *boundary)?,
    )
    .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)
}

fn internal_boundary_center(
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    boundary: InternalTangencyBoundary,
) -> Result<Point3, MixedShellPlanError> {
    cylinders
        .get(boundary.operand)
        .and_then(|source| source.boundaries().get(boundary.boundary))
        .map(|boundary| boundary.center())
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)
}

fn internal_ring_lineage(
    cylinders: [&super::curved_source::CertifiedCylinderSource; 2],
    contained_side: RawFaceId,
    contained: usize,
    boundaries: &[InternalTangencyBoundary],
) -> Result<MixedDerivedRingLineage, MixedShellPlanError> {
    if let Some(boundary) = boundaries
        .iter()
        .find(|boundary| boundary.operand == contained)
    {
        return cylinders
            .get(contained)
            .and_then(|source| source.boundaries().get(boundary.boundary))
            .map(|boundary| MixedDerivedRingLineage::Source(boundary.edge()))
            .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch);
    }
    let cutting = boundaries
        .first()
        .and_then(|boundary| {
            cylinders
                .get(boundary.operand)
                .and_then(|source| source.boundaries().get(boundary.boundary))
        })
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)?;
    Ok(MixedDerivedRingLineage::Derived([
        contained_side,
        cutting.cap_face(),
    ]))
}

fn ordered_internal_boundaries(
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    operand: usize,
) -> Result<[InternalTangencyBoundary; 2], MixedShellPlanError> {
    let boundaries = [0, 1].map(|boundary| InternalTangencyBoundary {
        operand,
        boundary,
        axial_parameter: relation
            .axial_parameter(operand, boundary)
            .unwrap_or(f64::NAN),
    });
    match compare_internal_boundaries(relation, boundaries[0], boundaries[1]) {
        core::cmp::Ordering::Less => Ok(boundaries),
        core::cmp::Ordering::Greater => Ok([boundaries[1], boundaries[0]]),
        core::cmp::Ordering::Equal => Err(MixedShellPlanError::InternalTangencyBoundaryMismatch),
    }
}

fn single_internal_boundary(
    boundaries: Vec<InternalTangencyBoundary>,
) -> Result<InternalTangencyBoundary, MixedShellPlanError> {
    let [boundary] = boundaries.as_slice() else {
        return Err(MixedShellPlanError::InternalTangencyBoundaryMismatch);
    };
    Ok(*boundary)
}

fn compare_internal_boundaries(
    relation: &super::parallel_cylinder_relation::CertifiedParallelCylinderInternalRadialTangency,
    first: InternalTangencyBoundary,
    second: InternalTangencyBoundary,
) -> core::cmp::Ordering {
    use super::axial_interval_sweep::{
        AuthoredAxialEndpoint, AxialEndpointContributor, AxialIntervalOperand,
    };

    let contributor = |boundary: InternalTangencyBoundary| {
        AxialEndpointContributor::new(
            if boundary.operand == 0 {
                AxialIntervalOperand::Left
            } else {
                AxialIntervalOperand::Right
            },
            if boundary.boundary == 0 {
                AuthoredAxialEndpoint::Start
            } else {
                AuthoredAxialEndpoint::End
            },
        )
    };
    relation
        .preorder()
        .compare(contributor(first), contributor(second))
}

fn same_internal_boundary(
    first: InternalTangencyBoundary,
    second: InternalTangencyBoundary,
) -> bool {
    first.operand == second.operand && first.boundary == second.boundary
}

fn exact_internal_axial_projection(frame: Frame, point: Point3) -> Option<(Point3, f64)> {
    let origin = frame.origin();
    let axis = frame.z();
    let delta = point - origin;
    let axis_components = axis.to_array();
    let delta_components = delta.to_array();
    let candidates = [
        delta.dot(axis),
        delta_components[0] / axis_components[0],
        delta_components[1] / axis_components[1],
        delta_components[2] / axis_components[2],
    ];
    candidates.into_iter().find_map(|parameter| {
        if !parameter.is_finite() {
            return None;
        }
        let center = origin + axis * parameter;
        (axis_parameter_identity_is_exact(center, origin, axis, parameter)
            && affine_dot3(axis.to_array(), center.to_array(), point.to_array(), 0.0)
                .is_some_and(|orientation| orientation.sign() == Orientation::Zero))
        .then_some((center, parameter))
    })
}

fn axis_parameter_identity_is_exact(
    point: Point3,
    origin: Point3,
    axis: kgeom::vec::Vec3,
    parameter: f64,
) -> bool {
    let point = point.to_array();
    let origin = origin.to_array();
    let axis = axis.to_array();
    (0..3).all(|component| {
        affine_dot3(
            [1.0, axis[component], -1.0],
            [origin[component], parameter, point[component]],
            [0.0; 3],
            0.0,
        )
        .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}

fn graft_common_support_spans(
    store: &Store,
    faces: &mut Vec<MixedShellFacePlan>,
    rings: &mut Vec<MixedCylinderCapRing>,
    interval: &super::axial_interval_sweep::AxialIntervalPlan,
    preorder: &super::axial_interval_sweep::CertifiedAxialEndpointPreorder,
    tolerance: f64,
) -> Result<(), MixedShellPlanError> {
    use core::cmp::Ordering;

    use super::axial_interval_sweep::{
        AuthoredAxialEndpoint, AxialEndpointContributor, AxialIntervalOperand,
    };

    let fail = || MixedShellPlanError::CommonSupportBoundaryMismatch;
    let source_faces = faces.clone();
    let source_rings = rings.clone();
    let split_operand = match interval.spans() {
        [first, second] => {
            let first = sole_interval_operand(first.side_operands());
            (first.is_some() && first == sole_interval_operand(second.side_operands()))
                .then_some(first)
                .flatten()
        }
        _ => None,
    };
    faces.clear();
    rings.clear();
    for (span_index, span) in interval.spans().iter().enumerate() {
        let target_operand = if span.side_operands().contains(AxialIntervalOperand::Left) {
            0
        } else if span.side_operands().contains(AxialIntervalOperand::Right) {
            1
        } else {
            return Err(fail());
        };
        let target_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == target_operand)
            .ok_or_else(fail)?;
        let target = source_faces
            .iter()
            .find(|face| {
                face.source == target_ring.side_source()
                    && face.source_face == *target_ring.side_face()
            })
            .ok_or_else(fail)?;
        let mut boundary_faces = Vec::with_capacity(2);
        let mut boundary_loops = Vec::with_capacity(2);
        for (output_end, contributors) in [span.low(), span.high()].into_iter().enumerate() {
            let primary_contributor = contributors.iter().next().ok_or_else(fail)?;
            let mut endpoint_rings = contributors
                .iter()
                .map(|contributor| {
                    let operand = match contributor.operand() {
                        AxialIntervalOperand::Left => 0,
                        AxialIntervalOperand::Right => 1,
                    };
                    let boundary = match contributor.endpoint() {
                        AuthoredAxialEndpoint::Start => 0,
                        AuthoredAxialEndpoint::End => 1,
                    };
                    source_rings
                        .iter()
                        .find(|ring| ring.operand() == operand && ring.boundary() == boundary)
                        .cloned()
                        .ok_or_else(fail)
                })
                .collect::<Result<Vec<_>, _>>()?;
            let [primary, rest @ ..] = endpoint_rings.as_mut_slice() else {
                return Err(fail());
            };
            if rest.len() > 1
                || rest
                    .first()
                    .is_some_and(|peer| peer.edge() == primary.edge())
            {
                return Err(fail());
            }
            let owner = source_faces
                .iter()
                .find(|face| {
                    face.source == primary.side_source() && face.source_face == *primary.side_face()
                })
                .ok_or_else(fail)?;
            let matching = owner
                .loops
                .iter()
                .filter(|loop_| {
                    loop_.uses.len() == 1
                        && loop_.uses[0].edge
                            == (MixedShellEdgeKey::PeriodicSource {
                                source: primary.side_source(),
                                loop_key: primary.side_loop_key(),
                            })
                })
                .cloned()
                .collect::<Vec<_>>();
            let [mut loop_] = matching.try_into().map_err(|_| fail())?;
            if primary.side_source() != target.source {
                let proof = ProjectedEndpointFreeSourceCircle::certify(
                    store,
                    primary,
                    target.source,
                    &target.source_face,
                    tolerance,
                )
                .map_err(MixedShellPlanError::ProjectedSourceCircle)?;
                loop_.uses[0].pcurve = MixedPcurveLineage::ProjectedEndpointFreeSourceCircle(proof);
            }
            let mut cap = source_faces
                .iter()
                .find(|face| {
                    face.source == primary.cap_source() && face.source_face == *primary.cap_face()
                })
                .cloned()
                .ok_or_else(fail)?;
            let other = AxialEndpointContributor::new(
                primary_contributor.operand(),
                match primary_contributor.endpoint() {
                    AuthoredAxialEndpoint::Start => AuthoredAxialEndpoint::End,
                    AuthoredAxialEndpoint::End => AuthoredAxialEndpoint::Start,
                },
            );
            let source_low = match preorder.compare(primary_contributor, other) {
                Ordering::Less => true,
                Ordering::Greater => false,
                Ordering::Equal => return Err(fail()),
            };
            if source_low != (output_end == 0) {
                cap.selected_orientation = match cap.selected_orientation {
                    SelectedOrientation::Preserved => SelectedOrientation::Reversed,
                    SelectedOrientation::Reversed => SelectedOrientation::Preserved,
                };
                loop_.uses[0].direction = opposite(loop_.uses[0].direction);
            }
            if let [peer] = rest {
                *primary = primary.clone().with_merge_edge_source(peer.edge());
                cap.merge_sources = Some([primary.cap_face().clone(), peer.cap_face().clone()]);
            }
            boundary_loops.push(loop_);
            boundary_faces.push(cap);
            rings.push(primary.clone());
        }
        let both = span.side_operands().contains(AxialIntervalOperand::Left)
            && span.side_operands().contains(AxialIntervalOperand::Right);
        let source_side_faces = [0, 1]
            .map(|operand| {
                source_rings
                    .iter()
                    .find(|ring| ring.operand() == operand)
                    .map(|ring| ring.side_face().clone())
                    .ok_or_else(fail)
            })
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?;
        let [first_side_face, second_side_face] =
            source_side_faces.try_into().map_err(|_| fail())?;
        faces.push(MixedShellFacePlan {
            source: target.source,
            source_face: target.source_face.clone(),
            selected_orientation: target.selected_orientation,
            loops: boundary_loops,
            merge_sources: both.then_some([first_side_face, second_side_face]),
            split_lineage: (split_operand
                == Some(if target_operand == 0 {
                    AxialIntervalOperand::Left
                } else {
                    AxialIntervalOperand::Right
                }))
            .then_some(if span_index == 0 {
                AnalyticFaceSplitPiece::First
            } else {
                AnalyticFaceSplitPiece::Second
            }),
        });
        faces.extend(boundary_faces);
    }
    Ok(())
}

fn sole_interval_operand(
    operands: super::axial_interval_sweep::AxialOperandContributors,
) -> Option<super::axial_interval_sweep::AxialIntervalOperand> {
    let mut operands = operands.iter();
    let first = operands.next()?;
    operands.next().is_none().then_some(first)
}

fn append_internal_contact_hole(
    store: &Store,
    faces: &mut [MixedShellFacePlan],
    cap_rings: &mut Vec<MixedCylinderCapRing>,
    outer: &MixedCylinderCapRing,
    inner: &MixedCylinderCapRing,
    tolerance: f64,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::AxialContactBoundaryMismatch;
    if outer.side_source() == inner.side_source()
        || cap_rings.iter().any(|ring| {
            ring.side_source() == inner.side_source()
                && ring.side_loop_key() == inner.side_loop_key()
        })
    {
        return Err(fail());
    }
    let matching = faces
        .iter()
        .enumerate()
        .filter(|face| {
            face.1.source == outer.cap_source() && face.1.source_face == *outer.cap_face()
        })
        .map(|(index, _)| index)
        .collect::<Vec<_>>();
    let [target_index] = matching.as_slice() else {
        return Err(fail());
    };
    let target = &mut faces[*target_index];
    if target.loops.len() != 1 {
        return Err(fail());
    }
    let proof = ProjectedEndpointFreeSourceCircle::certify(
        store,
        inner,
        target.source,
        &target.source_face,
        tolerance,
    )
    .map_err(MixedShellPlanError::ProjectedSourceCircle)?;
    let seam = MixedShellVertexKey::ProofSeam {
        source: inner.side_source(),
        loop_key: inner.side_loop_key(),
    };
    target.loops.push(MixedShellLoopPlan {
        uses: vec![MixedShellEdgeUse {
            edge: MixedShellEdgeKey::PeriodicSource {
                source: inner.side_source(),
                loop_key: inner.side_loop_key(),
            },
            direction: ArrangementDirection::Forward,
            pcurve: MixedPcurveLineage::ProjectedEndpointFreeSourceCircle(proof),
        }],
        vertices: vec![seam.clone(), seam],
    });
    cap_rings.push(inner.clone());
    Ok(())
}

fn merge_coincident_side_faces(
    store: &Store,
    faces: &mut Vec<MixedShellFacePlan>,
    far_rings: [&MixedCylinderCapRing; 2],
    tolerance: f64,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::AxialContactBoundaryMismatch;
    if far_rings[0].side_source() == far_rings[1].side_source() {
        return Err(fail());
    }
    let mut indices = [None; 2];
    for (ring_index, ring) in far_rings.iter().enumerate() {
        let matching = faces
            .iter()
            .enumerate()
            .filter(|(_, face)| {
                face.source == ring.side_source() && face.source_face == *ring.side_face()
            })
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        let [index] = matching.as_slice() else {
            return Err(fail());
        };
        indices[ring_index] = Some(*index);
    }
    let [Some(first_index), Some(second_index)] = indices else {
        return Err(fail());
    };
    if first_index == second_index {
        return Err(fail());
    }
    let first = faces[first_index].clone();
    let second = faces[second_index].clone();
    if first.selected_orientation != second.selected_orientation
        || first.merge_sources.is_some()
        || second.merge_sources.is_some()
    {
        return Err(fail());
    }
    let mut far_loops = Vec::with_capacity(2);
    for (face, ring) in [(&first, far_rings[0]), (&second, far_rings[1])] {
        let matching = face
            .loops
            .iter()
            .filter(|loop_| {
                loop_.uses.len() == 1
                    && loop_.uses[0].edge
                        == (MixedShellEdgeKey::PeriodicSource {
                            source: ring.side_source(),
                            loop_key: ring.side_loop_key(),
                        })
            })
            .cloned()
            .collect::<Vec<_>>();
        let [loop_] = matching.as_slice() else {
            return Err(fail());
        };
        far_loops.push(loop_.clone());
    }
    let proof = ProjectedEndpointFreeSourceCircle::certify(
        store,
        far_rings[1],
        first.source,
        &first.source_face,
        tolerance,
    )
    .map_err(MixedShellPlanError::ProjectedSourceCircle)?;
    far_loops[1].uses[0].pcurve = MixedPcurveLineage::ProjectedEndpointFreeSourceCircle(proof);

    let insert_at = first_index.min(second_index);
    for index in [first_index, second_index]
        .into_iter()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .rev()
    {
        faces.remove(index);
    }
    faces.insert(
        insert_at,
        MixedShellFacePlan {
            source: first.source,
            source_face: first.source_face.clone(),
            selected_orientation: first.selected_orientation,
            loops: far_loops,
            merge_sources: Some([first.source_face, second.source_face]),
            split_lineage: None,
        },
    );
    Ok(())
}

fn rebase_axial_contact_boundary_arcs(
    store: &Store,
    graph: &BodySectionGraph,
    arrangements: &BTreeMap<MixedSourceFaceKey, MixedArrangementBinding<'_>>,
    faces: &mut [MixedShellFacePlan],
    bounded_source_spans: &mut [MixedBoundedSourceSpanPlan],
    tolerance: f64,
) -> Result<(), MixedShellPlanError> {
    #[derive(Clone)]
    struct BoundaryRebase {
        fragment: usize,
        source: MixedSourceFaceKey,
        source_face: FaceId,
        span: MixedSourceSpanKey,
    }

    let fail = || MixedShellPlanError::AxialContactBoundaryMismatch;
    let mut rebases = BTreeMap::new();
    for (&source, binding) in arrangements {
        let MixedArrangementBinding::Periodic {
            face, arrangement, ..
        } = binding
        else {
            continue;
        };
        let overlays = arrangement
            .cells()
            .iter()
            .filter(|cell| cell.key() != &PeriodicArrangementCellKey::AnnularRemainder)
            .collect::<Vec<_>>();
        let [overlay] = overlays.as_slice() else {
            return Err(fail());
        };
        let mut source_keys = BTreeSet::new();
        let mut cut_keys = BTreeSet::new();
        for use_ in overlay.boundaries().iter().flat_map(ArrangementCycle::uses) {
            match use_.edge() {
                ArrangementEdgeKey::Source(key) if !key.is_whole_loop() => {
                    source_keys.insert(*key);
                }
                ArrangementEdgeKey::Cut(key) => {
                    cut_keys.insert(*key);
                }
                ArrangementEdgeKey::Source(_) => {}
            }
        }
        let mut source_keys = source_keys.into_iter();
        let Some(loop_key) = source_keys.next() else {
            return Err(fail());
        };
        let mut cut_keys = cut_keys.into_iter();
        let Some(cut) = cut_keys.next() else {
            return Err(fail());
        };
        if source_keys.next().is_some() || cut_keys.next().is_some() {
            return Err(fail());
        }
        let span = MixedSourceSpanKey {
            fin_loop_ordinal: loop_key.topology_ordinal(),
            traversal_ordinal: loop_key.cyclic_span_ordinal().ok_or_else(fail)?,
        };
        let retained = bounded_source_spans
            .iter()
            .find(|candidate| candidate.source == source && candidate.span == span)
            .ok_or_else(fail)?;
        let fragment = graph
            .curve_fragments()
            .get(cut.fragment())
            .ok_or_else(fail)?;
        let mut expected = fragment_endpoints(fragment).ok_or_else(fail)?;
        let mut actual = retained.roots.map(MixedBoundedSourceRoot::endpoint);
        expected.sort_unstable();
        actual.sort_unstable();
        if expected != actual
            || rebases
                .insert(
                    cut.fragment(),
                    BoundaryRebase {
                        fragment: cut.fragment(),
                        source,
                        source_face: face.clone(),
                        span,
                    },
                )
                .is_some()
        {
            return Err(fail());
        }
    }
    if rebases.len() != graph.curve_fragments().len() {
        return Err(fail());
    }

    for rebase in rebases.into_values() {
        let retained = bounded_source_spans
            .iter()
            .find(|candidate| candidate.source == rebase.source && candidate.span == rebase.span)
            .cloned()
            .ok_or_else(fail)?;
        let retained_endpoints = retained.roots.map(MixedBoundedSourceRoot::endpoint);
        let mut source_uses = 0_usize;
        let mut projected_uses = 0_usize;
        for face in faces.iter_mut() {
            let face_source = face.source;
            let target_face = face.source_face.clone();
            for loop_ in &mut face.loops {
                for use_index in 0..loop_.uses.len() {
                    if loop_.uses[use_index].edge
                        != MixedShellEdgeKey::SectionFragment(rebase.fragment)
                    {
                        continue;
                    }
                    let endpoints =
                        match (&loop_.vertices[use_index], &loop_.vertices[use_index + 1]) {
                            (
                                MixedShellVertexKey::SectionEndpoint(start),
                                MixedShellVertexKey::SectionEndpoint(end),
                            ) => [*start, *end],
                            _ => return Err(fail()),
                        };
                    let direction = if endpoints == retained_endpoints {
                        ArrangementDirection::Forward
                    } else if endpoints == [retained_endpoints[1], retained_endpoints[0]] {
                        ArrangementDirection::Reverse
                    } else {
                        return Err(fail());
                    };
                    let pcurve = if face_source == rebase.source {
                        if target_face != rebase.source_face {
                            return Err(fail());
                        }
                        source_uses = source_uses.checked_add(1).ok_or_else(fail)?;
                        MixedPcurveLineage::SourceTopology
                    } else {
                        projected_uses = projected_uses.checked_add(1).ok_or_else(fail)?;
                        MixedPcurveLineage::ProjectedSourceCircleOnPlane(
                            ProjectedSourceCircleOnPlane::certify(
                                store,
                                &rebase.source_face,
                                &retained,
                                face_source,
                                &target_face,
                                tolerance,
                            )
                            .map_err(MixedShellPlanError::ProjectedSourceCircle)?,
                        )
                    };
                    loop_.uses[use_index] = MixedShellEdgeUse {
                        edge: MixedShellEdgeKey::PlanarSource {
                            source: rebase.source,
                            span: rebase.span.clone(),
                        },
                        direction,
                        pcurve,
                    };
                }
            }
        }
        if source_uses != 1 || projected_uses != 1 {
            return Err(fail());
        }
    }
    Ok(())
}

fn plan_mixed_shell_with_augmentation<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    admission: SectionPlanningAdmission<'_>,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = (MixedShellCellKey, OperandSide, SelectedOrientation)>,
    augment: impl FnOnce(
        &BTreeMap<MixedSourceFaceKey, MixedArrangementBinding<'a>>,
        &mut Vec<MixedShellFacePlan>,
        &mut Vec<MixedBoundedSourceSpanPlan>,
        &mut Vec<MixedCylinderCapRing>,
        &mut Vec<MixedDerivedRingPlan>,
    ) -> Result<(), MixedShellPlanError>,
) -> Result<MixedShellProofPlan, MixedShellPlanError> {
    admission.validate(graph)?;

    let mut arrangements = BTreeMap::new();
    for binding in bindings {
        let source = source_face_key(store, graph, binding.face(), binding.operand())?;
        if arrangements.insert(source, binding).is_some() {
            return Err(MixedShellPlanError::DuplicateArrangement(source));
        }
    }

    let mut selected_cells = BTreeMap::new();
    for (key, operand, orientation) in selected {
        if operand != operand_side(key.source.operand) {
            return Err(MixedShellPlanError::SelectionOperandMismatch(key));
        }
        if selected_cells.insert(key, orientation).is_some() {
            return Err(MixedShellPlanError::DuplicateSelectedCell(key));
        }
    }
    if selected_cells.is_empty() {
        return Err(MixedShellPlanError::EmptySelection);
    }

    let mut faces = Vec::with_capacity(selected_cells.len());
    let mut cap_rings = Vec::new();
    let mut derived_rings = Vec::new();
    let mut bounded_source_spans = Vec::new();
    for (key, orientation) in selected_cells {
        let binding = arrangements
            .get(&key.source)
            .ok_or(MixedShellPlanError::MissingArrangement(key.source))?;
        let face = match (binding, key.cell) {
            (
                MixedArrangementBinding::Planar {
                    face,
                    operand,
                    arrangement,
                    lineage,
                },
                MixedShellCellKind::Planar(cell_key),
            ) => {
                let cell = arrangement
                    .cells()
                    .iter()
                    .find(|cell| cell.key() == cell_key)
                    .ok_or(MixedShellPlanError::MissingPlanarCell(key))?;
                validate_planar_lineage(
                    store,
                    graph,
                    face,
                    *operand,
                    arrangement,
                    lineage,
                    key.source,
                )?;
                let lineage = planar_cut_lineage(graph, face, *operand, arrangement, key.source)?;
                let loops = cell
                    .boundaries()
                    .iter()
                    .map(|boundary| {
                        planar_loop(
                            graph,
                            key,
                            key.source,
                            *operand,
                            boundary,
                            &lineage,
                            orientation,
                        )
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                MixedShellFacePlan {
                    source: key.source,
                    source_face: face.clone(),
                    selected_orientation: orientation,
                    loops,
                    merge_sources: None,
                    split_lineage: None,
                }
            }
            (
                MixedArrangementBinding::Disk {
                    face,
                    operand,
                    arranged,
                },
                MixedShellCellKind::Disk(cell_key),
            ) => {
                let cell = arranged
                    .arrangement()
                    .cells()
                    .iter()
                    .find(|cell| cell.key() == cell_key)
                    .ok_or(MixedShellPlanError::MissingDiskCell(key))?;
                let (source_spans, retained) =
                    bind_disk_source_spans(store, graph, face, *operand, arranged, key.source)?;
                for span in retained {
                    if !bounded_source_spans
                        .iter()
                        .any(|candidate: &MixedBoundedSourceSpanPlan| {
                            candidate.source == span.source && candidate.span == span.span
                        })
                    {
                        bounded_source_spans.push(span);
                    }
                }
                let cut_lineage = disk_cut_lineage(graph, face, *operand, arranged, key.source)?;
                let loop_plan = disk_loop(
                    graph,
                    key,
                    key.source,
                    *operand,
                    face,
                    cell.boundary(),
                    &source_spans,
                    &cut_lineage,
                    orientation,
                )?;
                MixedShellFacePlan {
                    source: key.source,
                    source_face: face.clone(),
                    selected_orientation: orientation,
                    loops: vec![loop_plan],
                    merge_sources: None,
                    split_lineage: None,
                }
            }
            (
                MixedArrangementBinding::Periodic {
                    face,
                    operand,
                    arrangement,
                    embedding,
                },
                MixedShellCellKind::Periodic(cell_key),
            ) => {
                let cell = arrangement
                    .cells()
                    .iter()
                    .find(|cell| *cell.key() == cell_key)
                    .ok_or(MixedShellPlanError::MissingPeriodicCell(key))?;
                let (periodic_spans, retained) = bind_periodic_source_spans(
                    store,
                    graph,
                    face,
                    *operand,
                    arrangement,
                    *embedding,
                    key.source,
                )?;
                for span in retained {
                    if !bounded_source_spans
                        .iter()
                        .any(|candidate: &MixedBoundedSourceSpanPlan| {
                            candidate.source == span.source && candidate.span == span.span
                        })
                    {
                        bounded_source_spans.push(span);
                    }
                }
                let lineage = periodic_cut_lineage(
                    graph,
                    face,
                    *operand,
                    arrangement,
                    *embedding,
                    key.source,
                )?;
                let loops = cell
                    .boundaries()
                    .iter()
                    .map(|cycle| {
                        periodic_loop(
                            graph,
                            key,
                            key.source,
                            *operand,
                            cycle,
                            &periodic_spans,
                            &lineage,
                            orientation,
                        )
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                MixedShellFacePlan {
                    source: key.source,
                    source_face: face.clone(),
                    selected_orientation: orientation,
                    loops,
                    merge_sources: None,
                    split_lineage: None,
                }
            }
            (
                MixedArrangementBinding::CylinderCap { ring },
                MixedShellCellKind::CylinderCap(boundary),
            ) => {
                if ring.cap_source() != key.source
                    || ring.operand() != key.source.operand()
                    || ring.boundary() != boundary
                    || ring.cap_face() != binding.face()
                {
                    return Err(MixedShellPlanError::CylinderCapBindingMismatch(key));
                }
                let seam = MixedShellVertexKey::ProofSeam {
                    source: ring.side_source(),
                    loop_key: ring.side_loop_key(),
                };
                let loop_ = MixedShellLoopPlan {
                    uses: vec![MixedShellEdgeUse {
                        edge: MixedShellEdgeKey::PeriodicSource {
                            source: ring.side_source(),
                            loop_key: ring.side_loop_key(),
                        },
                        // Resolved from the selected periodic-side use once
                        // every selected face has been planned.
                        direction: ArrangementDirection::Forward,
                        pcurve: MixedPcurveLineage::SourceTopology,
                    }],
                    vertices: vec![seam.clone(), seam],
                };
                cap_rings.push((*ring).clone());
                MixedShellFacePlan {
                    source: key.source,
                    source_face: ring.cap_face().clone(),
                    selected_orientation: orientation,
                    loops: vec![loop_],
                    merge_sources: None,
                    split_lineage: None,
                }
            }
            _ => return Err(MixedShellPlanError::ArrangementKindMismatch(key)),
        };
        faces.push(face);
    }

    augment(
        &arrangements,
        &mut faces,
        &mut bounded_source_spans,
        &mut cap_rings,
        &mut derived_rings,
    )?;
    resolve_endpoint_free_cap_directions(&mut faces, &cap_rings)?;
    validate_section_pairing(&faces)?;
    validate_endpoint_free_ring_pairing(&faces)?;
    validate_derived_ring_pairing(&faces, &derived_rings)?;
    bounded_source_spans.retain(|span| bounded_source_span_is_used(&faces, span));
    validate_bounded_source_pairing(store, &faces, &bounded_source_spans)?;
    let section_edges = collect_section_edges(store, graph, &faces)?;
    let materialization = materialize::retain_materialization_evidence(
        &faces,
        &arrangements,
        &bounded_source_spans,
        graph,
        &section_edges,
    );
    let materialization_gaps = materialize::remaining_gaps(&materialization);
    Ok(MixedShellProofPlan {
        faces,
        section_edges,
        bounded_source_spans,
        cap_rings,
        derived_rings,
        materialization,
        materialization_gaps,
    })
}

pub(crate) fn source_face_key(
    store: &Store,
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<MixedSourceFaceKey, MixedShellPlanError> {
    let body = graph
        .bodies()
        .get(operand)
        .ok_or(MixedShellPlanError::InvalidOperand(operand))?;
    if body.part() != face.part() {
        return Err(MixedShellPlanError::FacePartMismatch);
    }
    let faces = store
        .faces_of_body(body.raw())
        .map_err(|_| MixedShellPlanError::SourceBodyUnavailable(operand))?;
    let topology_ordinal = faces
        .iter()
        .position(|candidate| *candidate == face.raw())
        .ok_or_else(|| MixedShellPlanError::FaceNotOwnedByOperand {
            operand,
            face: face.clone(),
        })?;
    Ok(MixedSourceFaceKey {
        operand,
        topology_ordinal,
    })
}

const fn operand_side(operand: usize) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

fn validate_planar_lineage(
    store: &Store,
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPlanarFaceArrangement,
    lineage: &MixedPlanarSourceLineage,
    source: MixedSourceFaceKey,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::PlanarLineageMismatch(source);
    let raw_face = store.get(face.raw()).map_err(|_| fail())?;
    let [loop_id] = raw_face.loops() else {
        return Err(fail());
    };
    let loop_ = store.get(*loop_id).map_err(|_| fail())?;
    let mut expected_vertices = Vec::new();
    for fin_id in loop_.fins() {
        let fin = store.get(*fin_id).map_err(|_| fail())?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        let [Some(first), Some(second)] = edge.vertices() else {
            return Err(fail());
        };
        let pair = if fin.sense() == ktopo::entity::Sense::Forward {
            [first, second]
        } else {
            [second, first]
        };
        for vertex in pair {
            if !expected_vertices.contains(&vertex) {
                expected_vertices.push(vertex);
            }
        }
    }
    if lineage.source_vertices() != expected_vertices
        || lineage.spans().len() != arrangement.source_spans().len()
    {
        return Err(fail());
    }
    let mut seen = BTreeSet::new();
    for span in arrangement.source_spans() {
        let candidates = lineage
            .spans()
            .iter()
            .filter(|candidate| candidate.key() == span.key())
            .collect::<Vec<_>>();
        let [candidate] = candidates.as_slice() else {
            return Err(fail());
        };
        if !seen.insert(span.key().clone()) {
            return Err(fail());
        }
        let fin_id = *loop_
            .fins()
            .get(span.key().fin_loop_ordinal)
            .ok_or_else(fail)?;
        let fin = store.get(fin_id).map_err(|_| fail())?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        if candidate.loop_id() != *loop_id
            || candidate.fin() != fin_id
            || candidate.edge() != fin.edge()
        {
            return Err(fail());
        }
        for (vertex, evidence) in span.endpoints().into_iter().zip(candidate.range()) {
            match (vertex, evidence) {
                (
                    MixedArrangementVertex::SourceVertex(ordinal),
                    MixedSourceParameterEvidence::SourceVertex {
                        topology_ordinal,
                        vertex,
                        edge_parameter_bits,
                    },
                ) => {
                    let [Some(edge_start), Some(edge_end)] = edge.vertices() else {
                        return Err(fail());
                    };
                    let Some((lo, hi)) = edge.bounds() else {
                        return Err(fail());
                    };
                    let expected_parameter = if *vertex == edge_start {
                        lo
                    } else if *vertex == edge_end {
                        hi
                    } else {
                        return Err(fail());
                    };
                    if topology_ordinal != ordinal
                        || lineage.source_vertices().get(*ordinal) != Some(vertex)
                        || *edge_parameter_bits != expected_parameter.to_bits()
                    {
                        return Err(fail());
                    }
                }
                (
                    MixedArrangementVertex::SectionEndpoint(endpoint),
                    MixedSourceParameterEvidence::SectionRoot {
                        endpoint: claimed,
                        root_ordinal,
                        enclosure_bits,
                    },
                ) => {
                    let section = graph.curve_endpoints().get(*endpoint).ok_or_else(fail)?;
                    let SectionCurveEndpointTopology::Trim {
                        source_parameters, ..
                    } = section.topology()
                    else {
                        return Err(fail());
                    };
                    let parameter = source_parameters[operand].as_ref().ok_or_else(fail)?;
                    let enclosure = section.edge_parameters()[operand].ok_or_else(fail)?;
                    if claimed != endpoint
                        || parameter.edge().raw() != candidate.edge()
                        || parameter.root_ordinal() != *root_ordinal
                        || *enclosure_bits != [enclosure.lo().to_bits(), enclosure.hi().to_bits()]
                    {
                        return Err(fail());
                    }
                }
                _ => return Err(fail()),
            }
        }
    }
    Ok(())
}

fn bind_disk_source_spans(
    store: &Store,
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arranged: &ArrangedDiskFace,
    source: MixedSourceFaceKey,
) -> Result<
    (
        BTreeMap<DiskSourceArcKey, MixedSourceSpanKey>,
        Vec<MixedBoundedSourceSpanPlan>,
    ),
    MixedShellPlanError,
> {
    let fail = || MixedShellPlanError::DiskLineageMismatch(source);
    if operand != source.operand() {
        return Err(fail());
    }
    let mut source_spans = BTreeMap::new();
    let mut retained = Vec::with_capacity(arranged.source_arcs().len());
    for (traversal_ordinal, span) in arranged.arrangement().source_spans().iter().enumerate() {
        let candidates = arranged
            .source_arcs()
            .iter()
            .filter(|candidate| candidate.key() == *span.key())
            .collect::<Vec<_>>();
        let [lineage] = candidates.as_slice() else {
            return Err(fail());
        };
        if span.is_whole_loop()
            || span.endpoints().map(|endpoint| *endpoint) != lineage.key().endpoints()
        {
            return Err(fail());
        }
        let raw_fin = store.get(lineage.fin()).map_err(|_| fail())?;
        let raw_loop = store.get(raw_fin.parent()).map_err(|_| fail())?;
        if raw_loop.face() != face.raw()
            || raw_fin.edge() != lineage.edge()
            || raw_fin.sense() != lineage.key().sense()
        {
            return Err(fail());
        }

        let roots = lineage.roots();
        let period_shifts = lineage.period_shifts();
        let mut retained_roots = Vec::with_capacity(2);
        for (root, period_shift) in roots.into_iter().zip(period_shifts) {
            let root_key = root.key();
            let endpoint = graph
                .curve_endpoints()
                .get(root_key.endpoint())
                .ok_or_else(fail)?;
            let SectionCurveEndpointTopology::Trim {
                source_parameters, ..
            } = endpoint.topology()
            else {
                return Err(fail());
            };
            let parameter = source_parameters[operand].as_ref().ok_or_else(fail)?;
            let enclosure = parameter.root_parameter_enclosure();
            if parameter.edge().raw() != lineage.edge()
                || parameter.root_ordinal() != root_key.source_root_ordinal()
                || parameter.root_parameter().to_bits() != root.root_parameter().to_bits()
                || [enclosure.lo(), enclosure.hi()].map(f64::to_bits)
                    != root.root_enclosure().map(f64::to_bits)
            {
                return Err(fail());
            }
            retained_roots.push(MixedBoundedSourceRoot {
                endpoint: root_key.endpoint(),
                root_ordinal: root_key.source_root_ordinal(),
                parameter_bits: root.root_parameter().to_bits(),
                enclosure_bits: root.root_enclosure().map(f64::to_bits),
                period_shift,
            });
        }
        let roots: [MixedBoundedSourceRoot; 2] = retained_roots.try_into().map_err(|_| fail())?;
        if roots.map(MixedBoundedSourceRoot::endpoint) != lineage.key().endpoints() {
            return Err(fail());
        }
        let local = MixedSourceSpanKey {
            fin_loop_ordinal: 0,
            traversal_ordinal,
        };
        if source_spans.insert(*span.key(), local.clone()).is_some() {
            return Err(fail());
        }
        retained.push(MixedBoundedSourceSpanPlan {
            source,
            span: local,
            loop_id: raw_fin.parent(),
            fin: lineage.fin(),
            edge: lineage.edge(),
            roots,
        });
    }
    if source_spans.len() != arranged.source_arcs().len() {
        return Err(fail());
    }
    Ok((source_spans, retained))
}

fn bind_periodic_source_spans(
    store: &Store,
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPeriodicFaceArrangement,
    embedding: Option<&crate::CertifiedSectionPeriodicFaceEmbedding>,
    source: MixedSourceFaceKey,
) -> Result<
    (
        BTreeMap<PeriodicSourceLoopKey, MixedSourceSpanKey>,
        Vec<MixedBoundedSourceSpanPlan>,
    ),
    MixedShellPlanError,
> {
    let fail = || MixedShellPlanError::DiskLineageMismatch(source);
    if operand != source.operand() {
        return Err(fail());
    }
    let certified = embedding
        .filter(|value| value.operand() == operand && value.face() == *face)
        .or_else(|| {
            graph
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
        })
        .ok_or_else(fail)?;
    let mut source_spans = BTreeMap::new();
    let mut retained = Vec::new();
    for span in arrangement.source_spans() {
        let loop_key = *span.key();
        if loop_key.is_whole_loop() {
            if !span.is_whole_loop() {
                return Err(fail());
            }
            continue;
        }
        let roots = loop_key.terminal_roots().ok_or_else(fail)?;
        let span_ordinal = loop_key.cyclic_span_ordinal().ok_or_else(fail)?;
        let [
            PeriodicArrangementVertexKey::SectionEndpoint(start),
            PeriodicArrangementVertexKey::SectionEndpoint(end),
        ] = span.endpoints()
        else {
            return Err(fail());
        };
        if [*start, *end] != roots.map(|root| root.endpoint()) {
            return Err(fail());
        }
        let loop_id = certified
            .source_loops()
            .get(loop_key.topology_ordinal())
            .ok_or_else(fail)?
            .raw();
        let raw_loop = store.get(loop_id).map_err(|_| fail())?;
        let [fin_id] = raw_loop.fins() else {
            return Err(fail());
        };
        let raw_fin = store.get(*fin_id).map_err(|_| fail())?;
        if raw_loop.face() != face.raw() || raw_fin.parent() != loop_id {
            return Err(fail());
        }
        let edge = raw_fin.edge();
        let period_shifts = intrinsic_circle_period_shifts(
            raw_fin.sense(),
            roots.map(|root| root.root_parameter()),
        )
        .ok_or_else(fail)?;
        let mut retained_roots = Vec::with_capacity(2);
        for (root, period_shift) in roots.into_iter().zip(period_shifts) {
            let endpoint = graph
                .curve_endpoints()
                .get(root.endpoint())
                .ok_or_else(fail)?;
            let SectionCurveEndpointTopology::Trim {
                source_parameters, ..
            } = endpoint.topology()
            else {
                return Err(fail());
            };
            let parameter = source_parameters[operand].as_ref().ok_or_else(fail)?;
            let enclosure = parameter.root_parameter_enclosure();
            if parameter.edge().raw() != edge
                || parameter.root_ordinal() != root.source_root_ordinal()
                || parameter.root_parameter().to_bits() != root.root_parameter().to_bits()
                || [enclosure.lo(), enclosure.hi()].map(f64::to_bits)
                    != root.root_enclosure().map(f64::to_bits)
            {
                return Err(fail());
            }
            retained_roots.push(MixedBoundedSourceRoot {
                endpoint: root.endpoint(),
                root_ordinal: root.source_root_ordinal(),
                parameter_bits: root.root_parameter().to_bits(),
                enclosure_bits: root.root_enclosure().map(f64::to_bits),
                period_shift,
            });
        }
        let roots: [MixedBoundedSourceRoot; 2] = retained_roots.try_into().map_err(|_| fail())?;
        let local = MixedSourceSpanKey {
            fin_loop_ordinal: loop_key.topology_ordinal(),
            traversal_ordinal: span_ordinal,
        };
        if source_spans.insert(loop_key, local.clone()).is_some() {
            return Err(fail());
        }
        retained.push(MixedBoundedSourceSpanPlan {
            source,
            span: local,
            loop_id,
            fin: *fin_id,
            edge,
            roots,
        });
    }
    Ok((source_spans, retained))
}

fn intrinsic_circle_period_shifts(sense: Sense, parameters: [f64; 2]) -> Option<[i32; 2]> {
    if !parameters.into_iter().all(f64::is_finite) || parameters[0] == parameters[1] {
        return None;
    }
    Some(match sense {
        Sense::Forward if parameters[1] < parameters[0] => [0, 1],
        Sense::Reversed if parameters[0] < parameters[1] => [1, 0],
        _ => [0, 0],
    })
}

fn fragment_endpoints(fragment: &SectionCurveFragment) -> Option<[usize; 2]> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => None,
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            Some([endpoints[0].endpoint(), endpoints[1].endpoint()])
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            Some([endpoints[0].endpoint(), endpoints[1].endpoint()])
        }
        SectionCurveFragmentSpan::BoundedProcedural { endpoints } => Some(
            endpoints
                .each_ref()
                .map(|end| end.physical_root().endpoint()),
        ),
    }
}

fn direction_from_endpoint_order(
    arrangement: [usize; 2],
    section: [usize; 2],
) -> Option<ArrangementDirection> {
    if arrangement == section {
        Some(ArrangementDirection::Forward)
    } else if arrangement == [section[1], section[0]] {
        Some(ArrangementDirection::Reverse)
    } else {
        None
    }
}

fn planar_cut_lineage(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPlanarFaceArrangement,
    source: MixedSourceFaceKey,
) -> Result<BTreeMap<MixedCutFragmentKey, SectionUseLineage>, MixedShellPlanError> {
    let mut output = BTreeMap::new();
    for cut in arrangement.cut_fragments() {
        let [start, end] = cut.endpoints();
        let (
            MixedArrangementVertex::SectionEndpoint(start),
            MixedArrangementVertex::SectionEndpoint(end),
        ) = (start, end)
        else {
            return Err(MixedShellPlanError::PlanarCutEndpointIdentityUnavailable(
                source,
            ));
        };
        let arrangement_endpoints = [*start, *end];
        let mut found = None;
        for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
            let branch = graph.branches().get(fragment.branch()).ok_or(
                MixedShellPlanError::UnknownSectionBranch {
                    fragment: fragment_index,
                    branch: fragment.branch(),
                },
            )?;
            if branch.faces()[operand] != *face {
                continue;
            }
            let Some(section_endpoints) = fragment_endpoints(fragment) else {
                continue;
            };
            let Some(arrangement_to_section) =
                direction_from_endpoint_order(arrangement_endpoints, section_endpoints)
            else {
                continue;
            };
            if found
                .replace(SectionUseLineage {
                    fragment: fragment_index,
                    arrangement_to_section,
                    cylinder_period_shift: 0,
                })
                .is_some()
            {
                return Err(MixedShellPlanError::AmbiguousPlanarCutLineage(source));
            }
        }
        let lineage = found.ok_or(MixedShellPlanError::MissingPlanarCutLineage(source))?;
        output.insert(cut.key().clone(), lineage);
    }
    Ok(output)
}

fn disk_cut_lineage(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arranged: &ArrangedDiskFace,
    source: MixedSourceFaceKey,
) -> Result<BTreeMap<DiskChordKey, SectionUseLineage>, MixedShellPlanError> {
    let fail = || MixedShellPlanError::DiskLineageMismatch(source);
    let mut output = BTreeMap::new();
    for cut in arranged.arrangement().cut_fragments() {
        let key = *cut.key();
        let fragment = graph
            .curve_fragments()
            .get(key.fragment())
            .ok_or(MixedShellPlanError::UnknownSectionFragment(key.fragment()))?;
        let branch = graph.branches().get(fragment.branch()).ok_or(
            MixedShellPlanError::UnknownSectionBranch {
                fragment: key.fragment(),
                branch: fragment.branch(),
            },
        )?;
        if branch.faces().get(operand) != Some(face) {
            return Err(MixedShellPlanError::SectionFragmentLeavesFace {
                fragment: key.fragment(),
                source,
            });
        }
        let section_endpoints = fragment_endpoints(fragment).ok_or_else(fail)?;
        let arrangement_endpoints = cut.endpoints().map(|endpoint| *endpoint);
        let arrangement_to_section =
            direction_from_endpoint_order(arrangement_endpoints, section_endpoints)
                .ok_or_else(fail)?;
        if output
            .insert(
                key,
                SectionUseLineage {
                    fragment: key.fragment(),
                    arrangement_to_section,
                    cylinder_period_shift: 0,
                },
            )
            .is_some()
        {
            return Err(fail());
        }
    }
    Ok(output)
}

fn periodic_cut_lineage(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPeriodicFaceArrangement,
    embedding: Option<&crate::CertifiedSectionPeriodicFaceEmbedding>,
    source: MixedSourceFaceKey,
) -> Result<BTreeMap<PeriodicCutFragmentKey, SectionUseLineage>, MixedShellPlanError> {
    let certified = embedding
        .filter(|value| value.operand() == operand && value.face() == *face)
        .or_else(|| {
            graph
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
        });
    let Some(certified) = certified else {
        return Err(MixedShellPlanError::MissingPeriodicEmbedding {
            source,
            fragment: 0,
        });
    };
    let mut output = BTreeMap::new();
    for cut in arrangement.cut_fragments() {
        let key = *cut.key();
        match key.source_component() {
            Some(component_index) => {
                let component = graph
                    .curve_components()
                    .get(component_index)
                    .ok_or(MixedShellPlanError::PeriodicComponentMismatch(key))?;
                if component_index != key.component()
                    || component.fragments().get(key.ordinal()) != Some(&key.fragment())
                {
                    return Err(MixedShellPlanError::PeriodicComponentMismatch(key));
                }
            }
            None => {
                // A face-local mixed path can leave and later return to this
                // cylinder face, yielding several maximal traces under one
                // stable trace-group key. The path ordinal, not group
                // uniqueness, owns the exact occurrence.
                let mut occurrences = certified
                    .boundary_traces()
                    .iter()
                    .filter(|trace| {
                        trace.source_component().is_none() && trace.component() == key.component()
                    })
                    .flat_map(|trace| trace.component_ordinals().iter().zip(trace.fragments()))
                    .filter(|(ordinal, _)| **ordinal == key.ordinal());
                let (_, embedded) = occurrences
                    .next()
                    .filter(|_| occurrences.next().is_none())
                    .ok_or(MixedShellPlanError::PeriodicComponentMismatch(key))?;
                if embedded.fragment() != key.fragment()
                    || embedded.period_shift() != key.cylinder_period_shift()
                {
                    return Err(MixedShellPlanError::PeriodicComponentMismatch(key));
                }
            }
        }
        let fragment = graph
            .curve_fragments()
            .get(key.fragment())
            .ok_or(MixedShellPlanError::UnknownSectionFragment(key.fragment()))?;
        let branch = graph.branches().get(fragment.branch()).ok_or(
            MixedShellPlanError::UnknownSectionBranch {
                fragment: key.fragment(),
                branch: fragment.branch(),
            },
        )?;
        if branch.faces()[operand] != *face {
            return Err(MixedShellPlanError::SectionFragmentLeavesFace {
                fragment: key.fragment(),
                source,
            });
        }
        let [start, end] = cut.endpoints();
        let (
            PeriodicArrangementVertexKey::SectionEndpoint(start),
            PeriodicArrangementVertexKey::SectionEndpoint(end),
        ) = (start, end)
        else {
            return Err(MixedShellPlanError::PeriodicFragmentEndpointMismatch(key));
        };
        let Some(section_endpoints) = fragment_endpoints(fragment) else {
            return Err(MixedShellPlanError::PeriodicFragmentEndpointMismatch(key));
        };
        let arrangement_to_section =
            direction_from_endpoint_order([*start, *end], section_endpoints)
                .ok_or(MixedShellPlanError::PeriodicFragmentEndpointMismatch(key))?;
        let mut embeddings = certified
            .components()
            .iter()
            .flat_map(|component| component.fragments())
            .chain(
                certified
                    .boundary_traces()
                    .iter()
                    .flat_map(|trace| trace.fragments()),
            )
            .filter(|candidate| candidate.fragment() == key.fragment());
        let embedding = embeddings
            .next()
            .filter(|_| embeddings.next().is_none())
            .ok_or(MixedShellPlanError::MissingPeriodicEmbedding {
                source,
                fragment: key.fragment(),
            })?;
        if embedding.period_shift() != key.cylinder_period_shift() {
            return Err(MixedShellPlanError::MissingPeriodicEmbedding {
                source,
                fragment: key.fragment(),
            });
        }
        output.insert(
            key,
            SectionUseLineage {
                fragment: key.fragment(),
                arrangement_to_section,
                cylinder_period_shift: key.cylinder_period_shift(),
            },
        );
    }
    Ok(output)
}

fn opposite(direction: ArrangementDirection) -> ArrangementDirection {
    match direction {
        ArrangementDirection::Forward => ArrangementDirection::Reverse,
        ArrangementDirection::Reverse => ArrangementDirection::Forward,
    }
}

fn compose_direction(
    first: ArrangementDirection,
    second: ArrangementDirection,
) -> ArrangementDirection {
    if first == second {
        ArrangementDirection::Forward
    } else {
        ArrangementDirection::Reverse
    }
}

fn oriented_cycle<S: Clone, C: Clone, V: Clone>(
    cycle: &ArrangementCycle<S, C, V>,
    orientation: SelectedOrientation,
) -> OrientedCycleParts<S, C, V> {
    let mut uses = cycle
        .uses()
        .iter()
        .map(|use_| (use_.edge().clone(), use_.direction()))
        .collect::<Vec<_>>();
    let mut vertices = cycle.vertices().to_vec();
    if orientation == SelectedOrientation::Reversed {
        uses = uses
            .into_iter()
            .rev()
            .map(|(edge, direction)| (edge, opposite(direction)))
            .collect();
        if vertices.len() > 1 {
            let anchor = vertices[0].clone();
            let mut reversed = vec![anchor.clone()];
            reversed.extend(vertices[1..vertices.len() - 1].iter().rev().cloned());
            reversed.push(anchor);
            vertices = reversed;
        }
    }
    (uses, vertices)
}

fn planar_loop(
    graph: &BodySectionGraph,
    cell: MixedShellCellKey,
    source: MixedSourceFaceKey,
    operand: usize,
    cycle: &ArrangementCycle<MixedSourceSpanKey, MixedCutFragmentKey, MixedArrangementVertex>,
    lineage: &BTreeMap<MixedCutFragmentKey, SectionUseLineage>,
    orientation: SelectedOrientation,
) -> Result<MixedShellLoopPlan, MixedShellPlanError> {
    let (native_uses, native_vertices) = oriented_cycle(cycle, orientation);
    if native_vertices.len() != native_uses.len() + 1
        || native_vertices.first() != native_vertices.last()
    {
        return Err(MixedShellPlanError::MalformedArrangementCycle(cell));
    }
    let vertices = native_vertices
        .into_iter()
        .map(|vertex| match vertex {
            MixedArrangementVertex::SourceVertex(topology_ordinal) => {
                MixedShellVertexKey::PlanarSourceVertex {
                    source,
                    topology_ordinal,
                }
            }
            MixedArrangementVertex::SectionEndpoint(endpoint) => {
                MixedShellVertexKey::SectionEndpoint(endpoint)
            }
        })
        .collect();
    let mut uses = Vec::with_capacity(native_uses.len());
    for (edge, direction) in native_uses {
        uses.push(match edge {
            ArrangementEdgeKey::Source(span) => MixedShellEdgeUse {
                edge: MixedShellEdgeKey::PlanarSource { source, span },
                direction,
                pcurve: MixedPcurveLineage::SourceTopology,
            },
            ArrangementEdgeKey::Cut(cut) => {
                let section = lineage
                    .get(&cut)
                    .ok_or(MixedShellPlanError::MissingPlanarCutLineage(source))?;
                let fragment = graph.curve_fragments().get(section.fragment).ok_or(
                    MixedShellPlanError::UnknownSectionFragment(section.fragment),
                )?;
                MixedShellEdgeUse {
                    edge: MixedShellEdgeKey::SectionFragment(section.fragment),
                    direction: compose_direction(direction, section.arrangement_to_section),
                    pcurve: MixedPcurveLineage::Section {
                        branch: fragment.branch(),
                        operand,
                        cylinder_period_shift: 0,
                    },
                }
            }
        });
    }
    Ok(MixedShellLoopPlan { uses, vertices })
}

#[allow(clippy::too_many_arguments)]
fn disk_loop(
    graph: &BodySectionGraph,
    cell: MixedShellCellKey,
    source: MixedSourceFaceKey,
    operand: usize,
    face: &FaceId,
    cycle: &DiskArrangementCycle,
    source_spans: &BTreeMap<DiskSourceArcKey, MixedSourceSpanKey>,
    lineage: &BTreeMap<DiskChordKey, SectionUseLineage>,
    orientation: SelectedOrientation,
) -> Result<MixedShellLoopPlan, MixedShellPlanError> {
    let (native_uses, native_vertices) = oriented_cycle(cycle, orientation);
    if native_vertices.len() != native_uses.len() + 1
        || native_vertices.first() != native_vertices.last()
    {
        return Err(MixedShellPlanError::MalformedArrangementCycle(cell));
    }
    let vertices = native_vertices
        .into_iter()
        .map(MixedShellVertexKey::SectionEndpoint)
        .collect::<Vec<_>>();
    let mut uses = Vec::with_capacity(native_uses.len());
    for (edge, direction) in native_uses {
        uses.push(match edge {
            ArrangementEdgeKey::Source(arc) => {
                let span = source_spans
                    .get(&arc)
                    .ok_or(MixedShellPlanError::DiskLineageMismatch(source))?;
                MixedShellEdgeUse {
                    edge: MixedShellEdgeKey::PlanarSource {
                        source,
                        span: span.clone(),
                    },
                    direction,
                    pcurve: MixedPcurveLineage::SourceTopology,
                }
            }
            ArrangementEdgeKey::Cut(cut) => {
                let section = lineage
                    .get(&cut)
                    .ok_or(MixedShellPlanError::DiskLineageMismatch(source))?;
                let fragment = graph.curve_fragments().get(section.fragment).ok_or(
                    MixedShellPlanError::UnknownSectionFragment(section.fragment),
                )?;
                let branch = graph.branches().get(fragment.branch()).ok_or(
                    MixedShellPlanError::UnknownSectionBranch {
                        fragment: section.fragment,
                        branch: fragment.branch(),
                    },
                )?;
                if branch.faces().get(operand) != Some(face) {
                    return Err(MixedShellPlanError::SectionFragmentLeavesFace {
                        fragment: section.fragment,
                        source,
                    });
                }
                MixedShellEdgeUse {
                    edge: MixedShellEdgeKey::SectionFragment(section.fragment),
                    direction: compose_direction(direction, section.arrangement_to_section),
                    pcurve: MixedPcurveLineage::Section {
                        branch: fragment.branch(),
                        operand,
                        cylinder_period_shift: 0,
                    },
                }
            }
        });
    }
    Ok(MixedShellLoopPlan { uses, vertices })
}

#[allow(clippy::too_many_arguments)]
fn periodic_loop(
    graph: &BodySectionGraph,
    cell: MixedShellCellKey,
    source: MixedSourceFaceKey,
    operand: usize,
    cycle: &PeriodicArrangementCycle,
    bounded_spans: &BTreeMap<PeriodicSourceLoopKey, MixedSourceSpanKey>,
    lineage: &BTreeMap<PeriodicCutFragmentKey, SectionUseLineage>,
    orientation: SelectedOrientation,
) -> Result<MixedShellLoopPlan, MixedShellPlanError> {
    let (native_uses, native_vertices) = oriented_cycle(cycle, orientation);
    if native_vertices.len() != native_uses.len() + 1
        || native_vertices.first() != native_vertices.last()
    {
        return Err(MixedShellPlanError::MalformedArrangementCycle(cell));
    }
    let vertices = native_vertices
        .into_iter()
        .map(|vertex| match vertex {
            PeriodicArrangementVertexKey::SourceLoopSeam(loop_key) => {
                MixedShellVertexKey::ProofSeam { source, loop_key }
            }
            PeriodicArrangementVertexKey::SectionEndpoint(endpoint) => {
                MixedShellVertexKey::SectionEndpoint(endpoint)
            }
        })
        .collect::<Vec<_>>();
    let mut uses = Vec::with_capacity(native_uses.len());
    for (edge, direction) in native_uses {
        uses.push(match edge {
            ArrangementEdgeKey::Source(loop_key) if loop_key.is_whole_loop() => MixedShellEdgeUse {
                edge: MixedShellEdgeKey::PeriodicSource { source, loop_key },
                direction: compose_direction(direction, loop_key.source_direction()),
                pcurve: MixedPcurveLineage::SourceTopology,
            },
            ArrangementEdgeKey::Source(loop_key) => {
                let span = bounded_spans
                    .get(&loop_key)
                    .ok_or(MixedShellPlanError::DiskLineageMismatch(source))?;
                MixedShellEdgeUse {
                    edge: MixedShellEdgeKey::PlanarSource {
                        source,
                        span: span.clone(),
                    },
                    direction,
                    pcurve: MixedPcurveLineage::SourceTopology,
                }
            }
            ArrangementEdgeKey::Cut(cut) => {
                let section =
                    lineage
                        .get(&cut)
                        .ok_or(MixedShellPlanError::MissingPeriodicEmbedding {
                            source,
                            fragment: cut.fragment(),
                        })?;
                let fragment = graph.curve_fragments().get(section.fragment).ok_or(
                    MixedShellPlanError::UnknownSectionFragment(section.fragment),
                )?;
                MixedShellEdgeUse {
                    edge: MixedShellEdgeKey::SectionFragment(section.fragment),
                    direction: compose_direction(direction, section.arrangement_to_section),
                    pcurve: MixedPcurveLineage::Section {
                        branch: fragment.branch(),
                        operand,
                        cylinder_period_shift: section.cylinder_period_shift,
                    },
                }
            }
        });
    }
    if uses.iter().zip(&vertices).any(|(use_, vertex)| {
        matches!(use_.edge, MixedShellEdgeKey::SectionFragment(_))
            && matches!(vertex, MixedShellVertexKey::ProofSeam { .. })
    }) {
        return Err(MixedShellPlanError::PhysicalUseContainsProofSeam(cell));
    }
    Ok(MixedShellLoopPlan { uses, vertices })
}

fn validate_section_pairing(faces: &[MixedShellFacePlan]) -> Result<(), MixedShellPlanError> {
    let mut uses = BTreeMap::<usize, Vec<ArrangementDirection>>::new();
    for use_ in faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(MixedShellLoopPlan::uses)
    {
        if let MixedShellEdgeKey::SectionFragment(fragment) = use_.edge() {
            uses.entry(*fragment).or_default().push(use_.direction());
        }
    }
    for (fragment, directions) in uses {
        if directions.len() != 2 {
            return Err(MixedShellPlanError::SectionUseCount {
                fragment,
                actual: directions.len(),
            });
        }
        if directions[0] == directions[1] {
            return Err(MixedShellPlanError::SectionUseDirectionMismatch(fragment));
        }
    }
    Ok(())
}

fn bounded_source_span_is_used(
    faces: &[MixedShellFacePlan],
    span: &MixedBoundedSourceSpanPlan,
) -> bool {
    faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(MixedShellLoopPlan::uses)
        .any(|use_| {
            use_.edge()
                == &MixedShellEdgeKey::PlanarSource {
                    source: span.source,
                    span: span.span.clone(),
                }
        })
}

fn validate_bounded_source_pairing(
    store: &Store,
    faces: &[MixedShellFacePlan],
    spans: &[MixedBoundedSourceSpanPlan],
) -> Result<(), MixedShellPlanError> {
    struct PhysicalBoundedUse<'a> {
        span: &'a MixedBoundedSourceSpanPlan,
        endpoints: [usize; 2],
        direction: ArrangementDirection,
    }

    let mut uses = Vec::new();
    for use_ in faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(MixedShellLoopPlan::uses)
    {
        let MixedShellEdgeKey::PlanarSource { source, span } = use_.edge() else {
            continue;
        };
        let Some(retained) = spans
            .iter()
            .find(|candidate| candidate.source == *source && candidate.span == *span)
        else {
            continue;
        };
        let fin = store.get(retained.fin).map_err(|_| {
            MixedShellPlanError::BoundedSourceSpanDirectionMismatch {
                source: retained.source,
                span: retained.span.clone(),
            }
        })?;
        let sense = if fin.sense() == Sense::Forward {
            ArrangementDirection::Forward
        } else {
            ArrangementDirection::Reverse
        };
        let mut endpoints = retained.roots.map(MixedBoundedSourceRoot::endpoint);
        if fin.sense() == Sense::Reversed {
            endpoints.reverse();
        }
        uses.push(PhysicalBoundedUse {
            span: retained,
            endpoints,
            direction: compose_direction(use_.direction(), sense),
        });
    }

    for span in spans {
        let fin = store.get(span.fin).map_err(|_| {
            MixedShellPlanError::BoundedSourceSpanDirectionMismatch {
                source: span.source,
                span: span.span.clone(),
            }
        })?;
        let mut endpoints = span.roots.map(MixedBoundedSourceRoot::endpoint);
        if fin.sense() == Sense::Reversed {
            endpoints.reverse();
        }
        let matching = uses
            .iter()
            .filter(|use_| use_.span.edge == span.edge && use_.endpoints == endpoints)
            .collect::<Vec<_>>();
        if matching.len() != 2 {
            return Err(MixedShellPlanError::BoundedSourceSpanUseCount {
                source: span.source,
                span: span.span.clone(),
                actual: matching.len(),
            });
        }
        if matching[0].direction == matching[1].direction {
            return Err(MixedShellPlanError::BoundedSourceSpanDirectionMismatch {
                source: span.source,
                span: span.span.clone(),
            });
        }
    }
    Ok(())
}

fn validate_endpoint_free_ring_pairing(
    faces: &[MixedShellFacePlan],
) -> Result<(), MixedShellPlanError> {
    let mut uses =
        BTreeMap::<(MixedSourceFaceKey, PeriodicSourceLoopKey), Vec<ArrangementDirection>>::new();
    for use_ in faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(MixedShellLoopPlan::uses)
    {
        if let MixedShellEdgeKey::PeriodicSource { source, loop_key } = use_.edge() {
            uses.entry((*source, *loop_key))
                .or_default()
                .push(use_.direction());
        }
    }
    for ((source, loop_key), directions) in uses {
        if directions.len() != 2 {
            return Err(MixedShellPlanError::EndpointFreeRingUseCount {
                source,
                loop_key,
                actual: directions.len(),
            });
        }
        if directions[0] == directions[1] {
            return Err(MixedShellPlanError::EndpointFreeRingUseDirectionMismatch {
                source,
                loop_key,
            });
        }
    }
    Ok(())
}

fn validate_derived_ring_pairing(
    faces: &[MixedShellFacePlan],
    rings: &[MixedDerivedRingPlan],
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    let mut uses = BTreeMap::<usize, Vec<ArrangementDirection>>::new();
    for use_ in faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(MixedShellLoopPlan::uses)
    {
        if let MixedShellEdgeKey::DerivedRing(ring) = use_.edge() {
            if *ring >= rings.len() {
                return Err(fail());
            }
            uses.entry(*ring).or_default().push(use_.direction());
        }
    }
    if uses.len() != rings.len() {
        return Err(fail());
    }
    for directions in uses.into_values() {
        if directions.len() != 2 || directions[0] == directions[1] {
            return Err(fail());
        }
    }
    Ok(())
}

fn resolve_endpoint_free_cap_directions(
    faces: &mut [MixedShellFacePlan],
    rings: &[MixedCylinderCapRing],
) -> Result<(), MixedShellPlanError> {
    for ring in rings {
        let source = ring.side_source();
        let loop_key = ring.side_loop_key();
        let mut locations = Vec::new();
        let mut side_location = None;
        let mut projected_location = None;

        for (face_index, face) in faces.iter().enumerate() {
            for (loop_index, loop_) in face.loops.iter().enumerate() {
                for (use_index, use_) in loop_.uses.iter().enumerate() {
                    if use_.edge != (MixedShellEdgeKey::PeriodicSource { source, loop_key }) {
                        continue;
                    }
                    let location = (face_index, loop_index, use_index);
                    locations.push(location);
                    match &use_.pcurve {
                        MixedPcurveLineage::SourceTopology
                            if face.source == ring.side_source()
                                && face.source_face == *ring.side_face() =>
                        {
                            if side_location.replace(location).is_some() {
                                return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch {
                                    source,
                                    loop_key,
                                });
                            }
                        }
                        MixedPcurveLineage::SourceTopology
                            if face.source == ring.cap_source()
                                && face.source_face == *ring.cap_face() => {}
                        MixedPcurveLineage::ProjectedEndpointFreeSourceCircle(proof)
                            if proof.source() == source
                                && proof.source_face() == ring.side_face()
                                && proof.loop_key() == loop_key
                                && proof.edge() == ring.edge()
                                && proof.target() == face.source
                                && proof.target_face() == &face.source_face =>
                        {
                            if projected_location.replace(location).is_some() {
                                return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch {
                                    source,
                                    loop_key,
                                });
                            }
                        }
                        _ => {
                            return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch {
                                source,
                                loop_key,
                            });
                        }
                    }
                }
            }
        }

        if locations.len() != 2 {
            return Err(MixedShellPlanError::EndpointFreeRingUseCount {
                source,
                loop_key,
                actual: locations.len(),
            });
        }
        let authority = side_location
            .or(projected_location)
            .ok_or(MixedShellPlanError::EndpointFreeRingBindingMismatch { source, loop_key })?;
        let peer = locations
            .into_iter()
            .find(|location| *location != authority)
            .ok_or(MixedShellPlanError::EndpointFreeRingBindingMismatch { source, loop_key })?;
        let direction = faces[authority.0].loops[authority.1].uses[authority.2].direction;
        faces[peer.0].loops[peer.1].uses[peer.2].direction = opposite(direction);
    }
    Ok(())
}

fn collect_section_edges(
    store: &Store,
    graph: &BodySectionGraph,
    faces: &[MixedShellFacePlan],
) -> Result<Vec<MixedSectionEdgePlan>, MixedShellPlanError> {
    let fragment_indices = faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(MixedShellLoopPlan::uses)
        .filter_map(|use_| match use_.edge() {
            MixedShellEdgeKey::SectionFragment(fragment) => Some(*fragment),
            _ => None,
        })
        .collect::<BTreeSet<_>>();
    let mut output = Vec::with_capacity(fragment_indices.len());
    for fragment_index in fragment_indices {
        let fragment = graph
            .curve_fragments()
            .get(fragment_index)
            .ok_or(MixedShellPlanError::UnknownSectionFragment(fragment_index))?;
        let branch = graph.branches().get(fragment.branch()).ok_or(
            MixedShellPlanError::UnknownSectionBranch {
                fragment: fragment_index,
                branch: fragment.branch(),
            },
        )?;
        let endpoints = fragment_endpoints(fragment)
            .ok_or(MixedShellPlanError::UnknownSectionFragment(fragment_index))?;
        let carrier_faces = [
            source_face_key(store, graph, &branch.faces()[0], 0)?,
            source_face_key(store, graph, &branch.faces()[1], 1)?,
        ];
        let skew_persistence = if matches!(
            fragment.span(),
            SectionCurveFragmentSpan::BoundedProcedural { .. }
        ) {
            Some(
                bounded_skew_persistence_input(store, branch, fragment).ok_or(
                    MixedShellPlanError::InvalidSkewPersistence {
                        fragment: fragment_index,
                    },
                )?,
            )
        } else {
            None
        };
        output.push(MixedSectionEdgePlan {
            fragment_index,
            fragment: fragment.clone(),
            branch: branch.clone(),
            endpoints,
            carrier_faces,
            skew_persistence,
        });
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use kcore::{
        operation::{OperationContext, OperationScope},
        tolerance::Tolerances,
    };

    use super::super::boundary_select::{
        BoundaryFragmentClassification, ClassifiedBoundaryFragment, RegularizedBooleanOperation,
        select_boundary_fragments,
    };
    use super::super::curved_source::{CylinderSourceOutcome, extract_cylinder_source};
    use super::super::mixed_face_arrangement::arrange_mixed_planar_face_with_lineage;
    use super::super::mixed_periodic_arrangement::{
        arrange_mixed_periodic_face, arrange_mixed_periodic_face_from_embedding,
    };
    use super::super::parallel_cylinder_relation::{
        ParallelCylinderRelationOutcome, certify_parallel_cylinder_relation,
    };
    use super::*;
    use crate::{BlockRequest, CylinderRequest, Kernel, SectionBodiesRequest};
    use kgeom::frame::Frame;

    type PlanarArrangementSet = Vec<(
        FaceId,
        super::super::mixed_face_arrangement::MixedPlanarFaceOutput,
    )>;
    type SelectedMixedCells = Vec<SelectedBoundaryFragment<MixedShellCellKey, ()>>;

    fn store_shape(store: &Store) -> [usize; 5] {
        [
            store.count::<ktopo::entity::Face>(),
            store.count::<ktopo::entity::Loop>(),
            store.count::<ktopo::entity::Fin>(),
            store.count::<ktopo::entity::Edge>(),
            store.count::<ktopo::entity::Vertex>(),
        ]
    }

    #[test]
    fn bounded_circle_period_lifts_follow_physical_fin_traversal() {
        assert_eq!(
            intrinsic_circle_period_shifts(Sense::Forward, [0.25, 1.75]),
            Some([0, 0])
        );
        assert_eq!(
            intrinsic_circle_period_shifts(Sense::Forward, [5.75, 0.25]),
            Some([0, 1])
        );
        assert_eq!(
            intrinsic_circle_period_shifts(Sense::Reversed, [5.75, 0.25]),
            Some([0, 0])
        );
        assert_eq!(
            intrinsic_circle_period_shifts(Sense::Reversed, [0.25, 5.75]),
            Some([1, 0])
        );
        assert_eq!(
            intrinsic_circle_period_shifts(Sense::Forward, [1.0, 1.0]),
            None
        );
        assert_eq!(
            intrinsic_circle_period_shifts(Sense::Forward, [f64::NAN, 1.0]),
            None
        );
    }

    #[test]
    fn operation_local_path_ordinals_bind_lineage_without_global_promotion() {
        let frame = Frame::world();
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (first, second) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let first = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(-0.5, 0.0, -1.0)),
                    1.0,
                    3.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let second = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(0.5, 0.0, -1.0)),
                    1.0,
                    2.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (first, second)
        };
        let part = session.part(part_id.clone()).unwrap();
        let graph = part
            .section_bodies(SectionBodiesRequest::new(first.clone(), second.clone()))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(graph.periodic_face_embeddings().iter().all(|evidence| {
            matches!(
                evidence,
                SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
                    gap: crate::SectionPeriodicEmbeddingGap::UnstitchedFragmentPath { .. },
                    ..
                }
            )
        }));
        let tolerances = Tolerances::default();
        let context = OperationContext::new(part.policy(), tolerances)
            .unwrap()
            .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let mut extract = |body: &crate::BodyId| match extract_cylinder_source(
            &part.state.store,
            body.raw(),
            &mut scope,
        )
        .unwrap()
        {
            CylinderSourceOutcome::Ready(source) => source,
            other => panic!("fixture lost certified cylinder source: {other:?}"),
        };
        let sources = [extract(&first), extract(&second)];
        let relation = match certify_parallel_cylinder_relation(
            &part.state.store,
            &graph,
            [&sources[0], &sources[1]],
            &mut scope,
        )
        .unwrap()
        {
            ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(relation) => relation,
            other => panic!("fixture lost certified coincident-cap relation: {other:?}"),
        };
        let mut saw_face_local_trace = false;
        for (operand, operand_source) in sources.iter().enumerate() {
            let face = FaceId::new(part_id.clone(), operand_source.side_face());
            let evidence = crate::section::certify_periodic_face_fragment_subset(
                &part.state.store,
                face.clone().part(),
                &graph,
                operand,
                face,
                &relation.periodic_fragment_subset(operand),
                tolerances.linear(),
            )
            .unwrap();
            let mut occurrences = BTreeSet::new();
            for trace in evidence.boundary_traces() {
                assert_eq!(trace.source_component(), None);
                assert_eq!(trace.component_ordinals().len(), trace.fragments().len());
                for (&ordinal, fragment) in trace.component_ordinals().iter().zip(trace.fragments())
                {
                    assert!(occurrences.insert((trace.component(), ordinal, fragment.fragment(),)));
                }
            }
            saw_face_local_trace |= !occurrences.is_empty();
            let arrangement =
                arrange_mixed_periodic_face_from_embedding(&graph, &evidence).unwrap();
            assert_eq!(occurrences.len(), arrangement.cut_fragments().len());
            let source = source_face_key(
                &part.state.store,
                &graph,
                &evidence.face(),
                evidence.operand(),
            )
            .unwrap();
            let lineage = periodic_cut_lineage(
                &graph,
                &evidence.face(),
                evidence.operand(),
                &arrangement,
                Some(&evidence),
                source,
            )
            .unwrap();
            assert_eq!(lineage.len(), arrangement.cut_fragments().len());
            for cut in arrangement.cut_fragments() {
                let retained = lineage.get(cut.key()).unwrap();
                assert_eq!(retained.fragment, cut.key().fragment());
                assert_eq!(
                    retained.cylinder_period_shift,
                    cut.key().cylinder_period_shift()
                );
            }
        }
        assert!(saw_face_local_trace);
    }

    fn with_fixture(
        frame: Frame,
        test: impl FnOnce(&mut Store, &BodySectionGraph, usize, FaceId, MixedPeriodicFaceArrangement),
    ) {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    frame.with_origin(frame.point_at(0.0, 0.0, 1.0)),
                    [2.0, 5.0, 1.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(frame, 1.5, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let graph = session
            .part(part_id.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(block, cylinder))
            .unwrap()
            .into_result()
            .unwrap();
        let (periodic_operand, periodic_face) = graph
            .periodic_face_embeddings()
            .iter()
            .find_map(|evidence| match evidence {
                SectionPeriodicFaceEmbeddingEvidence::Certified(value) => {
                    Some((value.operand(), value.face()))
                }
                _ => None,
            })
            .unwrap();
        let periodic =
            arrange_mixed_periodic_face(&graph, periodic_face.clone(), periodic_operand).unwrap();
        let mut edit = session.edit_part(part_id).unwrap();
        test(
            edit.store_mut_for_test(),
            &graph,
            periodic_operand,
            periodic_face,
            periodic,
        );
    }

    fn selected_patch(
        store: &Store,
        graph: &BodySectionGraph,
        periodic_operand: usize,
        periodic_face: &FaceId,
        periodic: &MixedPeriodicFaceArrangement,
    ) -> (PlanarArrangementSet, SelectedMixedCells) {
        let periodic_source =
            source_face_key(store, graph, periodic_face, periodic_operand).unwrap();
        let periodic_cells = periodic
            .cells()
            .iter()
            .filter(|cell| matches!(cell.key(), PeriodicArrangementCellKey::ComponentDisk(_)))
            .collect::<Vec<_>>();
        assert!(!periodic_cells.is_empty());
        let periodic_lineage = periodic_cut_lineage(
            graph,
            periodic_face,
            periodic_operand,
            periodic,
            None,
            periodic_source,
        )
        .unwrap();
        let target_uses = periodic_cells
            .iter()
            .flat_map(|cell| cell.boundaries())
            .flat_map(ArrangementCycle::uses)
            .filter_map(|use_| match use_.edge() {
                ArrangementEdgeKey::Cut(key) => {
                    let lineage = periodic_lineage.get(key).unwrap();
                    Some((
                        lineage.fragment,
                        compose_direction(use_.direction(), lineage.arrangement_to_section),
                    ))
                }
                ArrangementEdgeKey::Source(_) => None,
            })
            .collect::<Vec<_>>();
        assert!(!target_uses.is_empty());

        let planar_operand = 1 - periodic_operand;
        let mut planar_faces = Vec::<FaceId>::new();
        for (fragment, _) in &target_uses {
            let branch = &graph.branches()[graph.curve_fragments()[*fragment].branch()];
            let face = branch.faces()[planar_operand].clone();
            if !planar_faces.contains(&face) {
                planar_faces.push(face);
            }
        }
        let arrangements = planar_faces
            .into_iter()
            .map(|face| {
                let output = arrange_mixed_planar_face_with_lineage(
                    store,
                    graph,
                    face.clone(),
                    planar_operand,
                )
                .unwrap();
                (face, output)
            })
            .collect::<Vec<_>>();

        let mut selected_keys = periodic_cells
            .iter()
            .map(|cell| MixedShellCellKey::periodic(periodic_source, *cell.key()))
            .collect::<BTreeSet<_>>();
        for (fragment, periodic_direction) in target_uses {
            let mut matched = None;
            for (face, output) in &arrangements {
                let arrangement = output.arrangement();
                let source = source_face_key(store, graph, face, planar_operand).unwrap();
                let lineage =
                    planar_cut_lineage(graph, face, planar_operand, arrangement, source).unwrap();
                for cell in arrangement.cells() {
                    for use_ in cell
                        .boundaries()
                        .iter()
                        .flat_map(|boundary| boundary.uses())
                    {
                        let ArrangementEdgeKey::Cut(key) = use_.edge() else {
                            continue;
                        };
                        let Some(candidate) = lineage.get(key) else {
                            continue;
                        };
                        let direction =
                            compose_direction(use_.direction(), candidate.arrangement_to_section);
                        if candidate.fragment == fragment && direction != periodic_direction {
                            let key = MixedShellCellKey::planar(source, cell.key());
                            assert!(matched.replace(key).is_none());
                        }
                    }
                }
            }
            selected_keys.insert(matched.expect("opposed planar cell use"));
        }

        let classified = selected_keys.into_iter().map(|key| {
            ClassifiedBoundaryFragment::new(
                key,
                operand_side(key.source().operand()),
                (),
                BoundaryFragmentClassification::Exterior,
            )
        });
        let selected =
            select_boundary_fragments(RegularizedBooleanOperation::Unite, classified).unwrap();
        (arrangements, selected)
    }

    #[test]
    fn certified_block_cylinder_patch_preserves_shared_identity_and_chart_lifts() {
        let oblique = Frame::new(
            kgeom::vec::Point3::new(3.0, -2.0, 1.25),
            kgeom::vec::Vec3::new(0.48, 0.64, 0.6),
            kgeom::vec::Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap();
        for frame in [Frame::world(), oblique] {
            with_fixture(
                frame,
                |store, graph, periodic_operand, periodic_face, periodic| {
                    let (planar, selected) =
                        selected_patch(store, graph, periodic_operand, &periodic_face, &periodic);
                    let bindings =
                        std::iter::once(MixedArrangementBinding::Periodic {
                            face: periodic_face,
                            operand: periodic_operand,
                            arrangement: &periodic,
                            embedding: None,
                        })
                        .chain(planar.iter().map(|(face, output)| {
                            MixedArrangementBinding::Planar {
                                face: face.clone(),
                                operand: 1 - periodic_operand,
                                arrangement: output.arrangement(),
                                lineage: output.lineage(),
                            }
                        }));
                    let plan = plan_mixed_shell(store, graph, bindings, selected).unwrap();
                    for edge in plan.section_edges() {
                        let uses = plan
                            .faces()
                            .iter()
                            .flat_map(MixedShellFacePlan::loops)
                            .flat_map(MixedShellLoopPlan::uses)
                            .filter(|use_| {
                                use_.edge()
                                    == &MixedShellEdgeKey::SectionFragment(edge.fragment_index())
                            })
                            .collect::<Vec<_>>();
                        assert_eq!(uses.len(), 2);
                        assert_ne!(uses[0].direction(), uses[1].direction());
                    }
                    assert!(plan.materialization_gaps().is_empty());
                    let blueprint =
                        materialize::prepare_mixed_shell_materialization(&plan, store).unwrap();
                    assert!(blueprint.all_edges_have_two_opposed_uses());
                    assert_eq!(
                        blueprint.planar_use_count(),
                        blueprint.planar_edge_count() * 2
                    );
                    let before = store_shape(store);
                    let input = materialize::materialize_mixed_shell_input(
                        &plan,
                        store,
                        &materialize::MixedShellScalarInputs::empty(),
                        1.0e-9,
                    )
                    .unwrap();
                    assert_eq!(store_shape(store), before);

                    let mut transaction = store.transaction().unwrap();
                    let output = transaction.assemble_analytic_shell(&input, 1.0e-9).unwrap();
                    let faults =
                        ktopo::check::check_body(transaction.store(), output.body()).unwrap();
                    assert!(faults.is_empty(), "{faults:#?}");
                    let full = ktopo::check::check_body_report(
                        transaction.store(),
                        output.body(),
                        ktopo::check::CheckLevel::Full,
                    )
                    .unwrap();
                    assert_eq!(
                        full.outcome(),
                        ktopo::check::CheckOutcome::Valid,
                        "{full:#?}"
                    );
                    transaction.rollback().unwrap();
                    assert_eq!(store_shape(store), before);
                },
            );
        }
    }

    #[test]
    fn binding_and_selection_order_do_not_change_the_plan() {
        with_fixture(
            Frame::world(),
            |store, graph, periodic_operand, periodic_face, periodic| {
                let (planar, selected) =
                    selected_patch(store, graph, periodic_operand, &periodic_face, &periodic);
                let make_bindings = || {
                    let mut bindings = planar
                        .iter()
                        .map(|(face, output)| MixedArrangementBinding::Planar {
                            face: face.clone(),
                            operand: 1 - periodic_operand,
                            arrangement: output.arrangement(),
                            lineage: output.lineage(),
                        })
                        .collect::<Vec<_>>();
                    bindings.push(MixedArrangementBinding::Periodic {
                        face: periodic_face.clone(),
                        operand: periodic_operand,
                        arrangement: &periodic,
                        embedding: None,
                    });
                    bindings
                };
                let expected =
                    plan_mixed_shell(store, graph, make_bindings(), selected.clone()).unwrap();
                let mut bindings = make_bindings();
                bindings.reverse();
                let mut reversed_selected = selected;
                reversed_selected.reverse();
                let actual = plan_mixed_shell(store, graph, bindings, reversed_selected).unwrap();
                assert_eq!(actual, expected);
                assert_eq!(
                    materialize::prepare_mixed_shell_materialization(&actual, store).unwrap(),
                    materialize::prepare_mixed_shell_materialization(&expected, store).unwrap()
                );
                assert_eq!(
                    materialize::materialize_mixed_shell_input(
                        &actual,
                        store,
                        &materialize::MixedShellScalarInputs::empty(),
                        1.0e-9,
                    )
                    .unwrap(),
                    materialize::materialize_mixed_shell_input(
                        &expected,
                        store,
                        &materialize::MixedShellScalarInputs::empty(),
                        1.0e-9,
                    )
                    .unwrap()
                );
            },
        );
    }

    #[test]
    fn missing_peer_and_forged_cell_fail_closed_without_metric_matching() {
        with_fixture(
            Frame::world(),
            |store, graph, periodic_operand, periodic_face, periodic| {
                let (planar, mut selected) =
                    selected_patch(store, graph, periodic_operand, &periodic_face, &periodic);
                let target = planar
                    .iter()
                    .position(|(_, output)| {
                        output.lineage().spans().iter().any(|span| {
                            span.range().iter().any(|value| {
                                matches!(value, MixedSourceParameterEvidence::SectionRoot { .. })
                            })
                        })
                    })
                    .unwrap();
                let mut forged_root = planar[target].1.lineage().clone();
                let root = forged_root
                    .spans
                    .iter_mut()
                    .flat_map(|span| &mut span.range)
                    .find_map(|value| match value {
                        MixedSourceParameterEvidence::SectionRoot { enclosure_bits, .. } => {
                            Some(enclosure_bits)
                        }
                        _ => None,
                    })
                    .unwrap();
                root[0] ^= 1;
                let mut forged_vertex = planar[target].1.lineage().clone();
                forged_vertex.source_vertices.swap(0, 1);
                for forged in [forged_root, forged_vertex] {
                    let bindings = std::iter::once(MixedArrangementBinding::Periodic {
                        face: periodic_face.clone(),
                        operand: periodic_operand,
                        arrangement: &periodic,
                        embedding: None,
                    })
                    .chain(planar.iter().enumerate().map(
                        |(index, (face, output))| MixedArrangementBinding::Planar {
                            face: face.clone(),
                            operand: 1 - periodic_operand,
                            arrangement: output.arrangement(),
                            lineage: if index == target {
                                &forged
                            } else {
                                output.lineage()
                            },
                        },
                    ));
                    assert!(matches!(
                        plan_mixed_shell(store, graph, bindings, selected.clone()),
                        Err(MixedShellPlanError::PlanarLineageMismatch(_))
                    ));
                }
                selected.pop();
                let bindings = std::iter::once(MixedArrangementBinding::Periodic {
                    face: periodic_face.clone(),
                    operand: periodic_operand,
                    arrangement: &periodic,
                    embedding: None,
                })
                .chain(planar.iter().map(|(face, output)| {
                    MixedArrangementBinding::Planar {
                        face: face.clone(),
                        operand: 1 - periodic_operand,
                        arrangement: output.arrangement(),
                        lineage: output.lineage(),
                    }
                }));
                assert!(matches!(
                    plan_mixed_shell(store, graph, bindings, selected),
                    Err(MixedShellPlanError::SectionUseCount { actual: 1, .. })
                ));

                let periodic_source =
                    source_face_key(store, graph, &periodic_face, periodic_operand).unwrap();
                let forged = ClassifiedBoundaryFragment::new(
                    MixedShellCellKey::periodic(
                        periodic_source,
                        PeriodicArrangementCellKey::ComponentDisk(usize::MAX),
                    ),
                    operand_side(periodic_operand),
                    (),
                    BoundaryFragmentClassification::Exterior,
                );
                let forged =
                    select_boundary_fragments(RegularizedBooleanOperation::Unite, [forged])
                        .unwrap();
                assert!(matches!(
                    plan_mixed_shell(
                        store,
                        graph,
                        [MixedArrangementBinding::Periodic {
                            face: periodic_face,
                            operand: periodic_operand,
                            arrangement: &periodic,
                            embedding: None,
                        }],
                        forged,
                    ),
                    Err(MixedShellPlanError::MissingPeriodicCell(_))
                ));
            },
        );
    }
}
