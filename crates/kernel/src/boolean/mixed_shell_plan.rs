//! Proof-bearing adoption of mixed planar/periodic face arrangements.
//!
//! Section owns exact endpoint identity, analytic carriers, paired pcurves,
//! deterministic carrier-evaluated trim scalars, and the integer lift of every
//! cylinder-side use. Planar lineage retains exact source topology and
//! isolated-root scalar evidence. [`MixedShellProofPlan`] retains that
//! certified proposal; its materializer coalesces raw source spans by equality,
//! preserves shared endpoint identity, and runs read-only analytic-shell
//! preflight before any transaction is opened.

use std::collections::{BTreeMap, BTreeSet};

use ktopo::entity::{EdgeId as RawEdgeId, FinId as RawFinId, LoopId as RawLoopId, Sense};
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
    ProjectedSourceCircleOnPlane, ProjectedSourceCircleOnPlaneError,
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

/// Topology-order identity of one source face within its operand body.
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

/// Arrangement-local cell identity, qualified by its source face.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellCellKind {
    Planar(usize),
    Disk(usize),
    Periodic(PeriodicArrangementCellKey),
    CylinderCap(usize),
}

/// Canonical identity consumed from representation-independent truth selection.
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

/// One already-certified arrangement made available to the bridge.
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

/// Exact physical vertex identity retained by the proof plan.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellVertexKey {
    /// A Section-owned trim endpoint shared across all incident source faces.
    SectionEndpoint(usize),
    /// A planar arrangement vertex whose raw source vertex is not exposed yet.
    PlanarSourceVertex {
        source: MixedSourceFaceKey,
        topology_ordinal: usize,
    },
    /// A combinatorial seam for an endpoint-free source ring; never physical.
    ProofSeam {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
    },
}

/// Exact identity of one physical edge proposal.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellEdgeKey {
    /// Planar source span backed by retained fin/root lineage evidence.
    PlanarSource {
        source: MixedSourceFaceKey,
        span: MixedSourceSpanKey,
    },
    /// Topology-owned complete source loop on the periodic face.
    PeriodicSource {
        source: MixedSourceFaceKey,
        loop_key: PeriodicSourceLoopKey,
    },
    /// Canonical index into `BodySectionGraph::curve_fragments`.
    SectionFragment(usize),
}

/// Pcurve/chart authority retained for one directed face use.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedPcurveLineage {
    /// Source topology remains the authority for its existing fin pcurve.
    SourceTopology,
    /// A retained cylinder source circle projected into a coincident Plane.
    ProjectedSourceCircleOnPlane(ProjectedSourceCircleOnPlane),
    /// Section branch pcurve plus an exact integer cylinder-period lift.
    Section {
        branch: usize,
        operand: usize,
        cylinder_period_shift: i64,
    },
}

/// One oriented use in a selected derived face loop.
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

/// One selected, oriented, closed face-boundary component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellLoopPlan {
    uses: Vec<MixedShellEdgeUse>,
    vertices: Vec<MixedShellVertexKey>,
}

impl MixedShellLoopPlan {
    pub(crate) fn uses(&self) -> &[MixedShellEdgeUse] {
        &self.uses
    }

    /// Traversed vertices, including the repeated closing identity.
    pub(crate) fn vertices(&self) -> &[MixedShellVertexKey] {
        &self.vertices
    }
}

/// One truth-selected derived face, still qualified by its source topology.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellFacePlan {
    source: MixedSourceFaceKey,
    source_face: FaceId,
    selected_orientation: SelectedOrientation,
    loops: Vec<MixedShellLoopPlan>,
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
}

/// Section payload retained once for every shared physical cut edge.
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

    /// Sealed graph/Section handoff for a bounded procedural skew span.
    pub(crate) const fn skew_persistence(&self) -> Option<SectionSkewCylinderPersistenceInput> {
        self.skew_persistence
    }
}

/// Exact Section-root endpoint retained for a bounded source-edge span.
///
/// The canonical scalar is realization evidence, while `endpoint`, `edge`,
/// and `root_ordinal` remain the identity authority. `period_shift` selects
/// the occurrence bounding this directed span without inventing another
/// root on a periodic carrier.
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

/// Raw source-topology and scalar lineage for one finite source span.
///
/// `source` and `span` identify its face-local arrangement use. Physical
/// coalescing across cap and periodic faces is instead keyed by equality of
/// `edge` plus the two Section endpoints, so independently arranged uses of
/// the same circle arc remain one edge proposal.
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

/// Exact evidence still required before analytic-shell materialization.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MixedShellMaterializationGap {
    /// Section must publish one exact intrinsic source-edge root parameter.
    ExactSourceRootParameterRequired {
        source: MixedSourceFaceKey,
        span: MixedSourceSpanKey,
        endpoint: usize,
    },
    /// Section must publish a proven exact carrier scalar, not a representative.
    ExactTrimParameterRequired { fragment: usize, endpoint: usize },
}

/// Exact intermediate produced for the admitted block/cylinder arrangement.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MixedShellProofPlan {
    faces: Vec<MixedShellFacePlan>,
    section_edges: Vec<MixedSectionEdgePlan>,
    bounded_source_spans: Vec<MixedBoundedSourceSpanPlan>,
    cap_rings: Vec<MixedCylinderCapRing>,
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

/// Typed refusal while building the exact intermediate.
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

/// Translate certified arrangements and truth-selected cells into one exact
/// shared-use proof plan.  The selector, not this bridge, owns side decisions.
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
        |_, _| Ok(()),
    )
}

fn plan_mixed_shell_with_augmentation<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    admission: SectionPlanningAdmission<'_>,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = (MixedShellCellKey, OperandSide, SelectedOrientation)>,
    augment: impl FnOnce(
        &mut Vec<MixedShellFacePlan>,
        &mut Vec<MixedBoundedSourceSpanPlan>,
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
                }
            }
            _ => return Err(MixedShellPlanError::ArrangementKindMismatch(key)),
        };
        faces.push(face);
    }

    augment(&mut faces, &mut bounded_source_spans)?;
    resolve_endpoint_free_cap_directions(&mut faces, &cap_rings)?;
    validate_section_pairing(&faces)?;
    validate_endpoint_free_ring_pairing(&faces)?;
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

fn resolve_endpoint_free_cap_directions(
    faces: &mut [MixedShellFacePlan],
    rings: &[MixedCylinderCapRing],
) -> Result<(), MixedShellPlanError> {
    for ring in rings {
        let source = ring.side_source();
        let loop_key = ring.side_loop_key();
        let mut side_direction = None;
        let mut cap_location = None;
        let mut actual = 0_usize;

        for (face_index, face) in faces.iter().enumerate() {
            for (loop_index, loop_) in face.loops.iter().enumerate() {
                for (use_index, use_) in loop_.uses.iter().enumerate() {
                    if use_.edge != (MixedShellEdgeKey::PeriodicSource { source, loop_key }) {
                        continue;
                    }
                    actual = actual.saturating_add(1);
                    if face.source == ring.side_source() {
                        if side_direction.replace(use_.direction).is_some() {
                            return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch {
                                source,
                                loop_key,
                            });
                        }
                    } else if face.source == ring.cap_source()
                        && face.source_face == *ring.cap_face()
                    {
                        if cap_location
                            .replace((face_index, loop_index, use_index))
                            .is_some()
                        {
                            return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch {
                                source,
                                loop_key,
                            });
                        }
                    } else {
                        return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch {
                            source,
                            loop_key,
                        });
                    }
                }
            }
        }

        if actual != 2 {
            return Err(MixedShellPlanError::EndpointFreeRingUseCount {
                source,
                loop_key,
                actual,
            });
        }
        let (Some(side_direction), Some((face_index, loop_index, use_index))) =
            (side_direction, cap_location)
        else {
            return Err(MixedShellPlanError::EndpointFreeRingBindingMismatch { source, loop_key });
        };
        faces[face_index].loops[loop_index].uses[use_index].direction = opposite(side_direction);
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
        for operand in 0..2 {
            let face = FaceId::new(part_id.clone(), sources[operand].side_face());
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
