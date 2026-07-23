//! Exact-scalar completion and read-only analytic-shell materialization.

use std::collections::{BTreeMap, BTreeSet};

use kcore::interval::Interval;
use kgeom::curve::{Circle, Curve, Line};
#[cfg(test)]
use kgeom::curve2d::Curve2d;
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::{Point2, Point3, Vec2};
use kgraph::AffineParamMap1d;
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellPcurve, AnalyticShellPlanError, AnalyticShellSurface,
    AnalyticShellVertex, AnalyticVertexKey, prepare_analytic_shell,
};
use ktopo::entity::{
    EdgeId as RawEdgeId, EntityRef, FinId as RawFinId, LoopId as RawLoopId, PcurveChart, Sense,
    VertexId as RawVertexId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use super::super::mixed_cap_boundary::MixedCylinderCapRing;
use super::super::mixed_periodic_arrangement::PeriodicSourceLoopKey;
use super::super::periodic_chart::{self, PeriodicChartError};
use super::components::MixedShellComponent;
use super::{
    ArrangementDirection, MixedArrangementBinding, MixedBoundedSourceSpanPlan, MixedPcurveLineage,
    MixedSectionEdgePlan, MixedShellEdgeKey, MixedShellFacePlan, MixedShellMaterializationGap,
    MixedShellProofPlan, MixedShellVertexKey, MixedSourceFaceKey, MixedSourceParameterEvidence,
    MixedSourceSpanKey, ProjectedSourceCircleOnPlane, ProjectedSourceCircleOnPlaneError,
    SelectedOrientation,
};
use crate::{
    BodySectionGraph, SectionCarrier, SectionCurveEndpointTopology, SectionCurveFragmentSpan,
    SectionPeriodicFaceEmbeddingEvidence, SectionUvCurve,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum RetainedSpanParameter {
    SourceVertex {
        topology_ordinal: usize,
        vertex: RawVertexId,
        edge_parameter_bits: u64,
    },
    SectionRoot {
        endpoint: usize,
        enclosure_bits: [u64; 2],
        parameter_bits: u64,
        /// Certified integer lift in the source carrier's canonical period.
        ///
        /// Root identity and its scalar remain canonical.  The lift only
        /// selects which occurrence bounds this directed source span, so the
        /// complementary arc of a split whole circle can retain an increasing
        /// finite carrier range without inventing a second root.
        period_shift: i32,
    },
}

/// Non-ordering raw topology payload retained for one selected planar span.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct RetainedPlanarSpan {
    pub(super) source: MixedSourceFaceKey,
    pub(super) span: MixedSourceSpanKey,
    pub(super) loop_id: RawLoopId,
    pub(super) fin: RawFinId,
    pub(super) edge: RawEdgeId,
    pub(super) range: [RetainedSpanParameter; 2],
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct RetainedSectionTrim {
    fragment: usize,
    endpoints: [usize; 2],
    certified: Option<[(f64, Point3); 2]>,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RetainedMaterializationEvidence {
    source_spans: Vec<RetainedPlanarSpan>,
    section_trims: Vec<RetainedSectionTrim>,
}

/// Comparable identity for one exact source-root scalar supplied later.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct SourceRootScalarKey {
    operand: usize,
    endpoint: usize,
}

impl SourceRootScalarKey {
    pub(crate) const fn new(operand: usize, endpoint: usize) -> Self {
        Self { operand, endpoint }
    }
}

/// Comparable identity for one exact Section-carrier trim scalar.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct SectionTrimScalarKey {
    fragment: usize,
    endpoint: usize,
}

impl SectionTrimScalarKey {
    pub(crate) const fn new(fragment: usize, endpoint: usize) -> Self {
        Self { fragment, endpoint }
    }
}

/// Candidate exact scalars. Certification remains read-only and exact.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MixedShellScalarInputs {
    source_roots: Vec<(SourceRootScalarKey, f64)>,
    section_trims: Vec<(SectionTrimScalarKey, f64)>,
}

impl MixedShellScalarInputs {
    pub(crate) const fn new(
        source_roots: Vec<(SourceRootScalarKey, f64)>,
        section_trims: Vec<(SectionTrimScalarKey, f64)>,
    ) -> Self {
        Self {
            source_roots,
            section_trims,
        }
    }

    pub(crate) const fn empty() -> Self {
        Self::new(Vec::new(), Vec::new())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PhysicalVertex {
    Source(RawVertexId),
    Section(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PhysicalCarrier {
    Source(RawEdgeId),
    Section(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PhysicalUse {
    face: usize,
    loop_index: usize,
    use_index: usize,
    forward: bool,
}

impl PhysicalUse {
    pub(crate) const fn face(self) -> usize {
        self.face
    }

    pub(crate) const fn loop_index(self) -> usize {
        self.loop_index
    }

    pub(crate) const fn use_index(self) -> usize {
        self.use_index
    }

    pub(crate) const fn forward(self) -> bool {
        self.forward
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PhysicalEdge {
    carrier: PhysicalCarrier,
    endpoints: Option<[PhysicalVertex; 2]>,
    uses: Vec<PhysicalUse>,
}

impl PhysicalEdge {
    pub(crate) const fn carrier(&self) -> PhysicalCarrier {
        self.carrier
    }

    pub(crate) const fn endpoints(&self) -> Option<[PhysicalVertex; 2]> {
        self.endpoints
    }

    pub(crate) fn uses(&self) -> &[PhysicalUse] {
        &self.uses
    }
}

/// Allocation-free, coalesced physical incidence ready for scalar completion.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellMaterializationBlueprint {
    edges: Vec<PhysicalEdge>,
    planar_use_count: usize,
    work: u64,
}

impl MixedShellMaterializationBlueprint {
    /// Canonical physical incidence after source-edge coalescing.
    pub(crate) fn edges(&self) -> &[PhysicalEdge] {
        &self.edges
    }

    pub(crate) const fn physical_edge_count(&self) -> usize {
        self.edges.len()
    }

    pub(crate) fn planar_edge_count(&self) -> usize {
        self.edges
            .iter()
            .filter(|edge| matches!(edge.carrier, PhysicalCarrier::Source(_)))
            .count()
    }

    pub(crate) const fn planar_use_count(&self) -> usize {
        self.planar_use_count
    }

    pub(crate) fn all_edges_have_two_opposed_uses(&self) -> bool {
        self.edges
            .iter()
            .all(|edge| edge.uses.len() == 2 && edge.uses[0].forward != edge.uses[1].forward)
    }

    /// Exact bounded work for orchestration-stage charging.
    pub(crate) const fn work(&self) -> u64 {
        self.work
    }
}

/// Read-only refusal before any topology transaction exists.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum MixedShellMaterializationError {
    UnresolvedMaterializationGap(MixedShellMaterializationGap),
    MissingPlanarLineage,
    EndpointFreeSourceRingMismatch,
    PlanVertexMismatch,
    EdgeUseCount { edge: usize, uses: usize },
    EdgeUsesNotOpposed(usize),
    SelfAdjacentEdge(usize),
    DuplicateSourceRootScalar(SourceRootScalarKey),
    DuplicateSectionTrimScalar(SectionTrimScalarKey),
    MissingSourceRootScalar(SourceRootScalarKey),
    MissingSectionTrimScalar(SectionTrimScalarKey),
    UnexpectedSourceRootScalar(SourceRootScalarKey),
    UnexpectedSectionTrimScalar(SectionTrimScalarKey),
    ScalarOutsideCertifiedRange,
    InvalidSourcePeriodLift,
    NonIncreasingEdgeRange(usize),
    EndpointBitsMismatch { distance: f64 },
    UnsupportedSourceCurve,
    UnsupportedSourceSurface,
    UnsupportedPcurve,
    ProjectedSourceCircleOnPlane(ProjectedSourceCircleOnPlaneError),
    InvalidAnalyticGeometry,
    InvalidEndpointFreePeriodicUse,
    NoCommonPeriodicWindow,
    PeriodShiftOverflow,
    MissingSourceDomain,
    ComponentFaceUnavailable(usize),
    ComponentEdgeUnavailable(AnalyticEdgeKey),
    ComponentVertexUnavailable(AnalyticVertexKey),
    ComponentEdgeCountMismatch { expected: usize, actual: usize },
    ComponentVertexCountMismatch { expected: usize, actual: usize },
    StoreRead,
    WorkCountOverflow,
    AnalyticPreflight(AnalyticShellPlanError),
}

impl From<PeriodicChartError> for MixedShellMaterializationError {
    fn from(error: PeriodicChartError) -> Self {
        match error {
            PeriodicChartError::InvalidAnalyticGeometry => Self::InvalidAnalyticGeometry,
            PeriodicChartError::InvalidEndpointFreePeriodicUse => {
                Self::InvalidEndpointFreePeriodicUse
            }
            PeriodicChartError::NoCommonPeriodicWindow => Self::NoCommonPeriodicWindow,
            PeriodicChartError::PeriodShiftOverflow => Self::PeriodShiftOverflow,
        }
    }
}

fn source_scalar_map(
    inputs: &MixedShellScalarInputs,
) -> Result<BTreeMap<SourceRootScalarKey, f64>, MixedShellMaterializationError> {
    let mut values = BTreeMap::new();
    for &(key, value) in &inputs.source_roots {
        if values.insert(key, value).is_some() {
            return Err(MixedShellMaterializationError::DuplicateSourceRootScalar(
                key,
            ));
        }
    }
    Ok(values)
}

fn trim_scalar_map(
    inputs: &MixedShellScalarInputs,
) -> Result<BTreeMap<SectionTrimScalarKey, f64>, MixedShellMaterializationError> {
    let mut values = BTreeMap::new();
    for &(key, value) in &inputs.section_trims {
        if values.insert(key, value).is_some() {
            return Err(MixedShellMaterializationError::DuplicateSectionTrimScalar(
                key,
            ));
        }
    }
    Ok(values)
}

fn source_parameter(
    source: MixedSourceFaceKey,
    evidence: &RetainedSpanParameter,
    inputs: &mut BTreeMap<SourceRootScalarKey, f64>,
    carrier_period: Option<f64>,
) -> Result<f64, MixedShellMaterializationError> {
    match evidence {
        RetainedSpanParameter::SourceVertex {
            edge_parameter_bits,
            ..
        } => Ok(f64::from_bits(*edge_parameter_bits)),
        RetainedSpanParameter::SectionRoot {
            endpoint,
            enclosure_bits,
            parameter_bits,
            period_shift,
        } => {
            let key = SourceRootScalarKey::new(source.operand(), *endpoint);
            let certified = f64::from_bits(*parameter_bits);
            let lo = f64::from_bits(enclosure_bits[0]);
            let hi = f64::from_bits(enclosure_bits[1]);
            if !lo.is_finite() || !hi.is_finite() || lo > hi {
                return Err(MixedShellMaterializationError::ScalarOutsideCertifiedRange);
            }
            let canonical = if certified.is_finite() && certified >= lo && certified <= hi {
                if let Some(candidate) = inputs.remove(&key)
                    && candidate.to_bits() != certified.to_bits()
                {
                    return Err(MixedShellMaterializationError::ScalarOutsideCertifiedRange);
                }
                certified
            } else {
                inputs
                    .remove(&key)
                    .filter(|candidate| {
                        candidate.is_finite() && *candidate >= lo && *candidate <= hi
                    })
                    .ok_or(MixedShellMaterializationError::MissingSourceRootScalar(key))?
            };
            if *period_shift == 0 {
                return Ok(canonical);
            }
            let period = carrier_period
                .filter(|period| period.is_finite() && *period > 0.0)
                .ok_or(MixedShellMaterializationError::InvalidSourcePeriodLift)?;
            let lifted = canonical + f64::from(*period_shift) * period;
            lifted
                .is_finite()
                .then_some(lifted)
                .ok_or(MixedShellMaterializationError::InvalidSourcePeriodLift)
        }
    }
}

fn retained_section_trim(
    plan: &MixedShellProofPlan,
    fragment: usize,
) -> Option<&RetainedSectionTrim> {
    plan.materialization
        .section_trims
        .iter()
        .find(|trim| trim.fragment == fragment)
}

fn section_parameters(
    plan: &MixedShellProofPlan,
    edge: &MixedSectionEdgePlan,
    inputs: &mut BTreeMap<SectionTrimScalarKey, f64>,
) -> Result<[f64; 2], MixedShellMaterializationError> {
    let retained = retained_section_trim(plan, edge.fragment_index()).ok_or(
        MixedShellMaterializationError::MissingSectionTrimScalar(SectionTrimScalarKey::new(
            edge.fragment_index(),
            edge.endpoints()[0],
        )),
    )?;
    let mut resolve = |index: usize| {
        let endpoint = retained.endpoints[index];
        let key = SectionTrimScalarKey::new(edge.fragment_index(), endpoint);
        if let Some(certified) = retained.certified.map(|values| values[index].0) {
            if let Some(candidate) = inputs.remove(&key)
                && candidate.to_bits() != certified.to_bits()
            {
                return Err(MixedShellMaterializationError::ScalarOutsideCertifiedRange);
            }
            Ok(certified)
        } else {
            inputs.remove(&key).filter(|value| value.is_finite()).ok_or(
                MixedShellMaterializationError::MissingSectionTrimScalar(key),
            )
        }
    };
    Ok([resolve(0)?, resolve(1)?])
}

fn source_carrier(
    store: &Store,
    edge_id: RawEdgeId,
) -> Result<AnalyticShellCurve, MixedShellMaterializationError> {
    let edge = store
        .get(edge_id)
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    let curve = edge
        .curve()
        .ok_or(MixedShellMaterializationError::UnsupportedSourceCurve)?;
    match store
        .curve(curve)
        .map_err(|_| MixedShellMaterializationError::StoreRead)?
    {
        CurveGeom::Line(line) => Ok(AnalyticShellCurve::Line(*line)),
        CurveGeom::Circle(circle) => Ok(AnalyticShellCurve::Circle(*circle)),
        _ => Err(MixedShellMaterializationError::UnsupportedSourceCurve),
    }
}

fn section_carrier(
    edge: &MixedSectionEdgePlan,
) -> Result<AnalyticShellCurve, MixedShellMaterializationError> {
    match edge.branch().carrier() {
        SectionCarrier::Line { origin, direction } => Line::new(origin, direction)
            .map(AnalyticShellCurve::Line)
            .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry),
        SectionCarrier::Circle {
            center,
            normal,
            x_direction,
            radius,
        } => Frame::new(center, normal, x_direction)
            .and_then(|frame| Circle::new(frame, radius))
            .map(AnalyticShellCurve::Circle)
            .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry),
        SectionCarrier::SkewCylinderBranch(_) => {
            Err(MixedShellMaterializationError::UnsupportedSourceCurve)
        }
    }
}

fn eval_carrier(carrier: AnalyticShellCurve, parameter: f64) -> Point3 {
    match carrier {
        AnalyticShellCurve::Line(line) => line.eval(parameter),
        AnalyticShellCurve::Circle(circle) => circle.eval(parameter),
        AnalyticShellCurve::PersistentSkewCylinderOpenSpan(curve) => curve.eval(parameter),
    }
}

fn same_point_bits(left: Point3, right: Point3) -> bool {
    left.x.to_bits() == right.x.to_bits()
        && left.y.to_bits() == right.y.to_bits()
        && left.z.to_bits() == right.z.to_bits()
}

fn intern_vertex(
    vertices: &mut Vec<(PhysicalVertex, Point3)>,
    key: PhysicalVertex,
    point: Point3,
) -> Result<AnalyticVertexKey, MixedShellMaterializationError> {
    if let Some((index, _)) = vertices
        .iter()
        .enumerate()
        .find(|(_, (candidate, _))| *candidate == key)
    {
        return u64::try_from(index)
            .map(AnalyticVertexKey::new)
            .map_err(|_| MixedShellMaterializationError::WorkCountOverflow);
    }
    let index = vertices.len();
    vertices.push((key, point));
    u64::try_from(index)
        .map(AnalyticVertexKey::new)
        .map_err(|_| MixedShellMaterializationError::WorkCountOverflow)
}

pub(super) fn retain_materialization_evidence(
    faces: &[MixedShellFacePlan],
    arrangements: &BTreeMap<MixedSourceFaceKey, MixedArrangementBinding<'_>>,
    bounded_source_spans: &[MixedBoundedSourceSpanPlan],
    graph: &BodySectionGraph,
    section_edges: &[MixedSectionEdgePlan],
) -> RetainedMaterializationEvidence {
    let used = faces
        .iter()
        .flat_map(MixedShellFacePlan::loops)
        .flat_map(|loop_| loop_.uses())
        .filter_map(|use_| match use_.edge() {
            MixedShellEdgeKey::PlanarSource { source, span } => Some((*source, span.clone())),
            _ => None,
        })
        .collect::<BTreeSet<_>>();
    let mut source_spans = used
        .into_iter()
        .filter_map(|(source, span)| {
            let MixedArrangementBinding::Planar { lineage, .. } = arrangements.get(&source)? else {
                return None;
            };
            let item = lineage.spans().iter().find(|item| item.key() == &span)?;
            let range = item.range().each_ref().map(|value| match value {
                MixedSourceParameterEvidence::SourceVertex {
                    topology_ordinal,
                    vertex,
                    edge_parameter_bits,
                    ..
                } => RetainedSpanParameter::SourceVertex {
                    topology_ordinal: *topology_ordinal,
                    vertex: *vertex,
                    edge_parameter_bits: *edge_parameter_bits,
                },
                MixedSourceParameterEvidence::SectionRoot {
                    endpoint,
                    enclosure_bits,
                    ..
                } => {
                    let parameter_bits = graph
                        .curve_endpoints()
                        .get(*endpoint)
                        .and_then(|endpoint| match endpoint.topology() {
                            SectionCurveEndpointTopology::Trim {
                                source_parameters, ..
                            } => source_parameters[source.operand()].as_ref(),
                            _ => None,
                        })
                        .map_or(u64::MAX, |parameter| parameter.root_parameter().to_bits());
                    RetainedSpanParameter::SectionRoot {
                        endpoint: *endpoint,
                        enclosure_bits: *enclosure_bits,
                        parameter_bits,
                        // Current bounded-polygon lineage is nonperiodic.
                        // Root-split whole-circle lineage must supply its
                        // certified source-span lift at this retention seam.
                        period_shift: 0,
                    }
                }
            });
            Some(RetainedPlanarSpan {
                source,
                span,
                loop_id: item.loop_id(),
                fin: item.fin(),
                edge: item.edge(),
                range,
            })
        })
        .collect::<Vec<_>>();
    source_spans.extend(bounded_source_spans.iter().map(|item| RetainedPlanarSpan {
        source: item.source(),
        span: item.span().clone(),
        loop_id: item.loop_id(),
        fin: item.fin(),
        edge: item.edge(),
        range: item.roots().map(|root| RetainedSpanParameter::SectionRoot {
            endpoint: root.endpoint(),
            enclosure_bits: root.enclosure().map(f64::to_bits),
            parameter_bits: root.parameter().to_bits(),
            period_shift: root.period_shift(),
        }),
    }));
    let section_trims = section_edges
        .iter()
        .map(|edge| {
            let certified =
                retained_periodic_trim_scalars(arrangements, graph, edge.fragment_index())
                    .or_else(|| retained_ruling_trim_scalars(edge.fragment()));
            RetainedSectionTrim {
                fragment: edge.fragment_index(),
                endpoints: edge.endpoints(),
                certified,
            }
        })
        .collect();
    RetainedMaterializationEvidence {
        source_spans,
        section_trims,
    }
}

fn retained_periodic_trim_scalars(
    arrangements: &BTreeMap<MixedSourceFaceKey, MixedArrangementBinding<'_>>,
    graph: &BodySectionGraph,
    fragment_index: usize,
) -> Option<[(f64, Point3); 2]> {
    if matches!(
        graph.curve_fragments().get(fragment_index)?.span(),
        crate::SectionCurveFragmentSpan::BoundedProcedural { .. }
    ) {
        // Periodic embedding trim scalars for a bounded procedural fragment
        // are residual guards strictly inside the physical roots.  The
        // Section-owned physical-root authority is an exact enclosure, not a
        // representable carrier scalar, so realization must keep its typed
        // trim-parameter gap until the composite carrier can consume that
        // authority directly.
        return None;
    }
    let global = graph
        .periodic_face_embeddings()
        .iter()
        .filter_map(|evidence| {
            let SectionPeriodicFaceEmbeddingEvidence::Certified(embedding) = evidence else {
                return None;
            };
            Some(embedding)
        });
    let local = arrangements.values().filter_map(|binding| {
        let MixedArrangementBinding::Periodic {
            face,
            operand,
            embedding: Some(embedding),
            ..
        } = binding
        else {
            return None;
        };
        (embedding.operand() == *operand && embedding.face() == *face).then_some(*embedding)
    });

    let mut retained = None;
    for embedding in global.chain(local) {
        let mut matches = embedding
            .components()
            .iter()
            .flat_map(|component| component.fragments())
            .chain(
                embedding
                    .boundary_traces()
                    .iter()
                    .flat_map(|trace| trace.fragments()),
            )
            .filter(|fragment| fragment.fragment() == fragment_index);
        let Some(fragment) = matches.next() else {
            continue;
        };
        if matches.next().is_some() {
            return None;
        }
        let candidate = fragment
            .trim_scalars()
            .each_ref()
            .map(|trim| (trim.carrier_parameter(), trim.point()));
        if retained.is_some_and(|current| !same_trim_scalars(current, candidate)) {
            return None;
        }
        retained = Some(candidate);
    }
    retained
}

fn same_trim_scalars(first: [(f64, Point3); 2], second: [(f64, Point3); 2]) -> bool {
    first.into_iter().zip(second).all(|(first, second)| {
        first.0.to_bits() == second.0.to_bits()
            && first.1.x.to_bits() == second.1.x.to_bits()
            && first.1.y.to_bits() == second.1.y.to_bits()
            && first.1.z.to_bits() == second.1.z.to_bits()
    })
}

fn retained_ruling_trim_scalars(
    fragment: &crate::SectionCurveFragment,
) -> Option<[(f64, Point3); 2]> {
    let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
        return None;
    };
    let values = endpoints.each_ref().map(|end| {
        let parameter = end.carrier_parameter();
        if !parameter.is_finite() {
            return None;
        }
        let trims = end.trims().iter().flatten().collect::<Vec<_>>();
        if trims.is_empty()
            || trims.iter().any(|trim| {
                let enclosure = trim.carrier_parameter();
                !enclosure.lo().is_finite()
                    || !enclosure.hi().is_finite()
                    || parameter < enclosure.lo()
                    || parameter > enclosure.hi()
            })
        {
            return None;
        }
        Some((parameter, end.point()))
    });
    Some([values[0]?, values[1]?])
}

fn opposite(direction: ArrangementDirection) -> ArrangementDirection {
    match direction {
        ArrangementDirection::Forward => ArrangementDirection::Reverse,
        ArrangementDirection::Reverse => ArrangementDirection::Forward,
    }
}

fn evidence_vertex(value: &RetainedSpanParameter) -> PhysicalVertex {
    match value {
        RetainedSpanParameter::SourceVertex { vertex, .. } => PhysicalVertex::Source(*vertex),
        RetainedSpanParameter::SectionRoot { endpoint, .. } => PhysicalVertex::Section(*endpoint),
    }
}

fn plan_vertex(
    value: &MixedShellVertexKey,
    retained: &RetainedPlanarSpan,
) -> Option<PhysicalVertex> {
    match value {
        MixedShellVertexKey::SectionEndpoint(endpoint) => Some(PhysicalVertex::Section(*endpoint)),
        MixedShellVertexKey::PlanarSourceVertex {
            source,
            topology_ordinal,
        } if *source == retained.source => retained.range.iter().find_map(|value| match value {
            RetainedSpanParameter::SourceVertex {
                topology_ordinal: candidate,
                vertex,
                ..
            } if candidate == topology_ordinal => Some(PhysicalVertex::Source(*vertex)),
            _ => None,
        }),
        MixedShellVertexKey::ProofSeam { .. } => None,
        _ => None,
    }
}

fn section_plan_vertex(value: &MixedShellVertexKey) -> Option<PhysicalVertex> {
    match value {
        MixedShellVertexKey::SectionEndpoint(endpoint) => Some(PhysicalVertex::Section(*endpoint)),
        _ => None,
    }
}

fn retained_span<'a>(
    plan: &'a MixedShellProofPlan,
    source: MixedSourceFaceKey,
    span: &MixedSourceSpanKey,
) -> Option<&'a RetainedPlanarSpan> {
    plan.materialization
        .source_spans
        .iter()
        .find(|item| item.source == source && &item.span == span)
}

fn add_physical_use(
    edges: &mut Vec<PhysicalEdge>,
    carrier: PhysicalCarrier,
    endpoints: Option<[PhysicalVertex; 2]>,
    use_: PhysicalUse,
) {
    if let Some(edge) = edges
        .iter_mut()
        .find(|edge| edge.carrier == carrier && edge.endpoints == endpoints)
    {
        edge.uses.push(use_);
    } else {
        edges.push(PhysicalEdge {
            carrier,
            endpoints,
            uses: vec![use_],
        });
    }
}

fn periodic_face_window_work(
    use_count: usize,
    bounded_count: usize,
    endpoint_free_count: usize,
) -> Option<u64> {
    let uses = u64::try_from(use_count).ok()?;
    let bounded = u64::try_from(bounded_count).ok()?;
    let endpoint_free = u64::try_from(endpoint_free_count).ok()?;
    // Per-use pcurve/bounds construction outside identity lookup is bounded
    // by 8U. Two endpoint sorts and all candidate-by-interval containment tests
    // are bounded by 12B²; endpoint canonicalization and candidate creation by
    // 16B; ring validation by 4R. The constant covers window selection.
    bounded
        .checked_mul(bounded)?
        .checked_mul(12)?
        .checked_add(bounded.checked_mul(16)?)?
        .checked_add(uses.checked_mul(8)?)?
        .checked_add(endpoint_free.checked_mul(4)?)?
        .checked_add(1)
}

fn periodic_window_application_work(edge_count: usize, endpoint_free_edges: usize) -> Option<u64> {
    let edges = u64::try_from(edge_count).ok()?;
    let rings = u64::try_from(endpoint_free_edges).ok()?;
    // Applying selected windows scans all E edge slots. Each of the R shifted
    // rings then searches at most R closed declarations by stable key.
    edges.checked_add(rings.checked_mul(rings)?)
}

fn work_count<T>(value: Option<T>) -> Result<T, MixedShellMaterializationError> {
    value.ok_or(MixedShellMaterializationError::WorkCountOverflow)
}

fn periodic_window_work(
    plan: &MixedShellProofPlan,
    physical_edges: &[PhysicalEdge],
    store: &Store,
) -> Result<u64, MixedShellMaterializationError> {
    let edge_count = physical_edges.len();
    let endpoint_free_edges = physical_edges
        .iter()
        .filter(|edge| edge.endpoints.is_none())
        .count();
    let mut work = work_count(periodic_window_application_work(
        edge_count,
        endpoint_free_edges,
    ))?;
    let mut total_uses = 0_usize;
    let mut planar_uses = 0_usize;
    let mut section_uses = 0_usize;
    let mut endpoint_free_uses = 0_usize;
    for face in plan.faces() {
        let mut uses = 0_usize;
        let mut bounded = 0_usize;
        let mut endpoint_free = 0_usize;
        for use_ in face.loops().iter().flat_map(|loop_| loop_.uses()) {
            uses = work_count(uses.checked_add(1))?;
            match use_.edge() {
                MixedShellEdgeKey::PlanarSource { .. } => {
                    planar_uses = work_count(planar_uses.checked_add(1))?;
                    bounded = work_count(bounded.checked_add(1))?;
                }
                MixedShellEdgeKey::SectionFragment(_) => {
                    section_uses = work_count(section_uses.checked_add(1))?;
                    bounded = work_count(bounded.checked_add(1))?;
                }
                MixedShellEdgeKey::PeriodicSource { .. } => {
                    endpoint_free_uses = work_count(endpoint_free_uses.checked_add(1))?;
                    endpoint_free = work_count(endpoint_free.checked_add(1))?;
                }
            }
        }
        total_uses = work_count(total_uses.checked_add(uses))?;
        // Preserve the initial all-face scan ceiling. Only an analytic
        // periodic face pays the common-window proof, including bounded-only
        // faces with no endpoint-free ring.
        let uses_u64 =
            u64::try_from(uses).map_err(|_| MixedShellMaterializationError::WorkCountOverflow)?;
        work = work_count(work.checked_add(uses_u64))?;
        let (surface, _, _) = source_face_geometry(face, store)?;
        if matches!(surface, AnalyticShellSurface::Cylinder(_))
            && (bounded != 0 || endpoint_free != 0)
        {
            let face_work = work_count(periodic_face_window_work(uses, bounded, endpoint_free))?;
            work = work_count(work.checked_add(face_work))?;
        }
    }
    let as_work =
        |value| u64::try_from(value).map_err(|_| MixedShellMaterializationError::WorkCountOverflow);
    let uses = as_work(total_uses)?;
    let edges = as_work(edge_count)?;
    let planar = as_work(planar_uses)?;
    let retained_spans = as_work(plan.materialization.source_spans.len())?;
    let section = as_work(section_uses)?;
    let section_edges = as_work(plan.section_edges.len())?;
    let endpoint_free = as_work(endpoint_free_uses)?;
    let cap_rings = as_work(plan.cap_rings().len())?;
    // Every use resolves its physical edge by scanning E edges and their two
    // uses. Lineage resolution then scans the relevant retained source spans,
    // section edges, or endpoint-free cap-ring bindings.
    let identity_work = work_count(
        uses.checked_mul(edges)
            .and_then(|value| value.checked_mul(2)),
    )?;
    work = work_count(work.checked_add(identity_work))?;
    work = work_count(work.checked_add(work_count(planar.checked_mul(retained_spans))?))?;
    work = work_count(work.checked_add(work_count(section.checked_mul(section_edges))?))?;
    work = work_count(work.checked_add(work_count(endpoint_free.checked_mul(cap_rings))?))?;
    Ok(work)
}

fn endpoint_free_ring(
    plan: &MixedShellProofPlan,
    source: MixedSourceFaceKey,
    loop_key: PeriodicSourceLoopKey,
) -> Result<&MixedCylinderCapRing, MixedShellMaterializationError> {
    let mut matching = plan
        .cap_rings()
        .iter()
        .filter(|ring| ring.side_source() == source && ring.side_loop_key() == loop_key);
    let ring = matching
        .next()
        .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
    if matching.next().is_some() {
        return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
    }
    Ok(ring)
}

#[allow(clippy::too_many_arguments)]
fn validate_periodic_source_use(
    plan: &MixedShellProofPlan,
    store: &Store,
    face_index: usize,
    loop_index: usize,
    use_index: usize,
    source: MixedSourceFaceKey,
    loop_key: PeriodicSourceLoopKey,
) -> Result<(RawEdgeId, RawFinId), MixedShellMaterializationError> {
    let ring = endpoint_free_ring(plan, source, loop_key)?;
    let face = plan
        .faces()
        .get(face_index)
        .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
    let loop_ = face
        .loops()
        .get(loop_index)
        .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
    let use_ = loop_
        .uses()
        .get(use_index)
        .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
    let [tail, head] = loop_
        .vertices()
        .get(use_index..=use_index + 1)
        .and_then(|vertices| <&[_; 2]>::try_from(vertices).ok())
        .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
    let expected_seam = MixedShellVertexKey::ProofSeam { source, loop_key };
    if tail != &expected_seam
        || head != &expected_seam
        || !matches!(use_.pcurve(), MixedPcurveLineage::SourceTopology)
    {
        return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
    }

    let (raw_loop, raw_fin) = if face.source() == ring.side_source()
        && face.source_face().raw()
            == store
                .get(ring.side_loop())
                .map_err(|_| MixedShellMaterializationError::StoreRead)?
                .face()
    {
        (ring.side_loop(), ring.side_fin())
    } else if face.source() == ring.cap_source() && face.source_face() == ring.cap_face() {
        (ring.cap_loop(), ring.cap_fin())
    } else {
        return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
    };
    let raw_loop_value = store
        .get(raw_loop)
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    let raw_fin_value = store
        .get(raw_fin)
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    let raw_edge_value = store
        .get(ring.edge())
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    if raw_loop_value.face() != face.source_face().raw()
        || raw_loop_value.fins() != [raw_fin]
        || raw_fin_value.parent() != raw_loop
        || raw_fin_value.edge() != ring.edge()
        || raw_edge_value.vertices() != [None, None]
        || raw_edge_value.bounds().is_some()
        || raw_edge_value.tolerance().is_some()
        || raw_edge_value.fins().len() != 2
        || !raw_edge_value.fins().contains(&ring.cap_fin())
        || !raw_edge_value.fins().contains(&ring.side_fin())
    {
        return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
    }
    let carrier = source_carrier(store, ring.edge())?;
    let AnalyticShellCurve::Circle(circle) = carrier else {
        return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
    };
    let pcurve = raw_fin_value
        .pcurve()
        .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
    let full = circle.param_range();
    let mapped = [
        pcurve.edge_to_pcurve().map(full.lo),
        pcurve.edge_to_pcurve().map(full.hi),
    ];
    if pcurve.range() != ParamRange::new(mapped[0].min(mapped[1]), mapped[0].max(mapped[1]))
        || pcurve.closure_winding().is_none()
    {
        return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
    }
    Ok((ring.edge(), raw_fin))
}

/// Coalesce face-qualified planar span uses by validated raw edge and exact
/// endpoint/root identity. Raw handles are compared only for equality and
/// never become ordering keys.
pub(crate) fn prepare_mixed_shell_materialization(
    plan: &MixedShellProofPlan,
    store: &Store,
) -> Result<MixedShellMaterializationBlueprint, MixedShellMaterializationError> {
    let mut edges = Vec::new();
    let mut planar_use_count = 0_usize;
    let section = plan
        .section_edges
        .iter()
        .map(|edge| (edge.fragment_index(), edge))
        .collect::<BTreeMap<_, _>>();
    for (face_index, face) in plan.faces.iter().enumerate() {
        for (loop_index, loop_) in face.loops().iter().enumerate() {
            for (use_index, use_) in loop_.uses().iter().enumerate() {
                let tail = &loop_.vertices()[use_index];
                let head = &loop_.vertices()[use_index + 1];
                match use_.edge() {
                    MixedShellEdgeKey::PlanarSource { source, span } => {
                        planar_use_count = planar_use_count
                            .checked_add(1)
                            .ok_or(MixedShellMaterializationError::WorkCountOverflow)?;
                        let retained = retained_span(plan, *source, span)
                            .ok_or(MixedShellMaterializationError::MissingPlanarLineage)?;
                        let fin = store
                            .get(retained.fin)
                            .map_err(|_| MixedShellMaterializationError::StoreRead)?;
                        if fin.edge() != retained.edge {
                            return Err(MixedShellMaterializationError::MissingPlanarLineage);
                        }
                        let mut range = retained.range.each_ref().map(evidence_vertex);
                        let mut direction = use_.direction();
                        if fin.sense() == Sense::Reversed {
                            range.reverse();
                            direction = opposite(direction);
                        }
                        let directed = if direction == ArrangementDirection::Forward {
                            range
                        } else {
                            [range[1], range[0]]
                        };
                        if plan_vertex(tail, retained) != Some(directed[0])
                            || plan_vertex(head, retained) != Some(directed[1])
                        {
                            return Err(MixedShellMaterializationError::PlanVertexMismatch);
                        }
                        add_physical_use(
                            &mut edges,
                            PhysicalCarrier::Source(retained.edge),
                            Some(range),
                            PhysicalUse {
                                face: face_index,
                                loop_index,
                                use_index,
                                forward: direction == ArrangementDirection::Forward,
                            },
                        );
                    }
                    MixedShellEdgeKey::SectionFragment(fragment) => {
                        let edge = section
                            .get(fragment)
                            .ok_or(MixedShellMaterializationError::PlanVertexMismatch)?;
                        let endpoints = edge.endpoints().map(PhysicalVertex::Section);
                        let directed = if use_.direction() == ArrangementDirection::Forward {
                            endpoints
                        } else {
                            [endpoints[1], endpoints[0]]
                        };
                        if section_plan_vertex(tail) != Some(directed[0])
                            || section_plan_vertex(head) != Some(directed[1])
                        {
                            return Err(MixedShellMaterializationError::PlanVertexMismatch);
                        }
                        add_physical_use(
                            &mut edges,
                            PhysicalCarrier::Section(*fragment),
                            Some(endpoints),
                            PhysicalUse {
                                face: face_index,
                                loop_index,
                                use_index,
                                forward: use_.direction() == ArrangementDirection::Forward,
                            },
                        );
                    }
                    MixedShellEdgeKey::PeriodicSource { source, loop_key } => {
                        let (raw_edge, _) = validate_periodic_source_use(
                            plan, store, face_index, loop_index, use_index, *source, *loop_key,
                        )?;
                        add_physical_use(
                            &mut edges,
                            PhysicalCarrier::Source(raw_edge),
                            None,
                            PhysicalUse {
                                face: face_index,
                                loop_index,
                                use_index,
                                forward: use_.direction() == ArrangementDirection::Forward,
                            },
                        );
                    }
                }
            }
        }
    }
    for (index, edge) in edges.iter().enumerate() {
        if edge.uses.len() != 2 {
            return Err(MixedShellMaterializationError::EdgeUseCount {
                edge: index,
                uses: edge.uses.len(),
            });
        }
        if edge.uses[0].forward == edge.uses[1].forward {
            return Err(MixedShellMaterializationError::EdgeUsesNotOpposed(index));
        }
        if edge.uses[0].face == edge.uses[1].face {
            return Err(MixedShellMaterializationError::SelfAdjacentEdge(index));
        }
    }
    let base_work = u64::try_from(plan.faces.len())
        .ok()
        .and_then(|value| value.checked_add(u64::try_from(edges.len()).ok()?))
        .and_then(|value| value.checked_add(u64::try_from(planar_use_count).ok()?))
        .and_then(|value| value.checked_add(u64::try_from(plan.section_edges.len()).ok()?))
        .ok_or(MixedShellMaterializationError::WorkCountOverflow)?;
    let periodic_window_work = periodic_window_work(plan, &edges, store)?;
    let work = base_work
        .checked_add(periodic_window_work)
        .ok_or(MixedShellMaterializationError::WorkCountOverflow)?;
    Ok(MixedShellMaterializationBlueprint {
        edges,
        planar_use_count,
        work,
    })
}

fn physical_use_at(
    plan: &MixedShellProofPlan,
    use_: PhysicalUse,
) -> Result<&super::MixedShellEdgeUse, MixedShellMaterializationError> {
    plan.faces
        .get(use_.face)
        .and_then(|face| face.loops().get(use_.loop_index))
        .and_then(|loop_| loop_.uses().get(use_.use_index))
        .ok_or(MixedShellMaterializationError::PlanVertexMismatch)
}

fn source_span_for_edge<'a>(
    plan: &'a MixedShellProofPlan,
    edge: &PhysicalEdge,
) -> Result<&'a RetainedPlanarSpan, MixedShellMaterializationError> {
    let use_ = edge
        .uses
        .first()
        .copied()
        .ok_or(MixedShellMaterializationError::MissingPlanarLineage)?;
    let MixedShellEdgeKey::PlanarSource { source, span } = physical_use_at(plan, use_)?.edge()
    else {
        return Err(MixedShellMaterializationError::MissingPlanarLineage);
    };
    let retained = retained_span(plan, *source, span)
        .ok_or(MixedShellMaterializationError::MissingPlanarLineage)?;
    if edge.carrier != PhysicalCarrier::Source(retained.edge) {
        return Err(MixedShellMaterializationError::MissingPlanarLineage);
    }
    Ok(retained)
}

fn intrinsic_source_range(
    plan: &MixedShellProofPlan,
    store: &Store,
    edge: &PhysicalEdge,
    source_scalars: &mut BTreeMap<SourceRootScalarKey, f64>,
) -> Result<[f64; 2], MixedShellMaterializationError> {
    let retained = source_span_for_edge(plan, edge)?;
    let fin = store
        .get(retained.fin)
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    let mut evidence = retained.range.each_ref();
    if fin.sense() == Sense::Reversed {
        evidence.reverse();
    }
    if Some(evidence.map(evidence_vertex)) != edge.endpoints {
        return Err(MixedShellMaterializationError::PlanVertexMismatch);
    }
    let carrier_period = match source_carrier(store, retained.edge)? {
        AnalyticShellCurve::Circle(circle) => Some(circle.param_range().width()),
        AnalyticShellCurve::Line(_) | AnalyticShellCurve::PersistentSkewCylinderOpenSpan(_) => None,
    };
    let parameters = [
        source_parameter(retained.source, evidence[0], source_scalars, carrier_period)?,
        source_parameter(retained.source, evidence[1], source_scalars, carrier_period)?,
    ];
    Ok(parameters)
}

fn section_edge(
    plan: &MixedShellProofPlan,
    fragment: usize,
) -> Result<&MixedSectionEdgePlan, MixedShellMaterializationError> {
    plan.section_edges
        .iter()
        .find(|edge| edge.fragment_index() == fragment)
        .ok_or(MixedShellMaterializationError::PlanVertexMismatch)
}

fn validate_range(
    edge_index: usize,
    parameters: [f64; 2],
) -> Result<ParamRange, MixedShellMaterializationError> {
    if parameters[0].is_finite() && parameters[1].is_finite() && parameters[0] < parameters[1] {
        Ok(ParamRange::new(parameters[0], parameters[1]))
    } else {
        Err(MixedShellMaterializationError::NonIncreasingEdgeRange(
            edge_index,
        ))
    }
}

fn checked_vertex_point(
    store: &Store,
    key: PhysicalVertex,
    evaluated: Point3,
    tolerance: f64,
) -> Result<Point3, MixedShellMaterializationError> {
    let authoritative = match key {
        PhysicalVertex::Source(vertex) => store
            .vertex_position(vertex)
            .map_err(|_| MixedShellMaterializationError::StoreRead)?,
        PhysicalVertex::Section(_) => evaluated,
    };
    if same_point_bits(authoritative, evaluated)
        || certify_point_distance(authoritative, evaluated, tolerance)
    {
        Ok(authoritative)
    } else {
        Err(MixedShellMaterializationError::EndpointBitsMismatch {
            distance: (authoritative - evaluated).norm(),
        })
    }
}

fn certify_point_distance(left: Point3, right: Point3, tolerance: f64) -> bool {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return false;
    }
    let distance_squared = [left.x, left.y, left.z]
        .into_iter()
        .zip([right.x, right.y, right.z])
        .fold(Interval::point(0.0), |sum, (left, right)| {
            sum + (Interval::point(left) - Interval::point(right)).square()
        });
    let allowed_squared = Interval::point(tolerance).square();
    distance_squared.hi().is_finite()
        && allowed_squared.lo().is_finite()
        && distance_squared.hi() <= allowed_squared.lo()
}

fn source_pcurve(
    store: &Store,
    retained: &RetainedPlanarSpan,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    let fin = store
        .get(retained.fin)
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    source_pcurve_for_fin(store, fin, false)
}

fn source_pcurve_for_fin(
    store: &Store,
    fin: &ktopo::entity::Fin,
    retain_closure_winding: bool,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    let pcurve = fin
        .pcurve()
        .ok_or(MixedShellMaterializationError::UnsupportedPcurve)?;
    let curve = match store
        .pcurve(pcurve.curve())
        .map_err(|_| MixedShellMaterializationError::StoreRead)?
    {
        Curve2dGeom::Line(line) => AnalyticShellPcurve::Line(*line),
        Curve2dGeom::Circle(circle) => AnalyticShellPcurve::Circle(*circle),
        _ => return Err(MixedShellMaterializationError::UnsupportedPcurve),
    };
    let raw_map = pcurve.edge_to_pcurve();
    let map = AffineParamMap1d::new(raw_map.scale(), raw_map.offset())
        .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?;
    Ok(apply_source_closure_winding(
        AnalyticPcurveUse::new(curve, map).with_chart(pcurve.chart()),
        pcurve.closure_winding(),
        retain_closure_winding,
    ))
}

fn apply_source_closure_winding(
    mut use_: AnalyticPcurveUse,
    winding: Option<[i32; 2]>,
    retain: bool,
) -> AnalyticPcurveUse {
    if retain && let Some(winding) = winding {
        use_ = use_.with_closure_winding(winding);
    }
    use_
}

fn section_pcurve(
    edge: &MixedSectionEdgePlan,
    lineage: &MixedPcurveLineage,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    let MixedPcurveLineage::Section {
        branch,
        operand,
        cylinder_period_shift,
    } = lineage
    else {
        return Err(MixedShellMaterializationError::UnsupportedPcurve);
    };
    if *branch != edge.fragment().branch() || *operand > 1 {
        return Err(MixedShellMaterializationError::UnsupportedPcurve);
    }
    let (curve, map) = match edge.branch().pcurves()[*operand] {
        SectionUvCurve::Line(line) => {
            let direction = Vec2::new(line.direction().x, line.direction().y);
            let scale = direction.norm();
            let curve = Line2d::new(line.origin(), direction)
                .map(AnalyticShellPcurve::Line)
                .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?;
            let map = AffineParamMap1d::new(scale, 0.0)
                .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?;
            (curve, map)
        }
        SectionUvCurve::Circle(circle) => {
            let curve = Circle2d::new(circle.center(), circle.radius(), circle.x_direction())
                .map(AnalyticShellPcurve::Circle)
                .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?;
            let map = AffineParamMap1d::new(circle.parameter_scale(), circle.parameter_offset())
                .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?;
            (curve, map)
        }
        SectionUvCurve::SkewCylinderBranch(_) => {
            return Err(MixedShellMaterializationError::UnsupportedPcurve);
        }
    };
    let shift = i32::try_from(*cylinder_period_shift)
        .map_err(|_| MixedShellMaterializationError::PeriodShiftOverflow)?;
    Ok(AnalyticPcurveUse::new(curve, map).with_chart(PcurveChart::shifted([shift, 0])))
}

fn source_face_geometry(
    plan: &MixedShellFacePlan,
    store: &Store,
) -> Result<(AnalyticShellSurface, Sense, ktopo::entity::FaceDomain), MixedShellMaterializationError>
{
    let face = store
        .get(plan.source_face().raw())
        .map_err(|_| MixedShellMaterializationError::StoreRead)?;
    let surface = match store
        .surface(face.surface())
        .map_err(|_| MixedShellMaterializationError::StoreRead)?
    {
        SurfaceGeom::Plane(plane) => AnalyticShellSurface::Plane(*plane),
        SurfaceGeom::Cylinder(cylinder) => AnalyticShellSurface::Cylinder(*cylinder),
        _ => return Err(MixedShellMaterializationError::UnsupportedSourceSurface),
    };
    let sense = match plan.selected_orientation() {
        SelectedOrientation::Preserved => face.sense(),
        SelectedOrientation::Reversed => face.sense().flipped(),
    };
    let domain = face
        .domain()
        .ok_or(MixedShellMaterializationError::MissingSourceDomain)?;
    Ok((surface, sense, domain))
}

#[cfg(test)]
fn surface_periodicity(surface: AnalyticShellSurface) -> [Option<f64>; 2] {
    match surface {
        AnalyticShellSurface::Plane(surface) => surface.periodicity(),
        AnalyticShellSurface::Cylinder(surface) => surface.periodicity(),
    }
}

fn pcurve_bounds(
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    edge_range: ParamRange,
) -> Result<(Point2, Point2), MixedShellMaterializationError> {
    periodic_chart::pcurve_bounds(surface, pcurve, edge_range).map_err(Into::into)
}

#[cfg(test)]
fn periodic_interval_shift(
    period: f64,
    window: ParamRange,
    interval: (f64, f64),
) -> Result<i32, MixedShellMaterializationError> {
    periodic_chart::periodic_interval_shift(period, window, interval).map_err(Into::into)
}

fn select_common_periodic_window(
    period: f64,
    authored: ParamRange,
    intervals: &[(f64, f64)],
) -> Result<ParamRange, MixedShellMaterializationError> {
    periodic_chart::select_common_periodic_window(period, authored, intervals).map_err(Into::into)
}

fn normalize_periodic_pcurve_chart(
    surface: AnalyticShellSurface,
    window: ParamRange,
    pcurve: AnalyticPcurveUse,
    edge_range: ParamRange,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    periodic_chart::normalize_periodic_pcurve_chart(surface, window, pcurve, edge_range)
        .map_err(Into::into)
}

fn include_pcurve_bounds(aggregate: &mut Option<(Point2, Point2)>, bounds: (Point2, Point2)) {
    if let Some((min, max)) = aggregate {
        min.x = min.x.min(bounds.0.x);
        min.y = min.y.min(bounds.0.y);
        max.x = max.x.max(bounds.1.x);
        max.y = max.y.max(bounds.1.y);
    } else {
        *aggregate = Some(bounds);
    }
}

fn physical_edge_for_use(
    blueprint: &MixedShellMaterializationBlueprint,
    location: PhysicalUse,
) -> Result<(usize, &PhysicalEdge, PhysicalUse), MixedShellMaterializationError> {
    blueprint
        .edges
        .iter()
        .enumerate()
        .find_map(|(edge_index, edge)| {
            edge.uses
                .iter()
                .copied()
                .find(|candidate| {
                    candidate.face == location.face
                        && candidate.loop_index == location.loop_index
                        && candidate.use_index == location.use_index
                })
                .map(|use_| (edge_index, edge, use_))
        })
        .ok_or(MixedShellMaterializationError::PlanVertexMismatch)
}

fn projected_source_circle_error(
    source: ProjectedSourceCircleOnPlaneError,
) -> MixedShellMaterializationError {
    MixedShellMaterializationError::ProjectedSourceCircleOnPlane(source)
}

#[allow(clippy::too_many_arguments)]
fn projected_source_circle_pcurve(
    plan: &MixedShellProofPlan,
    store: &Store,
    face_index: usize,
    loop_index: usize,
    use_index: usize,
    source: MixedSourceFaceKey,
    span: &MixedSourceSpanKey,
    retained: &RetainedPlanarSpan,
    physical: &PhysicalEdge,
    proof: &ProjectedSourceCircleOnPlane,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    let fail =
        || projected_source_circle_error(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    let target_face = plan.faces().get(face_index).ok_or_else(fail)?;
    if proof.source() != source
        || proof.span() != span
        || proof.target() != target_face.source()
        || proof.target_face() != target_face.source_face()
        || proof.loop_id() != retained.loop_id
        || proof.fin() != retained.fin
        || proof.edge() != retained.edge
        || retained.source != source
        || &retained.span != span
        || physical.carrier != PhysicalCarrier::Source(retained.edge)
        || physical.endpoints.is_none()
    {
        return Err(fail());
    }

    let mut bounded = plan
        .bounded_source_spans()
        .iter()
        .filter(|candidate| candidate.source() == source && candidate.span() == span);
    let source_span = bounded.next().ok_or_else(fail)?;
    if bounded.next().is_some()
        || source_span.loop_id() != retained.loop_id
        || source_span.fin() != retained.fin
        || source_span.edge() != retained.edge
    {
        return Err(fail());
    }

    let mut peers = physical.uses().iter().copied().filter(|candidate| {
        candidate.face() != face_index
            || candidate.loop_index() != loop_index
            || candidate.use_index() != use_index
    });
    let peer = peers.next().ok_or_else(fail)?;
    if peers.next().is_some() {
        return Err(fail());
    }
    let peer_face = plan.faces().get(peer.face()).ok_or_else(fail)?;
    let peer_use = physical_use_at(plan, peer)?;
    if peer_face.source() != source
        || peer_face.source_face() != proof.source_face()
        || !matches!(
            peer_use.edge(),
            MixedShellEdgeKey::PlanarSource {
                source: candidate_source,
                span: candidate_span,
            } if *candidate_source == source && candidate_span == span
        )
        || !matches!(peer_use.pcurve(), MixedPcurveLineage::SourceTopology)
    {
        return Err(fail());
    }

    let expected = ProjectedSourceCircleOnPlane::certify(
        store,
        proof.source_face(),
        source_span,
        target_face.source(),
        target_face.source_face(),
        proof.tolerance(),
    )
    .map_err(projected_source_circle_error)?;
    if &expected != proof {
        return Err(projected_source_circle_error(
            ProjectedSourceCircleOnPlaneError::InvalidProjection,
        ));
    }
    let curve = proof
        .circle()
        .map(AnalyticShellPcurve::Circle)
        .map_err(projected_source_circle_error)?;
    let map =
        AffineParamMap1d::new(proof.parameter_scale(), proof.parameter_offset()).map_err(|_| {
            projected_source_circle_error(ProjectedSourceCircleOnPlaneError::InvalidProjection)
        })?;
    Ok(AnalyticPcurveUse::new(curve, map))
}

#[allow(clippy::too_many_arguments)]
fn materialized_pcurve_for_use(
    plan: &MixedShellProofPlan,
    store: &Store,
    face_index: usize,
    loop_index: usize,
    use_index: usize,
    use_: &super::MixedShellEdgeUse,
    physical: &PhysicalEdge,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    match use_.edge() {
        MixedShellEdgeKey::PlanarSource { source, span } => {
            let retained = retained_span(plan, *source, span)
                .ok_or(MixedShellMaterializationError::MissingPlanarLineage)?;
            match use_.pcurve() {
                MixedPcurveLineage::SourceTopology => source_pcurve(store, retained),
                MixedPcurveLineage::ProjectedSourceCircleOnPlane(proof) => {
                    projected_source_circle_pcurve(
                        plan, store, face_index, loop_index, use_index, *source, span, retained,
                        physical, proof,
                    )
                }
                MixedPcurveLineage::Section { .. } => {
                    Err(MixedShellMaterializationError::UnsupportedPcurve)
                }
            }
        }
        MixedShellEdgeKey::SectionFragment(fragment) => {
            section_pcurve(section_edge(plan, *fragment)?, use_.pcurve())
        }
        MixedShellEdgeKey::PeriodicSource { source, loop_key } => {
            let (raw_edge, raw_fin) = validate_periodic_source_use(
                plan, store, face_index, loop_index, use_index, *source, *loop_key,
            )?;
            if physical.carrier != PhysicalCarrier::Source(raw_edge) || physical.endpoints.is_some()
            {
                return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
            }
            let fin = store
                .get(raw_fin)
                .map_err(|_| MixedShellMaterializationError::StoreRead)?;
            source_pcurve_for_fin(store, fin, true)
        }
    }
}

fn validate_endpoint_free_periodic_use(
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    edge_range: ParamRange,
) -> Result<(), MixedShellMaterializationError> {
    periodic_chart::shift_endpoint_free_periodic_ring(surface, pcurve, edge_range)
        .map(|_| ())
        .map_err(Into::into)
}

fn prepare_periodic_face_windows(
    plan: &MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
    store: &Store,
    closed_edges: &mut [AnalyticShellClosedEdge],
    edge_ranges: &mut [ParamRange],
) -> Result<Vec<Option<ParamRange>>, MixedShellMaterializationError> {
    let mut face_windows = vec![None; plan.faces.len()];
    let mut edge_windows = vec![None; blueprint.edges.len()];
    for (face_index, face) in plan.faces.iter().enumerate() {
        let (surface, _, source_domain) = source_face_geometry(face, store)?;
        let AnalyticShellSurface::Cylinder(cylinder) = surface else {
            continue;
        };
        let Some(period) = cylinder.periodicity()[0] else {
            return Err(MixedShellMaterializationError::InvalidAnalyticGeometry);
        };
        let mut bounded_intervals = Vec::new();
        let mut endpoint_free_uses = Vec::new();
        for (loop_index, loop_) in face.loops().iter().enumerate() {
            for (use_index, use_) in loop_.uses().iter().enumerate() {
                let location = PhysicalUse {
                    face: face_index,
                    loop_index,
                    use_index,
                    forward: false,
                };
                let (edge_index, physical, _) = physical_edge_for_use(blueprint, location)?;
                let pcurve = materialized_pcurve_for_use(
                    plan, store, face_index, loop_index, use_index, use_, physical,
                )?;
                if physical.endpoints.is_none() {
                    endpoint_free_uses.push((edge_index, pcurve));
                } else {
                    let (min, max) = pcurve_bounds(surface, pcurve, edge_ranges[edge_index])?;
                    bounded_intervals.push((min.x, max.x));
                }
            }
        }
        if endpoint_free_uses.is_empty() && bounded_intervals.is_empty() {
            continue;
        }
        let window = select_common_periodic_window(period, source_domain.u, &bounded_intervals)?;
        for (edge_index, pcurve) in endpoint_free_uses {
            validate_endpoint_free_periodic_use(surface, pcurve, window)?;
            if let Some(existing) = edge_windows[edge_index]
                && existing != window
            {
                return Err(MixedShellMaterializationError::NoCommonPeriodicWindow);
            }
            edge_windows[edge_index] = Some(window);
        }
        face_windows[face_index] = Some(window);
    }

    for (edge_index, window) in edge_windows.into_iter().enumerate() {
        let Some(window) = window else {
            continue;
        };
        let key = u64::try_from(edge_index)
            .map(AnalyticEdgeKey::new)
            .map_err(|_| MixedShellMaterializationError::WorkCountOverflow)?;
        let declaration = closed_edges
            .iter_mut()
            .find(|edge| edge.key() == key)
            .ok_or(MixedShellMaterializationError::EndpointFreeSourceRingMismatch)?;
        let mut shifted = AnalyticShellClosedEdge::new(key, declaration.carrier(), window);
        if let Some(source) = declaration.source() {
            shifted = shifted.with_source(source);
        }
        *declaration = shifted;
        edge_ranges[edge_index] = window;
    }
    Ok(face_windows)
}

/// Complete exact scalar evidence into a fully preflighted analytic-shell
/// proposal. This function is read-only; typed refusal cannot mutate topology.
fn build_mixed_shell_input(
    plan: &MixedShellProofPlan,
    store: &Store,
    scalars: &MixedShellScalarInputs,
    tolerance: f64,
) -> Result<AnalyticShellInput, MixedShellMaterializationError> {
    let blueprint = prepare_mixed_shell_materialization(plan, store)?;
    build_mixed_shell_input_from_blueprint(plan, &blueprint, store, scalars, tolerance)
}

fn build_mixed_shell_input_from_blueprint(
    plan: &MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
    store: &Store,
    scalars: &MixedShellScalarInputs,
    tolerance: f64,
) -> Result<AnalyticShellInput, MixedShellMaterializationError> {
    let mut source_scalars = source_scalar_map(scalars)?;
    let mut trim_scalars = trim_scalar_map(scalars)?;
    let mut retained_vertices = Vec::<(PhysicalVertex, Point3)>::new();
    let mut analytic_edges = Vec::with_capacity(blueprint.edges.len());
    let mut analytic_closed_edges = Vec::new();
    let mut analytic_edge_ranges = Vec::with_capacity(blueprint.edges.len());

    for (edge_index, physical) in blueprint.edges.iter().enumerate() {
        let key = u64::try_from(edge_index)
            .map(AnalyticEdgeKey::new)
            .map_err(|_| MixedShellMaterializationError::WorkCountOverflow)?;
        let Some(endpoints) = physical.endpoints else {
            let PhysicalCarrier::Source(raw_edge) = physical.carrier else {
                return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
            };
            let carrier = source_carrier(store, raw_edge)?;
            let AnalyticShellCurve::Circle(circle) = carrier else {
                return Err(MixedShellMaterializationError::EndpointFreeSourceRingMismatch);
            };
            let range = circle.param_range();
            analytic_closed_edges.push(
                AnalyticShellClosedEdge::new(key, carrier, range)
                    .with_source(EntityRef::Edge(raw_edge)),
            );
            analytic_edge_ranges.push(range);
            continue;
        };
        let (carrier, parameters, source) = match physical.carrier {
            PhysicalCarrier::Source(raw_edge) => {
                let parameters =
                    intrinsic_source_range(plan, store, physical, &mut source_scalars)?;
                (
                    source_carrier(store, raw_edge)?,
                    parameters,
                    Some(EntityRef::Edge(raw_edge)),
                )
            }
            PhysicalCarrier::Section(fragment) => {
                let section = section_edge(plan, fragment)?;
                let parameters = section_parameters(plan, section, &mut trim_scalars)?;
                if let Some(certified) =
                    retained_section_trim(plan, fragment).and_then(|trim| trim.certified)
                {
                    let carrier = section_carrier(section)?;
                    for endpoint in 0..2 {
                        let evaluated = eval_carrier(carrier, parameters[endpoint]);
                        if !same_point_bits(evaluated, certified[endpoint].1)
                            && !certify_point_distance(evaluated, certified[endpoint].1, tolerance)
                        {
                            return Err(MixedShellMaterializationError::EndpointBitsMismatch {
                                distance: (evaluated - certified[endpoint].1).norm(),
                            });
                        }
                    }
                    (carrier, parameters, None)
                } else {
                    (section_carrier(section)?, parameters, None)
                }
            }
        };
        let range = validate_range(edge_index, parameters)?;
        let evaluated = parameters.map(|parameter| eval_carrier(carrier, parameter));
        let mut vertices = [AnalyticVertexKey::new(0); 2];
        for endpoint in 0..2 {
            let point =
                checked_vertex_point(store, endpoints[endpoint], evaluated[endpoint], tolerance)?;
            vertices[endpoint] = intern_vertex(&mut retained_vertices, endpoints[endpoint], point)?;
        }
        let mut edge = AnalyticShellEdge::new(key, vertices, carrier, range);
        if let Some(source) = source {
            edge = edge.with_source(source);
        }
        analytic_edges.push(edge);
        analytic_edge_ranges.push(range);
    }

    if let Some((&key, _)) = source_scalars.first_key_value() {
        return Err(MixedShellMaterializationError::UnexpectedSourceRootScalar(
            key,
        ));
    }
    if let Some((&key, _)) = trim_scalars.first_key_value() {
        return Err(MixedShellMaterializationError::UnexpectedSectionTrimScalar(
            key,
        ));
    }

    let periodic_face_windows = prepare_periodic_face_windows(
        plan,
        blueprint,
        store,
        &mut analytic_closed_edges,
        &mut analytic_edge_ranges,
    )?;

    let analytic_vertices = retained_vertices
        .iter()
        .enumerate()
        .map(|(index, (_, point))| {
            u64::try_from(index)
                .map(AnalyticVertexKey::new)
                .map(|key| AnalyticShellVertex::new(key, *point))
                .map_err(|_| MixedShellMaterializationError::WorkCountOverflow)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut analytic_faces = Vec::with_capacity(plan.faces.len());
    for (face_index, face) in plan.faces.iter().enumerate() {
        let (surface, sense, source_domain) = source_face_geometry(face, store)?;
        let periodic_window = periodic_face_windows[face_index];
        let mut derived_bounds = None;
        let mut loops = Vec::with_capacity(face.loops().len());
        for (loop_index, loop_) in face.loops().iter().enumerate() {
            let mut fins = Vec::with_capacity(loop_.uses().len());
            for (use_index, use_) in loop_.uses().iter().enumerate() {
                let location = PhysicalUse {
                    face: face_index,
                    loop_index,
                    use_index,
                    forward: false,
                };
                let (edge_index, physical, physical_use) =
                    physical_edge_for_use(blueprint, location)?;
                let edge_key = u64::try_from(edge_index)
                    .map(AnalyticEdgeKey::new)
                    .map_err(|_| MixedShellMaterializationError::WorkCountOverflow)?;
                let pcurve = materialized_pcurve_for_use(
                    plan, store, face_index, loop_index, use_index, use_, physical,
                )?;
                let pcurve = if let Some(periodic_window) = periodic_window {
                    normalize_periodic_pcurve_chart(
                        surface,
                        periodic_window,
                        pcurve,
                        analytic_edge_ranges[edge_index],
                    )?
                } else {
                    pcurve
                };
                let sense = if physical_use.forward {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                if !physical.uses.contains(&physical_use) {
                    return Err(MixedShellMaterializationError::PlanVertexMismatch);
                }
                include_pcurve_bounds(
                    &mut derived_bounds,
                    pcurve_bounds(surface, pcurve, analytic_edge_ranges[edge_index])?,
                );
                fins.push(AnalyticShellFin::new(edge_key, sense, pcurve));
            }
            loops.push(AnalyticShellLoop::new(fins));
        }
        let domain = if let Some((min, max)) = derived_bounds {
            ktopo::entity::FaceDomain::from_bounds(min.x, max.x, min.y, max.y)
                .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?
        } else {
            source_domain
        };
        let key = u64::try_from(face_index)
            .map(AnalyticFaceKey::new)
            .map_err(|_| MixedShellMaterializationError::WorkCountOverflow)?;
        analytic_faces.push(
            AnalyticShellFace::new(key, surface, sense, domain, loops)
                .with_source(EntityRef::Face(face.source_face().raw())),
        );
    }
    Ok(
        AnalyticShellInput::new(analytic_vertices, analytic_edges, analytic_faces)
            .with_closed_edges(analytic_closed_edges),
    )
}

/// Materialize and preflight one connected mixed-shell proof plan.
pub(crate) fn materialize_mixed_shell_input(
    plan: &MixedShellProofPlan,
    store: &Store,
    scalars: &MixedShellScalarInputs,
    tolerance: f64,
) -> Result<AnalyticShellInput, MixedShellMaterializationError> {
    refuse_unresolved_materialization_gap(plan)?;
    let input = build_mixed_shell_input(plan, store, scalars, tolerance)?;
    prepare_analytic_shell(&input, store, tolerance)
        .map_err(MixedShellMaterializationError::AnalyticPreflight)?;
    Ok(input)
}

/// Materialize one disconnected proposal into independently connected inputs.
///
/// Component membership comes from exact physical edge incidence. This pass
/// retains the globally stable analytic keys but takes a complete face/edge/
/// vertex closure for each component, so keys remain comparable while the
/// topology batch assembler can treat them as component-local.
pub(crate) fn materialize_mixed_shell_component_inputs(
    plan: &MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
    components: &[MixedShellComponent],
    store: &Store,
    scalars: &MixedShellScalarInputs,
    tolerance: f64,
) -> Result<Vec<AnalyticShellInput>, MixedShellMaterializationError> {
    refuse_unresolved_materialization_gap(plan)?;
    let global =
        build_mixed_shell_input_from_blueprint(plan, blueprint, store, scalars, tolerance)?;
    components
        .iter()
        .map(|component| component_input(&global, component, store, tolerance))
        .collect()
}

fn refuse_unresolved_materialization_gap(
    plan: &MixedShellProofPlan,
) -> Result<(), MixedShellMaterializationError> {
    match plan.materialization_gaps().first() {
        Some(gap) => Err(MixedShellMaterializationError::UnresolvedMaterializationGap(gap.clone())),
        None => Ok(()),
    }
}

fn component_input(
    global: &AnalyticShellInput,
    component: &MixedShellComponent,
    store: &Store,
    tolerance: f64,
) -> Result<AnalyticShellInput, MixedShellMaterializationError> {
    let faces = component
        .faces()
        .iter()
        .map(|face| {
            global.faces().get(face.plan_index()).cloned().ok_or(
                MixedShellMaterializationError::ComponentFaceUnavailable(face.plan_index()),
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let edge_keys = faces
        .iter()
        .flat_map(|face| face.loops())
        .flat_map(|loop_| loop_.fins())
        .map(|fin| fin.edge())
        .collect::<BTreeSet<_>>();
    let edges = global
        .edges()
        .iter()
        .copied()
        .filter(|edge| edge_keys.contains(&edge.key()))
        .collect::<Vec<_>>();
    let closed_edges = global
        .closed_edges()
        .iter()
        .copied()
        .filter(|edge| edge_keys.contains(&edge.key()))
        .collect::<Vec<_>>();
    if edges.len() + closed_edges.len() != edge_keys.len() {
        let missing = edge_keys
            .iter()
            .find(|key| {
                !edges.iter().any(|edge| edge.key() == **key)
                    && !closed_edges.iter().any(|edge| edge.key() == **key)
            })
            .copied()
            .ok_or(MixedShellMaterializationError::PlanVertexMismatch)?;
        return Err(MixedShellMaterializationError::ComponentEdgeUnavailable(
            missing,
        ));
    }
    if edges.len() + closed_edges.len() != component.edges().len() {
        return Err(MixedShellMaterializationError::ComponentEdgeCountMismatch {
            expected: component.edges().len(),
            actual: edges.len() + closed_edges.len(),
        });
    }
    let vertex_keys = edges
        .iter()
        .flat_map(|edge| edge.vertices())
        .collect::<BTreeSet<_>>();
    let vertices = global
        .vertices()
        .iter()
        .copied()
        .filter(|vertex| vertex_keys.contains(&vertex.key()))
        .collect::<Vec<_>>();
    if vertices.len() != vertex_keys.len() {
        let missing = vertex_keys
            .iter()
            .find(|key| !vertices.iter().any(|vertex| vertex.key() == **key))
            .copied()
            .ok_or(MixedShellMaterializationError::PlanVertexMismatch)?;
        return Err(MixedShellMaterializationError::ComponentVertexUnavailable(
            missing,
        ));
    }
    if vertices.len() != component.vertices().len() {
        return Err(
            MixedShellMaterializationError::ComponentVertexCountMismatch {
                expected: component.vertices().len(),
                actual: vertices.len(),
            },
        );
    }
    let input = AnalyticShellInput::new(vertices, edges, faces).with_closed_edges(closed_edges);
    prepare_analytic_shell(&input, store, tolerance)
        .map_err(MixedShellMaterializationError::AnalyticPreflight)?;
    Ok(input)
}

pub(super) fn remaining_gaps(
    evidence: &RetainedMaterializationEvidence,
) -> Vec<MixedShellMaterializationGap> {
    let mut gaps = BTreeSet::new();
    for span in &evidence.source_spans {
        for parameter in &span.range {
            if let RetainedSpanParameter::SectionRoot {
                endpoint,
                parameter_bits,
                ..
            } = parameter
                && !f64::from_bits(*parameter_bits).is_finite()
            {
                gaps.insert(
                    MixedShellMaterializationGap::ExactSourceRootParameterRequired {
                        source: span.source,
                        span: span.span.clone(),
                        endpoint: *endpoint,
                    },
                );
            }
        }
    }
    for trim in &evidence.section_trims {
        if trim.certified.is_none() {
            for endpoint in trim.endpoints {
                gaps.insert(MixedShellMaterializationGap::ExactTrimParameterRequired {
                    fragment: trim.fragment,
                    endpoint,
                });
            }
        }
    }
    gaps.into_iter().collect()
}

#[cfg(test)]
#[path = "mixed_shell_materialize_tests.rs"]
mod tests;
