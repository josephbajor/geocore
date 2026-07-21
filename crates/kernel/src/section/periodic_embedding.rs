//! Certified Line2d embeddings of directed section components on cylinders.
//!
//! Exact endpoint indices own incidence. Outward parameter intervals own
//! chart alignment and winding. Numeric endpoint representatives are never
//! used for a join, period shift, orientation, or containment decision.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::{
    SECTION_WORK, SectionBranch, SectionCarrier, SectionCarrierParameterInterval,
    SectionCurveComponent, SectionCurveFragment, SectionCurveFragmentSpan, SectionUvCurve,
    SectionUvLine, curve_publish::carrier_point,
};
use crate::error::{Error, Result as KernelResult};
use crate::{FaceId, LoopId, PartId, Point3};

const PERIOD: f64 = core::f64::consts::TAU;

/// Outward enclosure of one cylinder surface parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionUvParameterInterval {
    lo: f64,
    hi: f64,
}

impl SectionUvParameterInterval {
    fn from_interval(value: Interval) -> Self {
        Self {
            lo: value.lo(),
            hi: value.hi(),
        }
    }

    /// Lower outward endpoint.
    pub const fn lo(self) -> f64 {
        self.lo
    }

    /// Upper outward endpoint.
    pub const fn hi(self) -> f64 {
        self.hi
    }
}

/// Certified orientation of one simple contractible lifted cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectionPeriodicCycleOrientation {
    /// Positive signed area in the certified lifted chart.
    Counterclockwise,
    /// Negative signed area in the certified lifted chart.
    Clockwise,
}

/// Authorable endpoint scalar retained by a certified periodic embedding.
///
/// The scalar lies inside the proof-owned carrier enclosure, its lifted UV
/// enclosure lies inside the fragment endpoint enclosure, and `point` is the
/// bit-exact result of evaluating the branch carrier at that scalar. Exact
/// source-root identity remains owned by the section endpoint; this numeric
/// witness does not replace it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionCarrierTrimScalarEvidence {
    endpoint: usize,
    carrier_parameter: f64,
    carrier_interval: SectionCarrierParameterInterval,
    point: Point3,
    lifted_uv: [SectionUvParameterInterval; 2],
}

impl SectionCarrierTrimScalarEvidence {
    /// Index into `BodySectionGraph::curve_endpoints`.
    pub const fn endpoint(&self) -> usize {
        self.endpoint
    }

    /// Finite scalar suitable for an analytic edge trim parameter.
    pub const fn carrier_parameter(&self) -> f64 {
        self.carrier_parameter
    }

    /// Proof-owned outward enclosure containing the retained scalar.
    pub const fn carrier_interval(&self) -> SectionCarrierParameterInterval {
        self.carrier_interval
    }

    /// Bit-exact carrier evaluation at [`Self::carrier_parameter`].
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Outward enclosure of the scalar's pcurve image in the lifted chart.
    pub const fn lifted_uv(&self) -> &[SectionUvParameterInterval; 2] {
        &self.lifted_uv
    }
}

/// One fragment's unique lift into a continuous cylinder chart.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionPeriodicFragmentEmbedding {
    fragment: usize,
    endpoints: [[SectionUvParameterInterval; 2]; 2],
    period_shift: i64,
    trim_scalars: [SectionCarrierTrimScalarEvidence; 2],
}

impl SectionPeriodicFragmentEmbedding {
    /// Index into `BodySectionGraph::curve_fragments`.
    pub const fn fragment(&self) -> usize {
        self.fragment
    }

    /// Lifted `[u, v]` endpoint enclosures in directed fragment order.
    pub const fn endpoints(&self) -> &[[SectionUvParameterInterval; 2]; 2] {
        &self.endpoints
    }

    /// Whole `u` periods added to the branch's canonical cylinder pcurve.
    pub const fn period_shift(&self) -> i64 {
        self.period_shift
    }

    /// Start/end carrier scalars certified against this lifted embedding.
    pub const fn trim_scalars(&self) -> &[SectionCarrierTrimScalarEvidence; 2] {
        &self.trim_scalars
    }
}

/// One exact directed component certified in a continuous cylinder chart.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionPeriodicComponentEmbedding {
    component: usize,
    fragments: Vec<SectionPeriodicFragmentEmbedding>,
    winding: i64,
    orientation: SectionPeriodicCycleOrientation,
    parent: Option<usize>,
}

impl SectionPeriodicComponentEmbedding {
    /// Index into `BodySectionGraph::curve_components`.
    pub const fn component(&self) -> usize {
        self.component
    }

    /// Directed fragment lifts in component traversal order.
    pub fn fragments(&self) -> &[SectionPeriodicFragmentEmbedding] {
        &self.fragments
    }

    /// Exact integer winding in the cylinder's periodic `u` direction.
    pub const fn winding(&self) -> i64 {
        self.winding
    }

    /// Certified signed orientation of the lifted simple cycle.
    pub const fn orientation(&self) -> SectionPeriodicCycleOrientation {
        self.orientation
    }

    /// Containing component, or `None` when certified directly in the
    /// annular source cell. The current prefix certifies only nonnested roots.
    pub const fn parent(&self) -> Option<usize> {
        self.parent
    }
}

/// Complete nonnested contractible-component evidence for one cylinder face.
#[derive(Debug, Clone, PartialEq)]
pub struct CertifiedSectionPeriodicFaceEmbedding {
    operand: usize,
    face: FaceId,
    source_loops: [LoopId; 2],
    source_loop_windings: [i32; 2],
    components: Vec<SectionPeriodicComponentEmbedding>,
}

impl CertifiedSectionPeriodicFaceEmbedding {
    /// Operand slot owning the cylinder face.
    pub const fn operand(&self) -> usize {
        self.operand
    }

    /// Cylinder face carrying every certified component.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// The two topology-owned ring loops bounding the finite cylinder side.
    pub fn source_loops(&self) -> &[LoopId; 2] {
        &self.source_loops
    }

    /// Exact signed `u` winding of each source loop in topology order.
    ///
    /// Fin sense is already composed into these nonzero, opposed values, so
    /// downstream arrangements can preserve which directed dart has the
    /// annular source domain on its left without relying on storage order.
    pub const fn source_loop_windings(&self) -> &[i32; 2] {
        &self.source_loop_windings
    }

    /// Nonnested contractible component embeddings on this face.
    pub fn components(&self) -> &[SectionPeriodicComponentEmbedding] {
        &self.components
    }
}

/// Typed missing obligation for periodic face embedding.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum SectionPeriodicEmbeddingGap {
    /// The cylinder face is not an annulus bounded by two whole ring loops.
    SourceFaceTopology,
    /// Only part of one directed component is carried by the face.
    ComponentLeavesFace {
        /// Graph component index.
        component: usize,
    },
    /// An endpoint lacks an outward carrier-parameter enclosure.
    CarrierIntervalUnavailable {
        /// Graph fragment index.
        fragment: usize,
    },
    /// A carrier endpoint cannot be assigned to one unique period.
    AmbiguousCarrierPeriod {
        /// Graph fragment index.
        fragment: usize,
    },
    /// A finite endpoint representative could not be placed uniquely inside
    /// its proof-owned carrier enclosure.
    CarrierTrimScalarUnavailable {
        /// Graph fragment index.
        fragment: usize,
        /// Graph endpoint index.
        endpoint: usize,
    },
    /// The retained scalar's pcurve image was not contained by the already
    /// certified endpoint box in the selected lifted chart.
    CarrierTrimScalarPcurveMismatch {
        /// Graph fragment index.
        fragment: usize,
        /// Graph endpoint index.
        endpoint: usize,
    },
    /// The face trace is not an affine line in cylinder parameters.
    NonLinearCylinderPcurve {
        /// Graph fragment index.
        fragment: usize,
    },
    /// Shared endpoint intervals do not select one unique chart lift.
    EndpointChartMismatch {
        /// Graph component index.
        component: usize,
    },
    /// The closed component winds around the cylinder and is not a disk cut.
    NonContractible {
        /// Graph component index.
        component: usize,
        /// Certified nonzero periodic winding.
        winding: i64,
    },
    /// Conservative segment boxes cannot exclude a self-intersection.
    SelfIntersectionProofRequired {
        /// Graph component index.
        component: usize,
    },
    /// Conservative segment boxes cannot separate two components.
    ComponentIntersectionProofRequired {
        /// First graph component index.
        first: usize,
        /// Second graph component index.
        second: usize,
    },
    /// Outward signed-area accumulation contains zero.
    OrientationIndeterminate {
        /// Graph component index.
        component: usize,
    },
    /// Disjoint components may nest and require an exact containment proof.
    ContainmentClassificationRequired {
        /// First graph component index.
        first: usize,
        /// Second graph component index.
        second: usize,
    },
    /// The checked geometry-independent pair-work precharge overflowed.
    PairWorkOverflow,
}

/// Certified result or precise missing obligation for one cylinder face.
#[derive(Debug, Clone, PartialEq)]
pub enum SectionPeriodicFaceEmbeddingEvidence {
    /// Every required periodic embedding invariant was certified.
    Certified(CertifiedSectionPeriodicFaceEmbedding),
    /// Evidence stopped at the named exact missing obligation.
    Indeterminate {
        /// Operand slot owning the cylinder face.
        operand: usize,
        /// Cylinder face whose embedding was attempted.
        face: FaceId,
        /// First obligation that could not be certified.
        gap: SectionPeriodicEmbeddingGap,
    },
}

impl SectionPeriodicFaceEmbeddingEvidence {
    /// Operand slot owning this evidence.
    pub const fn operand(&self) -> usize {
        match self {
            Self::Certified(value) => value.operand,
            Self::Indeterminate { operand, .. } => *operand,
        }
    }

    /// Cylinder face owning this evidence.
    pub fn face(&self) -> FaceId {
        match self {
            Self::Certified(value) => value.face(),
            Self::Indeterminate { face, .. } => face.clone(),
        }
    }

    /// Missing obligation, or `None` for certified evidence.
    pub const fn gap(&self) -> Option<&SectionPeriodicEmbeddingGap> {
        match self {
            Self::Certified(_) => None,
            Self::Indeterminate { gap, .. } => Some(gap),
        }
    }
}

#[derive(Clone, Copy)]
struct EndpointBox {
    endpoint: usize,
    uv: [Interval; 2],
}

#[derive(Clone)]
struct LiftedFragment {
    fragment: usize,
    endpoints: [EndpointBox; 2],
    shift: i64,
    direction: [f64; 2],
    origin: [Interval; 2],
    carrier: SectionCarrier,
    pcurve: SectionUvLine,
    parameters: [Interval; 2],
    representatives: [f64; 2],
}

#[derive(Clone, Copy)]
struct Bounds2 {
    u: Interval,
    v: Interval,
}

#[derive(Clone, Copy)]
struct PeriodicCertificationInput<'a> {
    store: &'a Store,
    part: &'a PartId,
    branches: &'a [SectionBranch],
    fragments: &'a [SectionCurveFragment],
    components: &'a [SectionCurveComponent],
    linear: f64,
}

pub(super) fn certify_periodic_faces(
    store: &Store,
    part: &PartId,
    branches: &[SectionBranch],
    fragments: &[SectionCurveFragment],
    components: &[SectionCurveComponent],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> KernelResult<Vec<SectionPeriodicFaceEmbeddingEvidence>> {
    let mut faces: Vec<(usize, FaceId)> = Vec::new();
    for fragment in fragments {
        let Some(branch) = branches.get(fragment.branch()) else {
            continue;
        };
        for operand in 0..2 {
            let face = branch.faces()[operand].clone();
            if faces
                .iter()
                .any(|candidate| candidate == &(operand, face.clone()))
            {
                continue;
            }
            let Ok(raw_face) = store.get(face.raw()) else {
                continue;
            };
            if matches!(store.get(raw_face.surface()), Ok(SurfaceGeom::Cylinder(_))) {
                faces.push((operand, face));
            }
        }
    }
    if !charge_pair_candidates(components, faces.len(), scope)? {
        return Ok(faces
            .into_iter()
            .map(
                |(operand, face)| SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
                    operand,
                    face,
                    gap: SectionPeriodicEmbeddingGap::PairWorkOverflow,
                },
            )
            .collect());
    }
    let input = PeriodicCertificationInput {
        store,
        part,
        branches,
        fragments,
        components,
        linear,
    };
    Ok(faces
        .into_iter()
        .map(
            |(operand, face)| match certify_face(input, operand, face.clone()) {
                Ok(value) => SectionPeriodicFaceEmbeddingEvidence::Certified(value),
                Err(gap) => {
                    SectionPeriodicFaceEmbeddingEvidence::Indeterminate { operand, face, gap }
                }
            },
        )
        .collect())
}

/// Precharge the geometry-independent ceiling for all pairwise simplicity
/// and component-separation candidates. Every unordered fragment pair is
/// owned exactly once: either by one component's simplicity proof or by one
/// cross-component separation proof. Component bounds add one candidate per
/// unordered component pair. Multiplication by the discovered periodic-face
/// count covers each face-local embedding attempt without depending on an
/// early geometric exit.
fn charge_pair_candidates(
    components: &[SectionCurveComponent],
    periodic_faces: usize,
    scope: &mut OperationScope<'_, '_>,
) -> KernelResult<bool> {
    let Some(amount) = periodic_pair_candidate_work(components, periodic_faces) else {
        return Ok(false);
    };
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)?;
    Ok(true)
}

fn periodic_pair_candidate_work(
    components: &[SectionCurveComponent],
    periodic_faces: usize,
) -> Option<u64> {
    let fragment_uses = components.iter().try_fold(0_u64, |total, component| {
        total.checked_add(u64::try_from(component.fragments().len()).ok()?)
    })?;
    let component_count = u64::try_from(components.len()).ok()?;
    let face_count = u64::try_from(periodic_faces).ok()?;
    unordered_pairs(fragment_uses)?
        .checked_add(unordered_pairs(component_count)?)?
        .checked_mul(face_count)
}

fn unordered_pairs(count: u64) -> Option<u64> {
    if count < 2 {
        return Some(0);
    }
    let predecessor = count.checked_sub(1)?;
    let (left, right) = if count.is_multiple_of(2) {
        (count / 2, predecessor)
    } else {
        (count, predecessor / 2)
    };
    left.checked_mul(right)
}

fn certify_face(
    input: PeriodicCertificationInput<'_>,
    operand: usize,
    face: FaceId,
) -> Result<CertifiedSectionPeriodicFaceEmbedding, SectionPeriodicEmbeddingGap> {
    let PeriodicCertificationInput {
        store,
        part,
        branches,
        fragments,
        components,
        linear,
    } = input;
    let raw = store
        .get(face.raw())
        .map_err(|_| SectionPeriodicEmbeddingGap::SourceFaceTopology)?;
    let [lower, upper] = raw.loops() else {
        return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
    };
    let mut ring_windings = Vec::new();
    let mut ring_heights = Vec::new();
    for loop_id in [*lower, *upper] {
        let loop_ = store
            .get(loop_id)
            .map_err(|_| SectionPeriodicEmbeddingGap::SourceFaceTopology)?;
        let [fin] = loop_.fins() else {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        };
        if loop_.face() != face.raw() {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        }
        let fin_data = store
            .get(*fin)
            .map_err(|_| SectionPeriodicEmbeddingGap::SourceFaceTopology)?;
        if fin_data.parent() != loop_id
            || certify_whole_fin_incidence(store, face.raw(), loop_id, *fin, linear)
                != WholeFinIncidence::Certified
        {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        }
        let edge = store
            .get(fin_data.edge())
            .map_err(|_| SectionPeriodicEmbeddingGap::SourceFaceTopology)?;
        let (Some(curve), Some(use_)) = (edge.curve(), fin_data.pcurve()) else {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        };
        if edge.vertices() != [None, None]
            || edge.bounds().is_some()
            || !edge.fins().contains(fin)
            || !matches!(store.get(curve), Ok(CurveGeom::Circle(_)))
            || use_.seam().is_some()
            || use_.chart().period_shifts()[1] != 0
        {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        }
        let Some(winding) = use_.closure_winding() else {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        };
        if winding[0] == 0 || winding[1] != 0 {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        }
        let Ok(Curve2dGeom::Line(boundary)) = store.get(use_.curve()) else {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        };
        if boundary.dir().x == 0.0 || boundary.dir().y != 0.0 {
            return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
        }
        ring_windings.push(if fin_data.sense().is_forward() {
            winding[0]
        } else {
            -winding[0]
        });
        ring_heights.push(boundary.origin().y);
    }
    if ring_windings[0].signum() == ring_windings[1].signum()
        || !ring_heights.iter().all(|height| height.is_finite())
        || ring_heights[0] == ring_heights[1]
    {
        return Err(SectionPeriodicEmbeddingGap::SourceFaceTopology);
    }

    let mut certified = Vec::new();
    for (component_index, component) in components.iter().enumerate() {
        let carried = component.fragments().iter().filter(|&&fragment| {
            fragments
                .get(fragment)
                .and_then(|fragment| branches.get(fragment.branch()))
                .is_some_and(|branch| branch.faces()[operand] == face)
        });
        let carried_count = carried.count();
        if carried_count == 0 {
            continue;
        }
        if carried_count != component.fragments().len() {
            return Err(SectionPeriodicEmbeddingGap::ComponentLeavesFace {
                component: component_index,
            });
        }
        certified.push(certify_component(
            branches,
            fragments,
            component_index,
            component,
            operand,
        )?);
    }
    certify_component_separation(&certified)?;
    Ok(CertifiedSectionPeriodicFaceEmbedding {
        operand,
        face,
        source_loops: [
            LoopId::new(part.clone(), *lower),
            LoopId::new(part.clone(), *upper),
        ],
        source_loop_windings: [ring_windings[0], ring_windings[1]],
        components: certified,
    })
}

fn certify_component(
    branches: &[SectionBranch],
    fragments: &[SectionCurveFragment],
    component_index: usize,
    component: &SectionCurveComponent,
    operand: usize,
) -> Result<SectionPeriodicComponentEmbedding, SectionPeriodicEmbeddingGap> {
    let mut lifted = Vec::new();
    for &fragment_index in component.fragments() {
        let fragment = fragments.get(fragment_index).ok_or(
            SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                fragment: fragment_index,
            },
        )?;
        let branch = branches.get(fragment.branch()).ok_or(
            SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                fragment: fragment_index,
            },
        )?;
        let SectionUvCurve::Line(pcurve) = branch.pcurves()[operand] else {
            return Err(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve {
                fragment: fragment_index,
            });
        };
        let parameters = fragment_parameter_intervals(fragment_index, fragment, branch)?;
        let representatives = fragment_endpoint_representatives(fragment).ok_or(
            SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                fragment: fragment_index,
            },
        )?;
        let endpoint_ids = fragment_endpoint_ids(fragment).ok_or(
            SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                fragment: fragment_index,
            },
        )?;
        let endpoints = core::array::from_fn(|end| EndpointBox {
            endpoint: endpoint_ids[end],
            uv: map_line_interval(pcurve, parameters[end]),
        });
        lifted.push(LiftedFragment {
            fragment: fragment_index,
            endpoints,
            shift: 0,
            direction: [pcurve.direction().x, pcurve.direction().y],
            origin: [
                Interval::point(pcurve.origin().x),
                Interval::point(pcurve.origin().y),
            ],
            carrier: branch.carrier(),
            pcurve,
            parameters,
            representatives,
        });
    }
    if lifted.is_empty() {
        return Err(SectionPeriodicEmbeddingGap::SelfIntersectionProofRequired {
            component: component_index,
        });
    }
    for index in 1..lifted.len() {
        let previous = lifted[index - 1].endpoints[1];
        let current = lifted[index].endpoints[0];
        if previous.endpoint != current.endpoint || !previous.uv[1].intersects(current.uv[1]) {
            return Err(SectionPeriodicEmbeddingGap::EndpointChartMismatch {
                component: component_index,
            });
        }
        let shift = unique_period_shift(previous.uv[0], current.uv[0]).ok_or(
            SectionPeriodicEmbeddingGap::EndpointChartMismatch {
                component: component_index,
            },
        )?;
        shift_fragment(&mut lifted[index], shift);
    }
    let first = lifted[0].endpoints[0];
    let last = lifted[lifted.len() - 1].endpoints[1];
    if first.endpoint != last.endpoint || !first.uv[1].intersects(last.uv[1]) {
        return Err(SectionPeriodicEmbeddingGap::EndpointChartMismatch {
            component: component_index,
        });
    }
    let winding = unique_period_shift(last.uv[0], first.uv[0]).ok_or(
        SectionPeriodicEmbeddingGap::EndpointChartMismatch {
            component: component_index,
        },
    )?;
    if winding != 0 {
        return Err(SectionPeriodicEmbeddingGap::NonContractible {
            component: component_index,
            winding,
        });
    }
    certify_simple_cycle(component_index, &lifted)?;
    let orientation = cycle_orientation(component_index, &lifted)?;
    let mut public_fragments = Vec::with_capacity(lifted.len());
    for fragment in &lifted {
        public_fragments.push(SectionPeriodicFragmentEmbedding {
            fragment: fragment.fragment,
            endpoints: fragment
                .endpoints
                .map(|endpoint| endpoint.uv.map(SectionUvParameterInterval::from_interval)),
            period_shift: fragment.shift,
            trim_scalars: certify_fragment_trim_scalars(fragment)?,
        });
    }
    Ok(SectionPeriodicComponentEmbedding {
        component: component_index,
        fragments: public_fragments,
        winding,
        orientation,
        parent: None,
    })
}

fn fragment_endpoint_representatives(fragment: &SectionCurveFragment) -> Option<[f64; 2]> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => None,
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            Some(endpoints.each_ref().map(|end| end.carrier_parameter()))
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            Some(endpoints.each_ref().map(|end| end.carrier_parameter()))
        }
    }
}

fn certify_fragment_trim_scalars(
    fragment: &LiftedFragment,
) -> Result<[SectionCarrierTrimScalarEvidence; 2], SectionPeriodicEmbeddingGap> {
    Ok([
        certify_trim_scalar(fragment, 0)?,
        certify_trim_scalar(fragment, 1)?,
    ])
}

fn certify_trim_scalar(
    fragment: &LiftedFragment,
    end: usize,
) -> Result<SectionCarrierTrimScalarEvidence, SectionPeriodicEmbeddingGap> {
    let endpoint = fragment.endpoints[end].endpoint;
    let parameter = select_trim_scalar(
        fragment.representatives[end],
        fragment.parameters[end],
        matches!(fragment.carrier, SectionCarrier::Circle { .. }),
    )
    .ok_or(SectionPeriodicEmbeddingGap::CarrierTrimScalarUnavailable {
        fragment: fragment.fragment,
        endpoint,
    })?;
    let point = carrier_point(fragment.carrier, parameter).ok_or(
        SectionPeriodicEmbeddingGap::CarrierTrimScalarUnavailable {
            fragment: fragment.fragment,
            endpoint,
        },
    )?;
    let mut lifted_uv = map_line_interval(fragment.pcurve, Interval::point(parameter));
    lifted_uv[0] = lifted_uv[0]
        + integer_period_interval(fragment.shift).ok_or(
            SectionPeriodicEmbeddingGap::CarrierTrimScalarPcurveMismatch {
                fragment: fragment.fragment,
                endpoint,
            },
        )?;
    if !(0..2).all(|axis| contains_interval(fragment.endpoints[end].uv[axis], lifted_uv[axis])) {
        return Err(
            SectionPeriodicEmbeddingGap::CarrierTrimScalarPcurveMismatch {
                fragment: fragment.fragment,
                endpoint,
            },
        );
    }
    Ok(SectionCarrierTrimScalarEvidence {
        endpoint,
        carrier_parameter: parameter,
        carrier_interval: SectionCarrierParameterInterval::from_interval(fragment.parameters[end]),
        point,
        lifted_uv: lifted_uv.map(SectionUvParameterInterval::from_interval),
    })
}

fn select_trim_scalar(representative: f64, enclosure: Interval, periodic: bool) -> Option<f64> {
    if !representative.is_finite() {
        return None;
    }
    if !periodic {
        return enclosure.contains(representative).then_some(representative);
    }
    let candidates =
        (enclosure - Interval::point(representative)).checked_div(Interval::point(PERIOD))?;
    let lower = candidates.lo().ceil();
    let upper = candidates.hi().floor();
    if !valid_unique_integer(lower, upper) {
        return None;
    }
    let shift = lower as i64;
    let parameter = representative + shift as f64 * PERIOD;
    (parameter.is_finite() && enclosure.contains(parameter)).then_some(parameter)
}

fn contains_interval(outer: Interval, inner: Interval) -> bool {
    outer.lo() <= inner.lo() && inner.hi() <= outer.hi()
}

fn fragment_endpoint_ids(fragment: &SectionCurveFragment) -> Option<[usize; 2]> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => None,
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            Some(endpoints.each_ref().map(|end| end.endpoint()))
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            Some(endpoints.each_ref().map(|end| end.endpoint()))
        }
    }
}

fn fragment_parameter_intervals(
    fragment_index: usize,
    fragment: &SectionCurveFragment,
    branch: &SectionBranch,
) -> Result<[Interval; 2], SectionPeriodicEmbeddingGap> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => {
            Err(SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                fragment: fragment_index,
            })
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            let values = endpoints.each_ref().map(|end| {
                end.trims()
                    .iter()
                    .flatten()
                    .next()
                    .map(|trim| public_interval(trim.carrier_parameter()))
            });
            let [Some(start), Some(end)] = values else {
                return Err(SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                    fragment: fragment_index,
                });
            };
            if start.hi() >= end.lo() {
                return Err(SectionPeriodicEmbeddingGap::AmbiguousCarrierPeriod {
                    fragment: fragment_index,
                });
            }
            Ok([start, end])
        }
        SectionCurveFragmentSpan::Arc {
            endpoints,
            wraps_pcurve_seam: _,
        } => {
            let values = endpoints.each_ref().map(|end| {
                arc_carrier_interval(end.trim().pcurve_half_angle(), end.trim().operand(), branch)
            });
            let [Some(start), Some(mut end)] = values else {
                return Err(SectionPeriodicEmbeddingGap::CarrierIntervalUnavailable {
                    fragment: fragment_index,
                });
            };
            if end.hi() <= start.lo() {
                end = end + Interval::point(PERIOD);
            }
            if start.hi() >= end.lo() {
                return Err(SectionPeriodicEmbeddingGap::AmbiguousCarrierPeriod {
                    fragment: fragment_index,
                });
            }
            Ok([start, end])
        }
    }
}

fn public_interval(value: SectionCarrierParameterInterval) -> Interval {
    Interval::new(value.lo(), value.hi())
}

fn arc_carrier_interval(
    half_angle: super::SectionProjectiveParameterInterval,
    trim_operand: usize,
    branch: &SectionBranch,
) -> Option<Interval> {
    let SectionUvCurve::Circle(circle) = branch.pcurves()[trim_operand] else {
        return None;
    };
    if circle.parameter_scale().abs() != 1.0 {
        return None;
    }
    let principal = twice_atan_interval(Interval::new(half_angle.lo(), half_angle.hi()))?;
    let base = (principal - Interval::point(circle.parameter_offset()))
        .checked_div(Interval::point(circle.parameter_scale()))?;
    unique_period_in_range(base, branch.range())
}

fn twice_atan_interval(value: Interval) -> Option<Interval> {
    let mut lo = 2.0 * math::atan(value.lo());
    let mut hi = 2.0 * math::atan(value.hi());
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return None;
    }
    for _ in 0..4 {
        lo = lo.next_down();
        hi = hi.next_up();
    }
    Some(Interval::new(lo, hi))
}

fn unique_period_in_range(value: Interval, range: kgeom::param::ParamRange) -> Option<Interval> {
    let divisor = Interval::point(PERIOD);
    let lower_bound =
        (Interval::point(range.lo) - Interval::point(value.lo())).checked_div(divisor)?;
    let upper_bound =
        (Interval::point(range.hi) - Interval::point(value.hi())).checked_div(divisor)?;
    let lower = lower_bound.hi().ceil();
    let upper = upper_bound.lo().floor();
    if !valid_unique_integer(lower, upper) {
        return None;
    }
    let shifted = value + integer_period_interval(lower as i64)?;
    (shifted.lo() >= range.lo && shifted.hi() <= range.hi).then_some(shifted)
}

fn map_line_interval(line: super::SectionUvLine, parameter: Interval) -> [Interval; 2] {
    let origin = line.origin();
    let direction = line.direction();
    [
        Interval::point(origin.x) + Interval::point(direction.x) * parameter,
        Interval::point(origin.y) + Interval::point(direction.y) * parameter,
    ]
}

fn unique_period_shift(target: Interval, source: Interval) -> Option<i64> {
    let candidates = (target - source).checked_div(Interval::point(PERIOD))?;
    let lower = candidates.lo().ceil();
    let upper = candidates.hi().floor();
    if !valid_unique_integer(lower, upper) {
        return None;
    }
    let shift = lower as i64;
    (source + integer_period_interval(shift)?)
        .intersects(target)
        .then_some(shift)
}

fn shift_fragment(fragment: &mut LiftedFragment, shift: i64) {
    let delta = integer_period_interval(shift).expect("certified period shift stays in range");
    for endpoint in &mut fragment.endpoints {
        endpoint.uv[0] = endpoint.uv[0] + delta;
    }
    fragment.origin[0] = fragment.origin[0] + delta;
    fragment.shift += shift;
}

fn valid_unique_integer(lower: f64, upper: f64) -> bool {
    lower.is_finite()
        && lower == upper
        && lower.abs() <= (1_u64 << 53) as f64
        && lower.abs() <= i64::MAX as f64
}

fn integer_period_interval(shift: i64) -> Option<Interval> {
    (shift.unsigned_abs() <= (1_u64 << 53))
        .then(|| Interval::point(shift as f64) * Interval::point(PERIOD))
}

fn fragment_bounds(fragment: &LiftedFragment) -> Bounds2 {
    Bounds2 {
        u: hull(fragment.endpoints[0].uv[0], fragment.endpoints[1].uv[0]),
        v: hull(fragment.endpoints[0].uv[1], fragment.endpoints[1].uv[1]),
    }
}

fn strictly_disjoint(first: Bounds2, second: Bounds2) -> bool {
    first.u.hi() < second.u.lo()
        || second.u.hi() < first.u.lo()
        || first.v.hi() < second.v.lo()
        || second.v.hi() < first.v.lo()
}

fn certify_simple_cycle(
    component: usize,
    fragments: &[LiftedFragment],
) -> Result<(), SectionPeriodicEmbeddingGap> {
    for first in 0..fragments.len() {
        for second in (first + 1)..fragments.len() {
            let adjacent = second == first + 1 || (first == 0 && second + 1 == fragments.len());
            if adjacent {
                let (left_end, right_start) = if second == first + 1 {
                    (
                        fragments[first].endpoints[1],
                        fragments[second].endpoints[0],
                    )
                } else {
                    (
                        fragments[second].endpoints[1],
                        fragments[first].endpoints[0],
                    )
                };
                if !carriers_cross_at_shared_endpoint(
                    &fragments[first],
                    &fragments[second],
                    left_end,
                    right_start,
                ) {
                    return Err(SectionPeriodicEmbeddingGap::SelfIntersectionProofRequired {
                        component,
                    });
                }
                continue;
            }
            if !strictly_disjoint(
                fragment_bounds(&fragments[first]),
                fragment_bounds(&fragments[second]),
            ) {
                return Err(SectionPeriodicEmbeddingGap::SelfIntersectionProofRequired {
                    component,
                });
            }
        }
    }
    Ok(())
}

fn carriers_cross_at_shared_endpoint(
    first: &LiftedFragment,
    second: &LiftedFragment,
    first_endpoint: EndpointBox,
    second_endpoint: EndpointBox,
) -> bool {
    if first_endpoint.endpoint != second_endpoint.endpoint {
        return false;
    }
    let first_direction = first.direction.map(Interval::point);
    let second_direction = second.direction.map(Interval::point);
    let denominator = cross2(first_direction, second_direction);
    if denominator.contains_zero() {
        return false;
    }
    let delta = [
        second.origin[0] - first.origin[0],
        second.origin[1] - first.origin[1],
    ];
    let Some(first_parameter) = cross2(delta, second_direction).checked_div(denominator) else {
        return false;
    };
    let Some(second_parameter) = cross2(delta, first_direction).checked_div(denominator) else {
        return false;
    };
    let first_root = [
        first.origin[0] + first_direction[0] * first_parameter,
        first.origin[1] + first_direction[1] * first_parameter,
    ];
    let second_root = [
        second.origin[0] + second_direction[0] * second_parameter,
        second.origin[1] + second_direction[1] * second_parameter,
    ];
    // The exact endpoint identity is the existence/equality authority: both
    // published pcurves carry that same section endpoint, and component lift
    // construction already selected one unique aligned cylinder chart for
    // it. A nonzero determinant proves the two infinite carriers have only
    // one common point, so their bounded fragments can meet only at that
    // topology-owned endpoint. The independently solved root enclosures are
    // consistency guards. They need only intersect the endpoint enclosures;
    // requiring containment between two independently outward-rounded
    // enclosures would reject a valid shared root by a few ulps.
    (0..2).all(|axis| {
        first_endpoint.uv[axis].intersects(second_endpoint.uv[axis])
            && first_endpoint.uv[axis].intersects(first_root[axis])
            && second_endpoint.uv[axis].intersects(second_root[axis])
            && first_root[axis].intersects(second_root[axis])
    })
}

fn cross2(first: [Interval; 2], second: [Interval; 2]) -> Interval {
    first[0] * second[1] - first[1] * second[0]
}

fn cycle_orientation(
    component: usize,
    fragments: &[LiftedFragment],
) -> Result<SectionPeriodicCycleOrientation, SectionPeriodicEmbeddingGap> {
    let mut twice_area = Interval::point(0.0);
    for fragment in fragments {
        let [start, end] = fragment.endpoints;
        twice_area = twice_area + start.uv[0] * end.uv[1] - start.uv[1] * end.uv[0];
    }
    if twice_area.lo() > 0.0 {
        Ok(SectionPeriodicCycleOrientation::Counterclockwise)
    } else if twice_area.hi() < 0.0 {
        Ok(SectionPeriodicCycleOrientation::Clockwise)
    } else {
        Err(SectionPeriodicEmbeddingGap::OrientationIndeterminate { component })
    }
}

fn component_bounds(component: &SectionPeriodicComponentEmbedding) -> Bounds2 {
    let mut fragments = component.fragments.iter();
    let first = fragments.next().expect("certified component is nonempty");
    let mut bounds = public_fragment_bounds(first);
    for fragment in fragments {
        let next = public_fragment_bounds(fragment);
        bounds.u = hull(bounds.u, next.u);
        bounds.v = hull(bounds.v, next.v);
    }
    bounds
}

fn public_fragment_bounds(fragment: &SectionPeriodicFragmentEmbedding) -> Bounds2 {
    let endpoints = fragment.endpoints();
    let start_u = Interval::new(endpoints[0][0].lo(), endpoints[0][0].hi());
    let end_u = Interval::new(endpoints[1][0].lo(), endpoints[1][0].hi());
    let start_v = Interval::new(endpoints[0][1].lo(), endpoints[0][1].hi());
    let end_v = Interval::new(endpoints[1][1].lo(), endpoints[1][1].hi());
    Bounds2 {
        u: hull(start_u, end_u),
        v: hull(start_v, end_v),
    }
}

fn hull(first: Interval, second: Interval) -> Interval {
    Interval::new(first.lo().min(second.lo()), first.hi().max(second.hi()))
}

fn certify_component_separation(
    components: &[SectionPeriodicComponentEmbedding],
) -> Result<(), SectionPeriodicEmbeddingGap> {
    for first in 0..components.len() {
        for second in (first + 1)..components.len() {
            let first_bounds = component_bounds(&components[first]);
            let second_bounds = component_bounds(&components[second]);
            if strictly_disjoint(first_bounds, second_bounds) {
                continue;
            }
            for left in &components[first].fragments {
                for right in &components[second].fragments {
                    if !strictly_disjoint(
                        public_fragment_bounds(left),
                        public_fragment_bounds(right),
                    ) {
                        return Err(
                            SectionPeriodicEmbeddingGap::ComponentIntersectionProofRequired {
                                first: components[first].component,
                                second: components[second].component,
                            },
                        );
                    }
                }
            }
            return Err(
                SectionPeriodicEmbeddingGap::ContainmentClassificationRequired {
                    first: components[first].component,
                    second: components[second].component,
                },
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::vec::{Point2, Vec3};

    use super::*;

    fn point(value: f64) -> SectionUvParameterInterval {
        SectionUvParameterInterval {
            lo: value,
            hi: value,
        }
    }

    fn public_segment(
        fragment: usize,
        start: [f64; 2],
        end: [f64; 2],
    ) -> SectionPeriodicFragmentEmbedding {
        let trim_scalar = |endpoint, uv: [f64; 2]| SectionCarrierTrimScalarEvidence {
            endpoint,
            carrier_parameter: 0.0,
            carrier_interval: SectionCarrierParameterInterval { lo: 0.0, hi: 0.0 },
            point: Point3::new(0.0, 0.0, 0.0),
            lifted_uv: [point(uv[0]), point(uv[1])],
        };
        SectionPeriodicFragmentEmbedding {
            fragment,
            endpoints: [
                [point(start[0]), point(start[1])],
                [point(end[0]), point(end[1])],
            ],
            period_shift: 0,
            trim_scalars: [trim_scalar(fragment, start), trim_scalar(fragment + 1, end)],
        }
    }

    fn test_line_fragment(
        fragment: usize,
        parameters: [Interval; 2],
        representatives: [f64; 2],
    ) -> LiftedFragment {
        let pcurve = SectionUvLine {
            origin: Point2::new(-2.0, 3.0),
            direction: Point2::new(2.0, -0.5),
        };
        LiftedFragment {
            fragment,
            endpoints: [
                EndpointBox {
                    endpoint: 20,
                    uv: map_line_interval(pcurve, parameters[0]),
                },
                EndpointBox {
                    endpoint: 21,
                    uv: map_line_interval(pcurve, parameters[1]),
                },
            ],
            shift: 0,
            direction: [pcurve.direction().x, pcurve.direction().y],
            origin: [
                Interval::point(pcurve.origin().x),
                Interval::point(pcurve.origin().y),
            ],
            carrier: SectionCarrier::Line {
                origin: Point3::new(1.0, -2.0, 4.0),
                direction: Vec3::new(0.25, 0.5, -1.0),
            },
            pcurve,
            parameters,
            representatives,
        }
    }

    fn rectangle(
        component: usize,
        lo: [f64; 2],
        hi: [f64; 2],
    ) -> SectionPeriodicComponentEmbedding {
        let points = [
            [lo[0], lo[1]],
            [hi[0], lo[1]],
            [hi[0], hi[1]],
            [lo[0], hi[1]],
        ];
        SectionPeriodicComponentEmbedding {
            component,
            fragments: (0..4)
                .map(|index| public_segment(index, points[index], points[(index + 1) % 4]))
                .collect(),
            winding: 0,
            orientation: SectionPeriodicCycleOrientation::Counterclockwise,
            parent: None,
        }
    }

    #[test]
    fn period_shift_requires_one_interval_owned_integer() {
        let target = Interval::new(PERIOD + 0.25, PERIOD + 0.5);
        let source = Interval::new(0.25, 0.5);
        assert_eq!(unique_period_shift(target, source), Some(1));
        assert_eq!(
            unique_period_shift(Interval::new(0.0, PERIOD), Interval::point(0.0)),
            None
        );
        assert_eq!(
            select_trim_scalar(0.0, Interval::new(-0.25, PERIOD + 0.25), true),
            None
        );
    }

    #[test]
    fn carrier_trim_scalar_is_enclosed_and_bit_exact_to_carrier_evaluation() {
        let fragment = test_line_fragment(
            17,
            [Interval::new(0.24, 0.26), Interval::new(0.74, 0.76)],
            [0.25, 0.75],
        );
        let evidence = certify_fragment_trim_scalars(&fragment).unwrap();
        let SectionCarrier::Line { origin, direction } = fragment.carrier else {
            unreachable!()
        };
        for (end, trim) in evidence.iter().enumerate() {
            let scalar = trim.carrier_parameter();
            let interval = trim.carrier_interval();
            assert!(interval.lo() <= scalar && scalar <= interval.hi());
            let expected = origin + direction * scalar;
            assert_eq!(
                [
                    trim.point().x.to_bits(),
                    trim.point().y.to_bits(),
                    trim.point().z.to_bits(),
                ],
                [
                    expected.x.to_bits(),
                    expected.y.to_bits(),
                    expected.z.to_bits(),
                ]
            );
            for axis in 0..2 {
                let endpoint = fragment.endpoints[end].uv[axis];
                let scalar_uv = trim.lifted_uv()[axis];
                assert!(endpoint.lo() <= scalar_uv.lo());
                assert!(scalar_uv.hi() <= endpoint.hi());
            }
        }
    }

    #[test]
    fn periodic_circle_representative_is_uniquely_lifted_before_evaluation() {
        let representative = 0.25;
        let lifted = representative + PERIOD;
        let pcurve = SectionUvLine {
            origin: Point2::new(0.0, 2.0),
            direction: Point2::new(1.0, 0.0),
        };
        let parameters = [
            Interval::new(0.24, 0.26),
            Interval::new(lifted - 1.0e-12, lifted + 1.0e-12),
        ];
        let carrier = SectionCarrier::Circle {
            center: Point3::new(1.0, 2.0, 3.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            x_direction: Vec3::new(1.0, 0.0, 0.0),
            radius: 2.0,
        };
        let fragment = LiftedFragment {
            fragment: 31,
            endpoints: [
                EndpointBox {
                    endpoint: 40,
                    uv: map_line_interval(pcurve, parameters[0]),
                },
                EndpointBox {
                    endpoint: 41,
                    uv: map_line_interval(pcurve, parameters[1]),
                },
            ],
            shift: 0,
            direction: [1.0, 0.0],
            origin: [Interval::point(0.0), Interval::point(2.0)],
            carrier,
            pcurve,
            parameters,
            representatives: [representative, representative],
        };
        let evidence = certify_fragment_trim_scalars(&fragment).unwrap();
        assert_eq!(evidence[1].carrier_parameter().to_bits(), lifted.to_bits());
        let SectionCarrier::Circle {
            center,
            normal,
            x_direction,
            radius,
        } = carrier
        else {
            unreachable!()
        };
        let (sin, cos) = math::sincos(lifted);
        let expected =
            center + x_direction * (radius * cos) + normal.cross(x_direction) * (radius * sin);
        assert_eq!(
            [
                evidence[1].point().x.to_bits(),
                evidence[1].point().y.to_bits(),
                evidence[1].point().z.to_bits(),
            ],
            [
                expected.x.to_bits(),
                expected.y.to_bits(),
                expected.z.to_bits(),
            ]
        );
    }

    #[test]
    fn tampered_scalar_and_lifted_pcurve_box_fail_closed() {
        let mut fragment = test_line_fragment(
            52,
            [Interval::new(0.24, 0.26), Interval::new(0.74, 0.76)],
            [2.0, 0.75],
        );
        assert_eq!(
            certify_fragment_trim_scalars(&fragment),
            Err(SectionPeriodicEmbeddingGap::CarrierTrimScalarUnavailable {
                fragment: 52,
                endpoint: 20,
            })
        );

        fragment.representatives[0] = 0.25;
        fragment.endpoints[0].uv[0] = Interval::new(99.0, 100.0);
        assert_eq!(
            certify_fragment_trim_scalars(&fragment),
            Err(
                SectionPeriodicEmbeddingGap::CarrierTrimScalarPcurveMismatch {
                    fragment: 52,
                    endpoint: 20,
                }
            )
        );
    }

    #[test]
    fn pair_precharge_accepts_n_refuses_n_minus_one_and_checks_overflow() {
        let components = [
            SectionCurveComponent {
                fragments: vec![0, 1, 2, 3],
                closed: true,
            },
            SectionCurveComponent {
                fragments: vec![4, 5, 6, 7],
                closed: true,
            },
        ];
        // C(8, 2) fragment-pair candidates plus C(2, 2) component bounds.
        let exact = periodic_pair_candidate_work(&components, 1).unwrap();
        assert_eq!(exact, 29);
        assert_eq!(unordered_pairs(u64::MAX), None);

        let policy = SessionPolicy::v1();
        let tolerances = Tolerances::default();
        let run = |allowed| {
            let overrides = BudgetPlan::new([LimitSpec::new(
                SECTION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap();
            let context = OperationContext::new(&policy, tolerances)
                .unwrap()
                .with_family_budget_defaults(super::super::BodySectionBudgetProfile::v1_defaults())
                .with_budget_overrides(overrides);
            let mut scope = OperationScope::new(&context);
            charge_pair_candidates(&components, 1, &mut scope)
        };
        assert!(run(exact).unwrap());
        let error = run(exact - 1).unwrap_err();
        let crossing = error.limit().expect("N-1 must retain limit evidence");
        assert_eq!(crossing.stage, SECTION_WORK);
        assert_eq!(crossing.resource, ResourceKind::Work);
        assert_eq!(crossing.consumed, exact);
        assert_eq!(crossing.allowed, exact - 1);
    }

    #[test]
    fn nonadjacent_box_overlap_refuses_simple_cycle_proof() {
        let fragment = |key: usize, start: [f64; 2], end: [f64; 2]| LiftedFragment {
            fragment: key,
            endpoints: [
                EndpointBox {
                    endpoint: key,
                    uv: [Interval::point(start[0]), Interval::point(start[1])],
                },
                EndpointBox {
                    endpoint: key + 1,
                    uv: [Interval::point(end[0]), Interval::point(end[1])],
                },
            ],
            shift: 0,
            direction: [end[0] - start[0], end[1] - start[1]],
            origin: [Interval::point(start[0]), Interval::point(start[1])],
            carrier: SectionCarrier::Line {
                origin: Point3::new(0.0, 0.0, 0.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
            },
            pcurve: SectionUvLine {
                origin: Point2::new(start[0], start[1]),
                direction: Point2::new(end[0] - start[0], end[1] - start[1]),
            },
            parameters: [Interval::point(0.0), Interval::point(1.0)],
            representatives: [0.0, 1.0],
        };
        let crossing = vec![
            fragment(0, [0.0, 0.0], [2.0, 2.0]),
            fragment(1, [2.0, 2.0], [0.0, 2.0]),
            fragment(2, [0.0, 2.0], [2.0, 0.0]),
            fragment(3, [2.0, 0.0], [0.0, 0.0]),
        ];
        assert_eq!(
            certify_simple_cycle(7, &crossing),
            Err(SectionPeriodicEmbeddingGap::SelfIntersectionProofRequired { component: 7 })
        );
    }

    #[test]
    fn adjacent_collinear_overlap_remains_a_typed_refusal() {
        let fragment = |key: usize, start: [f64; 2], end: [f64; 2]| LiftedFragment {
            fragment: key,
            endpoints: [
                EndpointBox {
                    endpoint: key,
                    uv: [Interval::point(start[0]), Interval::point(start[1])],
                },
                EndpointBox {
                    endpoint: key + 1,
                    uv: [Interval::point(end[0]), Interval::point(end[1])],
                },
            ],
            shift: 0,
            direction: [end[0] - start[0], end[1] - start[1]],
            origin: [Interval::point(start[0]), Interval::point(start[1])],
            carrier: SectionCarrier::Line {
                origin: Point3::new(0.0, 0.0, 0.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
            },
            pcurve: SectionUvLine {
                origin: Point2::new(start[0], start[1]),
                direction: Point2::new(end[0] - start[0], end[1] - start[1]),
            },
            parameters: [Interval::point(0.0), Interval::point(1.0)],
            representatives: [0.0, 1.0],
        };
        let overlapping = vec![
            fragment(0, [0.0, 0.0], [2.0, 0.0]),
            fragment(1, [2.0, 0.0], [1.0, 0.0]),
            fragment(2, [1.0, 0.0], [0.0, 0.0]),
        ];
        assert_eq!(
            certify_simple_cycle(11, &overlapping),
            Err(SectionPeriodicEmbeddingGap::SelfIntersectionProofRequired { component: 11 })
        );
    }

    #[test]
    fn nesting_is_a_typed_missing_obligation_not_a_sampled_guess() {
        let outer = rectangle(2, [0.0, 0.0], [4.0, 4.0]);
        let inner = rectangle(9, [1.0, 1.0], [2.0, 2.0]);
        assert_eq!(
            certify_component_separation(&[outer, inner]),
            Err(
                SectionPeriodicEmbeddingGap::ContainmentClassificationRequired {
                    first: 2,
                    second: 9
                }
            )
        );
    }
}
