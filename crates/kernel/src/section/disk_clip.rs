//! Proof-keyed clipping of a vertexless circular cap by one opposing Plane.
//!
//! A finite-cylinder cap is a disk whose sole boundary is a vertexless,
//! whole-period circle edge.  A transverse Plane cuts that disk in one chord.
//! This module binds the already-certified Plane/Plane carrier to the complete
//! circle/Plane root order owned by [`RootIdentityAuthority`].  Endpoint
//! identity is therefore `(source edge, intrinsic root ordinal)`, never a
//! rounded point comparison.  Carrier parameters and model points are retained
//! only as publication evidence after the two source roots are certified.
//!
//! The theorem is independent of portal, face, or layout counts: one proven
//! disk boundary and one opposing Plane produce either one oriented chord or
//! one typed fail-closed gap.  Tangency, coincidence, parameter-seam contact,
//! an empty disk cut, malformed cap topology, and overlapping projected root
//! enclosures are never promoted to section fragments.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kgeom::curve::Circle;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec3};
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, LoopId as RawLoopId, Sense,
};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::clip::SectionCarrierLine;
use super::root_identity::{
    CertifiedSourceRootScalar, RootIdentityAuthority, RootIdentityGap, RootOrderOutcome,
    SourceRootKey, SourceRootQuery,
};
use super::{SECTION_WORK, SectionCarrier, SectionUvLine};
use crate::error::{Error, Result};

/// Plane-pair evidence already certified by Section's graph intersection.
///
/// The parent section pipeline constructs this directly from `PairCarrier`.
/// Keeping the adapter here avoids exposing `PairCarrier` outside its owning
/// module while retaining the exact carrier, paired pcurves, and residuals
/// needed to publish the resulting chord.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct DiskCapPlanePairEvidence {
    carrier: SectionCarrierLine,
    uv_lines: [SectionUvLine; 2],
    residual_bounds: [f64; 2],
}

impl DiskCapPlanePairEvidence {
    pub(super) const fn new(
        carrier: SectionCarrierLine,
        uv_lines: [SectionUvLine; 2],
        residual_bounds: [f64; 2],
    ) -> Self {
        Self {
            carrier,
            uv_lines,
            residual_bounds,
        }
    }
}

/// Stable semantic refusals from the disk-cap theorem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum DiskCapClipGap {
    InvalidCapOperand,
    UnsupportedCapBoundary,
    UnsupportedPlanePair,
    ArithmeticGuard,
    CarrierOrientation,
    TangentialContact,
    CoincidentGeometry,
    ParameterSeamContact,
    EmptyIntersection,
    UnexpectedRootCount,
    UnorderedChordEndpoints,
    RootIdentity(RootIdentityGap),
}

impl DiskCapClipGap {
    /// Stable diagnostic suitable for a parent [`super::SectionGap`].
    pub(super) const fn reason(self) -> &'static str {
        match self {
            Self::InvalidCapOperand => "disk-cap clipping received an invalid cap operand slot",
            Self::UnsupportedCapBoundary => {
                "disk-cap clipping requires one source-provenanced vertexless whole-circle boundary"
            }
            Self::UnsupportedPlanePair => {
                "disk-cap clipping requires a planar cap and an opposing Plane"
            }
            Self::ArithmeticGuard => {
                "disk-cap clipping could not retain finite certified arithmetic evidence"
            }
            Self::CarrierOrientation => {
                "disk-cap chord carrier does not have the certified operand-order orientation"
            }
            Self::TangentialContact => "the opposing Plane is tangent to the circular cap boundary",
            Self::CoincidentGeometry => {
                "the opposing Plane is coincident with the circular cap boundary plane"
            }
            Self::ParameterSeamContact => {
                "a disk-cap chord endpoint lies on the source circle parameter seam"
            }
            Self::EmptyIntersection => {
                "the opposing Plane has no intersection with the closed disk cap"
            }
            Self::UnexpectedRootCount => {
                "the disk-cap boundary did not have exactly two certified transverse roots"
            }
            Self::UnorderedChordEndpoints => {
                "disk-cap root endpoints could not be strictly ordered on the chord carrier"
            }
            Self::RootIdentity(gap) => gap.reason(),
        }
    }
}

/// One source-root endpoint ordered in the chord carrier direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CertifiedDiskCapEndpoint {
    root: SourceRootKey,
    source_root_scalar: CertifiedSourceRootScalar,
    source_parameter: Interval,
    carrier_parameter: Interval,
    carrier_parameter_representative: f64,
    point: Point3,
    cap_loop: RawLoopId,
    cap_fin: RawFinId,
}

impl CertifiedDiskCapEndpoint {
    pub(super) const fn root(self) -> SourceRootKey {
        self.root
    }

    pub(super) const fn source_root_scalar(self) -> CertifiedSourceRootScalar {
        self.source_root_scalar
    }

    pub(super) const fn source_parameter(self) -> Interval {
        self.source_parameter
    }

    pub(super) const fn carrier_parameter(self) -> Interval {
        self.carrier_parameter
    }

    pub(super) const fn carrier_parameter_representative(self) -> f64 {
        self.carrier_parameter_representative
    }

    pub(super) const fn point(self) -> Point3 {
        self.point
    }

    pub(super) const fn cap_loop(self) -> RawLoopId {
        self.cap_loop
    }

    pub(super) const fn cap_fin(self) -> RawFinId {
        self.cap_fin
    }
}

/// One canonically oriented, proof-keyed chord across a circular cap disk.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct CertifiedDiskCapChord {
    faces: [RawFaceId; 2],
    cap_operand: usize,
    boundary_edge: RawEdgeId,
    carrier: SectionCarrier,
    range: ParamRange,
    uv_lines: [SectionUvLine; 2],
    residual_bounds: [f64; 2],
    endpoints: [CertifiedDiskCapEndpoint; 2],
}

impl CertifiedDiskCapChord {
    pub(super) const fn faces(&self) -> &[RawFaceId; 2] {
        &self.faces
    }

    pub(super) const fn cap_operand(&self) -> usize {
        self.cap_operand
    }

    pub(super) const fn boundary_edge(&self) -> RawEdgeId {
        self.boundary_edge
    }

    pub(super) const fn carrier(&self) -> SectionCarrier {
        self.carrier
    }

    pub(super) const fn range(&self) -> ParamRange {
        self.range
    }

    pub(super) const fn uv_lines(&self) -> &[SectionUvLine; 2] {
        &self.uv_lines
    }

    pub(super) const fn residual_bounds(&self) -> [f64; 2] {
        self.residual_bounds
    }

    pub(super) const fn endpoints(&self) -> &[CertifiedDiskCapEndpoint; 2] {
        &self.endpoints
    }
}

/// Fail-closed result of clipping one circular cap disk.
// The certified chord stays inline so the clip outcome hands the certificate
// off by value without indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub(super) enum DiskCapClipOutcome {
    Chord(CertifiedDiskCapChord),
    Indeterminate(DiskCapClipGap),
}

/// Read-only admission evidence for one exact circular disk cap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CertifiedDiskCapAdmission {
    face: RawFaceId,
    boundary_edge: RawEdgeId,
    plane: Plane,
}

impl CertifiedDiskCapAdmission {
    pub(super) const fn face(self) -> RawFaceId {
        self.face
    }

    pub(super) const fn boundary_edge(self) -> RawEdgeId {
        self.boundary_edge
    }

    pub(super) const fn plane(self) -> Plane {
        self.plane
    }
}

#[derive(Debug, Clone, Copy)]
struct CapBoundary {
    loop_id: RawLoopId,
    fin: RawFinId,
    circle: Circle,
    plane: Plane,
}

/// Admit one planar face as a source-provenanced circular disk cap.
///
/// The scan is read-only. It returns the authored Plane and the sole
/// vertexless whole-circle boundary edge after delegating the actual topology
/// and whole-fin-incidence theorem to [`cap_boundary`].
pub(super) fn admit_disk_cap(
    store: &Store,
    face: RawFaceId,
    scope: &OperationScope<'_, '_>,
) -> Result<core::result::Result<CertifiedDiskCapAdmission, DiskCapClipGap>> {
    let face_data = read(store.get(face))?;
    if !matches!(
        read(store.surface(face_data.surface()))?,
        SurfaceGeom::Plane(_)
    ) {
        return Ok(Err(DiskCapClipGap::UnsupportedPlanePair));
    }
    let [loop_id] = face_data.loops() else {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    };
    let loop_ = read(store.get(*loop_id))?;
    let [fin_id] = loop_.fins() else {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    };
    let edge = read(store.get(*fin_id))?.edge();
    let boundary = match cap_boundary(store, face, edge, scope)? {
        Ok(boundary) => boundary,
        Err(gap) => return Ok(Err(gap)),
    };
    Ok(Ok(CertifiedDiskCapAdmission {
        face,
        boundary_edge: edge,
        plane: boundary.plane,
    }))
}

/// Clip one vertexless whole-circle cap boundary by an opposing Plane.
///
/// `faces` retains section operand order and `cap_operand` selects the disk.
/// `pair` must be the certified, canonically oriented Plane/Plane carrier for
/// that exact face pair.  Resource failures propagate as operation errors;
/// every geometric or representation uncertainty is a typed semantic gap.
pub(super) fn clip_disk_cap(
    store: &Store,
    faces: [RawFaceId; 2],
    cap_operand: usize,
    boundary_edge: RawEdgeId,
    pair: DiskCapPlanePairEvidence,
    roots: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
) -> Result<DiskCapClipOutcome> {
    charge(scope, 1)?;
    if cap_operand >= faces.len() {
        return Ok(DiskCapClipOutcome::Indeterminate(
            DiskCapClipGap::InvalidCapOperand,
        ));
    }
    let cap_face = faces[cap_operand];
    let opposing_face = faces[1 - cap_operand];
    let boundary = match cap_boundary(store, cap_face, boundary_edge, scope)? {
        Ok(boundary) => boundary,
        Err(gap) => return Ok(DiskCapClipOutcome::Indeterminate(gap)),
    };
    if !plane_pair(store, faces)? {
        return Ok(DiskCapClipOutcome::Indeterminate(
            DiskCapClipGap::UnsupportedPlanePair,
        ));
    }

    let query = SourceRootQuery::new(boundary_edge, opposing_face);
    let order = match roots.certify_order(store, query, scope)? {
        RootOrderOutcome::Certified(order) => order,
        RootOrderOutcome::Indeterminate(gap) => {
            return Ok(DiskCapClipOutcome::Indeterminate(map_root_gap(gap)));
        }
    };
    match order.roots().len() {
        0 => {
            return Ok(DiskCapClipOutcome::Indeterminate(
                DiskCapClipGap::EmptyIntersection,
            ));
        }
        2 => {}
        _ => {
            return Ok(DiskCapClipOutcome::Indeterminate(
                DiskCapClipGap::UnexpectedRootCount,
            ));
        }
    }
    if let Err(gap) = validate_pair_evidence(store, faces, pair) {
        return Ok(DiskCapClipOutcome::Indeterminate(gap));
    }

    let mut endpoints = Vec::with_capacity(2);
    for ordinal in 0..2 {
        charge(scope, 1)?;
        let root = SourceRootKey::new(boundary_edge, ordinal);
        let Some(source_root_scalar) = order.materialize(root) else {
            return Ok(DiskCapClipOutcome::Indeterminate(
                DiskCapClipGap::ArithmeticGuard,
            ));
        };
        let source_parameter = order.roots()[ordinal];
        let Some(carrier_parameter) =
            project_circle_root(boundary.circle, source_parameter, pair.carrier)
        else {
            return Ok(DiskCapClipOutcome::Indeterminate(
                DiskCapClipGap::ArithmeticGuard,
            ));
        };
        let Some(carrier_parameter_representative) = midpoint(carrier_parameter) else {
            return Ok(DiskCapClipOutcome::Indeterminate(
                DiskCapClipGap::ArithmeticGuard,
            ));
        };
        let Some(point) = carrier_point(pair.carrier, carrier_parameter_representative) else {
            return Ok(DiskCapClipOutcome::Indeterminate(
                DiskCapClipGap::ArithmeticGuard,
            ));
        };
        endpoints.push(CertifiedDiskCapEndpoint {
            root,
            source_root_scalar,
            source_parameter,
            carrier_parameter,
            carrier_parameter_representative,
            point,
            cap_loop: boundary.loop_id,
            cap_fin: boundary.fin,
        });
    }
    endpoints.sort_by(|a, b| {
        a.carrier_parameter
            .lo()
            .total_cmp(&b.carrier_parameter.lo())
            .then(
                a.carrier_parameter
                    .hi()
                    .total_cmp(&b.carrier_parameter.hi()),
            )
    });
    let [start, end]: [CertifiedDiskCapEndpoint; 2] = endpoints
        .try_into()
        .expect("exactly two disk-cap endpoints were constructed");
    if start.carrier_parameter.hi() >= end.carrier_parameter.lo()
        || start.carrier_parameter_representative >= end.carrier_parameter_representative
    {
        return Ok(DiskCapClipOutcome::Indeterminate(
            DiskCapClipGap::UnorderedChordEndpoints,
        ));
    }

    Ok(DiskCapClipOutcome::Chord(CertifiedDiskCapChord {
        faces,
        cap_operand,
        boundary_edge,
        carrier: SectionCarrier::Line {
            origin: Point3::new(
                pair.carrier.origin[0],
                pair.carrier.origin[1],
                pair.carrier.origin[2],
            ),
            direction: Vec3::new(
                pair.carrier.direction[0],
                pair.carrier.direction[1],
                pair.carrier.direction[2],
            ),
        },
        range: ParamRange::new(
            start.carrier_parameter_representative,
            end.carrier_parameter_representative,
        ),
        uv_lines: pair.uv_lines,
        residual_bounds: pair.residual_bounds,
        endpoints: [start, end],
    }))
}

fn cap_boundary(
    store: &Store,
    cap_face: RawFaceId,
    boundary_edge: RawEdgeId,
    scope: &OperationScope<'_, '_>,
) -> Result<core::result::Result<CapBoundary, DiskCapClipGap>> {
    let face = read(store.get(cap_face))?;
    let SurfaceGeom::Plane(plane) = read(store.surface(face.surface()))? else {
        return Ok(Err(DiskCapClipGap::UnsupportedPlanePair));
    };
    let [loop_id] = face.loops() else {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    };
    let loop_ = read(store.get(*loop_id))?;
    let [fin_id] = loop_.fins() else {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    };
    let fin = read(store.get(*fin_id))?;
    if loop_.face() != cap_face
        || fin.parent() != *loop_id
        || fin.edge() != boundary_edge
        || certify_whole_fin_incidence(
            store,
            cap_face,
            *loop_id,
            *fin_id,
            scope.context().tolerances().linear(),
        ) != WholeFinIncidence::Certified
    {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    }
    let edge = read(store.get(boundary_edge))?;
    if edge.tolerance().is_some()
        || edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || !edge.fins().contains(fin_id)
    {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    }
    let Some(curve) = edge.curve() else {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    };
    let CurveGeom::Circle(circle) = read(store.curve(curve))? else {
        return Ok(Err(DiskCapClipGap::UnsupportedCapBoundary));
    };
    Ok(Ok(CapBoundary {
        loop_id: *loop_id,
        fin: *fin_id,
        circle: *circle,
        plane: *plane,
    }))
}

fn plane_pair(store: &Store, faces: [RawFaceId; 2]) -> Result<bool> {
    for face in faces {
        let face = read(store.get(face))?;
        if !matches!(read(store.surface(face.surface()))?, SurfaceGeom::Plane(_)) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn validate_pair_evidence(
    store: &Store,
    faces: [RawFaceId; 2],
    pair: DiskCapPlanePairEvidence,
) -> core::result::Result<(), DiskCapClipGap> {
    let values = pair
        .carrier
        .origin
        .into_iter()
        .chain(pair.carrier.direction)
        .chain(pair.residual_bounds)
        .chain(pair.uv_lines.iter().flat_map(|line| {
            [
                line.origin().x,
                line.origin().y,
                line.direction().x,
                line.direction().y,
            ]
        }));
    if values.into_iter().any(|value| !value.is_finite())
        || pair.residual_bounds.into_iter().any(|value| value < 0.0)
    {
        return Err(DiskCapClipGap::ArithmeticGuard);
    }
    let direction = pair.carrier.direction.map(Interval::point);
    let Some(normal_a) = plane_outward_normal(store, faces[0]) else {
        return Err(DiskCapClipGap::UnsupportedPlanePair);
    };
    let Some(normal_b) = plane_outward_normal(store, faces[1]) else {
        return Err(DiskCapClipGap::UnsupportedPlanePair);
    };
    certified_positive_orientation(direction, normal_a, normal_b)
        .then_some(())
        .ok_or(DiskCapClipGap::CarrierOrientation)
}

fn plane_outward_normal(store: &Store, face: RawFaceId) -> Option<[Interval; 3]> {
    let face = store.get(face).ok()?;
    let SurfaceGeom::Plane(plane) = store.surface(face.surface()).ok()? else {
        return None;
    };
    let sign = if face.sense() == Sense::Forward {
        1.0
    } else {
        -1.0
    };
    Some((plane.frame().z() * sign).to_array().map(Interval::point))
}

fn certified_positive_orientation(
    direction: [Interval; 3],
    normal_a: [Interval; 3],
    normal_b: [Interval; 3],
) -> bool {
    let cross = [
        normal_a[1] * normal_b[2] - normal_a[2] * normal_b[1],
        normal_a[2] * normal_b[0] - normal_a[0] * normal_b[2],
        normal_a[0] * normal_b[1] - normal_a[1] * normal_b[0],
    ];
    dot(direction, cross).lo() > 0.0
}

fn project_circle_root(
    circle: Circle,
    root: Interval,
    carrier: SectionCarrierLine,
) -> Option<Interval> {
    let point = circle_point_enclosure(circle, root)?;
    let origin = carrier.origin.map(Interval::point);
    let direction = carrier.direction.map(Interval::point);
    let relative = core::array::from_fn(|axis| point[axis] - origin[axis]);
    dot(relative, direction).checked_div(dot(direction, direction))
}

fn circle_point_enclosure(circle: Circle, root: Interval) -> Option<[Interval; 3]> {
    if !finite(root) || root.lo() > root.hi() {
        return None;
    }
    let midpoint = midpoint(root)?;
    let delta = (root.hi() - root.lo()).next_up();
    let (sin, cos) = math::sincos(midpoint);
    if !delta.is_finite() || !sin.is_finite() || !cos.is_finite() {
        return None;
    }
    // Kernel deterministic trig is within one ulp; the full root width plus
    // outward endpoint rounding safely covers sin/cos over the enclosure.
    let sin = Interval::new(
        (sin.next_down() - delta).next_down(),
        (sin.next_up() + delta).next_up(),
    );
    let cos = Interval::new(
        (cos.next_down() - delta).next_down(),
        (cos.next_up() + delta).next_up(),
    );
    let frame = circle.frame();
    let center = frame.origin().to_array().map(Interval::point);
    let x = frame.x().to_array().map(Interval::point);
    let y = frame.y().to_array().map(Interval::point);
    let radius = Interval::point(circle.radius());
    Some(core::array::from_fn(|axis| {
        center[axis] + radius * (x[axis] * cos + y[axis] * sin)
    }))
}

fn carrier_point(carrier: SectionCarrierLine, parameter: f64) -> Option<Point3> {
    let point = Point3::new(
        carrier.origin[0] + carrier.direction[0] * parameter,
        carrier.origin[1] + carrier.direction[1] * parameter,
        carrier.origin[2] + carrier.direction[2] * parameter,
    );
    point
        .to_array()
        .into_iter()
        .all(f64::is_finite)
        .then_some(point)
}

fn midpoint(interval: Interval) -> Option<f64> {
    if !finite(interval) || interval.lo() > interval.hi() {
        return None;
    }
    let value = 0.5 * interval.lo() + 0.5 * interval.hi();
    value.is_finite().then_some(value)
}

fn dot(a: [Interval; 3], b: [Interval; 3]) -> Interval {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn map_root_gap(gap: RootIdentityGap) -> DiskCapClipGap {
    match gap {
        RootIdentityGap::TangentialOrUnresolvedMultiplicity => DiskCapClipGap::TangentialContact,
        RootIdentityGap::CoincidentGeometry => DiskCapClipGap::CoincidentGeometry,
        RootIdentityGap::ParameterSeamContact => DiskCapClipGap::ParameterSeamContact,
        other => DiskCapClipGap::RootIdentity(other),
    }
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)
}

fn read<T>(result: kcore::error::Result<T>) -> Result<T> {
    result.map_err(|source| Error::InconsistentTopology { source })
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::Point2;
    use ktopo::entity::BodyId as RawBodyId;

    use super::*;
    use crate::section::BodySectionBudgetProfile;

    #[derive(Debug, Clone, Copy)]
    struct PlaneCase {
        name: &'static str,
        origin: Point3,
        normal: Vec3,
        expected: DiskCapClipGap,
    }

    struct Fixture {
        store: Store,
        faces: [RawFaceId; 2],
        cap_operand: usize,
        boundary_edge: RawEdgeId,
        cap_loop: RawLoopId,
        cap_fin: RawFinId,
        pair: DiskCapPlanePairEvidence,
    }

    fn with_scope<T>(run: impl FnOnce(&mut OperationScope<'_, '_>) -> T) -> T {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        run(&mut scope)
    }

    fn disk_boundary(
        store: &Store,
        body: RawBodyId,
    ) -> (RawFaceId, RawLoopId, RawFinId, RawEdgeId) {
        let cap_face = store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|face_id| {
                let face = store.get(*face_id).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Plane(plane) if plane.frame().origin() == Point3::new(0.0, 0.0, 0.0)
                )
            })
            .expect("finite cylinder has a cap at the authored origin");
        let [loop_id] = store.get(cap_face).unwrap().loops() else {
            panic!("cylinder cap must have one loop")
        };
        let [fin_id] = store.get(*loop_id).unwrap().fins() else {
            panic!("cylinder cap must have one fin")
        };
        (
            cap_face,
            *loop_id,
            *fin_id,
            store.get(*fin_id).unwrap().edge(),
        )
    }

    fn fixture(origin: Point3, normal: Vec3, cap_operand: usize) -> Fixture {
        let mut store = Store::new();
        let cylinder = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let (cap_face, cap_loop, cap_fin, boundary_edge) = disk_boundary(&store, cylinder);
        let opposing_frame = Frame::from_z(origin, normal).unwrap();
        let sheet = ktopo::make::planar_sheet(
            &mut store,
            &opposing_frame,
            &[
                Point2::new(-3.0, -3.0),
                Point2::new(3.0, -3.0),
                Point2::new(3.0, 3.0),
                Point2::new(-3.0, 3.0),
            ],
        )
        .unwrap();
        let opposing_face = store.faces_of_body(sheet).unwrap()[0];
        let faces = if cap_operand == 0 {
            [cap_face, opposing_face]
        } else {
            [opposing_face, cap_face]
        };
        let normals = faces.map(|face| {
            let face = store.get(face).unwrap();
            let SurfaceGeom::Plane(plane) = store.surface(face.surface()).unwrap() else {
                unreachable!()
            };
            plane.frame().z()
                * if face.sense() == Sense::Forward {
                    1.0
                } else {
                    -1.0
                }
        });
        let direction = normals[0]
            .cross(normals[1])
            .normalized()
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
        let pair = DiskCapPlanePairEvidence::new(
            SectionCarrierLine {
                origin: origin.to_array(),
                direction: direction.to_array(),
            },
            [
                SectionUvLine {
                    origin: Point2::new(0.0, 0.0),
                    direction: Point2::new(1.0, 0.0),
                },
                SectionUvLine {
                    origin: Point2::new(0.0, 0.0),
                    direction: Point2::new(0.0, 1.0),
                },
            ],
            [0.0, 0.0],
        );
        Fixture {
            store,
            faces,
            cap_operand,
            boundary_edge,
            cap_loop,
            cap_fin,
            pair,
        }
    }

    #[test]
    fn semantic_gap_matrix_is_typed_and_fail_closed() {
        let cases = [
            PlaneCase {
                name: "tangent",
                origin: Point3::new(1.0, 0.0, 0.0),
                normal: Vec3::new(1.0, 0.0, 0.0),
                expected: DiskCapClipGap::TangentialContact,
            },
            PlaneCase {
                name: "empty",
                origin: Point3::new(2.0, 0.0, 0.0),
                normal: Vec3::new(1.0, 0.0, 0.0),
                expected: DiskCapClipGap::EmptyIntersection,
            },
            PlaneCase {
                name: "source_parameter_seam",
                origin: Point3::new(0.0, 0.0, 0.0),
                normal: Vec3::new(0.0, 1.0, 0.0),
                expected: DiskCapClipGap::ParameterSeamContact,
            },
            PlaneCase {
                name: "coincident",
                origin: Point3::new(0.0, 0.0, 0.0),
                normal: Vec3::new(0.0, 0.0, 1.0),
                expected: DiskCapClipGap::CoincidentGeometry,
            },
        ];

        for case in cases {
            let fixture = fixture(case.origin, case.normal, 0);
            let outcome = with_scope(|scope| {
                clip_disk_cap(
                    &fixture.store,
                    fixture.faces,
                    fixture.cap_operand,
                    fixture.boundary_edge,
                    fixture.pair,
                    &mut RootIdentityAuthority::new(),
                    scope,
                )
                .unwrap()
            });
            assert_eq!(
                outcome,
                DiskCapClipOutcome::Indeterminate(case.expected),
                "{}",
                case.name,
            );
        }
    }

    #[test]
    fn admission_refuses_polygonal_planes_and_nonplanar_faces() {
        let mut store = Store::new();
        let cylinder = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let side_face = store
            .faces_of_body(cylinder)
            .unwrap()
            .into_iter()
            .find(|face| {
                let face = store.get(*face).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let sheet = ktopo::make::planar_sheet(
            &mut store,
            &Frame::world(),
            &[
                Point2::new(-1.0, -1.0),
                Point2::new(1.0, -1.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        let polygonal_face = store.faces_of_body(sheet).unwrap()[0];
        let cases = [
            (
                "polygonal_plane",
                polygonal_face,
                DiskCapClipGap::UnsupportedCapBoundary,
            ),
            (
                "cylinder_side",
                side_face,
                DiskCapClipGap::UnsupportedPlanePair,
            ),
        ];

        for (name, face, expected) in cases {
            let outcome = with_scope(|scope| admit_disk_cap(&store, face, scope).unwrap());
            assert_eq!(outcome, Err(expected), "{name}");
        }
    }

    #[test]
    fn secant_chord_is_root_keyed_and_canonically_oriented_in_both_operand_orders() {
        for cap_operand in 0..2 {
            let fixture = fixture(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                cap_operand,
            );
            let chord = with_scope(|scope| {
                let outcome = clip_disk_cap(
                    &fixture.store,
                    fixture.faces,
                    fixture.cap_operand,
                    fixture.boundary_edge,
                    fixture.pair,
                    &mut RootIdentityAuthority::new(),
                    scope,
                )
                .unwrap();
                let DiskCapClipOutcome::Chord(chord) = outcome else {
                    panic!("diametral disk cut was not certified: {outcome:?}")
                };
                chord
            });

            assert_eq!(chord.faces(), &fixture.faces);
            assert_eq!(chord.cap_operand(), cap_operand);
            assert_eq!(chord.boundary_edge(), fixture.boundary_edge);
            assert!(chord.residual_bounds().iter().all(|value| *value == 0.0));
            assert!((chord.range().lo + 1.0).abs() <= 1.0e-14);
            assert!((chord.range().hi - 1.0).abs() <= 1.0e-14);

            let endpoints = chord.endpoints();
            assert!(endpoints[0].carrier_parameter().contains(-1.0));
            assert!(endpoints[1].carrier_parameter().contains(1.0));
            let expected_ordinals = if cap_operand == 0 { [0, 1] } else { [1, 0] };
            assert_eq!(
                [endpoints[0].root().ordinal(), endpoints[1].root().ordinal()],
                expected_ordinals,
            );
            for endpoint in endpoints {
                let exact_angle = match endpoint.root().ordinal() {
                    0 => core::f64::consts::FRAC_PI_2,
                    1 => 3.0 * core::f64::consts::FRAC_PI_2,
                    ordinal => panic!("unexpected source root ordinal {ordinal}"),
                };
                assert!(endpoint.source_parameter().contains(exact_angle));
                assert_eq!(
                    endpoint.source_root_scalar().enclosure(),
                    endpoint.source_parameter(),
                );
                assert!(
                    endpoint
                        .source_root_scalar()
                        .enclosure()
                        .contains(exact_angle)
                );
                assert_eq!(endpoint.cap_loop(), fixture.cap_loop);
                assert_eq!(endpoint.cap_fin(), fixture.cap_fin);
                let point = endpoint.point();
                assert!(point.x.abs() <= 1.0e-15);
                assert!(point.z.abs() <= 1.0e-15);
                assert!((point.y * point.y - 1.0).abs() <= 1.0e-14);
            }

            let SectionCarrier::Line { direction, .. } = chord.carrier() else {
                panic!("disk cap chord lost its line carrier")
            };
            let normals = fixture.faces.map(|face| {
                let face = fixture.store.get(face).unwrap();
                let SurfaceGeom::Plane(plane) = fixture.store.surface(face.surface()).unwrap()
                else {
                    unreachable!()
                };
                plane.frame().z()
                    * if face.sense() == Sense::Forward {
                        1.0
                    } else {
                        -1.0
                    }
            });
            assert!(direction.dot(normals[0].cross(normals[1])) > 0.0);
        }
    }
}
