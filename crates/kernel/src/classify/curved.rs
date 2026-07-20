//! Certified circular-plane and full-period cylindrical-face classification.
//!
//! This module is deliberately topology-driven.  It admits any circular
//! plane trim and any cylindrical trim whose boundary components prove that
//! they are full-period, constant-axial-parameter rings.  It does not inspect
//! face positions, body face counts, or primitive-constructor order.

use std::collections::HashSet;

use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::Circle;
use kgeom::frame::Frame;
use kgeom::surface::{Cylinder, Plane};
use ktopo::entity::{
    BodyId as RawBodyId, EdgeId as RawEdgeId, FaceId as RawFaceId, Loop, RegionKind,
};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::{RawSite, SiteOutcome, as_coords, charge, read};
use crate::error::Result;

pub(super) const GAP_CYLINDER_TRIM: &str =
    "cylindrical face classification requires full-period constant-axial ring boundaries";
pub(super) const GAP_CIRCULAR_PLANE_TRIM: &str =
    "circular planar classification requires vertex-less full-circle boundary loops";
pub(super) const GAP_CURVED_BODY_PARITY: &str =
    "curved body classification found no certified common-axis parity ray";

#[derive(Debug)]
pub(super) enum CurvedPrepOutcome {
    Ready(PreparedCurvedFace),
    /// The planar face is not circular and should be offered to the polygon
    /// classifier rather than treated as a curved capability gap.
    NotApplicable,
    Gap(&'static str),
}

#[derive(Debug)]
pub(super) enum PreparedCurvedFace {
    CircularPlane(CircularPlaneFace),
    Cylinder(CylinderFace),
}

impl PreparedCurvedFace {
    pub(super) const fn raw(&self) -> RawFaceId {
        match self {
            Self::CircularPlane(face) => face.raw,
            Self::Cylinder(face) => face.raw,
        }
    }
}

#[derive(Debug)]
pub(super) struct CircularPlaneFace {
    raw: RawFaceId,
    origin: [f64; 3],
    normal: [f64; 3],
    normal_sq: Interval,
    rings: Vec<CircleRing>,
    on_tol: f64,
    guard: f64,
}

#[derive(Debug)]
struct CircleRing {
    edge: RawEdgeId,
    circle: Circle,
    edge_tol: f64,
}

#[derive(Debug)]
pub(super) struct CylinderFace {
    raw: RawFaceId,
    frame: Frame,
    radius: f64,
    rings: Vec<CylinderRing>,
    on_tol: f64,
    guard: f64,
}

#[derive(Debug)]
struct CylinderRing {
    edge: RawEdgeId,
    axial_parameter: f64,
    edge_tol: f64,
}

pub(super) fn prepare_curved_face(
    store: &Store,
    raw: RawFaceId,
    linear: f64,
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<CurvedPrepOutcome> {
    let face = read(store.get(raw))?;
    charge(scope, 1 + face.loops().len() as u64)?;
    match read(store.surface(face.surface))? {
        SurfaceGeom::Plane(plane) => prepare_circular_plane(store, raw, *plane, linear, scope),
        SurfaceGeom::Cylinder(cylinder) => prepare_cylinder(store, raw, *cylinder, linear, scope),
        _ => Ok(CurvedPrepOutcome::NotApplicable),
    }
}

fn prepare_circular_plane(
    store: &Store,
    raw: RawFaceId,
    plane: Plane,
    linear: f64,
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<CurvedPrepOutcome> {
    let face = read(store.get(raw))?;
    if face.loops().is_empty() {
        return Ok(CurvedPrepOutcome::NotApplicable);
    }
    let mut rings = Vec::with_capacity(face.loops().len());
    let mut max_tol = linear.max(face.tolerance().map_or(0.0, |value| value.value()));
    for &loop_id in face.loops() {
        let ring = read(store.get::<Loop>(loop_id))?;
        charge(scope, ring.fins().len() as u64)?;
        let [fin_id] = ring.fins() else {
            return if rings.is_empty() {
                Ok(CurvedPrepOutcome::NotApplicable)
            } else {
                Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM))
            };
        };
        let fin = read(store.get(*fin_id))?;
        let edge = read(store.get(fin.edge))?;
        if ring.face() != raw || fin.parent() != loop_id || !edge.fins().contains(fin_id) {
            return Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM));
        }
        if edge.vertices() != [None, None] || edge.bounds().is_some() {
            return Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM));
        }
        let edge_tol = linear.max(edge.tolerance.map_or(0.0, |value| value.value()));
        charge_whole_fin_authority(store, raw, loop_id, *fin_id, scope)?;
        if certify_whole_fin_incidence(store, raw, loop_id, *fin_id, edge_tol)
            != WholeFinIncidence::Certified
        {
            return Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM));
        }
        let Some(curve_id) = edge.curve else {
            return Ok(CurvedPrepOutcome::NotApplicable);
        };
        let CurveGeom::Circle(circle) = read(store.curve(curve_id))? else {
            return if rings.is_empty() {
                Ok(CurvedPrepOutcome::NotApplicable)
            } else {
                Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM))
            };
        };
        max_tol = max_tol.max(edge_tol);
        rings.push(CircleRing {
            edge: fin.edge,
            circle: *circle,
            edge_tol,
        });
    }

    let normal = as_coords(plane.frame().z());
    let normal_sq = norm_sq_interval(normal);
    if normal_sq.lo() <= 0.0 || normal_sq.lo().is_nan() {
        return Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM));
    }
    let guard = (4.0 * max_tol).next_up();
    if !guard.is_finite() {
        return Ok(CurvedPrepOutcome::Gap(GAP_CIRCULAR_PLANE_TRIM));
    }
    Ok(CurvedPrepOutcome::Ready(PreparedCurvedFace::CircularPlane(
        CircularPlaneFace {
            raw,
            origin: as_coords(plane.frame().origin()),
            normal,
            normal_sq,
            rings,
            on_tol: linear,
            guard,
        },
    )))
}

fn prepare_cylinder(
    store: &Store,
    raw: RawFaceId,
    cylinder: Cylinder,
    linear: f64,
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<CurvedPrepOutcome> {
    let face = read(store.get(raw))?;
    if face.loops().is_empty() {
        return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
    }
    let mut rings = Vec::with_capacity(face.loops().len());
    let mut max_tol = linear.max(face.tolerance().map_or(0.0, |value| value.value()));
    for &loop_id in face.loops() {
        let ring = read(store.get::<Loop>(loop_id))?;
        charge(scope, ring.fins().len() as u64)?;
        let [fin_id] = ring.fins() else {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        };
        let fin = read(store.get(*fin_id))?;
        let edge = read(store.get(fin.edge))?;
        if ring.face() != raw || fin.parent() != loop_id || !edge.fins().contains(fin_id) {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        }
        if edge.vertices() != [None, None] || edge.bounds().is_some() {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        }
        let edge_tol = linear.max(edge.tolerance.map_or(0.0, |value| value.value()));
        charge_whole_fin_authority(store, raw, loop_id, *fin_id, scope)?;
        if certify_whole_fin_incidence(store, raw, loop_id, *fin_id, edge_tol)
            != WholeFinIncidence::Certified
        {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        }
        let Some(use_) = fin.pcurve() else {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        };
        let Some(line) = read(store.pcurve(use_.curve()))?.as_line() else {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        };
        let winding = use_.closure_winding();
        let horizontal = line.dir().y == 0.0 && line.dir().x != 0.0;
        let full_period = matches!(winding, Some([1 | -1, 0]));
        let chart_is_axially_fixed = use_.chart().period_shifts()[1] == 0;
        let rate = line.dir().x * use_.edge_to_pcurve().scale();
        let winding_matches_rate = matches!(winding, Some([1, 0])) && rate > 0.0
            || matches!(winding, Some([-1, 0])) && rate < 0.0;
        if !horizontal || !full_period || !chart_is_axially_fixed || !winding_matches_rate {
            return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
        }
        max_tol = max_tol.max(edge_tol);
        rings.push(CylinderRing {
            edge: fin.edge,
            axial_parameter: line.origin().y,
            edge_tol,
        });
    }
    let guard = (4.0 * max_tol).next_up();
    if !guard.is_finite() || rings.len() != 2 || rings[0].edge == rings[1].edge {
        return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
    }
    let separation = (Interval::point(rings[0].axial_parameter)
        - Interval::point(rings[1].axial_parameter))
    .square()
    .sqrt();
    if separation.is_none_or(|value| value.lo() <= 2.0 * guard) {
        return Ok(CurvedPrepOutcome::Gap(GAP_CYLINDER_TRIM));
    }
    Ok(CurvedPrepOutcome::Ready(PreparedCurvedFace::Cylinder(
        CylinderFace {
            raw,
            frame: *cylinder.frame(),
            radius: cylinder.radius(),
            rings,
            on_tol: linear,
            guard,
        },
    )))
}

pub(super) fn face_site(
    face: &PreparedCurvedFace,
    point: [f64; 3],
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<SiteOutcome> {
    match face {
        PreparedCurvedFace::CircularPlane(face) => circular_plane_site(face, point, scope),
        PreparedCurvedFace::Cylinder(face) => cylinder_site(face, point, scope),
    }
}

fn circular_plane_site(
    face: &CircularPlaneFace,
    point: [f64; 3],
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<SiteOutcome> {
    charge(scope, 2 + 2 * face.rings.len() as u64)?;
    match plane_band(
        face.origin,
        face.normal,
        face.normal_sq,
        point,
        face.on_tol,
        face.guard,
    ) {
        MetricBand::Off => return Ok(SiteOutcome::Off),
        MetricBand::Gap => return Ok(SiteOutcome::Gap(super::GAP_GUARD_BAND)),
        MetricBand::On => {}
    }
    match circle_edge_scan(&face.rings, point, face.guard) {
        RingScan::Hit(edge) => return Ok(SiteOutcome::On(RawSite::EdgeInterior(edge))),
        RingScan::Gap => return Ok(SiteOutcome::Gap(super::GAP_GUARD_BAND)),
        RingScan::Clear => {}
    }
    match circular_trim_parity(&face.rings, point) {
        TrimParity::Inside => Ok(SiteOutcome::On(RawSite::Interior)),
        TrimParity::Outside => Ok(SiteOutcome::Off),
        TrimParity::Gap => Ok(SiteOutcome::Gap(super::GAP_PROJECTED_CONTACT)),
    }
}

fn cylinder_site(
    face: &CylinderFace,
    point: [f64; 3],
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<SiteOutcome> {
    charge(scope, 2 + 2 * face.rings.len() as u64)?;
    match cylinder_ring_scan(face, point) {
        RingScan::Hit(edge) => return Ok(SiteOutcome::On(RawSite::EdgeInterior(edge))),
        RingScan::Gap => return Ok(SiteOutcome::Gap(super::GAP_GUARD_BAND)),
        RingScan::Clear => {}
    }
    match cylinder_radial_band(face, point) {
        MetricBand::Off => return Ok(SiteOutcome::Off),
        MetricBand::Gap => return Ok(SiteOutcome::Gap(super::GAP_GUARD_BAND)),
        MetricBand::On => {}
    }

    let axis = as_coords(face.frame.z());
    let origin = as_coords(face.frame.origin());
    let mut below = 0_u64;
    for ring in &face.rings {
        let Some(side) = affine_dot3(axis, point, origin, -ring.axial_parameter) else {
            return Ok(SiteOutcome::Gap(GAP_CYLINDER_TRIM));
        };
        match side.sign() {
            Orientation::Positive => below += 1,
            Orientation::Negative => {}
            Orientation::Zero => return Ok(SiteOutcome::Gap(super::GAP_GUARD_BAND)),
        }
    }
    if below % 2 == 1 {
        Ok(SiteOutcome::On(RawSite::Interior))
    } else {
        Ok(SiteOutcome::Off)
    }
}

#[derive(Debug, Clone, Copy)]
enum MetricBand {
    On,
    Off,
    Gap,
}

fn plane_band(
    origin: [f64; 3],
    normal: [f64; 3],
    normal_sq: Interval,
    point: [f64; 3],
    on_tol: f64,
    guard: f64,
) -> MetricBand {
    let offset_sq = dot_interval(normal, point, origin).square();
    if (offset_sq - Interval::point(on_tol).square() * normal_sq).hi() <= 0.0 {
        return MetricBand::On;
    }
    if (offset_sq - Interval::point(guard).square() * normal_sq).lo() >= 0.0 {
        return MetricBand::Off;
    }
    MetricBand::Gap
}

fn cylinder_radial_band(face: &CylinderFace, point: [f64; 3]) -> MetricBand {
    let Some(radial) = cylinder_radial_sq(face.frame, point).sqrt() else {
        return MetricBand::Gap;
    };
    let distance_sq = (radial - Interval::point(face.radius)).square();
    if (distance_sq - Interval::point(face.on_tol).square()).hi() <= 0.0 {
        MetricBand::On
    } else if (distance_sq - Interval::point(face.guard).square()).lo() >= 0.0 {
        MetricBand::Off
    } else {
        MetricBand::Gap
    }
}

enum RingScan {
    Hit(RawEdgeId),
    Clear,
    Gap,
}

fn circle_edge_scan(rings: &[CircleRing], point: [f64; 3], guard: f64) -> RingScan {
    let guard_sq = Interval::point(guard).square();
    let mut gap = false;
    for ring in rings {
        let Some(distance_sq) = point_circle_distance_sq(ring.circle, point) else {
            return RingScan::Gap;
        };
        if (distance_sq - Interval::point(ring.edge_tol).square()).hi() <= 0.0 {
            return RingScan::Hit(ring.edge);
        }
        if (distance_sq - guard_sq).lo() < 0.0 {
            gap = true;
        }
    }
    if gap { RingScan::Gap } else { RingScan::Clear }
}

fn cylinder_ring_scan(face: &CylinderFace, point: [f64; 3]) -> RingScan {
    let Some(radial) = cylinder_radial_sq(face.frame, point).sqrt() else {
        return RingScan::Gap;
    };
    let radial_delta_sq = (radial - Interval::point(face.radius)).square();
    let axial = dot_interval(
        as_coords(face.frame.z()),
        point,
        as_coords(face.frame.origin()),
    );
    let guard_sq = Interval::point(face.guard).square();
    let mut gap = false;
    for ring in &face.rings {
        let distance_sq =
            radial_delta_sq + (axial - Interval::point(ring.axial_parameter)).square();
        if (distance_sq - Interval::point(ring.edge_tol).square()).hi() <= 0.0 {
            return RingScan::Hit(ring.edge);
        }
        if (distance_sq - guard_sq).lo() < 0.0 {
            gap = true;
        }
    }
    if gap { RingScan::Gap } else { RingScan::Clear }
}

enum TrimParity {
    Inside,
    Outside,
    Gap,
}

fn circular_trim_parity(rings: &[CircleRing], point: [f64; 3]) -> TrimParity {
    let mut containing = 0_u64;
    for ring in rings {
        let local = local_intervals(*ring.circle.frame(), point);
        let radial_sq = local[0].square() + local[1].square();
        let sign = radial_sq - Interval::point(ring.circle.radius()).square();
        if sign.hi() < 0.0 {
            containing += 1;
        } else if sign.lo() <= 0.0 {
            return TrimParity::Gap;
        }
    }
    if containing % 2 == 1 {
        TrimParity::Inside
    } else {
        TrimParity::Outside
    }
}

pub(super) struct RawCurvedParityWitness {
    pub(super) far_point: [f64; 3],
    pub(super) crossings: u32,
    pub(super) crossed_faces: Vec<RawFaceId>,
}

pub(super) enum CurvedParityOutcome {
    Decided {
        inside: bool,
        witness: RawCurvedParityWitness,
    },
    Gap,
}

/// Count crossings along a shared cylinder-axis direction.
///
/// A line parallel to every admitted cylinder axis cannot cross a cylindrical
/// face.  Its crossings against circular planar faces are isolated with
/// outward intervals and accepted only when the plane intersection lies
/// strictly inside the exact circular trim parity.  This is general over
/// face order, cylinder placement/radius, and any number of ring trims.
pub(super) fn axial_parity_refs(
    faces: &[&PreparedCurvedFace],
    point: [f64; 3],
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<CurvedParityOutcome> {
    let attempt_work: u64 = faces
        .iter()
        .map(|face| match face {
            PreparedCurvedFace::CircularPlane(face) => 1 + face.rings.len() as u64,
            PreparedCurvedFace::Cylinder(_) => 1,
        })
        .sum();
    charge(scope, attempt_work)?;
    let axes: Vec<[f64; 3]> = faces
        .iter()
        .filter_map(|face| match face {
            PreparedCurvedFace::Cylinder(face) => Some(as_coords(face.frame.z())),
            PreparedCurvedFace::CircularPlane(_) => None,
        })
        .collect();
    if axes.is_empty()
        || !faces
            .iter()
            .any(|face| matches!(face, PreparedCurvedFace::CircularPlane(_)))
    {
        return Ok(CurvedParityOutcome::Gap);
    }
    for base in axes {
        for direction in [base, [-base[0], -base[1], -base[2]]] {
            charge(scope, attempt_work)?;
            if let Some(witness) = axial_parity_direction(faces, point, direction) {
                return Ok(CurvedParityOutcome::Decided {
                    inside: witness.crossings % 2 == 1,
                    witness,
                });
            }
        }
    }
    Ok(CurvedParityOutcome::Gap)
}

/// Exact ownership/manifold precondition for the curved parity path.
///
/// This does not infer validity from a primitive face count: it audits every
/// material-region face and every edge-use backlink, and requires each edge
/// to have exactly two uses within the admitted boundary closure.
pub(super) fn certify_closed_boundary(
    store: &Store,
    body: RawBodyId,
    prepared: &[&PreparedCurvedFace],
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<bool> {
    charge(scope, 1)?;
    let body_value = read(store.get(body))?;
    charge(scope, prepared.len() as u64)?;
    let mut expected = HashSet::with_capacity(prepared.len());
    for face in prepared {
        if !expected.insert(face.raw()) {
            return Ok(false);
        }
    }
    let mut seen_faces = HashSet::with_capacity(prepared.len());
    charge(scope, body_value.regions().len() as u64)?;
    for &region_id in body_value.regions() {
        charge(scope, 1)?;
        let region = read(store.get(region_id))?;
        if region.kind() != RegionKind::Solid {
            continue;
        }
        if region.body != body {
            return Ok(false);
        }
        charge(scope, region.shells().len() as u64)?;
        for &shell_id in region.shells() {
            charge(scope, 1)?;
            let shell = read(store.get(shell_id))?;
            if shell.region != region_id {
                return Ok(false);
            }
            charge(scope, shell.faces().len() as u64)?;
            for &face_id in shell.faces() {
                if !seen_faces.insert(face_id) || !expected.contains(&face_id) {
                    return Ok(false);
                }
                charge(scope, 1)?;
                let face = read(store.get(face_id))?;
                if face.shell() != shell_id {
                    return Ok(false);
                }
                charge(scope, face.loops().len() as u64)?;
                for &loop_id in face.loops() {
                    charge(scope, 1)?;
                    let ring = read(store.get::<Loop>(loop_id))?;
                    if ring.face() != face_id {
                        return Ok(false);
                    }
                    charge(scope, ring.fins().len() as u64)?;
                    for &fin_id in ring.fins() {
                        charge(scope, 2)?;
                        let fin = read(store.get(fin_id))?;
                        let edge = read(store.get(fin.edge()))?;
                        if fin.parent() != loop_id
                            || edge.fins().len() != 2
                            || !edge.fins().contains(&fin_id)
                        {
                            return Ok(false);
                        }
                        charge(scope, edge.fins().len() as u64)?;
                        for &peer_id in edge.fins() {
                            charge(scope, 2)?;
                            let peer = read(store.get(peer_id))?;
                            if peer.edge() != fin.edge() {
                                return Ok(false);
                            }
                            let peer_loop = read(store.get::<Loop>(peer.parent()))?;
                            if !expected.contains(&peer_loop.face()) {
                                return Ok(false);
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(seen_faces.len() == expected.len())
}

/// Charge the topology scans delegated to the topology-owned incidence
/// certifier. The preflight mirrors only collection lengths; authority and
/// geometric conclusions remain exclusively in `ktopo`.
fn charge_whole_fin_authority(
    store: &Store,
    face_id: RawFaceId,
    loop_id: ktopo::entity::LoopId,
    fin_id: ktopo::entity::FinId,
    scope: &mut kcore::operation::OperationScope<'_, '_>,
) -> Result<()> {
    charge(scope, 3)?;
    let Ok(face) = store.get(face_id) else {
        return Ok(());
    };
    let Ok(loop_) = store.get(loop_id) else {
        return Ok(());
    };
    let Ok(fin) = store.get(fin_id) else {
        return Ok(());
    };
    charge(scope, (face.loops().len() + loop_.fins().len()) as u64)?;

    charge(scope, 2)?;
    let Ok(shell) = store.get(face.shell()) else {
        return Ok(());
    };
    let Ok(edge) = store.get(fin.edge()) else {
        return Ok(());
    };
    charge(scope, (shell.faces().len() + edge.fins().len()) as u64)?;
    for &peer_id in edge.fins() {
        charge(scope, 4)?;
        let Ok(peer) = store.get(peer_id) else {
            continue;
        };
        let Ok(peer_loop) = store.get::<Loop>(peer.parent()) else {
            continue;
        };
        let Ok(peer_face) = store.get(peer_loop.face()) else {
            continue;
        };
        let Ok(peer_shell) = store.get(peer_face.shell()) else {
            continue;
        };
        charge(
            scope,
            (peer_loop.fins().len() + peer_face.loops().len() + peer_shell.faces().len()) as u64,
        )?;
    }
    Ok(())
}

fn axial_parity_direction(
    faces: &[&PreparedCurvedFace],
    point: [f64; 3],
    direction: [f64; 3],
) -> Option<RawCurvedParityWitness> {
    let mut crossings = 0_u32;
    let mut crossed_faces = Vec::new();
    let mut far_t = 1.0_f64;
    for &face in faces {
        match face {
            PreparedCurvedFace::Cylinder(cylinder) => {
                let axis = as_coords(cylinder.frame.z());
                // Exact stored-axis identity is the admitted common-axis
                // proof. Near-parallel axes are never rounded into this
                // family; they make this ray candidate fail closed.
                if axis != direction && axis != [-direction[0], -direction[1], -direction[2]] {
                    return None;
                }
            }
            PreparedCurvedFace::CircularPlane(plane) => {
                let numerator = -dot_interval(plane.normal, point, plane.origin);
                let denominator = dot_vectors_interval(plane.normal, direction);
                if denominator.contains(0.0) {
                    if numerator.contains(0.0) {
                        return None;
                    }
                    continue;
                }
                let t = numerator.checked_div(denominator)?;
                if t.hi() <= 0.0 {
                    continue;
                }
                if t.lo() <= 0.0 || !t.lo().is_finite() || !t.hi().is_finite() {
                    return None;
                }
                far_t = far_t.max(t.hi());
                let mut containing = 0_u64;
                for ring in &plane.rings {
                    let local =
                        line_point_local_intervals(*ring.circle.frame(), point, direction, t);
                    let sign = local[0].square() + local[1].square()
                        - Interval::point(ring.circle.radius()).square();
                    if sign.hi() < 0.0 {
                        containing += 1;
                    } else if sign.lo() <= 0.0 {
                        return None;
                    }
                }
                if containing % 2 == 1 {
                    crossings = crossings.checked_add(1)?;
                    crossed_faces.push(plane.raw);
                }
            }
        }
    }
    far_t = (2.0 * far_t).max(1.0);
    let far_point = [
        point[0] + far_t * direction[0],
        point[1] + far_t * direction[1],
        point[2] + far_t * direction[2],
    ];
    far_point
        .iter()
        .all(|value| value.is_finite())
        .then_some(RawCurvedParityWitness {
            far_point,
            crossings,
            crossed_faces,
        })
}

fn point_circle_distance_sq(circle: Circle, point: [f64; 3]) -> Option<Interval> {
    let local = local_intervals(*circle.frame(), point);
    let radial = (local[0].square() + local[1].square()).sqrt()?;
    Some((radial - Interval::point(circle.radius())).square() + local[2].square())
}

fn cylinder_radial_sq(frame: Frame, point: [f64; 3]) -> Interval {
    let local = local_intervals(frame, point);
    local[0].square() + local[1].square()
}

fn local_intervals(frame: Frame, point: [f64; 3]) -> [Interval; 3] {
    let origin = as_coords(frame.origin());
    [
        dot_interval(as_coords(frame.x()), point, origin),
        dot_interval(as_coords(frame.y()), point, origin),
        dot_interval(as_coords(frame.z()), point, origin),
    ]
}

fn line_point_local_intervals(
    frame: Frame,
    point: [f64; 3],
    direction: [f64; 3],
    t: Interval,
) -> [Interval; 3] {
    let origin = as_coords(frame.origin());
    [frame.x(), frame.y(), frame.z()].map(|axis| {
        let axis = as_coords(axis);
        dot_interval(axis, point, origin) + dot_vectors_interval(axis, direction) * t
    })
}

fn norm_sq_interval(vector: [f64; 3]) -> Interval {
    vector.into_iter().fold(Interval::point(0.0), |sum, value| {
        sum + Interval::point(value).square()
    })
}

fn dot_interval(normal: [f64; 3], point: [f64; 3], origin: [f64; 3]) -> Interval {
    (0..3).fold(Interval::point(0.0), |sum, axis| {
        sum + Interval::point(normal[axis])
            * (Interval::point(point[axis]) - Interval::point(origin[axis]))
    })
}

fn dot_vectors_interval(first: [f64; 3], second: [f64; 3]) -> Interval {
    (0..3).fold(Interval::point(0.0), |sum, axis| {
        sum + Interval::point(first[axis]) * Interval::point(second[axis])
    })
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
        SessionPolicy,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::curve::Circle;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};
    use ktopo::geom::{CurveGeom, SurfaceGeom};
    use ktopo::store::Store;

    use crate::{
        BodyId, ClassifyPointInBodyRequest, ClassifyPointOnFaceRequest, FaceId, Kernel,
        OperationSettings, POINT_CLASSIFICATION_WORK, PartId, PointBodyClassification,
        PointBodyVerdict, PointFaceSite, PointFaceVerdict, Session,
    };

    fn tilted() -> Frame {
        Frame::new(
            Point3::new(3.25, -1.75, 0.625),
            Vec3::new(0.3, -0.7, 1.1),
            Vec3::new(1.0, 0.4, 0.2),
        )
        .unwrap()
    }

    fn cylinder_part(frame: Frame, reorder: bool) -> (Session, PartId, BodyId) {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let raw = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            let store = edit.store_mut_for_test();
            let body = ktopo::make::cylinder(store, &frame, 2.0, 3.0).unwrap();
            if reorder {
                let material_region = store
                    .get(body)
                    .unwrap()
                    .regions()
                    .iter()
                    .copied()
                    .find(|&region| {
                        store.get(region).unwrap().kind() == ktopo::entity::RegionKind::Solid
                    })
                    .unwrap();
                let shell = store.get(material_region).unwrap().shells()[0];
                let side = store
                    .get(shell)
                    .unwrap()
                    .faces()
                    .iter()
                    .copied()
                    .find(|&face| {
                        let face = store.get(face).unwrap();
                        matches!(
                            store.surface(face.surface()).unwrap(),
                            SurfaceGeom::Cylinder(_)
                        )
                    })
                    .unwrap();
                let mut transaction = store.transaction().unwrap();
                {
                    let mut assembly = transaction.assembly();
                    assembly.get_mut(shell).unwrap().faces.reverse();
                    assembly.get_mut(side).unwrap().loops.reverse();
                }
                transaction.commit_checked_body(body).unwrap();
            }
            body
        };
        (session, part_id.clone(), BodyId::new(part_id, raw))
    }

    fn classify_body(
        session: &Session,
        part_id: &PartId,
        body: &BodyId,
        point: Point3,
    ) -> PointBodyClassification {
        session
            .part(part_id.clone())
            .unwrap()
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap()
    }

    #[test]
    fn finite_cylinder_body_matches_closed_form_under_rigid_frames_and_reordering() {
        for (frame, reorder) in [(Frame::world(), false), (tilted(), true)] {
            let (session, part_id, body) = cylinder_part(frame, reorder);
            for local in [[0.0, 0.0, 1.5], [1.25, -0.5, 0.25], [-1.5, 0.25, 2.75]] {
                assert_eq!(
                    classify_body(
                        &session,
                        &part_id,
                        &body,
                        frame.point_at(local[0], local[1], local[2]),
                    )
                    .verdict(),
                    &PointBodyVerdict::Interior
                );
            }
            for local in [
                [2.5, 0.0, 1.5],
                [0.0, 0.0, -0.5],
                [0.0, 0.0, 3.5],
                [2.5, 0.0, -0.5],
            ] {
                let result = classify_body(
                    &session,
                    &part_id,
                    &body,
                    frame.point_at(local[0], local[1], local[2]),
                );
                assert_eq!(result.verdict(), &PointBodyVerdict::Exterior);
                assert_eq!(result.witness().unwrap().crossings() % 2, 0);
            }
            let interior = classify_body(&session, &part_id, &body, frame.point_at(0.5, 0.25, 1.5));
            let witness = interior.witness().unwrap();
            assert_eq!(witness.crossings() % 2, 1);
            assert_eq!(witness.crossed_faces().len(), witness.crossings() as usize);
        }
    }

    #[test]
    fn cylinder_side_caps_and_ring_are_classified_from_geometry() {
        let frame = tilted();
        let (session, part_id, body) = cylinder_part(frame, true);
        let part = session.part(part_id.clone()).unwrap();
        let store = &part.state.store;
        let faces = store.faces_of_body(body.raw()).unwrap();
        let side = faces
            .iter()
            .copied()
            .find(|&raw| {
                let face = store.get(raw).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let caps = faces
            .iter()
            .copied()
            .filter(|&raw| {
                let face = store.get(raw).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Plane(_)
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(caps.len(), 2);

        let side_verdict = part
            .classify_point_on_face(ClassifyPointOnFaceRequest::new(
                FaceId::new(part_id.clone(), side),
                frame.point_at(2.0, 0.0, 1.5),
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            side_verdict.verdict(),
            &PointFaceVerdict::On(PointFaceSite::Interior)
        );

        for cap in caps {
            let face = store.get(cap).unwrap();
            let SurfaceGeom::Plane(plane) = store.surface(face.surface()).unwrap() else {
                unreachable!()
            };
            let result = part
                .classify_point_on_face(ClassifyPointOnFaceRequest::new(
                    FaceId::new(part_id.clone(), cap),
                    plane.frame().origin(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                result.verdict(),
                &PointFaceVerdict::On(PointFaceSite::Interior)
            );
        }

        let ring = classify_body(&session, &part_id, &body, frame.point_at(2.0, 0.0, 3.0));
        assert!(matches!(
            ring.verdict(),
            PointBodyVerdict::Boundary {
                site: PointFaceSite::EdgeInterior(_),
                ..
            }
        ));
    }

    #[test]
    fn cylinder_metric_guard_bands_fail_closed() {
        let frame = Frame::world();
        let (session, part_id, body) = cylinder_part(frame, false);

        let on_side = classify_body(
            &session,
            &part_id,
            &body,
            frame.point_at(2.0 + 1e-9, 0.0, 1.5),
        );
        assert!(matches!(
            on_side.verdict(),
            PointBodyVerdict::Boundary { .. }
        ));

        for point in [
            frame.point_at(2.0 + 2e-8, 0.0, 1.5),
            frame.point_at(0.0, 0.0, 3.0 + 2e-8),
        ] {
            let guarded = classify_body(&session, &part_id, &body, point);
            assert_eq!(
                guarded.verdict(),
                &PointBodyVerdict::Indeterminate {
                    reason: super::super::GAP_GUARD_BAND
                }
            );
            assert!(guarded.witness().is_none());
        }

        for point in [
            frame.point_at(2.0 + 1e-7, 0.0, 1.5),
            frame.point_at(0.0, 0.0, 3.0 + 1e-7),
        ] {
            assert_eq!(
                classify_body(&session, &part_id, &body, point).verdict(),
                &PointBodyVerdict::Exterior
            );
        }
    }

    #[test]
    fn whole_ring_incidence_is_required_not_inferred_from_pcurve_metadata() {
        let frame = tilted();
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &frame, 2.0, 3.0).unwrap();
        let side = store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|&raw| {
                let face = store.get(raw).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let loop_id = store.get(side).unwrap().loops()[0];
        let fin = store.get(loop_id).unwrap().fins()[0];
        let edge = store.get(fin).unwrap().edge;
        let curve_id = store.get(edge).unwrap().curve.unwrap();
        let CurveGeom::Circle(circle) = store.curve(curve_id).unwrap() else {
            unreachable!()
        };
        let displaced = Circle::new(
            circle
                .frame()
                .with_origin(circle.frame().origin() + circle.frame().x() * 0.25),
            circle.radius(),
        )
        .unwrap();

        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .replace_curve(curve_id, CurveGeom::Circle(displaced))
            .unwrap();
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                super::super::PointClassificationBudgetProfile::v1_defaults(),
            );
        let mut scope = OperationScope::new(&context);
        assert!(matches!(
            super::prepare_curved_face(transaction.store(), side, 1e-8, &mut scope).unwrap(),
            super::CurvedPrepOutcome::Gap(super::GAP_CYLINDER_TRIM)
        ));
        transaction.rollback().unwrap();
    }

    #[test]
    fn bounded_full_turn_circle_with_a_seam_vertex_is_not_a_ring_edge() {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let side = store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|&raw| {
                let face = store.get(raw).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let loop_id = store.get(side).unwrap().loops()[0];
        let fin = store.get(loop_id).unwrap().fins()[0];
        let edge = store.get(fin).unwrap().edge;
        let acorn = ktopo::make::acorn(&mut store, Point3::new(9.0, 9.0, 9.0)).unwrap();
        let seam_vertex = store.vertices_of_body(acorn).unwrap()[0];

        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let edge = assembly.get_mut(edge).unwrap();
            edge.vertices = [Some(seam_vertex), Some(seam_vertex)];
            edge.bounds = Some((0.0, core::f64::consts::TAU));
        }
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                super::super::PointClassificationBudgetProfile::v1_defaults(),
            );
        let mut scope = OperationScope::new(&context);
        assert!(matches!(
            super::prepare_curved_face(transaction.store(), side, 1e-8, &mut scope).unwrap(),
            super::CurvedPrepOutcome::Gap(super::GAP_CYLINDER_TRIM)
        ));
        transaction.rollback().unwrap();
    }

    #[test]
    fn finite_cylinder_band_requires_two_distinct_separated_ring_loops() {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let side = store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|&raw| {
                let face = store.get(raw).unwrap();
                matches!(
                    store.surface(face.surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let mut transaction = store.transaction().unwrap();
        transaction.assembly().get_mut(side).unwrap().loops.pop();
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                super::super::PointClassificationBudgetProfile::v1_defaults(),
            );
        let mut scope = OperationScope::new(&context);
        assert!(matches!(
            super::prepare_curved_face(transaction.store(), side, 1e-8, &mut scope).unwrap(),
            super::CurvedPrepOutcome::Gap(super::GAP_CYLINDER_TRIM)
        ));
        transaction.rollback().unwrap();
    }

    #[test]
    fn malformed_curved_shell_is_indeterminate_before_a_boundary_verdict() {
        let frame = Frame::world();
        let (session, part_id, body) = cylinder_part(frame, false);
        let source = session.part(part_id.clone()).unwrap();
        let mut malformed_store = source.state.store.clone();
        let material_region = malformed_store
            .get(body.raw())
            .unwrap()
            .regions()
            .iter()
            .copied()
            .find(|&region| {
                malformed_store.get(region).unwrap().kind() == ktopo::entity::RegionKind::Solid
            })
            .unwrap();
        let shell = malformed_store.get(material_region).unwrap().shells()[0];
        let cap = malformed_store
            .get(shell)
            .unwrap()
            .faces()
            .iter()
            .copied()
            .find(|&face| {
                matches!(
                    malformed_store
                        .surface(malformed_store.get(face).unwrap().surface())
                        .unwrap(),
                    SurfaceGeom::Plane(_)
                )
            })
            .unwrap();

        let mut transaction = malformed_store.transaction().unwrap();
        transaction
            .assembly()
            .get_mut(shell)
            .unwrap()
            .faces
            .retain(|&face| face != cap);
        let malformed_state = crate::session::PartState {
            store: transaction.store().clone(),
        };
        let malformed_part = crate::session::Part {
            policy: session.policy(),
            id: part_id.clone(),
            state: &malformed_state,
        };
        let result = malformed_part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(
                body,
                frame.point_at(2.0, 0.0, 1.5),
            ))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            result.verdict(),
            &PointBodyVerdict::Indeterminate {
                reason: super::GAP_CYLINDER_TRIM
            }
        );
        transaction.rollback().unwrap();
    }

    #[test]
    fn curved_parity_work_has_an_exact_n_and_n_minus_one_boundary() {
        let (session, part_id, body) = cylinder_part(Frame::world(), false);
        let point = Point3::new(0.25, -0.5, 1.5);
        let part = session.part(part_id).unwrap();
        let baseline = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap();
        assert_eq!(
            baseline.result().unwrap().verdict(),
            &PointBodyVerdict::Interior
        );
        let consumed = baseline
            .report()
            .usage()
            .iter()
            .find(|usage| {
                usage.stage == POINT_CLASSIFICATION_WORK && usage.resource == ResourceKind::Work
            })
            .unwrap()
            .consumed;
        assert!(consumed > 0);

        let run = |allowed| {
            let plan = BudgetPlan::new([LimitSpec::new(
                POINT_CLASSIFICATION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap();
            part.classify_point_in_body(
                ClassifyPointInBodyRequest::new(body.clone(), point)
                    .with_settings(OperationSettings::new().with_budget_overrides(plan)),
            )
            .unwrap()
        };
        let refused = run(consumed - 1);
        assert_eq!(
            refused.result().unwrap_err().limit().unwrap().stage,
            POINT_CLASSIFICATION_WORK
        );
        let accepted = run(consumed);
        assert_eq!(
            accepted.result().unwrap().verdict(),
            &PointBodyVerdict::Interior
        );
    }
}
