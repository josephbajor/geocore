//! Structural proof for a convex cylinder clipped by planar halfspaces.
//!
//! This recognizer does not consume constructor provenance or enumerate a
//! named solid layout. It reconstructs one or more pairwise-disjoint finite
//! rectangular charts on one exact analytic cylinder from live pcurves,
//! obtains every planar constraint through manifold peer incidence, and
//! proves with outward intervals that every complete cylinder patch and every
//! boundary edge lie in every constraint. Planar face cells are simple Jordan
//! domains whose boundaries lie in the convex intersection. A strict interior
//! witness makes that intersection full dimensional.
//!
//! Consequently the connected, closed manifold is a boundaryless subset of
//! the connected boundary of the convex intersection; local whole-fin
//! incidence makes the inclusion open as well as closed, so it is the entire
//! boundary. Unsupported charts, curves, arithmetic, or constraint systems
//! fail closed.

use super::*;
use crate::entity::{FaceDomain, FinId};
use kcore::math;
use kgeom::param::ParamRange;

/// Cumulative structural and constraint work for clipped cylinders.
pub(crate) const CONVEX_CYLINDRICAL_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.convex-cylindrical-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid convex cylindrical shell work stage"),
    };

const DEFAULT_CONVEX_CYLINDRICAL_SHELL_WORK: u64 = 1_048_576;
const FLOATING_PROOF_GUARD: f64 = 16_384.0 * f64::EPSILON;

pub(super) fn convex_cylindrical_shell_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        CONVEX_CYLINDRICAL_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_CONVEX_CYLINDRICAL_SHELL_WORK,
    )])
    .expect("built-in convex cylindrical shell proof budget is valid")
}

#[derive(Debug)]
struct RectangularCylinderPatch {
    face: FaceId,
    cylinder: Cylinder,
    domain: FaceDomain,
    boundary_fins: Vec<FinId>,
}

#[derive(Debug, Clone, Copy)]
struct PlanarSupport {
    face: FaceId,
    origin: Point3,
    outward: Vec3,
    outward_sense: Sense,
}

/// Certify the representation class described in the module theorem.
pub(super) fn certify_convex_cylindrical_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }

    let mut cylinder_faces = Vec::new();
    let mut planar_faces = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => cylinder_faces.push((face_id, *cylinder)),
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            _ => return Ok(None),
        }
    }
    let Some((_, cylinder)) = cylinder_faces.first().copied() else {
        return Ok(None);
    };
    if planar_faces.is_empty() {
        return Ok(None);
    }

    if let Some(scope) = scope {
        scope.ledger().require_limit(
            CONVEX_CYLINDRICAL_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id, planar_faces.len(), cylinder_faces.len())?
        else {
            return Ok(Some(indeterminate()));
        };
        scope
            .ledger_mut()
            .charge(CONVEX_CYLINDRICAL_SHELL_WORK, work)?;
    }

    let mut patches = Vec::with_capacity(cylinder_faces.len());
    for (face, candidate) in cylinder_faces {
        if !same_cylinder_representation(candidate, cylinder) {
            return Ok(Some(indeterminate()));
        }
        let Some(patch) = prepare_rectangular_patch(store, face, candidate)? else {
            return Ok(Some(indeterminate()));
        };
        patches.push(patch);
    }
    if !patch_interiors_are_pairwise_disjoint(&patches) {
        return Ok(Some(indeterminate()));
    }
    if !certify_face_cells_and_incidence(store, shell_id)? {
        return Ok(Some(indeterminate()));
    }
    let Some((witness, supports)) =
        find_strict_witness_and_planar_supports(store, &patches, &planar_faces)?
    else {
        return Ok(Some(indeterminate()));
    };
    if !strictly_inside_cylinder(cylinder, witness)
        || !all_boundary_traces_satisfy_constraints(store, shell_id, cylinder, &patches, &supports)?
    {
        return Ok(Some(indeterminate()));
    }

    let orientation = certify_orientation(store, shell_id, &patches, &supports)?;
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation,
    }))
}

fn proof_work(
    store: &Store,
    shell_id: ShellId,
    plane_count: usize,
    cylinder_count: usize,
) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut fin_count = 0_usize;
    let mut edges = Vec::new();
    for &face_id in &shell.faces {
        for &loop_id in &store.get(face_id)?.loops {
            for &fin_id in &store.get(loop_id)?.fins {
                let fin = store.get(fin_id)?;
                fin_count = match fin_count.checked_add(1) {
                    Some(value) => value,
                    None => return Ok(None),
                };
                if !edges.contains(&fin.edge) {
                    edges.push(fin.edge);
                }
            }
        }
    }
    let Some(faces) = u64::try_from(shell.faces.len()).ok() else {
        return Ok(None);
    };
    let Some(fins) = u64::try_from(fin_count).ok() else {
        return Ok(None);
    };
    let Some(edges) = u64::try_from(edges.len()).ok() else {
        return Ok(None);
    };
    let Some(planes) = u64::try_from(plane_count).ok() else {
        return Ok(None);
    };
    let Some(cylinders) = u64::try_from(cylinder_count).ok() else {
        return Ok(None);
    };
    let Some(patch_pairs) = cylinders
        .checked_sub(1)
        .and_then(|less| cylinders.checked_mul(less))
        .map(|ordered| ordered / 2)
    else {
        return Ok(None);
    };
    // Face/fin structure, exact cylinder-representation and disjoint-chart
    // checks, every edge against every planar constraint plus one radial
    // cylinder decision, and both signed support passes for every
    // plane/patch pair.
    Ok(faces
        .checked_add(fins)
        .and_then(|work| work.checked_add(cylinders))
        .and_then(|work| work.checked_add(patch_pairs))
        .and_then(|work| {
            edges
                .checked_mul(planes.checked_add(1)?)
                .and_then(|pairs| work.checked_add(pairs))
        })
        .and_then(|work| {
            planes
                .checked_mul(cylinders)?
                .checked_mul(4)
                .and_then(|supports| work.checked_add(supports))
        }))
}

fn prepare_rectangular_patch(
    store: &Store,
    face_id: FaceId,
    cylinder: Cylinder,
) -> Result<Option<RectangularCylinderPatch>> {
    let face = store.get(face_id)?;
    let [loop_id] = face.loops.as_slice() else {
        return Ok(None);
    };
    let Some(declared_domain) = face.domain else {
        return Ok(None);
    };
    let loop_ = store.get(*loop_id)?;
    if loop_.fins.len() < 4
        || certify_loop_simplicity(store, *loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }

    let mut traversal = Vec::with_capacity(loop_.fins.len());
    let mut traversal_vertices = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, face_id, *loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some((lo, hi)), Some(pcurve)) = (edge.bounds, fin.pcurve) else {
            return Ok(None);
        };
        if edge.tolerance.is_some() || pcurve.closure_winding().is_some() || pcurve.seam().is_some()
        {
            return Ok(None);
        }
        let curve = store.get(pcurve.curve())?;
        if !matches!(curve, Curve2dGeom::Line(_)) {
            return Ok(None);
        }
        let map = pcurve.edge_to_pcurve();
        let mapped = [map.map(lo), map.map(hi)];
        let active = pcurve.range();
        if mapped.iter().any(|value| !value.is_finite())
            || mapped[0].min(mapped[1]).to_bits() != active.lo.to_bits()
            || mapped[0].max(mapped[1]).to_bits() != active.hi.to_bits()
        {
            return Ok(None);
        }
        let edge_parameters = if fin.sense == Sense::Forward {
            [lo, hi]
        } else {
            [hi, lo]
        };
        let [Some(tail), Some(head)] = edge.vertices else {
            return Ok(None);
        };
        traversal_vertices.push(if fin.sense == Sense::Forward {
            [tail, head]
        } else {
            [head, tail]
        });
        let start = pcurve.evaluate_uv(
            curve.as_curve(),
            edge_parameters[0],
            [Some(core::f64::consts::TAU), None],
        )?;
        let end = pcurve.evaluate_uv(
            curve.as_curve(),
            edge_parameters[1],
            [Some(core::f64::consts::TAU), None],
        )?;
        if start == end {
            return Ok(None);
        }
        traversal.push((start, end));
    }
    if traversal
        .iter()
        .zip(traversal.iter().cycle().skip(1))
        .zip(
            traversal_vertices
                .iter()
                .zip(traversal_vertices.iter().cycle().skip(1)),
        )
        .any(|(((_, end), (next, _)), (vertices, next_vertices))| {
            vertices[1] != next_vertices[0]
                || !certify_uv_join(patch_metric_scales(cylinder), *end, *next)
        })
    {
        return Ok(None);
    }
    let Some((nominal_domain, proof_domain)) = live_rectangular_domains(&traversal) else {
        return Ok(None);
    };
    if !declared_domain_contains(declared_domain, proof_domain) {
        return Ok(None);
    }
    let u_width = proof_domain.u.width();
    if !u_width.is_finite() || u_width <= 0.0 || u_width > core::f64::consts::PI {
        return Ok(None);
    }

    let mut coverage: [Vec<(f64, f64)>; 4] = core::array::from_fn(|_| Vec::new());
    for &(start, end) in &traversal {
        let Some(side) = rectangle_side(nominal_domain, start, end, patch_metric_scales(cylinder))
        else {
            return Ok(None);
        };
        let varying = if side < 2 {
            (start.y.min(end.y), start.y.max(end.y))
        } else {
            (start.x.min(end.x), start.x.max(end.x))
        };
        coverage[side].push(varying);
    }
    for (side, intervals) in coverage.iter_mut().enumerate() {
        let expected = if side < 2 {
            (nominal_domain.v.lo, nominal_domain.v.hi)
        } else {
            (nominal_domain.u.lo, nominal_domain.u.hi)
        };
        let metric_scale = if side < 2 { 1.0 } else { cylinder.radius() };
        if !intervals_cover_with_certified_joins(intervals, expected, metric_scale) {
            return Ok(None);
        }
    }
    Ok(Some(RectangularCylinderPatch {
        face: face_id,
        cylinder,
        domain: proof_domain,
        boundary_fins: loop_.fins.clone(),
    }))
}

fn live_rectangular_domains(traversal: &[(Point2, Point2)]) -> Option<(FaceDomain, FaceDomain)> {
    let mut points = traversal.iter().flat_map(|(start, end)| [*start, *end]);
    let first = points.next()?;
    if !first.x.is_finite() || !first.y.is_finite() {
        return None;
    }
    let (mut u_lo, mut u_hi, mut v_lo, mut v_hi) = (first.x, first.x, first.y, first.y);
    for point in points {
        if !point.x.is_finite() || !point.y.is_finite() {
            return None;
        }
        u_lo = u_lo.min(point.x);
        u_hi = u_hi.max(point.x);
        v_lo = v_lo.min(point.y);
        v_hi = v_hi.max(point.y);
    }
    let proof = FaceDomain::from_bounds(u_lo, u_hi, v_lo, v_hi).ok()?;
    let mut vertical = Vec::new();
    let mut horizontal = Vec::new();
    for &(start, end) in traversal {
        if start.x == end.x && start.y != end.y {
            if !vertical.contains(&start.x) {
                vertical.push(start.x);
            }
        } else if start.y == end.y && start.x != end.x {
            if !horizontal.contains(&start.y) {
                horizontal.push(start.y);
            }
        } else {
            return None;
        }
    }
    let [first_u, second_u] = vertical.as_slice() else {
        return None;
    };
    let [first_v, second_v] = horizontal.as_slice() else {
        return None;
    };
    let nominal = FaceDomain::from_bounds(
        first_u.min(*second_u),
        first_u.max(*second_u),
        first_v.min(*second_v),
        first_v.max(*second_v),
    )
    .ok()?;
    Some((nominal, proof))
}

fn declared_domain_contains(declared: FaceDomain, live: FaceDomain) -> bool {
    declared.u.lo <= live.u.lo
        && live.u.hi <= declared.u.hi
        && declared.v.lo <= live.v.lo
        && live.v.hi <= declared.v.hi
}

fn same_cylinder_representation(first: Cylinder, second: Cylinder) -> bool {
    first.radius().to_bits() == second.radius().to_bits()
        && same_point_bits(first.frame().origin(), second.frame().origin())
        && same_vec_bits(first.frame().x(), second.frame().x())
        && same_vec_bits(first.frame().y(), second.frame().y())
        && same_vec_bits(first.frame().z(), second.frame().z())
}

/// Prove that no two chart interiors cover the same cylinder point.
///
/// Authored longitude intervals may use different integer-period lifts.  The
/// interval quotient below bounds every period shift that could make their
/// open interiors overlap; each candidate is then refused unless outward
/// arithmetic proves separation.  Unmanageably large chart lifts and
/// rounding-ambiguous comparisons therefore fail closed.
fn patch_interiors_are_pairwise_disjoint(patches: &[RectangularCylinderPatch]) -> bool {
    patches.iter().enumerate().all(|(index, first)| {
        patches[index + 1..]
            .iter()
            .all(|second| patch_interiors_are_disjoint(first, second))
    })
}

fn patch_interiors_are_disjoint(
    first: &RectangularCylinderPatch,
    second: &RectangularCylinderPatch,
) -> bool {
    if first.domain.v.hi <= second.domain.v.lo || second.domain.v.hi <= first.domain.v.lo {
        return true;
    }

    let first_center_twice =
        Interval::point(first.domain.u.lo) + Interval::point(first.domain.u.hi);
    let second_center_twice =
        Interval::point(second.domain.u.lo) + Interval::point(second.domain.u.hi);
    let Some(relative_turn) = (first_center_twice - second_center_twice)
        .checked_div(Interval::point(2.0 * core::f64::consts::TAU))
    else {
        return false;
    };
    let candidate_turns = relative_turn + Interval::new(-0.5, 0.5);
    const EXACT_INTEGER_LIMIT: f64 = (1_u64 << 52) as f64;
    if !candidate_turns.lo().is_finite()
        || !candidate_turns.hi().is_finite()
        || candidate_turns.lo().abs() > EXACT_INTEGER_LIMIT
        || candidate_turns.hi().abs() > EXACT_INTEGER_LIMIT
    {
        return false;
    }
    let first_turn = candidate_turns.lo().ceil() as i64;
    let last_turn = candidate_turns.hi().floor() as i64;
    if last_turn < first_turn {
        return true;
    }
    let Some(candidate_count) = last_turn
        .checked_sub(first_turn)
        .and_then(|span| span.checked_add(1))
    else {
        return false;
    };
    if candidate_count > 4 {
        return false;
    }

    (first_turn..=last_turn).all(|turn| {
        let shift = Interval::point(turn as f64) * Interval::point(core::f64::consts::TAU);
        let shifted_lo = Interval::point(second.domain.u.lo) + shift;
        let shifted_hi = Interval::point(second.domain.u.hi) + shift;
        first.domain.u.hi <= shifted_lo.lo() || shifted_hi.hi() <= first.domain.u.lo
    })
}

fn rectangle_side(
    domain: FaceDomain,
    start: Point2,
    end: Point2,
    metric_scales: [f64; 2],
) -> Option<usize> {
    if start.x == end.x
        && parameter_in_certified_range(start.y, domain.v, metric_scales[1])
        && parameter_in_certified_range(end.y, domain.v, metric_scales[1])
    {
        if start.x == domain.u.lo {
            return Some(0);
        }
        if start.x == domain.u.hi {
            return Some(1);
        }
    }
    if start.y == end.y
        && parameter_in_certified_range(start.x, domain.u, metric_scales[0])
        && parameter_in_certified_range(end.x, domain.u, metric_scales[0])
    {
        if start.y == domain.v.lo {
            return Some(2);
        }
        if start.y == domain.v.hi {
            return Some(3);
        }
    }
    None
}

fn parameter_in_certified_range(value: f64, range: ParamRange, metric_scale: f64) -> bool {
    range.contains(value)
        || (value < range.lo && certify_parameter_join(value, range.lo, metric_scale))
        || (value > range.hi && certify_parameter_join(value, range.hi, metric_scale))
}

fn intervals_cover_with_certified_joins(
    intervals: &mut [(f64, f64)],
    expected: (f64, f64),
    metric_scale: f64,
) -> bool {
    if intervals.is_empty() {
        return false;
    }
    intervals.sort_by(|left, right| {
        left.0
            .total_cmp(&right.0)
            .then_with(|| left.1.total_cmp(&right.1))
    });
    certify_parameter_join(intervals[0].0, expected.0, metric_scale)
        && certify_parameter_join(intervals[intervals.len() - 1].1, expected.1, metric_scale)
        && intervals
            .windows(2)
            .all(|pair| certify_parameter_join(pair[0].1, pair[1].0, metric_scale))
}

fn patch_metric_scales(cylinder: Cylinder) -> [f64; 2] {
    [cylinder.radius(), 1.0]
}

fn certify_uv_join(scales: [f64; 2], first: Point2, second: Point2) -> bool {
    let u = Interval::point(first.x) - Interval::point(second.x);
    let v = Interval::point(first.y) - Interval::point(second.y);
    let bound = Interval::point(scales[0]) * Interval::point(interval_abs_upper(u))
        + Interval::point(scales[1]) * Interval::point(interval_abs_upper(v));
    bound.hi().is_finite() && bound.hi() <= LINEAR_RESOLUTION
}

fn certify_parameter_join(first: f64, second: f64, metric_scale: f64) -> bool {
    if first.to_bits() == second.to_bits() {
        return true;
    }
    if !metric_scale.is_finite() || metric_scale <= 0.0 {
        return false;
    }
    let delta = Interval::point(first) - Interval::point(second);
    let distance = Interval::point(interval_abs_upper(delta)) * Interval::point(metric_scale);
    distance.hi().is_finite() && distance.hi() <= LINEAR_RESOLUTION
}

fn certify_face_cells_and_incidence(store: &Store, shell_id: ShellId) -> Result<bool> {
    for &face_id in &store.get(shell_id)?.faces {
        let face = store.get(face_id)?;
        let [loop_id] = face.loops.as_slice() else {
            return Ok(false);
        };
        let loop_ = store.get(*loop_id)?;
        if loop_.fins.is_empty()
            || certify_loop_simplicity(store, *loop_id)? != LoopSimplicity::Certified
            || certify_loop_orientation(store, face_id, *loop_id)?.is_none()
        {
            return Ok(false);
        }
        for &fin_id in &loop_.fins {
            let fin = store.get(fin_id)?;
            let edge = store.get(fin.edge)?;
            let Some(pcurve) = fin.pcurve else {
                return Ok(false);
            };
            if face.tolerance.is_some()
                || edge.tolerance.is_some()
                || !matches!(
                    store.get(pcurve.curve())?,
                    Curve2dGeom::Line(_) | Curve2dGeom::Circle(_)
                )
                || certify_whole_fin_incidence(store, face_id, *loop_id, fin_id, LINEAR_RESOLUTION)
                    != WholeFinIncidence::Certified
            {
                return Ok(false);
            }
            for vertex in edge.vertices.into_iter().flatten() {
                if store.get(vertex)?.tolerance.is_some() {
                    return Ok(false);
                }
            }
        }
    }
    Ok(true)
}

fn strict_patch_witness(patch: &RectangularCylinderPatch) -> Option<Point3> {
    let u = patch.domain.u.lo + patch.domain.u.width() * 0.5;
    let v = patch.domain.v.lo + patch.domain.v.width() * 0.5;
    if !u.is_finite() || !v.is_finite() {
        return None;
    }
    let (sine, cosine) = math::sincos(u);
    let frame = patch.cylinder.frame();
    let witness = frame.origin()
        + frame.z() * v
        + (frame.x() * cosine + frame.y() * sine) * (patch.cylinder.radius() * 0.5);
    if !finite_point(witness) {
        return None;
    }
    let radial = radial_squared_interval(patch.cylinder, witness)?;
    let radius_squared = Interval::point(patch.cylinder.radius()).square();
    (radial.hi() < radius_squared.lo()).then_some(witness)
}

fn strict_axis_witness(patch: &RectangularCylinderPatch) -> Option<Point3> {
    let v = patch.domain.v.lo + patch.domain.v.width() * 0.5;
    if !v.is_finite() {
        return None;
    }
    let witness = patch.cylinder.frame().origin() + patch.cylinder.frame().z() * v;
    strictly_inside_cylinder(patch.cylinder, witness).then_some(witness)
}

fn strictly_inside_cylinder(cylinder: Cylinder, witness: Point3) -> bool {
    let Some(radial) = radial_squared_interval(cylinder, witness) else {
        return false;
    };
    let radius_squared = Interval::point(cylinder.radius()).square();
    radial.hi() < radius_squared.lo()
}

fn find_strict_witness_and_planar_supports(
    store: &Store,
    patches: &[RectangularCylinderPatch],
    planar_faces: &[FaceId],
) -> Result<Option<(Point3, Vec<PlanarSupport>)>> {
    for witness in patches
        .iter()
        .filter_map(strict_axis_witness)
        .chain(patches.iter().filter_map(strict_patch_witness))
    {
        if let Some(supports) = prepare_planar_supports(store, patches, planar_faces, witness)? {
            return Ok(Some((witness, supports)));
        }
    }
    Ok(None)
}

fn prepare_planar_supports(
    store: &Store,
    patches: &[RectangularCylinderPatch],
    planar_faces: &[FaceId],
    witness: Point3,
) -> Result<Option<Vec<PlanarSupport>>> {
    let mut peer_faces = Vec::new();
    for patch in patches {
        for &fin_id in &patch.boundary_fins {
            let Some(peer) = peer_face(store, fin_id)? else {
                return Ok(None);
            };
            if peer == patch.face
                || !matches!(store.get(store.get(peer)?.surface)?, SurfaceGeom::Plane(_))
            {
                return Ok(None);
            }
            if !peer_faces.contains(&peer) {
                peer_faces.push(peer);
            }
        }
    }
    if planar_faces.iter().any(|face| !peer_faces.contains(face))
        || peer_faces.iter().any(|face| !planar_faces.contains(face))
    {
        return Ok(None);
    }

    let mut supports = Vec::with_capacity(peer_faces.len());
    for face_id in peer_faces {
        let face = store.get(face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(None);
        };
        let raw = plane.frame().z();
        let origin = plane.frame().origin();
        let (Some(raw_patches), Some(raw_witness)) = (
            cylinder_patches_affine_range(patches, raw, origin),
            affine_value(raw, witness, origin),
        ) else {
            return Ok(None);
        };
        let Some(guard) = cylinder_patches_arithmetic_guard(patches, origin) else {
            return Ok(None);
        };
        let (outward, outward_sense) = if raw_patches.hi() <= guard && raw_witness.hi() < 0.0 {
            (raw, Sense::Forward)
        } else if raw_patches.lo() >= -guard && raw_witness.lo() > 0.0 {
            (-raw, Sense::Reversed)
        } else {
            return Ok(None);
        };
        let (Some(outward_patches), Some(outward_witness)) = (
            cylinder_patches_affine_range(patches, outward, origin),
            affine_value(outward, witness, origin),
        ) else {
            return Ok(None);
        };
        if outward_patches.hi() > guard || outward_witness.hi() >= 0.0 {
            return Ok(None);
        }
        supports.push(PlanarSupport {
            face: face_id,
            origin,
            outward,
            outward_sense,
        });
    }
    Ok((!supports.is_empty()).then_some(supports))
}

fn peer_face(store: &Store, fin_id: FinId) -> Result<Option<FaceId>> {
    let fin = store.get(fin_id)?;
    let edge = store.get(fin.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let peer = if *first == fin_id {
        *second
    } else if *second == fin_id {
        *first
    } else {
        return Ok(None);
    };
    Ok(Some(store.get(store.get(peer)?.parent)?.face))
}

fn all_boundary_traces_satisfy_constraints(
    store: &Store,
    shell_id: ShellId,
    cylinder: Cylinder,
    patches: &[RectangularCylinderPatch],
    supports: &[PlanarSupport],
) -> Result<bool> {
    let mut edges = Vec::new();
    for &face_id in &store.get(shell_id)?.faces {
        for &loop_id in &store.get(face_id)?.loops {
            for &fin_id in &store.get(loop_id)?.fins {
                let edge = store.get(fin_id)?.edge;
                if !edges.contains(&edge) {
                    edges.push(edge);
                }
            }
        }
    }
    for edge_id in edges {
        let edge = store.get(edge_id)?;
        let (Some(curve_id), Some((lo, hi))) = (edge.curve, edge.bounds) else {
            return Ok(false);
        };
        let curve = store.get(curve_id)?;
        for support in supports {
            let (range, scale) = if let Some(line) = exact_line_carrier(curve) {
                let endpoints = [line.eval(lo), line.eval(hi)];
                let (Some(first), Some(second)) = (
                    affine_value(support.outward, endpoints[0], support.origin),
                    affine_value(support.outward, endpoints[1], support.origin),
                ) else {
                    return Ok(false);
                };
                (
                    union(first, second),
                    point_scale(endpoints[0])
                        .max(point_scale(endpoints[1]))
                        .max(point_scale(support.origin)),
                )
            } else if let CurveGeom::Circle(circle) = curve {
                let Some(range) =
                    circle_affine_range(*circle, lo, hi, support.outward, support.origin)
                else {
                    return Ok(false);
                };
                (
                    range,
                    point_scale(circle.frame().origin())
                        .max(point_scale(support.origin))
                        .max(circle.radius()),
                )
            } else {
                return Ok(false);
            };
            let Some(guard) = arithmetic_guard(scale) else {
                return Ok(false);
            };
            if range.hi() > guard {
                return Ok(false);
            }
        }
        if let Some(line) = exact_line_carrier(curve) {
            for point in [line.eval(lo), line.eval(hi)] {
                let Some(radial) = radial_squared_interval(cylinder, point) else {
                    return Ok(false);
                };
                let radius_squared = Interval::point(cylinder.radius()).square();
                let Some(guard) = arithmetic_guard(
                    radial
                        .hi()
                        .abs()
                        .max(radius_squared.hi().abs())
                        .max(point_scale(point)),
                ) else {
                    return Ok(false);
                };
                if radial.hi() > radius_squared.hi() + guard {
                    return Ok(false);
                }
            }
        } else if let CurveGeom::Circle(circle) = curve {
            if !edge_has_any_patch_face(store, edge_id, patches)?
                || !circle_matches_cylinder(*circle, cylinder)
            {
                return Ok(false);
            }
        } else {
            return Ok(false);
        }
    }
    Ok(true)
}

fn edge_has_any_patch_face(
    store: &Store,
    edge_id: EdgeId,
    patches: &[RectangularCylinderPatch],
) -> Result<bool> {
    for &fin_id in &store.get(edge_id)?.fins {
        let loop_id = store.get(fin_id)?.parent;
        let face = store.get(loop_id)?.face;
        if patches.iter().any(|patch| patch.face == face) {
            return Ok(true);
        }
    }
    Ok(false)
}

fn certify_orientation(
    store: &Store,
    shell_id: ShellId,
    patches: &[RectangularCylinderPatch],
    supports: &[PlanarSupport],
) -> Result<ShellOrientation> {
    let mut all_outward = true;
    let mut all_inward = true;
    for patch in patches {
        let sense = store.get(patch.face)?.sense;
        all_outward &= sense == Sense::Forward;
        all_inward &= sense == Sense::Reversed;
    }
    for support in supports {
        let sense = store.get(support.face)?.sense;
        all_outward &= sense == support.outward_sense;
        all_inward &= sense != support.outward_sense;
    }
    for &face_id in &store.get(shell_id)?.faces {
        let face = store.get(face_id)?;
        let [loop_id] = face.loops.as_slice() else {
            return Ok(ShellOrientation::Indeterminate);
        };
        let expected = if face.sense == Sense::Forward {
            PredicateOrientation::Positive
        } else {
            PredicateOrientation::Negative
        };
        if certify_loop_orientation(store, face_id, *loop_id)? != Some(expected) {
            return Ok(ShellOrientation::Invalid);
        }
    }
    Ok(if all_outward {
        ShellOrientation::Positive
    } else if all_inward {
        ShellOrientation::Negative
    } else {
        ShellOrientation::Invalid
    })
}

fn cylinder_patch_affine_range(
    patch: &RectangularCylinderPatch,
    normal: Vec3,
    plane_origin: Point3,
) -> Option<Interval> {
    let frame = patch.cylinder.frame();
    let v = Interval::new(patch.domain.v.lo, patch.domain.v.hi);
    let constant = affine_value(normal, frame.origin(), plane_origin)?;
    let x = dot_interval(normal, frame.x())? * Interval::point(patch.cylinder.radius());
    let y = dot_interval(normal, frame.y())? * Interval::point(patch.cylinder.radius());
    let z = dot_interval(normal, frame.z())?;
    let radial = harmonic_range(x, y, patch.domain.u.lo, patch.domain.u.hi)?;
    finite_interval(constant + radial + z * v)
}

fn cylinder_patches_affine_range(
    patches: &[RectangularCylinderPatch],
    normal: Vec3,
    plane_origin: Point3,
) -> Option<Interval> {
    let mut patches = patches.iter();
    let mut range = cylinder_patch_affine_range(patches.next()?, normal, plane_origin)?;
    for patch in patches {
        range = union(
            range,
            cylinder_patch_affine_range(patch, normal, plane_origin)?,
        );
    }
    Some(range)
}

fn circle_affine_range(
    circle: kgeom::curve::Circle,
    lo: f64,
    hi: f64,
    normal: Vec3,
    plane_origin: Point3,
) -> Option<Interval> {
    if !lo.is_finite() || !hi.is_finite() || lo >= hi {
        return None;
    }
    let frame = circle.frame();
    let constant = affine_value(normal, frame.origin(), plane_origin)?;
    let x = dot_interval(normal, frame.x())? * Interval::point(circle.radius());
    let y = dot_interval(normal, frame.y())? * Interval::point(circle.radius());
    finite_interval(constant + harmonic_range(x, y, lo, hi)?)
}

/// Outward range of `a*cos(u) + b*sin(u)` over one finite interval.
///
/// Ranging sine and cosine independently loses their unit-circle correlation
/// and can place a clipped cylinder patch outside a sloped support that owns
/// both of its exact endpoints.  Here endpoint values are evaluated together.
/// The only possible interior extrema are the coefficient direction and its
/// antipode.  An outward phase enclosure includes coefficient-dot rounding;
/// an uncertain direction therefore inserts the full amplitude and fails
/// loose rather than omitting an extremum.
fn harmonic_range(a: Interval, b: Interval, lo: f64, hi: f64) -> Option<Interval> {
    if finite_interval(a).is_none()
        || finite_interval(b).is_none()
        || !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
    {
        return None;
    }
    let amplitude = (Interval::point(interval_abs_upper(a)).square()
        + Interval::point(interval_abs_upper(b)).square())
    .sqrt()?
    .hi();
    if !amplitude.is_finite() {
        return None;
    }
    if amplitude == 0.0 {
        return Some(Interval::point(0.0));
    }
    let first = harmonic_value(a, b, lo)?;
    let second = harmonic_value(a, b, hi)?;
    let mut range = union(first, second);
    let Some(phase) = harmonic_maximum_phase(a, b) else {
        return Some(Interval::new(-amplitude, amplitude));
    };
    if periodic_phase_intersects(lo, hi, phase)? {
        range = Interval::new(range.lo(), range.hi().max(amplitude));
    }
    let minimum_phase = phase
        + Interval::new(
            core::f64::consts::PI.next_down(),
            core::f64::consts::PI.next_up(),
        );
    if periodic_phase_intersects(lo, hi, minimum_phase)? {
        range = Interval::new(range.lo().min(-amplitude), range.hi());
    }
    finite_interval(range)
}

fn harmonic_value(a: Interval, b: Interval, parameter: f64) -> Option<Interval> {
    let (sine, cosine) = math::sincos(parameter);
    if !sine.is_finite() || !cosine.is_finite() {
        return None;
    }
    let sine = Interval::new(sine.next_down(), sine.next_up());
    let cosine = Interval::new(cosine.next_down(), cosine.next_up());
    finite_interval(a * cosine + b * sine)
}

/// Enclose the phase of every coefficient vector represented by `a × b`.
fn harmonic_maximum_phase(a: Interval, b: Interval) -> Option<Interval> {
    let a_center = 0.5 * a.lo() + 0.5 * a.hi();
    let b_center = 0.5 * b.lo() + 0.5 * b.hi();
    if !a_center.is_finite()
        || !b_center.is_finite()
        || !a.contains(a_center)
        || !b.contains(b_center)
    {
        return None;
    }
    let a_error = interval_abs_upper(a - Interval::point(a_center));
    let b_error = interval_abs_upper(b - Interval::point(b_center));
    let error = (Interval::point(a_error).square() + Interval::point(b_error).square())
        .sqrt()?
        .hi();
    let center_norm =
        (Interval::point(a_center).square() + Interval::point(b_center).square()).sqrt()?;
    let parallel_lower = (center_norm.lo() - error).next_down();
    if !error.is_finite() || !parallel_lower.is_finite() || parallel_lower <= 0.0 {
        return None;
    }
    let phase = math::atan2(b_center, a_center);
    let uncertainty = math::atan2(error, parallel_lower).next_up().next_up();
    if !phase.is_finite() || !uncertainty.is_finite() {
        return None;
    }
    Some(Interval::new(
        (phase - uncertainty).next_down().next_down(),
        (phase + uncertainty).next_up().next_up(),
    ))
}

fn periodic_phase_intersects(lo: f64, hi: f64, phase: Interval) -> Option<bool> {
    if !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
        || !phase.lo().is_finite()
        || !phase.hi().is_finite()
    {
        return None;
    }
    let period = Interval::new(
        core::f64::consts::TAU.next_down(),
        core::f64::consts::TAU.next_up(),
    );
    let turns = (Interval::new(lo, hi) - phase).checked_div(period)?;
    const EXACT_INTEGER_LIMIT: f64 = (1_u64 << 52) as f64;
    if turns.lo().abs() > EXACT_INTEGER_LIMIT || turns.hi().abs() > EXACT_INTEGER_LIMIT {
        return None;
    }
    Some(turns.lo().ceil() <= turns.hi().floor())
}

fn circle_matches_cylinder(circle: kgeom::curve::Circle, cylinder: Cylinder) -> bool {
    if circle.radius().to_bits() != cylinder.radius().to_bits() {
        return false;
    }
    let scale = point_scale(circle.frame().origin())
        .max(point_scale(cylinder.frame().origin()))
        .max(circle.radius());
    let Some(guard) = arithmetic_guard(scale) else {
        return false;
    };
    let center_offset = circle.frame().origin() - cylinder.frame().origin();
    [
        dot_interval(cylinder.frame().x(), center_offset),
        dot_interval(cylinder.frame().y(), center_offset),
        dot_interval(circle.frame().x(), cylinder.frame().z()),
        dot_interval(circle.frame().y(), cylinder.frame().z()),
    ]
    .into_iter()
    .all(|value| value.is_some_and(|interval| interval_abs_upper(interval) <= guard))
}

fn cylinder_patch_arithmetic_guard(
    patch: &RectangularCylinderPatch,
    plane_origin: Point3,
) -> Option<f64> {
    arithmetic_guard(
        point_scale(patch.cylinder.frame().origin())
            .max(point_scale(plane_origin))
            .max(patch.cylinder.radius())
            .max(patch.domain.v.lo.abs())
            .max(patch.domain.v.hi.abs()),
    )
}

fn cylinder_patches_arithmetic_guard(
    patches: &[RectangularCylinderPatch],
    plane_origin: Point3,
) -> Option<f64> {
    patches.iter().try_fold(0.0_f64, |guard, patch| {
        cylinder_patch_arithmetic_guard(patch, plane_origin).map(|next| guard.max(next))
    })
}

fn arithmetic_guard(scale: f64) -> Option<f64> {
    if !scale.is_finite() || scale < 0.0 {
        return None;
    }
    let guard = FLOATING_PROOF_GUARD * (1.0 + scale);
    guard.is_finite().then_some(guard.next_up())
}

fn point_scale(point: Point3) -> f64 {
    point.x.abs().max(point.y.abs()).max(point.z.abs())
}

fn interval_abs_upper(interval: Interval) -> f64 {
    interval.lo().abs().max(interval.hi().abs())
}

fn radial_squared_interval(cylinder: Cylinder, point: Point3) -> Option<Interval> {
    let offset = point - cylinder.frame().origin();
    let x = dot_interval(cylinder.frame().x(), offset)?;
    let y = dot_interval(cylinder.frame().y(), offset)?;
    finite_interval(x.square() + y.square())
}

fn affine_value(normal: Vec3, point: Point3, origin: Point3) -> Option<Interval> {
    if !finite_vec(normal) || !finite_point(point) || !finite_point(origin) {
        return None;
    }
    dot_interval(normal, point - origin)
}

fn dot_interval(left: Vec3, right: Vec3) -> Option<Interval> {
    if !finite_vec(left) || !finite_vec(right) {
        return None;
    }
    finite_interval(
        Interval::point(left.x) * Interval::point(right.x)
            + Interval::point(left.y) * Interval::point(right.y)
            + Interval::point(left.z) * Interval::point(right.z),
    )
}

fn union(first: Interval, second: Interval) -> Interval {
    Interval::new(first.lo().min(second.lo()), first.hi().max(second.hi()))
}

fn finite_interval(interval: Interval) -> Option<Interval> {
    (interval.lo().is_finite() && interval.hi().is_finite()).then_some(interval)
}

fn finite_point(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn finite_vec(vector: Vec3) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
}

fn same_point_bits(first: Point3, second: Point3) -> bool {
    first.x.to_bits() == second.x.to_bits()
        && first.y.to_bits() == second.y.to_bits()
        && first.z.to_bits() == second.z.to_bits()
}

fn same_vec_bits(first: Vec3, second: Vec3) -> bool {
    first.x.to_bits() == second.x.to_bits()
        && first.y.to_bits() == second.y.to_bits()
        && first.z.to_bits() == second.z.to_bits()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic_shell::tests::half_cylinder_input;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellCurve, AnalyticShellEdge,
        AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
        AnalyticShellPcurve, AnalyticShellSurface, AnalyticShellVertex, AnalyticVertexKey,
    };
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::transaction::FullCommitRequirement;
    use kgeom::curve::{Circle, Curve, Line};
    use kgeom::curve2d::{Circle2d, Line2d};
    use kgeom::frame::Frame;
    use kgeom::param::ParamRange;
    use kgeom::surface::Plane;
    use kgeom::vec::Vec2;
    use kgraph::AffineParamMap1d;

    fn cylinder_face(store: &Store, shell_id: ShellId) -> FaceId {
        store
            .get(shell_id)
            .unwrap()
            .faces
            .iter()
            .copied()
            .find(|&face_id| {
                let face = store.get(face_id).unwrap();
                matches!(store.get(face.surface).unwrap(), SurfaceGeom::Cylinder(_))
            })
            .unwrap()
    }

    fn session_with_limit(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            CONVEX_CYLINDRICAL_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        kcore::operation::SessionPolicy::new(
            kcore::operation::SessionPrecision::parasolid(),
            kcore::operation::NumericalPolicy::v1(),
            kcore::operation::ExecutionPolicy::Serial,
            budget,
            kcore::operation::PolicyVersion::V1,
        )
    }

    fn parameter_map(scale: f64) -> AffineParamMap1d {
        AffineParamMap1d::new(scale, 0.0).unwrap()
    }

    fn cylinder_ruling_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        longitude: f64,
    ) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn cylinder_arc_use(edge: AnalyticEdgeKey, sense: Sense, height: f64) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn plane_line_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        plane: Plane,
        line: Line,
    ) -> AnalyticShellFin {
        let origin = plane.frame().to_local(line.origin());
        let direction = line.dir();
        let local_direction = Vec2::new(
            direction.dot(plane.frame().x()),
            direction.dot(plane.frame().y()),
        );
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(origin.x, origin.y), local_direction).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn plane_circle_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        plane: Plane,
        circle: Circle,
    ) -> AnalyticShellFin {
        let center = plane.frame().to_local(circle.frame().origin());
        let circle_x = circle.frame().x();
        let circle_y = circle.frame().y();
        let local_x = Vec2::new(
            circle_x.dot(plane.frame().x()),
            circle_x.dot(plane.frame().y()),
        );
        let local_y = Vec2::new(
            circle_y.dot(plane.frame().x()),
            circle_y.dot(plane.frame().y()),
        );
        let orientation = local_x.perp().dot(local_y);
        let scale = if orientation > 0.0 { 1.0 } else { -1.0 };
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Circle(
                    Circle2d::new(Point2::new(center.x, center.y), circle.radius(), local_x)
                        .unwrap(),
                ),
                parameter_map(scale),
            ),
        )
    }

    fn line_edge(
        key: u64,
        vertices: [u64; 2],
        start: Point3,
        end: Point3,
    ) -> (AnalyticShellEdge, Line) {
        let displacement = end - start;
        let line = Line::new(start, displacement).unwrap();
        (
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Line(line),
                ParamRange::new(0.0, displacement.norm()),
            ),
            line,
        )
    }

    /// Cylinder clipped by four planar halfspaces (`z` bounds and a finite
    /// central `x` slab).  The surviving cylindrical boundary has two
    /// disjoint rectangular longitude charts; no layout tag is retained in
    /// the assembled representation.
    fn two_patch_clipped_cylinder_input() -> AnalyticShellInput {
        let radius = 2.0;
        let bottom_frame = Frame::world();
        let top_frame = bottom_frame.with_origin(Point3::new(0.0, 0.0, 1.0));
        let cylinder = Cylinder::new(bottom_frame, radius).unwrap();
        let bottom_circle = Circle::new(bottom_frame, radius).unwrap();
        let top_circle = Circle::new(top_frame, radius).unwrap();
        let longitudes = [
            core::f64::consts::PI / 3.0,
            2.0 * core::f64::consts::PI / 3.0,
            4.0 * core::f64::consts::PI / 3.0,
            5.0 * core::f64::consts::PI / 3.0,
        ];
        let bottom = longitudes.map(|longitude| bottom_circle.eval(longitude));
        let top = longitudes.map(|longitude| top_circle.eval(longitude));
        let vertices = bottom
            .into_iter()
            .chain(top)
            .enumerate()
            .map(|(index, position)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), position)
            })
            .collect::<Vec<_>>();

        let arc_ranges = [
            ParamRange::new(longitudes[0], longitudes[1]),
            ParamRange::new(longitudes[2], longitudes[3]),
        ];
        let mut edges = vec![
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(0),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(1)],
                AnalyticShellCurve::Circle(bottom_circle),
                arc_ranges[0],
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(1),
                [AnalyticVertexKey::new(4), AnalyticVertexKey::new(5)],
                AnalyticShellCurve::Circle(top_circle),
                arc_ranges[0],
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(4),
                [AnalyticVertexKey::new(2), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Circle(bottom_circle),
                arc_ranges[1],
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(5),
                [AnalyticVertexKey::new(6), AnalyticVertexKey::new(7)],
                AnalyticShellCurve::Circle(top_circle),
                arc_ranges[1],
            ),
        ];
        let (ruling_0, line_2) = line_edge(2, [0, 4], bottom[0], top[0]);
        let (ruling_1, line_3) = line_edge(3, [1, 5], bottom[1], top[1]);
        let (ruling_2, line_6) = line_edge(6, [2, 6], bottom[2], top[2]);
        let (ruling_3, line_7) = line_edge(7, [3, 7], bottom[3], top[3]);
        let (bottom_negative, line_8) = line_edge(8, [1, 2], bottom[1], bottom[2]);
        let (top_negative, line_9) = line_edge(9, [5, 6], top[1], top[2]);
        let (bottom_positive, line_10) = line_edge(10, [3, 0], bottom[3], bottom[0]);
        let (top_positive, line_11) = line_edge(11, [7, 4], top[3], top[0]);
        edges.extend([
            ruling_0,
            ruling_1,
            ruling_2,
            ruling_3,
            bottom_negative,
            top_negative,
            bottom_positive,
            top_positive,
        ]);

        let bottom_plane = Plane::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let top_plane = Plane::new(top_frame);
        let positive_plane = Plane::new(
            Frame::new(
                bottom[0],
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let negative_plane = Plane::new(
            Frame::new(
                bottom[1],
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
            )
            .unwrap(),
        );

        let patch_a = AnalyticShellLoop::new(vec![
            cylinder_ruling_use(AnalyticEdgeKey::new(2), Sense::Reversed, longitudes[0]),
            cylinder_arc_use(AnalyticEdgeKey::new(0), Sense::Forward, 0.0),
            cylinder_ruling_use(AnalyticEdgeKey::new(3), Sense::Forward, longitudes[1]),
            cylinder_arc_use(AnalyticEdgeKey::new(1), Sense::Reversed, 1.0),
        ]);
        let patch_b = AnalyticShellLoop::new(vec![
            cylinder_ruling_use(AnalyticEdgeKey::new(6), Sense::Reversed, longitudes[2]),
            cylinder_arc_use(AnalyticEdgeKey::new(4), Sense::Forward, 0.0),
            cylinder_ruling_use(AnalyticEdgeKey::new(7), Sense::Forward, longitudes[3]),
            cylinder_arc_use(AnalyticEdgeKey::new(5), Sense::Reversed, 1.0),
        ]);
        let bottom_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(
                AnalyticEdgeKey::new(0),
                Sense::Reversed,
                bottom_plane,
                bottom_circle,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(10),
                Sense::Reversed,
                bottom_plane,
                line_10,
            ),
            plane_circle_use(
                AnalyticEdgeKey::new(4),
                Sense::Reversed,
                bottom_plane,
                bottom_circle,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(8),
                Sense::Reversed,
                bottom_plane,
                line_8,
            ),
        ]);
        let top_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(
                AnalyticEdgeKey::new(5),
                Sense::Forward,
                top_plane,
                top_circle,
            ),
            plane_line_use(AnalyticEdgeKey::new(11), Sense::Forward, top_plane, line_11),
            plane_circle_use(
                AnalyticEdgeKey::new(1),
                Sense::Forward,
                top_plane,
                top_circle,
            ),
            plane_line_use(AnalyticEdgeKey::new(9), Sense::Forward, top_plane, line_9),
        ]);
        let positive_loop = AnalyticShellLoop::new(vec![
            plane_line_use(
                AnalyticEdgeKey::new(10),
                Sense::Forward,
                positive_plane,
                line_10,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(2),
                Sense::Forward,
                positive_plane,
                line_2,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(11),
                Sense::Reversed,
                positive_plane,
                line_11,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(7),
                Sense::Reversed,
                positive_plane,
                line_7,
            ),
        ]);
        let negative_loop = AnalyticShellLoop::new(vec![
            plane_line_use(
                AnalyticEdgeKey::new(8),
                Sense::Forward,
                negative_plane,
                line_8,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(6),
                Sense::Forward,
                negative_plane,
                line_6,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(9),
                Sense::Reversed,
                negative_plane,
                line_9,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(3),
                Sense::Reversed,
                negative_plane,
                line_3,
            ),
        ]);

        let faces = vec![
            AnalyticShellFace::new(
                AnalyticFaceKey::new(0),
                AnalyticShellSurface::Cylinder(cylinder),
                Sense::Forward,
                FaceDomain::from_bounds(longitudes[0], longitudes[1], 0.0, 1.0).unwrap(),
                vec![patch_a],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(1),
                AnalyticShellSurface::Cylinder(cylinder),
                Sense::Forward,
                FaceDomain::from_bounds(longitudes[2], longitudes[3], 0.0, 1.0).unwrap(),
                vec![patch_b],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(2),
                AnalyticShellSurface::Plane(bottom_plane),
                Sense::Forward,
                FaceDomain::from_bounds(-radius, radius, -radius, radius).unwrap(),
                vec![bottom_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(3),
                AnalyticShellSurface::Plane(top_plane),
                Sense::Forward,
                FaceDomain::from_bounds(-radius, radius, -radius, radius).unwrap(),
                vec![top_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(4),
                AnalyticShellSurface::Plane(positive_plane),
                Sense::Forward,
                FaceDomain::from_bounds(-2.0 * radius, radius, 0.0, 1.0).unwrap(),
                vec![positive_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(5),
                AnalyticShellSurface::Plane(negative_plane),
                Sense::Forward,
                FaceDomain::from_bounds(-radius, 2.0 * radius, 0.0, 1.0).unwrap(),
                vec![negative_loop],
            ),
        ];
        AnalyticShellInput::new(vertices, edges, faces)
    }

    #[test]
    fn harmonic_range_keeps_unit_circle_correlation_and_inserts_true_extrema() {
        let a = Interval::new(3.0_f64.next_down(), 3.0_f64.next_up());
        let b = Interval::new(4.0_f64.next_down(), 4.0_f64.next_up());

        // On [0, pi/4], -3 sin(u) + 4 cos(u) stays positive, so the
        // mathematical maximum is 7/sqrt(2) < 5. Independent cosine/sine
        // boxes would instead admit the impossible value 3 + 4/sqrt(2).
        let monotone = harmonic_range(a, b, 0.0, core::f64::consts::FRAC_PI_4).unwrap();
        assert!(monotone.contains(3.0));
        assert!(monotone.hi() < 5.0, "correlation was lost: {monotone:?}");

        // The coefficient direction lies strictly inside the first quadrant,
        // where Cauchy-Schwarz gives the exact maximum sqrt(3^2 + 4^2) = 5.
        let critical = harmonic_range(a, b, 0.0, core::f64::consts::FRAC_PI_2).unwrap();
        assert!(
            critical.contains(5.0),
            "missed interior maximum: {critical:?}"
        );

        let antipode = harmonic_range(
            a,
            b,
            core::f64::consts::PI,
            core::f64::consts::PI + core::f64::consts::FRAC_PI_2,
        )
        .unwrap();
        assert!(
            antipode.contains(-5.0),
            "missed interior minimum: {antipode:?}"
        );
    }

    #[test]
    fn structural_half_cylinder_satisfies_every_convex_constraint() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
            .unwrap();
        let shell = transaction.store().get(output.shell()).unwrap();
        let (cylinder_face, cylinder) = shell
            .faces
            .iter()
            .find_map(|&face_id| {
                let face = transaction.store().get(face_id).unwrap();
                match transaction.store().get(face.surface).unwrap() {
                    SurfaceGeom::Cylinder(cylinder) => Some((face_id, *cylinder)),
                    _ => None,
                }
            })
            .unwrap();
        let planar_faces = shell
            .faces
            .iter()
            .copied()
            .filter(|&face_id| {
                let face = transaction.store().get(face_id).unwrap();
                matches!(
                    transaction.store().get(face.surface).unwrap(),
                    SurfaceGeom::Plane(_)
                )
            })
            .collect::<Vec<_>>();
        let patch = prepare_rectangular_patch(transaction.store(), cylinder_face, cylinder)
            .unwrap()
            .unwrap();
        let patches = vec![patch];
        assert!(certify_face_cells_and_incidence(transaction.store(), output.shell()).unwrap());
        let (witness, supports) =
            find_strict_witness_and_planar_supports(transaction.store(), &patches, &planar_faces)
                .unwrap()
                .unwrap();
        assert!(strictly_inside_cylinder(cylinder, witness));

        let mut edges = Vec::new();
        for &face_id in &shell.faces {
            for &loop_id in &transaction.store().get(face_id).unwrap().loops {
                for &fin_id in &transaction.store().get(loop_id).unwrap().fins {
                    let edge = transaction.store().get(fin_id).unwrap().edge;
                    if !edges.contains(&edge) {
                        edges.push(edge);
                    }
                }
            }
        }
        for edge_id in edges {
            let edge = transaction.store().get(edge_id).unwrap();
            let curve = transaction.store().get(edge.curve.unwrap()).unwrap();
            let (lo, hi) = edge.bounds.unwrap();
            for support in &supports {
                let range = if let Some(line) = exact_line_carrier(curve) {
                    union(
                        affine_value(support.outward, line.eval(lo), support.origin).unwrap(),
                        affine_value(support.outward, line.eval(hi), support.origin).unwrap(),
                    )
                } else if let CurveGeom::Circle(circle) = curve {
                    circle_affine_range(*circle, lo, hi, support.outward, support.origin).unwrap()
                } else {
                    panic!("unsupported test curve: {curve:?}")
                };
                assert!(
                    range.hi() <= LINEAR_RESOLUTION,
                    "edge {edge_id:?}, support {:?}, range {range:?}",
                    support.face
                );
            }
        }
        assert!(
            all_boundary_traces_satisfy_constraints(
                transaction.store(),
                output.shell(),
                cylinder,
                &patches,
                &supports,
            )
            .unwrap()
        );
    }

    #[test]
    fn two_disjoint_cylinder_charts_are_full_valid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&two_patch_clipped_cylinder_input(), 1.0e-12)
            .unwrap();
        let cylinder_faces = transaction
            .store()
            .get(output.shell())
            .unwrap()
            .faces
            .iter()
            .filter(|&&face_id| {
                let face = transaction.store().get(face_id).unwrap();
                matches!(
                    transaction.store().get(face.surface).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .count();
        assert_eq!(cylinder_faces, 2);
        assert_eq!(
            certify_convex_cylindrical_shell(transaction.store(), output.shell(), None)
                .unwrap()
                .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );

        let report =
            check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
        assert!(report.faults.is_empty(), "{report:#?}");
        assert!(report.gaps.is_empty(), "{report:#?}");
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
    }

    #[test]
    fn overlapping_or_nonidentical_cylinder_chart_tampering_fails_closed() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&two_patch_clipped_cylinder_input(), 1.0e-12)
            .unwrap();
        let cylinder_faces = transaction
            .store()
            .get(output.shell())
            .unwrap()
            .faces
            .iter()
            .copied()
            .filter(|&face_id| {
                let face = transaction.store().get(face_id).unwrap();
                matches!(
                    transaction.store().get(face.surface).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(cylinder_faces.len(), 2);

        let mut overlap = transaction.store().clone();
        let first_domain = overlap.get(cylinder_faces[0]).unwrap().domain.unwrap();
        overlap.get_mut(cylinder_faces[1]).unwrap().domain = Some(first_domain);
        assert_eq!(
            certify_convex_cylindrical_shell(&overlap, output.shell(), None)
                .unwrap()
                .unwrap(),
            indeterminate(),
            "overlapping chart declarations must never certify a duplicated cylindrical sheet"
        );

        let mut nonidentical_store = transaction.store().clone();
        let mut nonidentical = nonidentical_store.transaction().unwrap();
        let second_surface = nonidentical.store().get(cylinder_faces[1]).unwrap().surface;
        let SurfaceGeom::Cylinder(second) = *nonidentical.store().get(second_surface).unwrap()
        else {
            unreachable!()
        };
        nonidentical
            .store_mut()
            .replace_surface(
                second_surface,
                SurfaceGeom::Cylinder(
                    Cylinder::new(
                        second
                            .frame()
                            .with_origin(second.frame().origin() + second.frame().z()),
                        second.radius(),
                    )
                    .unwrap(),
                ),
            )
            .unwrap();
        assert_eq!(
            certify_convex_cylindrical_shell(nonidentical.store(), output.shell(), None)
                .unwrap()
                .unwrap(),
            indeterminate(),
            "cylinder faces without one exact analytic representation must fail closed"
        );
    }

    #[test]
    fn structural_certificate_rejects_chart_extent_and_orientation_mutations() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_convex_cylindrical_shell(transaction.store(), output.shell(), None)
                .unwrap()
                .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );

        let mut bad_extent = transaction.store().clone();
        let side = cylinder_face(&bad_extent, output.shell());
        let domain = bad_extent.get(side).unwrap().domain.unwrap();
        bad_extent.get_mut(side).unwrap().domain = Some(
            FaceDomain::from_bounds(
                domain.u.lo,
                domain.u.hi.next_down(),
                domain.v.lo,
                domain.v.hi,
            )
            .unwrap(),
        );
        assert_eq!(
            certify_convex_cylindrical_shell(&bad_extent, output.shell(), None)
                .unwrap()
                .unwrap(),
            indeterminate(),
            "a face-domain declaration that no longer equals the live pcurve perimeter must fail closed"
        );

        let mut wrong_sense = transaction.store().clone();
        let planar_face = wrong_sense
            .get(output.shell())
            .unwrap()
            .faces
            .iter()
            .copied()
            .find(|&face_id| face_id != side)
            .unwrap();
        wrong_sense.get_mut(planar_face).unwrap().sense =
            wrong_sense.get(planar_face).unwrap().sense.flipped();
        assert_eq!(
            certify_convex_cylindrical_shell(&wrong_sense, output.shell(), None)
                .unwrap()
                .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            },
            "one wrong-facing support is an orientation fault, not an embedding guess"
        );
    }

    #[test]
    fn convex_cylindrical_work_budget_accepts_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&two_patch_clipped_cylinder_input(), 1.0e-12)
            .unwrap();

        let default_session = session_with_limit(DEFAULT_CONVEX_CYLINDRICAL_SHELL_WORK);
        let default_context = kcore::operation::OperationContext::new(
            &default_session,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut default_scope = OperationScope::new(&default_context);
        assert_eq!(
            certify_convex_cylindrical_shell(
                transaction.store(),
                output.shell(),
                Some(&mut default_scope),
            )
            .unwrap()
            .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );
        let required = default_scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == CONVEX_CYLINDRICAL_SHELL_WORK)
            .unwrap()
            .consumed;
        assert!(required > 0);

        let exact_session = session_with_limit(required);
        let exact_context = kcore::operation::OperationContext::new(
            &exact_session,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut exact_scope = OperationScope::new(&exact_context);
        assert_eq!(
            certify_convex_cylindrical_shell(
                transaction.store(),
                output.shell(),
                Some(&mut exact_scope),
            )
            .unwrap()
            .unwrap()
            .embedding,
            ShellEmbedding::Certified
        );

        let short_session = session_with_limit(required - 1);
        let short_context = kcore::operation::OperationContext::new(
            &short_session,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut short_scope = OperationScope::new(&short_context);
        let error = certify_convex_cylindrical_shell(
            transaction.store(),
            output.shell(),
            Some(&mut short_scope),
        )
        .unwrap_err();
        assert_eq!(
            error.limit().map(|limit| limit.stage),
            Some(CONVEX_CYLINDRICAL_SHELL_WORK)
        );
    }
}
