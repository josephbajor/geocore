use super::circle_sphere::intersect_bounded_circle_sphere;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::parameter::{
    PeriodicOverlapPiece, affine_preimage_overlap, fit_scalar_parameter,
    periodic_preimage_overlaps, range_midpoint, validate_period_span,
};
use super::result::{
    ArbitrarySphereOctantMap, ContactKind, GeneralSphereWindowMap, OrthogonalSphereOctantMap,
    SurfaceIntersectionCurve, SurfaceRegionCorrespondence, SurfaceRegionOrientation,
    SurfaceSurfaceCurve, SurfaceSurfaceIntersections, SurfaceSurfacePoint, SurfaceSurfaceRegion,
    SurfaceSurfaceRegionVertex, accept_surface_surface_candidate,
};
use super::support_curve_pair::{SupportCurvePairConfig, emit_support_curve_pair};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::predicates::{Orientation, orient3d};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect two finite sphere parameter windows.
pub fn intersect_bounded_spheres(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let delta = b.frame().origin() - a.frame().origin();
    let distance = delta.norm();
    let radius_a = a.radius();
    let radius_b = b.radius();
    if distance <= tolerances.linear() {
        if (radius_a - radius_b).abs() <= tolerances.linear() {
            if a.frame().origin() == b.frame().origin() && radius_a == radius_b {
                if compare_sphere_windows(a, a_range, b, b_range).is_gt() {
                    return intersect_exact_coincident_sphere_windows(
                        b, b_range, a, a_range, tolerances,
                    )
                    .map(SurfaceSurfaceIntersections::swapped);
                }
                return intersect_exact_coincident_sphere_windows(
                    a, a_range, b, b_range, tolerances,
                );
            }
            return Err(Error::InvalidGeometry {
                reason: "near-coincident non-identical spheres require the general certified fallback",
            });
        }
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    if distance > radius_a + radius_b + tolerances.linear()
        || distance < (radius_a - radius_b).abs() - tolerances.linear()
    {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let axis = delta / distance;
    let center_offset =
        (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance);
    let circle_radius_sq = radius_a * radius_a - center_offset * center_offset;
    let sq_tol = squared_tolerance(distance, radius_a, radius_b, tolerances);
    if circle_radius_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }
    if circle_radius_sq <= sq_tol {
        let point = tangent_point(a.frame().origin(), axis, center_offset, radius_a);
        let mut points = Vec::new();
        add_point(
            &mut points,
            point,
            a,
            a_range,
            b,
            b_range,
            ContactKind::Tangent,
            tolerances,
        );
        return SurfaceSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let circle_center = a.frame().origin() + axis * center_offset;
    let circle = Circle::new(
        Frame::from_z(circle_center, axis)?,
        circle_radius_sq.max(0.0).sqrt(),
    )?;
    let a_hit =
        intersect_bounded_circle_sphere(&circle, circle.param_range(), a, a_range, tolerances)?;
    let b_hit =
        intersect_bounded_circle_sphere(&circle, circle.param_range(), b, b_range, tolerances)?;

    let parameter_tolerance = parameter_tolerance(circle.radius(), tolerances);
    let mut points = Vec::new();
    let mut curves = Vec::new();
    let curve = SurfaceIntersectionCurve::Circle(circle);
    let first_uv = |point| sphere_uv_at(point, a, a_range, tolerances);
    let second_uv = |point| sphere_uv_at(point, b, b_range, tolerances);
    emit_support_curve_pair(
        SupportCurvePairConfig {
            curve: &curve,
            curve_range: curve.param_range(),
            first_hit: &a_hit,
            second_hit: &b_hit,
            kind: ContactKind::Transverse,
            parameter_tolerance,
            parameter_period: Some(core::f64::consts::TAU),
            branch_tolerance: parameter_tolerance,
            first_surface: a,
            second_surface: b,
            first_uv: &first_uv,
            second_uv: &second_uv,
            tolerances,
        },
        &mut points,
        &mut curves,
    );

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn intersect_exact_coincident_sphere_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    if !sphere_planes_are_exactly_parallel(a.frame().z(), b.frame().z()) {
        intersect_orthogonal_sphere_octants(a, a_range, b, b_range, tolerances)
    } else {
        intersect_coincident_sphere_windows(a, a_range, b, b_range, tolerances)
    }
}

fn compare_sphere_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
) -> core::cmp::Ordering {
    let a_values = a
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .chain(a.frame().z().to_array())
        .chain(a.frame().x().to_array())
        .chain([
            a.radius(),
            a_range[0].lo,
            a_range[0].hi,
            a_range[1].lo,
            a_range[1].hi,
        ]);
    let b_values = b
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .chain(b.frame().z().to_array())
        .chain(b.frame().x().to_array())
        .chain([
            b.radius(),
            b_range[0].lo,
            b_range[0].hi,
            b_range[1].lo,
            b_range[1].hi,
        ]);
    a_values
        .zip(b_values)
        .map(|(a, b)| a.total_cmp(&b))
        .find(|ordering| !ordering.is_eq())
        .unwrap_or(core::cmp::Ordering::Equal)
}

#[derive(Clone, Copy, Debug)]
struct SignedCoordinateAxis {
    coordinate: usize,
    sign: f64,
}

fn intersect_orthogonal_sphere_octants(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_coincident_sphere_ranges(a_range, b_range)?;
    let (Some(a_signs), Some(b_local_signs)) = (
        exact_sphere_octant_signs(a_range, tolerances),
        exact_sphere_octant_signs(b_range, tolerances),
    ) else {
        let (pair_limit, arc_limit) = match (
            a_range[0].width() >= core::f64::consts::PI,
            b_range[0].width() >= core::f64::consts::PI,
        ) {
            (true, true) => (
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
            ),
            (true, false) | (false, true)
                if exact_general_sphere_window_pole(a_range).is_some()
                    ^ exact_general_sphere_window_pole(b_range).is_some() =>
            {
                (
                    GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                )
            }
            (true, false) | (false, true) => (
                GENERAL_SPHERE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_WIDE_ARC_LIMIT,
            ),
            (false, false)
                if exact_general_sphere_window_pole(a_range).is_some()
                    ^ exact_general_sphere_window_pole(b_range).is_some() =>
            {
                (
                    GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT,
                )
            }
            (false, false) => (
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                GENERAL_SPHERE_WINDOW_ARC_LIMIT,
            ),
        };
        return intersect_certified_general_sphere_windows(
            a, a_range, b, b_range, tolerances, pair_limit, arc_limit,
        );
    };
    let Some(axis_map) = exact_signed_coordinate_axis_map(a, b) else {
        return intersect_arbitrary_sphere_octants(
            a,
            a_range,
            b,
            b_range,
            a_signs,
            b_local_signs,
            tolerances,
        );
    };
    let mut b_signs = [0.0; 3];
    for local_axis in 0..3 {
        let mapped = axis_map[local_axis];
        b_signs[mapped.coordinate] = b_local_signs[local_axis] * mapped.sign;
    }
    let differing = (0..3)
        .filter(|&axis| a_signs[axis] != b_signs[axis])
        .collect::<Vec<_>>();
    match differing.len() {
        0 => coincident_orthogonal_sphere_octant_region(
            a, a_range, b, b_range, axis_map, a_signs, tolerances,
        ),
        1 => coincident_orthogonal_sphere_octant_edge(
            a,
            a_range,
            b,
            b_range,
            a_signs,
            differing[0],
            tolerances,
        ),
        2 => coincident_orthogonal_sphere_octant_vertex(
            a, a_range, b, b_range, a_signs, b_signs, tolerances,
        ),
        3 => Ok(SurfaceSurfaceIntersections::complete_empty()),
        _ => unreachable!("three coordinate signs have at most three differences"),
    }
}

fn unsupported_nonparallel_sphere_charts() -> Result<SurfaceSurfaceIntersections> {
    Err(Error::InvalidGeometry {
        reason: "coincident sphere charts with nonparallel latitude axes require the general certified fallback",
    })
}

const GENERAL_SPHERE_WINDOW_PAIR_LIMIT: usize = 28;
// Eight boundary circles meet at most fourteen roots each. Sampling every
// open arrangement arc therefore consumes at most 8 * 14 fixed witnesses.
const GENERAL_SPHERE_WINDOW_ARC_LIMIT: usize = 112;
// One exact polar window contributes two longitude constraints and one
// nondegenerate latitude constraint. Paired with one pole-clear window, the
// seven boundary circles have at most 21 pairs and 84 open arrangement arcs.
const GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT: usize = 21;
const GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT: usize = 84;
const GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT: usize = 2;
const GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT: usize =
    GENERAL_SPHERE_WINDOW_PAIR_LIMIT + GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT;
const GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT: usize =
    GENERAL_SPHERE_WINDOW_ARC_LIMIT + GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT;
const GENERAL_SPHERE_WIDE_PIECE_LIMIT: usize = 3;
const GENERAL_SPHERE_WIDE_PAIR_LIMIT: usize =
    GENERAL_SPHERE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_PAIR_LIMIT;
const GENERAL_SPHERE_WIDE_ARC_LIMIT: usize =
    GENERAL_SPHERE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_ARC_LIMIT;
const GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT: usize =
    GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT * GENERAL_SPHERE_WIDE_PIECE_LIMIT;
const GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT: usize = GENERAL_SPHERE_WIDE_PIECE_LIMIT
    * (GENERAL_SPHERE_WINDOW_PAIR_LIMIT + GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT);
const GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT: usize = GENERAL_SPHERE_WIDE_PIECE_LIMIT
    * (GENERAL_SPHERE_WINDOW_ARC_LIMIT + GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT);
const GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON: &str = "general coincident sphere polar-by-wide union supports one occupied child, one exact adjacent same-row pair, one exact adjacent same-column pair, one exact mixed-axis three-cell path, one exact full latitude-row path, one exact connected four-cell shared-seam path, T-shaped tree, or 2x2 cycle with two certified-empty siblings, one exact disconnected outer-column vertical-pair layout or singleton-plus-three-cell path separated by two certified-empty cut siblings, one exact five-cell simultaneous shared-seam union with one certified-empty sibling, or the exact simultaneous six-cell union with no empty sibling";
type PolarWideParameterSeam = (bool, usize, f64);
type PolarWideFourAdjacency = [[Option<PolarWideParameterSeam>; 4]; 4];
const GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT: usize =
    GENERAL_SPHERE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WIDE_PIECE_LIMIT;
const GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT: usize =
    GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_PAIR_LIMIT;
const GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT: usize =
    GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_ARC_LIMIT;
const GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT: usize = 9;
const GENERAL_SPHERE_DOUBLE_WIDE_LAYOUT_REASON: &str = "general coincident sphere both-wide union supports at most nine positive cells; three cells require pairwise independence, one exact adjacent pair plus an isolated cell, or an exact shared-seam path; four, six, seven, eight, and nine require an exact connected shared-seam union; five require an exact connected union or exact sibling-separated components";

#[derive(Clone, Copy, Debug, PartialEq)]
struct SphereWindowConstraint {
    normal: Vec3,
    offset: f64,
}

#[derive(Clone, Copy, Debug)]
struct CertifiedSphereBoundaryRoot {
    direction: Vec3,
    enclosure: [Interval; 3],
    active: [usize; 2],
    feasible: bool,
}

#[derive(Clone, Copy, Debug)]
struct CertifiedSphereBoundaryArc {
    first: usize,
    second: usize,
    midpoint: Vec3,
}

#[derive(Debug)]
struct CertifiedSphereBoundaryArrangement {
    feasible_arcs: Vec<CertifiedSphereBoundaryArc>,
    all_boundaries_excluded: bool,
}

#[derive(Debug)]
struct ExactSphereBoundaryLock {
    plane: SphereWindowConstraint,
    representative: usize,
    members: Vec<usize>,
}

#[allow(clippy::too_many_arguments)]
fn intersect_certified_general_sphere_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
    pair_limit: usize,
    arc_limit: usize,
) -> Result<SurfaceSurfaceIntersections> {
    let parameter_allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range)?;
    if parameter_allowance > tolerances.angular() {
        return unsupported_nonparallel_sphere_charts();
    }
    match certify_general_sphere_windows(
        a,
        a_range,
        b,
        b_range,
        tolerances,
        pair_limit,
        arc_limit,
        parameter_allowance,
    ) {
        Ok(hit) => Ok(hit),
        Err(Error::InvalidGeometry { reason }) => {
            Ok(SurfaceSurfaceIntersections::indeterminate_empty(reason))
        }
        Err(error) => Err(error),
    }
}

#[allow(clippy::too_many_arguments)]
fn certify_general_sphere_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
    pair_limit: usize,
    arc_limit: usize,
    parameter_allowance: f64,
) -> Result<SurfaceSurfaceIntersections> {
    validate_general_sphere_window_base(a_range, parameter_allowance)?;
    validate_general_sphere_window_base(b_range, parameter_allowance)?;
    let polar_windows = usize::from(exact_general_sphere_window_pole(a_range).is_some())
        + usize::from(exact_general_sphere_window_pole(b_range).is_some());
    if polar_windows > 0 && polar_windows != 1 {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window proof requires exactly one exact-pole sub-pi window and one pole-clear window",
        });
    }
    if polar_windows == 1 {
        let first_is_polar = exact_general_sphere_window_pole(a_range).is_some();
        let polar_range = if first_is_polar { a_range } else { b_range };
        let peer_range = if first_is_polar { b_range } else { a_range };
        if polar_range[0].width() >= core::f64::consts::PI {
            return Err(Error::InvalidGeometry {
                reason: "general coincident sphere polar-window proof requires exactly one exact-pole sub-pi window and one pole-clear window",
            });
        }
        if peer_range[0].width() >= core::f64::consts::PI {
            return certify_polar_by_wide_sphere_window_union(
                a,
                a_range,
                b,
                b_range,
                first_is_polar,
                tolerances,
                parameter_allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                pair_limit,
                arc_limit,
            );
        }
        return certify_single_polar_sphere_window_union(
            a,
            a_range,
            b,
            b_range,
            first_is_polar,
            tolerances,
            parameter_allowance,
            GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT,
            pair_limit,
            arc_limit,
        );
    }
    match (
        a_range[0].width() >= core::f64::consts::PI,
        b_range[0].width() >= core::f64::consts::PI,
    ) {
        (true, false) => {
            return certify_single_wide_sphere_window_union(
                a,
                a_range,
                b,
                b_range,
                true,
                tolerances,
                parameter_allowance,
                GENERAL_SPHERE_WIDE_PIECE_LIMIT,
                pair_limit,
                arc_limit,
            );
        }
        (false, true) => {
            return certify_single_wide_sphere_window_union(
                a,
                a_range,
                b,
                b_range,
                false,
                tolerances,
                parameter_allowance,
                GENERAL_SPHERE_WIDE_PIECE_LIMIT,
                pair_limit,
                arc_limit,
            );
        }
        (true, true) => {
            return certify_double_wide_sphere_window_union(
                a,
                a_range,
                b,
                b_range,
                tolerances,
                parameter_allowance,
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                pair_limit,
                arc_limit,
            );
        }
        (false, false) => {}
    }
    validate_general_sphere_window_slice(a_range, parameter_allowance)?;
    validate_general_sphere_window_slice(b_range, parameter_allowance)?;

    certify_general_sphere_window_arrangement(
        a,
        a_range,
        b,
        b_range,
        tolerances,
        pair_limit,
        arc_limit,
        parameter_allowance,
    )
}

#[allow(clippy::too_many_arguments)]
fn certify_general_sphere_window_arrangement(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
    pair_limit: usize,
    arc_limit: usize,
    parameter_allowance: f64,
) -> Result<SurfaceSurfaceIntersections> {
    let polar_windows = usize::from(exact_general_sphere_window_pole(a_range).is_some())
        + usize::from(exact_general_sphere_window_pole(b_range).is_some());
    let constraints = general_sphere_window_constraints(a, a_range)?
        .into_iter()
        .chain(general_sphere_window_constraints(b, b_range)?)
        .collect::<Vec<_>>();
    debug_assert_eq!(constraints.len(), 8 - polar_windows);

    let mut remaining_pairs = pair_limit;
    let mut roots = Vec::new();
    for first in 0..constraints.len() {
        for second in first + 1..constraints.len() {
            if remaining_pairs == 0 {
                return Err(Error::InvalidGeometry {
                    reason: "general coincident sphere window proof pair limit exhausted",
                });
            }
            remaining_pairs -= 1;
            roots.extend(certified_sphere_boundary_pair(
                constraints[first],
                constraints[second],
                [first, second],
                tolerances,
            )?);
        }
    }

    if let Some(collapsed) = certify_collapsed_general_sphere_windows(
        a,
        a_range,
        b,
        b_range,
        &constraints,
        &roots,
        tolerances,
        arc_limit,
        parameter_allowance,
    )? {
        return Ok(collapsed);
    }

    for index in 0..roots.len() {
        roots[index].feasible =
            certify_sphere_root_membership(roots[index], &constraints, tolerances)?;
        for other in 0..index {
            if (roots[index].direction - roots[other].direction).norm() <= tolerances.angular() {
                return Err(Error::InvalidGeometry {
                    reason: "general coincident sphere window proof encountered an unresolved multiple boundary vertex",
                });
            }
        }
    }

    let arrangement = certify_sphere_boundary_arcs(&constraints, &roots, tolerances, arc_limit)?;
    if arrangement.all_boundaries_excluded {
        // Pairwise interval discriminants found every crossing of all retained
        // boundary circles. Each circle was partitioned at those crossings,
        // every endpoint has a certified violated halfspace, and one witness
        // on every open arc has a certified violated halfspace. Constraint
        // signs cannot change inside an open arc, so the feasible set has no
        // boundary point. It also cannot be the whole sphere: every retained
        // pole-clear window contributes a longitude halfspace with offset
        // zero, which its negated unit normal strictly violates. A nonempty
        // proper closed subset of the connected sphere has nonempty boundary;
        // therefore the mutual window intersection is empty.
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }
    let arcs = arrangement.feasible_arcs;
    let feasible = roots
        .iter()
        .enumerate()
        .filter_map(|(index, root)| root.feasible.then_some(index))
        .collect::<Vec<_>>();
    certify_single_sphere_boundary_cycle(&feasible, &arcs)?;

    let mut directions = feasible
        .iter()
        .map(|&index| roots[index].direction)
        .collect::<Vec<_>>();
    if directions.len() == 2 {
        directions.extend(arcs.iter().map(|arc| arc.midpoint));
    }
    directions.sort_by(|first, second| compare_sphere_directions(*first, *second));
    directions.dedup_by(|first, second| (*first - *second).norm() <= tolerances.angular());
    if directions.len() < 3
        || !certify_sphere_region_interior(&directions, &constraints, tolerances)?
    {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window fallback did not certify a positive-area single-cycle region",
        });
    }
    sort_arbitrary_sphere_polygon(&mut directions)?;

    let mut max_residual = arbitrary_sphere_octant_residual_bound(a, b, parameter_allowance)?;
    let mut boundary = Vec::with_capacity(directions.len());
    for direction in directions {
        let sample = paired_general_sphere_direction(
            a,
            a_range,
            b,
            b_range,
            direction,
            parameter_allowance,
            tolerances,
        )?;
        max_residual = max_residual.max(sample.residual_bound);
        boundary.push(SurfaceSurfaceRegionVertex {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
        });
    }
    let region = SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: SurfaceRegionCorrespondence::GeneralSphereWindow(
            general_sphere_window_map(a, a_range, b, b_range, parameter_allowance),
        ),
        max_residual,
    };
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![region],
    )
}

#[allow(clippy::too_many_arguments)]
fn certify_single_polar_sphere_window_union(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    first_is_polar: bool,
    tolerances: Tolerances,
    parent_parameter_allowance: f64,
    piece_limit: usize,
    pair_limit: usize,
    arc_limit: usize,
) -> Result<SurfaceSurfaceIntersections> {
    if piece_limit < GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window union piece limit exhausted",
        });
    }
    if pair_limit < GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window union pair limit exhausted",
        });
    }
    if arc_limit < GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window union arc limit exhausted",
        });
    }

    let polar_range = if first_is_polar { a_range } else { b_range };
    let [pole_clear_piece, polar_cap] = decompose_general_sphere_polar_window(polar_range)?;
    let mut occupied = None;
    let mut empty_pieces = 0;
    for (piece_range, piece_pair_limit, piece_arc_limit) in [
        (
            pole_clear_piece,
            GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
            GENERAL_SPHERE_WINDOW_ARC_LIMIT,
        ),
        (
            polar_cap,
            GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
        ),
    ] {
        let (piece_a_range, piece_b_range) = if first_is_polar {
            (piece_range, b_range)
        } else {
            (a_range, piece_range)
        };
        let piece_allowance =
            arbitrary_sphere_octant_parameter_allowance(piece_a_range, piece_b_range)?;
        validate_general_sphere_window_slice(piece_a_range, piece_allowance)?;
        validate_general_sphere_window_slice(piece_b_range, piece_allowance)?;
        let hit = certify_general_sphere_window_arrangement(
            a,
            piece_a_range,
            b,
            piece_b_range,
            tolerances,
            piece_pair_limit,
            piece_arc_limit,
            piece_allowance,
        )?;
        if hit.is_proven_empty() {
            empty_pieces += 1;
        } else if !hit.is_complete() || occupied.replace(hit).is_some() {
            return Err(Error::InvalidGeometry {
                reason: "general coincident sphere polar-window union requires one occupied cell and one certified-empty sibling",
            });
        }
    }

    let Some(mut hit) = occupied else {
        if empty_pieces == GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT {
            return Ok(SurfaceSurfaceIntersections::complete_empty());
        }
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window union did not cover both decomposition cells",
        });
    };
    if empty_pieces != 1 {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window union did not cancel its artificial latitude seam",
        });
    }

    // Both latitude cells are closed. The certified-empty sibling proves that
    // the occupied cell cannot touch the artificial latitude seam, so its
    // evidence has only true parent boundaries. A retained singular pole is
    // represented once at the polar source range's lower longitude; the
    // nonlinear map applies the same canonical alias in both directions.
    let parent_map = general_sphere_window_map(a, a_range, b, b_range, parent_parameter_allowance);
    let parent_residual = arbitrary_sphere_octant_residual_bound(a, b, parent_parameter_allowance)?;
    for region in &mut hit.regions {
        region.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
        region.max_residual = region.max_residual.max(parent_residual);
    }
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        hit.points,
        hit.curves,
        hit.regions,
    )
}

#[allow(clippy::too_many_arguments)]
fn certify_polar_by_wide_sphere_window_union(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    first_is_polar: bool,
    tolerances: Tolerances,
    parent_parameter_allowance: f64,
    piece_limit: usize,
    pair_limit: usize,
    arc_limit: usize,
) -> Result<SurfaceSurfaceIntersections> {
    if piece_limit < GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-by-wide union piece limit exhausted",
        });
    }
    if pair_limit < GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-by-wide union pair limit exhausted",
        });
    }
    if arc_limit < GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-by-wide union arc limit exhausted",
        });
    }

    let polar_range = if first_is_polar { a_range } else { b_range };
    let wide_range = if first_is_polar { b_range } else { a_range };
    let polar_pieces = decompose_general_sphere_polar_window(polar_range)?;
    let wide_pieces = decompose_general_sphere_wide_window(wide_range, parent_parameter_allowance)?;
    let mut occupied = Vec::with_capacity(GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT);
    let mut empty_cells = 0;
    for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
        let (piece_pair_limit, piece_arc_limit) = if polar_index == 0 {
            (
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                GENERAL_SPHERE_WINDOW_ARC_LIMIT,
            )
        } else {
            (
                GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
            )
        };
        for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
            let (piece_a_range, piece_b_range) = if first_is_polar {
                (polar_piece, wide_piece)
            } else {
                (wide_piece, polar_piece)
            };
            let piece_allowance =
                arbitrary_sphere_octant_parameter_allowance(piece_a_range, piece_b_range)?;
            validate_general_sphere_window_slice(piece_a_range, piece_allowance)?;
            validate_general_sphere_window_slice(piece_b_range, piece_allowance)?;
            let hit = certify_general_sphere_window_arrangement(
                a,
                piece_a_range,
                b,
                piece_b_range,
                tolerances,
                piece_pair_limit,
                piece_arc_limit,
                piece_allowance,
            )?;
            if hit.is_proven_empty() {
                empty_cells += 1;
            } else if !hit.is_complete()
                || occupied.len() == GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT
            {
                return Err(Error::InvalidGeometry {
                    reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
                });
            } else {
                occupied.push(([polar_index, wide_index], hit));
            }
        }
    }

    if occupied.is_empty() {
        if empty_cells == GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT {
            return Ok(SurfaceSurfaceIntersections::complete_empty());
        }
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-by-wide union did not cover all six decomposition cells",
        });
    }

    let parent_map = general_sphere_window_map(a, a_range, b, b_range, parent_parameter_allowance);
    let parent_residual = arbitrary_sphere_octant_residual_bound(a, b, parent_parameter_allowance)?;
    let mut hit = if occupied.len() == 1 {
        if empty_cells + 1 != GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT {
            return Err(Error::InvalidGeometry {
                reason: "general coincident sphere polar-by-wide union did not cancel every artificial seam",
            });
        }
        occupied
            .pop()
            .expect("one occupied polar-by-wide child was required")
            .1
    } else if occupied.len() == 2 {
        if occupied.len() != 2 || empty_cells + 2 != GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        let (second_cell, second_hit) = occupied
            .pop()
            .expect("second occupied polar-by-wide child was required");
        let (first_cell, first_hit) = occupied
            .pop()
            .expect("first occupied polar-by-wide child was required");
        let same_row =
            first_cell[0] == second_cell[0] && first_cell[1].abs_diff(second_cell[1]) == 1;
        let same_column =
            first_cell[0].abs_diff(second_cell[0]) == 1 && first_cell[1] == second_cell[1];
        if (!same_row && !same_column)
            || !first_hit.points.is_empty()
            || !first_hit.curves.is_empty()
            || first_hit.regions.len() != 1
            || !second_hit.points.is_empty()
            || !second_hit.curves.is_empty()
            || second_hit.regions.len() != 1
        {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        let (seam_on_first_operand, seam_parameter, seam) = if same_row {
            (
                !first_is_polar,
                0,
                wide_pieces[first_cell[1].max(second_cell[1])][0].lo,
            )
        } else {
            (first_is_polar, 1, polar_pieces[0][1].hi)
        };
        let mut merged = merge_exact_adjacent_sphere_regions_on_parameter(
            &first_hit.regions[0],
            &second_hit.regions[0],
            seam_on_first_operand,
            seam_parameter,
            seam,
        )
        .ok_or(Error::InvalidGeometry {
            reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
        })?;
        let latitude_seam = polar_pieces[0][1].hi;
        let unused_longitude_seams = if same_row {
            let unused = if seam.to_bits() == wide_pieces[1][0].lo.to_bits() {
                wide_pieces[2][0].lo
            } else {
                wide_pieces[1][0].lo
            };
            [Some(unused), None]
        } else {
            [Some(wide_pieces[1][0].lo), Some(wide_pieces[2][0].lo)]
        };
        if exact_sphere_region_parameter_seam_edge(
            &merged,
            seam_on_first_operand,
            seam_parameter,
            seam,
        )
        .is_some()
            || merged.boundary.iter().any(|vertex| {
                let polar_uv = if first_is_polar {
                    vertex.uv_a
                } else {
                    vertex.uv_b
                };
                let wide_uv = if first_is_polar {
                    vertex.uv_b
                } else {
                    vertex.uv_a
                };
                (same_row && polar_uv[1].to_bits() == latitude_seam.to_bits())
                    || unused_longitude_seams
                        .iter()
                        .flatten()
                        .any(|unused| wide_uv[0].to_bits() == unused.to_bits())
            })
        {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        merged.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
        merged.max_residual = merged.max_residual.max(parent_residual);
        return SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![merged],
        );
    } else if occupied.len() == 3 {
        if empty_cells + 3 != GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        if occupied.iter().any(|(_, hit)| {
            !hit.points.is_empty() || !hit.curves.is_empty() || hit.regions.len() != 1
        }) {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }

        let occupied_row = occupied[0].0[0];
        let is_full_row = occupied
            .iter()
            .enumerate()
            .all(|(wide_index, (cell, _))| *cell == [occupied_row, wide_index]);
        let latitude_seam = polar_pieces[0][1].hi;
        let mut merged = if is_full_row {
            // The exact full-row indices make the other three closed children
            // the complete sibling latitude row. Their certified emptiness
            // excludes the artificial latitude seam on either side. The two
            // regular wide-chart seams are removed independently through the
            // strict adjacent-region bit-exact reverse-owner gate.
            let seam_on_first_operand = !first_is_polar;
            let mut regions = occupied.into_iter().map(|(_, hit)| {
                hit.regions
                    .into_iter()
                    .next()
                    .expect("one occupied full-row child region was required")
            });
            let mut merged = regions
                .next()
                .expect("the first occupied full-row child was required");
            for wide_piece in wide_pieces.iter().skip(1) {
                let next = regions
                    .next()
                    .expect("the next occupied full-row child was required");
                merged = merge_exact_adjacent_sphere_regions(
                    &merged,
                    &next,
                    seam_on_first_operand,
                    wide_piece[0].lo,
                )
                .ok_or(Error::InvalidGeometry {
                    reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
                })?;
            }
            if merged.boundary.iter().any(|vertex| {
                let polar_uv = if first_is_polar {
                    vertex.uv_a
                } else {
                    vertex.uv_b
                };
                polar_uv[1].to_bits() == latitude_seam.to_bits()
            }) {
                return Err(Error::InvalidGeometry {
                    reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
                });
            }
            merged
        } else {
            let regions = occupied
                .into_iter()
                .map(|(cell, hit)| {
                    (
                        cell,
                        hit.regions
                            .into_iter()
                            .next()
                            .expect("one occupied mixed-axis child region was required"),
                    )
                })
                .collect::<Vec<_>>();
            let (merged, used_longitude_seam) = merge_exact_polar_wide_sphere_region_path(
                &regions,
                first_is_polar,
                &polar_pieces,
                &wide_pieces,
            )
            .ok_or(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            })?;
            let seam_on_polar_operand = first_is_polar;
            let seam_on_wide_operand = !first_is_polar;
            let longitude_seams = [wide_pieces[1][0].lo, wide_pieces[2][0].lo];
            if exact_sphere_region_parameter_seam_edge(
                &merged,
                seam_on_polar_operand,
                1,
                latitude_seam,
            )
            .is_some()
                || longitude_seams.iter().any(|seam| {
                    exact_sphere_region_parameter_seam_edge(&merged, seam_on_wide_operand, 0, *seam)
                        .is_some()
                })
                || merged.boundary.iter().any(|vertex| {
                    let wide_uv = if first_is_polar {
                        vertex.uv_b
                    } else {
                        vertex.uv_a
                    };
                    longitude_seams.iter().any(|seam| {
                        seam.to_bits() != used_longitude_seam.to_bits()
                            && wide_uv[0].to_bits() == seam.to_bits()
                    })
                })
            {
                return Err(Error::InvalidGeometry {
                    reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
                });
            }
            merged
        };
        merged.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
        merged.max_residual = merged.max_residual.max(parent_residual);
        return SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![merged],
        );
    } else if occupied.len() == 4 {
        if empty_cells + 4 != GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT
            || occupied.iter().any(|(_, hit)| {
                !hit.points.is_empty() || !hit.curves.is_empty() || hit.regions.len() != 1
            })
        {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        let regions = occupied
            .into_iter()
            .map(|(cell, hit)| {
                (
                    cell,
                    hit.regions
                        .into_iter()
                        .next()
                        .expect("one occupied four-cell union region was required"),
                )
            })
            .collect::<Vec<_>>();
        if let Some(mut components) =
            merge_exact_polar_wide_disconnected_vertical_sphere_region_pairs(
                &regions,
                first_is_polar,
                &polar_pieces,
                &wide_pieces,
            )
        {
            for component in &mut components {
                component.correspondence =
                    SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
                component.max_residual = component.max_residual.max(parent_residual);
            }
            return SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
                Vec::new(),
                Vec::new(),
                components,
            );
        }
        if let Some(mut components) = merge_exact_polar_wide_singleton_and_three_cell_sphere_regions(
            &regions,
            first_is_polar,
            &polar_pieces,
            &wide_pieces,
        ) {
            for component in &mut components {
                component.correspondence =
                    SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
                component.max_residual = component.max_residual.max(parent_residual);
            }
            return SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
                Vec::new(),
                Vec::new(),
                components,
            );
        }
        // The path merger remains the only route for degree sequence 2,2,1,1.
        // T-shaped trees and 2x2 cycles instead require simultaneous ownership
        // of all three or four reverse-oriented, bit-exact grid seams before
        // any edge is removed.
        let mut merged = merge_exact_polar_wide_four_sphere_region_path(
            &regions,
            first_is_polar,
            &polar_pieces,
            &wide_pieces,
        )
        .or_else(|| {
            merge_exact_polar_wide_simultaneous_sphere_region_union(
                &regions,
                first_is_polar,
                &polar_pieces,
                &wide_pieces,
            )
        })
        .ok_or(Error::InvalidGeometry {
            reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
        })?;
        let artificial_seams = [
            (first_is_polar, 1, polar_pieces[0][1].hi),
            (!first_is_polar, 0, wide_pieces[1][0].lo),
            (!first_is_polar, 0, wide_pieces[2][0].lo),
        ];
        if artificial_seams.iter().any(|(on_first, parameter, seam)| {
            sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
        }) {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        merged.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
        merged.max_residual = merged.max_residual.max(parent_residual);
        return SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![merged],
        );
    } else {
        if !(5..=GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT).contains(&occupied.len())
            || empty_cells + occupied.len() != GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT
            || occupied.iter().any(|(_, hit)| {
                !hit.points.is_empty() || !hit.curves.is_empty() || hit.regions.len() != 1
            })
        {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        let regions = occupied
            .into_iter()
            .map(|(cell, hit)| {
                (
                    cell,
                    hit.regions
                        .into_iter()
                        .next()
                        .expect("one occupied simultaneous-union region was required"),
                )
            })
            .collect::<Vec<_>>();
        let mut merged = merge_exact_polar_wide_simultaneous_sphere_region_union(
            &regions,
            first_is_polar,
            &polar_pieces,
            &wide_pieces,
        )
        .ok_or(Error::InvalidGeometry {
            reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
        })?;
        let artificial_seams = [
            (first_is_polar, 1, polar_pieces[0][1].hi),
            (!first_is_polar, 0, wide_pieces[1][0].lo),
            (!first_is_polar, 0, wide_pieces[2][0].lo),
        ];
        if artificial_seams.iter().any(|(on_first, parameter, seam)| {
            sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
        }) {
            return Err(Error::InvalidGeometry {
                reason: GENERAL_SPHERE_POLAR_WIDE_LAYOUT_REASON,
            });
        }
        merged.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
        merged.max_residual = merged.max_residual.max(parent_residual);
        return SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![merged],
        );
    };

    // Every latitude and longitude cell is closed. Certified-empty siblings
    // keep this single retained child off both artificial seam families.
    for region in &mut hit.regions {
        region.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
        region.max_residual = region.max_residual.max(parent_residual);
    }
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        hit.points,
        hit.curves,
        hit.regions,
    )
}

fn merge_exact_polar_wide_sphere_region_path(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    first_is_polar: bool,
    polar_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
    wide_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<(SurfaceSurfaceRegion, f64)> {
    if regions.len() != 3 {
        return None;
    }
    let latitude_seam = polar_pieces[0][1].hi;
    let mut adjacent = [[None; 3]; 3];
    let mut degrees = [0_u8; 3];
    let mut longitude_edges = 0;
    let mut latitude_edges = 0;
    let mut used_longitude_seam = None;
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            let first_cell = regions[first].0;
            let second_cell = regions[second].0;
            let polar_delta = first_cell[0].abs_diff(second_cell[0]);
            let wide_delta = first_cell[1].abs_diff(second_cell[1]);
            let seam = if polar_delta == 0 && wide_delta == 1 {
                let seam = wide_pieces[first_cell[1].max(second_cell[1])][0].lo;
                longitude_edges += 1;
                used_longitude_seam = Some(seam);
                Some((!first_is_polar, 0, seam))
            } else if polar_delta == 1 && wide_delta == 0 {
                latitude_edges += 1;
                Some((first_is_polar, 1, latitude_seam))
            } else {
                None
            };
            if let Some(seam) = seam {
                adjacent[first][second] = Some(seam);
                adjacent[second][first] = Some(seam);
                degrees[first] += 1;
                degrees[second] += 1;
            }
        }
    }
    if longitude_edges != 1
        || latitude_edges != 1
        || degrees.iter().filter(|degree| **degree == 1).count() != 2
        || degrees.iter().filter(|degree| **degree == 2).count() != 1
    {
        return None;
    }

    // A mixed-axis L has two possible endpoint-first merge orders. Either is
    // safe only when both successive reverse-owned seam edges are bit exact;
    // trying them in child-index order keeps the result deterministic while
    // avoiding an orientation-specific association rule at the bend.
    for start in (0..regions.len()).filter(|index| degrees[*index] == 1) {
        let bend = (0..regions.len()).find(|index| adjacent[start][*index].is_some())?;
        let finish = (0..regions.len())
            .find(|index| *index != start && *index != bend && adjacent[bend][*index].is_some())?;
        let (first_operand_seam, first_parameter, first_seam) = adjacent[start][bend]?;
        let Some(first_merge) = merge_exact_adjacent_sphere_regions_on_parameter(
            &regions[start].1,
            &regions[bend].1,
            first_operand_seam,
            first_parameter,
            first_seam,
        ) else {
            continue;
        };
        let (second_operand_seam, second_parameter, second_seam) = adjacent[bend][finish]?;
        if let Some(merged) = merge_exact_adjacent_sphere_regions_on_parameter(
            &first_merge,
            &regions[finish].1,
            second_operand_seam,
            second_parameter,
            second_seam,
        ) {
            return Some((merged, used_longitude_seam?));
        }
    }
    None
}

fn merge_exact_polar_wide_disconnected_vertical_sphere_region_pairs(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    first_is_polar: bool,
    polar_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
    wide_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<Vec<SurfaceSurfaceRegion>> {
    let expected_cells = [[0, 0], [0, 2], [1, 0], [1, 2]];
    if regions.len() != expected_cells.len()
        || expected_cells
            .iter()
            .any(|expected| regions.iter().filter(|(cell, _)| cell == expected).count() != 1)
    {
        return None;
    }

    let latitude_seam = polar_pieces[0][1].hi;
    let longitude_seams = [wide_pieces[1][0].lo, wide_pieces[2][0].lo];
    let mut components = Vec::with_capacity(2);
    for column in [0, 2] {
        let lower = regions.iter().find(|(cell, _)| *cell == [0, column])?;
        let upper = regions.iter().find(|(cell, _)| *cell == [1, column])?;
        let merged = merge_exact_adjacent_sphere_regions_on_parameter(
            &lower.1,
            &upper.1,
            first_is_polar,
            1,
            latitude_seam,
        )?;
        if sphere_region_has_parameter_seam_edge(&merged, first_is_polar, 1, latitude_seam)
            || merged.boundary.iter().any(|vertex| {
                let wide_uv = if first_is_polar {
                    vertex.uv_b
                } else {
                    vertex.uv_a
                };
                longitude_seams
                    .iter()
                    .any(|seam| wide_uv[0].to_bits() == seam.to_bits())
            })
        {
            return None;
        }
        components.push(merged);
    }
    Some(components)
}

fn merge_exact_polar_wide_singleton_and_three_cell_sphere_regions(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    first_is_polar: bool,
    polar_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
    wide_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<Vec<SurfaceSurfaceRegion>> {
    const SUPPORTED_LAYOUTS: [[[usize; 2]; 4]; 4] = [
        [[0, 0], [0, 2], [1, 1], [1, 2]],
        [[0, 0], [0, 1], [1, 0], [1, 2]],
        [[0, 1], [0, 2], [1, 0], [1, 2]],
        [[0, 0], [0, 2], [1, 0], [1, 1]],
    ];
    if regions.len() != 4 {
        return None;
    }
    let mut cells = regions.iter().map(|(cell, _)| *cell).collect::<Vec<_>>();
    cells.sort_unstable();
    if !SUPPORTED_LAYOUTS
        .iter()
        .any(|supported| cells.as_slice() == supported)
    {
        return None;
    }

    let all_cells = [[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2]];
    let empty_cells = all_cells
        .into_iter()
        .filter(|cell| !cells.contains(cell))
        .collect::<Vec<_>>();
    let mut degrees = vec![0_u8; regions.len()];
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            if polar_wide_grid_shared_parameter_seam(
                regions[first].0,
                regions[second].0,
                first_is_polar,
                polar_pieces,
                wide_pieces,
            )
            .is_some()
            {
                degrees[first] += 1;
                degrees[second] += 1;
            }
        }
    }
    if degrees.iter().filter(|degree| **degree == 0).count() != 1
        || degrees.iter().filter(|degree| **degree == 1).count() != 2
        || degrees.iter().filter(|degree| **degree == 2).count() != 1
    {
        return None;
    }

    // Both omitted siblings are a graph cut around the singleton. Exact
    // emptiness is useful only when every occupied neighbor stays strictly off
    // the corresponding artificial separator, including corner-only contact.
    if regions.iter().any(|(occupied_cell, region)| {
        empty_cells.iter().any(|empty_cell| {
            polar_wide_grid_shared_parameter_seam(
                *occupied_cell,
                *empty_cell,
                first_is_polar,
                polar_pieces,
                wide_pieces,
            )
            .is_some_and(|(on_first, parameter, seam)| {
                region.boundary.iter().any(|vertex| {
                    let uv = if on_first { vertex.uv_a } else { vertex.uv_b };
                    uv[parameter].to_bits() == seam.to_bits()
                })
            })
        })
    }) {
        return None;
    }

    let singleton_index = degrees.iter().position(|degree| *degree == 0)?;
    let singleton = regions[singleton_index].1.clone();
    let component = regions
        .iter()
        .enumerate()
        .filter(|(index, _)| *index != singleton_index)
        .map(|(_, entry)| entry.clone())
        .collect::<Vec<_>>();
    let (merged, _) = merge_exact_polar_wide_sphere_region_path(
        &component,
        first_is_polar,
        polar_pieces,
        wide_pieces,
    )?;
    let artificial_seams = [
        (first_is_polar, 1, polar_pieces[0][1].hi),
        (!first_is_polar, 0, wide_pieces[1][0].lo),
        (!first_is_polar, 0, wide_pieces[2][0].lo),
    ];
    if [&singleton, &merged].iter().any(|region| {
        artificial_seams.iter().any(|(on_first, parameter, seam)| {
            sphere_region_has_parameter_seam_edge(region, *on_first, *parameter, *seam)
        })
    }) || singleton.boundary.iter().any(|first| {
        merged
            .boundary
            .iter()
            .any(|second| sphere_region_vertices_are_bit_exact(*first, *second))
    }) {
        return None;
    }
    Some(vec![singleton, merged])
}

fn merge_exact_polar_wide_four_sphere_region_path(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    first_is_polar: bool,
    polar_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
    wide_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<SurfaceSurfaceRegion> {
    if regions.len() != 4 {
        return None;
    }
    let mut adjacent = [[None; 4]; 4];
    let mut degrees = [0_u8; 4];
    let mut edge_count = 0;
    let mut longitude_edges = 0;
    let mut latitude_edges = 0;
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            let Some(seam) = polar_wide_grid_shared_parameter_seam(
                regions[first].0,
                regions[second].0,
                first_is_polar,
                polar_pieces,
                wide_pieces,
            ) else {
                continue;
            };
            if seam.1 == 0 {
                longitude_edges += 1;
            } else {
                latitude_edges += 1;
            }
            adjacent[first][second] = Some(seam);
            adjacent[second][first] = Some(seam);
            degrees[first] += 1;
            degrees[second] += 1;
            edge_count += 1;
        }
    }
    if edge_count != 3
        || longitude_edges != 2
        || latitude_edges != 1
        || degrees.iter().filter(|degree| **degree == 1).count() != 2
        || degrees.iter().filter(|degree| **degree == 2).count() != 2
    {
        return None;
    }

    // A four-cell path has five binary association orders. Explore them in
    // stable component order because a previously canceled seam may leave an
    // exact grid-corner vertex on a later seam. Every individual component
    // join still crosses exactly one original path edge and passes the same
    // reverse-owned, bit-exact gate; this is association backtracking, not an
    // approximate or connectivity-only merge.
    let components = regions
        .iter()
        .enumerate()
        .map(|(index, (_, region))| (1_u8 << index, region.clone()))
        .collect::<Vec<_>>();
    merge_exact_polar_wide_path_components(&components, &adjacent)
}

fn merge_exact_polar_wide_path_components(
    components: &[(u8, SurfaceSurfaceRegion)],
    adjacent: &PolarWideFourAdjacency,
) -> Option<SurfaceSurfaceRegion> {
    if let [(mask, region)] = components {
        return (*mask == 0b1111).then(|| region.clone());
    }
    for first in 0..components.len() {
        for second in first + 1..components.len() {
            let mut shared = None;
            let mut ambiguous = false;
            for (first_cell, row) in adjacent.iter().enumerate() {
                if components[first].0 & (1 << first_cell) == 0 {
                    continue;
                }
                for (second_cell, &seam) in row.iter().enumerate() {
                    if components[second].0 & (1 << second_cell) == 0 {
                        continue;
                    }
                    if let Some(seam) = seam {
                        if shared.is_some() {
                            ambiguous = true;
                        } else {
                            shared = Some(seam);
                        }
                    }
                }
            }
            if ambiguous {
                continue;
            }
            let Some((seam_on_first_operand, seam_parameter, seam)) = shared else {
                continue;
            };
            for (first_region, second_region) in [
                (&components[first].1, &components[second].1),
                (&components[second].1, &components[first].1),
            ] {
                let Some(merged) = merge_exact_adjacent_sphere_regions_on_parameter(
                    first_region,
                    second_region,
                    seam_on_first_operand,
                    seam_parameter,
                    seam,
                ) else {
                    continue;
                };
                let mut next = Vec::with_capacity(components.len() - 1);
                for (index, component) in components.iter().enumerate() {
                    if index == first {
                        next.push((components[first].0 | components[second].0, merged.clone()));
                    } else if index != second {
                        next.push(component.clone());
                    }
                }
                if let Some(region) = merge_exact_polar_wide_path_components(&next, adjacent) {
                    return Some(region);
                }
            }
        }
    }
    None
}

fn merge_exact_polar_wide_simultaneous_sphere_region_union(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    first_is_polar: bool,
    polar_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
    wide_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<SurfaceSurfaceRegion> {
    if !(4..=GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT).contains(&regions.len())
        || regions
            .iter()
            .any(|(_, region)| region.orientation != SurfaceRegionOrientation::Same)
    {
        return None;
    }
    let mut removed_edges = regions
        .iter()
        .map(|(_, region)| vec![false; region.boundary.len()])
        .collect::<Vec<_>>();
    let mut internal_edges = 0;
    let mut degrees = vec![0_u8; regions.len()];
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            let Some((seam_on_first_operand, seam_parameter, seam)) =
                polar_wide_grid_shared_parameter_seam(
                    regions[first].0,
                    regions[second].0,
                    first_is_polar,
                    polar_pieces,
                    wide_pieces,
                )
            else {
                continue;
            };
            let first_edge = exact_sphere_region_parameter_seam_edge(
                &regions[first].1,
                seam_on_first_operand,
                seam_parameter,
                seam,
            )?;
            let second_edge = exact_sphere_region_parameter_seam_edge(
                &regions[second].1,
                seam_on_first_operand,
                seam_parameter,
                seam,
            )?;
            if removed_edges[first][first_edge[0]]
                || removed_edges[second][second_edge[0]]
                || !sphere_region_vertices_are_bit_exact(
                    regions[first].1.boundary[first_edge[0]],
                    regions[second].1.boundary[second_edge[1]],
                )
                || !sphere_region_vertices_are_bit_exact(
                    regions[first].1.boundary[first_edge[1]],
                    regions[second].1.boundary[second_edge[0]],
                )
            {
                return None;
            }
            removed_edges[first][first_edge[0]] = true;
            removed_edges[second][second_edge[0]] = true;
            degrees[first] += 1;
            degrees[second] += 1;
            internal_edges += 1;
        }
    }
    // A four-cell T has three grid adjacencies and degree sequence 3,1,1,1;
    // a 2x2 cycle has four and degree sequence 2,2,2,2. The separate path arm
    // retains sole ownership of degree sequence 2,2,1,1. A five-cell layout
    // has four grid edges when the missing sibling is edge middle and five
    // when it is a corner. The complete 2x3 grid has seven.
    let is_supported_four_cell_layout = regions.len() == 4
        && ((internal_edges == 3
            && degrees.iter().filter(|degree| **degree == 1).count() == 3
            && degrees.iter().filter(|degree| **degree == 3).count() == 1)
            || (internal_edges == 4 && degrees.iter().all(|degree| *degree == 2)));
    if !is_supported_four_cell_layout
        && !matches!((regions.len(), internal_edges), (5, 4 | 5) | (6, 7))
    {
        return None;
    }

    let mut outer_edges = Vec::new();
    for (region_index, (_, region)) in regions.iter().enumerate() {
        if region.boundary.len() < 3 {
            return None;
        }
        for (edge_index, &start) in region.boundary.iter().enumerate() {
            if removed_edges[region_index][edge_index] {
                continue;
            }
            let end = region.boundary[(edge_index + 1) % region.boundary.len()];
            if sphere_region_vertices_are_bit_exact(start, end) {
                return None;
            }
            outer_edges.push((start, end));
        }
    }
    if outer_edges.len() < 3 {
        return None;
    }
    // The retained directed edges must form one manifold cycle: each endpoint
    // has exactly one bit-exact successor and one bit-exact predecessor.
    for (edge_index, (start, end)) in outer_edges.iter().enumerate() {
        if outer_edges
            .iter()
            .enumerate()
            .filter(|(candidate, (candidate_start, _))| {
                *candidate != edge_index
                    && sphere_region_vertices_are_bit_exact(*end, *candidate_start)
            })
            .count()
            != 1
            || outer_edges
                .iter()
                .enumerate()
                .filter(|(candidate, (_, candidate_end))| {
                    *candidate != edge_index
                        && sphere_region_vertices_are_bit_exact(*candidate_end, *start)
                })
                .count()
                != 1
        {
            return None;
        }
    }

    let mut used = vec![false; outer_edges.len()];
    used[0] = true;
    let mut boundary = vec![outer_edges[0].0];
    let mut current = outer_edges[0].1;
    for _ in 1..outer_edges.len() {
        let mut next = outer_edges
            .iter()
            .enumerate()
            .filter(|(index, (start, _))| {
                !used[*index] && sphere_region_vertices_are_bit_exact(current, *start)
            })
            .map(|(index, _)| index);
        let next_index = next.next()?;
        if next.next().is_some() {
            return None;
        }
        used[next_index] = true;
        boundary.push(outer_edges[next_index].0);
        current = outer_edges[next_index].1;
    }
    if !used.into_iter().all(core::convert::identity)
        || !sphere_region_vertices_are_bit_exact(current, boundary[0])
        || boundary.len() < 3
    {
        return None;
    }
    Some(SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: regions[0].1.correspondence,
        max_residual: regions
            .iter()
            .map(|(_, region)| region.max_residual)
            .fold(0.0, f64::max),
    })
}

fn polar_wide_grid_shared_parameter_seam(
    first: [usize; 2],
    second: [usize; 2],
    first_is_polar: bool,
    polar_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
    wide_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<(bool, usize, f64)> {
    let polar_delta = first[0].abs_diff(second[0]);
    let wide_delta = first[1].abs_diff(second[1]);
    if polar_delta == 0 && wide_delta == 1 {
        Some((
            !first_is_polar,
            0,
            wide_pieces[first[1].max(second[1])][0].lo,
        ))
    } else if polar_delta == 1 && wide_delta == 0 {
        Some((first_is_polar, 1, polar_pieces[0][1].hi))
    } else {
        None
    }
}

fn decompose_general_sphere_polar_window(
    polar_range: [ParamRange; 2],
) -> Result<[[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT]> {
    let pole = exact_general_sphere_window_pole(polar_range).ok_or(Error::InvalidGeometry {
        reason: "general coincident sphere polar-window decomposition requires one exact natural pole",
    })?;
    let seam = polar_range[1].lo + 0.5 * polar_range[1].width();
    if !seam.is_finite() || seam <= polar_range[1].lo || seam >= polar_range[1].hi {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere polar-window latitude decomposition is not ordered",
        });
    }
    let lower = [polar_range[0], ParamRange::new(polar_range[1].lo, seam)];
    let upper = [polar_range[0], ParamRange::new(seam, polar_range[1].hi)];
    Ok(if pole > 0 {
        [lower, upper]
    } else {
        [upper, lower]
    })
}

#[allow(clippy::too_many_arguments)]
fn certify_double_wide_sphere_window_union(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
    parent_parameter_allowance: f64,
    piece_limit: usize,
    pair_limit: usize,
    arc_limit: usize,
) -> Result<SurfaceSurfaceIntersections> {
    if piece_limit < GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere both-wide union piece limit exhausted",
        });
    }
    if pair_limit < GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere both-wide union pair limit exhausted",
        });
    }
    if arc_limit < GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere both-wide union arc limit exhausted",
        });
    }

    let a_pieces = decompose_general_sphere_wide_window(a_range, parent_parameter_allowance)?;
    let b_pieces = decompose_general_sphere_wide_window(b_range, parent_parameter_allowance)?;
    let mut certified_empty_pairs = 0;
    let mut certified_empty_cells =
        [[false; GENERAL_SPHERE_WIDE_PIECE_LIMIT]; GENERAL_SPHERE_WIDE_PIECE_LIMIT];
    let mut occupied_regions = Vec::with_capacity(GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT);
    // Each parent window is exactly the union of its three closed longitude
    // cells, so distributivity gives
    // (union A_i) intersect (union B_j) = union (A_i intersect B_j).
    for (a_index, &a_piece) in a_pieces.iter().enumerate() {
        for (b_index, &b_piece) in b_pieces.iter().enumerate() {
            let piece_allowance = arbitrary_sphere_octant_parameter_allowance(a_piece, b_piece)?;
            let hit = certify_general_sphere_windows(
                a,
                a_piece,
                b,
                b_piece,
                tolerances,
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                piece_allowance,
            )?;
            if hit.is_proven_empty() {
                certified_empty_pairs += 1;
                certified_empty_cells[a_index][b_index] = true;
                continue;
            }
            if !hit.is_complete()
                || !hit.points.is_empty()
                || !hit.curves.is_empty()
                || hit.regions.len() != 1
            {
                return Err(Error::InvalidGeometry {
                    reason: GENERAL_SPHERE_DOUBLE_WIDE_LAYOUT_REASON,
                });
            }
            if occupied_regions.len() == GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT {
                return Err(Error::InvalidGeometry {
                    reason: GENERAL_SPHERE_DOUBLE_WIDE_LAYOUT_REASON,
                });
            }
            occupied_regions.push((
                [a_index, b_index],
                hit.regions
                    .into_iter()
                    .next()
                    .expect("one certified child region was required"),
            ));
        }
    }

    if occupied_regions.is_empty() {
        if certified_empty_pairs == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT {
            return Ok(SurfaceSurfaceIntersections::complete_empty());
        }
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere both-wide union did not cover every decomposition cell pair",
        });
    }
    let bounded_multi_cell_parents = a_range[0].width()
        < core::f64::consts::TAU - parent_parameter_allowance
        && b_range[0].width() < core::f64::consts::TAU - parent_parameter_allowance;
    let mut resolved_regions = None;
    let supported_positive_cells = match occupied_regions.as_slice() {
        [_] => certified_empty_pairs + 1 == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
        [(first, first_region), (second, second_region)] => {
            let bounded_two_cell_proof = certified_empty_pairs + 2
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_two_cell_proof {
                false
            } else if let Some((seam_on_first_operand, seam)) =
                sphere_grid_shared_seam(*first, *second, &a_pieces, &b_pieces)
            {
                resolved_regions = merge_exact_adjacent_sphere_regions(
                    first_region,
                    second_region,
                    seam_on_first_operand,
                    seam,
                )
                .map(|region| vec![region]);
                resolved_regions.is_some()
            } else {
                sphere_grid_regions_are_pairwise_independent(
                    &occupied_regions,
                    &certified_empty_cells,
                )
            }
        }
        [_, _, _] => {
            let bounded_three_cell_proof = certified_empty_pairs + 3
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_three_cell_proof {
                false
            } else if sphere_grid_regions_are_pairwise_independent(
                &occupied_regions,
                &certified_empty_cells,
            ) {
                true
            } else {
                resolved_regions =
                    merge_exact_sphere_region_path(&occupied_regions, &a_pieces, &b_pieces)
                        .map(|region| vec![region])
                        .or_else(|| {
                            merge_exact_sphere_region_pair_and_isolate(
                                &occupied_regions,
                                &a_pieces,
                                &b_pieces,
                                &certified_empty_cells,
                            )
                        });
                resolved_regions.is_some()
            }
        }
        [_, _, _, _] => {
            let bounded_four_cell_proof = certified_empty_pairs + 4
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_four_cell_proof {
                false
            } else {
                resolved_regions =
                    merge_exact_sphere_region_path(&occupied_regions, &a_pieces, &b_pieces)
                        .or_else(|| {
                            merge_exact_sphere_region_non_path_union(
                                &occupied_regions,
                                &a_pieces,
                                &b_pieces,
                            )
                        })
                        .map(|region| vec![region]);
                resolved_regions.is_some()
            }
        }
        [_, _, _, _, _] => {
            let bounded_five_cell_proof = certified_empty_pairs + 5
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_five_cell_proof {
                false
            } else {
                resolved_regions =
                    merge_exact_sphere_region_path(&occupied_regions, &a_pieces, &b_pieces)
                        .or_else(|| {
                            merge_exact_sphere_region_non_path_union(
                                &occupied_regions,
                                &a_pieces,
                                &b_pieces,
                            )
                        })
                        .map(|region| vec![region])
                        .or_else(|| {
                            merge_exact_sphere_region_components(
                                &occupied_regions,
                                &a_pieces,
                                &b_pieces,
                                &certified_empty_cells,
                            )
                        });
                resolved_regions.is_some()
            }
        }
        [_, _, _, _, _, _] => {
            let bounded_six_cell_proof = certified_empty_pairs + 6
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_six_cell_proof {
                false
            } else {
                resolved_regions =
                    merge_exact_sphere_region_path(&occupied_regions, &a_pieces, &b_pieces)
                        .or_else(|| {
                            merge_exact_sphere_region_non_path_union(
                                &occupied_regions,
                                &a_pieces,
                                &b_pieces,
                            )
                        })
                        .map(|region| vec![region]);
                resolved_regions.is_some()
            }
        }
        [_, _, _, _, _, _, _] => {
            let bounded_seven_cell_proof = certified_empty_pairs + 7
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_seven_cell_proof {
                false
            } else {
                resolved_regions = merge_exact_sphere_region_non_path_union(
                    &occupied_regions,
                    &a_pieces,
                    &b_pieces,
                )
                .map(|region| vec![region]);
                resolved_regions.is_some()
            }
        }
        [_, _, _, _, _, _, _, _] => {
            let bounded_eight_cell_proof = certified_empty_pairs + 8
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_eight_cell_proof {
                false
            } else {
                resolved_regions = merge_exact_sphere_region_non_path_union(
                    &occupied_regions,
                    &a_pieces,
                    &b_pieces,
                )
                .map(|region| vec![region]);
                resolved_regions.is_some()
            }
        }
        [_, _, _, _, _, _, _, _, _] => {
            // All nine closed cells are positive, so the Cartesian
            // decomposition itself is exhaustive and no empty sibling is
            // required. The non-path merger must still prove and cancel all
            // twelve internal grid adjacencies before accepting exactly one
            // unambiguous outer cycle.
            let bounded_nine_cell_proof = certified_empty_pairs + 9
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && bounded_multi_cell_parents;
            if !bounded_nine_cell_proof {
                false
            } else {
                resolved_regions = merge_exact_sphere_region_non_path_union(
                    &occupied_regions,
                    &a_pieces,
                    &b_pieces,
                )
                .map(|region| vec![region]);
                resolved_regions.is_some()
            }
        }
        _ => false,
    };
    if !supported_positive_cells {
        return Err(Error::InvalidGeometry {
            reason: GENERAL_SPHERE_DOUBLE_WIDE_LAYOUT_REASON,
        });
    }

    // Certified-empty orthogonal corner owners isolate each diagonal pair in
    // an independent set or between a merged pair and its singleton; the
    // remaining empty siblings exclude every other artificial seam. Three-
    // through six-cell paths are merged in deterministic adjacency order.
    // Four- through nine-cell non-path unions cancel every shared edge only
    // after paired owners prove reverse-oriented bit-exact seam records or one
    // exact owner supplies the closed-cell/complementary-chart proof described
    // by `exact_sphere_region_shared_seam_edges`; the remaining edges must
    // trace one unambiguous outer cycle. Disconnected
    // five-cell layouts partition in canonical grid order, merge each component
    // under the same seam rules, and require certified-empty sibling owners to
    // separate every pair of components, including both owners of a diagonal
    // grid corner.
    // Pole-clear sub-full-turn parent charts are injective, so the resulting
    // cycles have only true parent boundaries and may use the parent map.
    let parent_residual = arbitrary_sphere_octant_residual_bound(a, b, parent_parameter_allowance)?;
    let parent_map = general_sphere_window_map(a, a_range, b, b_range, parent_parameter_allowance);
    let source_regions = if let Some(regions) = resolved_regions {
        regions
    } else {
        occupied_regions
            .into_iter()
            .map(|(_, region)| region)
            .collect()
    };
    let regions = source_regions
        .into_iter()
        .map(|mut region| {
            region.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(parent_map);
            region.max_residual = region.max_residual.max(parent_residual);
            region
        })
        .collect();
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        regions,
    )
}

fn sphere_grid_regions_are_pairwise_independent(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    certified_empty_cells: &[[bool; GENERAL_SPHERE_WIDE_PIECE_LIMIT];
         GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> bool {
    regions.iter().enumerate().all(|(first_index, first)| {
        regions.iter().skip(first_index + 1).all(|second| {
            sphere_grid_cells_are_independent(first.0, second.0, certified_empty_cells)
        })
    })
}

fn sphere_grid_cells_are_independent(
    first: [usize; 2],
    second: [usize; 2],
    certified_empty_cells: &[[bool; GENERAL_SPHERE_WIDE_PIECE_LIMIT];
         GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> bool {
    let a_delta = first[0].abs_diff(second[0]);
    let b_delta = first[1].abs_diff(second[1]);
    if a_delta + b_delta <= 1 {
        return false;
    }
    if a_delta == 1 && b_delta == 1 {
        // Diagonal closed cells share one grid corner. Both orthogonal cells
        // own that same corner, so both must have certified empty before
        // corner contact is excluded.
        let orthogonal = [[first[0], second[1]], [second[0], first[1]]];
        return orthogonal
            .into_iter()
            .all(|cell| certified_empty_cells[cell[0]][cell[1]]);
    }
    true
}

fn merge_exact_sphere_region_pair_and_isolate(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    certified_empty_cells: &[[bool; GENERAL_SPHERE_WIDE_PIECE_LIMIT];
         GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<Vec<SurfaceSurfaceRegion>> {
    if regions.len() != 3 {
        return None;
    }
    let adjacent = (0..regions.len())
        .flat_map(|first| (first + 1..regions.len()).map(move |second| (first, second)))
        .filter_map(|(first, second)| {
            sphere_grid_shared_seam(regions[first].0, regions[second].0, a_pieces, b_pieces)
                .map(|seam| (first, second, seam))
        })
        .collect::<Vec<_>>();
    let [(first, second, (seam_on_first_operand, seam))]: [(usize, usize, (bool, f64)); 1] =
        adjacent.try_into().ok()?;
    let isolated = (0..regions.len()).find(|index| *index != first && *index != second)?;
    if ![first, second].into_iter().all(|paired| {
        sphere_grid_cells_are_independent(
            regions[paired].0,
            regions[isolated].0,
            certified_empty_cells,
        )
    }) {
        return None;
    }
    let merged = merge_exact_adjacent_sphere_regions(
        &regions[first].1,
        &regions[second].1,
        seam_on_first_operand,
        seam,
    )?;
    if isolated < first.min(second) {
        Some(vec![regions[isolated].1.clone(), merged])
    } else {
        Some(vec![merged, regions[isolated].1.clone()])
    }
}

fn merge_exact_sphere_region_components(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    certified_empty_cells: &[[bool; GENERAL_SPHERE_WIDE_PIECE_LIMIT];
         GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<Vec<SurfaceSurfaceRegion>> {
    if regions.len() != 5 {
        return None;
    }

    let mut component_for = [usize::MAX; GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
    let mut component_count = 0;
    for seed in 0..regions.len() {
        if component_for[seed] != usize::MAX {
            continue;
        }
        component_for[seed] = component_count;
        loop {
            let Some(next) = (0..regions.len()).find(|candidate| {
                component_for[*candidate] == usize::MAX
                    && (0..regions.len()).any(|owner| {
                        component_for[owner] == component_count
                            && sphere_grid_shared_seam(
                                regions[owner].0,
                                regions[*candidate].0,
                                a_pieces,
                                b_pieces,
                            )
                            .is_some()
                    })
            }) else {
                break;
            };
            component_for[next] = component_count;
        }
        component_count += 1;
    }
    if component_count < 2 {
        return None;
    }

    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            if component_for[first] != component_for[second]
                && !sphere_grid_cells_are_independent(
                    regions[first].0,
                    regions[second].0,
                    certified_empty_cells,
                )
            {
                return None;
            }
        }
    }

    let mut merged = Vec::with_capacity(component_count);
    for component in 0..component_count {
        let members = regions
            .iter()
            .enumerate()
            .filter(|(index, _)| component_for[*index] == component)
            .map(|(_, region)| (*region).clone())
            .collect::<Vec<_>>();
        let region = match members.as_slice() {
            [(_, region)] => region.clone(),
            [(first_cell, first_region), (second_cell, second_region)] => {
                let (seam_on_first_operand, seam) =
                    sphere_grid_shared_seam(*first_cell, *second_cell, a_pieces, b_pieces)?;
                merge_exact_adjacent_sphere_regions(
                    first_region,
                    second_region,
                    seam_on_first_operand,
                    seam,
                )?
            }
            _ => merge_exact_sphere_region_path(&members, a_pieces, b_pieces).or_else(|| {
                merge_exact_sphere_region_non_path_union(&members, a_pieces, b_pieces)
            })?,
        };
        merged.push(region);
    }
    Some(merged)
}

fn sphere_grid_shared_seam(
    first: [usize; 2],
    second: [usize; 2],
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<(bool, f64)> {
    let a_delta = first[0].abs_diff(second[0]);
    let b_delta = first[1].abs_diff(second[1]);
    if a_delta == 1 && b_delta == 0 {
        Some((true, a_pieces[first[0].max(second[0])][0].lo))
    } else if a_delta == 0 && b_delta == 1 {
        Some((false, b_pieces[first[1].max(second[1])][0].lo))
    } else {
        None
    }
}

fn merge_exact_sphere_region_path(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<SurfaceSurfaceRegion> {
    if !(3..=GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT).contains(&regions.len()) {
        return None;
    }
    let mut adjacent = [[false; GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
        GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
    let mut degrees = [0_u8; GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
    let mut edge_count = 0;
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            if sphere_grid_shared_seam(regions[first].0, regions[second].0, a_pieces, b_pieces)
                .is_some()
            {
                adjacent[first][second] = true;
                adjacent[second][first] = true;
                degrees[first] += 1;
                degrees[second] += 1;
                edge_count += 1;
            }
        }
    }
    if edge_count != regions.len() - 1
        || degrees[..regions.len()]
            .iter()
            .filter(|degree| **degree == 1)
            .count()
            != 2
        || degrees[..regions.len()]
            .iter()
            .filter(|degree| **degree == 2)
            .count()
            != regions.len() - 2
    {
        return None;
    }

    let mut current = degrees[..regions.len()]
        .iter()
        .position(|degree| *degree == 1)?;
    let mut previous = None;
    let mut path = Vec::with_capacity(regions.len());
    while path.len() < regions.len() {
        path.push(current);
        let next = (0..regions.len())
            .find(|candidate| adjacent[current][*candidate] && Some(*candidate) != previous);
        previous = Some(current);
        if let Some(next) = next {
            current = next;
        } else if path.len() != regions.len() {
            return None;
        }
    }

    let mut merged = regions[path[0]].1.clone();
    for edge in path.windows(2) {
        let (seam_on_first_operand, seam) =
            sphere_grid_shared_seam(regions[edge[0]].0, regions[edge[1]].0, a_pieces, b_pieces)?;
        merged = merge_exact_adjacent_sphere_regions(
            &merged,
            &regions[edge[1]].1,
            seam_on_first_operand,
            seam,
        )?;
    }
    Some(merged)
}

fn merge_exact_sphere_region_non_path_union(
    regions: &[([usize; 2], SurfaceSurfaceRegion)],
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<SurfaceSurfaceRegion> {
    if !(4..=GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT).contains(&regions.len()) {
        return None;
    }
    let mut adjacent = [[false; GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
        GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
    let mut degrees = [0_u8; GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
    let mut edge_count = 0;
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            if sphere_grid_shared_seam(regions[first].0, regions[second].0, a_pieces, b_pieces)
                .is_some()
            {
                adjacent[first][second] = true;
                adjacent[second][first] = true;
                degrees[first] += 1;
                degrees[second] += 1;
                edge_count += 1;
            }
        }
    }
    let is_path = edge_count == regions.len() - 1
        && degrees[..regions.len()]
            .iter()
            .filter(|degree| **degree == 1)
            .count()
            == 2
        && degrees[..regions.len()]
            .iter()
            .filter(|degree| **degree == 2)
            .count()
            == regions.len() - 2;
    if is_path {
        return None;
    }

    let mut included = [false; GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT];
    included[0] = true;
    let mut included_count = 1;
    while included_count < regions.len() {
        let next = (0..regions.len()).find(|candidate| {
            !included[*candidate]
                && (0..regions.len()).any(|owner| included[owner] && adjacent[owner][*candidate])
        })?;
        included[next] = true;
        included_count += 1;
    }

    if regions
        .iter()
        .any(|(_, region)| region.orientation != SurfaceRegionOrientation::Same)
    {
        return None;
    }
    let mut removed_edges = regions
        .iter()
        .map(|(_, region)| vec![false; region.boundary.len()])
        .collect::<Vec<_>>();
    let mut canonical_vertices = regions
        .iter()
        .map(|(_, region)| vec![None; region.boundary.len()])
        .collect::<Vec<Vec<Option<SurfaceSurfaceRegionVertex>>>>();
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            if !adjacent[first][second] {
                continue;
            }
            let (seam_on_first_operand, seam) =
                sphere_grid_shared_seam(regions[first].0, regions[second].0, a_pieces, b_pieces)?;
            let shared = exact_sphere_region_shared_seam_edges(
                &regions[first].1,
                &regions[second].1,
                seam_on_first_operand,
                seam,
                a_pieces,
                b_pieces,
            )?;
            let first_edge = shared.first_edge;
            let second_edge = shared.second_edge;
            if removed_edges[first][first_edge[0]] || removed_edges[second][second_edge[0]] {
                return None;
            }
            match shared.exact_owner {
                ExactSphereSeamOwner::Both => {}
                ExactSphereSeamOwner::First => {
                    if !record_canonical_sphere_region_vertex(
                        &mut canonical_vertices[second][second_edge[0]],
                        regions[first].1.boundary[first_edge[1]],
                    ) || !record_canonical_sphere_region_vertex(
                        &mut canonical_vertices[second][second_edge[1]],
                        regions[first].1.boundary[first_edge[0]],
                    ) {
                        return None;
                    }
                }
                ExactSphereSeamOwner::Second => {
                    if !record_canonical_sphere_region_vertex(
                        &mut canonical_vertices[first][first_edge[0]],
                        regions[second].1.boundary[second_edge[1]],
                    ) || !record_canonical_sphere_region_vertex(
                        &mut canonical_vertices[first][first_edge[1]],
                        regions[second].1.boundary[second_edge[0]],
                    ) {
                        return None;
                    }
                }
            }
            removed_edges[first][first_edge[0]] = true;
            removed_edges[second][second_edge[0]] = true;
        }
    }

    let mut outer_edges = Vec::new();
    for (region_index, (_, region)) in regions.iter().enumerate() {
        for edge in 0..region.boundary.len() {
            if removed_edges[region_index][edge] {
                continue;
            }
            let next = (edge + 1) % region.boundary.len();
            outer_edges.push([
                canonical_vertices[region_index][edge].unwrap_or(region.boundary[edge]),
                canonical_vertices[region_index][next].unwrap_or(region.boundary[next]),
            ]);
        }
    }
    if outer_edges.len() < 3 {
        return None;
    }
    let mut used_edges = vec![false; outer_edges.len()];
    let mut boundary = Vec::with_capacity(outer_edges.len());
    let mut current = 0;
    loop {
        if used_edges[current] {
            return None;
        }
        used_edges[current] = true;
        boundary.push(outer_edges[current][0]);
        let endpoint = outer_edges[current][1];
        if sphere_region_vertices_are_bit_exact(endpoint, outer_edges[0][0]) {
            if used_edges.iter().all(|used| *used) {
                break;
            }
            return None;
        }
        let mut next_edges = (0..outer_edges.len()).filter(|next| {
            !used_edges[*next]
                && sphere_region_vertices_are_bit_exact(endpoint, outer_edges[*next][0])
        });
        current = next_edges.next()?;
        if next_edges.next().is_some() {
            return None;
        }
    }

    Some(SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: regions[0].1.correspondence,
        max_residual: regions
            .iter()
            .map(|(_, region)| region.max_residual)
            .fold(0.0_f64, f64::max),
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ExactSphereSeamOwner {
    Both,
    First,
    Second,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExactSphereSharedSeamEdges {
    first_edge: [usize; 2],
    second_edge: [usize; 2],
    exact_owner: ExactSphereSeamOwner,
}

fn exact_sphere_region_shared_seam_edges(
    first: &SurfaceSurfaceRegion,
    second: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam: f64,
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<ExactSphereSharedSeamEdges> {
    let first_exact = exact_sphere_region_seam_edge(first, seam_on_first_operand, seam);
    let second_exact = exact_sphere_region_seam_edge(second, seam_on_first_operand, seam);
    match (first_exact, second_exact) {
        (Some(first_edge), Some(second_edge)) => {
            if !sphere_region_vertices_are_bit_exact(
                first.boundary[first_edge[0]],
                second.boundary[second_edge[1]],
            ) || !sphere_region_vertices_are_bit_exact(
                first.boundary[first_edge[1]],
                second.boundary[second_edge[0]],
            ) {
                return None;
            }
            Some(ExactSphereSharedSeamEdges {
                first_edge,
                second_edge,
                exact_owner: ExactSphereSeamOwner::Both,
            })
        }
        (Some(first_edge), None) => {
            let second_edge = exact_complementary_chart_sphere_region_edge(
                second,
                first,
                first_edge,
                seam_on_first_operand,
                a_pieces,
                b_pieces,
            )?;
            Some(ExactSphereSharedSeamEdges {
                first_edge,
                second_edge,
                exact_owner: ExactSphereSeamOwner::First,
            })
        }
        (None, Some(second_edge)) => {
            let first_edge = exact_complementary_chart_sphere_region_edge(
                first,
                second,
                second_edge,
                seam_on_first_operand,
                a_pieces,
                b_pieces,
            )?;
            Some(ExactSphereSharedSeamEdges {
                first_edge,
                second_edge,
                exact_owner: ExactSphereSeamOwner::Second,
            })
        }
        (None, None) => None,
    }
}

fn exact_complementary_chart_sphere_region_edge(
    non_owner: &SurfaceSurfaceRegion,
    exact_owner: &SurfaceSurfaceRegion,
    exact_owner_edge: [usize; 2],
    seam_on_first_operand: bool,
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> Option<[usize; 2]> {
    // The two grid cells are closed, differ only in the longitude interval on
    // the seam-owning chart, and share that interval endpoint exactly. Hence
    // the exact owner's complete seam arc belongs to the non-owner cell too.
    // It is also on that cell's boundary: every child chart is pole-clear and
    // narrower than pi, so its longitude parameterization is injective. The
    // only child constraints that change across the adjacency are the two
    // exterior longitude planes; either meets the shared longitude plane only
    // at the excluded poles. An owner edge with consecutive anchors therefore
    // remains one whole arrangement arc in the neighboring child.
    //
    // The complementary chart is unchanged across the adjacency and is
    // injective over the retained pole-clear window. Bit-identical endpoint
    // parameters there prove exact physical endpoint identity without using a
    // distance or ULP allowance. Requiring one unique reverse consecutive edge
    // identifies the same whole arc; ambiguity and merely approximate endpoint
    // recovery fail closed. The exact owner's records can then canonically
    // replace the neighboring reconstructions before outer-cycle tracing.
    let owner_start = exact_owner.boundary[exact_owner_edge[0]];
    let owner_end = exact_owner.boundary[exact_owner_edge[1]];
    let mut found = None;
    for edge_start in 0..non_owner.boundary.len() {
        let edge_end = (edge_start + 1) % non_owner.boundary.len();
        if exact_complementary_chart_sphere_endpoint(
            non_owner.boundary[edge_start],
            owner_end,
            seam_on_first_operand,
            a_pieces,
            b_pieces,
        ) && exact_complementary_chart_sphere_endpoint(
            non_owner.boundary[edge_end],
            owner_start,
            seam_on_first_operand,
            a_pieces,
            b_pieces,
        ) {
            if found.is_some() {
                return None;
            }
            found = Some([edge_start, edge_end]);
        }
    }
    found
}

fn exact_complementary_chart_sphere_endpoint(
    non_owner: SurfaceSurfaceRegionVertex,
    exact_owner: SurfaceSurfaceRegionVertex,
    seam_on_first_operand: bool,
    a_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
    b_pieces: &[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
) -> bool {
    if !sphere_region_vertices_are_bit_exact_in_complementary_chart(
        non_owner,
        exact_owner,
        seam_on_first_operand,
    ) {
        return false;
    }
    if sphere_region_vertices_are_bit_exact(non_owner, exact_owner) {
        return true;
    }

    // A non-exact reconstruction at the crossing of both decomposition seam
    // families would have up to four closed-cell owners. Two-cell evidence is
    // insufficient to choose one canonical multi-owner vertex, so retain the
    // established fail-closed behavior there. The admitted one-owner
    // seven-cell seam differs at a true latitude boundary, while its
    // grid-corner endpoint is already bit exact in both records.
    let complementary_u = if seam_on_first_operand {
        exact_owner.uv_b[0]
    } else {
        exact_owner.uv_a[0]
    };
    let complementary_pieces = if seam_on_first_operand {
        b_pieces
    } else {
        a_pieces
    };
    [complementary_pieces[1][0].lo, complementary_pieces[2][0].lo]
        .into_iter()
        .all(|seam| complementary_u.to_bits() != seam.to_bits())
}

fn sphere_region_vertices_are_bit_exact_in_complementary_chart(
    first: SurfaceSurfaceRegionVertex,
    second: SurfaceSurfaceRegionVertex,
    seam_on_first_operand: bool,
) -> bool {
    let (first_uv, second_uv) = if seam_on_first_operand {
        (first.uv_b, second.uv_b)
    } else {
        (first.uv_a, second.uv_a)
    };
    first_uv[0].to_bits() == second_uv[0].to_bits()
        && first_uv[1].to_bits() == second_uv[1].to_bits()
}

fn record_canonical_sphere_region_vertex(
    slot: &mut Option<SurfaceSurfaceRegionVertex>,
    vertex: SurfaceSurfaceRegionVertex,
) -> bool {
    match *slot {
        Some(existing) => sphere_region_vertices_are_bit_exact(existing, vertex),
        None => {
            *slot = Some(vertex);
            true
        }
    }
}

fn merge_exact_adjacent_sphere_regions(
    first: &SurfaceSurfaceRegion,
    second: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam: f64,
) -> Option<SurfaceSurfaceRegion> {
    merge_exact_adjacent_sphere_regions_on_parameter(first, second, seam_on_first_operand, 0, seam)
}

fn merge_exact_adjacent_sphere_regions_on_parameter(
    first: &SurfaceSurfaceRegion,
    second: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam_parameter: usize,
    seam: f64,
) -> Option<SurfaceSurfaceRegion> {
    if first.orientation != SurfaceRegionOrientation::Same
        || second.orientation != SurfaceRegionOrientation::Same
    {
        return None;
    }
    let first_edge = exact_sphere_region_parameter_seam_edge(
        first,
        seam_on_first_operand,
        seam_parameter,
        seam,
    )?;
    let second_edge = exact_sphere_region_parameter_seam_edge(
        second,
        seam_on_first_operand,
        seam_parameter,
        seam,
    )?;
    if !sphere_region_vertices_are_bit_exact(
        first.boundary[first_edge[0]],
        second.boundary[second_edge[1]],
    ) || !sphere_region_vertices_are_bit_exact(
        first.boundary[first_edge[1]],
        second.boundary[second_edge[0]],
    ) {
        return None;
    }

    let first_outer = sphere_region_complementary_path(&first.boundary, first_edge);
    let second_outer = sphere_region_complementary_path(&second.boundary, second_edge);
    let mut boundary = first_outer;
    boundary.extend_from_slice(&second_outer[1..second_outer.len() - 1]);
    if boundary.len() < 3 {
        return None;
    }
    Some(SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: first.correspondence,
        max_residual: first.max_residual.max(second.max_residual),
    })
}

fn sphere_region_vertices_are_bit_exact(
    first: SurfaceSurfaceRegionVertex,
    second: SurfaceSurfaceRegionVertex,
) -> bool {
    first.point.x.to_bits() == second.point.x.to_bits()
        && first.point.y.to_bits() == second.point.y.to_bits()
        && first.point.z.to_bits() == second.point.z.to_bits()
        && first.uv_a[0].to_bits() == second.uv_a[0].to_bits()
        && first.uv_a[1].to_bits() == second.uv_a[1].to_bits()
        && first.uv_b[0].to_bits() == second.uv_b[0].to_bits()
        && first.uv_b[1].to_bits() == second.uv_b[1].to_bits()
        && first.residual.to_bits() == second.residual.to_bits()
}

fn exact_sphere_region_seam_edge(
    region: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam: f64,
) -> Option<[usize; 2]> {
    exact_sphere_region_parameter_seam_edge(region, seam_on_first_operand, 0, seam)
}

fn sphere_region_has_parameter_seam_edge(
    region: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam_parameter: usize,
    seam: f64,
) -> bool {
    if seam_parameter > 1 || region.boundary.len() < 2 {
        return false;
    }
    let seam_bits = seam.to_bits();
    region.boundary.iter().enumerate().any(|(index, vertex)| {
        let next = region.boundary[(index + 1) % region.boundary.len()];
        let parameter = if seam_on_first_operand {
            vertex.uv_a[seam_parameter]
        } else {
            vertex.uv_b[seam_parameter]
        };
        let next_parameter = if seam_on_first_operand {
            next.uv_a[seam_parameter]
        } else {
            next.uv_b[seam_parameter]
        };
        parameter.to_bits() == seam_bits && next_parameter.to_bits() == seam_bits
    })
}

fn exact_sphere_region_parameter_seam_edge(
    region: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam_parameter: usize,
    seam: f64,
) -> Option<[usize; 2]> {
    if seam_parameter > 1 {
        return None;
    }
    let seam_bits = seam.to_bits();
    let vertices = region
        .boundary
        .iter()
        .enumerate()
        .filter_map(|(index, vertex)| {
            let parameter = if seam_on_first_operand {
                vertex.uv_a[seam_parameter]
            } else {
                vertex.uv_b[seam_parameter]
            };
            (parameter.to_bits() == seam_bits).then_some(index)
        })
        .collect::<Vec<_>>();
    let [first, second]: [usize; 2] = vertices.try_into().ok()?;
    if (first + 1) % region.boundary.len() == second {
        Some([first, second])
    } else if (second + 1) % region.boundary.len() == first {
        Some([second, first])
    } else {
        None
    }
}

fn sphere_region_complementary_path(
    boundary: &[SurfaceSurfaceRegionVertex],
    seam_edge: [usize; 2],
) -> Vec<SurfaceSurfaceRegionVertex> {
    let mut path = Vec::with_capacity(boundary.len());
    let mut index = seam_edge[1];
    path.push(boundary[index]);
    while index != seam_edge[0] {
        index = (index + 1) % boundary.len();
        path.push(boundary[index]);
    }
    path
}

#[allow(clippy::too_many_arguments)]
fn certify_single_wide_sphere_window_union(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    first_is_wide: bool,
    tolerances: Tolerances,
    parent_parameter_allowance: f64,
    piece_limit: usize,
    pair_limit: usize,
    arc_limit: usize,
) -> Result<SurfaceSurfaceIntersections> {
    if piece_limit < GENERAL_SPHERE_WIDE_PIECE_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window piece limit exhausted",
        });
    }
    if pair_limit < GENERAL_SPHERE_WIDE_PAIR_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window pair limit exhausted",
        });
    }
    if arc_limit < GENERAL_SPHERE_WIDE_ARC_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window arc limit exhausted",
        });
    }

    let wide_range = if first_is_wide { a_range } else { b_range };
    let wide_pieces = decompose_general_sphere_wide_window(wide_range, parent_parameter_allowance)?;
    let mut occupied_region = None;
    let mut empty_pieces = 0;
    for piece_range in wide_pieces {
        let (piece_a_range, piece_b_range) = if first_is_wide {
            (piece_range, b_range)
        } else {
            (a_range, piece_range)
        };
        let piece_allowance =
            arbitrary_sphere_octant_parameter_allowance(piece_a_range, piece_b_range)?;
        let hit = certify_general_sphere_windows(
            a,
            piece_a_range,
            b,
            piece_b_range,
            tolerances,
            GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
            GENERAL_SPHERE_WINDOW_ARC_LIMIT,
            piece_allowance,
        )?;
        if hit.is_proven_empty() {
            empty_pieces += 1;
            continue;
        }
        if !hit.is_complete()
            || !hit.points.is_empty()
            || !hit.curves.is_empty()
            || hit.regions.len() != 1
            || occupied_region.is_some()
        {
            return Err(Error::InvalidGeometry {
                reason: "general coincident sphere wide-window union requires one positive-area cell and certified-empty siblings",
            });
        }
        occupied_region = hit.regions.into_iter().next();
    }

    let Some(mut region) = occupied_region else {
        if empty_pieces == GENERAL_SPHERE_WIDE_PIECE_LIMIT {
            return Ok(SurfaceSurfaceIntersections::complete_empty());
        }
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window union did not cover every decomposition cell",
        });
    };
    if empty_pieces + 1 != GENERAL_SPHERE_WIDE_PIECE_LIMIT {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window union did not cancel every artificial seam",
        });
    }

    // All cells are closed. A point on either artificial seam belongs to both
    // adjacent cells, so a certified-empty sibling proves that the occupied
    // cell cannot meet that seam. The retained region therefore has only true
    // source-window boundaries, and replacing its cell map with the parent
    // source map is an exact union identity rather than an interpolation.
    region.correspondence = SurfaceRegionCorrespondence::GeneralSphereWindow(
        general_sphere_window_map(a, a_range, b, b_range, parent_parameter_allowance),
    );
    region.max_residual = region
        .max_residual
        .max(arbitrary_sphere_octant_residual_bound(
            a,
            b,
            parent_parameter_allowance,
        )?);
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![region],
    )
}

fn decompose_general_sphere_wide_window(
    wide_range: [ParamRange; 2],
    parameter_allowance: f64,
) -> Result<[[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT]> {
    let width = wide_range[0].width();
    if width / GENERAL_SPHERE_WIDE_PIECE_LIMIT as f64 >= core::f64::consts::PI - parameter_allowance
    {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window union requires three sub-pi decomposition cells",
        });
    }
    let seams = [
        wide_range[0].lo,
        wide_range[0].lo + width / 3.0,
        wide_range[0].lo + 2.0 * width / 3.0,
        wide_range[0].hi,
    ];
    if seams[0] != wide_range[0].lo
        || seams[3] != wide_range[0].hi
        || seams.windows(2).any(|pair| pair[0] >= pair[1])
    {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere wide-window decomposition is not ordered and exact at source endpoints",
        });
    }
    Ok(core::array::from_fn(|piece| {
        [
            ParamRange::new(seams[piece], seams[piece + 1]),
            wide_range[1],
        ]
    }))
}

#[allow(clippy::too_many_arguments)]
fn certify_collapsed_general_sphere_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    constraints: &[SphereWindowConstraint],
    roots: &[CertifiedSphereBoundaryRoot],
    tolerances: Tolerances,
    arc_limit: usize,
    parameter_allowance: f64,
) -> Result<Option<SurfaceSurfaceIntersections>> {
    let locks = exact_sphere_boundary_locks(constraints);
    match locks.as_slice() {
        [] => Ok(None),
        [lock] => certify_locked_sphere_circle(
            a,
            a_range,
            b,
            b_range,
            constraints,
            roots,
            lock,
            tolerances,
            arc_limit,
            parameter_allowance,
        )
        .map(Some),
        [first, second] => {
            if sphere_planes_are_exactly_parallel(first.plane.normal, second.plane.normal) {
                if first.plane == second.plane {
                    return Err(Error::InvalidGeometry {
                        reason: "general coincident sphere collapsed proof retained duplicate equality locks",
                    });
                }
                return Ok(Some(SurfaceSurfaceIntersections::complete_empty()));
            }
            certify_locked_sphere_points(
                a,
                a_range,
                b,
                b_range,
                constraints,
                roots,
                first,
                second,
                tolerances,
                parameter_allowance,
            )
            .map(Some)
        }
        _ => Err(Error::InvalidGeometry {
            reason: "general coincident sphere collapsed proof supports at most two independent equality locks",
        }),
    }
}

fn exact_sphere_boundary_locks(
    constraints: &[SphereWindowConstraint],
) -> Vec<ExactSphereBoundaryLock> {
    let mut locks: Vec<ExactSphereBoundaryLock> = Vec::new();
    for first in 0..constraints.len() {
        for second in first + 1..constraints.len() {
            // A collapsed result is admitted only for bit-exact opposing
            // normalized plane equations. Angular or offset tolerances never
            // create equality locks; near locks remain in the indeterminate
            // arrangement path.
            if constraints[first].normal != -constraints[second].normal
                || constraints[first].offset != -constraints[second].offset
            {
                continue;
            }
            let (plane, representative) =
                canonical_sphere_constraint(constraints[first], first, constraints[second], second);
            if let Some(existing) = locks.iter_mut().find(|lock| lock.plane == plane) {
                existing.members.extend([first, second]);
                existing.members.sort_unstable();
                existing.members.dedup();
                existing.representative = existing.representative.min(representative);
            } else {
                locks.push(ExactSphereBoundaryLock {
                    plane,
                    representative,
                    members: vec![first, second],
                });
            }
        }
    }
    locks.sort_by(|first, second| {
        compare_sphere_constraints(first.plane, second.plane)
            .then(first.representative.cmp(&second.representative))
    });
    locks
}

fn canonical_sphere_constraint(
    first: SphereWindowConstraint,
    first_index: usize,
    second: SphereWindowConstraint,
    second_index: usize,
) -> (SphereWindowConstraint, usize) {
    if compare_sphere_constraints(first, second).is_le() {
        (first, first_index)
    } else {
        (second, second_index)
    }
}

fn compare_sphere_constraints(
    first: SphereWindowConstraint,
    second: SphereWindowConstraint,
) -> core::cmp::Ordering {
    compare_sphere_directions(first.normal, second.normal)
        .then(first.offset.total_cmp(&second.offset))
}

#[allow(clippy::too_many_arguments)]
fn certify_locked_sphere_circle(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    constraints: &[SphereWindowConstraint],
    roots: &[CertifiedSphereBoundaryRoot],
    lock: &ExactSphereBoundaryLock,
    tolerances: Tolerances,
    arc_limit: usize,
    parameter_allowance: f64,
) -> Result<SurfaceSurfaceIntersections> {
    let unit_frame = Frame::from_z(Point3::new(0.0, 0.0, 0.0), lock.plane.normal)?;
    let radius_squared = 1.0 - lock.plane.offset * lock.plane.offset;
    if radius_squared <= 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere collapsed circle is singular",
        });
    }
    let unit_radius = radius_squared.sqrt();
    let unit_center = lock.plane.normal * lock.plane.offset;
    let circle = Circle::new(
        Frame::new(
            a.frame().origin() + lock.plane.normal * (a.radius() * lock.plane.offset),
            lock.plane.normal,
            unit_frame.x(),
        )?,
        a.radius() * unit_radius,
    )?;

    let mut circle_roots = roots
        .iter()
        .copied()
        .filter(|root| root.active.contains(&lock.representative))
        .filter(|root| {
            root.active
                .iter()
                .any(|index| !lock.members.contains(index))
        })
        .map(|mut root| {
            root.feasible =
                certify_sphere_root_membership_ignoring(root, constraints, &lock.members)?;
            let radial = root.direction - unit_center;
            let mut angle = math::atan2(radial.dot(unit_frame.y()), radial.dot(unit_frame.x()));
            if angle < 0.0 {
                angle += core::f64::consts::TAU;
            }
            Ok((angle, root))
        })
        .collect::<Result<Vec<_>>>()?;
    circle_roots.sort_by(|first, second| {
        first
            .0
            .total_cmp(&second.0)
            .then_with(|| compare_sphere_directions(first.1.direction, second.1.direction))
    });
    for first in 0..circle_roots.len() {
        for second in first + 1..circle_roots.len() {
            if (circle_roots[first].1.direction - circle_roots[second].1.direction).norm()
                <= tolerances.angular()
            {
                return Err(Error::InvalidGeometry {
                    reason: "general coincident sphere collapsed circle has an unresolved multiple boundary root",
                });
            }
        }
    }

    let mut remaining_arcs = arc_limit;
    let mut feasible_arcs = Vec::new();
    if circle_roots.is_empty() {
        spend_sphere_boundary_arc(&mut remaining_arcs)?;
        let direction = sphere_boundary_direction(unit_center, unit_frame, unit_radius, 0.0);
        if certify_sphere_direction_membership_ignoring(
            direction,
            None,
            constraints,
            tolerances,
            false,
            &lock.members,
        )? {
            feasible_arcs.push((0.0, core::f64::consts::TAU));
        }
    } else {
        for index in 0..circle_roots.len() {
            spend_sphere_boundary_arc(&mut remaining_arcs)?;
            let (lo, first) = circle_roots[index];
            let (mut hi, second) = circle_roots[(index + 1) % circle_roots.len()];
            if index + 1 == circle_roots.len() {
                hi += core::f64::consts::TAU;
            }
            let midpoint = sphere_boundary_direction(
                unit_center,
                unit_frame,
                unit_radius,
                lo + 0.5 * (hi - lo),
            );
            // Every remaining inequality can change sign on the locked circle
            // only at one of the interval-certified roots above. One strict
            // midpoint classification therefore certifies the complete open
            // arc between consecutive roots.
            if certify_sphere_direction_membership_ignoring(
                midpoint,
                None,
                constraints,
                tolerances,
                false,
                &lock.members,
            )? {
                if !first.feasible || !second.feasible {
                    return Err(Error::InvalidGeometry {
                        reason: "general coincident sphere collapsed circle arc topology is not certified",
                    });
                }
                feasible_arcs.push((lo, hi));
            }
        }
    }

    if !feasible_arcs.is_empty() {
        let mut curves = Vec::with_capacity(feasible_arcs.len());
        for (lo, hi) in feasible_arcs {
            let start_direction =
                sphere_boundary_direction(unit_center, unit_frame, unit_radius, lo);
            let end_direction = sphere_boundary_direction(unit_center, unit_frame, unit_radius, hi);
            let start = paired_general_sphere_direction(
                a,
                a_range,
                b,
                b_range,
                start_direction,
                parameter_allowance,
                tolerances,
            )?;
            let end = paired_general_sphere_direction(
                a,
                a_range,
                b,
                b_range,
                end_direction,
                parameter_allowance,
                tolerances,
            )?;
            curves.push(SurfaceSurfaceCurve {
                curve: SurfaceIntersectionCurve::Circle(circle),
                curve_range: ParamRange::new(lo, hi),
                uv_a_start: start.uv_a,
                uv_a_end: end.uv_a,
                uv_b_start: start.uv_b,
                uv_b_end: end.uv_b,
                kind: ContactKind::Tangent,
            });
        }
        return SurfaceSurfaceIntersections::canonicalized_complete(Vec::new(), curves);
    }

    let points = circle_roots
        .into_iter()
        .filter_map(|(_, root)| root.feasible.then_some(root.direction))
        .map(|direction| {
            paired_general_sphere_contact(
                a,
                a_range,
                b,
                b_range,
                direction,
                parameter_allowance,
                tolerances,
            )
        })
        .collect::<Result<Vec<_>>>()?;
    SurfaceSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

#[allow(clippy::too_many_arguments)]
fn certify_locked_sphere_points(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    constraints: &[SphereWindowConstraint],
    roots: &[CertifiedSphereBoundaryRoot],
    first: &ExactSphereBoundaryLock,
    second: &ExactSphereBoundaryLock,
    tolerances: Tolerances,
    parameter_allowance: f64,
) -> Result<SurfaceSurfaceIntersections> {
    let mut ignored = first.members.clone();
    ignored.extend(&second.members);
    ignored.sort_unstable();
    ignored.dedup();
    let candidates = roots
        .iter()
        .copied()
        .filter(|root| {
            root.active.contains(&first.representative)
                && root.active.contains(&second.representative)
        })
        .collect::<Vec<_>>();
    let mut points = Vec::new();
    for mut candidate in candidates {
        candidate.feasible =
            certify_sphere_root_membership_ignoring(candidate, constraints, &ignored)?;
        if candidate.feasible {
            points.push(paired_general_sphere_contact(
                a,
                a_range,
                b,
                b_range,
                candidate.direction,
                parameter_allowance,
                tolerances,
            )?);
        }
    }
    SurfaceSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn sphere_boundary_direction(center: Vec3, frame: Frame, radius: f64, parameter: f64) -> Vec3 {
    let (sin_parameter, cos_parameter) = math::sincos(parameter);
    center + (frame.x() * cos_parameter + frame.y() * sin_parameter) * radius
}

#[allow(clippy::too_many_arguments)]
fn paired_general_sphere_contact(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    direction: Vec3,
    parameter_allowance: f64,
    tolerances: Tolerances,
) -> Result<SurfaceSurfacePoint> {
    let sample = paired_general_sphere_direction(
        a,
        a_range,
        b,
        b_range,
        direction,
        parameter_allowance,
        tolerances,
    )?;
    let kind = if a.normal(sample.uv_a).is_none() || b.normal(sample.uv_b).is_none() {
        ContactKind::Singular
    } else {
        ContactKind::Tangent
    };
    Ok(SurfaceSurfacePoint {
        point: sample.point,
        uv_a: sample.uv_a,
        uv_b: sample.uv_b,
        residual: sample.residual,
        kind,
    })
}

fn validate_general_sphere_window_slice(
    range: [ParamRange; 2],
    parameter_allowance: f64,
) -> Result<()> {
    validate_general_sphere_window_base(range, parameter_allowance)?;
    if range[0].width() >= core::f64::consts::PI - parameter_allowance {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window fallback supports only positive-area pole-clear windows with longitude span below pi",
        });
    }
    Ok(())
}

fn validate_general_sphere_window_base(
    range: [ParamRange; 2],
    parameter_allowance: f64,
) -> Result<()> {
    let half_pi = core::f64::consts::FRAC_PI_2;
    let pole = exact_general_sphere_window_pole(range);
    let latitude_is_supported = match pole {
        Some(1) => range[1].lo > -half_pi + parameter_allowance,
        Some(-1) => range[1].hi < half_pi - parameter_allowance,
        Some(_) => unreachable!("sphere pole sign is normalized"),
        None => {
            range[1].lo > -half_pi + parameter_allowance
                && range[1].hi < half_pi - parameter_allowance
        }
    };
    if range[0].width() <= parameter_allowance
        || range[1].width() <= parameter_allowance
        || !latitude_is_supported
    {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window proof supports only positive-area pole-clear windows or one exact natural-pole boundary",
        });
    }
    Ok(())
}

fn exact_general_sphere_window_pole(range: [ParamRange; 2]) -> Option<i8> {
    let half_pi = core::f64::consts::FRAC_PI_2;
    if range[1].hi.to_bits() == half_pi.to_bits() {
        Some(1)
    } else if range[1].lo.to_bits() == (-half_pi).to_bits() {
        Some(-1)
    } else {
        None
    }
}

fn general_sphere_window_constraints(
    sphere: &Sphere,
    range: [ParamRange; 2],
) -> Result<Vec<SphereWindowConstraint>> {
    let frame = sphere.frame();
    let (sin_u_lo, cos_u_lo) = math::sincos(range[0].lo);
    let (sin_u_hi, cos_u_hi) = math::sincos(range[0].hi);
    let (sin_v_lo, _) = math::sincos(range[1].lo);
    let (sin_v_hi, _) = math::sincos(range[1].hi);
    let mut planes = vec![
        (frame.y() * cos_u_lo - frame.x() * sin_u_lo, 0.0),
        (frame.x() * sin_u_hi - frame.y() * cos_u_hi, 0.0),
    ];
    if range[1].lo.to_bits() != (-core::f64::consts::FRAC_PI_2).to_bits() {
        planes.push((frame.z(), sin_v_lo));
    }
    if range[1].hi.to_bits() != core::f64::consts::FRAC_PI_2.to_bits() {
        planes.push((-frame.z(), -sin_v_hi));
    }
    planes
        .into_iter()
        .map(|(normal, offset)| {
            let norm = normal.norm();
            if !norm.is_finite() || norm == 0.0 {
                return Err(Error::InvalidGeometry {
                    reason: "general coincident sphere window boundary plane is singular",
                });
            }
            Ok(SphereWindowConstraint {
                normal: normal / norm,
                offset: offset / norm,
            })
        })
        .collect()
}

fn certified_sphere_boundary_pair(
    first: SphereWindowConstraint,
    second: SphereWindowConstraint,
    active: [usize; 2],
    tolerances: Tolerances,
) -> Result<Vec<CertifiedSphereBoundaryRoot>> {
    if sphere_planes_are_exactly_parallel(first.normal, second.normal) {
        return Ok(Vec::new());
    }

    let first_interval = first.normal.to_array().map(Interval::point);
    let second_interval = second.normal.to_array().map(Interval::point);
    let cross_interval = interval_cross(first_interval, second_interval);
    let determinant = interval_dot(cross_interval, cross_interval);
    let angular_squared = Interval::point(tolerances.angular()).square();
    if determinant.lo() <= angular_squared.hi() {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window boundary planes exceed the certified angular corridor",
        });
    }

    let first_numerator = interval_cross(second_interval, cross_interval)
        .map(|value| value * Interval::point(first.offset));
    let second_numerator = interval_cross(cross_interval, first_interval)
        .map(|value| value * Interval::point(second.offset));
    let point_interval = core::array::from_fn(|axis| {
        (first_numerator[axis] + second_numerator[axis])
            .checked_div(determinant)
            .expect("certified determinant excludes zero")
    });
    let discriminant = Interval::point(1.0) - interval_dot(point_interval, point_interval);
    if discriminant.hi() < 0.0 {
        return Ok(Vec::new());
    }
    if discriminant.lo() <= 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window boundary tangency is not certified by this fallback arm",
        });
    }
    let scale = discriminant
        .checked_div(determinant)
        .and_then(Interval::sqrt)
        .ok_or(Error::InvalidGeometry {
            reason: "general coincident sphere window boundary intersection arithmetic is non-finite",
        })?;

    let cross = first.normal.cross(second.normal);
    let determinant_nominal = cross.dot(cross);
    let point = (second.normal.cross(cross) * first.offset
        + cross.cross(first.normal) * second.offset)
        / determinant_nominal;
    let scale_nominal = ((1.0 - point.dot(point)) / determinant_nominal).sqrt();
    if !scale_nominal.is_finite() {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window boundary intersection arithmetic is non-finite",
        });
    }

    let roots = [-1.0, 1.0].map(|sign| {
        let enclosure = core::array::from_fn(|axis| {
            if sign < 0.0 {
                point_interval[axis] - cross_interval[axis] * scale
            } else {
                point_interval[axis] + cross_interval[axis] * scale
            }
        });
        let direction = (point + cross * (sign * scale_nominal))
            .normalized()
            .ok_or(Error::InvalidGeometry {
                reason: "general coincident sphere window boundary intersection is singular",
            })?;
        Ok(CertifiedSphereBoundaryRoot {
            direction,
            enclosure,
            active,
            feasible: false,
        })
    });
    roots.into_iter().collect()
}

fn certify_sphere_root_membership(
    root: CertifiedSphereBoundaryRoot,
    constraints: &[SphereWindowConstraint],
    _tolerances: Tolerances,
) -> Result<bool> {
    certify_sphere_root_membership_ignoring(root, constraints, &[])
}

fn certify_sphere_root_membership_ignoring(
    root: CertifiedSphereBoundaryRoot,
    constraints: &[SphereWindowConstraint],
    ignored: &[usize],
) -> Result<bool> {
    let mut undecided = false;
    for (index, constraint) in constraints.iter().enumerate() {
        if root.active.contains(&index) || ignored.contains(&index) {
            continue;
        }
        let margin = interval_dot(
            root.enclosure,
            constraint.normal.to_array().map(Interval::point),
        ) - Interval::point(constraint.offset);
        if margin.hi() < 0.0 {
            return Ok(false);
        }
        if margin.lo() <= 0.0 {
            undecided = true;
        }
    }
    if undecided {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window proof encountered an unresolved multiple boundary vertex",
        });
    }
    Ok(true)
}

fn certify_sphere_boundary_arcs(
    constraints: &[SphereWindowConstraint],
    roots: &[CertifiedSphereBoundaryRoot],
    tolerances: Tolerances,
    arc_limit: usize,
) -> Result<CertifiedSphereBoundaryArrangement> {
    let mut arcs = Vec::new();
    let mut remaining_arcs = arc_limit;
    let mut has_feasible_boundary = roots.iter().any(|root| root.feasible);
    for (constraint_index, constraint) in constraints.iter().copied().enumerate() {
        let frame = Frame::from_z(Point3::new(0.0, 0.0, 0.0), constraint.normal)?;
        let radius_squared = 1.0 - constraint.offset * constraint.offset;
        if radius_squared <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "general coincident sphere window fallback excludes pole boundary circles",
            });
        }
        let radius = radius_squared.sqrt();
        let center = constraint.normal * constraint.offset;
        let mut ordered = roots
            .iter()
            .enumerate()
            .filter(|(_, root)| root.active.contains(&constraint_index))
            .map(|(index, root)| {
                let radial = root.direction - center;
                (
                    math::atan2(radial.dot(frame.y()), radial.dot(frame.x())),
                    index,
                )
            })
            .collect::<Vec<_>>();
        ordered.sort_by(|first, second| first.0.total_cmp(&second.0).then(first.1.cmp(&second.1)));
        if ordered.len() < 2 {
            spend_sphere_boundary_arc(&mut remaining_arcs)?;
            let sample = center + frame.x() * radius;
            has_feasible_boundary |= certify_sphere_direction_membership(
                sample,
                Some(constraint_index),
                constraints,
                tolerances,
                false,
            )?;
            continue;
        }
        for edge in 0..ordered.len() {
            spend_sphere_boundary_arc(&mut remaining_arcs)?;
            let (first_angle, first) = ordered[edge];
            let (mut second_angle, second) = ordered[(edge + 1) % ordered.len()];
            if edge + 1 == ordered.len() {
                second_angle += core::f64::consts::TAU;
            }
            let midpoint_angle = first_angle + 0.5 * (second_angle - first_angle);
            let (sin_midpoint, cos_midpoint) = math::sincos(midpoint_angle);
            let midpoint = center + (frame.x() * cos_midpoint + frame.y() * sin_midpoint) * radius;
            let feasible = certify_sphere_direction_membership(
                midpoint,
                Some(constraint_index),
                constraints,
                tolerances,
                false,
            )?;
            if feasible {
                has_feasible_boundary = true;
                if !roots[first].feasible || !roots[second].feasible {
                    return Err(Error::InvalidGeometry {
                        reason: "general coincident sphere window boundary cycle is not topologically certified",
                    });
                }
                arcs.push(CertifiedSphereBoundaryArc {
                    first,
                    second,
                    midpoint,
                });
            }
        }
    }
    Ok(CertifiedSphereBoundaryArrangement {
        feasible_arcs: arcs,
        all_boundaries_excluded: !has_feasible_boundary,
    })
}

fn spend_sphere_boundary_arc(remaining: &mut usize) -> Result<()> {
    if *remaining == 0 {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window proof arc limit exhausted",
        });
    }
    *remaining -= 1;
    Ok(())
}

fn certify_sphere_direction_membership(
    direction: Vec3,
    active: Option<usize>,
    constraints: &[SphereWindowConstraint],
    tolerances: Tolerances,
    strict: bool,
) -> Result<bool> {
    certify_sphere_direction_membership_ignoring(
        direction,
        active,
        constraints,
        tolerances,
        strict,
        &[],
    )
}

fn certify_sphere_direction_membership_ignoring(
    direction: Vec3,
    active: Option<usize>,
    constraints: &[SphereWindowConstraint],
    tolerances: Tolerances,
    strict: bool,
    ignored: &[usize],
) -> Result<bool> {
    let arithmetic_allowance = 256.0 * f64::EPSILON;
    let enclosure = direction
        .to_array()
        .map(|value| Interval::new(value - arithmetic_allowance, value + arithmetic_allowance));
    let mut undecided = false;
    for (index, constraint) in constraints.iter().enumerate() {
        if active == Some(index) || ignored.contains(&index) {
            continue;
        }
        let margin = interval_dot(enclosure, constraint.normal.to_array().map(Interval::point))
            - Interval::point(constraint.offset);
        if margin.hi() < 0.0 {
            return Ok(false);
        }
        if margin.lo() <= if strict { tolerances.angular() } else { 0.0 } {
            undecided = true;
        }
    }
    if undecided {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window membership is inside the unresolved proof corridor",
        });
    }
    Ok(true)
}

fn certify_single_sphere_boundary_cycle(
    feasible: &[usize],
    arcs: &[CertifiedSphereBoundaryArc],
) -> Result<()> {
    if feasible.len() < 2 || arcs.len() != feasible.len() {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window fallback did not certify a positive-area single-cycle region",
        });
    }
    for &node in feasible {
        let degree = arcs
            .iter()
            .filter(|arc| arc.first == node || arc.second == node)
            .count();
        if degree != 2 {
            return Err(Error::InvalidGeometry {
                reason: "general coincident sphere window boundary cycle is not topologically certified",
            });
        }
    }
    let mut visited = vec![false; feasible.iter().copied().max().unwrap_or(0) + 1];
    let mut stack = vec![feasible[0]];
    while let Some(node) = stack.pop() {
        if visited[node] {
            continue;
        }
        visited[node] = true;
        for arc in arcs {
            if arc.first == node && !visited[arc.second] {
                stack.push(arc.second);
            } else if arc.second == node && !visited[arc.first] {
                stack.push(arc.first);
            }
        }
    }
    if feasible.iter().any(|&node| !visited[node]) {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window fallback found multiple boundary cycles",
        });
    }
    Ok(())
}

fn certify_sphere_region_interior(
    directions: &[Vec3],
    constraints: &[SphereWindowConstraint],
    tolerances: Tolerances,
) -> Result<bool> {
    let sum = directions
        .iter()
        .copied()
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, direction| sum + direction);
    let Some(interior) = sum.normalized() else {
        return Ok(false);
    };
    certify_sphere_direction_membership(interior, None, constraints, tolerances, true)
}

fn paired_general_sphere_direction(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    direction: Vec3,
    parameter_allowance: f64,
    tolerances: Tolerances,
) -> Result<PairedSphereSample> {
    let tolerance = parameter_allowance.max(tolerances.angular());
    let uv_a = sphere_uv_for_model_direction(direction, a, a_range, tolerance).ok_or(
        Error::InvalidGeometry {
            reason: "general sphere window boundary did not lift into the first chart",
        },
    )?;
    let uv_b = sphere_uv_for_model_direction(direction, b, b_range, tolerance).ok_or(
        Error::InvalidGeometry {
            reason: "general sphere window boundary did not lift into the second chart",
        },
    )?;
    paired_sphere_sample_at(a, uv_a, b, uv_b)
}

fn general_sphere_window_map(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    parameter_allowance: f64,
) -> GeneralSphereWindowMap {
    let a_axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let b_axes = [b.frame().x(), b.frame().y(), b.frame().z()];
    let second_from_first = b_axes.map(|target| a_axes.map(|source| target.dot(source)));
    let first_from_second = a_axes.map(|target| b_axes.map(|source| target.dot(source)));
    GeneralSphereWindowMap::new(
        a_range,
        b_range,
        second_from_first,
        first_from_second,
        parameter_allowance,
    )
}

fn interval_dot(first: [Interval; 3], second: [Interval; 3]) -> Interval {
    first
        .into_iter()
        .zip(second)
        .fold(Interval::point(0.0), |sum, (first, second)| {
            sum + first * second
        })
}

fn interval_cross(first: [Interval; 3], second: [Interval; 3]) -> [Interval; 3] {
    [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]
}

fn exact_signed_coordinate_axis_map(a: &Sphere, b: &Sphere) -> Option<[SignedCoordinateAxis; 3]> {
    let a_axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let b_axes = [b.frame().x(), b.frame().y(), b.frame().z()];
    let mut used = [false; 3];
    let mut result = [SignedCoordinateAxis {
        coordinate: 0,
        sign: 1.0,
    }; 3];
    for (b_index, b_axis) in b_axes.into_iter().enumerate() {
        let mut mapped = None;
        for (a_index, a_axis) in a_axes.into_iter().enumerate() {
            let sign = if b_axis == a_axis {
                Some(1.0)
            } else if b_axis == -a_axis {
                Some(-1.0)
            } else {
                None
            };
            if let Some(sign) = sign {
                if mapped.is_some() || used[a_index] {
                    return None;
                }
                mapped = Some(SignedCoordinateAxis {
                    coordinate: a_index,
                    sign,
                });
            }
        }
        let mapped = mapped?;
        used[mapped.coordinate] = true;
        result[b_index] = mapped;
    }
    used.into_iter()
        .all(core::convert::identity)
        .then_some(result)
}

fn exact_sphere_octant_signs(range: [ParamRange; 2], tolerances: Tolerances) -> Option<[f64; 3]> {
    let u_lo = exact_quarter_turn_index(range[0].lo, tolerances)?;
    let u_hi = exact_quarter_turn_index(range[0].hi, tolerances)?;
    if u_hi.checked_sub(u_lo)? != 1 {
        return None;
    }
    // Both endpoints passed the active angular-resolution corridor below. The
    // bidirectional chart map's 256*eps parameter allowance and the region's
    // 4*eps*parameter_scale residual term each dominate that corridor's
    // 2*eps*(|k|+1) endpoint bound. Thus both complete represented windows
    // remain pairable within the kernel's angular identity policy in both
    // directions. More distant representatives fail closed before a boundary
    // drift could change region/edge/point/miss dimension.
    let horizontal = match u_lo.rem_euclid(4) {
        0 => [1.0, 1.0],
        1 => [-1.0, 1.0],
        2 => [-1.0, -1.0],
        3 => [1.0, -1.0],
        _ => unreachable!("Euclidean remainder modulo four is in 0..4"),
    };
    let half_pi = core::f64::consts::FRAC_PI_2;
    let vertical = if range[1] == ParamRange::new(0.0, half_pi) {
        1.0
    } else if range[1] == ParamRange::new(-half_pi, 0.0) {
        -1.0
    } else {
        return None;
    };
    Some([horizontal[0], horizontal[1], vertical])
}

fn exact_quarter_turn_index(parameter: f64, tolerances: Tolerances) -> Option<i64> {
    let half_pi = core::f64::consts::FRAC_PI_2;
    let quotient = parameter / half_pi;
    let rounded = quotient.round();
    // Let u = EPSILON/2 be binary64 unit roundoff and h = fl(pi/2). For an
    // integer k, |h - pi/2| < u and the rounded product contributes less than
    // 2u|k|/(1-u), because h < 2. Thus
    //
    //   |fl(k*h) - k*pi/2| < 4u(|k| + 1) = 2*EPSILON*(|k| + 1).
    //
    // Admission requires that bound to fit inside the active angular identity
    // tolerance. At the default 1e-11 policy this accepts endpoint indices
    // |k| <= 22_516 and rejects |k| >= 22_517. This is deliberately much
    // smaller than the integer-exactness limit: near 2^52 the phase error is
    // measured in radians even though multiplication still round-trips.
    if !rounded.is_finite() || rounded.abs() > (1_u64 << 52) as f64 {
        return None;
    }
    let index = rounded as i64;
    let phase_error = Interval::point(2.0 * f64::EPSILON)
        * (Interval::point(index.unsigned_abs() as f64) + Interval::point(1.0));
    if phase_error.hi() > tolerances.angular() {
        return None;
    }
    ((index as f64) * half_pi == parameter).then_some(index)
}

fn coincident_orthogonal_sphere_octant_region(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    axis_map: [SignedCoordinateAxis; 3],
    signs: [f64; 3],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let mut boundary = Vec::with_capacity(3);
    let mut max_residual = coincident_sphere_set_residual_bound(a, a_range, b_range)?;
    for axis in 0..3 {
        let point = a.frame().origin() + axes[axis] * (a.radius() * signs[axis]);
        let sample = paired_sphere_point_in_windows(a, a_range, b, b_range, point, tolerances)?;
        max_residual = max_residual.max(sample.residual_bound);
        boundary.push(SurfaceSurfaceRegionVertex {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
        });
    }
    let region = SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: SurfaceRegionCorrespondence::OrthogonalSphereOctant(
            OrthogonalSphereOctantMap::new(
                a_range,
                b_range,
                axis_map.map(|mapped| mapped.coordinate as u8),
                axis_map.map(|mapped| mapped.sign),
            ),
        ),
        max_residual,
    };
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![region],
    )
}

#[allow(clippy::too_many_arguments)]
fn coincident_orthogonal_sphere_octant_edge(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    signs: [f64; 3],
    differing_axis: usize,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let common = (0..3)
        .filter(|&axis| axis != differing_axis)
        .collect::<Vec<_>>();
    let first_direction = axes[common[0]] * signs[common[0]];
    let second_direction = axes[common[1]] * signs[common[1]];
    let frame = Frame::new(
        a.frame().origin(),
        first_direction.cross(second_direction),
        first_direction,
    )?;
    let circle = Circle::new(frame, a.radius())?;
    let start =
        paired_sphere_point_in_windows(a, a_range, b, b_range, circle.eval(0.0), tolerances)?;
    let end = paired_sphere_point_in_windows(
        a,
        a_range,
        b,
        b_range,
        circle.eval(core::f64::consts::FRAC_PI_2),
        tolerances,
    )?;
    SurfaceSurfaceIntersections::canonicalized_complete(
        Vec::new(),
        vec![SurfaceSurfaceCurve {
            curve: SurfaceIntersectionCurve::Circle(circle),
            curve_range: ParamRange::new(0.0, core::f64::consts::FRAC_PI_2),
            uv_a_start: start.uv_a,
            uv_a_end: end.uv_a,
            uv_b_start: start.uv_b,
            uv_b_end: end.uv_b,
            kind: ContactKind::Tangent,
        }],
    )
}

fn coincident_orthogonal_sphere_octant_vertex(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    a_signs: [f64; 3],
    b_signs: [f64; 3],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let common_axis = (0..3)
        .find(|&axis| a_signs[axis] == b_signs[axis])
        .expect("two differing signs leave exactly one common coordinate");
    let axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let point = a.frame().origin() + axes[common_axis] * (a.radius() * a_signs[common_axis]);
    let sample = paired_sphere_point_in_windows(a, a_range, b, b_range, point, tolerances)?;
    let kind = if a.normal(sample.uv_a).is_none() || b.normal(sample.uv_b).is_none() {
        ContactKind::Singular
    } else {
        ContactKind::Tangent
    };
    SurfaceSurfaceIntersections::canonicalized_complete(
        vec![SurfaceSurfacePoint {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
            kind,
        }],
        Vec::new(),
    )
}

#[allow(clippy::too_many_arguments)]
fn intersect_arbitrary_sphere_octants(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    a_signs: [f64; 3],
    b_signs: [f64; 3],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let a_axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let b_axes = [b.frame().x(), b.frame().y(), b.frame().z()];
    let normals = [
        a_axes[0] * a_signs[0],
        a_axes[1] * a_signs[1],
        a_axes[2] * a_signs[2],
        b_axes[0] * b_signs[0],
        b_axes[1] * b_signs[1],
        b_axes[2] * b_signs[2],
    ];
    let rays = arbitrary_sphere_octant_rays(normals, tolerances)?;
    if rays.is_empty() {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }
    let mut directions = rays.iter().map(|ray| ray.direction).collect::<Vec<_>>();
    for first in 0..directions.len() {
        for second in first + 1..directions.len() {
            if directions[first].cross(directions[second]).norm() <= tolerances.angular() {
                return Err(Error::InvalidGeometry {
                    reason: "arbitrary sphere octant boundary planes exceed the certified angular corridor",
                });
            }
        }
    }
    if directions.len() >= 3 {
        sort_arbitrary_sphere_polygon(&mut directions)?;
        return arbitrary_sphere_octant_region(a, a_range, b, b_range, directions, tolerances);
    }

    if directions.len() == 2 {
        return arbitrary_sphere_octant_edge(
            a,
            a_range,
            b,
            b_range,
            directions[0],
            directions[1],
            tolerances,
        );
    }
    arbitrary_sphere_octant_point(a, a_range, b, b_range, directions[0], tolerances)
}

#[derive(Clone, Copy, Debug)]
struct ArbitrarySphereRay {
    direction: Vec3,
    first_plane: usize,
    second_plane: usize,
}

fn arbitrary_sphere_octant_rays(
    normals: [Vec3; 6],
    tolerances: Tolerances,
) -> Result<Vec<ArbitrarySphereRay>> {
    let mut rays = Vec::new();
    for first in 0..normals.len() {
        for second in first + 1..normals.len() {
            if sphere_planes_are_exactly_parallel(normals[first], normals[second]) {
                continue;
            }
            let cross = normals[first].cross(normals[second]);
            let cross_norm = cross.norm();
            if cross_norm <= tolerances.angular() {
                return Err(Error::InvalidGeometry {
                    reason: "arbitrary sphere octant boundary planes exceed the certified angular corridor",
                });
            }
            let direction = cross / cross_norm;
            for sign in [-1_i8, 1_i8] {
                if !normals.iter().all(|normal| {
                    let orientation = orient3d(
                        normals[first].to_array(),
                        normals[second].to_array(),
                        normal.to_array(),
                        [0.0; 3],
                    );
                    orientation.as_i8() * sign >= 0
                }) {
                    continue;
                }
                let candidate = direction * f64::from(sign);
                if rays.iter().any(|ray: &ArbitrarySphereRay| {
                    ray.direction.dot(candidate).is_sign_positive()
                        && sphere_plane_pairs_define_same_line(
                            normals,
                            first,
                            second,
                            ray.first_plane,
                            ray.second_plane,
                        )
                }) {
                    continue;
                }
                rays.push(ArbitrarySphereRay {
                    direction: candidate,
                    first_plane: first,
                    second_plane: second,
                });
            }
        }
    }
    rays.sort_by(|first, second| compare_sphere_directions(first.direction, second.direction));
    Ok(rays)
}

fn sphere_planes_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ]
    .into_iter()
    .all(|axis| {
        orient3d(
            first.to_array(),
            second.to_array(),
            axis.to_array(),
            [0.0; 3],
        ) == Orientation::Zero
    })
}

fn sphere_plane_pairs_define_same_line(
    normals: [Vec3; 6],
    first: usize,
    second: usize,
    other_first: usize,
    other_second: usize,
) -> bool {
    [other_first, other_second].into_iter().all(|other| {
        orient3d(
            normals[first].to_array(),
            normals[second].to_array(),
            normals[other].to_array(),
            [0.0; 3],
        ) == Orientation::Zero
    })
}

fn sort_arbitrary_sphere_polygon(rays: &mut [Vec3]) -> Result<()> {
    let interior = rays
        .iter()
        .copied()
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, ray| sum + ray)
        .normalized()
        .ok_or(Error::InvalidGeometry {
            reason: "arbitrary sphere octant polygon has no certified interior direction",
        })?;
    let x = (rays[0] - interior * rays[0].dot(interior))
        .normalized()
        .ok_or(Error::InvalidGeometry {
            reason: "arbitrary sphere octant polygon basis is ill-conditioned",
        })?;
    let y = interior.cross(x);
    rays.sort_by(|first, second| {
        math::atan2(first.dot(y), first.dot(x))
            .total_cmp(&math::atan2(second.dot(y), second.dot(x)))
            .then_with(|| compare_sphere_directions(*first, *second))
    });
    Ok(())
}

fn compare_sphere_directions(first: Vec3, second: Vec3) -> core::cmp::Ordering {
    first
        .x
        .total_cmp(&second.x)
        .then(first.y.total_cmp(&second.y))
        .then(first.z.total_cmp(&second.z))
}

fn arbitrary_sphere_octant_region(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    rays: Vec<Vec3>,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let parameter_allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range)?;
    let mut max_residual = arbitrary_sphere_octant_residual_bound(a, b, parameter_allowance)?;
    let mut boundary = Vec::with_capacity(rays.len());
    for ray in rays {
        let sample = paired_arbitrary_sphere_direction(a, a_range, b, b_range, ray, tolerances)?;
        max_residual = max_residual.max(sample.residual_bound);
        boundary.push(SurfaceSurfaceRegionVertex {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
        });
    }
    let region = SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: SurfaceRegionCorrespondence::ArbitrarySphereOctant(
            arbitrary_sphere_octant_map(a, a_range, b, b_range, parameter_allowance),
        ),
        max_residual,
    };
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![region],
    )
}

#[allow(clippy::too_many_arguments)]
fn arbitrary_sphere_octant_edge(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    first: Vec3,
    second: Vec3,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let normal = first.cross(second);
    let sine = normal.norm();
    let cosine = first.dot(second);
    if cosine <= -1.0 + tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "arbitrary sphere octant edge is antipodal and not uniquely bounded",
        });
    }
    let frame = Frame::new(a.frame().origin(), normal, first)?;
    let circle = Circle::new(frame, a.radius())?;
    let curve_range = ParamRange::new(0.0, math::atan2(sine, cosine));
    let start = paired_arbitrary_sphere_direction(a, a_range, b, b_range, first, tolerances)?;
    let end = paired_arbitrary_sphere_direction(a, a_range, b, b_range, second, tolerances)?;
    SurfaceSurfaceIntersections::canonicalized_complete(
        Vec::new(),
        vec![SurfaceSurfaceCurve {
            curve: SurfaceIntersectionCurve::Circle(circle),
            curve_range,
            uv_a_start: start.uv_a,
            uv_a_end: end.uv_a,
            uv_b_start: start.uv_b,
            uv_b_end: end.uv_b,
            kind: ContactKind::Tangent,
        }],
    )
}

fn arbitrary_sphere_octant_point(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    ray: Vec3,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let sample = paired_arbitrary_sphere_direction(a, a_range, b, b_range, ray, tolerances)?;
    let kind = if a.normal(sample.uv_a).is_none() || b.normal(sample.uv_b).is_none() {
        ContactKind::Singular
    } else {
        ContactKind::Tangent
    };
    SurfaceSurfaceIntersections::canonicalized_complete(
        vec![SurfaceSurfacePoint {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
            kind,
        }],
        Vec::new(),
    )
}

fn paired_arbitrary_sphere_direction(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    direction: Vec3,
    tolerances: Tolerances,
) -> Result<PairedSphereSample> {
    let parameter_tolerance =
        arbitrary_sphere_octant_parameter_allowance(a_range, b_range)?.max(tolerances.angular());
    let uv_a = sphere_uv_for_model_direction(direction, a, a_range, parameter_tolerance).ok_or(
        Error::InvalidGeometry {
            reason: "arbitrary sphere octant boundary did not lift into the first chart",
        },
    )?;
    let uv_b = sphere_uv_for_model_direction(direction, b, b_range, parameter_tolerance).ok_or(
        Error::InvalidGeometry {
            reason: "arbitrary sphere octant boundary did not lift into the second chart",
        },
    )?;
    paired_sphere_sample_at(a, uv_a, b, uv_b)
}

fn sphere_uv_for_model_direction(
    direction: Vec3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerance: f64,
) -> Option<[f64; 2]> {
    let local = Vec3::new(
        direction.dot(sphere.frame().x()),
        direction.dot(sphere.frame().y()),
        direction.dot(sphere.frame().z()),
    );
    let radial = (local.x * local.x + local.y * local.y).sqrt();
    let v = fit_scalar_parameter(math::atan2(local.z, radial), sphere_range[1], tolerance)?;
    let u = if radial <= tolerance {
        sphere_range[0].lo
    } else {
        fit_periodic_parameter(math::atan2(local.y, local.x), sphere_range[0], tolerance)?
    };
    Some([u, v])
}

fn arbitrary_sphere_octant_map(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    parameter_allowance: f64,
) -> ArbitrarySphereOctantMap {
    let a_axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let b_axes = [b.frame().x(), b.frame().y(), b.frame().z()];
    let second_from_first = b_axes.map(|target| a_axes.map(|source| target.dot(source)));
    let first_from_second = a_axes.map(|target| b_axes.map(|source| target.dot(source)));
    ArbitrarySphereOctantMap::new(
        a_range,
        b_range,
        second_from_first,
        first_from_second,
        parameter_allowance,
    )
}

const ARBITRARY_SPHERE_MAP_ROUNDOFF_UNITS: f64 = 512.0;

fn arbitrary_sphere_octant_parameter_allowance(
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
) -> Result<f64> {
    let periodic_error = orthogonal_periodic_phase_error(a_range, b_range)?;
    let allowance = Interval::point(periodic_error)
        + Interval::point(ARBITRARY_SPHERE_MAP_ROUNDOFF_UNITS * f64::EPSILON);
    allowance
        .hi()
        .is_finite()
        .then_some(allowance.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "arbitrary sphere octant parameter allowance is non-finite",
        })
}

fn arbitrary_sphere_octant_residual_bound(
    a: &Sphere,
    b: &Sphere,
    parameter_allowance: f64,
) -> Result<f64> {
    let a_axes = [a.frame().x(), a.frame().y(), a.frame().z()];
    let b_axes = [b.frame().x(), b.frame().y(), b.frame().z()];
    let projection_error = |source: [Vec3; 3], target: [Vec3; 3]| -> Result<f64> {
        let mut bound = Interval::point(0.0);
        for axis in source {
            let reconstructed = target
                .into_iter()
                .fold(Vec3::new(0.0, 0.0, 0.0), |sum, basis| {
                    sum + basis * axis.dot(basis)
                });
            bound = bound + Interval::point(conservative_sphere_vec_norm(axis - reconstructed)?);
        }
        bound
            .hi()
            .is_finite()
            .then_some(bound.hi())
            .ok_or(Error::InvalidGeometry {
                reason: "arbitrary sphere octant frame projection bound is non-finite",
            })
    };
    let frame_error = projection_error(a_axes, b_axes)?.max(projection_error(b_axes, a_axes)?);
    const ERROR_UNITS: f64 = 1024.0;
    let gamma = (ERROR_UNITS * f64::EPSILON) / (1.0 - ERROR_UNITS * f64::EPSILON);
    let origin_scale = a
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    let model_scale =
        Interval::point(origin_scale) + Interval::point(3.0) * Interval::point(a.radius());
    let coefficient_error =
        Interval::point(4.0) * Interval::point(a.radius()) * Interval::point(frame_error);
    let lift_error = Interval::point(2.0 * 3.0_f64.sqrt()) * Interval::point(gamma) * model_scale;
    // The same allowance retained by `ArbitrarySphereOctantMap` bounds both
    // remote periodic phase reconstruction and the fixed frame/trigonometric
    // roundoff used when mapping and clamping a chart parameter. Charging it
    // here therefore covers the complete domain admitted by the public map.
    let parameter_error = Interval::point(3.0_f64.sqrt())
        * Interval::point(a.radius())
        * Interval::point(parameter_allowance);
    let underflow_error = Interval::point(ERROR_UNITS * f64::from_bits(1));
    let bound = coefficient_error + lift_error + parameter_error + underflow_error;
    bound
        .hi()
        .is_finite()
        .then_some(bound.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "arbitrary sphere octant residual bound is non-finite",
        })
}

fn paired_sphere_point_in_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    point: Point3,
    tolerances: Tolerances,
) -> Result<PairedSphereSample> {
    let periodic_error = orthogonal_periodic_phase_error(a_range, b_range)?;
    let uv_a = sphere_uv_at_with_parameter_tolerance(
        point,
        a,
        a_range,
        parameter_tolerance(a.radius(), tolerances).max(periodic_error),
        tolerances.linear(),
    )
    .ok_or(Error::InvalidGeometry {
        reason: "orthogonal sphere octant boundary did not lift into the first chart",
    })?;
    let uv_b = sphere_uv_at_with_parameter_tolerance(
        point,
        b,
        b_range,
        parameter_tolerance(b.radius(), tolerances).max(periodic_error),
        tolerances.linear(),
    )
    .ok_or(Error::InvalidGeometry {
        reason: "orthogonal sphere octant boundary did not lift into the second chart",
    })?;
    paired_sphere_sample_at(a, uv_a, b, uv_b)
}

fn coincident_sphere_set_residual_bound(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
) -> Result<f64> {
    // The accepted frames are an exact component-wise signed permutation and
    // the centers/radii compare bit-for-bit equal. In exact arithmetic the
    // local-coordinate permutation, atan2 reconstruction, and frame lift
    // therefore describe the same point. Bound only their floating-point
    // realization here.
    //
    // `math::{sincos,atan2}` are each accurate to < 1 ulp. Counting one
    // epsilon per transcendental output, the source local products, hypot,
    // atan2 reconstruction, target local products, and both three-axis model
    // lifts consume fewer than 256 error units per model coordinate. This
    // includes the normalization condition number: the computed source local
    // vector remains within 6 eps of unit length, so normalization amplifies
    // by less than 1 / (1 - 6 eps). The standard gamma_n bound below uses eps
    // (twice the IEEE unit roundoff), so it also covers subexpression grouping.
    const ERROR_UNITS: f64 = 256.0;
    let gamma = (ERROR_UNITS * f64::EPSILON) / (1.0 - ERROR_UNITS * f64::EPSILON);

    let origin_scale = a
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    // Each coordinate of X*q_x + Y*q_y + Z*q_z has absolute sum at most 3.
    let model_scale =
        Interval::point(origin_scale) + Interval::point(3.0) * Interval::point(a.radius());

    // Fitting atan2 back into a remote periodic window evaluates
    // raw + k*TAU. TAU's representation error, the multiplication, and the
    // addition contribute at most four eps times the represented parameter
    // magnitude. This is the only term that grows with large turn indices.
    let periodic_phase_error = Interval::point(orthogonal_periodic_phase_error(a_range, b_range)?);

    // Two independently rounded model-space evaluations are covered by the
    // factor two; sqrt(3) converts the coordinatewise bound to Euclidean
    // distance. Interval operations widen every step by one ulp.
    let lift_error = Interval::point(2.0 * 3.0_f64.sqrt()) * Interval::point(gamma) * model_scale;
    let periodic_error =
        Interval::point(3.0_f64.sqrt()) * Interval::point(a.radius()) * periodic_phase_error;
    let underflow_error = Interval::point(ERROR_UNITS * f64::from_bits(1));
    let bound = lift_error + periodic_error + underflow_error;
    bound
        .hi()
        .is_finite()
        .then_some(bound.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "orthogonal sphere octant residual bound is non-finite",
        })
}

fn orthogonal_periodic_phase_error(
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
) -> Result<f64> {
    let parameter_scale = a_range
        .into_iter()
        .chain(b_range)
        .flat_map(|range| [range.lo.abs(), range.hi.abs()])
        .fold(0.0_f64, f64::max);
    let error = Interval::point(4.0 * f64::EPSILON)
        * (Interval::point(parameter_scale) + Interval::point(2.0 * core::f64::consts::TAU));
    error
        .hi()
        .is_finite()
        .then_some(error.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "orthogonal sphere octant periodic phase bound is non-finite",
        })
}

#[derive(Clone, Copy, Debug)]
struct CoincidentSphereMap {
    sign: f64,
    u_phase: f64,
}

#[derive(Clone, Copy, Debug)]
struct PairedSphereSample {
    point: Point3,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    residual: f64,
    residual_bound: f64,
}

fn intersect_coincident_sphere_windows(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_coincident_sphere_ranges(a_range, b_range)?;
    let tau = core::f64::consts::TAU;
    let sign = if a.frame().z().dot(b.frame().z()).is_sign_negative() {
        -1.0
    } else {
        1.0
    };
    let map = CoincidentSphereMap {
        sign,
        u_phase: math::atan2(
            a.frame().x().dot(b.frame().y()),
            a.frame().x().dot(b.frame().x()),
        ),
    };
    let parameter_tolerance = parameter_tolerance(a.radius(), tolerances);
    let u_overlaps = periodic_preimage_overlaps(
        a_range[0],
        b_range[0],
        map.sign,
        map.u_phase,
        tau,
        parameter_tolerance,
        "coincident sphere periodic chart shift is outside the exact integer corridor",
    )?;
    let Some(v_overlap) =
        affine_preimage_overlap(a_range[1], b_range[1], map.sign, 0.0, parameter_tolerance)
    else {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    };

    if u_overlaps.is_empty() {
        let mut points = Vec::new();
        for pole in [-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2] {
            if fit_scalar_parameter(pole, v_overlap, parameter_tolerance).is_some() {
                let sample =
                    paired_sphere_pole_sample(a, a_range, b, b_range, pole, map, tolerances)?;
                push_coincident_sphere_point(
                    &mut points,
                    sample,
                    ContactKind::Singular,
                    tolerances,
                );
            }
        }
        return SurfaceSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut points = Vec::new();
    let mut curves = Vec::new();
    let mut regions = Vec::new();
    for overlap in u_overlaps {
        let u_positive = overlap.a.width() > parameter_tolerance;
        let v_positive = v_overlap.width() > parameter_tolerance;
        let v_midpoint = range_midpoint(v_overlap);
        match (u_positive, v_positive) {
            (true, true) => regions.push(coincident_sphere_region(
                a, b, overlap, v_overlap, b_range, map, tolerances,
            )?),
            (true, false) if is_sphere_pole(v_midpoint, parameter_tolerance) => {
                let sample = paired_sphere_sample(
                    a,
                    b,
                    [range_midpoint(overlap.a), v_midpoint],
                    overlap.shift,
                    b_range,
                    map,
                    tolerances,
                )?;
                push_coincident_sphere_point(
                    &mut points,
                    sample,
                    ContactKind::Singular,
                    tolerances,
                );
            }
            (true, false) => curves.push(coincident_sphere_latitude_branch(
                a, b, overlap, v_midpoint, b_range, map, tolerances,
            )?),
            (false, true) => curves.push(coincident_sphere_meridian_branch(
                a,
                b,
                range_midpoint(overlap.a),
                overlap.shift,
                v_overlap,
                b_range,
                map,
                tolerances,
            )?),
            (false, false) => {
                let sample = paired_sphere_sample(
                    a,
                    b,
                    [range_midpoint(overlap.a), v_midpoint],
                    overlap.shift,
                    b_range,
                    map,
                    tolerances,
                )?;
                let kind = if is_sphere_pole(v_midpoint, parameter_tolerance) {
                    ContactKind::Singular
                } else {
                    ContactKind::Tangent
                };
                push_coincident_sphere_point(&mut points, sample, kind, tolerances);
            }
        }
    }
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(points, curves, regions)
}

fn validate_coincident_sphere_ranges(
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
) -> Result<()> {
    let tau = core::f64::consts::TAU;
    validate_period_span(
        a_range[0],
        tau,
        0.0,
        "coincident sphere longitude windows cannot span more than one turn",
    )?;
    validate_period_span(
        b_range[0],
        tau,
        0.0,
        "coincident sphere longitude windows cannot span more than one turn",
    )?;
    let half_pi = core::f64::consts::FRAC_PI_2;
    if [a_range[1], b_range[1]]
        .into_iter()
        .any(|range| range.lo < -half_pi || range.hi > half_pi)
    {
        return Err(Error::InvalidGeometry {
            reason: "coincident sphere latitude windows must stay inside the natural pole range",
        });
    }
    Ok(())
}

fn is_sphere_pole(latitude: f64, tolerance: f64) -> bool {
    (latitude.abs() - core::f64::consts::FRAC_PI_2).abs() <= tolerance
}

#[allow(clippy::too_many_arguments)]
fn coincident_sphere_region(
    a: &Sphere,
    b: &Sphere,
    u: PeriodicOverlapPiece,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentSphereMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceRegion> {
    let mut boundary = Vec::with_capacity(4);
    let mut max_residual = coincident_sphere_whole_residual_bound(a, b, u, v, b_range, map)?;
    for uv_a in [
        [u.a.lo, v.lo],
        [u.a.hi, v.lo],
        [u.a.hi, v.hi],
        [u.a.lo, v.hi],
    ] {
        let sample = paired_sphere_sample(a, b, uv_a, u.shift, b_range, map, tolerances)?;
        max_residual = max_residual.max(sample.residual_bound);
        boundary.push(SurfaceSurfaceRegionVertex {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
        });
    }
    Ok(SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: super::result::SurfaceRegionCorrespondence::Polygonal,
        max_residual,
    })
}

#[allow(clippy::too_many_arguments)]
fn coincident_sphere_latitude_branch(
    a: &Sphere,
    b: &Sphere,
    u: PeriodicOverlapPiece,
    v: f64,
    b_range: [ParamRange; 2],
    map: CoincidentSphereMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_sphere_sample(a, b, [u.a.lo, v], u.shift, b_range, map, tolerances)?;
    let end = paired_sphere_sample(a, b, [u.a.hi, v], u.shift, b_range, map, tolerances)?;
    let (sin_v, cos_v) = math::sincos(v);
    let frame = Frame::new(
        a.frame().origin() + a.frame().z() * (a.radius() * sin_v),
        a.frame().z(),
        a.frame().x(),
    )?;
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Circle(Circle::new(frame, a.radius() * cos_v.abs())?),
        curve_range: u.a,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn coincident_sphere_meridian_branch(
    a: &Sphere,
    b: &Sphere,
    u: f64,
    shift: f64,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentSphereMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_sphere_sample(a, b, [u, v.lo], shift, b_range, map, tolerances)?;
    let end = paired_sphere_sample(a, b, [u, v.hi], shift, b_range, map, tolerances)?;
    let (sin_u, cos_u) = math::sincos(u);
    let radial = a.frame().x() * cos_u + a.frame().y() * sin_u;
    let tangential = a.frame().y() * cos_u - a.frame().x() * sin_u;
    let frame = Frame::new(a.frame().origin(), -tangential, radial)?;
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Circle(Circle::new(frame, a.radius())?),
        curve_range: v,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn paired_sphere_sample(
    a: &Sphere,
    b: &Sphere,
    uv_a: [f64; 2],
    u_shift: f64,
    b_range: [ParamRange; 2],
    map: CoincidentSphereMap,
    tolerances: Tolerances,
) -> Result<PairedSphereSample> {
    let parameter_tolerance = parameter_tolerance(a.radius(), tolerances);
    let [Some(u_b), Some(v_b)] = [
        fit_scalar_parameter(
            map.sign * uv_a[0] + map.u_phase + u_shift,
            b_range[0],
            parameter_tolerance,
        ),
        fit_scalar_parameter(map.sign * uv_a[1], b_range[1], parameter_tolerance),
    ] else {
        return Err(Error::InvalidGeometry {
            reason: "coincident sphere chart overlap did not lift into both source windows",
        });
    };
    paired_sphere_sample_at(a, [uv_a[0], uv_a[1]], b, [u_b, v_b])
}

#[allow(clippy::too_many_arguments)]
fn paired_sphere_pole_sample(
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    pole: f64,
    map: CoincidentSphereMap,
    tolerances: Tolerances,
) -> Result<PairedSphereSample> {
    let parameter_tolerance = parameter_tolerance(a.radius(), tolerances);
    let Some(v_b) = fit_scalar_parameter(map.sign * pole, b_range[1], parameter_tolerance) else {
        return Err(Error::InvalidGeometry {
            reason: "coincident sphere pole did not lift into the second source window",
        });
    };
    paired_sphere_sample_at(a, [a_range[0].lo, pole], b, [b_range[0].lo, v_b])
}

fn paired_sphere_sample_at(
    a: &Sphere,
    uv_a: [f64; 2],
    b: &Sphere,
    uv_b: [f64; 2],
) -> Result<PairedSphereSample> {
    let pa = a.eval(uv_a);
    let pb = b.eval(uv_b);
    let residual = pa.dist(pb);
    let residual_bound =
        conservative_sphere_point_distance(pa, pb).ok_or(Error::InvalidGeometry {
            reason: "coincident sphere residual arithmetic is non-finite",
        })?;
    Ok(PairedSphereSample {
        point: (pa + pb) / 2.0,
        uv_a,
        uv_b,
        residual,
        residual_bound,
    })
}

fn push_coincident_sphere_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    sample: PairedSphereSample,
    kind: ContactKind,
    tolerances: Tolerances,
) {
    if points
        .iter()
        .any(|point| point.point.dist(sample.point) <= tolerances.linear())
    {
        return;
    }
    points.push(SurfaceSurfacePoint {
        point: sample.point,
        uv_a: sample.uv_a,
        uv_b: sample.uv_b,
        residual: sample.residual,
        kind,
    });
}

fn coincident_sphere_whole_residual_bound(
    a: &Sphere,
    b: &Sphere,
    u: PeriodicOverlapPiece,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentSphereMap,
) -> Result<f64> {
    let (sin_phase, cos_phase) = math::sincos(map.u_phase);
    let b_cos = b.frame().x() * cos_phase + b.frame().y() * sin_phase;
    let b_sin = (b.frame().y() * cos_phase - b.frame().x() * sin_phase) * map.sign;
    let origin_difference = a.frame().origin() - b.frame().origin();
    let cosine_difference = a.frame().x() * a.radius() - b_cos * b.radius();
    let sine_difference = a.frame().y() * a.radius() - b_sin * b.radius();
    let axial_difference = a.frame().z() * a.radius() - b.frame().z() * (b.radius() * map.sign);

    let mut bound = Interval::point(conservative_sphere_vec_norm(origin_difference)?);
    bound = bound + Interval::point(conservative_sphere_vec_norm(cosine_difference)?);
    bound = bound + Interval::point(conservative_sphere_vec_norm(sine_difference)?);
    bound = bound + Interval::point(conservative_sphere_vec_norm(axial_difference)?);

    let parameter_scale =
        u.a.lo
            .abs()
            .max(u.a.hi.abs())
            .max(v.lo.abs())
            .max(v.hi.abs())
            .max(b_range[0].lo.abs())
            .max(b_range[0].hi.abs())
            .max(b_range[1].lo.abs())
            .max(b_range[1].hi.abs());
    let model_scale = a
        .frame()
        .origin()
        .norm()
        .max(b.frame().origin().norm())
        .max(a.radius())
        .max(parameter_scale)
        .max(1.0);
    let result = bound + Interval::point(8192.0 * f64::EPSILON) * Interval::point(model_scale);
    result
        .hi()
        .is_finite()
        .then_some(result.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident sphere whole-region residual bound is non-finite",
        })
}

fn conservative_sphere_vec_norm(value: Vec3) -> Result<f64> {
    let components = value.to_array().map(Interval::point);
    let squared = components
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square());
    squared
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident sphere coefficient residual bound is non-finite",
        })
}

fn conservative_sphere_point_distance(a: Point3, b: Point3) -> Option<f64> {
    let difference = [
        Interval::point(a.x) - Interval::point(b.x),
        Interval::point(a.y) - Interval::point(b.y),
        Interval::point(a.z) - Interval::point(b.z),
    ];
    difference
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square())
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
}

#[allow(clippy::too_many_arguments)]
fn add_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    a: &Sphere,
    a_range: [ParamRange; 2],
    b: &Sphere,
    b_range: [ParamRange; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(uv_a) = sphere_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = sphere_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    let kind = if a.normal(uv_a).is_none() || b.normal(uv_b).is_none() {
        ContactKind::Singular
    } else {
        kind
    };
    if let Some(point) = accept_surface_surface_candidate(a, uv_a, b, uv_b, kind, tolerances) {
        push_point(points, point, tolerances);
    }
}

fn tangent_point(origin: Point3, axis: Vec3, center_offset: f64, radius: f64) -> Point3 {
    let sign = if center_offset < 0.0 { -1.0 } else { 1.0 };
    origin + axis * (sign * radius)
}

fn sphere_uv_at(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    sphere_uv_at_with_parameter_tolerance(
        point,
        sphere,
        sphere_range,
        parameter_tolerance(sphere.radius(), tolerances),
        tolerances.linear(),
    )
}

fn sphere_uv_at_with_parameter_tolerance(
    point: Point3,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    parameter_tolerance: f64,
    linear_tolerance: f64,
) -> Option<[f64; 2]> {
    let local = sphere.frame().to_local(point);
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_v = math::atan2(local.z, xy);
    let v = fit_scalar_parameter(raw_v, sphere_range[1], parameter_tolerance)?;
    let u = if xy <= linear_tolerance {
        sphere_range[0].lo
    } else {
        let raw_u = math::atan2(local.y, local.x);
        fit_periodic_parameter(raw_u, sphere_range[0], parameter_tolerance)?
    };
    Some([u, v])
}

fn push_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    candidate: SurfaceSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn squared_tolerance(
    center_distance: f64,
    radius_a: f64,
    radius_b: f64,
    tolerances: Tolerances,
) -> f64 {
    tolerances.linear() * (center_distance + radius_a + radius_b).max(1.0)
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/sphere intersection requires finite non-reversed first-sphere ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "sphere/sphere intersection requires finite non-reversed second-sphere ranges",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn disconnected_five_cell_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 1.0979596226858495;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-1.784130623703192, 3.1666669823039633),
                ParamRange::new(-0.6257036326267779, -0.034628902538759054),
            ],
            [
                ParamRange::new(-0.08900954540924966, 4.688766215574653),
                ParamRange::new(-0.795717438717723, 0.2960335275545031),
            ],
        )
    }

    fn seven_cell_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.9054345637982375;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-0.5929589703265958, 4.542973834373202),
                ParamRange::new(-0.8524779757705633, 0.9706234964995796),
            ],
            [
                ParamRange::new(-0.4436018548841436, 3.5517236306126825),
                ParamRange::new(-0.6769143527370888, 0.8493651816976383),
            ],
        )
    }

    fn opposite_corner_empty_seven_cell_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.4122861498111098;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-2.05071789320265, 2.7033606413045987),
                ParamRange::new(-1.1927387289259745, 1.2989310446048354),
            ],
            [
                ParamRange::new(-1.9242493983177624, 2.9793519615452766),
                ParamRange::new(-0.8315857731105628, 1.446288841486333),
            ],
        )
    }

    fn exact_polar_window_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.4;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-0.5, 0.5),
                ParamRange::new(0.3, core::f64::consts::FRAC_PI_2),
            ],
            [ParamRange::new(2.7, 3.5), ParamRange::new(0.6, 1.3)],
        )
    }

    fn exact_polar_by_wide_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, a_range, mut b_range) = exact_polar_window_fixture();
        b_range[0] = ParamRange::new(1.6, 4.9);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_adjacent_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2])
    {
        let (a, b, a_range, mut b_range) = exact_polar_window_fixture();
        b_range[0] = ParamRange::new(1.9, 1.9 + core::f64::consts::PI);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_cap_row_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-1.55, 1.55);
        a_range[1].lo = -1.0;
        let u_lo = core::f64::consts::FRAC_PI_2 + 2.0 * core::f64::consts::PI;
        let u_hi = u_lo + core::f64::consts::PI;
        b_range[0] = ParamRange::new(u_lo, u_hi);
        b_range[1] = ParamRange::new(1.13, 1.5);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_non_cap_row_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-1.55, 1.55);
        let turn = 2.0 * core::f64::consts::PI;
        b_range[0] = ParamRange::new(
            -core::f64::consts::FRAC_PI_2 + turn,
            core::f64::consts::FRAC_PI_2 + turn,
        );
        b_range[1] = ParamRange::new(0.75, 0.9);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_lower_adjacent_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[1].lo = -1.3;
        let turn = 2.0 * core::f64::consts::PI;
        b_range[0] = ParamRange::new(1.9 - core::f64::consts::PI + turn, 1.9 + turn);
        b_range[1] = ParamRange::new(-1.3, -0.6);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_vertical_adjacent_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-1.2, -0.8);
        a_range[1].lo = 1.0 - core::f64::consts::FRAC_PI_2;
        b_range[0] = ParamRange::new(-4.4, -4.4 + core::f64::consts::PI);
        b_range[1] = ParamRange::new(0.0, 0.8);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_cap_right_l_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-1.8, -0.8);
        a_range[1] = ParamRange::new(-1.0, core::f64::consts::FRAC_PI_2);
        b_range[0] = ParamRange::new(-4.8, -4.8 + 3.6);
        b_range[1] = ParamRange::new(0.0, 1.0);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_lower_middle_l_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-1.8, -0.8);
        a_range[1] = ParamRange::new(
            1.0 - core::f64::consts::FRAC_PI_2,
            core::f64::consts::FRAC_PI_2,
        );
        b_range[0] = ParamRange::new(-3.8, -3.8 + 3.6);
        b_range[1] = ParamRange::new(-0.4, 0.6);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_cap_row_right_four_path_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-2.4, -0.8);
        a_range[1] = ParamRange::new(-1.0, core::f64::consts::FRAC_PI_2);
        b_range[0] = ParamRange::new(-4.6, -4.6 + 4.5);
        b_range[1] = ParamRange::new(0.4, 1.4);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_zigzag_four_path_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-2.4, -0.8);
        a_range[1] = ParamRange::new(
            1.0 - core::f64::consts::FRAC_PI_2,
            core::f64::consts::FRAC_PI_2,
        );
        b_range[0] = ParamRange::new(-4.0, -4.0 + 4.5);
        b_range[1] = ParamRange::new(-0.2, 1.0);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_corner_empty_five_cell_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = exact_polar_window_fixture();
        a_range[0] = ParamRange::new(-2.8, -0.8);
        a_range[1] = ParamRange::new(-1.0, core::f64::consts::FRAC_PI_2);
        b_range[0] = ParamRange::new(-4.8, 0.2999999999999998);
        b_range[1] = ParamRange::new(0.2, 1.4);
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_edge_empty_five_cell_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.8246411620561659;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-1.4619434836661054, 1.39366191877374),
                ParamRange::new(0.017611340128706576, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(4.3262436700502995, 9.432940315058415),
                ParamRange::new(0.6920611792777787, 1.0961109123453459),
            ],
        )
    }

    fn exact_polar_by_wide_six_cell_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2])
    {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.14716980102990423;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-0.6749164680039561, 1.1201771047460314),
                ParamRange::new(-0.9492200325713294, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(4.444932358245097, 8.447686067139076),
                ParamRange::new(-0.19969245736453534, 1.1090931156623112),
            ],
        )
    }

    fn exact_polar_by_wide_four_cell_fixture(
        angle: f64,
        a_range: [ParamRange; 2],
        b_range: [ParamRange; 2],
    ) -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (a, b, a_range, b_range)
    }

    fn exact_polar_by_wide_left_four_cell_cycle_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.8094306659089616,
            [
                ParamRange::new(0.10072528893857235, 2.636121125850239),
                ParamRange::new(-0.34517949012112936, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(1.1036352876488191, 5.540477288708379),
                ParamRange::new(-1.137007449448292, -0.011598091482376782),
            ],
        )
    }

    fn exact_polar_by_wide_right_four_cell_cycle_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.3840978458329001,
            [
                ParamRange::new(-0.7839294031122588, 0.907057870675744),
                ParamRange::new(-1.425706528264274, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(2.9631102874774697, 8.194862608035262),
                ParamRange::new(-0.11521651883525941, 0.9971690060375422),
            ],
        )
    }

    fn exact_polar_by_wide_lower_stem_four_cell_t_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.8390485119441324,
            [
                ParamRange::new(0.7546277362780804, 3.7939203965000243),
                ParamRange::new(-0.2375268585430419, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(-0.027484612266702513, 6.062230214562831),
                ParamRange::new(-1.1546508280435746, 0.47953555934550596),
            ],
        )
    }

    fn exact_polar_by_wide_upper_stem_four_cell_t_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.8675104737097041,
            [
                ParamRange::new(-0.8336879368685368, 0.2832840690395422),
                ParamRange::new(-1.4950993192780773, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(3.148262199977504, 8.528650698835483),
                ParamRange::new(-0.0029988539136878156, 1.299680704998795),
            ],
        )
    }

    fn exact_polar_by_wide_disconnected_vertical_pairs_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.4,
            [
                ParamRange::new(-0.8, 0.2),
                ParamRange::new(-1.3, core::f64::consts::FRAC_PI_2),
            ],
            [ParamRange::new(-0.5, 5.65), ParamRange::new(-1.5, 0.6)],
        )
    }

    fn exact_polar_by_wide_zero_zero_singleton_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            1.2641733116298481,
            [
                ParamRange::new(-2.1792714803605975, 0.23159622415365666),
                ParamRange::new(-0.8443620544103326, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(0.2770343914821476, 4.458851031712419),
                ParamRange::new(0.35108647851526786, 1.263262146312806),
            ],
        )
    }

    fn exact_polar_by_wide_one_two_singleton_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            1.0421460686226904,
            [
                ParamRange::new(-1.6960087068831846, -0.17663691928352354),
                ParamRange::new(-0.5068962186083287, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(3.446220149698494, 9.601890541153121),
                ParamRange::new(-0.20048894581341026, 0.6664998186963655),
            ],
        )
    }

    fn exact_polar_by_wide_one_zero_singleton_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.7689495924455754,
            [
                ParamRange::new(-0.6725341301611243, 0.8021638258595267),
                ParamRange::new(-0.8110127562233151, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(1.555315994398029, 7.6047049454535),
                ParamRange::new(-1.048198669914953, 0.8057776574051865),
            ],
        )
    }

    fn exact_polar_by_wide_zero_two_singleton_fixture()
    -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        exact_polar_by_wide_four_cell_fixture(
            0.8862379569121048,
            [
                ParamRange::new(-0.3684687150823871, 0.8669729245355371),
                ParamRange::new(-0.22128242474864823, core::f64::consts::FRAC_PI_2),
            ],
            [
                ParamRange::new(-5.545040393598143, -0.41537863217402204),
                ParamRange::new(-0.4077140983624745, 1.1640971788151662),
            ],
        )
    }

    type PolarWideChildRegions = (
        [[ParamRange; 2]; GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT],
        [[ParamRange; 2]; GENERAL_SPHERE_WIDE_PIECE_LIMIT],
        Vec<([usize; 2], SurfaceSurfaceRegion)>,
        Vec<[usize; 2]>,
    );

    fn exact_polar_by_wide_child_regions(
        a: &Sphere,
        a_range: [ParamRange; 2],
        b: &Sphere,
        b_range: [ParamRange; 2],
    ) -> PolarWideChildRegions {
        try_polar_by_wide_child_regions(a, a_range, b, b_range)
            .expect("the exact polar-by-wide fixture must certify all six children")
    }

    fn try_polar_by_wide_child_regions(
        a: &Sphere,
        a_range: [ParamRange; 2],
        b: &Sphere,
        b_range: [ParamRange; 2],
    ) -> Option<PolarWideChildRegions> {
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).ok()?;
        let polar_pieces = decompose_general_sphere_polar_window(a_range).ok()?;
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).ok()?;
        let mut occupied = Vec::new();
        let mut empty = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).ok()?;
                let mut child = certify_general_sphere_window_arrangement(
                    a,
                    polar_piece,
                    b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .ok()?;
                if child.is_proven_empty() {
                    empty.push([polar_index, wide_index]);
                } else {
                    if !child.is_complete()
                        || !child.points.is_empty()
                        || !child.curves.is_empty()
                        || child.regions.len() != 1
                    {
                        return None;
                    }
                    occupied.push((
                        [polar_index, wide_index],
                        child
                            .regions
                            .pop()
                            .expect("one occupied polar-by-wide child was required"),
                    ));
                }
            }
        }
        Some((polar_pieces, wide_pieces, occupied, empty))
    }

    fn eight_cell_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let (a, b, mut a_range, mut b_range) = seven_cell_fixture();
        a_range[1].lo = -1.0834779757705633;
        b_range[1].lo = -1.1949143527370887;
        (a, b, a_range, b_range)
    }

    fn corner_empty_eight_cell_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 1.0232556972921847;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-0.5962248359795472, 3.724488143015183),
                ParamRange::new(-0.5588244443273177, 1.2462339159163787),
            ],
            [
                ParamRange::new(-0.02643539714706078, 3.7993244942837436),
                ParamRange::new(-0.6331431121684868, 1.3252780316345976),
            ],
        )
    }

    fn nine_cell_fixture() -> (Sphere, Sphere, [ParamRange; 2], [ParamRange; 2]) {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.941731645814849;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        (
            a,
            b,
            [
                ParamRange::new(-0.6905707622863242, 2.7325627610063625),
                ParamRange::new(-1.0093591690898873, 1.403712886650005),
            ],
            [
                ParamRange::new(-0.5347960267606893, 4.295577322685924),
                ParamRange::new(-1.0272317177041015, 1.3776408967251323),
            ],
        )
    }

    #[test]
    fn general_window_proof_limits_are_exact_at_n_and_n_minus_one() {
        let a = Sphere::new(Frame::world(), 1.0).unwrap();
        let angle = 0.4;
        let b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
                Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        let a_range = [ParamRange::new(0.15, 1.25), ParamRange::new(-0.55, 0.65)];
        let b_range = [ParamRange::new(0.05, 1.15), ParamRange::new(-0.45, 0.55)];
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();

        let hit = certify_general_sphere_windows(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
            GENERAL_SPHERE_WINDOW_ARC_LIMIT,
            allowance,
        )
        .unwrap();
        assert!(hit.is_complete());

        assert_eq!(
            certify_general_sphere_windows(
                &a,
                a_range,
                &b,
                b_range,
                Tolerances::default(),
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT - 1,
                GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                allowance,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere window proof pair limit exhausted"
            }
        );

        let empty_a_range = [ParamRange::new(0.1, 0.7), ParamRange::new(-0.3, 0.3)];
        let empty_b_range = [ParamRange::new(2.0, 2.6), ParamRange::new(-0.3, 0.3)];
        let empty_allowance =
            arbitrary_sphere_octant_parameter_allowance(empty_a_range, empty_b_range).unwrap();
        const EMPTY_EXEMPLAR_ARC_LIMIT: usize = 96;
        let empty = certify_general_sphere_windows(
            &a,
            empty_a_range,
            &b,
            empty_b_range,
            Tolerances::default(),
            GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
            EMPTY_EXEMPLAR_ARC_LIMIT,
            empty_allowance,
        )
        .unwrap();
        assert!(empty.is_proven_empty());
        assert_eq!(
            certify_general_sphere_windows(
                &a,
                empty_a_range,
                &b,
                empty_b_range,
                Tolerances::default(),
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                EMPTY_EXEMPLAR_ARC_LIMIT - 1,
                empty_allowance,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere window proof arc limit exhausted"
            }
        );

        let curve_a_range = [ParamRange::new(0.0, 0.8), ParamRange::new(-0.3, 0.5)];
        let curve_b_range = [ParamRange::new(-0.8, 0.0), ParamRange::new(-0.2, 0.4)];
        let curve_allowance =
            arbitrary_sphere_octant_parameter_allowance(curve_a_range, curve_b_range).unwrap();
        const COLLAPSED_CURVE_ARC_LIMIT: usize = 12;
        let curve = certify_general_sphere_windows(
            &a,
            curve_a_range,
            &b,
            curve_b_range,
            Tolerances::default(),
            GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
            COLLAPSED_CURVE_ARC_LIMIT,
            curve_allowance,
        )
        .unwrap();
        assert!(curve.is_complete());
        assert_eq!(curve.curves.len(), 1);
        assert_eq!(
            certify_general_sphere_windows(
                &a,
                curve_a_range,
                &b,
                curve_b_range,
                Tolerances::default(),
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                COLLAPSED_CURVE_ARC_LIMIT - 1,
                curve_allowance,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere window proof arc limit exhausted"
            }
        );

        let wide_angle = 0.2;
        let wide_b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(wide_angle), 0.0, math::cos(wide_angle)),
                Vec3::new(math::cos(wide_angle), 0.0, -math::sin(wide_angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        let wide_a_range = [
            ParamRange::new(-0.6, core::f64::consts::PI - 0.6),
            ParamRange::new(-0.8, 0.8),
        ];
        let wide_b_range = [ParamRange::new(-0.25, 0.25), ParamRange::new(-0.2, 0.2)];
        let wide_allowance =
            arbitrary_sphere_octant_parameter_allowance(wide_a_range, wide_b_range).unwrap();
        let wide = certify_single_wide_sphere_window_union(
            &a,
            wide_a_range,
            &wide_b,
            wide_b_range,
            true,
            Tolerances::default(),
            wide_allowance,
            GENERAL_SPHERE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(wide.is_complete());
        assert_eq!(wide.regions.len(), 1);
        let wide_swapped = certify_single_wide_sphere_window_union(
            &wide_b,
            wide_b_range,
            &a,
            wide_a_range,
            false,
            Tolerances::default(),
            wide_allowance,
            GENERAL_SPHERE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert_eq!(wide.clone().swapped(), wide_swapped);

        assert_eq!(
            certify_single_wide_sphere_window_union(
                &a,
                wide_a_range,
                &wide_b,
                wide_b_range,
                true,
                Tolerances::default(),
                wide_allowance,
                GENERAL_SPHERE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_WIDE_ARC_LIMIT,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere wide-window piece limit exhausted"
            }
        );
        assert_eq!(
            certify_single_wide_sphere_window_union(
                &a,
                wide_a_range,
                &wide_b,
                wide_b_range,
                true,
                Tolerances::default(),
                wide_allowance,
                GENERAL_SPHERE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_WIDE_ARC_LIMIT,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere wide-window pair limit exhausted"
            }
        );
        assert_eq!(
            certify_single_wide_sphere_window_union(
                &a,
                wide_a_range,
                &wide_b,
                wide_b_range,
                true,
                Tolerances::default(),
                wide_allowance,
                GENERAL_SPHERE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_WIDE_ARC_LIMIT - 1,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere wide-window arc limit exhausted"
            }
        );

        let non_path_angle = 0.9054345637982375;
        let non_path_b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(non_path_angle), 0.0, math::cos(non_path_angle)),
                Vec3::new(math::cos(non_path_angle), 0.0, -math::sin(non_path_angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        let double_wide_a_range = [
            ParamRange::new(-0.5929589703265958, 4.542973834373202),
            ParamRange::new(-0.45247797577056326, 0.9706234964995796),
        ];
        let double_wide_b_range = [
            ParamRange::new(-0.4436018548841436, 3.5517236306126825),
            ParamRange::new(-0.27691435273708875, 0.8493651816976383),
        ];
        let double_wide_allowance =
            arbitrary_sphere_octant_parameter_allowance(double_wide_a_range, double_wide_b_range)
                .unwrap();
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT, 9);
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_POSITIVE_CELL_LIMIT, 9);
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT, 252);
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT, 1_008);
        let double_wide = certify_double_wide_sphere_window_union(
            &a,
            double_wide_a_range,
            &non_path_b,
            double_wide_b_range,
            Tolerances::default(),
            double_wide_allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(double_wide.is_complete());
        assert_eq!(double_wide.regions.len(), 1);
        assert_eq!(double_wide.regions[0].boundary.len(), 14);
        let transposed_allowance =
            arbitrary_sphere_octant_parameter_allowance(double_wide_b_range, double_wide_a_range)
                .unwrap();
        let transposed_double_wide = certify_double_wide_sphere_window_union(
            &non_path_b,
            double_wide_b_range,
            &a,
            double_wide_a_range,
            Tolerances::default(),
            transposed_allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(transposed_double_wide.is_complete());
        assert_eq!(transposed_double_wide.regions.len(), 1);
        assert_eq!(transposed_double_wide.regions[0].boundary.len(), 14);
        assert_eq!(
            certify_double_wide_sphere_window_union(
                &a,
                double_wide_a_range,
                &non_path_b,
                double_wide_b_range,
                Tolerances::default(),
                double_wide_allowance,
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere both-wide union piece limit exhausted"
            }
        );
        assert_eq!(
            certify_double_wide_sphere_window_union(
                &a,
                double_wide_a_range,
                &non_path_b,
                double_wide_b_range,
                Tolerances::default(),
                double_wide_allowance,
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere both-wide union pair limit exhausted"
            }
        );
        assert_eq!(
            certify_double_wide_sphere_window_union(
                &a,
                double_wide_a_range,
                &non_path_b,
                double_wide_b_range,
                Tolerances::default(),
                double_wide_allowance,
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere both-wide union arc limit exhausted"
            }
        );

        let recursively_wide_a_range = [
            ParamRange::new(-0.6, 3.0 * core::f64::consts::PI - 0.6),
            ParamRange::new(-0.8, 0.8),
        ];
        let recursively_wide_allowance =
            arbitrary_sphere_octant_parameter_allowance(recursively_wide_a_range, wide_b_range)
                .unwrap();
        assert_eq!(
            certify_single_wide_sphere_window_union(
                &a,
                recursively_wide_a_range,
                &wide_b,
                wide_b_range,
                true,
                Tolerances::default(),
                recursively_wide_allowance,
                GENERAL_SPHERE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_WIDE_ARC_LIMIT,
            )
            .unwrap_err(),
            Error::InvalidGeometry {
                reason: "general coincident sphere wide-window union requires three sub-pi decomposition cells"
            }
        );
    }

    #[test]
    fn exact_polar_window_cells_and_limits_are_exact() {
        let (a, b, a_range, b_range) = exact_polar_window_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_single_polar_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 3);

        let [pole_clear_piece, polar_cap] = decompose_general_sphere_polar_window(a_range).unwrap();
        let pole_clear_allowance =
            arbitrary_sphere_octant_parameter_allowance(pole_clear_piece, b_range).unwrap();
        let pole_clear = certify_general_sphere_window_arrangement(
            &a,
            pole_clear_piece,
            &b,
            b_range,
            Tolerances::default(),
            GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
            GENERAL_SPHERE_WINDOW_ARC_LIMIT,
            pole_clear_allowance,
        )
        .unwrap();
        assert!(pole_clear.is_proven_empty());
        let cap_allowance =
            arbitrary_sphere_octant_parameter_allowance(polar_cap, b_range).unwrap();
        let cap = certify_general_sphere_window_arrangement(
            &a,
            polar_cap,
            &b,
            b_range,
            Tolerances::default(),
            GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
            cap_allowance,
        )
        .unwrap();
        assert!(cap.is_complete());
        assert_eq!(cap.regions.len(), 1);
        for (piece, piece_allowance, pair_limit, arc_limit, reason) in [
            (
                pole_clear_piece,
                pole_clear_allowance,
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT - 1,
                GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                "general coincident sphere window proof pair limit exhausted",
            ),
            (
                polar_cap,
                cap_allowance,
                GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                "general coincident sphere window proof pair limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_general_sphere_window_arrangement(
                    &a,
                    piece,
                    &b,
                    b_range,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
        let seam = pole_clear_piece[1].hi;
        assert!(
            hit.regions[0]
                .boundary
                .iter()
                .all(|vertex| vertex.uv_a[1].to_bits() != seam.to_bits())
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT,
                "general coincident sphere polar-window union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT,
                "general coincident sphere polar-window union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-window union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_single_polar_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_cells_and_limits_are_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 3);

        let polar_pieces = decompose_general_sphere_polar_window(a_range).unwrap();
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut occupied = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).unwrap();
                let child = certify_general_sphere_window_arrangement(
                    &a,
                    polar_piece,
                    &b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    continue;
                }
                assert!(child.is_complete());
                assert_eq!(child.regions.len(), 1);
                occupied.push([polar_index, wide_index]);
            }
        }
        assert_eq!(occupied, [[1, 1]]);
        let latitude_seam = polar_pieces[0][1].hi;
        let longitude_seams = [wide_pieces[0][0].hi, wide_pieces[1][0].hi];
        assert!(hit.regions[0].boundary.iter().all(|vertex| {
            vertex.uv_a[1].to_bits() != latitude_seam.to_bits()
                && longitude_seams
                    .iter()
                    .all(|seam| vertex.uv_b[0].to_bits() != seam.to_bits())
        }));

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_adjacent_seam_and_limits_are_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_adjacent_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 5);

        let polar_pieces = decompose_general_sphere_polar_window(a_range).unwrap();
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut occupied = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).unwrap();
                let mut child = certify_general_sphere_window_arrangement(
                    &a,
                    polar_piece,
                    &b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    continue;
                }
                assert!(child.is_complete());
                assert!(child.points.is_empty());
                assert!(child.curves.is_empty());
                assert_eq!(child.regions.len(), 1);
                occupied.push((
                    [polar_index, wide_index],
                    child
                        .regions
                        .pop()
                        .expect("one occupied child region was required"),
                ));
            }
        }
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[1, 0], [1, 1]]
        );
        let seam = wide_pieces[1][0].lo;
        let merged =
            merge_exact_adjacent_sphere_regions(&occupied[0].1, &occupied[1].1, false, seam)
                .expect("the adjacent cap-row children require one exact shared edge");
        assert_eq!(merged.boundary.len(), 5);

        let mut mismatched = occupied[1].1.clone();
        let seam_edge = exact_sphere_region_seam_edge(&mismatched, false, seam)
            .expect("the second child owns the shared longitude edge");
        let endpoint = seam_edge[0];
        mismatched.boundary[endpoint].uv_a[0] =
            f64::from_bits(mismatched.boundary[endpoint].uv_a[0].to_bits() + 1);
        assert!(
            merge_exact_adjacent_sphere_regions(&occupied[0].1, &mismatched, false, seam).is_none()
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_lower_adjacent_seam_is_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_lower_adjacent_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let polar_pieces = decompose_general_sphere_polar_window(a_range).unwrap();
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut occupied = Vec::new();
        let mut empty = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).unwrap();
                let mut child = certify_general_sphere_window_arrangement(
                    &a,
                    polar_piece,
                    &b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    empty.push([polar_index, wide_index]);
                    continue;
                }
                assert!(child.is_complete());
                assert!(child.points.is_empty());
                assert!(child.curves.is_empty());
                assert_eq!(child.regions.len(), 1);
                occupied.push((
                    [polar_index, wide_index],
                    child
                        .regions
                        .pop()
                        .expect("one occupied lower-row child region was required"),
                ));
            }
        }
        assert_eq!(empty, [[0, 2], [1, 0], [1, 1], [1, 2]]);
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[0, 0], [0, 1]]
        );

        let seam = wide_pieces[1][0].lo;
        let merged =
            merge_exact_adjacent_sphere_regions(&occupied[0].1, &occupied[1].1, false, seam)
                .expect("the adjacent lower-row children require one exact shared edge");
        assert_eq!(merged.boundary.len(), 6);
        assert!(exact_sphere_region_seam_edge(&merged, false, seam).is_none());
        let latitude_seam = polar_pieces[0][1].hi;
        let unused_longitude_seam = wide_pieces[2][0].lo;
        assert!(merged.boundary.iter().all(|vertex| {
            vertex.uv_a[1].to_bits() != latitude_seam.to_bits()
                && vertex.uv_b[0].to_bits() != unused_longitude_seam.to_bits()
        }));

        let mut mismatched = occupied[1].1.clone();
        let seam_edge = exact_sphere_region_seam_edge(&mismatched, false, seam)
            .expect("the second lower-row child owns the shared longitude edge");
        let endpoint = seam_edge[0];
        mismatched.boundary[endpoint].uv_a[0] =
            f64::from_bits(mismatched.boundary[endpoint].uv_a[0].to_bits() + 1);
        assert!(
            merge_exact_adjacent_sphere_regions(&occupied[0].1, &mismatched, false, seam).is_none()
        );

        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert!(hit.points.is_empty());
        assert!(hit.curves.is_empty());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 6);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_vertical_adjacent_seam_is_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_vertical_adjacent_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let polar_pieces = decompose_general_sphere_polar_window(a_range).unwrap();
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut occupied = Vec::new();
        let mut empty = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).unwrap();
                let mut child = certify_general_sphere_window_arrangement(
                    &a,
                    polar_piece,
                    &b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    empty.push([polar_index, wide_index]);
                    continue;
                }
                assert!(child.is_complete());
                assert!(child.points.is_empty());
                assert!(child.curves.is_empty());
                assert_eq!(child.regions.len(), 1);
                occupied.push((
                    [polar_index, wide_index],
                    child
                        .regions
                        .pop()
                        .expect("one occupied same-column child region was required"),
                ));
            }
        }
        assert_eq!(empty, [[0, 0], [0, 1], [1, 0], [1, 1]]);
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[0, 2], [1, 2]]
        );

        let latitude_seam = polar_pieces[0][1].hi;
        let merged = merge_exact_adjacent_sphere_regions_on_parameter(
            &occupied[0].1,
            &occupied[1].1,
            true,
            1,
            latitude_seam,
        )
        .expect("the vertically adjacent children require one exact shared latitude edge");
        assert_eq!(merged.boundary.len(), 5);
        assert!(exact_sphere_region_parameter_seam_edge(&merged, true, 1, latitude_seam).is_none());
        assert!(merged.boundary.iter().all(|vertex| {
            wide_pieces
                .iter()
                .skip(1)
                .all(|wide_piece| vertex.uv_b[0].to_bits() != wide_piece[0].lo.to_bits())
        }));

        let mut mismatched = occupied[1].1.clone();
        let seam_edge =
            exact_sphere_region_parameter_seam_edge(&mismatched, true, 1, latitude_seam)
                .expect("the cap child owns the shared latitude edge");
        let endpoint = seam_edge[0];
        mismatched.boundary[endpoint].uv_b[0] =
            f64::from_bits(mismatched.boundary[endpoint].uv_b[0].to_bits() + 1);
        assert!(
            merge_exact_adjacent_sphere_regions_on_parameter(
                &occupied[0].1,
                &mismatched,
                true,
                1,
                latitude_seam,
            )
            .is_none()
        );

        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert!(hit.points.is_empty());
        assert!(hit.curves.is_empty());
        assert_eq!(hit.regions.len(), 1);
        let canonical_start = merged
            .boundary
            .iter()
            .position(|vertex| *vertex == hit.regions[0].boundary[0])
            .expect("canonicalization must retain every merged boundary vertex");
        let cyclic_merged = (0..merged.boundary.len())
            .map(|offset| merged.boundary[(canonical_start + offset) % merged.boundary.len()])
            .collect::<Vec<_>>();
        assert_eq!(hit.regions[0].boundary, cyclic_merged);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_mixed_axis_paths_are_exact() {
        let fixtures = [
            (
                exact_polar_by_wide_cap_right_l_fixture(),
                [[0, 2], [1, 1], [1, 2]],
                [[0, 0], [0, 1], [1, 0]],
            ),
            (
                exact_polar_by_wide_lower_middle_l_fixture(),
                [[0, 1], [0, 2], [1, 1]],
                [[0, 0], [1, 0], [1, 2]],
            ),
        ];
        for (fixture_index, ((a, b, a_range, b_range), expected, expected_empty)) in
            fixtures.into_iter().enumerate()
        {
            let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
            let (polar_pieces, wide_pieces, occupied, empty) =
                exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
            assert_eq!(
                occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
                expected
            );
            assert_eq!(empty, expected_empty);

            let (merged, used_longitude_seam) = merge_exact_polar_wide_sphere_region_path(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .expect("the mixed-row L must own one exact seam on each axis");
            let latitude_seam = polar_pieces[0][1].hi;
            let longitude_seams = [wide_pieces[1][0].lo, wide_pieces[2][0].lo];
            assert!(
                exact_sphere_region_parameter_seam_edge(&merged, true, 1, latitude_seam,).is_none()
            );
            assert!(longitude_seams.iter().all(|seam| {
                exact_sphere_region_parameter_seam_edge(&merged, false, 0, *seam).is_none()
            }));
            let unused_longitude_seam = longitude_seams
                .into_iter()
                .find(|seam| seam.to_bits() != used_longitude_seam.to_bits())
                .expect("one longitude seam is unused by an L path");
            assert!(
                merged
                    .boundary
                    .iter()
                    .all(|vertex| { vertex.uv_b[0].to_bits() != unused_longitude_seam.to_bits() })
            );

            let hit = certify_polar_by_wide_sphere_window_union(
                &a,
                a_range,
                &b,
                b_range,
                true,
                Tolerances::default(),
                allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
            )
            .unwrap();
            assert!(hit.is_complete());
            assert!(hit.points.is_empty());
            assert!(hit.curves.is_empty());
            assert_eq!(hit.regions.len(), 1);
            assert_eq!(hit.regions[0].boundary.len(), merged.boundary.len());
            assert!(matches!(
                hit.regions[0].correspondence,
                SurfaceRegionCorrespondence::GeneralSphereWindow(_)
            ));
            assert!(hit.regions[0].boundary.iter().all(|vertex| {
                a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
            }));

            if fixture_index == 0 {
                let horizontal = (0..occupied.len())
                    .flat_map(|first| {
                        (first + 1..occupied.len()).map(move |second| (first, second))
                    })
                    .find(|(first, second)| {
                        occupied[*first].0[0] == occupied[*second].0[0]
                            && occupied[*first].0[1].abs_diff(occupied[*second].0[1]) == 1
                    })
                    .expect("one horizontal L edge was required");
                let vertical = (0..occupied.len())
                    .flat_map(|first| {
                        (first + 1..occupied.len()).map(move |second| (first, second))
                    })
                    .find(|(first, second)| {
                        occupied[*first].0[0].abs_diff(occupied[*second].0[0]) == 1
                            && occupied[*first].0[1] == occupied[*second].0[1]
                    })
                    .expect("one vertical L edge was required");

                let mut longitude_mismatch = occupied.clone();
                let longitude_owner = horizontal.1;
                let longitude_edge = exact_sphere_region_parameter_seam_edge(
                    &longitude_mismatch[longitude_owner].1,
                    false,
                    0,
                    used_longitude_seam,
                )
                .expect("the longitude sibling owns its exact seam edge");
                let endpoint = longitude_edge[0];
                longitude_mismatch[longitude_owner].1.boundary[endpoint].uv_b[0] = f64::from_bits(
                    longitude_mismatch[longitude_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1,
                );
                assert!(
                    merge_exact_polar_wide_sphere_region_path(
                        &longitude_mismatch,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut latitude_mismatch = occupied.clone();
                let latitude_owner = vertical.1;
                let latitude_edge = exact_sphere_region_parameter_seam_edge(
                    &latitude_mismatch[latitude_owner].1,
                    true,
                    1,
                    latitude_seam,
                )
                .expect("the latitude sibling owns its exact seam edge");
                let endpoint = latitude_edge[0];
                latitude_mismatch[latitude_owner].1.boundary[endpoint].uv_a[1] = f64::from_bits(
                    latitude_mismatch[latitude_owner].1.boundary[endpoint].uv_a[1].to_bits() + 1,
                );
                assert!(
                    merge_exact_polar_wide_sphere_region_path(
                        &latitude_mismatch,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut opposite = occupied.clone();
                opposite[0].0 = [0, 0];
                opposite[1].0 = [0, 2];
                opposite[2].0 = [1, 1];
                assert!(
                    merge_exact_polar_wide_sphere_region_path(
                        &opposite,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                for (piece_limit, pair_limit, arc_limit, reason) in [
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union piece limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union pair limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                        "general coincident sphere polar-by-wide union arc limit exhausted",
                    ),
                ] {
                    assert_eq!(
                        certify_polar_by_wide_sphere_window_union(
                            &a,
                            a_range,
                            &b,
                            b_range,
                            true,
                            Tolerances::default(),
                            allowance,
                            piece_limit,
                            pair_limit,
                            arc_limit,
                        )
                        .unwrap_err(),
                        Error::InvalidGeometry { reason }
                    );
                }
            }
        }
    }

    #[test]
    fn exact_polar_by_wide_four_cell_paths_are_exact() {
        let fixtures = [
            (
                exact_polar_by_wide_cap_row_right_four_path_fixture(),
                [[0, 2], [1, 0], [1, 1], [1, 2]],
                [[0, 0], [0, 1]],
            ),
            (
                exact_polar_by_wide_zigzag_four_path_fixture(),
                [[0, 1], [0, 2], [1, 0], [1, 1]],
                [[0, 0], [1, 2]],
            ),
        ];
        for (fixture_index, ((a, b, a_range, b_range), expected, expected_empty)) in
            fixtures.into_iter().enumerate()
        {
            let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
            let (polar_pieces, wide_pieces, occupied, empty) =
                exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
            assert_eq!(
                occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
                expected
            );
            assert_eq!(empty, expected_empty);

            let merged = merge_exact_polar_wide_four_sphere_region_path(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .expect("the four-cell path must own three exact shared seams");
            assert!(
                merge_exact_polar_wide_simultaneous_sphere_region_union(
                    &occupied,
                    true,
                    &polar_pieces,
                    &wide_pieces,
                )
                .is_none(),
                "a three-adjacency path must not enter the simultaneous cycle arm"
            );
            let artificial_seams = [
                (true, 1, polar_pieces[0][1].hi),
                (false, 0, wide_pieces[1][0].lo),
                (false, 0, wide_pieces[2][0].lo),
            ];
            assert!(artificial_seams.iter().all(|(on_first, parameter, seam)| {
                !sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
            }));

            let hit = certify_polar_by_wide_sphere_window_union(
                &a,
                a_range,
                &b,
                b_range,
                true,
                Tolerances::default(),
                allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
            )
            .unwrap();
            assert!(hit.is_complete());
            assert!(hit.points.is_empty());
            assert!(hit.curves.is_empty());
            assert_eq!(hit.regions.len(), 1);
            assert_eq!(hit.regions[0].boundary.len(), merged.boundary.len());
            let SurfaceRegionCorrespondence::GeneralSphereWindow(map) =
                hit.regions[0].correspondence
            else {
                unreachable!()
            };
            assert_eq!(map.first_range(), a_range);
            assert_eq!(map.second_range(), b_range);
            assert!(hit.regions[0].boundary.iter().all(|vertex| {
                vertex.residual <= hit.regions[0].max_residual
                    && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
            }));

            if fixture_index == 0 {
                let longitude_seam = wide_pieces[2][0].lo;
                let longitude_owner = occupied
                    .iter()
                    .position(|(cell, _)| *cell == [1, 2])
                    .expect("the right cap child was required");
                let longitude_edge = exact_sphere_region_parameter_seam_edge(
                    &occupied[longitude_owner].1,
                    false,
                    0,
                    longitude_seam,
                )
                .expect("the right cap child owns the exact longitude seam");

                let mut one_ulp = occupied.clone();
                let endpoint = longitude_edge[0];
                one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0] = f64::from_bits(
                    one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1,
                );
                assert!(
                    merge_exact_polar_wide_four_sphere_region_path(
                        &one_ulp,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut ambiguous = occupied.clone();
                let duplicate = ambiguous[longitude_owner].1.boundary[longitude_edge[0]];
                ambiguous[longitude_owner]
                    .1
                    .boundary
                    .insert(longitude_edge[0] + 1, duplicate);
                assert!(
                    merge_exact_polar_wide_four_sphere_region_path(
                        &ambiguous,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut cycle = occupied.clone();
                for (entry, cell) in cycle.iter_mut().zip([[0, 0], [0, 1], [1, 0], [1, 1]]) {
                    entry.0 = cell;
                }
                assert!(
                    merge_exact_polar_wide_four_sphere_region_path(
                        &cycle,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                for (piece_limit, pair_limit, arc_limit, reason) in [
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union piece limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union pair limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                        "general coincident sphere polar-by-wide union arc limit exhausted",
                    ),
                ] {
                    assert_eq!(
                        certify_polar_by_wide_sphere_window_union(
                            &a,
                            a_range,
                            &b,
                            b_range,
                            true,
                            Tolerances::default(),
                            allowance,
                            piece_limit,
                            pair_limit,
                            arc_limit,
                        )
                        .unwrap_err(),
                        Error::InvalidGeometry { reason }
                    );
                }
            }
        }
    }

    #[test]
    fn exact_polar_by_wide_four_cell_cycles_are_exact() {
        let fixtures = [
            (
                exact_polar_by_wide_left_four_cell_cycle_fixture(),
                [[0, 0], [0, 1], [1, 0], [1, 1]],
                [[0, 2], [1, 2]],
            ),
            (
                exact_polar_by_wide_right_four_cell_cycle_fixture(),
                [[0, 1], [0, 2], [1, 1], [1, 2]],
                [[0, 0], [1, 0]],
            ),
        ];
        for (fixture_index, ((a, b, a_range, b_range), expected, expected_empty)) in
            fixtures.into_iter().enumerate()
        {
            let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
            let parent_residual =
                arbitrary_sphere_octant_residual_bound(&a, &b, allowance).unwrap();
            let (polar_pieces, wide_pieces, occupied, empty) =
                exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
            assert_eq!(
                occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
                expected
            );
            assert_eq!(empty, expected_empty);
            assert!(
                merge_exact_polar_wide_four_sphere_region_path(
                    &occupied,
                    true,
                    &polar_pieces,
                    &wide_pieces,
                )
                .is_none(),
                "a four-adjacency cycle must not enter the path arm"
            );

            let merged = merge_exact_polar_wide_simultaneous_sphere_region_union(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .expect("the four-cell cycle must simultaneously own four exact shared seams");
            let child_residual = occupied
                .iter()
                .map(|(_, region)| region.max_residual)
                .fold(0.0, f64::max);
            let artificial_seams = [
                (true, 1, polar_pieces[0][1].hi),
                (false, 0, wide_pieces[1][0].lo),
                (false, 0, wide_pieces[2][0].lo),
            ];
            assert!(artificial_seams.iter().all(|(on_first, parameter, seam)| {
                !sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
            }));

            let hit = certify_polar_by_wide_sphere_window_union(
                &a,
                a_range,
                &b,
                b_range,
                true,
                Tolerances::default(),
                allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
            )
            .unwrap();
            assert!(hit.is_complete());
            assert!(hit.points.is_empty());
            assert!(hit.curves.is_empty());
            assert_eq!(hit.regions.len(), 1);
            assert_eq!(hit.regions[0].boundary.len(), merged.boundary.len());
            assert_eq!(
                hit.regions[0].max_residual,
                child_residual.max(parent_residual)
            );
            let SurfaceRegionCorrespondence::GeneralSphereWindow(map) =
                hit.regions[0].correspondence
            else {
                unreachable!()
            };
            assert_eq!(map.first_range(), a_range);
            assert_eq!(map.second_range(), b_range);
            assert!(hit.regions[0].boundary.iter().all(|vertex| {
                vertex.residual <= hit.regions[0].max_residual
                    && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
            }));

            if fixture_index == 0 {
                let longitude_seam = wide_pieces[1][0].lo;
                let longitude_owner = occupied
                    .iter()
                    .position(|(cell, _)| *cell == [0, 1])
                    .expect("the middle lower child was required");
                let longitude_edge = exact_sphere_region_parameter_seam_edge(
                    &occupied[longitude_owner].1,
                    false,
                    0,
                    longitude_seam,
                )
                .expect("the middle lower child owns the exact longitude seam");

                let mut one_ulp = occupied.clone();
                let endpoint = longitude_edge[0];
                one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0] = f64::from_bits(
                    one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1,
                );
                assert!(
                    merge_exact_polar_wide_simultaneous_sphere_region_union(
                        &one_ulp,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut ambiguous = occupied.clone();
                let duplicate = ambiguous[longitude_owner].1.boundary[longitude_edge[0]];
                ambiguous[longitude_owner]
                    .1
                    .boundary
                    .insert(longitude_edge[0] + 1, duplicate);
                assert!(
                    merge_exact_polar_wide_simultaneous_sphere_region_union(
                        &ambiguous,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                for (piece_limit, pair_limit, arc_limit, reason) in [
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union piece limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union pair limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                        "general coincident sphere polar-by-wide union arc limit exhausted",
                    ),
                ] {
                    assert_eq!(
                        certify_polar_by_wide_sphere_window_union(
                            &a,
                            a_range,
                            &b,
                            b_range,
                            true,
                            Tolerances::default(),
                            allowance,
                            piece_limit,
                            pair_limit,
                            arc_limit,
                        )
                        .unwrap_err(),
                        Error::InvalidGeometry { reason }
                    );
                }
            }
        }
    }

    #[test]
    fn exact_polar_by_wide_four_cell_trees_are_exact() {
        let fixtures = [
            (
                exact_polar_by_wide_lower_stem_four_cell_t_fixture(),
                [[0, 0], [0, 1], [0, 2], [1, 1]],
                [[1, 0], [1, 2]],
            ),
            (
                exact_polar_by_wide_upper_stem_four_cell_t_fixture(),
                [[0, 1], [1, 0], [1, 1], [1, 2]],
                [[0, 0], [0, 2]],
            ),
        ];
        for (fixture_index, ((a, b, a_range, b_range), expected, expected_empty)) in
            fixtures.into_iter().enumerate()
        {
            let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
            let parent_residual =
                arbitrary_sphere_octant_residual_bound(&a, &b, allowance).unwrap();
            let (polar_pieces, wide_pieces, occupied, empty) =
                exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
            assert_eq!(
                occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
                expected
            );
            assert_eq!(empty, expected_empty);
            assert!(
                merge_exact_polar_wide_four_sphere_region_path(
                    &occupied,
                    true,
                    &polar_pieces,
                    &wide_pieces,
                )
                .is_none(),
                "a degree-three T must not enter the four-cell path arm"
            );

            let merged = merge_exact_polar_wide_simultaneous_sphere_region_union(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .expect("the four-cell T must simultaneously own three exact shared seams");
            let child_residual = occupied
                .iter()
                .map(|(_, region)| region.max_residual)
                .fold(0.0, f64::max);
            let artificial_seams = [
                (true, 1, polar_pieces[0][1].hi),
                (false, 0, wide_pieces[1][0].lo),
                (false, 0, wide_pieces[2][0].lo),
            ];
            assert!(artificial_seams.iter().all(|(on_first, parameter, seam)| {
                !sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
            }));

            let hit = certify_polar_by_wide_sphere_window_union(
                &a,
                a_range,
                &b,
                b_range,
                true,
                Tolerances::default(),
                allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
            )
            .unwrap();
            assert!(hit.is_complete());
            assert!(hit.points.is_empty());
            assert!(hit.curves.is_empty());
            assert_eq!(hit.regions.len(), 1);
            assert_eq!(hit.regions[0].boundary.len(), merged.boundary.len());
            assert_eq!(
                hit.regions[0].max_residual,
                child_residual.max(parent_residual)
            );
            let SurfaceRegionCorrespondence::GeneralSphereWindow(map) =
                hit.regions[0].correspondence
            else {
                unreachable!()
            };
            assert_eq!(map.first_range(), a_range);
            assert_eq!(map.second_range(), b_range);
            assert!(hit.regions[0].boundary.iter().all(|vertex| {
                vertex.residual <= hit.regions[0].max_residual
                    && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
            }));

            if fixture_index == 0 {
                let longitude_seam = wide_pieces[1][0].lo;
                let longitude_owner = occupied
                    .iter()
                    .position(|(cell, _)| *cell == [0, 1])
                    .expect("the middle lower child was required");
                let longitude_edge = exact_sphere_region_parameter_seam_edge(
                    &occupied[longitude_owner].1,
                    false,
                    0,
                    longitude_seam,
                )
                .expect("the middle lower child owns the exact longitude seam");

                let mut one_ulp = occupied.clone();
                let endpoint = longitude_edge[0];
                one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0] = f64::from_bits(
                    one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1,
                );
                assert!(
                    merge_exact_polar_wide_simultaneous_sphere_region_union(
                        &one_ulp,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut ambiguous = occupied.clone();
                let duplicate = ambiguous[longitude_owner].1.boundary[longitude_edge[0]];
                ambiguous[longitude_owner]
                    .1
                    .boundary
                    .insert(longitude_edge[0] + 1, duplicate);
                assert!(
                    merge_exact_polar_wide_simultaneous_sphere_region_union(
                        &ambiguous,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                for (piece_limit, pair_limit, arc_limit, reason) in [
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union piece limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union pair limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                        "general coincident sphere polar-by-wide union arc limit exhausted",
                    ),
                ] {
                    assert_eq!(
                        certify_polar_by_wide_sphere_window_union(
                            &a,
                            a_range,
                            &b,
                            b_range,
                            true,
                            Tolerances::default(),
                            allowance,
                            piece_limit,
                            pair_limit,
                            arc_limit,
                        )
                        .unwrap_err(),
                        Error::InvalidGeometry { reason }
                    );
                }
            }
        }
    }

    #[test]
    fn exact_polar_by_wide_disconnected_vertical_pairs_are_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_disconnected_vertical_pairs_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let parent_residual = arbitrary_sphere_octant_residual_bound(&a, &b, allowance).unwrap();
        let (polar_pieces, wide_pieces, occupied, empty) =
            exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[0, 0], [0, 2], [1, 0], [1, 2]]
        );
        assert_eq!(empty, [[0, 1], [1, 1]]);
        assert!(
            merge_exact_polar_wide_four_sphere_region_path(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .is_none()
        );
        assert!(
            merge_exact_polar_wide_simultaneous_sphere_region_union(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .is_none()
        );

        let components = merge_exact_polar_wide_disconnected_vertical_sphere_region_pairs(
            &occupied,
            true,
            &polar_pieces,
            &wide_pieces,
        )
        .expect("the outer columns must form two exact vertical-pair components");
        assert_eq!(components.len(), 2);
        let latitude_seam = polar_pieces[0][1].hi;
        let longitude_seams = [wide_pieces[1][0].lo, wide_pieces[2][0].lo];
        assert!(components.iter().all(|component| {
            !sphere_region_has_parameter_seam_edge(component, true, 1, latitude_seam)
                && component.boundary.iter().all(|vertex| {
                    longitude_seams
                        .iter()
                        .all(|seam| vertex.uv_b[0].to_bits() != seam.to_bits())
                })
        }));
        for (component, column) in components.iter().zip([0, 2]) {
            let expected_residual = occupied
                .iter()
                .filter(|(cell, _)| cell[1] == column)
                .map(|(_, region)| region.max_residual)
                .fold(0.0, f64::max);
            assert_eq!(component.max_residual, expected_residual);
        }

        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert!(hit.points.is_empty());
        assert!(hit.curves.is_empty());
        assert_eq!(hit.regions.len(), 2);
        let mut expected_residuals = components
            .iter()
            .map(|component| component.max_residual.max(parent_residual))
            .collect::<Vec<_>>();
        expected_residuals.sort_by(f64::total_cmp);
        let mut actual_residuals = hit
            .regions
            .iter()
            .map(|region| region.max_residual)
            .collect::<Vec<_>>();
        actual_residuals.sort_by(f64::total_cmp);
        assert_eq!(actual_residuals, expected_residuals);
        for region in &hit.regions {
            let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = region.correspondence
            else {
                unreachable!()
            };
            assert_eq!(map.first_range(), a_range);
            assert_eq!(map.second_range(), b_range);
            assert!(region.boundary.iter().all(|vertex| {
                vertex.residual <= region.max_residual
                    && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= region.max_residual
            }));
        }

        let seam_owner = occupied
            .iter()
            .position(|(cell, _)| *cell == [1, 0])
            .expect("the upper left child was required");
        let seam_edge = exact_sphere_region_parameter_seam_edge(
            &occupied[seam_owner].1,
            true,
            1,
            latitude_seam,
        )
        .expect("the upper left child owns the exact latitude seam");
        let mut one_ulp = occupied.clone();
        let endpoint = seam_edge[0];
        one_ulp[seam_owner].1.boundary[endpoint].uv_a[1] =
            f64::from_bits(one_ulp[seam_owner].1.boundary[endpoint].uv_a[1].to_bits() + 1);
        assert!(
            merge_exact_polar_wide_disconnected_vertical_sphere_region_pairs(
                &one_ulp,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .is_none()
        );

        let mut ambiguous = occupied.clone();
        let duplicate = ambiguous[seam_owner].1.boundary[seam_edge[0]];
        ambiguous[seam_owner]
            .1
            .boundary
            .insert(seam_edge[0] + 1, duplicate);
        assert!(
            merge_exact_polar_wide_disconnected_vertical_sphere_region_pairs(
                &ambiguous,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .is_none()
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_singleton_plus_three_cell_components_are_exact() {
        let fixtures = [
            (
                exact_polar_by_wide_zero_zero_singleton_fixture(),
                [[0, 0], [0, 2], [1, 1], [1, 2]],
                [[0, 1], [1, 0]],
                [0, 0],
            ),
            (
                exact_polar_by_wide_one_two_singleton_fixture(),
                [[0, 0], [0, 1], [1, 0], [1, 2]],
                [[0, 2], [1, 1]],
                [1, 2],
            ),
            (
                exact_polar_by_wide_one_zero_singleton_fixture(),
                [[0, 1], [0, 2], [1, 0], [1, 2]],
                [[0, 0], [1, 1]],
                [1, 0],
            ),
            (
                exact_polar_by_wide_zero_two_singleton_fixture(),
                [[0, 0], [0, 2], [1, 0], [1, 1]],
                [[0, 1], [1, 2]],
                [0, 2],
            ),
        ];
        for (fixture_index, ((a, b, a_range, b_range), expected, expected_empty, singleton_cell)) in
            fixtures.into_iter().enumerate()
        {
            let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
            let parent_residual =
                arbitrary_sphere_octant_residual_bound(&a, &b, allowance).unwrap();
            let (polar_pieces, wide_pieces, occupied, empty) =
                exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
            assert_eq!(
                occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
                expected
            );
            assert_eq!(empty, expected_empty);
            assert!(
                merge_exact_polar_wide_disconnected_vertical_sphere_region_pairs(
                    &occupied,
                    true,
                    &polar_pieces,
                    &wide_pieces,
                )
                .is_none()
            );
            assert!(
                merge_exact_polar_wide_four_sphere_region_path(
                    &occupied,
                    true,
                    &polar_pieces,
                    &wide_pieces,
                )
                .is_none()
            );
            assert!(
                merge_exact_polar_wide_simultaneous_sphere_region_union(
                    &occupied,
                    true,
                    &polar_pieces,
                    &wide_pieces,
                )
                .is_none()
            );

            let components = merge_exact_polar_wide_singleton_and_three_cell_sphere_regions(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .expect("the isolated corner and exact three-cell path must remain two components");
            assert_eq!(components.len(), 2);
            let singleton_residual = occupied
                .iter()
                .find(|(cell, _)| *cell == singleton_cell)
                .expect("the singleton cell was required")
                .1
                .max_residual;
            let path_residual = occupied
                .iter()
                .filter(|(cell, _)| *cell != singleton_cell)
                .map(|(_, region)| region.max_residual)
                .fold(0.0, f64::max);
            assert_eq!(components[0].max_residual, singleton_residual);
            assert_eq!(components[1].max_residual, path_residual);
            let artificial_seams = [
                (true, 1, polar_pieces[0][1].hi),
                (false, 0, wide_pieces[1][0].lo),
                (false, 0, wide_pieces[2][0].lo),
            ];
            assert!(components.iter().all(|component| {
                artificial_seams.iter().all(|(on_first, parameter, seam)| {
                    !sphere_region_has_parameter_seam_edge(component, *on_first, *parameter, *seam)
                })
            }));
            assert!(components[0].boundary.iter().all(|first| {
                components[1]
                    .boundary
                    .iter()
                    .all(|second| !sphere_region_vertices_are_bit_exact(*first, *second))
            }));

            let hit = certify_polar_by_wide_sphere_window_union(
                &a,
                a_range,
                &b,
                b_range,
                true,
                Tolerances::default(),
                allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
            )
            .unwrap();
            assert!(hit.is_complete());
            assert!(hit.points.is_empty());
            assert!(hit.curves.is_empty());
            assert_eq!(hit.regions.len(), 2);
            let mut expected_residuals = [
                singleton_residual.max(parent_residual),
                path_residual.max(parent_residual),
            ];
            expected_residuals.sort_by(f64::total_cmp);
            let mut actual_residuals = hit
                .regions
                .iter()
                .map(|region| region.max_residual)
                .collect::<Vec<_>>();
            actual_residuals.sort_by(f64::total_cmp);
            assert_eq!(actual_residuals, expected_residuals);
            for region in &hit.regions {
                let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = region.correspondence
                else {
                    unreachable!()
                };
                assert_eq!(map.first_range(), a_range);
                assert_eq!(map.second_range(), b_range);
                assert!(region.boundary.iter().all(|vertex| {
                    vertex.residual <= region.max_residual
                        && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= region.max_residual
                }));
            }

            if fixture_index == 0 {
                let longitude_seam = wide_pieces[2][0].lo;
                let seam_owner = occupied
                    .iter()
                    .position(|(cell, _)| *cell == [1, 2])
                    .expect("the right upper path child was required");
                let seam_edge = exact_sphere_region_parameter_seam_edge(
                    &occupied[seam_owner].1,
                    false,
                    0,
                    longitude_seam,
                )
                .expect("the right upper path child owns the exact longitude seam");
                let mut one_ulp = occupied.clone();
                let endpoint = seam_edge[0];
                one_ulp[seam_owner].1.boundary[endpoint].uv_b[0] =
                    f64::from_bits(one_ulp[seam_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1);
                assert!(
                    merge_exact_polar_wide_singleton_and_three_cell_sphere_regions(
                        &one_ulp,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut ambiguous = occupied.clone();
                let duplicate = ambiguous[seam_owner].1.boundary[seam_edge[0]];
                ambiguous[seam_owner]
                    .1
                    .boundary
                    .insert(seam_edge[0] + 1, duplicate);
                assert!(
                    merge_exact_polar_wide_singleton_and_three_cell_sphere_regions(
                        &ambiguous,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                for (piece_limit, pair_limit, arc_limit, reason) in [
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union piece limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union pair limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                        "general coincident sphere polar-by-wide union arc limit exhausted",
                    ),
                ] {
                    assert_eq!(
                        certify_polar_by_wide_sphere_window_union(
                            &a,
                            a_range,
                            &b,
                            b_range,
                            true,
                            Tolerances::default(),
                            allowance,
                            piece_limit,
                            pair_limit,
                            arc_limit,
                        )
                        .unwrap_err(),
                        Error::InvalidGeometry { reason }
                    );
                }
            }
        }
    }

    #[test]
    fn exact_polar_by_wide_five_cell_unions_are_exact() {
        let fixtures = [
            (
                exact_polar_by_wide_corner_empty_five_cell_fixture(),
                [[0, 1], [0, 2], [1, 0], [1, 1], [1, 2]],
                [0, 0],
            ),
            (
                exact_polar_by_wide_edge_empty_five_cell_fixture(),
                [[0, 0], [0, 1], [0, 2], [1, 0], [1, 2]],
                [1, 1],
            ),
        ];
        for (fixture_index, ((a, b, a_range, b_range), expected, expected_empty)) in
            fixtures.into_iter().enumerate()
        {
            let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
            let (polar_pieces, wide_pieces, occupied, empty) =
                exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
            assert_eq!(
                occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
                expected
            );
            assert_eq!(empty, [expected_empty]);

            let merged = merge_exact_polar_wide_simultaneous_sphere_region_union(
                &occupied,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .expect("five occupied cells must form one exact simultaneous outer cycle");
            let artificial_seams = [
                (true, 1, polar_pieces[0][1].hi),
                (false, 0, wide_pieces[1][0].lo),
                (false, 0, wide_pieces[2][0].lo),
            ];
            assert!(artificial_seams.iter().all(|(on_first, parameter, seam)| {
                !sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
            }));

            let hit = certify_polar_by_wide_sphere_window_union(
                &a,
                a_range,
                &b,
                b_range,
                true,
                Tolerances::default(),
                allowance,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
            )
            .unwrap();
            assert!(hit.is_complete());
            assert!(hit.points.is_empty());
            assert!(hit.curves.is_empty());
            assert_eq!(hit.regions.len(), 1);
            assert_eq!(hit.regions[0].boundary.len(), merged.boundary.len());
            let SurfaceRegionCorrespondence::GeneralSphereWindow(map) =
                hit.regions[0].correspondence
            else {
                unreachable!()
            };
            assert_eq!(map.first_range(), a_range);
            assert_eq!(map.second_range(), b_range);
            assert!(hit.regions[0].boundary.iter().all(|vertex| {
                vertex.residual <= hit.regions[0].max_residual
                    && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
            }));

            if fixture_index == 0 {
                let longitude_seam = wide_pieces[2][0].lo;
                let longitude_owner = occupied
                    .iter()
                    .position(|(cell, _)| *cell == [0, 2])
                    .expect("the right lower child was required");
                let longitude_edge = exact_sphere_region_parameter_seam_edge(
                    &occupied[longitude_owner].1,
                    false,
                    0,
                    longitude_seam,
                )
                .expect("the right lower child owns the exact longitude seam");

                let mut one_ulp = occupied.clone();
                let endpoint = longitude_edge[0];
                one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0] = f64::from_bits(
                    one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1,
                );
                assert!(
                    merge_exact_polar_wide_simultaneous_sphere_region_union(
                        &one_ulp,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                let mut ambiguous = occupied.clone();
                let duplicate = ambiguous[longitude_owner].1.boundary[longitude_edge[0]];
                ambiguous[longitude_owner]
                    .1
                    .boundary
                    .insert(longitude_edge[0] + 1, duplicate);
                assert!(
                    merge_exact_polar_wide_simultaneous_sphere_region_union(
                        &ambiguous,
                        true,
                        &polar_pieces,
                        &wide_pieces,
                    )
                    .is_none()
                );

                for (piece_limit, pair_limit, arc_limit, reason) in [
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union piece limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                        "general coincident sphere polar-by-wide union pair limit exhausted",
                    ),
                    (
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                        GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                        "general coincident sphere polar-by-wide union arc limit exhausted",
                    ),
                ] {
                    assert_eq!(
                        certify_polar_by_wide_sphere_window_union(
                            &a,
                            a_range,
                            &b,
                            b_range,
                            true,
                            Tolerances::default(),
                            allowance,
                            piece_limit,
                            pair_limit,
                            arc_limit,
                        )
                        .unwrap_err(),
                        Error::InvalidGeometry { reason }
                    );
                }
            }
        }
    }

    #[test]
    fn exact_polar_by_wide_six_cell_union_is_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_six_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let parent_residual = arbitrary_sphere_octant_residual_bound(&a, &b, allowance).unwrap();
        let (polar_pieces, wide_pieces, occupied, empty) =
            exact_polar_by_wide_child_regions(&a, a_range, &b, b_range);
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2]]
        );
        assert!(empty.is_empty());

        let merged = merge_exact_polar_wide_simultaneous_sphere_region_union(
            &occupied,
            true,
            &polar_pieces,
            &wide_pieces,
        )
        .expect("all six occupied cells must form one exact simultaneous outer cycle");
        let child_residual = occupied
            .iter()
            .map(|(_, region)| region.max_residual)
            .fold(0.0, f64::max);
        let artificial_seams = [
            (true, 1, polar_pieces[0][1].hi),
            (false, 0, wide_pieces[1][0].lo),
            (false, 0, wide_pieces[2][0].lo),
        ];
        assert!(artificial_seams.iter().all(|(on_first, parameter, seam)| {
            !sphere_region_has_parameter_seam_edge(&merged, *on_first, *parameter, *seam)
        }));

        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert!(hit.points.is_empty());
        assert!(hit.curves.is_empty());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), merged.boundary.len());
        assert_eq!(
            hit.regions[0].max_residual,
            child_residual.max(parent_residual)
        );
        let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
        else {
            unreachable!()
        };
        assert_eq!(map.first_range(), a_range);
        assert_eq!(map.second_range(), b_range);
        assert!(hit.regions[0].boundary.iter().all(|vertex| {
            vertex.residual <= hit.regions[0].max_residual
                && a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
        }));

        let longitude_seam = wide_pieces[2][0].lo;
        let longitude_owner = occupied
            .iter()
            .position(|(cell, _)| *cell == [0, 2])
            .expect("the right lower child was required");
        let longitude_edge = exact_sphere_region_parameter_seam_edge(
            &occupied[longitude_owner].1,
            false,
            0,
            longitude_seam,
        )
        .expect("the right lower child owns the exact longitude seam");

        let mut one_ulp = occupied.clone();
        let endpoint = longitude_edge[0];
        one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0] =
            f64::from_bits(one_ulp[longitude_owner].1.boundary[endpoint].uv_b[0].to_bits() + 1);
        assert!(
            merge_exact_polar_wide_simultaneous_sphere_region_union(
                &one_ulp,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .is_none()
        );

        let mut ambiguous = occupied.clone();
        let duplicate = ambiguous[longitude_owner].1.boundary[longitude_edge[0]];
        ambiguous[longitude_owner]
            .1
            .boundary
            .insert(longitude_edge[0] + 1, duplicate);
        assert!(
            merge_exact_polar_wide_simultaneous_sphere_region_union(
                &ambiguous,
                true,
                &polar_pieces,
                &wide_pieces,
            )
            .is_none()
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_cap_row_path_and_limits_are_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_cap_row_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let polar_pieces = decompose_general_sphere_polar_window(a_range).unwrap();
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut occupied = Vec::new();
        let mut empty = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).unwrap();
                let mut child = certify_general_sphere_window_arrangement(
                    &a,
                    polar_piece,
                    &b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    empty.push([polar_index, wide_index]);
                    continue;
                }
                assert!(child.is_complete());
                assert!(child.points.is_empty());
                assert!(child.curves.is_empty());
                assert_eq!(child.regions.len(), 1);
                occupied.push((
                    [polar_index, wide_index],
                    child
                        .regions
                        .pop()
                        .expect("one occupied cap-row child region was required"),
                ));
            }
        }
        assert_eq!(empty, [[0, 0], [0, 1], [0, 2]]);
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[1, 0], [1, 1], [1, 2]]
        );

        let first_seam = wide_pieces[1][0].lo;
        let second_seam = wide_pieces[2][0].lo;
        let second_left_edge = exact_sphere_region_seam_edge(&occupied[1].1, false, second_seam)
            .expect("the middle child must own the second cap-row seam");
        let second_right_edge = exact_sphere_region_seam_edge(&occupied[2].1, false, second_seam)
            .expect("the third child must own the second cap-row seam");
        assert!(sphere_region_vertices_are_bit_exact(
            occupied[1].1.boundary[second_left_edge[0]],
            occupied[2].1.boundary[second_right_edge[1]],
        ));
        assert!(sphere_region_vertices_are_bit_exact(
            occupied[1].1.boundary[second_left_edge[1]],
            occupied[2].1.boundary[second_right_edge[0]],
        ));
        merge_exact_adjacent_sphere_regions(&occupied[1].1, &occupied[2].1, false, second_seam)
            .expect("the second cap-row sibling pair must own one exact seam");
        let first_merge =
            merge_exact_adjacent_sphere_regions(&occupied[0].1, &occupied[1].1, false, first_seam)
                .expect("the first cap-row seam must be exact");
        let merged =
            merge_exact_adjacent_sphere_regions(&first_merge, &occupied[2].1, false, second_seam)
                .expect("the second cap-row seam must be exact");
        assert_eq!(merged.boundary.len(), 11);
        assert!(exact_sphere_region_seam_edge(&merged, false, first_seam).is_none());
        assert!(exact_sphere_region_seam_edge(&merged, false, second_seam).is_none());
        let latitude_seam = polar_pieces[0][1].hi;
        assert!(
            merged
                .boundary
                .iter()
                .all(|vertex| vertex.uv_a[1].to_bits() != latitude_seam.to_bits())
        );

        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 11);

        let mut mismatched = occupied[2].1.clone();
        let seam_edge = exact_sphere_region_seam_edge(&mismatched, false, second_seam)
            .expect("the third child owns the second shared seam");
        let endpoint = seam_edge[0];
        mismatched.boundary[endpoint].uv_a[0] =
            f64::from_bits(mismatched.boundary[endpoint].uv_a[0].to_bits() + 1);
        assert!(
            merge_exact_adjacent_sphere_regions(&first_merge, &mismatched, false, second_seam,)
                .is_none()
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn exact_polar_by_wide_non_cap_row_path_and_limits_are_exact() {
        let (a, b, a_range, b_range) = exact_polar_by_wide_non_cap_row_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let polar_pieces = decompose_general_sphere_polar_window(a_range).unwrap();
        let wide_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut occupied = Vec::new();
        let mut empty = Vec::new();
        for (polar_index, polar_piece) in polar_pieces.into_iter().enumerate() {
            let (pair_limit, arc_limit) = if polar_index == 0 {
                (
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                )
            } else {
                (
                    GENERAL_SPHERE_POLAR_CELL_PAIR_LIMIT,
                    GENERAL_SPHERE_POLAR_CELL_ARC_LIMIT,
                )
            };
            for (wide_index, wide_piece) in wide_pieces.into_iter().enumerate() {
                let piece_allowance =
                    arbitrary_sphere_octant_parameter_allowance(polar_piece, wide_piece).unwrap();
                let mut child = certify_general_sphere_window_arrangement(
                    &a,
                    polar_piece,
                    &b,
                    wide_piece,
                    Tolerances::default(),
                    pair_limit,
                    arc_limit,
                    piece_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    empty.push([polar_index, wide_index]);
                } else {
                    assert!(child.is_complete());
                    assert!(child.points.is_empty());
                    assert!(child.curves.is_empty());
                    assert_eq!(child.regions.len(), 1);
                    occupied.push((
                        [polar_index, wide_index],
                        child
                            .regions
                            .pop()
                            .expect("one occupied non-cap-row child region was required"),
                    ));
                }
            }
        }
        assert_eq!(empty, [[1, 0], [1, 1], [1, 2]]);
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[0, 0], [0, 1], [0, 2]]
        );

        let first_seam = wide_pieces[1][0].lo;
        let second_seam = wide_pieces[2][0].lo;
        let first_merge =
            merge_exact_adjacent_sphere_regions(&occupied[0].1, &occupied[1].1, false, first_seam)
                .expect("the first non-cap-row seam must be exact");
        let merged =
            merge_exact_adjacent_sphere_regions(&first_merge, &occupied[2].1, false, second_seam)
                .expect("the second non-cap-row seam must be exact");
        assert!(exact_sphere_region_seam_edge(&merged, false, first_seam).is_none());
        assert!(exact_sphere_region_seam_edge(&merged, false, second_seam).is_none());
        let latitude_seam = polar_pieces[0][1].hi;
        assert!(
            merged
                .boundary
                .iter()
                .all(|vertex| vertex.uv_a[1].to_bits() != latitude_seam.to_bits())
        );

        let hit = certify_polar_by_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            true,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
            GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert!(hit.points.is_empty());
        assert!(hit.curves.is_empty());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 8);
        assert!(
            merged
                .boundary
                .iter()
                .all(|vertex| hit.regions[0].boundary.contains(vertex))
        );
        assert!(matches!(
            hit.regions[0].correspondence,
            SurfaceRegionCorrespondence::GeneralSphereWindow(_)
        ));
        assert!(hit.regions[0].boundary.iter().all(|vertex| {
            a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= hit.regions[0].max_residual
        }));

        let mut mismatched = occupied[2].1.clone();
        let seam_edge = exact_sphere_region_seam_edge(&mismatched, false, second_seam)
            .expect("the third child owns the second shared seam");
        let endpoint = seam_edge[0];
        mismatched.boundary[endpoint].uv_a[0] =
            f64::from_bits(mismatched.boundary[endpoint].uv_a[0].to_bits() + 1);
        assert!(
            merge_exact_adjacent_sphere_regions(&first_merge, &mismatched, false, second_seam)
                .is_none()
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT - 1,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT,
                "general coincident sphere polar-by-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_POLAR_WIDE_UNION_PIECE_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_PAIR_LIMIT,
                GENERAL_SPHERE_POLAR_WIDE_UNION_ARC_LIMIT - 1,
                "general coincident sphere polar-by-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_polar_by_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    true,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn nine_cell_exhaustive_union_and_limits_are_exact() {
        let (a, b, a_range, b_range) = nine_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_double_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 17);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
                "general coincident sphere both-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_double_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn corner_empty_eight_cell_union_and_limits_are_exact() {
        let (a, b, a_range, b_range) = corner_empty_eight_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_double_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 17);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
                "general coincident sphere both-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_double_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn eight_cell_union_and_limits_are_exact() {
        let (a, b, a_range, b_range) = eight_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_double_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 18);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
                "general coincident sphere both-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_double_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn seven_cell_closed_seam_proof_and_limits_are_exact() {
        let (a, b, a_range, b_range) = seven_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_double_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 15);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
                "general coincident sphere both-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_double_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }

        let a_pieces = decompose_general_sphere_wide_window(a_range, allowance).unwrap();
        let b_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let child_region = |a_index, b_index| {
            let child_range_a = a_pieces[a_index];
            let child_range_b = b_pieces[b_index];
            let child_allowance =
                arbitrary_sphere_octant_parameter_allowance(child_range_a, child_range_b).unwrap();
            certify_general_sphere_windows(
                &a,
                child_range_a,
                &b,
                child_range_b,
                Tolerances::default(),
                GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                child_allowance,
            )
            .unwrap()
            .regions
            .into_iter()
            .next()
            .unwrap()
        };
        let first = child_region(1, 0);
        let second = child_region(1, 1);
        let shared = exact_sphere_region_shared_seam_edges(
            &first,
            &second,
            false,
            b_pieces[1][0].lo,
            &a_pieces,
            &b_pieces,
        )
        .unwrap();
        assert_eq!(shared.exact_owner, ExactSphereSeamOwner::First);

        // Exact identity in the unchanged chart is mandatory. A one-ULP
        // mismatch there cannot borrow the closed-cell proof.
        let mut mismatched = second;
        let endpoint = shared.second_edge[0];
        mismatched.boundary[endpoint].uv_a[0] =
            f64::from_bits(mismatched.boundary[endpoint].uv_a[0].to_bits() + 1);
        assert!(
            exact_sphere_region_shared_seam_edges(
                &first,
                &mismatched,
                false,
                b_pieces[1][0].lo,
                &a_pieces,
                &b_pieces,
            )
            .is_none()
        );
    }

    #[test]
    fn opposite_corner_empty_seven_cell_union_and_limits_are_exact() {
        let (a, b, a_range, b_range) = opposite_corner_empty_seven_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_double_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 1);
        assert_eq!(hit.regions[0].boundary.len(), 17);

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
                "general coincident sphere both-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_double_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }
    }

    #[test]
    fn disconnected_five_cell_limits_and_separation_are_exact() {
        let (a, b, a_range, b_range) = disconnected_five_cell_fixture();
        let allowance = arbitrary_sphere_octant_parameter_allowance(a_range, b_range).unwrap();
        let hit = certify_double_wide_sphere_window_union(
            &a,
            a_range,
            &b,
            b_range,
            Tolerances::default(),
            allowance,
            GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
            GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
        )
        .unwrap();
        assert!(hit.is_complete());
        assert_eq!(hit.regions.len(), 2);
        assert_eq!(
            hit.regions
                .iter()
                .map(|region| region.boundary.len())
                .collect::<Vec<_>>(),
            [3, 8]
        );

        for (piece_limit, pair_limit, arc_limit, reason) in [
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union piece limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT - 1,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT,
                "general coincident sphere both-wide union pair limit exhausted",
            ),
            (
                GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT - 1,
                "general coincident sphere both-wide union arc limit exhausted",
            ),
        ] {
            assert_eq!(
                certify_double_wide_sphere_window_union(
                    &a,
                    a_range,
                    &b,
                    b_range,
                    Tolerances::default(),
                    allowance,
                    piece_limit,
                    pair_limit,
                    arc_limit,
                )
                .unwrap_err(),
                Error::InvalidGeometry { reason }
            );
        }

        let a_pieces = decompose_general_sphere_wide_window(a_range, allowance).unwrap();
        let b_pieces = decompose_general_sphere_wide_window(b_range, allowance).unwrap();
        let mut empty = [[false; GENERAL_SPHERE_WIDE_PIECE_LIMIT]; GENERAL_SPHERE_WIDE_PIECE_LIMIT];
        let mut occupied = Vec::new();
        for (a_index, &a_piece) in a_pieces.iter().enumerate() {
            for (b_index, &b_piece) in b_pieces.iter().enumerate() {
                let child_allowance =
                    arbitrary_sphere_octant_parameter_allowance(a_piece, b_piece).unwrap();
                let child = certify_general_sphere_windows(
                    &a,
                    a_piece,
                    &b,
                    b_piece,
                    Tolerances::default(),
                    GENERAL_SPHERE_WINDOW_PAIR_LIMIT,
                    GENERAL_SPHERE_WINDOW_ARC_LIMIT,
                    child_allowance,
                )
                .unwrap();
                if child.is_proven_empty() {
                    empty[a_index][b_index] = true;
                } else {
                    assert!(child.is_complete());
                    assert_eq!(child.regions.len(), 1);
                    occupied.push((
                        [a_index, b_index],
                        child.regions.into_iter().next().unwrap(),
                    ));
                }
            }
        }
        assert_eq!(
            occupied.iter().map(|(cell, _)| *cell).collect::<Vec<_>>(),
            [[0, 2], [1, 0], [1, 1], [2, 0], [2, 1]]
        );
        assert_eq!(
            merge_exact_sphere_region_components(&occupied, &a_pieces, &b_pieces, &empty)
                .unwrap()
                .iter()
                .map(|region| region.boundary.len())
                .collect::<Vec<_>>(),
            [3, 8]
        );

        // The diagonal pair [0, 2]/[1, 1] is separated only when both
        // orthogonal closed-cell owners [0, 1] and [1, 2] certify empty.
        for missing_owner in [[0, 1], [1, 2]] {
            let mut incomplete_empty = empty;
            incomplete_empty[missing_owner[0]][missing_owner[1]] = false;
            assert!(
                merge_exact_sphere_region_components(
                    &occupied,
                    &a_pieces,
                    &b_pieces,
                    &incomplete_empty,
                )
                .is_none()
            );
        }
    }
}
