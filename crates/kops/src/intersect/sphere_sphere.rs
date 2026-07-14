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
            (true, false) | (false, true) => (
                GENERAL_SPHERE_WIDE_PAIR_LIMIT,
                GENERAL_SPHERE_WIDE_ARC_LIMIT,
            ),
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
const GENERAL_SPHERE_WIDE_PIECE_LIMIT: usize = 3;
const GENERAL_SPHERE_WIDE_PAIR_LIMIT: usize =
    GENERAL_SPHERE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_PAIR_LIMIT;
const GENERAL_SPHERE_WIDE_ARC_LIMIT: usize =
    GENERAL_SPHERE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_ARC_LIMIT;
const GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT: usize =
    GENERAL_SPHERE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WIDE_PIECE_LIMIT;
const GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT: usize =
    GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_PAIR_LIMIT;
const GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT: usize =
    GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT * GENERAL_SPHERE_WINDOW_ARC_LIMIT;

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

    let constraints = general_sphere_window_constraints(a, a_range)?
        .into_iter()
        .chain(general_sphere_window_constraints(b, b_range)?)
        .collect::<Vec<_>>();
    debug_assert_eq!(constraints.len(), 8);

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
        // Pairwise interval discriminants found every crossing of the eight
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
    let mut occupied_regions = Vec::with_capacity(2);
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
                continue;
            }
            if !hit.is_complete()
                || !hit.points.is_empty()
                || !hit.curves.is_empty()
                || hit.regions.len() != 1
            {
                return Err(Error::InvalidGeometry {
                    reason: "general coincident sphere both-wide union supports at most two positive cells with certified-empty siblings and exact shared-seam evidence",
                });
            }
            if occupied_regions.len() == 2 {
                return Err(Error::InvalidGeometry {
                    reason: "general coincident sphere both-wide union supports at most two positive cells with certified-empty siblings and exact shared-seam evidence",
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
    let mut merged_adjacent_region = None;
    let supported_positive_cells = match occupied_regions.as_slice() {
        [_] => certified_empty_pairs + 1 == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT,
        [(first, first_region), (second, second_region)] => {
            let a_delta = first[0].abs_diff(second[0]);
            let b_delta = first[1].abs_diff(second[1]);
            let bounded_two_cell_proof = certified_empty_pairs + 2
                == GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT
                && a_range[0].width() < core::f64::consts::TAU - parent_parameter_allowance
                && b_range[0].width() < core::f64::consts::TAU - parent_parameter_allowance;
            if !bounded_two_cell_proof {
                false
            } else if a_delta == 1 && b_delta == 0 {
                let seam = a_pieces[first[0].max(second[0])][0].lo;
                merged_adjacent_region =
                    merge_exact_adjacent_sphere_regions(first_region, second_region, true, seam);
                merged_adjacent_region.is_some()
            } else if a_delta == 0 && b_delta == 1 {
                let seam = b_pieces[first[1].max(second[1])][0].lo;
                merged_adjacent_region =
                    merge_exact_adjacent_sphere_regions(first_region, second_region, false, seam);
                merged_adjacent_region.is_some()
            } else {
                true
            }
        }
        _ => false,
    };
    if !supported_positive_cells {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere both-wide union supports at most two positive cells with certified-empty siblings and exact shared-seam evidence",
        });
    }

    // Empty artificial-seam neighbors isolate nonadjacent retained cells.
    // Adjacent cells are admitted only when both child cycles expose the same
    // bit-exact seam edge with reverse orientation; splicing their two outer
    // paths removes that edge. Pole-clear sub-full-turn parent charts are
    // injective, so the resulting one or two cycles have only true parent
    // boundaries and may use the parent correspondence.
    let parent_residual = arbitrary_sphere_octant_residual_bound(a, b, parent_parameter_allowance)?;
    let parent_map = general_sphere_window_map(a, a_range, b, b_range, parent_parameter_allowance);
    let source_regions = if let Some(region) = merged_adjacent_region {
        vec![region]
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

fn merge_exact_adjacent_sphere_regions(
    first: &SurfaceSurfaceRegion,
    second: &SurfaceSurfaceRegion,
    seam_on_first_operand: bool,
    seam: f64,
) -> Option<SurfaceSurfaceRegion> {
    if first.orientation != SurfaceRegionOrientation::Same
        || second.orientation != SurfaceRegionOrientation::Same
    {
        return None;
    }
    let first_edge = exact_sphere_region_seam_edge(first, seam_on_first_operand, seam)?;
    let second_edge = exact_sphere_region_seam_edge(second, seam_on_first_operand, seam)?;
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
    let seam_bits = seam.to_bits();
    let vertices = region
        .boundary
        .iter()
        .enumerate()
        .filter_map(|(index, vertex)| {
            let parameter = if seam_on_first_operand {
                vertex.uv_a[0]
            } else {
                vertex.uv_b[0]
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
    if range[0].width() <= parameter_allowance
        || range[1].width() <= parameter_allowance
        || range[1].lo <= -half_pi + parameter_allowance
        || range[1].hi >= half_pi - parameter_allowance
    {
        return Err(Error::InvalidGeometry {
            reason: "general coincident sphere window proof supports only positive-area pole-clear windows",
        });
    }
    Ok(())
}

fn general_sphere_window_constraints(
    sphere: &Sphere,
    range: [ParamRange; 2],
) -> Result<[SphereWindowConstraint; 4]> {
    let frame = sphere.frame();
    let (sin_u_lo, cos_u_lo) = math::sincos(range[0].lo);
    let (sin_u_hi, cos_u_hi) = math::sincos(range[0].hi);
    let (sin_v_lo, _) = math::sincos(range[1].lo);
    let (sin_v_hi, _) = math::sincos(range[1].hi);
    [
        (frame.y() * cos_u_lo - frame.x() * sin_u_lo, 0.0),
        (frame.x() * sin_u_hi - frame.y() * cos_u_hi, 0.0),
        (frame.z(), sin_v_lo),
        (-frame.z(), -sin_v_hi),
    ]
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
    .into_iter()
    .collect::<Result<Vec<_>>>()?
    .try_into()
    .map_err(|_| Error::InvalidGeometry {
        reason: "general coincident sphere window boundary plane count is invalid",
    })
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

        let adjacent_angle = 0.05;
        let adjacent_b = Sphere::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(math::sin(adjacent_angle), 0.0, math::cos(adjacent_angle)),
                Vec3::new(math::cos(adjacent_angle), 0.0, -math::sin(adjacent_angle)),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        let double_wide_a_range = [
            ParamRange::new(-0.6, -0.6 + 1.01 * core::f64::consts::PI),
            ParamRange::new(-0.2, 0.2),
        ];
        let double_wide_b_range = [
            ParamRange::new(1.4, 1.4 + 1.3 * core::f64::consts::PI),
            ParamRange::new(-0.2, 0.2),
        ];
        let double_wide_allowance =
            arbitrary_sphere_octant_parameter_allowance(double_wide_a_range, double_wide_b_range)
                .unwrap();
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_PIECE_LIMIT, 9);
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_PAIR_LIMIT, 252);
        assert_eq!(GENERAL_SPHERE_DOUBLE_WIDE_ARC_LIMIT, 1_008);
        let double_wide = certify_double_wide_sphere_window_union(
            &a,
            double_wide_a_range,
            &adjacent_b,
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
        assert_eq!(double_wide.regions[0].boundary.len(), 8);
        let transposed_allowance =
            arbitrary_sphere_octant_parameter_allowance(double_wide_b_range, double_wide_a_range)
                .unwrap();
        let transposed_double_wide = certify_double_wide_sphere_window_union(
            &a,
            double_wide_b_range,
            &adjacent_b,
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
        assert_eq!(transposed_double_wide.regions[0].boundary.len(), 8);
        assert_eq!(
            certify_double_wide_sphere_window_union(
                &a,
                double_wide_a_range,
                &adjacent_b,
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
                &adjacent_b,
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
                &adjacent_b,
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
}
