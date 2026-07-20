//! Conservative numeric realization of symbolic three-plane vertices.
//!
//! The symbolic BSP remains the authority for every topological decision.
//! This module only supplies finite model-space representatives to the
//! topology assembler.  It encloses the exact intersection with outward
//! interval Cramer arithmetic, rejects ill-conditioned systems, and checks
//! both defining-plane residuals and strict source-plane sides before a
//! representative may escape.

use kcore::interval::Interval;
use kcore::operation::OperationContext;
use kcore::plane_triple::{PlaneTripleEnclosureError, enclose_plane_triple_intersection};
use kcore::predicates::{
    Orientation, OrientedPlanePoints, oriented_plane_triple_intersection_side,
};
use kgeom::vec::Point3;

use super::planar_bsp::{PlaneTripleVertexKey, SourcePlane, SourcePlaneRef};

/// One strict source-plane relation certified for the numeric representative.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedSide {
    plane: SourcePlaneRef,
    side: Orientation,
    distance_lower_bound: f64,
}

impl CertifiedSide {
    /// Stable source-plane identity carried with the side evidence.
    pub(crate) const fn plane(self) -> SourcePlaneRef {
        self.plane
    }

    /// Exact side shared by the symbolic vertex and numeric representative.
    pub(crate) const fn side(self) -> Orientation {
        self.side
    }

    /// Conservative positive distance from the representative to the plane.
    pub(crate) const fn distance_lower_bound(self) -> f64 {
        self.distance_lower_bound
    }
}

/// One defining-plane residual bound retained with its stable identity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedDefiningPlaneResidual {
    plane: SourcePlaneRef,
    distance_upper_bound: f64,
}

impl CertifiedDefiningPlaneResidual {
    /// Stable identity of the defining plane whose residual was bounded.
    pub(crate) const fn plane(self) -> SourcePlaneRef {
        self.plane
    }

    /// Conservative distance from the representative to the defining plane.
    pub(crate) const fn distance_upper_bound(self) -> f64 {
        self.distance_upper_bound
    }
}

/// A finite representative with proof-bearing numeric error bounds.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RealizedPlaneTriple {
    point: Point3,
    position_error_bound: f64,
    defining_plane_residuals: [CertifiedDefiningPlaneResidual; 3],
    certified_sides: Vec<CertifiedSide>,
}

impl RealizedPlaneTriple {
    /// Deterministic finite point passed to geometric/topological assembly.
    pub(crate) const fn point(&self) -> Point3 {
        self.point
    }

    /// Conservative distance from the point to the exact three-plane vertex.
    pub(crate) const fn position_error_bound(&self) -> f64 {
        self.position_error_bound
    }

    /// Defining-plane residual evidence in stable plane-identity order.
    pub(crate) const fn defining_plane_residuals(&self) -> &[CertifiedDefiningPlaneResidual; 3] {
        &self.defining_plane_residuals
    }

    /// Strict non-defining plane relations checked during realization.
    pub(crate) fn certified_sides(&self) -> &[CertifiedSide] {
        &self.certified_sides
    }
}

/// Honest refusal from numeric three-plane realization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RealizationError {
    /// A symbolic plane identity is absent or duplicated in the registry.
    UnknownPlane,
    /// A plane witness is non-finite, degenerate, or outside the session box.
    InvalidPlane,
    /// The interval solve could not certify a unique intersection.
    UncertifiedIntersection,
    /// The unique intersection is too ill-conditioned for numeric assembly.
    IllConditioned,
    /// The exact vertex is not tightly representable inside the session box.
    UnrepresentablePoint,
    /// A defining-plane metric residual could not be bounded at tolerance.
    ResidualTooLarge,
    /// A requested strict side is an exact boundary contact.
    BoundaryContact,
    /// Strict-side identities are duplicated or name a defining plane.
    InvalidSideSet,
    /// A strict symbolic side could not be retained by stable numeric evaluation.
    UncertifiedSide,
}

type PlaneIntervals = [Interval; 4];

/// Realize one BSP vertex from its stable symbolic key and source-plane set.
///
/// Plane and side identities are resolved uniquely before any arithmetic.
/// The defining identities are already canonical in the vertex key, while
/// the numeric witness path independently canonicalizes whole plane records.
pub(crate) fn realize_symbolic_vertex(
    context: &OperationContext<'_>,
    source_planes: &[SourcePlane],
    vertex: PlaneTripleVertexKey,
    strict_side_planes: &[SourcePlaneRef],
) -> Result<RealizedPlaneTriple, RealizationError> {
    let defining_ids = vertex.planes();
    let defining_planes = defining_ids
        .map(|plane| {
            unique_source_plane(source_planes, plane).map(|source| (plane, source.points()))
        })
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .map_err(|_| RealizationError::UnknownPlane)?;
    let mut side_ids = strict_side_planes.to_vec();
    side_ids.sort_unstable();
    if side_ids.windows(2).any(|pair| pair[0] == pair[1])
        || side_ids.iter().any(|plane| defining_ids.contains(plane))
    {
        return Err(RealizationError::InvalidSideSet);
    }
    let side_planes = side_ids
        .iter()
        .map(|&plane| {
            unique_source_plane(source_planes, plane).map(|witness| (plane, witness.points()))
        })
        .collect::<Result<Vec<_>, _>>()?;
    realize_witness_triple(context, defining_planes, &side_planes)
}

fn unique_source_plane(
    source_planes: &[SourcePlane],
    id: SourcePlaneRef,
) -> Result<SourcePlane, RealizationError> {
    let mut matching = source_planes
        .iter()
        .copied()
        .filter(|plane| plane.id() == id);
    let plane = matching.next().ok_or(RealizationError::UnknownPlane)?;
    if matching.next().is_some() {
        return Err(RealizationError::UnknownPlane);
    }
    Ok(plane)
}

/// Realize one exact plane triple without granting rounded arithmetic any
/// topological authority.
///
/// The three defining planes are canonicalized as whole witnesses, so their
/// caller permutation does not alter output bits. `side_planes` are
/// non-defining source planes against which the exact intersection has a
/// strict relation. Their returned evidence remains in caller order.
fn realize_witness_triple(
    context: &OperationContext<'_>,
    mut defining_planes: [(SourcePlaneRef, OrientedPlanePoints); 3],
    side_planes: &[(SourcePlaneRef, OrientedPlanePoints)],
) -> Result<RealizedPlaneTriple, RealizationError> {
    defining_planes.sort_by(compare_plane_records);
    let defining_witnesses = defining_planes.map(|(_, witness)| witness);
    let size_box_half = context.session().precision().size_box_half();
    let enclosure =
        enclose_plane_triple_intersection(defining_witnesses, size_box_half, f64::MIN_POSITIVE)
            .map_err(map_enclosure_error)?;
    if !context
        .session()
        .numerical()
        .reciprocal_condition_is_usable(enclosure.reciprocal_condition_lower_bound())
    {
        return Err(RealizationError::IllConditioned);
    }
    let coordinates = enclosure.coordinates();

    let planes = defining_witnesses
        .map(|plane| interval_oriented_plane(plane, size_box_half))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    let planes: [PlaneIntervals; 3] = planes
        .try_into()
        .map_err(|_| RealizationError::UncertifiedIntersection)?;

    let point_coordinates = coordinates.map(interval_midpoint);
    if point_coordinates
        .iter()
        .any(|coordinate| !coordinate.is_finite() || coordinate.abs() > size_box_half)
    {
        return Err(RealizationError::UnrepresentablePoint);
    }
    let point = Point3::from_array(point_coordinates);
    let position_error_bound = point_box_error_bound(coordinates, point_coordinates)
        .ok_or(RealizationError::UnrepresentablePoint)?;
    if position_error_bound > context.tolerances().linear() {
        return Err(RealizationError::UnrepresentablePoint);
    }

    let mut defining_plane_residuals = Vec::with_capacity(3);
    for (index, plane) in planes.iter().enumerate() {
        let distance = plane_distance_upper(*plane, point_coordinates)
            .ok_or(RealizationError::ResidualTooLarge)?;
        if distance > context.tolerances().linear() {
            return Err(RealizationError::ResidualTooLarge);
        }
        defining_plane_residuals.push(CertifiedDefiningPlaneResidual {
            plane: defining_planes[index].0,
            distance_upper_bound: distance,
        });
    }
    defining_plane_residuals.sort_by_key(|residual| residual.plane);
    let defining_plane_residuals = defining_plane_residuals
        .try_into()
        .map_err(|_| RealizationError::UncertifiedIntersection)?;

    let mut certified_sides = Vec::with_capacity(side_planes.len());
    for &(plane_id, side_plane) in side_planes {
        let exact_side = oriented_plane_triple_intersection_side(defining_witnesses, side_plane)
            .ok_or(RealizationError::UncertifiedSide)?
            .sign();
        if exact_side == Orientation::Zero {
            return Err(RealizationError::BoundaryContact);
        }
        let plane = interval_oriented_plane(side_plane, size_box_half)?;
        let value = evaluate_plane(plane, point_coordinates);
        let numeric_side = value
            .sign()
            .map(|sign| match sign {
                -1 => Orientation::Negative,
                0 => Orientation::Zero,
                1 => Orientation::Positive,
                _ => unreachable!("Interval::sign returns only -1, 0, or 1"),
            })
            .ok_or(RealizationError::UncertifiedSide)?;
        if numeric_side != exact_side {
            return Err(RealizationError::UncertifiedSide);
        }
        let distance_lower_bound =
            plane_distance_lower(plane, value).ok_or(RealizationError::UncertifiedSide)?;
        certified_sides.push(CertifiedSide {
            plane: plane_id,
            side: exact_side,
            distance_lower_bound,
        });
    }

    Ok(RealizedPlaneTriple {
        point,
        position_error_bound,
        defining_plane_residuals,
        certified_sides,
    })
}

fn map_enclosure_error(error: PlaneTripleEnclosureError) -> RealizationError {
    match error {
        PlaneTripleEnclosureError::NonFinitePlane
        | PlaneTripleEnclosureError::PlaneWitnessOutsideSizeBox
        | PlaneTripleEnclosureError::UncertifiedPlane => RealizationError::InvalidPlane,
        PlaneTripleEnclosureError::UncertifiedIntersection => {
            RealizationError::UncertifiedIntersection
        }
        PlaneTripleEnclosureError::IllConditioned => RealizationError::IllConditioned,
        PlaneTripleEnclosureError::IntersectionOutsideSizeBox => {
            RealizationError::UnrepresentablePoint
        }
        PlaneTripleEnclosureError::InvalidLimits => RealizationError::UncertifiedIntersection,
    }
}

fn compare_plane_records(
    first: &(SourcePlaneRef, OrientedPlanePoints),
    second: &(SourcePlaneRef, OrientedPlanePoints),
) -> core::cmp::Ordering {
    compare_plane_witnesses(&first.1, &second.1).then_with(|| first.0.cmp(&second.0))
}

fn compare_plane_witnesses(
    first: &OrientedPlanePoints,
    second: &OrientedPlanePoints,
) -> core::cmp::Ordering {
    first
        .iter()
        .flatten()
        .zip(second.iter().flatten())
        .map(|(first, second)| first.total_cmp(second))
        .find(|ordering| !ordering.is_eq())
        .unwrap_or(core::cmp::Ordering::Equal)
}

fn interval_oriented_plane(
    points: OrientedPlanePoints,
    size_box_half: f64,
) -> Result<PlaneIntervals, RealizationError> {
    if points
        .iter()
        .flatten()
        .any(|coordinate| !coordinate.is_finite() || coordinate.abs() > size_box_half)
    {
        return Err(RealizationError::InvalidPlane);
    }
    let u =
        [0, 1, 2].map(|axis| Interval::point(points[1][axis]) - Interval::point(points[0][axis]));
    let v =
        [0, 1, 2].map(|axis| Interval::point(points[2][axis]) - Interval::point(points[0][axis]));
    let conventional = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    if conventional
        .iter()
        .all(|component| component.contains_zero())
    {
        return Err(RealizationError::InvalidPlane);
    }
    let spatial = conventional.map(|component| -component);
    let constant = conventional[0] * Interval::point(points[0][0])
        + conventional[1] * Interval::point(points[0][1])
        + conventional[2] * Interval::point(points[0][2]);
    let plane = [spatial[0], spatial[1], spatial[2], constant];
    plane
        .iter()
        .all(|component| interval_is_finite(*component))
        .then_some(plane)
        .ok_or(RealizationError::InvalidPlane)
}

fn interval_is_finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}

fn interval_abs_upper(interval: Interval) -> Option<f64> {
    let upper = interval.lo().abs().max(interval.hi().abs());
    upper.is_finite().then_some(upper)
}

fn strict_magnitude_lower(interval: Interval) -> Option<f64> {
    let lower = if interval.lo() > 0.0 {
        interval.lo()
    } else if interval.hi() < 0.0 {
        -interval.hi()
    } else {
        return None;
    };
    (lower.is_finite() && lower > 0.0).then_some(lower)
}

fn interval_midpoint(interval: Interval) -> f64 {
    interval.lo() + (interval.hi() - interval.lo()) * 0.5
}

fn point_box_error_bound(coordinates: [Interval; 3], point: [f64; 3]) -> Option<f64> {
    let mut squared = Interval::point(0.0);
    for axis in 0..3 {
        squared = squared + (coordinates[axis] - Interval::point(point[axis])).square();
    }
    let bound = squared.sqrt()?.hi();
    bound.is_finite().then_some(bound)
}

fn evaluate_plane(plane: PlaneIntervals, point: [f64; 3]) -> Interval {
    plane[0] * Interval::point(point[0])
        + plane[1] * Interval::point(point[1])
        + plane[2] * Interval::point(point[2])
        + plane[3]
}

fn plane_normal_norm(plane: PlaneIntervals) -> Option<Interval> {
    let squared = plane[0].square() + plane[1].square() + plane[2].square();
    let norm = squared.sqrt()?;
    (interval_is_finite(norm) && norm.lo() > 0.0).then_some(norm)
}

fn plane_distance_upper(plane: PlaneIntervals, point: [f64; 3]) -> Option<f64> {
    let residual = interval_abs_upper(evaluate_plane(plane, point))?;
    let normal = plane_normal_norm(plane)?;
    let distance = Interval::point(residual)
        .checked_div(Interval::point(normal.lo()))?
        .hi();
    distance.is_finite().then_some(distance)
}

fn plane_distance_lower(plane: PlaneIntervals, value: Interval) -> Option<f64> {
    let residual = strict_magnitude_lower(value)?;
    let normal = plane_normal_norm(plane)?;
    let distance = Interval::point(residual)
        .checked_div(Interval::point(normal.hi()))?
        .lo();
    (distance.is_finite() && distance > 0.0).then_some(distance)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kcore::operation::SessionPolicy;
    use kcore::predicates::orient3d;
    use kcore::tolerance::Tolerances;

    fn context() -> OperationContext<'static> {
        let policy = Box::leak(Box::new(SessionPolicy::v1()));
        OperationContext::new(policy, Tolerances::default()).unwrap()
    }

    fn cross(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
        [
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        ]
    }

    fn add(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
        core::array::from_fn(|axis| first[axis] + second[axis])
    }

    fn scale(vector: [f64; 3], factor: f64) -> [f64; 3] {
        vector.map(|component| component * factor)
    }

    fn euclidean_plane_distance(normal: [f64; 3], anchor: [f64; 3], point: [f64; 3]) -> f64 {
        let residual = normal
            .into_iter()
            .zip(point.into_iter().zip(anchor))
            .map(|(normal, (point, anchor))| normal * (point - anchor))
            .sum::<f64>()
            .abs();
        let norm = normal
            .into_iter()
            .map(|component| component * component)
            .sum::<f64>()
            .sqrt();
        residual / norm
    }

    /// Three exact points on `normal . (x - anchor) = 0`. The least-aligned
    /// coordinate direction is selected only to keep this independent test
    /// oracle numerically broad; the production solve has no such pivot.
    fn plane_witness(normal: [f64; 3], anchor: [f64; 3]) -> OrientedPlanePoints {
        let least_aligned = (0..3)
            .min_by(|&first, &second| normal[first].abs().total_cmp(&normal[second].abs()))
            .unwrap();
        let mut basis = [0.0; 3];
        basis[least_aligned] = 1.0;
        let first = cross(normal, basis);
        let second = cross(normal, first);
        [anchor, add(anchor, first), add(anchor, second)]
    }

    fn determinant(matrix: [[i64; 3]; 3]) -> i64 {
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    }

    fn side_witness(
        face: u32,
        witness: OrientedPlanePoints,
    ) -> (SourcePlaneRef, OrientedPlanePoints) {
        (SourcePlaneRef::new(2, face), witness)
    }

    fn defining_witness(
        face: u32,
        witness: OrientedPlanePoints,
    ) -> (SourcePlaneRef, OrientedPlanePoints) {
        (SourcePlaneRef::new(0, face), witness)
    }

    fn defining_triple(
        witnesses: [OrientedPlanePoints; 3],
    ) -> [(SourcePlaneRef, OrientedPlanePoints); 3] {
        core::array::from_fn(|index| defining_witness(index as u32, witnesses[index]))
    }

    #[test]
    fn randomized_integer_intersections_match_independent_exact_oracle() {
        let context = context();
        let mut state = 0xD1B5_4A32_D192_ED03_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let mut accepted = 0;
        while accepted < 2_000 {
            let solution = core::array::from_fn(|_| (next() % 31) as i64 - 15);
            let matrix =
                core::array::from_fn(|_| core::array::from_fn(|_| (next() % 11) as i64 - 5));
            let determinant = determinant(matrix);
            let row_abs_max = matrix
                .iter()
                .flatten()
                .map(|value| value.abs())
                .max()
                .unwrap();
            if determinant.abs() < 8 || row_abs_max == 0 {
                continue;
            }
            let defining_witnesses = matrix.map(|normal| {
                let normal = normal.map(|value| value as f64);
                let solution = solution.map(|value| value as f64);
                let first = cross(normal, [1.0, -2.0, 3.0]);
                let second = cross(normal, first);
                let anchor = add(solution, add(scale(first, 0.25), scale(second, 0.125)));
                plane_witness(normal, anchor)
            });
            if defining_witnesses
                .iter()
                .flatten()
                .flatten()
                .any(|coordinate| coordinate.abs() > 500.0)
            {
                continue;
            }
            let query_normal = [2.0, -3.0, 5.0];
            let solution_f64 = solution.map(|value| value as f64);
            let query = plane_witness(query_normal, add(solution_f64, scale(query_normal, 2.0)));
            let defining = defining_triple(defining_witnesses);
            let realized =
                realize_witness_triple(&context, defining, &[side_witness(0, query)]).unwrap();
            let expected = Point3::from_array(solution_f64);
            let actual_error = realized.point().dist(expected);
            assert!(actual_error <= realized.position_error_bound());
            assert!(realized.position_error_bound() <= context.tolerances().linear());
            for residual in realized.defining_plane_residuals() {
                let index = usize::try_from(residual.plane().face()).unwrap();
                let normal = matrix[index].map(|value| value as f64);
                let actual_distance =
                    euclidean_plane_distance(normal, solution_f64, realized.point().to_array());
                assert!(actual_distance <= residual.distance_upper_bound() + 1e-12);
                assert!(residual.distance_upper_bound() <= context.tolerances().linear());
            }
            let exact_numeric_side =
                orient3d(query[0], query[1], query[2], realized.point().to_array());
            assert_eq!(
                realized.certified_sides()[0].plane(),
                SourcePlaneRef::new(2, 0)
            );
            assert_eq!(realized.certified_sides()[0].side(), exact_numeric_side);
            let query_anchor = add(solution_f64, scale(query_normal, 2.0));
            let actual_clearance =
                euclidean_plane_distance(query_normal, query_anchor, realized.point().to_array());
            assert!(realized.certified_sides()[0].distance_lower_bound() > 1.0);
            assert!(
                realized.certified_sides()[0].distance_lower_bound() <= actual_clearance + 1e-12
            );
            accepted += 1;
        }
    }

    #[test]
    fn defining_plane_permutation_preserves_representative_bits() {
        let context = context();
        let solution = [7.25, -11.5, 3.75];
        let planes = [
            defining_witness(90, plane_witness([2.0, -3.0, 5.0], solution)),
            defining_witness(3, plane_witness([-7.0, 4.0, 3.0], solution)),
            defining_witness(41, plane_witness([6.0, 5.0, -2.0], solution)),
        ];
        let mut witness_order = planes;
        witness_order.sort_by(compare_plane_records);
        assert_ne!(
            witness_order.map(|(id, _)| id),
            [planes[1].0, planes[2].0, planes[0].0],
            "fixture must make witness-coordinate and stable-ID order adversarial"
        );
        let permutations = [
            [planes[0], planes[1], planes[2]],
            [planes[0], planes[2], planes[1]],
            [planes[1], planes[0], planes[2]],
            [planes[1], planes[2], planes[0]],
            [planes[2], planes[0], planes[1]],
            [planes[2], planes[1], planes[0]],
        ];
        let expected = realize_witness_triple(&context, permutations[0], &[]).unwrap();
        assert_eq!(
            expected
                .defining_plane_residuals()
                .map(|residual| residual.plane()),
            [planes[1].0, planes[2].0, planes[0].0]
        );
        for residual in expected.defining_plane_residuals() {
            let witness = planes
                .iter()
                .find(|(id, _)| *id == residual.plane())
                .unwrap()
                .1;
            let interval =
                interval_oriented_plane(witness, context.session().precision().size_box_half())
                    .unwrap();
            let independently_indexed =
                plane_distance_upper(interval, expected.point().to_array()).unwrap();
            assert_eq!(
                residual.distance_upper_bound().to_bits(),
                independently_indexed.to_bits()
            );
        }
        for permutation in permutations.into_iter().skip(1) {
            let actual = realize_witness_triple(&context, permutation, &[]).unwrap();
            assert_eq!(actual.point().x.to_bits(), expected.point().x.to_bits());
            assert_eq!(actual.point().y.to_bits(), expected.point().y.to_bits());
            assert_eq!(actual.point().z.to_bits(), expected.point().z.to_bits());
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn singular_ill_conditioned_and_outside_vertices_fail_closed() {
        let context = context();
        let origin = [0.0; 3];
        let x = plane_witness([1.0, 0.0, 0.0], origin);
        let duplicate_x = plane_witness([2.0, 0.0, 0.0], origin);
        let z = plane_witness([0.0, 0.0, 1.0], origin);
        assert_eq!(
            realize_witness_triple(&context, defining_triple([x, duplicate_x, z]), &[]),
            Err(RealizationError::UncertifiedIntersection)
        );

        let nearly_x = plane_witness([1.0, 1e-15, 0.0], origin);
        let shared_enclosure = enclose_plane_triple_intersection(
            [x, nearly_x, z],
            context.session().precision().size_box_half(),
            f64::MIN_POSITIVE,
        )
        .unwrap();
        assert!(
            !context
                .session()
                .numerical()
                .reciprocal_condition_is_usable(
                    shared_enclosure.reciprocal_condition_lower_bound()
                )
        );
        assert_eq!(
            realize_witness_triple(&context, defining_triple([x, nearly_x, z]), &[]),
            Err(RealizationError::IllConditioned)
        );

        let outside = [
            plane_witness([1.0, 1.0, 0.0], [300.0, 300.0, 0.0]),
            plane_witness([1.0, -1.0, 0.0], [300.0, -300.0, 0.0]),
            plane_witness([0.0, 0.0, 1.0], origin),
        ];
        assert_eq!(
            realize_witness_triple(&context, defining_triple(outside), &[]),
            Err(RealizationError::UnrepresentablePoint)
        );
    }

    #[test]
    fn exact_boundary_and_numerically_unstable_strict_side_are_refused() {
        let context = context();
        let vertex = [100.0, -7.0, 3.0];
        let defining = [
            plane_witness([1.0, 0.0, 0.0], vertex),
            plane_witness([0.0, 1.0, 0.0], vertex),
            plane_witness([0.0, 0.0, 1.0], vertex),
        ];
        let boundary = plane_witness([2.0, -3.0, 5.0], vertex);
        assert_eq!(
            realize_witness_triple(
                &context,
                defining_triple(defining),
                &[side_witness(0, boundary)],
            ),
            Err(RealizationError::BoundaryContact)
        );

        let next_x = f64::from_bits(vertex[0].to_bits() + 1);
        let unstable = plane_witness([1.0, 0.0, 0.0], [next_x, vertex[1], vertex[2]]);
        assert_eq!(
            realize_witness_triple(
                &context,
                defining_triple(defining),
                &[side_witness(0, unstable)],
            ),
            Err(RealizationError::UncertifiedSide)
        );
    }

    #[test]
    fn invalid_source_witnesses_are_rejected_without_interval_panics() {
        let context = context();
        let valid = plane_witness([1.0, 0.0, 0.0], [0.0; 3]);
        let mut non_finite = valid;
        non_finite[0][0] = f64::INFINITY;
        assert_eq!(
            realize_witness_triple(&context, defining_triple([valid, valid, non_finite]), &[],),
            Err(RealizationError::InvalidPlane)
        );
        let mut outside = valid;
        outside[0][1] = 501.0;
        assert_eq!(
            realize_witness_triple(&context, defining_triple([valid, valid, outside]), &[],),
            Err(RealizationError::InvalidPlane)
        );
    }

    #[test]
    fn symbolic_vertex_resolution_requires_unique_stable_plane_ids() {
        let context = context();
        let vertex_point = [3.25, -4.5, 6.75];
        let normals = [
            [2.0, -3.0, 5.0],
            [-7.0, 4.0, 3.0],
            [6.0, 5.0, -2.0],
            [3.0, 2.0, -4.0],
            [-2.0, 6.0, 3.0],
        ];
        let ids = [
            SourcePlaneRef::new(0, 2),
            SourcePlaneRef::new(1, 7),
            SourcePlaneRef::new(0, 11),
            SourcePlaneRef::new(1, 13),
            SourcePlaneRef::new(0, 17),
        ];
        let anchors = [
            vertex_point,
            vertex_point,
            vertex_point,
            add(vertex_point, scale(normals[3], 2.0)),
            add(vertex_point, scale(normals[4], -3.0)),
        ];
        let source_planes: [SourcePlane; 5] = core::array::from_fn(|index| {
            SourcePlane::from_interior_sample(
                ids[index],
                plane_witness(normals[index], anchors[index]),
                add(anchors[index], normals[index]),
            )
            .unwrap()
        });
        let key = PlaneTripleVertexKey::new([ids[2], ids[0], ids[1]]).unwrap();
        let realized =
            realize_symbolic_vertex(&context, &source_planes, key, &[ids[4], ids[3]]).unwrap();
        let permuted =
            realize_symbolic_vertex(&context, &source_planes, key, &[ids[3], ids[4]]).unwrap();
        assert!(realized.point().dist(Point3::from_array(vertex_point)) <= 1e-12);
        assert_eq!(realized, permuted);
        assert_eq!(
            realized
                .defining_plane_residuals()
                .map(|residual| residual.plane()),
            key.planes()
        );
        let mut expected_side_ids = [ids[3], ids[4]];
        expected_side_ids.sort_unstable();
        assert_eq!(
            realized
                .certified_sides()
                .iter()
                .map(|side| side.plane())
                .collect::<Vec<_>>(),
            expected_side_ids
        );

        assert_eq!(
            realize_symbolic_vertex(&context, &source_planes, key, &[ids[3], ids[3]]),
            Err(RealizationError::InvalidSideSet)
        );
        assert_eq!(
            realize_symbolic_vertex(&context, &source_planes, key, &[ids[0]]),
            Err(RealizationError::InvalidSideSet)
        );

        assert_eq!(
            realize_symbolic_vertex(&context, &source_planes[..2], key, &[]),
            Err(RealizationError::UnknownPlane)
        );
        let mut duplicate_registry = source_planes.to_vec();
        duplicate_registry.push(source_planes[0]);
        assert_eq!(
            realize_symbolic_vertex(&context, &duplicate_registry, key, &[]),
            Err(RealizationError::UnknownPlane)
        );
    }
}
