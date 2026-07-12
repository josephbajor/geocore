//! Bounded structured-input contracts for NURBS curves and surfaces.

use kgeom::aabb::Aabb3;
use kgeom::curve::{Curve, CurveDerivs};
use kgeom::frame::Frame;
use kgeom::nurbs::{ImplicitPatchIsolation, NurbsCurve, NurbsSurface, NurbsSurfaceBvh};
use kgeom::param::ParamRange;
use kgeom::project::{CurveProjection, SurfaceProjection, project_to_curve, project_to_surface};
use kgeom::surface::{Dir, Plane, Surface, SurfaceDerivs};
use kgeom::vec::{Point3, Vec3};

/// Largest structured input admitted by this target.
pub const MAX_INPUT_BYTES: usize = 4 * 1024;
/// Largest degree decoded before construction.
pub const MAX_DEGREE: usize = 5;
/// Largest knot-vector length decoded in either parameter direction.
pub const MAX_KNOTS_PER_DIRECTION: usize = 20;
/// Largest curve control polygon decoded before construction.
pub const MAX_CURVE_POINTS: usize = 16;
/// Largest surface control net decoded before construction.
pub const MAX_SURFACE_POINTS: usize = 64;
/// Largest optional weight vector decoded before construction.
pub const MAX_WEIGHTS: usize = 64;
/// Deepest exact implicit-isolation request made by this target.
pub const MAX_ISOLATION_DEPTH: u32 = 2;
/// Soft candidate-cell budget supplied to implicit isolation.
pub const MAX_ISOLATION_CANDIDATE_CELLS: usize = 32;

const FAMILY_SURFACE: u8 = 1 << 0;
const RATIONAL: u8 = 1 << 1;
const PROJECT: u8 = 1 << 2;
const SPLIT_V: u8 = 1 << 3;

#[derive(Debug, Clone)]
struct Descriptor {
    selector: u8,
    degree_u: usize,
    degree_v: usize,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    points: Vec<Point3>,
    weights: Option<Vec<f64>>,
    parameters: [f64; 2],
    projection_point: Point3,
}

impl Descriptor {
    fn is_surface(&self) -> bool {
        self.selector & FAMILY_SURFACE != 0
    }
}

struct Decoder<'a> {
    bytes: &'a [u8],
    position: usize,
}

impl<'a> Decoder<'a> {
    const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, position: 0 }
    }

    fn byte(&mut self) -> Option<u8> {
        let value = *self.bytes.get(self.position)?;
        self.position += 1;
        Some(value)
    }

    fn float(&mut self) -> Option<f64> {
        let end = self.position.checked_add(8)?;
        let raw: [u8; 8] = self.bytes.get(self.position..end)?.try_into().ok()?;
        self.position = end;
        Some(f64::from_bits(u64::from_le_bytes(raw)))
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.position)
    }
}

/// Exercise one capped structured descriptor.
///
/// The seven-byte header carries selector flags, degrees, knot counts,
/// control-point count, and weight count. It is followed by two raw-bit query
/// parameters, one raw-bit projection point, knot vectors, row-major control
/// points, and optional weights. Counts and total byte requirements are
/// validated before any descriptor vector is allocated.
pub fn exercise(input: &[u8]) {
    if input.len() > MAX_INPUT_BYTES {
        return;
    }
    let Some(descriptor) = decode(input) else {
        return;
    };
    if descriptor.is_surface() {
        exercise_surface(descriptor);
    } else {
        exercise_curve(descriptor);
    }
}

fn decode(input: &[u8]) -> Option<Descriptor> {
    let mut decoder = Decoder::new(input);
    let selector = decoder.byte()?;
    let degree_u = usize::from(decoder.byte()?);
    let degree_v = usize::from(decoder.byte()?);
    let knots_u_count = usize::from(decoder.byte()?);
    let knots_v_count = usize::from(decoder.byte()?);
    let points_count = usize::from(decoder.byte()?);
    let weights_count = usize::from(decoder.byte()?);
    let surface = selector & FAMILY_SURFACE != 0;
    let rational = selector & RATIONAL != 0;

    if degree_u > MAX_DEGREE
        || degree_v > MAX_DEGREE
        || knots_u_count > MAX_KNOTS_PER_DIRECTION
        || knots_v_count > MAX_KNOTS_PER_DIRECTION
        || weights_count > MAX_WEIGHTS
        || (!surface && (degree_v != 0 || knots_v_count != 0))
        || (!surface && points_count > MAX_CURVE_POINTS)
        || (surface && points_count > MAX_SURFACE_POINTS)
        || (!rational && weights_count != 0)
    {
        return None;
    }

    let float_count = 5_usize
        .checked_add(knots_u_count)?
        .checked_add(knots_v_count)?
        .checked_add(points_count.checked_mul(3)?)?
        .checked_add(weights_count)?;
    if decoder.remaining() < float_count.checked_mul(8)? {
        return None;
    }

    let parameters = [decoder.float()?, decoder.float()?];
    let projection_point = Point3::new(decoder.float()?, decoder.float()?, decoder.float()?);
    let knots_u = (0..knots_u_count)
        .map(|_| decoder.float())
        .collect::<Option<Vec<_>>>()?;
    let knots_v = (0..knots_v_count)
        .map(|_| decoder.float())
        .collect::<Option<Vec<_>>>()?;
    let points = (0..points_count)
        .map(|_| {
            Some(Point3::new(
                decoder.float()?,
                decoder.float()?,
                decoder.float()?,
            ))
        })
        .collect::<Option<Vec<_>>>()?;
    let weights = if rational {
        Some(
            (0..weights_count)
                .map(|_| decoder.float())
                .collect::<Option<Vec<_>>>()?,
        )
    } else {
        None
    };
    Some(Descriptor {
        selector,
        degree_u,
        degree_v,
        knots_u,
        knots_v,
        points,
        weights,
        parameters,
        projection_point,
    })
}

fn exercise_curve(descriptor: Descriptor) {
    let selector = descriptor.selector;
    let parameters = descriptor.parameters;
    let projection_point = descriptor.projection_point;
    let result = NurbsCurve::new(
        descriptor.degree_u,
        descriptor.knots_u,
        descriptor.points,
        descriptor.weights,
    );
    let Ok(curve) = result else {
        return;
    };
    assert_curve_invariants(&curve);

    let domain = curve.param_range();
    let parameter = in_domain(parameters[0], domain);
    let order = usize::from((selector >> 4) & 0b11);
    assert_curve_derivs_bits(
        curve.eval_derivs(parameter, order),
        curve.eval_derivs(parameter, order),
    );

    if let Some(inner) = inner_range(domain) {
        assert_curve_pair_result(
            curve.split_at(midpoint(inner)),
            curve.split_at(midpoint(inner)),
        );
        assert_curve_result(curve.restricted_to(inner), curve.restricted_to(inner));
        if selector & PROJECT != 0 && point_finite(projection_point) {
            assert_curve_projection_bits(
                project_to_curve(&curve, projection_point, domain),
                project_to_curve(&curve, projection_point, domain),
            );
        }
    }
}

fn exercise_surface(descriptor: Descriptor) {
    let selector = descriptor.selector;
    let parameters = descriptor.parameters;
    let projection_point = descriptor.projection_point;
    let result = NurbsSurface::new(
        descriptor.degree_u,
        descriptor.degree_v,
        descriptor.knots_u,
        descriptor.knots_v,
        descriptor.points,
        descriptor.weights,
    );
    let Ok(surface) = result else {
        return;
    };
    assert_surface_invariants(&surface);
    assert_surface_isolation_determinism(&surface);

    let domain = surface.param_range();
    let parameters = [
        in_domain(parameters[0], domain[0]),
        in_domain(parameters[1], domain[1]),
    ];
    let order = usize::from((selector >> 4) & 0b11).min(2);
    assert_surface_derivs_bits(
        surface.eval_derivs(parameters, order),
        surface.eval_derivs(parameters, order),
    );

    let Some(inner_u) = inner_range(domain[0]) else {
        return;
    };
    let Some(inner_v) = inner_range(domain[1]) else {
        return;
    };
    let direction = if selector & SPLIT_V == 0 {
        Dir::U
    } else {
        Dir::V
    };
    let split_parameter = match direction {
        Dir::U => midpoint(inner_u),
        Dir::V => midpoint(inner_v),
    };
    assert_surface_pair_result(
        surface.split_at(direction, split_parameter),
        surface.split_at(direction, split_parameter),
    );
    assert_surface_result(
        surface.restricted_to([inner_u, inner_v]),
        surface.restricted_to([inner_u, inner_v]),
    );
    if selector & PROJECT != 0 && point_finite(projection_point) {
        assert_surface_projection_bits(
            project_to_surface(&surface, projection_point, domain),
            project_to_surface(&surface, projection_point, domain),
        );
    }
}

fn assert_curve_invariants(curve: &NurbsCurve) {
    assert!((1..=MAX_DEGREE).contains(&curve.degree()));
    assert_eq!(curve.knots().control_count(), curve.points().len());
    assert!(
        curve
            .knots()
            .as_slice()
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(
        curve
            .knots()
            .as_slice()
            .windows(2)
            .all(|window| window[0] <= window[1])
    );
    assert!(curve.points().iter().copied().all(point_finite));
    if let Some(weights) = curve.weights() {
        assert_eq!(weights.len(), curve.points().len());
        assert!(
            weights
                .iter()
                .all(|weight| weight.is_finite() && *weight > 0.0)
        );
    }
}

fn assert_surface_invariants(surface: &NurbsSurface) {
    assert!((1..=MAX_DEGREE).contains(&surface.degree_u()));
    assert!((1..=MAX_DEGREE).contains(&surface.degree_v()));
    let (nu, nv) = surface.net_size();
    assert_eq!(surface.points().len(), nu * nv);
    assert!(surface.points().iter().copied().all(point_finite));
    for direction in [Dir::U, Dir::V] {
        let knots = surface.knots(direction);
        assert!(knots.as_slice().iter().all(|value| value.is_finite()));
        assert!(
            knots
                .as_slice()
                .windows(2)
                .all(|window| window[0] <= window[1])
        );
    }
    if let Some(weights) = surface.weights() {
        assert_eq!(weights.len(), surface.points().len());
        assert!(
            weights
                .iter()
                .all(|weight| weight.is_finite() && *weight > 0.0)
        );
    }
}

fn assert_surface_isolation_determinism(surface: &NurbsSurface) {
    match (
        NurbsSurfaceBvh::build(surface),
        NurbsSurfaceBvh::build(surface),
    ) {
        (Ok(left), Ok(right)) => {
            assert_surface_bvh_bits(&left, &right);
            let plane = Plane::new(Frame::world());
            assert_surface_isolation_result(
                left.isolate_implicit_candidates(
                    &plane,
                    0.0,
                    MAX_ISOLATION_DEPTH,
                    MAX_ISOLATION_CANDIDATE_CELLS,
                ),
                right.isolate_implicit_candidates(
                    &plane,
                    0.0,
                    MAX_ISOLATION_DEPTH,
                    MAX_ISOLATION_CANDIDATE_CELLS,
                ),
            );
        }
        (Err(left), Err(right)) => assert_eq!(left, right),
        _ => panic!("repeated surface hierarchy construction changed result class"),
    }
}

fn assert_aabb_bits(left: Aabb3, right: Aabb3) {
    assert_vec_bits(left.min, right.min);
    assert_vec_bits(left.max, right.max);
}

fn assert_surface_bvh_bits(left: &NurbsSurfaceBvh, right: &NurbsSurfaceBvh) {
    assert_eq!(left.patch_count(), right.patch_count());
    assert_eq!(left.node_count(), right.node_count());
    assert_aabb_bits(left.root_bounds(), right.root_bounds());
    for index in 0..left.patch_count() {
        assert_surface_bits(
            left.patch(index).expect("index is below patch count"),
            right.patch(index).expect("index is below patch count"),
        );
        assert_aabb_bits(
            left.patch_bounds(index)
                .expect("index is below patch count"),
            right
                .patch_bounds(index)
                .expect("index is below patch count"),
        );
    }
}

fn assert_surface_isolation_result(
    left: kcore::error::Result<ImplicitPatchIsolation>,
    right: kcore::error::Result<ImplicitPatchIsolation>,
) {
    match (left, right) {
        (Ok(left), Ok(right)) => {
            assert_eq!(left.requested_depth(), right.requested_depth());
            assert_eq!(left.limits(), right.limits());
            assert_eq!(left.candidates().len(), right.candidates().len());
            for (left, right) in left.candidates().iter().zip(right.candidates()) {
                assert_eq!(left.source_patch(), right.source_patch());
                assert_eq!(left.depth(), right.depth());
                assert_surface_bits(left.patch(), right.patch());
                assert_aabb_bits(left.bounds(), right.bounds());
            }
        }
        (Err(left), Err(right)) => assert_eq!(left, right),
        _ => panic!("repeated surface isolation changed result class"),
    }
}

fn in_domain(raw: f64, range: ParamRange) -> f64 {
    if raw.is_finite() {
        raw.clamp(range.lo, range.hi)
    } else {
        range.lo
    }
}

fn midpoint(range: ParamRange) -> f64 {
    range.lo + range.width() * 0.5
}

fn inner_range(range: ParamRange) -> Option<ParamRange> {
    let width = range.width();
    if !width.is_finite() || width <= 0.0 {
        return None;
    }
    let inner = ParamRange::new(range.lo + width * 0.25, range.lo + width * 0.75);
    (range.lo < inner.lo && inner.lo < inner.hi && inner.hi < range.hi).then_some(inner)
}

fn point_finite(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn assert_vec_bits(left: Vec3, right: Vec3) {
    assert_eq!(left.x.to_bits(), right.x.to_bits());
    assert_eq!(left.y.to_bits(), right.y.to_bits());
    assert_eq!(left.z.to_bits(), right.z.to_bits());
}

fn assert_curve_derivs_bits(left: CurveDerivs, right: CurveDerivs) {
    assert_eq!(left.d.len(), right.d.len());
    for (left, right) in left.d.into_iter().zip(right.d) {
        assert_vec_bits(left, right);
    }
}

fn assert_surface_derivs_bits(left: SurfaceDerivs, right: SurfaceDerivs) {
    for (left, right) in [
        (left.p, right.p),
        (left.du, right.du),
        (left.dv, right.dv),
        (left.duu, right.duu),
        (left.duv, right.duv),
        (left.dvv, right.dvv),
    ] {
        assert_vec_bits(left, right);
    }
}

fn assert_float_slices_bits(left: &[f64], right: &[f64]) {
    assert_eq!(left.len(), right.len());
    for (left, right) in left.iter().zip(right) {
        assert_eq!(left.to_bits(), right.to_bits());
    }
}

fn assert_points_bits(left: &[Point3], right: &[Point3]) {
    assert_eq!(left.len(), right.len());
    for (&left, &right) in left.iter().zip(right) {
        assert_vec_bits(left, right);
    }
}

fn assert_curve_bits(left: &NurbsCurve, right: &NurbsCurve) {
    assert_eq!(left.degree(), right.degree());
    assert_float_slices_bits(left.knots().as_slice(), right.knots().as_slice());
    assert_points_bits(left.points(), right.points());
    match (left.weights(), right.weights()) {
        (Some(left), Some(right)) => assert_float_slices_bits(left, right),
        (None, None) => {}
        _ => panic!("repeated curve operation changed rationality"),
    }
}

fn assert_surface_bits(left: &NurbsSurface, right: &NurbsSurface) {
    assert_eq!(left.degree_u(), right.degree_u());
    assert_eq!(left.degree_v(), right.degree_v());
    assert_float_slices_bits(
        left.knots(Dir::U).as_slice(),
        right.knots(Dir::U).as_slice(),
    );
    assert_float_slices_bits(
        left.knots(Dir::V).as_slice(),
        right.knots(Dir::V).as_slice(),
    );
    assert_points_bits(left.points(), right.points());
    match (left.weights(), right.weights()) {
        (Some(left), Some(right)) => assert_float_slices_bits(left, right),
        (None, None) => {}
        _ => panic!("repeated surface operation changed rationality"),
    }
}

fn assert_curve_result(
    left: kcore::error::Result<NurbsCurve>,
    right: kcore::error::Result<NurbsCurve>,
) {
    match (left, right) {
        (Ok(left), Ok(right)) => assert_curve_bits(&left, &right),
        (Err(left), Err(right)) => assert_eq!(left, right),
        _ => panic!("repeated curve operation changed result class"),
    }
}

fn assert_curve_pair_result(
    left: kcore::error::Result<(NurbsCurve, NurbsCurve)>,
    right: kcore::error::Result<(NurbsCurve, NurbsCurve)>,
) {
    match (left, right) {
        (Ok((left_a, left_b)), Ok((right_a, right_b))) => {
            assert_curve_bits(&left_a, &right_a);
            assert_curve_bits(&left_b, &right_b);
        }
        (Err(left), Err(right)) => assert_eq!(left, right),
        _ => panic!("repeated curve split changed result class"),
    }
}

fn assert_surface_result(
    left: kcore::error::Result<NurbsSurface>,
    right: kcore::error::Result<NurbsSurface>,
) {
    match (left, right) {
        (Ok(left), Ok(right)) => assert_surface_bits(&left, &right),
        (Err(left), Err(right)) => assert_eq!(left, right),
        _ => panic!("repeated surface operation changed result class"),
    }
}

fn assert_surface_pair_result(
    left: kcore::error::Result<(NurbsSurface, NurbsSurface)>,
    right: kcore::error::Result<(NurbsSurface, NurbsSurface)>,
) {
    match (left, right) {
        (Ok((left_a, left_b)), Ok((right_a, right_b))) => {
            assert_surface_bits(&left_a, &right_a);
            assert_surface_bits(&left_b, &right_b);
        }
        (Err(left), Err(right)) => assert_eq!(left, right),
        _ => panic!("repeated surface split changed result class"),
    }
}

fn assert_curve_projection_bits(left: Option<CurveProjection>, right: Option<CurveProjection>) {
    match (left, right) {
        (Some(left), Some(right)) => {
            assert_eq!(left.t.to_bits(), right.t.to_bits());
            assert_vec_bits(left.point, right.point);
            assert_eq!(left.dist.to_bits(), right.dist.to_bits());
        }
        (None, None) => {}
        _ => panic!("repeated curve projection changed result class"),
    }
}

fn assert_surface_projection_bits(
    left: Option<SurfaceProjection>,
    right: Option<SurfaceProjection>,
) {
    match (left, right) {
        (Some(left), Some(right)) => {
            assert_eq!(left.uv[0].to_bits(), right.uv[0].to_bits());
            assert_eq!(left.uv[1].to_bits(), right.uv[1].to_bits());
            assert_vec_bits(left.point, right.point);
            assert_eq!(left.dist.to_bits(), right.dist.to_bits());
        }
        (None, None) => {}
        _ => panic!("repeated surface projection changed result class"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CURVE_CORPUS: &[&[u8]] = &[
        include_bytes!("../corpus/nurbs_constructors/curve-polynomial-linear.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/curve-rational-quadratic.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/curve-invalid-degree-zero.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/curve-invalid-nonfinite-point.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/curve-invalid-weight-count.nurbsseed"),
    ];
    const SURFACE_CORPUS: &[&[u8]] = &[
        include_bytes!("../corpus/nurbs_constructors/surface-polynomial-bilinear.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/surface-rational-quadratic.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/surface-invalid-point-count.nurbsseed"),
        include_bytes!("../corpus/nurbs_constructors/surface-invalid-nonfinite-weight.nurbsseed"),
    ];

    fn constructor_accepts(seed: &[u8]) -> bool {
        let Some(descriptor) = decode(seed) else {
            return false;
        };
        if descriptor.is_surface() {
            NurbsSurface::new(
                descriptor.degree_u,
                descriptor.degree_v,
                descriptor.knots_u,
                descriptor.knots_v,
                descriptor.points,
                descriptor.weights,
            )
            .is_ok()
        } else {
            NurbsCurve::new(
                descriptor.degree_u,
                descriptor.knots_u,
                descriptor.points,
                descriptor.weights,
            )
            .is_ok()
        }
    }

    #[test]
    fn curve_and_surface_seed_families_replay_without_panics() {
        for seed in CURVE_CORPUS.iter().chain(SURFACE_CORPUS) {
            exercise(seed);
        }
    }

    #[test]
    fn corpus_families_select_distinct_constructor_shapes() {
        assert!(
            CURVE_CORPUS
                .iter()
                .all(|seed| { decode(seed).is_some_and(|descriptor| !descriptor.is_surface()) })
        );
        assert!(
            SURFACE_CORPUS
                .iter()
                .all(|seed| { decode(seed).is_some_and(|descriptor| descriptor.is_surface()) })
        );
    }

    #[test]
    fn corpus_constructor_expectations_are_explicit() {
        for seed in &CURVE_CORPUS[..2] {
            assert!(constructor_accepts(seed));
        }
        for seed in &CURVE_CORPUS[2..] {
            assert!(!constructor_accepts(seed));
        }
        for seed in &SURFACE_CORPUS[..2] {
            assert!(constructor_accepts(seed));
        }
        for seed in &SURFACE_CORPUS[2..] {
            assert!(!constructor_accepts(seed));
        }
    }

    #[test]
    fn caps_reject_before_descriptor_allocation() {
        exercise(&[]);
        exercise(&vec![0; MAX_INPUT_BYTES + 1]);
        exercise(&[0, (MAX_DEGREE + 1) as u8, 0, 0, 0, 0, 0]);
        exercise(&[0, 1, 0, (MAX_KNOTS_PER_DIRECTION + 1) as u8, 0, 0, 0]);
        exercise(&[0, 1, 0, 4, 0, (MAX_CURVE_POINTS + 1) as u8, 0]);
        exercise(&[
            FAMILY_SURFACE,
            1,
            1,
            4,
            4,
            (MAX_SURFACE_POINTS + 1) as u8,
            0,
        ]);
    }
}
