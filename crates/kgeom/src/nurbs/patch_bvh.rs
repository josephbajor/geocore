//! Conservative BVH and affine half-space certificates for NURBS patches.

use super::NurbsSurface;
use crate::aabb::Aabb3;
use crate::bvh::AabbBvh;
use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::tolerance::LINEAR_RESOLUTION;

/// Certified relation of a positive-weight Bezier patch's control hull to a
/// plane tolerance slab.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanePatchRelation {
    /// The complete patch lies on the negative side beyond the slab.
    Negative,
    /// The control hull meets the slab; the patch requires further work.
    Candidate,
    /// The complete patch lies on the positive side beyond the slab.
    Positive,
}

/// Deterministic hierarchy over the exact Bezier decomposition of one
/// clamped NURBS surface.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsSurfaceBvh {
    patches: Vec<NurbsSurface>,
    hierarchy: AabbBvh,
}

#[derive(Debug, Clone, Copy)]
struct PlaneFilter {
    normal: Vec3,
    scaled_half_width: f64,
}

impl NurbsSurfaceBvh {
    /// Extract the surface's tensor-product Bezier patches and build a
    /// deterministic hierarchy over their positive-weight control hulls.
    pub fn build(surface: &NurbsSurface) -> Result<Self> {
        let patches = surface.to_bezier_patches()?;
        let bounds: Vec<_> = patches
            .iter()
            .map(|patch| Aabb3::from_points(patch.points()))
            .collect();
        let hierarchy = AabbBvh::build(&bounds)?;
        Ok(Self { patches, hierarchy })
    }

    /// Number of Bezier patches, in deterministic source `u`/`v` order.
    pub fn patch_count(&self) -> usize {
        self.patches.len()
    }

    /// Number of nodes in the balanced spatial hierarchy.
    pub fn node_count(&self) -> usize {
        self.hierarchy.node_count()
    }

    /// One extracted patch by deterministic source index.
    pub fn patch(&self, index: usize) -> Option<&NurbsSurface> {
        self.patches.get(index)
    }

    /// Conservative control-hull bound for one patch.
    pub fn patch_bounds(&self, index: usize) -> Option<Aabb3> {
        self.hierarchy.primitive_bounds(index)
    }

    /// Conservative bound containing the whole represented surface.
    pub fn root_bounds(&self) -> Aabb3 {
        self.hierarchy.root_bounds().unwrap_or_else(Aabb3::empty)
    }

    /// Patch indices whose control-hull boxes meet `query` after outward
    /// growth by `margin`.
    pub fn query_aabb(&self, query: Aabb3, margin: f64) -> Result<Vec<usize>> {
        self.hierarchy.query_aabb(query, margin)
    }

    /// Candidate patch pairs whose control-hull boxes are separated by no
    /// more than `max_separation`. Empty is a certified broad-phase miss.
    pub fn overlapping_patch_pairs(
        &self,
        other: &NurbsSurfaceBvh,
        max_separation: f64,
    ) -> Result<Vec<(usize, usize)>> {
        self.hierarchy
            .overlapping_pairs(&other.hierarchy, max_separation)
    }

    /// Classify one patch against the plane through `origin` with the given
    /// normal. `half_width` is a model-space tolerance on each side of the
    /// plane. Normal scale and signed-distance arithmetic are enclosed with
    /// outward-rounded intervals before a side is certified.
    pub fn classify_patch_against_plane(
        &self,
        patch: usize,
        origin: Point3,
        normal: Vec3,
        half_width: f64,
    ) -> Result<PlanePatchRelation> {
        let plane = validate_plane(origin, normal, half_width)?;
        let patch = self.patches.get(patch).ok_or(Error::InvalidGeometry {
            reason: "NURBS BVH patch index is out of range",
        })?;
        Ok(classify_points(
            patch.points(),
            origin,
            plane.normal,
            plane.scaled_half_width,
        ))
    }

    /// Patches whose control hulls meet a plane tolerance slab. Hierarchy
    /// nodes wholly on either side are pruned first, then leaf control nets
    /// provide the tighter affine certificate. Empty proves the complete
    /// surface misses the slab.
    pub fn plane_candidates(
        &self,
        origin: Point3,
        normal: Vec3,
        half_width: f64,
    ) -> Result<Vec<usize>> {
        let plane = validate_plane(origin, normal, half_width)?;
        let broad = self.hierarchy.query_pruned(|bounds| {
            classify_box(bounds, origin, plane.normal, plane.scaled_half_width)
                == PlanePatchRelation::Candidate
        });
        Ok(broad
            .into_iter()
            .filter(|&index| {
                classify_points(
                    self.patches[index].points(),
                    origin,
                    plane.normal,
                    plane.scaled_half_width,
                ) == PlanePatchRelation::Candidate
            })
            .collect())
    }
}

fn validate_plane(origin: Point3, normal: Vec3, half_width: f64) -> Result<PlaneFilter> {
    if !finite_point(origin) || !finite_point(normal) {
        return Err(Error::InvalidGeometry {
            reason: "NURBS patch plane must have finite origin and normal",
        });
    }
    if !half_width.is_finite() || half_width < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "NURBS patch plane half-width must be finite and non-negative",
        });
    }
    let length = normal.norm();
    if !length.is_finite() || length <= LINEAR_RESOLUTION {
        return Err(Error::InvalidGeometry {
            reason: "NURBS patch plane normal is degenerate",
        });
    }
    let nx = Interval::point(normal.x);
    let ny = Interval::point(normal.y);
    let nz = Interval::point(normal.z);
    let norm_squared = nx.square() + ny.square() + nz.square();
    let norm_upper = norm_squared.hi().sqrt().next_up();
    let scaled_half_width = if half_width == 0.0 {
        0.0
    } else {
        (half_width * norm_upper).next_up()
    };
    Ok(PlaneFilter {
        normal,
        scaled_half_width,
    })
}

fn finite_point(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn classify_box(
    bounds: Aabb3,
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> PlanePatchRelation {
    let minimum = Vec3::new(
        if normal.x >= 0.0 {
            bounds.min.x
        } else {
            bounds.max.x
        },
        if normal.y >= 0.0 {
            bounds.min.y
        } else {
            bounds.max.y
        },
        if normal.z >= 0.0 {
            bounds.min.z
        } else {
            bounds.max.z
        },
    );
    let maximum = Vec3::new(
        if normal.x >= 0.0 {
            bounds.max.x
        } else {
            bounds.min.x
        },
        if normal.y >= 0.0 {
            bounds.max.y
        } else {
            bounds.min.y
        },
        if normal.z >= 0.0 {
            bounds.max.z
        } else {
            bounds.min.z
        },
    );
    let lo = signed_distance_interval(minimum, origin, normal);
    let hi = signed_distance_interval(maximum, origin, normal);
    if !lo.lo().is_finite() || !lo.hi().is_finite() || !hi.lo().is_finite() || !hi.hi().is_finite()
    {
        return PlanePatchRelation::Candidate;
    }
    classify_interval(lo.lo(), hi.hi(), half_width)
}

fn classify_points(
    points: &[Point3],
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> PlanePatchRelation {
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    for &point in points {
        let distance = signed_distance_interval(point, origin, normal);
        if !distance.lo().is_finite() || !distance.hi().is_finite() {
            return PlanePatchRelation::Candidate;
        }
        minimum = minimum.min(distance.lo());
        maximum = maximum.max(distance.hi());
    }
    classify_interval(minimum, maximum, half_width)
}

fn signed_distance_interval(point: Point3, origin: Point3, normal: Vec3) -> Interval {
    let dx = Interval::point(point.x) - Interval::point(origin.x);
    let dy = Interval::point(point.y) - Interval::point(origin.y);
    let dz = Interval::point(point.z) - Interval::point(origin.z);
    Interval::point(normal.x) * dx + Interval::point(normal.y) * dy + Interval::point(normal.z) * dz
}

fn classify_interval(minimum: f64, maximum: f64, half_width: f64) -> PlanePatchRelation {
    if maximum < -half_width {
        PlanePatchRelation::Negative
    } else if minimum > half_width {
        PlanePatchRelation::Positive
    } else {
        PlanePatchRelation::Candidate
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::param::ParamRange;
    use crate::surface::Surface;

    fn rational_multi_patch(offset: Vec3) -> NurbsSurface {
        let knots = vec![0.0, 0.0, 0.5, 1.0, 1.0];
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for u in 0..3 {
            for v in 0..3 {
                points
                    .push(Point3::new(f64::from(u), f64::from(v), 0.1 * f64::from(u * v)) + offset);
                weights.push(0.75 + 0.125 * f64::from((u * 3 + v) % 5));
            }
        }
        NurbsSurface::new(1, 1, knots.clone(), knots, points, Some(weights)).unwrap()
    }

    #[test]
    fn hierarchy_preserves_patch_order_and_conservatively_queries() {
        let surface = rational_multi_patch(Vec3::default());
        let first = NurbsSurfaceBvh::build(&surface).unwrap();
        let second = NurbsSurfaceBvh::build(&surface).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.patch_count(), 4);
        assert_eq!(first.node_count(), 7);
        let expected = [
            [ParamRange::new(0.0, 0.5), ParamRange::new(0.0, 0.5)],
            [ParamRange::new(0.0, 0.5), ParamRange::new(0.5, 1.0)],
            [ParamRange::new(0.5, 1.0), ParamRange::new(0.0, 0.5)],
            [ParamRange::new(0.5, 1.0), ParamRange::new(0.5, 1.0)],
        ];
        for (index, range) in expected.into_iter().enumerate() {
            assert_eq!(first.patch(index).unwrap().param_range(), range);
        }

        let query =
            Aabb3::from_points(&[Point3::new(-0.1, -0.1, -1.0), Point3::new(0.9, 0.9, 1.0)]);
        assert_eq!(first.query_aabb(query, 0.0).unwrap(), vec![0]);
        assert!(
            first
                .query_aabb(Aabb3::from_point(Point3::new(20.0, 20.0, 20.0)), 0.0)
                .unwrap()
                .is_empty()
        );
        for i in 0..=20 {
            for j in 0..=20 {
                let point = surface.eval([f64::from(i) / 20.0, f64::from(j) / 20.0]);
                assert!(first.root_bounds().inflated(1.0e-12).contains(point));
            }
        }
    }

    #[test]
    fn pair_broad_phase_has_no_false_negative_and_proves_far_misses() {
        let surface = rational_multi_patch(Vec3::default());
        let a = NurbsSurfaceBvh::build(&surface).unwrap();
        let same = NurbsSurfaceBvh::build(&surface).unwrap();
        let pairs = a.overlapping_patch_pairs(&same, 0.0).unwrap();
        for index in 0..a.patch_count() {
            assert!(pairs.contains(&(index, index)));
        }

        let far = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::new(0.0, 0.0, 10.0))).unwrap();
        assert!(a.overlapping_patch_pairs(&far, 0.0).unwrap().is_empty());
    }

    #[test]
    fn plane_control_hulls_certify_sides_without_sampled_proof() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        let normal = Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(
            hierarchy
                .plane_candidates(Point3::new(0.75, 0.0, 0.0), normal, 0.0)
                .unwrap(),
            vec![0, 1]
        );
        assert_eq!(
            hierarchy
                .plane_candidates(Point3::new(0.75, 0.0, 0.0), normal * 10.0, 0.0)
                .unwrap(),
            vec![0, 1]
        );
        assert_eq!(
            hierarchy
                .classify_patch_against_plane(0, Point3::new(0.75, 0.0, 0.0), normal, 0.0)
                .unwrap(),
            PlanePatchRelation::Candidate
        );
        assert_eq!(
            hierarchy
                .classify_patch_against_plane(2, Point3::new(0.75, 0.0, 0.0), normal, 0.0)
                .unwrap(),
            PlanePatchRelation::Positive
        );
        assert!(
            hierarchy
                .plane_candidates(Point3::new(-1.0, 0.0, 0.0), normal, 0.0)
                .unwrap()
                .is_empty()
        );
        assert!(
            hierarchy
                .plane_candidates(Point3::new(3.0, 0.0, 0.0), normal, 0.0)
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            hierarchy
                .plane_candidates(Point3::new(0.9, 0.0, 0.0), normal, 0.2)
                .unwrap(),
            vec![0, 1, 2, 3]
        );
    }

    #[test]
    fn certified_plane_sides_contain_every_evaluated_patch_point() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        let origin = Point3::new(0.8, 0.7, 0.05);
        let normal = Vec3::new(1.0, -0.3, 0.2).normalized().unwrap();
        let half_width = 0.01;
        for index in 0..hierarchy.patch_count() {
            let patch = hierarchy.patch(index).unwrap();
            let relation = hierarchy
                .classify_patch_against_plane(index, origin, normal, half_width)
                .unwrap();
            let range = patch.param_range();
            for i in 0..=20 {
                for j in 0..=20 {
                    let point = patch.eval([
                        range[0].lerp(f64::from(i) / 20.0),
                        range[1].lerp(f64::from(j) / 20.0),
                    ]);
                    let distance = normal.dot(point - origin);
                    match relation {
                        PlanePatchRelation::Negative => assert!(distance < -half_width),
                        PlanePatchRelation::Positive => assert!(distance > half_width),
                        PlanePatchRelation::Candidate => {}
                    }
                }
            }
        }
    }

    #[test]
    fn invalid_plane_and_unclamped_surface_are_rejected() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        assert!(
            hierarchy
                .plane_candidates(Point3::default(), Vec3::default(), 0.0)
                .is_err()
        );
        assert!(
            hierarchy
                .plane_candidates(Point3::default(), Vec3::new(1.0, 0.0, 0.0), f64::NAN)
                .is_err()
        );
        assert!(
            hierarchy
                .classify_patch_against_plane(
                    100,
                    Point3::default(),
                    Vec3::new(1.0, 0.0, 0.0),
                    0.0,
                )
                .is_err()
        );

        let unclamped = NurbsSurface::new(
            1,
            1,
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::default(); 4],
            None,
        )
        .unwrap();
        assert!(NurbsSurfaceBvh::build(&unclamped).is_err());
    }
}
