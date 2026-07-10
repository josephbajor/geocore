//! Certified interval filters for implicit analytic surfaces.
//!
//! The zero set of an [`ImplicitSurface`] contains the represented analytic
//! surface. Implementations may deliberately contain extra sheets (the cone
//! field does) because that can only retain false-positive candidates; it
//! cannot create a false-negative exclusion.

use crate::aabb::Aabb3;
use crate::frame::Frame;
use crate::surface::{Cone, Cylinder, Plane, Sphere, Torus};
use crate::vec::Vec3;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;

/// Certified sign relation of an axis-aligned box to an implicit zero set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImplicitBoxRelation {
    /// The field is strictly negative throughout the box.
    Negative,
    /// The field interval contains zero or arithmetic was inconclusive.
    Candidate,
    /// The field is strictly positive throughout the box.
    Positive,
}

/// An analytic surface represented by a conservative interval implicit field.
///
/// For every point on the represented surface, the mathematical field value
/// must be zero. [`ImplicitSurface::implicit_interval`] must enclose the field
/// at every point of the supplied finite box. A field may have additional
/// zeros; these reduce pruning efficiency but preserve soundness.
pub trait ImplicitSurface {
    /// Conservative range of the implicit field throughout `bounds`.
    fn implicit_interval(&self, bounds: Aabb3) -> Interval;
}

/// Classify `bounds` against an implicit surface after outward growth by a
/// model-space `margin`.
///
/// Inflating the box before evaluating the field gives `margin` geometric
/// meaning without converting length tolerance into surface-specific
/// polynomial residual units. If this returns [`ImplicitBoxRelation::Negative`]
/// or [`ImplicitBoxRelation::Positive`], every point of the original box is
/// farther than `margin` (in the axis-aligned L-infinity sense, and therefore
/// not within Euclidean distance `margin`) from the represented zero set.
pub fn classify_implicit_box(
    surface: &dyn ImplicitSurface,
    bounds: Aabb3,
    margin: f64,
) -> Result<ImplicitBoxRelation> {
    if bounds.is_empty() || !bounds.is_finite() {
        return Err(Error::InvalidGeometry {
            reason: "implicit-surface bounds must be finite and non-empty",
        });
    }
    if !margin.is_finite() || margin < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "implicit-surface margin must be finite and non-negative",
        });
    }

    let expanded = bounds.inflated(margin);
    if !expanded.is_finite() {
        return Ok(ImplicitBoxRelation::Candidate);
    }
    Ok(classify_interval(surface.implicit_interval(expanded)))
}

impl ImplicitSurface for Plane {
    fn implicit_interval(&self, bounds: Aabb3) -> Interval {
        local_coordinate(bounds, self.frame(), self.frame().z())
    }
}

impl ImplicitSurface for Sphere {
    fn implicit_interval(&self, bounds: Aabb3) -> Interval {
        let [x, y, z] = local_box(bounds, self.frame());
        x.square() + y.square() + z.square() - Interval::point(self.radius()).square()
    }
}

impl ImplicitSurface for Cylinder {
    fn implicit_interval(&self, bounds: Aabb3) -> Interval {
        let [x, y, _] = local_box(bounds, self.frame());
        x.square() + y.square() - Interval::point(self.radius()).square()
    }
}

impl ImplicitSurface for Cone {
    fn implicit_interval(&self, bounds: Aabb3) -> Interval {
        let [x, y, z] = local_box(bounds, self.frame());
        let (sin, cos) = math::sincos(self.half_angle());
        let sin = Interval::point(sin);
        let cos = Interval::point(cos);
        let radius = Interval::point(self.radius());

        // The unsquared signed equation is
        //   cos(a) * radial = cos(a) * radius + sin(a) * z.
        // Squaring avoids an interval square root. It adds the reflected cone
        // sheet, which is conservative for exclusion.
        cos.square() * (x.square() + y.square()) - (cos * radius + sin * z).square()
    }
}

impl ImplicitSurface for Torus {
    fn implicit_interval(&self, bounds: Aabb3) -> Interval {
        let [x, y, z] = local_box(bounds, self.frame());
        let radial_squared = x.square() + y.square();
        let radial = radial_squared
            .sqrt()
            .unwrap_or_else(|| Interval::new(f64::NEG_INFINITY, f64::INFINITY));
        let radial_offset = radial - Interval::point(self.major_radius());
        let tube_squared = Interval::point(self.minor_radius()).square();
        radial_offset.square() + z.square() - tube_squared
    }
}

fn local_box(bounds: Aabb3, frame: &Frame) -> [Interval; 3] {
    [
        local_coordinate(bounds, frame, frame.x()),
        local_coordinate(bounds, frame, frame.y()),
        local_coordinate(bounds, frame, frame.z()),
    ]
}

fn local_coordinate(bounds: Aabb3, frame: &Frame, axis: Vec3) -> Interval {
    let x = Interval::new(bounds.min.x, bounds.max.x) - Interval::point(frame.origin().x);
    let y = Interval::new(bounds.min.y, bounds.max.y) - Interval::point(frame.origin().y);
    let z = Interval::new(bounds.min.z, bounds.max.z) - Interval::point(frame.origin().z);
    Interval::point(axis.x) * x + Interval::point(axis.y) * y + Interval::point(axis.z) * z
}

fn classify_interval(range: Interval) -> ImplicitBoxRelation {
    if !range.lo().is_finite() || !range.hi().is_finite() || range.lo() > range.hi() {
        ImplicitBoxRelation::Candidate
    } else if range.hi() < 0.0 {
        ImplicitBoxRelation::Negative
    } else if range.lo() > 0.0 {
        ImplicitBoxRelation::Positive
    } else {
        ImplicitBoxRelation::Candidate
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::Frame;
    use crate::surface::Surface;
    use crate::vec::Point3;

    fn point(x: f64, y: f64, z: f64) -> Aabb3 {
        Aabb3::from_point(Point3::new(x, y, z))
    }

    #[test]
    fn analytic_fields_certify_inside_outside_and_zero_boxes() {
        let frame = Frame::world();
        let plane = Plane::new(frame);
        assert_eq!(
            classify_implicit_box(&plane, point(0.0, 0.0, -1.0), 0.0).unwrap(),
            ImplicitBoxRelation::Negative
        );
        assert_eq!(
            classify_implicit_box(&plane, point(0.0, 0.0, 1.0), 0.0).unwrap(),
            ImplicitBoxRelation::Positive
        );
        assert_eq!(
            classify_implicit_box(&plane, point(0.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Candidate
        );

        let sphere = Sphere::new(frame, 2.0).unwrap();
        assert_eq!(
            classify_implicit_box(&sphere, point(0.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Negative
        );
        assert_eq!(
            classify_implicit_box(&sphere, point(3.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Positive
        );
        assert_eq!(
            classify_implicit_box(&sphere, point(2.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Candidate
        );

        let cylinder = Cylinder::new(frame, 2.0).unwrap();
        assert_eq!(
            classify_implicit_box(&cylinder, point(0.0, 0.0, 20.0), 0.0).unwrap(),
            ImplicitBoxRelation::Negative
        );
        assert_eq!(
            classify_implicit_box(&cylinder, point(3.0, 0.0, -20.0), 0.0).unwrap(),
            ImplicitBoxRelation::Positive
        );
        assert_eq!(
            classify_implicit_box(&cylinder, point(2.0, 0.0, 5.0), 0.0).unwrap(),
            ImplicitBoxRelation::Candidate
        );

        let cone = Cone::new(frame, 1.0, core::f64::consts::FRAC_PI_4).unwrap();
        assert_eq!(
            classify_implicit_box(&cone, point(0.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Negative
        );
        assert_eq!(
            classify_implicit_box(&cone, point(2.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Positive
        );
        assert_eq!(
            classify_implicit_box(&cone, point(1.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Candidate
        );

        let torus = Torus::new(frame, 3.0, 1.0).unwrap();
        assert_eq!(
            classify_implicit_box(&torus, point(0.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Positive
        );
        assert_eq!(
            classify_implicit_box(&torus, point(3.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Negative
        );
        assert_eq!(
            classify_implicit_box(&torus, point(4.0, 0.0, 0.0), 0.0).unwrap(),
            ImplicitBoxRelation::Candidate
        );
    }

    #[test]
    fn model_space_margin_retains_nearby_zero_sets() {
        let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
        assert_eq!(
            classify_implicit_box(&sphere, point(3.0, 0.0, 0.0), 0.999).unwrap(),
            ImplicitBoxRelation::Positive
        );
        assert_eq!(
            classify_implicit_box(&sphere, point(3.0, 0.0, 0.0), 1.0).unwrap(),
            ImplicitBoxRelation::Candidate
        );
    }

    #[test]
    fn tilted_surface_samples_are_never_excluded_with_resolution_padding() {
        let frame = Frame::new(
            Point3::new(0.5, -1.0, 2.0),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(-1.0, 1.0, 0.5),
        )
        .unwrap();
        let sphere = Sphere::new(frame, 1.75).unwrap();
        let cylinder = Cylinder::new(frame, 1.25).unwrap();
        let cone = Cone::new(frame, 0.75, 0.4).unwrap();
        let torus = Torus::new(frame, 2.5, 0.6).unwrap();

        for &(surface, uv) in &[
            (&sphere as &dyn Surface, [1.1, 0.3]),
            (&cylinder as &dyn Surface, [2.2, -3.0]),
            (&cone as &dyn Surface, [0.7, 1.4]),
            (&torus as &dyn Surface, [1.8, 4.1]),
        ] {
            let sample = surface.eval(uv);
            let implicit: &dyn ImplicitSurface = if surface.as_any().is::<Sphere>() {
                &sphere
            } else if surface.as_any().is::<Cylinder>() {
                &cylinder
            } else if surface.as_any().is::<Cone>() {
                &cone
            } else {
                &torus
            };
            assert_eq!(
                classify_implicit_box(implicit, Aabb3::from_point(sample), 1.0e-12).unwrap(),
                ImplicitBoxRelation::Candidate
            );
        }
    }

    #[test]
    fn invalid_boxes_and_margins_are_typed_errors() {
        let plane = Plane::new(Frame::world());
        assert!(classify_implicit_box(&plane, Aabb3::empty(), 0.0).is_err());
        assert!(classify_implicit_box(&plane, point(0.0, 0.0, 0.0), -1.0).is_err());
        assert!(classify_implicit_box(&plane, point(0.0, 0.0, 0.0), f64::NAN).is_err());
    }

    #[test]
    fn analytic_field_intervals_enclose_nested_point_boxes() {
        let frame = Frame::new(
            Point3::new(0.5, -1.0, 2.0),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(-1.0, 1.0, 0.5),
        )
        .unwrap();
        let plane = Plane::new(frame);
        let sphere = Sphere::new(frame, 1.75).unwrap();
        let cylinder = Cylinder::new(frame, 1.25).unwrap();
        let cone = Cone::new(frame, 0.75, 0.4).unwrap();
        let torus = Torus::new(frame, 2.5, 0.6).unwrap();
        let fields: [&dyn ImplicitSurface; 5] = [&plane, &sphere, &cylinder, &cone, &torus];

        let mut state = 0xD1B5_4A32_D192_ED03_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state as f64 / u64::MAX as f64
        };
        for _ in 0..256 {
            let min = Point3::new(
                -4.0 + 8.0 * next(),
                -4.0 + 8.0 * next(),
                -4.0 + 8.0 * next(),
            );
            let max = min
                + Vec3::new(
                    0.01 + 2.0 * next(),
                    0.01 + 2.0 * next(),
                    0.01 + 2.0 * next(),
                );
            let bounds = Aabb3::from_points(&[min, max]);
            let samples = [
                min,
                max,
                Point3::new(
                    0.5 * (min.x + max.x),
                    0.5 * (min.y + max.y),
                    0.5 * (min.z + max.z),
                ),
            ];
            for field in fields {
                let outer = field.implicit_interval(bounds);
                for sample in samples {
                    let inner = field.implicit_interval(Aabb3::from_point(sample));
                    assert!(outer.lo() <= inner.lo());
                    assert!(inner.hi() <= outer.hi());
                }
            }
        }
    }
}
