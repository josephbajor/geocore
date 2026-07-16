//! Orthonormal axis frames.
//!
//! Every analytic curve and surface is positioned by a [`Frame`]: an origin
//! plus a right-handed orthonormal basis. This mirrors XT's
//! position/direction fields on analytic geometry and keeps parameterization
//! formulas uniform (`cos u · X + sin u · Y` everywhere).

use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};
use kcore::tolerance::{ANGULAR_RESOLUTION, LINEAR_RESOLUTION};

/// An origin with a right-handed orthonormal basis `(x, y, z)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    origin: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl Frame {
    /// The world frame at the model origin.
    pub fn world() -> Frame {
        Frame {
            origin: Point3::new(0.0, 0.0, 0.0),
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, 1.0, 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        }
    }

    /// Frame from an origin, a `z` direction, and an `x` hint.
    ///
    /// `z` is normalized; the component of `x_hint` parallel to `z` is
    /// removed before normalizing; `y = z × x` completes the right-handed
    /// basis. Fails if either input is zero-length or the hint is parallel
    /// to `z` (within session tolerance).
    pub fn new(origin: Point3, z: Vec3, x_hint: Vec3) -> Result<Frame> {
        let z = z.normalized().ok_or(Error::InvalidGeometry {
            reason: "frame z axis has zero length",
        })?;
        let projected = x_hint - z * x_hint.dot(z);
        if projected.x.is_finite()
            && projected.y.is_finite()
            && projected.z.is_finite()
            && let Some(x) = projected.normalized()
        {
            let frame = Frame {
                origin,
                x,
                y: z.cross(x),
                z,
            };
            if frame.is_orthonormal() {
                return Ok(frame);
            }
        }

        let x = if x_hint.x.is_finite() && x_hint.y.is_finite() && x_hint.z.is_finite() {
            // The ordinary path either overflowed or was too ill-conditioned
            // to satisfy the stored orthonormal-frame contract. Homogeneous
            // scaling plus the cross/cross projection keeps every
            // intermediate bounded and avoids near-parallel subtraction.
            let scale = x_hint.x.abs().max(x_hint.y.abs()).max(x_hint.z.abs());
            let scaled_hint = x_hint / scale;
            let scaled_projected = z.cross(scaled_hint).cross(z);
            let scaled_length = scaled_projected.norm();
            let degeneracy_floor =
                (LINEAR_RESOLUTION / scale).max(ANGULAR_RESOLUTION * scaled_hint.norm());
            (scaled_length > degeneracy_floor).then(|| scaled_projected / scaled_length)
        } else {
            None
        }
        .ok_or(Error::InvalidGeometry {
            reason: "frame x hint is parallel to z axis",
        })?;
        let frame = Frame {
            origin,
            x,
            y: z.cross(x),
            z,
        };
        frame
            .is_orthonormal()
            .then_some(frame)
            .ok_or(Error::InvalidGeometry {
                reason: "frame x hint is parallel to z axis",
            })
    }

    /// Frame from an origin and a `z` direction, with `x` chosen
    /// deterministically (the world axis least aligned with `z`, projected).
    pub fn from_z(origin: Point3, z: Vec3) -> Result<Frame> {
        let zn = z.normalized().ok_or(Error::InvalidGeometry {
            reason: "frame z axis has zero length",
        })?;
        // Pick the world axis with the smallest |component| in z; ties break
        // toward x then y for determinism.
        let ax = zn.x.abs();
        let ay = zn.y.abs();
        let az = zn.z.abs();
        let hint = if ax <= ay && ax <= az {
            Vec3::new(1.0, 0.0, 0.0)
        } else if ay <= az {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        Frame::new(origin, zn, hint)
    }

    /// Return this exact orientation at a different origin.
    ///
    /// Unlike [`Frame::new`], this does not renormalize already validated axes.
    pub const fn with_origin(self, origin: Point3) -> Frame {
        Frame { origin, ..self }
    }

    /// Frame origin.
    pub fn origin(&self) -> Point3 {
        self.origin
    }
    /// Unit x axis.
    pub fn x(&self) -> Vec3 {
        self.x
    }
    /// Unit y axis.
    pub fn y(&self) -> Vec3 {
        self.y
    }
    /// Unit z axis.
    pub fn z(&self) -> Vec3 {
        self.z
    }

    /// Map local coordinates to model space.
    pub fn point_at(&self, u: f64, v: f64, w: f64) -> Point3 {
        self.origin + self.x * u + self.y * v + self.z * w
    }

    /// Express a model-space point in local coordinates.
    pub fn to_local(&self, p: Point3) -> Vec3 {
        let d = p - self.origin;
        Vec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    /// Debug-check orthonormality (used by conformance tests).
    pub fn is_orthonormal(&self) -> bool {
        let unit = |v: Vec3| (v.norm() - 1.0).abs() < 16.0 * ANGULAR_RESOLUTION;
        unit(self.x)
            && unit(self.y)
            && unit(self.z)
            && self.x.dot(self.y).abs() < 16.0 * ANGULAR_RESOLUTION
            && self.y.dot(self.z).abs() < 16.0 * ANGULAR_RESOLUTION
            && self.z.dot(self.x).abs() < 16.0 * ANGULAR_RESOLUTION
            && (self.x.cross(self.y) - self.z).norm() < 16.0 * ANGULAR_RESOLUTION
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn construction_orthonormalizes_hint() {
        let f = Frame::new(
            Point3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 0.0, 2.0),
            Vec3::new(1.0, 1.0, 5.0), // z-parallel part must be stripped
        )
        .unwrap();
        assert!(f.is_orthonormal());
        assert_eq!(f.z(), Vec3::new(0.0, 0.0, 1.0));
        assert!((f.x() - Vec3::new(1.0, 1.0, 0.0).normalized().unwrap()).norm() < 1e-15);
    }

    #[test]
    fn degenerate_inputs_rejected() {
        let o = Point3::new(0.0, 0.0, 0.0);
        assert!(Frame::new(o, Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).is_err());
        assert!(Frame::new(o, Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 3.0)).is_err());
        assert!(
            Frame::new(
                o,
                Vec3::new(f64::INFINITY, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0)
            )
            .is_err()
        );
    }

    #[test]
    fn from_z_is_deterministic_and_valid() {
        for z in [
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-0.3, 0.9, 0.1),
        ] {
            let f = Frame::from_z(Point3::new(0.0, 0.0, 0.0), z).unwrap();
            assert!(f.is_orthonormal(), "z = {z:?}");
        }
    }

    #[test]
    fn local_roundtrip() {
        let f = Frame::new(
            Point3::new(5.0, -1.0, 2.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        let p = f.point_at(0.5, -2.0, 3.5);
        let l = f.to_local(p);
        assert!((l - Vec3::new(0.5, -2.0, 3.5)).norm() < 1e-12);
    }

    #[test]
    fn extreme_finite_axes_preserve_valid_frame_construction() {
        let origin = Point3::new(1.0, -2.0, 3.0);
        let base = Frame::new(origin, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap();
        let huge = Frame::new(
            origin,
            Vec3::new(1.0e308, 0.0, 0.0),
            Vec3::new(0.0, f64::MAX, 0.0),
        )
        .unwrap();
        assert_eq!(huge, base);
        assert!(huge.is_orthonormal());

        assert_eq!(
            Frame::from_z(origin, Vec3::new(1.0e308, 0.0, 0.0)).unwrap(),
            Frame::from_z(origin, Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        );
    }

    #[test]
    fn projection_overflow_retries_homogeneously_without_accepting_parallel_hints() {
        let origin = Point3::default();
        let z = Vec3::new(1.0, 1.0, 1.0);
        let frame = Frame::new(origin, z, Vec3::new(f64::MAX, f64::MAX, 0.0)).unwrap();
        assert!(frame.is_orthonormal());

        assert!(Frame::new(origin, z, Vec3::new(f64::MAX, f64::MAX, f64::MAX),).is_err());
    }

    #[test]
    fn projection_overflow_preserves_scale_aware_near_parallel_hints() {
        let origin = Point3::default();
        let z = Vec3::new(1.0, 1.0, 1.0);
        let epsilon = 1.0e-9;
        let moderate_scale = 1.0e4;
        let moderate = Frame::new(
            origin,
            z,
            Vec3::new(
                moderate_scale,
                moderate_scale,
                (1.0 - epsilon) * moderate_scale,
            ),
        )
        .unwrap();
        let huge = Frame::new(
            origin,
            z,
            Vec3::new(f64::MAX, f64::MAX, (1.0 - epsilon) * f64::MAX),
        )
        .unwrap();

        assert!(moderate.is_orthonormal());
        assert!(huge.is_orthonormal());
        assert!(moderate.x().dot(huge.x()) > 1.0 - 1.0e-10);
    }
}
