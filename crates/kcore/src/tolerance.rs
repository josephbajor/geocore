//! The session numeric regime and tolerance policy.
//!
//! The kernel adopts Parasolid's numeric regime verbatim so that everything
//! we author is expressible in XT and passes the Parasolid checker:
//!
//! - model space is SI **meters**;
//! - all geometry lives in a **size box** of 1000 m (coordinates within
//!   ±500 m of the origin);
//! - two points closer than the **linear resolution** (1e-8 m) are
//!   coincident;
//! - two directions within the **angular resolution** (1e-11 rad) are
//!   parallel.
//!
//! Tolerance *decisions* belong here, not scattered through operation code.
//! Higher layers receive a [`Tolerances`] value and ask it questions; they
//! never compare against literals. Tolerant entities (edges/vertices carrying
//! their own tolerance ≥ resolution, as in Parasolid tolerant modeling) build
//! on this via [`Tolerances::entity_tolerance`].

use crate::error::{Error, Result};

/// Linear session resolution in meters (Parasolid-compatible).
pub const LINEAR_RESOLUTION: f64 = 1e-8;

/// Angular session resolution in radians (Parasolid-compatible).
pub const ANGULAR_RESOLUTION: f64 = 1e-11;

/// Half-extent of the size box: every coordinate must lie in
/// `[-SIZE_BOX_HALF, SIZE_BOX_HALF]` meters.
pub const SIZE_BOX_HALF: f64 = 500.0;

/// The tolerance policy threaded through all kernel operations.
///
/// Defaults to session resolution; operations that need looser working
/// tolerances (e.g. sewing imported geometry) construct a variant with
/// [`Tolerances::with_linear`], which validates against the floor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerances {
    linear: f64,
    angular: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Tolerances {
            linear: LINEAR_RESOLUTION,
            angular: ANGULAR_RESOLUTION,
        }
    }
}

impl Tolerances {
    /// Policy with a custom linear tolerance (must be ≥ session resolution
    /// and finite).
    pub fn with_linear(linear: f64) -> Result<Self> {
        if !linear.is_finite() || linear < LINEAR_RESOLUTION {
            return Err(Error::InvalidTolerance { value: linear });
        }
        Ok(Tolerances {
            linear,
            angular: ANGULAR_RESOLUTION,
        })
    }

    /// Active linear tolerance in meters.
    pub fn linear(self) -> f64 {
        self.linear
    }

    /// Active angular tolerance in radians.
    pub fn angular(self) -> f64 {
        self.angular
    }

    /// True if a length is indistinguishable from zero.
    pub fn zero_length(self, d: f64) -> bool {
        d.abs() <= self.linear
    }

    /// True if two lengths are indistinguishable.
    pub fn same_length(self, a: f64, b: f64) -> bool {
        self.zero_length(a - b)
    }

    /// True if an angle (radians) is indistinguishable from zero.
    pub fn zero_angle(self, r: f64) -> bool {
        r.abs() <= self.angular
    }

    /// True if two 3D points are coincident under the linear tolerance
    /// (component-wise distance bound, then exact squared distance).
    pub fn same_point(self, p: [f64; 3], q: [f64; 3]) -> bool {
        let dx = p[0] - q[0];
        let dy = p[1] - q[1];
        let dz = p[2] - q[2];
        dx * dx + dy * dy + dz * dz <= self.linear * self.linear
    }

    /// Validate a per-entity tolerance for a tolerant edge/vertex: it must be
    /// finite and at least the session linear resolution.
    pub fn entity_tolerance(self, tol: f64) -> Result<f64> {
        if !tol.is_finite() || tol < LINEAR_RESOLUTION {
            return Err(Error::InvalidTolerance { value: tol });
        }
        Ok(tol)
    }
}

/// Validate that a point lies inside the size box.
pub fn check_in_size_box(p: [f64; 3]) -> Result<()> {
    for &coordinate in &p {
        if !coordinate.is_finite() || coordinate.abs() > SIZE_BOX_HALF {
            return Err(Error::OutsideSizeBox { coordinate });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_parasolid_regime() {
        let t = Tolerances::default();
        assert_eq!(t.linear(), 1e-8);
        assert_eq!(t.angular(), 1e-11);
    }

    #[test]
    fn coincidence_at_resolution_boundary() {
        let t = Tolerances::default();
        assert!(t.same_point([0.0, 0.0, 0.0], [1e-8, 0.0, 0.0]));
        assert!(!t.same_point([0.0, 0.0, 0.0], [2e-8, 0.0, 0.0]));
    }

    #[test]
    fn custom_linear_tolerance_floors_at_resolution() {
        assert!(Tolerances::with_linear(1e-6).is_ok());
        assert!(Tolerances::with_linear(1e-9).is_err());
        assert!(Tolerances::with_linear(f64::NAN).is_err());
    }

    #[test]
    fn size_box_enforced() {
        assert!(check_in_size_box([499.9, -499.9, 0.0]).is_ok());
        assert!(check_in_size_box([500.1, 0.0, 0.0]).is_err());
        assert!(check_in_size_box([f64::INFINITY, 0.0, 0.0]).is_err());
        assert!(check_in_size_box([f64::NAN, 0.0, 0.0]).is_err());
    }
}
