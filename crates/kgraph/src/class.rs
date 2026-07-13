//! Stable geometry class identity.

use core::fmt;

/// A stable, namespaced geometry class identifier.
///
/// The string is the persistence/interchange contract; Rust discriminants and
/// debug formatting are deliberately not used as class identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GeometryClassKey(&'static str);

impl GeometryClassKey {
    const fn new(key: &'static str) -> Self {
        Self(key)
    }

    /// The stable string representation.
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}

impl fmt::Display for GeometryClassKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}

/// Exact 3D curve class used for closed internal dispatch.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CurveClass {
    /// Straight line.
    Line,
    /// Circle.
    Circle,
    /// Ellipse.
    Ellipse,
    /// B-spline or NURBS curve.
    Nurbs,
    /// Graph-owned verified intersection curve.
    Intersection,
}

impl CurveClass {
    /// Stable external class key.
    pub const fn key(self) -> GeometryClassKey {
        GeometryClassKey::new(match self {
            Self::Line => "kernel.curve.line.v1",
            Self::Circle => "kernel.curve.circle.v1",
            Self::Ellipse => "kernel.curve.ellipse.v1",
            Self::Nurbs => "kernel.curve.nurbs.v1",
            Self::Intersection => "kernel.curve.intersection.v1",
        })
    }
}

/// Exact surface class used for closed internal dispatch.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SurfaceClass {
    /// Plane.
    Plane,
    /// Right circular cylinder.
    Cylinder,
    /// Right circular cone.
    Cone,
    /// Sphere.
    Sphere,
    /// Ring torus.
    Torus,
    /// Tensor-product B-spline or NURBS surface.
    Nurbs,
    /// Constant signed true-normal offset of another graph surface.
    Offset,
}

impl SurfaceClass {
    /// Stable external class key.
    pub const fn key(self) -> GeometryClassKey {
        GeometryClassKey::new(match self {
            Self::Plane => "kernel.surface.plane.v1",
            Self::Cylinder => "kernel.surface.cylinder.v1",
            Self::Cone => "kernel.surface.cone.v1",
            Self::Sphere => "kernel.surface.sphere.v1",
            Self::Torus => "kernel.surface.torus.v1",
            Self::Nurbs => "kernel.surface.nurbs.v1",
            Self::Offset => "kernel.surface.offset.v1",
        })
    }
}

/// Exact parameter-space curve class used for closed internal dispatch.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Curve2dClass {
    /// Straight line.
    Line,
    /// Circle.
    Circle,
    /// B-spline or NURBS curve.
    Nurbs,
    /// Finite certifier-minted inverse sphere chart of a spatial circle.
    SphericalCircle,
}

impl Curve2dClass {
    /// Stable external class key.
    pub const fn key(self) -> GeometryClassKey {
        GeometryClassKey::new(match self {
            Self::Line => "kernel.curve2d.line.v1",
            Self::Circle => "kernel.curve2d.circle.v1",
            Self::Nurbs => "kernel.curve2d.nurbs.v1",
            Self::SphericalCircle => "kernel.curve2d.spherical-circle.v1",
        })
    }
}
