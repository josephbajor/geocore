//! Immutable leaf descriptors and dependency reporting.

use kgeom::curve::{Circle, Ellipse, Line};
use kgeom::curve2d::{Circle2d, Line2d, NurbsCurve2d};
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Torus};

use crate::class::{Curve2dClass, CurveClass, GeometryClassKey, SurfaceClass};
use crate::graph::GeometryRef;

/// A 3D curve descriptor.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum CurveDescriptor {
    /// Straight line.
    Line(Line),
    /// Circle.
    Circle(Circle),
    /// Ellipse.
    Ellipse(Ellipse),
    /// B-spline or NURBS curve.
    Nurbs(NurbsCurve),
}

impl CurveDescriptor {
    /// Exact class of this descriptor.
    pub const fn class(&self) -> CurveClass {
        match self {
            Self::Line(_) => CurveClass::Line,
            Self::Circle(_) => CurveClass::Circle,
            Self::Ellipse(_) => CurveClass::Ellipse,
            Self::Nurbs(_) => CurveClass::Nurbs,
        }
    }

    /// Stable external class key.
    pub const fn class_key(&self) -> GeometryClassKey {
        self.class().key()
    }

    /// Borrow this descriptor as a line when its class matches.
    pub const fn as_line(&self) -> Option<&Line> {
        if let Self::Line(value) = self {
            Some(value)
        } else {
            None
        }
    }

    /// Borrow this descriptor as a circle when its class matches.
    pub const fn as_circle(&self) -> Option<&Circle> {
        if let Self::Circle(value) = self {
            Some(value)
        } else {
            None
        }
    }

    /// Borrow this descriptor as an ellipse when its class matches.
    pub const fn as_ellipse(&self) -> Option<&Ellipse> {
        if let Self::Ellipse(value) = self {
            Some(value)
        } else {
            None
        }
    }

    /// Borrow this descriptor as a NURBS curve when its class matches.
    pub const fn as_nurbs(&self) -> Option<&NurbsCurve> {
        if let Self::Nurbs(value) = self {
            Some(value)
        } else {
            None
        }
    }
}

impl From<Line> for CurveDescriptor {
    fn from(value: Line) -> Self {
        Self::Line(value)
    }
}
impl From<Circle> for CurveDescriptor {
    fn from(value: Circle) -> Self {
        Self::Circle(value)
    }
}
impl From<Ellipse> for CurveDescriptor {
    fn from(value: Ellipse) -> Self {
        Self::Ellipse(value)
    }
}
impl From<NurbsCurve> for CurveDescriptor {
    fn from(value: NurbsCurve) -> Self {
        Self::Nurbs(value)
    }
}

/// A surface descriptor.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceDescriptor {
    /// Plane.
    Plane(Plane),
    /// Cylinder.
    Cylinder(Cylinder),
    /// Cone.
    Cone(Cone),
    /// Sphere.
    Sphere(Sphere),
    /// Torus.
    Torus(Torus),
    /// Tensor-product B-spline or NURBS surface.
    Nurbs(NurbsSurface),
}

impl SurfaceDescriptor {
    /// Exact class of this descriptor.
    pub const fn class(&self) -> SurfaceClass {
        match self {
            Self::Plane(_) => SurfaceClass::Plane,
            Self::Cylinder(_) => SurfaceClass::Cylinder,
            Self::Cone(_) => SurfaceClass::Cone,
            Self::Sphere(_) => SurfaceClass::Sphere,
            Self::Torus(_) => SurfaceClass::Torus,
            Self::Nurbs(_) => SurfaceClass::Nurbs,
        }
    }

    /// Stable external class key.
    pub const fn class_key(&self) -> GeometryClassKey {
        self.class().key()
    }

    /// Borrow as a plane when the class matches.
    pub const fn as_plane(&self) -> Option<&Plane> {
        if let Self::Plane(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a cylinder when the class matches.
    pub const fn as_cylinder(&self) -> Option<&Cylinder> {
        if let Self::Cylinder(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a cone when the class matches.
    pub const fn as_cone(&self) -> Option<&Cone> {
        if let Self::Cone(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a sphere when the class matches.
    pub const fn as_sphere(&self) -> Option<&Sphere> {
        if let Self::Sphere(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a torus when the class matches.
    pub const fn as_torus(&self) -> Option<&Torus> {
        if let Self::Torus(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a NURBS surface when the class matches.
    pub const fn as_nurbs(&self) -> Option<&NurbsSurface> {
        if let Self::Nurbs(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<Plane> for SurfaceDescriptor {
    fn from(v: Plane) -> Self {
        Self::Plane(v)
    }
}
impl From<Cylinder> for SurfaceDescriptor {
    fn from(v: Cylinder) -> Self {
        Self::Cylinder(v)
    }
}
impl From<Cone> for SurfaceDescriptor {
    fn from(v: Cone) -> Self {
        Self::Cone(v)
    }
}
impl From<Sphere> for SurfaceDescriptor {
    fn from(v: Sphere) -> Self {
        Self::Sphere(v)
    }
}
impl From<Torus> for SurfaceDescriptor {
    fn from(v: Torus) -> Self {
        Self::Torus(v)
    }
}
impl From<NurbsSurface> for SurfaceDescriptor {
    fn from(v: NurbsSurface) -> Self {
        Self::Nurbs(v)
    }
}

/// A two-dimensional parameter-space curve descriptor.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Curve2dDescriptor {
    /// Straight line.
    Line(Line2d),
    /// Circle.
    Circle(Circle2d),
    /// B-spline or NURBS curve.
    Nurbs(NurbsCurve2d),
}

impl Curve2dDescriptor {
    /// Exact class of this descriptor.
    pub const fn class(&self) -> Curve2dClass {
        match self {
            Self::Line(_) => Curve2dClass::Line,
            Self::Circle(_) => Curve2dClass::Circle,
            Self::Nurbs(_) => Curve2dClass::Nurbs,
        }
    }
    /// Stable external class key.
    pub const fn class_key(&self) -> GeometryClassKey {
        self.class().key()
    }
    /// Borrow as a line when the class matches.
    pub const fn as_line(&self) -> Option<&Line2d> {
        if let Self::Line(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a circle when the class matches.
    pub const fn as_circle(&self) -> Option<&Circle2d> {
        if let Self::Circle(v) = self {
            Some(v)
        } else {
            None
        }
    }
    /// Borrow as a NURBS curve when the class matches.
    pub const fn as_nurbs(&self) -> Option<&NurbsCurve2d> {
        if let Self::Nurbs(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<Line2d> for Curve2dDescriptor {
    fn from(v: Line2d) -> Self {
        Self::Line(v)
    }
}
impl From<Circle2d> for Curve2dDescriptor {
    fn from(v: Circle2d) -> Self {
        Self::Circle(v)
    }
}
impl From<NurbsCurve2d> for Curve2dDescriptor {
    fn from(v: NurbsCurve2d) -> Self {
        Self::Nurbs(v)
    }
}

/// Deterministic dependency inspection implemented by every descriptor.
pub trait GeometryDependencies {
    /// Visit direct dependencies in stable descriptor-field order.
    fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef));
}

impl GeometryDependencies for CurveDescriptor {
    fn visit_dependencies(&self, _: &mut dyn FnMut(GeometryRef)) {}
}
impl GeometryDependencies for SurfaceDescriptor {
    fn visit_dependencies(&self, _: &mut dyn FnMut(GeometryRef)) {}
}
impl GeometryDependencies for Curve2dDescriptor {
    fn visit_dependencies(&self, _: &mut dyn FnMut(GeometryRef)) {}
}
