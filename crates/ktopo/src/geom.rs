//! Geometry attachments: closed enums over the L1 geometry classes.
//!
//! Topology references geometry through [`CurveGeom`] / [`SurfaceGeom`]
//! rather than trait objects so the store stays `'static`, cloneable, and
//! serializable, and so interchange (XT) can match on the exact class —
//! analytic classes must round-trip *as themselves*, never as NURBS.

use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};

/// An attached curve: one of the kernel's curve classes.
#[derive(Debug, Clone)]
pub enum CurveGeom {
    /// Straight line (arc-length parameterization).
    Line(Line),
    /// Circle.
    Circle(Circle),
    /// Ellipse.
    Ellipse(Ellipse),
    /// B-spline / NURBS curve.
    Nurbs(NurbsCurve),
}

impl CurveGeom {
    /// The uniform evaluator view.
    pub fn as_curve(&self) -> &dyn Curve {
        match self {
            CurveGeom::Line(c) => c,
            CurveGeom::Circle(c) => c,
            CurveGeom::Ellipse(c) => c,
            CurveGeom::Nurbs(c) => c,
        }
    }
}

impl From<Line> for CurveGeom {
    fn from(c: Line) -> Self {
        CurveGeom::Line(c)
    }
}
impl From<Circle> for CurveGeom {
    fn from(c: Circle) -> Self {
        CurveGeom::Circle(c)
    }
}
impl From<Ellipse> for CurveGeom {
    fn from(c: Ellipse) -> Self {
        CurveGeom::Ellipse(c)
    }
}
impl From<NurbsCurve> for CurveGeom {
    fn from(c: NurbsCurve) -> Self {
        CurveGeom::Nurbs(c)
    }
}

/// An attached surface: one of the kernel's surface classes.
#[derive(Debug, Clone)]
pub enum SurfaceGeom {
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
    /// B-spline / NURBS surface.
    Nurbs(NurbsSurface),
}

impl SurfaceGeom {
    /// The uniform evaluator view.
    pub fn as_surface(&self) -> &dyn Surface {
        match self {
            SurfaceGeom::Plane(s) => s,
            SurfaceGeom::Cylinder(s) => s,
            SurfaceGeom::Cone(s) => s,
            SurfaceGeom::Sphere(s) => s,
            SurfaceGeom::Torus(s) => s,
            SurfaceGeom::Nurbs(s) => s,
        }
    }
}

impl From<Plane> for SurfaceGeom {
    fn from(s: Plane) -> Self {
        SurfaceGeom::Plane(s)
    }
}
impl From<Cylinder> for SurfaceGeom {
    fn from(s: Cylinder) -> Self {
        SurfaceGeom::Cylinder(s)
    }
}
impl From<Cone> for SurfaceGeom {
    fn from(s: Cone) -> Self {
        SurfaceGeom::Cone(s)
    }
}
impl From<Sphere> for SurfaceGeom {
    fn from(s: Sphere) -> Self {
        SurfaceGeom::Sphere(s)
    }
}
impl From<Torus> for SurfaceGeom {
    fn from(s: Torus) -> Self {
        SurfaceGeom::Torus(s)
    }
}
impl From<NurbsSurface> for SurfaceGeom {
    fn from(s: NurbsSurface) -> Self {
        SurfaceGeom::Nurbs(s)
    }
}
