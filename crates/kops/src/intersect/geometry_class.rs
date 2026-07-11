//! Internal runtime classification for leaf geometry dispatch.
//!
//! Keep exact-type inspection in one place so pair dispatchers can operate on
//! typed references without each maintaining their own `Any` downcast table.

use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};

#[derive(Clone, Copy)]
pub(super) enum CurveClass<'a> {
    Line(&'a Line),
    Circle(&'a Circle),
    Ellipse(&'a Ellipse),
    Nurbs(&'a NurbsCurve),
}

impl<'a> CurveClass<'a> {
    pub(super) fn inspect(curve: &'a dyn Curve) -> Option<Self> {
        let curve = curve.as_any();
        if let Some(curve) = curve.downcast_ref() {
            return Some(Self::Line(curve));
        }
        if let Some(curve) = curve.downcast_ref() {
            return Some(Self::Circle(curve));
        }
        if let Some(curve) = curve.downcast_ref() {
            return Some(Self::Ellipse(curve));
        }
        curve.downcast_ref().map(Self::Nurbs)
    }
}

#[derive(Clone, Copy)]
pub(super) enum SurfaceClass<'a> {
    Plane(&'a Plane),
    Cylinder(&'a Cylinder),
    Cone(&'a Cone),
    Sphere(&'a Sphere),
    Torus(&'a Torus),
    Nurbs(&'a NurbsSurface),
}

impl<'a> SurfaceClass<'a> {
    pub(super) fn inspect(surface: &'a dyn Surface) -> Option<Self> {
        let surface = surface.as_any();
        if let Some(surface) = surface.downcast_ref() {
            return Some(Self::Plane(surface));
        }
        if let Some(surface) = surface.downcast_ref() {
            return Some(Self::Cylinder(surface));
        }
        if let Some(surface) = surface.downcast_ref() {
            return Some(Self::Cone(surface));
        }
        if let Some(surface) = surface.downcast_ref() {
            return Some(Self::Sphere(surface));
        }
        if let Some(surface) = surface.downcast_ref() {
            return Some(Self::Torus(surface));
        }
        surface.downcast_ref().map(Self::Nurbs)
    }
}
