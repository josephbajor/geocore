//! Private runtime leaf inspection around the canonical `kgraph` class enums.
//!
//! Exact-type downcasts stay local to dispatch. Stable identities and their
//! string registry are owned exclusively by `kgraph`.

use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgraph::{CurveClass, SurfaceClass};

#[derive(Clone, Copy)]
pub(super) enum CurveDispatch<'a> {
    Line(&'a Line),
    Circle(&'a Circle),
    Ellipse(&'a Ellipse),
    Nurbs(&'a NurbsCurve),
}

impl<'a> CurveDispatch<'a> {
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

    pub(super) const fn class(self) -> CurveClass {
        match self {
            Self::Line(_) => CurveClass::Line,
            Self::Circle(_) => CurveClass::Circle,
            Self::Ellipse(_) => CurveClass::Ellipse,
            Self::Nurbs(_) => CurveClass::Nurbs,
        }
    }
}

#[derive(Clone, Copy)]
pub(super) enum SurfaceDispatch<'a> {
    Plane(&'a Plane),
    Cylinder(&'a Cylinder),
    Cone(&'a Cone),
    Sphere(&'a Sphere),
    Torus(&'a Torus),
    Nurbs(&'a NurbsSurface),
}

impl<'a> SurfaceDispatch<'a> {
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

    pub(super) const fn class(self) -> SurfaceClass {
        match self {
            Self::Plane(_) => SurfaceClass::Plane,
            Self::Cylinder(_) => SurfaceClass::Cylinder,
            Self::Cone(_) => SurfaceClass::Cone,
            Self::Sphere(_) => SurfaceClass::Sphere,
            Self::Torus(_) => SurfaceClass::Torus,
            Self::Nurbs(_) => SurfaceClass::Nurbs,
        }
    }
}
