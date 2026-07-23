//! Immutable leaf descriptors and dependency reporting.

use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::{Circle2d, Curve2d, Line2d, NurbsCurve2d};
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};

use crate::class::{Curve2dClass, CurveClass, GeometryClassKey, SurfaceClass};
use crate::graph::{GeometryRef, SurfaceHandle};
use crate::intersection::{
    SphericalCirclePcurve, TransmittedIntersectionCurveDescriptor,
    TransmittedNurbsIntersectionCurveDescriptor, VerifiedIntersectionCurveDescriptor,
    VerifiedNurbsIntersectionCurveDescriptor,
};
use crate::{SkewCylinderBranchCarrier, SkewCylinderBranchPcurve};

/// Constant signed displacement along a basis surface's natural unit normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OffsetSurfaceDescriptor {
    basis: SurfaceHandle,
    signed_distance: f64,
}

impl OffsetSurfaceDescriptor {
    /// Construct a descriptor. Graph insertion validates finiteness and basis liveness.
    pub const fn new(basis: SurfaceHandle, signed_distance: f64) -> Self {
        Self {
            basis,
            signed_distance,
        }
    }

    /// Basis surface identity.
    pub const fn basis(self) -> SurfaceHandle {
        self.basis
    }

    /// Signed model-space displacement along the natural unit normal.
    pub const fn signed_distance(self) -> f64 {
        self.signed_distance
    }
}

/// A 3D curve descriptor.
// The operation-local procedural variant intentionally stays inline so its
// certifier-minted Copy identity survives value handoff without indirection.
// Graph insertion rejects it before arena allocation.
#[allow(clippy::large_enum_variant)]
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
    /// Finite graph-owned intersection branch with paired trace proof.
    Intersection(Box<VerifiedIntersectionCurveDescriptor>),
    /// Operation-generated degree-1 Plane/NURBS branch with paired trace proof.
    VerifiedNurbsIntersection(Box<VerifiedNurbsIntersectionCurveDescriptor>),
    /// Verified chordal intersection geometry retained from interchange.
    TransmittedIntersection(Box<TransmittedIntersectionCurveDescriptor>),
    /// Verified chordal chart with original NURBS traces retained from interchange.
    TransmittedNurbsIntersection(Box<TransmittedNurbsIntersectionCurveDescriptor>),
    /// Operation-local certified procedural skew Cylinder/Cylinder sheet.
    SkewCylinderBranch(SkewCylinderBranchCarrier),
}

impl CurveDescriptor {
    /// Borrow the node's immutable descriptor payload.
    pub const fn descriptor(&self) -> &Self {
        self
    }

    /// Uniform leaf evaluator view.
    pub fn as_curve(&self) -> &dyn Curve {
        match self {
            Self::Line(value) => value,
            Self::Circle(value) => value,
            Self::Ellipse(value) => value,
            Self::Nurbs(value) => value,
            Self::Intersection(value) => value.as_ref(),
            Self::VerifiedNurbsIntersection(value) => value.as_ref(),
            Self::TransmittedIntersection(value) => value.as_ref(),
            Self::TransmittedNurbsIntersection(value) => value.as_ref(),
            Self::SkewCylinderBranch(value) => value,
        }
    }

    /// Exact class of this descriptor.
    pub const fn class(&self) -> CurveClass {
        match self {
            Self::Line(_) => CurveClass::Line,
            Self::Circle(_) => CurveClass::Circle,
            Self::Ellipse(_) => CurveClass::Ellipse,
            Self::Nurbs(_) => CurveClass::Nurbs,
            Self::Intersection(_) => CurveClass::Intersection,
            Self::VerifiedNurbsIntersection(_) => CurveClass::Intersection,
            Self::TransmittedIntersection(_) => CurveClass::Intersection,
            Self::TransmittedNurbsIntersection(_) => CurveClass::Intersection,
            Self::SkewCylinderBranch(_) => CurveClass::Intersection,
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

    /// Borrow this descriptor as a verified intersection branch when its
    /// class matches.
    pub fn as_intersection(&self) -> Option<&VerifiedIntersectionCurveDescriptor> {
        if let Self::Intersection(value) = self {
            Some(value.as_ref())
        } else {
            None
        }
    }

    /// Borrow this descriptor as an operation-generated verified Plane/NURBS
    /// branch.
    pub fn as_verified_nurbs_intersection(
        &self,
    ) -> Option<&VerifiedNurbsIntersectionCurveDescriptor> {
        if let Self::VerifiedNurbsIntersection(value) = self {
            Some(value.as_ref())
        } else {
            None
        }
    }

    /// Borrow this descriptor as a verified transmitted chordal intersection.
    pub fn as_transmitted_intersection(&self) -> Option<&TransmittedIntersectionCurveDescriptor> {
        if let Self::TransmittedIntersection(value) = self {
            Some(value.as_ref())
        } else {
            None
        }
    }

    /// Borrow as an operation-local skew-cylinder sheet.
    pub fn as_skew_cylinder_branch(&self) -> Option<&SkewCylinderBranchCarrier> {
        if let Self::SkewCylinderBranch(value) = self {
            Some(value)
        } else {
            None
        }
    }

    /// Borrow this descriptor as a verified transmitted original-NURBS chart.
    pub fn as_transmitted_nurbs_intersection(
        &self,
    ) -> Option<&TransmittedNurbsIntersectionCurveDescriptor> {
        if let Self::TransmittedNurbsIntersection(value) = self {
            Some(value.as_ref())
        } else {
            None
        }
    }

    /// Whether this descriptor is any graph-owned verified intersection
    /// family.
    pub const fn is_verified_intersection(&self) -> bool {
        matches!(
            self,
            Self::Intersection(_)
                | Self::VerifiedNurbsIntersection(_)
                | Self::TransmittedIntersection(_)
                | Self::TransmittedNurbsIntersection(_)
        )
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
impl From<SkewCylinderBranchCarrier> for CurveDescriptor {
    fn from(value: SkewCylinderBranchCarrier) -> Self {
        Self::SkewCylinderBranch(value)
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
    /// Constant signed true-normal offset of another graph surface.
    Offset(OffsetSurfaceDescriptor),
}

impl SurfaceDescriptor {
    /// Borrow the node's immutable descriptor payload.
    pub const fn descriptor(&self) -> &Self {
        self
    }

    /// Uniform evaluator view for leaf classes; procedural nodes return `None`.
    pub fn as_leaf_surface(&self) -> Option<&dyn Surface> {
        Some(match self {
            Self::Plane(value) => value,
            Self::Cylinder(value) => value,
            Self::Cone(value) => value,
            Self::Sphere(value) => value,
            Self::Torus(value) => value,
            Self::Nurbs(value) => value,
            Self::Offset(_) => return None,
        })
    }

    /// Exact class of this descriptor.
    pub const fn class(&self) -> SurfaceClass {
        match self {
            Self::Plane(_) => SurfaceClass::Plane,
            Self::Cylinder(_) => SurfaceClass::Cylinder,
            Self::Cone(_) => SurfaceClass::Cone,
            Self::Sphere(_) => SurfaceClass::Sphere,
            Self::Torus(_) => SurfaceClass::Torus,
            Self::Nurbs(_) => SurfaceClass::Nurbs,
            Self::Offset(_) => SurfaceClass::Offset,
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
    /// Borrow as an offset descriptor when the class matches.
    pub const fn as_offset(&self) -> Option<&OffsetSurfaceDescriptor> {
        if let Self::Offset(v) = self {
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
impl From<OffsetSurfaceDescriptor> for SurfaceDescriptor {
    fn from(v: OffsetSurfaceDescriptor) -> Self {
        Self::Offset(v)
    }
}

/// A two-dimensional parameter-space curve descriptor.
// The certified nonlinear variant intentionally stays inline: graph
// descriptors are immutable values, and retaining value semantics keeps
// certificate/pcurve equality and transactional validation exact.
#[allow(clippy::large_enum_variant)]
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Curve2dDescriptor {
    /// Straight line.
    Line(Line2d),
    /// Circle.
    Circle(Circle2d),
    /// B-spline or NURBS curve.
    Nurbs(NurbsCurve2d),
    /// Finite certifier-minted inverse sphere chart of a spatial circle.
    SphericalCircle(SphericalCirclePcurve),
    /// Operation-local certified procedural skew Cylinder/Cylinder sheet chart.
    SkewCylinderBranch(SkewCylinderBranchPcurve),
}

impl Curve2dDescriptor {
    /// Borrow the node's immutable descriptor payload.
    pub const fn descriptor(&self) -> &Self {
        self
    }

    /// Uniform leaf evaluator view.
    pub fn as_curve(&self) -> &dyn Curve2d {
        match self {
            Self::Line(value) => value,
            Self::Circle(value) => value,
            Self::Nurbs(value) => value,
            Self::SphericalCircle(value) => value,
            Self::SkewCylinderBranch(value) => value,
        }
    }

    /// Exact class of this descriptor.
    pub const fn class(&self) -> Curve2dClass {
        match self {
            Self::Line(_) => Curve2dClass::Line,
            Self::Circle(_) => Curve2dClass::Circle,
            Self::Nurbs(_) => Curve2dClass::Nurbs,
            Self::SphericalCircle(_) => Curve2dClass::SphericalCircle,
            Self::SkewCylinderBranch(_) => Curve2dClass::SkewCylinderBranch,
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
    /// Borrow as a certified inverse sphere-chart circle when its class
    /// matches.
    pub const fn as_spherical_circle(&self) -> Option<&SphericalCirclePcurve> {
        if let Self::SphericalCircle(value) = self {
            Some(value)
        } else {
            None
        }
    }

    /// Borrow as an operation-local skew-cylinder sheet chart.
    pub const fn as_skew_cylinder_branch(&self) -> Option<&SkewCylinderBranchPcurve> {
        if let Self::SkewCylinderBranch(value) = self {
            Some(value)
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
impl From<SkewCylinderBranchPcurve> for Curve2dDescriptor {
    fn from(value: SkewCylinderBranchPcurve) -> Self {
        Self::SkewCylinderBranch(value)
    }
}

/// Deterministic dependency inspection implemented by every descriptor.
pub trait GeometryDependencies {
    /// Visit direct dependencies in stable descriptor-field order.
    fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef));
}

impl GeometryDependencies for CurveDescriptor {
    fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef)) {
        match self {
            Self::Intersection(intersection) => intersection.visit_dependencies(visit),
            Self::VerifiedNurbsIntersection(intersection) => {
                intersection.visit_dependencies(visit);
            }
            Self::TransmittedIntersection(intersection) => {
                intersection.visit_dependencies(visit);
            }
            Self::TransmittedNurbsIntersection(intersection) => {
                intersection.visit_dependencies(visit);
            }
            _ => {}
        }
    }
}
impl GeometryDependencies for SurfaceDescriptor {
    fn visit_dependencies(&self, visit: &mut dyn FnMut(GeometryRef)) {
        if let Self::Offset(offset) = self {
            visit(GeometryRef::Surface(offset.basis()));
        }
    }
}
impl GeometryDependencies for Curve2dDescriptor {
    fn visit_dependencies(&self, _: &mut dyn FnMut(GeometryRef)) {}
}
