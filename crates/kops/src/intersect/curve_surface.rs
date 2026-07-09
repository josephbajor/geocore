use super::circle_cone::intersect_bounded_circle_cone;
use super::circle_cylinder::intersect_bounded_circle_cylinder;
use super::circle_sphere::intersect_bounded_circle_sphere;
use super::circle_torus::intersect_bounded_circle_torus;
use super::ellipse_cone::intersect_bounded_ellipse_cone;
use super::ellipse_cylinder::intersect_bounded_ellipse_cylinder;
use super::ellipse_sphere::intersect_bounded_ellipse_sphere;
use super::ellipse_torus::intersect_bounded_ellipse_torus;
use super::line_cone::intersect_bounded_line_cone;
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::line_plane::intersect_bounded_line_plane;
use super::line_sphere::intersect_bounded_line_sphere;
use super::line_torus::intersect_bounded_line_torus;
use super::nurbs_cone::intersect_bounded_nurbs_cone;
use super::nurbs_cylinder::intersect_bounded_nurbs_cylinder;
use super::nurbs_plane::intersect_bounded_nurbs_plane;
use super::nurbs_sphere::intersect_bounded_nurbs_sphere;
use super::planar_curve_plane::{intersect_bounded_circle_plane, intersect_bounded_ellipse_plane};
use super::result::CurveSurfaceIntersections;
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};

/// Intersect a curve with a surface over finite curve and surface windows.
///
/// This currently dispatches bounded line/surface analytic cases, planar
/// circle-or-ellipse/plane cases, NURBS/plane/sphere/cylinder/cone cases,
/// circle/cone/cylinder/sphere/torus cases, and ellipse/sphere/cylinder/
/// cone/torus cases.
/// Unsupported curve or surface classes fail explicitly; broader analytic
/// cases and the general subdivision/Newton curve/surface solver remain later
/// M4 work.
pub fn intersect_bounded_curve_surface(
    curve: &dyn Curve,
    curve_range: ParamRange,
    surface: &dyn Surface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    if let Some(line) = as_line(curve) {
        if let Some(plane) = as_plane(surface) {
            return intersect_bounded_line_plane(
                line,
                curve_range,
                plane,
                surface_range,
                tolerances,
            );
        }
        if let Some(cylinder) = as_cylinder(surface) {
            return intersect_bounded_line_cylinder(
                line,
                curve_range,
                cylinder,
                surface_range,
                tolerances,
            );
        }
        if let Some(cone) = as_cone(surface) {
            return intersect_bounded_line_cone(line, curve_range, cone, surface_range, tolerances);
        }
        if let Some(sphere) = as_sphere(surface) {
            return intersect_bounded_line_sphere(
                line,
                curve_range,
                sphere,
                surface_range,
                tolerances,
            );
        }
        if let Some(torus) = as_torus(surface) {
            return intersect_bounded_line_torus(
                line,
                curve_range,
                torus,
                surface_range,
                tolerances,
            );
        }
    }
    if let Some(plane) = as_plane(surface) {
        if let Some(circle) = as_circle(curve) {
            return intersect_bounded_circle_plane(
                circle,
                curve_range,
                plane,
                surface_range,
                tolerances,
            );
        }
        if let Some(ellipse) = as_ellipse(curve) {
            return intersect_bounded_ellipse_plane(
                ellipse,
                curve_range,
                plane,
                surface_range,
                tolerances,
            );
        }
        if let Some(nurbs) = as_nurbs(curve) {
            return intersect_bounded_nurbs_plane(
                nurbs,
                curve_range,
                plane,
                surface_range,
                tolerances,
            );
        }
    }
    if let Some(sphere) = as_sphere(surface)
        && let Some(circle) = as_circle(curve)
    {
        return intersect_bounded_circle_sphere(
            circle,
            curve_range,
            sphere,
            surface_range,
            tolerances,
        );
    }
    if let Some(sphere) = as_sphere(surface)
        && let Some(ellipse) = as_ellipse(curve)
    {
        return intersect_bounded_ellipse_sphere(
            ellipse,
            curve_range,
            sphere,
            surface_range,
            tolerances,
        );
    }
    if let Some(sphere) = as_sphere(surface)
        && let Some(nurbs) = as_nurbs(curve)
    {
        return intersect_bounded_nurbs_sphere(
            nurbs,
            curve_range,
            sphere,
            surface_range,
            tolerances,
        );
    }
    if let Some(cylinder) = as_cylinder(surface)
        && let Some(circle) = as_circle(curve)
    {
        return intersect_bounded_circle_cylinder(
            circle,
            curve_range,
            cylinder,
            surface_range,
            tolerances,
        );
    }
    if let Some(cylinder) = as_cylinder(surface)
        && let Some(ellipse) = as_ellipse(curve)
    {
        return intersect_bounded_ellipse_cylinder(
            ellipse,
            curve_range,
            cylinder,
            surface_range,
            tolerances,
        );
    }
    if let Some(cylinder) = as_cylinder(surface)
        && let Some(nurbs) = as_nurbs(curve)
    {
        return intersect_bounded_nurbs_cylinder(
            nurbs,
            curve_range,
            cylinder,
            surface_range,
            tolerances,
        );
    }
    if let Some(cone) = as_cone(surface)
        && let Some(circle) = as_circle(curve)
    {
        return intersect_bounded_circle_cone(circle, curve_range, cone, surface_range, tolerances);
    }
    if let Some(torus) = as_torus(surface)
        && let Some(circle) = as_circle(curve)
    {
        return intersect_bounded_circle_torus(
            circle,
            curve_range,
            torus,
            surface_range,
            tolerances,
        );
    }
    if let Some(cone) = as_cone(surface)
        && let Some(ellipse) = as_ellipse(curve)
    {
        return intersect_bounded_ellipse_cone(
            ellipse,
            curve_range,
            cone,
            surface_range,
            tolerances,
        );
    }
    if let Some(cone) = as_cone(surface)
        && let Some(nurbs) = as_nurbs(curve)
    {
        return intersect_bounded_nurbs_cone(nurbs, curve_range, cone, surface_range, tolerances);
    }
    if let Some(torus) = as_torus(surface)
        && let Some(ellipse) = as_ellipse(curve)
    {
        return intersect_bounded_ellipse_torus(
            ellipse,
            curve_range,
            torus,
            surface_range,
            tolerances,
        );
    }

    Err(Error::InvalidGeometry {
        reason: "unsupported curve/surface intersection class",
    })
}

fn as_line(curve: &dyn Curve) -> Option<&Line> {
    curve.as_any().downcast_ref()
}

fn as_circle(curve: &dyn Curve) -> Option<&Circle> {
    curve.as_any().downcast_ref()
}

fn as_ellipse(curve: &dyn Curve) -> Option<&Ellipse> {
    curve.as_any().downcast_ref()
}

fn as_nurbs(curve: &dyn Curve) -> Option<&NurbsCurve> {
    curve.as_any().downcast_ref()
}

fn as_plane(surface: &dyn Surface) -> Option<&Plane> {
    surface.as_any().downcast_ref()
}

fn as_cylinder(surface: &dyn Surface) -> Option<&Cylinder> {
    surface.as_any().downcast_ref()
}

fn as_cone(surface: &dyn Surface) -> Option<&Cone> {
    surface.as_any().downcast_ref()
}

fn as_sphere(surface: &dyn Surface) -> Option<&Sphere> {
    surface.as_any().downcast_ref()
}

fn as_torus(surface: &dyn Surface) -> Option<&Torus> {
    surface.as_any().downcast_ref()
}
