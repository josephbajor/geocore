//! Complete-carrier discovery windows for axial Plane/Cylinder circles.
//!
//! Stored face domains are finite broad-phase work boxes, not trim authority.
//! A narrow planar face can cut a circle into bounded arcs even when its work
//! box does not enclose the whole carrier. This module widens only an exact
//! shared-axis plane domain; topology-owned fins still perform every trim.

use kcore::interval::Interval;
use kgeom::param::ParamRange;
use ktopo::geom::SurfaceGeom;

/// Widen only the plane work box of an exact axial Plane/Cylinder pair far
/// enough to discover its complete analytic circle carrier.
///
/// Shared or opposed authored axes are admitted by exact component identity.
/// All derived bounds use outward interval arithmetic. Failure to form finite
/// bounds preserves the original domain and therefore remains fail-closed in
/// the downstream graph intersection.
pub(super) fn plane_cylinder_discovery_domains(
    surfaces: [&SurfaceGeom; 2],
    mut domains: [[ParamRange; 2]; 2],
) -> [[ParamRange; 2]; 2] {
    let (plane_index, plane, cylinder) = match surfaces {
        [SurfaceGeom::Plane(plane), SurfaceGeom::Cylinder(cylinder)] => (0, plane, cylinder),
        [SurfaceGeom::Cylinder(cylinder), SurfaceGeom::Plane(plane)] => (1, plane, cylinder),
        _ => return domains,
    };
    let plane_axis = plane.frame().z();
    let cylinder_axis = cylinder.frame().z();
    if plane_axis != cylinder_axis && plane_axis != -cylinder_axis {
        return domains;
    }

    let cylinder_origin = cylinder.frame().origin().to_array();
    let plane_origin = plane.frame().origin().to_array();
    let axis = cylinder_axis.to_array();
    let offset: [Interval; 3] = core::array::from_fn(|index| {
        Interval::point(plane_origin[index]) - Interval::point(cylinder_origin[index])
    });
    let height = (0..3).fold(Interval::point(0.0), |sum, index| {
        sum + offset[index] * Interval::point(axis[index])
    });
    let center: [Interval; 3] = core::array::from_fn(|index| {
        Interval::point(cylinder_origin[index]) + Interval::point(axis[index]) * height
    });
    let center_offset: [Interval; 3] =
        core::array::from_fn(|index| center[index] - Interval::point(plane_origin[index]));
    let radius = Interval::point(cylinder.radius());
    let mut widened_plane = domains[plane_index];
    for (parameter_axis, direction) in [plane.frame().x(), plane.frame().y()]
        .into_iter()
        .enumerate()
    {
        let direction = direction.to_array();
        let coordinate = (0..3).fold(Interval::point(0.0), |sum, index| {
            sum + center_offset[index] * Interval::point(direction[index])
        });
        let low = (coordinate.lo() - radius.hi()).next_down();
        let high = (coordinate.hi() + radius.hi()).next_up();
        if !(low.is_finite() && high.is_finite() && low < high) {
            return domains;
        }
        let current = widened_plane[parameter_axis];
        widened_plane[parameter_axis] = ParamRange::new(current.lo.min(low), current.hi.max(high));
    }
    domains[plane_index] = widened_plane;
    domains
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::surface::{Cylinder, Plane};
    use kgeom::vec::{Point3, Vec3};

    use super::*;

    #[test]
    fn axial_circle_widens_only_the_plane_work_box_outward() {
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap();
        let plane =
            SurfaceGeom::Plane(Plane::new(frame.with_origin(frame.point_at(0.0, 0.0, 0.5))));
        let cylinder = SurfaceGeom::Cylinder(Cylinder::new(frame, 1.5).unwrap());
        let plane_domain = [ParamRange::new(-1.0, 1.0), ParamRange::new(-3.0, 3.0)];
        let cylinder_domain = [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(0.0, 2.0),
        ];

        let widened =
            plane_cylinder_discovery_domains([&plane, &cylinder], [plane_domain, cylinder_domain]);
        assert!(widened[0][0].lo < -1.5);
        assert!(widened[0][0].hi > 1.5);
        assert!(widened[0][1].lo <= -3.0 && widened[0][1].hi >= 3.0);
        assert_eq!(widened[1], cylinder_domain);

        let swapped =
            plane_cylinder_discovery_domains([&cylinder, &plane], [cylinder_domain, plane_domain]);
        assert_eq!(swapped[0], cylinder_domain);
        assert_eq!(swapped[1], widened[0]);

        let perpendicular = SurfaceGeom::Cylinder(
            Cylinder::new(Frame::from_z(frame.origin(), frame.x()).unwrap(), 1.5).unwrap(),
        );
        assert_eq!(
            plane_cylinder_discovery_domains(
                [&plane, &perpendicular],
                [plane_domain, cylinder_domain],
            ),
            [plane_domain, cylinder_domain]
        );
    }
}
