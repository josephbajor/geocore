//! Exact and interval-certified containment for convex whole-shell inputs.

use crate::cylindrical_band::CylindricalBandSolidInput;
use crate::entity::Sense;
use crate::planar::{PlanarSolidInput, PreparedSolid};
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::vec::{Point3, Vec3};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy)]
pub(crate) struct PlanarSupport {
    pub(crate) outward: Vec3,
    pub(crate) origin: Point3,
}

#[derive(Debug)]
pub(crate) struct ConvexPlanarInputProof {
    pub(crate) vertices: Vec<Point3>,
    pub(crate) supports: Vec<PlanarSupport>,
}

/// Prove that a prepared positive planar shell bounds one convex solid.
pub(crate) fn certify_convex_planar_input(
    input: &PlanarSolidInput,
    prepared: &PreparedSolid,
    store: &Store,
) -> Result<ConvexPlanarInputProof> {
    let vertices = input
        .vertices()
        .iter()
        .map(|vertex| vertex.position())
        .collect::<Vec<_>>();
    let mut supports = Vec::with_capacity(input.faces().len());
    for index in 0..input.faces().len() {
        let incident = input.faces()[index]
            .vertices()
            .iter()
            .copied()
            .collect::<BTreeSet<_>>();
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "convex planar face disappeared during semantic preflight",
            })?;
        let outward = plane.frame().z() * sense_factor(sense);
        let support = PlanarSupport {
            outward,
            origin: plane.frame().origin(),
        };
        let mut nonincident = false;
        for vertex in input.vertices() {
            if incident.contains(&vertex.key()) {
                continue;
            }
            nonincident = true;
            if exact_affine(outward, vertex.position(), support.origin)? != Orientation::Negative {
                return invalid(
                    "nonincident planar vertices must lie strictly inside every face support",
                );
            }
        }
        if !nonincident {
            return invalid("planar face must support a bounded convex whole shell");
        }
        supports.push(support);
    }
    Ok(ConvexPlanarInputProof { vertices, supports })
}

/// Prove every vertex of a convex planar solid strictly inside a finite cylinder.
///
/// Convexity makes the vertex certificate complete: the finite cylinder is a
/// convex set, and the planar solid is the convex hull of its certified
/// vertices.
pub(crate) fn certify_planar_inside_cylinder(
    planar: &ConvexPlanarInputProof,
    cylinder: &CylindricalBandSolidInput,
) -> Result<()> {
    let frame = cylinder.frame();
    let range = cylinder.axial_range();
    let low = frame.origin() + frame.z() * range.lo;
    let high = frame.origin() + frame.z() * range.hi;
    for &vertex in &planar.vertices {
        if exact_affine(frame.z(), vertex, low)? != Orientation::Positive
            || exact_affine(frame.z(), vertex, high)? != Orientation::Negative
            || radial_relation(vertex, low, frame.x(), frame.y(), cylinder.radius())
                != StrictRelation::Inside
        {
            return invalid(
                "convex planar whole shell must lie strictly inside the finite cylinder",
            );
        }
    }
    Ok(())
}

/// Prove two convex planar inputs strictly separated by a support of either.
pub(crate) fn certify_planar_inputs_separated(
    first: &ConvexPlanarInputProof,
    second: &ConvexPlanarInputProof,
) -> Result<()> {
    if separated_by_supports(&first.supports, &second.vertices)?
        || separated_by_supports(&second.supports, &first.vertices)?
    {
        Ok(())
    } else {
        invalid("convex planar cavity shells require certified pairwise separation")
    }
}

fn separated_by_supports(supports: &[PlanarSupport], vertices: &[Point3]) -> Result<bool> {
    for support in supports {
        let mut separates = true;
        for &vertex in vertices {
            if exact_affine(support.outward, vertex, support.origin)? != Orientation::Positive {
                separates = false;
                break;
            }
        }
        if separates {
            return Ok(true);
        }
    }
    Ok(false)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StrictRelation {
    Inside,
    Outside,
    Indeterminate,
}

pub(crate) fn axial_lower_relation(point: Point3, center: Point3, axis: Vec3) -> StrictRelation {
    signed_relation(interval_dot(axis, point - center), true)
}

pub(crate) fn axial_upper_relation(point: Point3, center: Point3, axis: Vec3) -> StrictRelation {
    signed_relation(interval_dot(axis, point - center), false)
}

pub(crate) fn radial_relation(
    point: Point3,
    axis_origin: Point3,
    radial_x: Vec3,
    radial_y: Vec3,
    radius: f64,
) -> StrictRelation {
    let relative = point - axis_origin;
    let squared =
        interval_dot(radial_x, relative).square() + interval_dot(radial_y, relative).square();
    let radius_squared = Interval::point(radius).square();
    if squared.hi() < radius_squared.lo() {
        StrictRelation::Inside
    } else if squared.lo() > radius_squared.hi() {
        StrictRelation::Outside
    } else {
        StrictRelation::Indeterminate
    }
}

fn signed_relation(value: Interval, positive_inside: bool) -> StrictRelation {
    if positive_inside {
        if value.lo() > 0.0 {
            StrictRelation::Inside
        } else if value.hi() < 0.0 {
            StrictRelation::Outside
        } else {
            StrictRelation::Indeterminate
        }
    } else if value.hi() < 0.0 {
        StrictRelation::Inside
    } else if value.lo() > 0.0 {
        StrictRelation::Outside
    } else {
        StrictRelation::Indeterminate
    }
}

pub(crate) fn exact_affine(normal: Vec3, point: Point3, origin: Point3) -> Result<Orientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "convex containment exact affine predicate is indeterminate",
        })
}

pub(crate) fn interval_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
}

const fn sense_factor(sense: Sense) -> f64 {
    if matches!(sense, Sense::Forward) {
        1.0
    } else {
        -1.0
    }
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}
