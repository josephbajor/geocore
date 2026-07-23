//! Topology-licensed recovery of rounded oblique Plane/Cylinder rulings.
//!
//! An authored prism side owns two distinct line edges parallel to its sweep
//! axis. Their complete fin pcurves can prove the stored rounded Plane is a
//! tolerance representation of the ideal axis-containing side Plane even
//! when an exact dot product of the separately normalized frame fields is not
//! zero. This module uses that semantic fact only after the graph certifier
//! reports precisely that missing exact-zero relation. The raw analytic
//! solver must still return a complete strict two-ruling secant, and outward
//! affine residuals must certify both stored surface traces over the whole
//! carrier interval.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::{Curve, Line};
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::IntersectionCertificateError;
use kops::intersect::{
    ContactKind, GraphSurfaceIntersectionError, SurfaceIntersectionCurve,
    intersect_bounded_plane_cylinder,
};
use ktopo::entity::{FaceId as RawFaceId, PcurveEndpointKind, Sense, VertexId as RawVertexId};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::{
    RulingRecertification, SectionBranch, SectionBranchEvidence, SectionBranchTopology,
    SectionCarrier, SectionFragmentSite, SectionUvCurve, SectionUvLine,
};
use crate::FaceId;
use crate::error::{Error, Result};

const EXACT_ZERO_GAP: &str =
    "Plane/Cylinder ruling requires an exact plane-normal/cylinder-axis zero";

type AffineCoefficients = [[Interval; 3]; 2];

/// Stored surfaces behind one topology-licensed ruling branch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct SemanticRulingRecertification {
    plane: Plane,
    cylinder: Cylinder,
    plane_operand: usize,
    tolerance: f64,
}

/// Recover graph promotion only at the exact rounded-axis proof boundary.
#[allow(clippy::too_many_arguments)]
pub(super) fn recover(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    domains: [[ParamRange; 2]; 2],
    error: &GraphSurfaceIntersectionError,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<Vec<SectionBranch>>> {
    if !matches!(
        error,
        GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::UnsupportedCarrierParameterization { reason }
        ) if *reason == EXACT_ZERO_GAP
    ) {
        return Ok(None);
    }
    let (plane_operand, plane, cylinder) = match surfaces {
        [SurfaceGeom::Plane(plane), SurfaceGeom::Cylinder(cylinder)] => (0, *plane, *cylinder),
        [SurfaceGeom::Cylinder(cylinder), SurfaceGeom::Plane(plane)] => (1, *plane, *cylinder),
        _ => return Ok(None),
    };
    let plane_face = raw_faces[plane_operand];
    let face = store
        .get(plane_face)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let fin_count = face.loops().iter().try_fold(0_usize, |count, loop_id| {
        store
            .get(*loop_id)
            .map(|loop_| count.checked_add(loop_.fins().len()))
            .map_err(|source| Error::InconsistentTopology { source })?
            .ok_or_else(work_overflow)
    })?;
    let work = u64::try_from(fin_count)
        .ok()
        .and_then(|count| count.checked_mul(8))
        .and_then(|count| count.checked_add(32))
        .ok_or_else(work_overflow)?;
    super::charge(scope, work)?;
    let tolerance = scope.context().tolerances().linear();
    let angular = scope.context().tolerances().angular();
    if !certify_semantic_axis_containing_face(
        store,
        plane_face,
        plane,
        cylinder.frame().z(),
        tolerance,
        angular,
    )? {
        return Ok(None);
    }

    let cylinder_operand = 1 - plane_operand;
    let raw = intersect_bounded_plane_cylinder(
        &plane,
        domains[plane_operand],
        &cylinder,
        domains[cylinder_operand],
        scope.context().tolerances(),
    )
    .map_err(Error::from)?;
    if !raw.is_complete()
        || !raw.points.is_empty()
        || !raw.regions.is_empty()
        || raw.curves.len() != 2
    {
        return Ok(None);
    }

    let source = SemanticRulingRecertification {
        plane,
        cylinder,
        plane_operand,
        tolerance,
    };
    let mut branches = Vec::with_capacity(raw.curves.len());
    for raw_branch in raw.curves {
        if raw_branch.kind != ContactKind::Transverse {
            return Ok(None);
        }
        let SurfaceIntersectionCurve::Line(line) = raw_branch.curve else {
            return Ok(None);
        };
        if !certifiably_signed_axis(line.dir(), cylinder.frame().z(), angular) {
            return Ok(None);
        }
        let raw_pcurves = [
            plane_pcurve(line, plane),
            cylinder_pcurve(line, cylinder, raw_branch.uv_b_start[0]),
        ];
        let raw_uv = [
            [raw_branch.uv_a_start, raw_branch.uv_a_end],
            [raw_branch.uv_b_start, raw_branch.uv_b_end],
        ];
        let (pcurves, uv) = if plane_operand == 0 {
            (raw_pcurves, raw_uv)
        } else {
            ([raw_pcurves[1], raw_pcurves[0]], [raw_uv[1], raw_uv[0]])
        };
        let Some(flipped) = super::canonical_plane_cylinder_ruling_flip(
            surfaces[0],
            senses[0],
            surfaces[1],
            senses[1],
            line.origin(),
            line.dir(),
        ) else {
            return Ok(None);
        };
        let range = if flipped {
            ParamRange::new(-raw_branch.curve_range.hi, -raw_branch.curve_range.lo)
        } else {
            raw_branch.curve_range
        };
        let pcurves = if flipped {
            pcurves.map(reverse_uv_line)
        } else {
            pcurves
        };
        let low = usize::from(flipped);
        let high = usize::from(!flipped);
        let raw_points = [
            line.eval(raw_branch.curve_range.lo),
            line.eval(raw_branch.curve_range.hi),
        ];
        let sites = [low, high].map(|endpoint| SectionFragmentSite {
            point: raw_points[endpoint],
            surface_parameters: [uv[0][endpoint], uv[1][endpoint]],
            surface_window_boundaries: [
                on_window_boundary(uv[0][endpoint], domains[0]),
                on_window_boundary(uv[1][endpoint], domains[1]),
            ],
        });
        let mut branch = SectionBranch {
            faces: facades.clone(),
            carrier: SectionCarrier::Line {
                origin: line.origin(),
                direction: if flipped { -line.dir() } else { line.dir() },
            },
            range,
            topology: SectionBranchTopology::Open,
            pcurves: pcurves.map(SectionUvCurve::Line),
            fragment_sites: sites.to_vec(),
            endpoint_sites: [0, 1],
            evidence: SectionBranchEvidence {
                residual_bounds: [0.0; 2],
                tolerance,
            },
            ruling_recertification: Some(RulingRecertification::Semantic(source)),
            ruling_parameter_flipped: false,
        };
        let Some(residual_bounds) = recertify(&branch, range, &source) else {
            return Ok(None);
        };
        branch.evidence.residual_bounds = residual_bounds;
        branches.push(branch);
    }
    Ok(Some(branches))
}

/// Reissue the stored-surface residual proof over an expanded range.
pub(super) fn recertify(
    branch: &SectionBranch,
    range: ParamRange,
    source: &SemanticRulingRecertification,
) -> Option<[f64; 2]> {
    if !range.is_finite() || range.lo >= range.hi {
        return None;
    }
    let SectionCarrier::Line { origin, direction } = branch.carrier else {
        return None;
    };
    let carrier = line_coefficients(origin, direction)?;
    let mut bounds = [0.0; 2];
    for (operand, bound_slot) in bounds.iter_mut().enumerate() {
        let SectionUvCurve::Line(pcurve) = branch.pcurves[operand] else {
            return None;
        };
        let lifted = if operand == source.plane_operand {
            plane_coefficients(source.plane, pcurve)?
        } else {
            cylinder_coefficients(source.cylinder, pcurve)?
        };
        let bound = affine_residual_bound(carrier, lifted, range)?;
        if bound > source.tolerance {
            return None;
        }
        *bound_slot = bound;
    }
    Some(bounds)
}

/// Admit either polygon-side incidence or a topology-owned circular cap whose
/// adjacent source cylinder is exactly perpendicular to the opposing axis.
fn certify_semantic_axis_containing_face(
    store: &Store,
    face_id: RawFaceId,
    plane: Plane,
    opposing_axis: Vec3,
    tolerance: f64,
    angular: f64,
) -> Result<bool> {
    if certify_axis_containing_face(store, face_id, plane, opposing_axis, tolerance, angular)? {
        return Ok(true);
    }
    Ok(
        certify_circular_cap_source(store, face_id, plane, tolerance)?
            .is_some_and(|source| source_axis_is_perpendicular(source, opposing_axis)),
    )
}

/// Prove the ideal face contains the signed axis through two independent,
/// whole-fin, tolerance-incidence line uses.
fn certify_axis_containing_face(
    store: &Store,
    face_id: RawFaceId,
    plane: Plane,
    axis: Vec3,
    tolerance: f64,
    angular: f64,
) -> Result<bool> {
    let face = store
        .get(face_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    if face.tolerance().is_some() || face.loops().len() != 1 {
        return Ok(false);
    }
    let loop_ = store
        .get(face.loops()[0])
        .map_err(|source| Error::InconsistentTopology { source })?;
    if loop_.face() != face_id || loop_.fins().len() < 4 {
        return Ok(false);
    }
    let mut axial_vertices = Vec::<[RawVertexId; 2]>::new();
    for &fin_id in loop_.fins() {
        let fin = store
            .get(fin_id)
            .map_err(|source| Error::InconsistentTopology { source })?;
        if fin.parent() != face.loops()[0] {
            return Ok(false);
        }
        let edge = store
            .get(fin.edge())
            .map_err(|source| Error::InconsistentTopology { source })?;
        let Some(curve_id) = edge.curve() else {
            continue;
        };
        let Some(line) = store
            .geometry()
            .curve(curve_id)
            .and_then(CurveGeom::as_line)
        else {
            continue;
        };
        if !certifiably_signed_axis(line.dir(), axis, angular) {
            continue;
        }
        let (Some((lo, hi)), [Some(first), Some(second)], Some(use_)) =
            (edge.bounds(), edge.vertices(), fin.pcurve())
        else {
            return Ok(false);
        };
        if first == second
            || edge.tolerance().is_some()
            || edge.fins().len() != 2
            || !use_.chart().is_identity()
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
            || use_.endpoint_kinds() != [PcurveEndpointKind::Regular; 2]
        {
            return Ok(false);
        }
        let active = use_.range();
        let map = use_.edge_to_pcurve();
        let mapped = [map.map(lo), map.map(hi)];
        if active.lo != mapped[0].min(mapped[1]) || active.hi != mapped[0].max(mapped[1]) {
            return Ok(false);
        }
        let Some(pcurve) = store
            .geometry()
            .curve2d(use_.curve())
            .and_then(Curve2dGeom::as_line)
        else {
            return Ok(false);
        };
        let uv = SectionUvLine {
            origin: pcurve.origin() + pcurve.dir() * map.offset(),
            direction: pcurve.dir() * map.scale(),
        };
        let (Some(carrier), Some(lifted)) = (
            line_coefficients(line.origin(), line.dir()),
            plane_coefficients(plane, uv),
        ) else {
            return Ok(false);
        };
        let Some(bound) = affine_residual_bound(carrier, lifted, ParamRange::new(lo, hi)) else {
            return Ok(false);
        };
        if bound > tolerance {
            return Ok(false);
        }
        axial_vertices.push([first, second]);
    }
    if axial_vertices.len() != 2 {
        return Ok(false);
    }
    Ok(axial_vertices[0]
        .iter()
        .all(|vertex| !axial_vertices[1].contains(vertex)))
}

/// Recover a finite-cylinder source axis only from the cap's complete
/// Plane/Circle/Circle2d ring and the other manifold fin's Cylinder ownership.
fn certify_circular_cap_source(
    store: &Store,
    face_id: RawFaceId,
    plane: Plane,
    tolerance: f64,
) -> Result<Option<Cylinder>> {
    let face = store
        .get(face_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let [loop_id] = face.loops() else {
        return Ok(None);
    };
    let loop_ = store
        .get(*loop_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let [cap_fin_id] = loop_.fins() else {
        return Ok(None);
    };
    let cap_fin = store
        .get(*cap_fin_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let edge = store
        .get(cap_fin.edge())
        .map_err(|source| Error::InconsistentTopology { source })?;
    let (Some(curve_id), Some(cap_use)) = (edge.curve(), cap_fin.pcurve()) else {
        return Ok(None);
    };
    let (Some(circle), Some(_cap_circle)) = (
        store
            .geometry()
            .curve(curve_id)
            .and_then(CurveGeom::as_circle),
        store
            .geometry()
            .curve2d(cap_use.curve())
            .and_then(Curve2dGeom::as_circle),
    ) else {
        return Ok(None);
    };
    let [first_fin, second_fin] = edge.fins() else {
        return Ok(None);
    };
    let side_fin_id = if first_fin == cap_fin_id {
        *second_fin
    } else if second_fin == cap_fin_id {
        *first_fin
    } else {
        return Ok(None);
    };
    let side_fin = store
        .get(side_fin_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let side_loop = store
        .get(side_fin.parent())
        .map_err(|source| Error::InconsistentTopology { source })?;
    let side_face = store
        .get(side_loop.face())
        .map_err(|source| Error::InconsistentTopology { source })?;
    let Some(SurfaceGeom::Cylinder(source_cylinder)) =
        store.geometry().surface(side_face.surface())
    else {
        return Ok(None);
    };

    let active = cap_use.range();
    let map = cap_use.edge_to_pcurve();
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
    let circle_range = circle.param_range();
    let exact_ownership = face.tolerance().is_none()
        && matches!(
            store.geometry().surface(face.surface()),
            Some(SurfaceGeom::Plane(stored)) if *stored == plane
        )
        && loop_.face() == face_id
        && cap_fin.parent() == *loop_id
        && cap_fin.edge() == side_fin.edge()
        && side_loop.fins().contains(&side_fin_id)
        && side_face.shell() == face.shell()
        && side_loop.face() != face_id
        && edge.vertices() == [None, None]
        && edge.bounds().is_none()
        && edge.tolerance().is_none();
    let cap_map_is_whole = cap_use.chart().is_identity()
        && cap_use.endpoint_kinds() == [PcurveEndpointKind::Regular; 2]
        && cap_use.closure_winding() == Some([0, 0])
        && cap_use.seam().is_none()
        && cap_use.edge_to_pcurve().scale().abs() == 1.0
        && active.width() == circle_range.width()
        && edge_parameters[0].min(edge_parameters[1]) == circle_range.lo
        && edge_parameters[0].max(edge_parameters[1]) == circle_range.hi
        && cap_fin.sense().times(cap_use.sense()) == face.sense();
    if !exact_ownership
        || !cap_map_is_whole
        || certify_whole_fin_incidence(store, face_id, *loop_id, *cap_fin_id, tolerance)
            != WholeFinIncidence::Certified
        || certify_whole_fin_incidence(
            store,
            side_loop.face(),
            side_fin.parent(),
            side_fin_id,
            tolerance,
        ) != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    Ok(Some(*source_cylinder))
}

/// Cross-body perpendicularity is exact dyadic zero or exact identity with
/// one of the source frame's semantically orthogonal radial basis axes.
fn source_axis_is_perpendicular(source: Cylinder, opposing_axis: Vec3) -> bool {
    let source_frame = source.frame();
    let source_axis = source_frame.z();
    exactly_perpendicular(source_axis, opposing_axis)
        || [source_frame.x(), source_frame.y()]
            .into_iter()
            .any(|radial| opposing_axis == radial || opposing_axis == -radial)
}

fn exactly_perpendicular(first: Vec3, second: Vec3) -> bool {
    affine_dot3(first.to_array(), second.to_array(), [0.0; 3], 0.0)
        .is_some_and(|dot| dot.sign() == Orientation::Zero)
}

fn plane_pcurve(line: Line, plane: Plane) -> SectionUvLine {
    let offset = line.origin() - plane.frame().origin();
    SectionUvLine {
        origin: Point2::new(offset.dot(plane.frame().x()), offset.dot(plane.frame().y())),
        direction: Vec2::new(
            line.dir().dot(plane.frame().x()),
            line.dir().dot(plane.frame().y()),
        ),
    }
}

fn cylinder_pcurve(line: Line, cylinder: Cylinder, longitude: f64) -> SectionUvLine {
    let offset = line.origin() - cylinder.frame().origin();
    SectionUvLine {
        origin: Point2::new(longitude, offset.dot(cylinder.frame().z())),
        direction: Vec2::new(0.0, line.dir().dot(cylinder.frame().z())),
    }
}

const fn reverse_uv_line(line: SectionUvLine) -> SectionUvLine {
    SectionUvLine {
        origin: line.origin,
        direction: Vec2::new(-line.direction.x, -line.direction.y),
    }
}

fn certifiably_signed_axis(direction: Vec3, axis: Vec3, angular: f64) -> bool {
    if direction == axis || direction == -axis {
        return true;
    }
    if !angular.is_finite() || angular < 0.0 {
        return false;
    }
    let direction = direction.to_array().map(Interval::point);
    let axis = axis.to_array().map(Interval::point);
    let cross = [
        direction[1] * axis[2] - direction[2] * axis[1],
        direction[2] * axis[0] - direction[0] * axis[2],
        direction[0] * axis[1] - direction[1] * axis[0],
    ];
    let squared = cross
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square());
    let Some(norm) = squared.sqrt() else {
        return false;
    };
    let dot = direction[0] * axis[0] + direction[1] * axis[1] + direction[2] * axis[2];
    norm.hi() <= angular && !dot.contains_zero()
}

fn on_window_boundary(uv: [f64; 2], window: [ParamRange; 2]) -> bool {
    uv[0] == window[0].lo || uv[0] == window[0].hi || uv[1] == window[1].lo || uv[1] == window[1].hi
}

fn line_coefficients(origin: Point3, direction: Vec3) -> Option<AffineCoefficients> {
    coefficients(origin.to_array(), direction.to_array())
}

fn plane_coefficients(plane: Plane, pcurve: SectionUvLine) -> Option<AffineCoefficients> {
    let frame = plane.frame();
    let origin = frame.origin().to_array();
    let x = frame.x().to_array();
    let y = frame.y().to_array();
    let mut lifted = [[Interval::point(0.0); 3]; 2];
    for axis in 0..3 {
        lifted[0][axis] = finite(
            Interval::point(origin[axis])
                + Interval::point(x[axis]) * Interval::point(pcurve.origin.x)
                + Interval::point(y[axis]) * Interval::point(pcurve.origin.y),
        )?;
        lifted[1][axis] = finite(
            Interval::point(x[axis]) * Interval::point(pcurve.direction.x)
                + Interval::point(y[axis]) * Interval::point(pcurve.direction.y),
        )?;
    }
    Some(lifted)
}

fn cylinder_coefficients(cylinder: Cylinder, pcurve: SectionUvLine) -> Option<AffineCoefficients> {
    if pcurve.direction.x != 0.0 || !pcurve.origin.x.is_finite() {
        return None;
    }
    let (sin, cos) = math::sincos(pcurve.origin.x);
    let sin = primitive(sin)?;
    let cos = primitive(cos)?;
    let frame = cylinder.frame();
    let origin = frame.origin().to_array();
    let x = frame.x().to_array();
    let y = frame.y().to_array();
    let z = frame.z().to_array();
    let radius = Interval::point(cylinder.radius());
    let mut lifted = [[Interval::point(0.0); 3]; 2];
    for axis in 0..3 {
        lifted[0][axis] = finite(
            Interval::point(origin[axis])
                + radius * (Interval::point(x[axis]) * cos + Interval::point(y[axis]) * sin)
                + Interval::point(z[axis]) * Interval::point(pcurve.origin.y),
        )?;
        lifted[1][axis] = finite(Interval::point(z[axis]) * Interval::point(pcurve.direction.y))?;
    }
    Some(lifted)
}

fn coefficients(origin: [f64; 3], direction: [f64; 3]) -> Option<AffineCoefficients> {
    if origin
        .into_iter()
        .chain(direction)
        .any(|value| !value.is_finite())
    {
        return None;
    }
    Some([origin.map(Interval::point), direction.map(Interval::point)])
}

fn affine_residual_bound(
    carrier: AffineCoefficients,
    lifted: AffineCoefficients,
    range: ParamRange,
) -> Option<f64> {
    if !range.is_finite() || range.lo >= range.hi {
        return None;
    }
    let parameter = Interval::new(range.lo, range.hi);
    let mut squared = Interval::point(0.0);
    for axis in 0..3 {
        let residual = finite(
            (carrier[0][axis] - lifted[0][axis]) + (carrier[1][axis] - lifted[1][axis]) * parameter,
        )?;
        squared = finite(squared + residual.square())?;
    }
    let bound = squared.sqrt()?.hi();
    bound.is_finite().then_some(bound)
}

fn primitive(value: f64) -> Option<Interval> {
    value
        .is_finite()
        .then(|| Interval::new(value.next_down(), value.next_up()))
}

fn finite(value: Interval) -> Option<Interval> {
    (value.lo().is_finite() && value.hi().is_finite()).then_some(value)
}

fn work_overflow() -> Error {
    Error::Core {
        source: kcore::error::Error::InvalidGeometry {
            reason: "semantic ruling proof work count overflow",
        },
    }
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use ktopo::entity::FinPcurve;

    use super::*;
    use crate::section::BodySectionBudgetProfile;
    use crate::{FaceId, Kernel};

    const TEST_TOLERANCE: f64 = 1.0e-9;

    fn cap_faces(store: &Store, body: ktopo::entity::BodyId) -> Vec<RawFaceId> {
        store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .filter(|face| {
                matches!(
                    store
                        .geometry()
                        .surface(store.get(*face).unwrap().surface()),
                    Some(SurfaceGeom::Plane(_))
                )
            })
            .collect()
    }

    fn recover_cap_rulings(frame: Frame, top_cap: bool, swapped: bool) -> Vec<SectionBranch> {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let (cap_face, opposing_face) = {
            let edit = session.edit_part(part_id.clone()).unwrap();
            let store = &mut edit.state.store;
            let source = ktopo::make::cylinder(store, &frame, 1.0, 0.1).unwrap();
            let caps = cap_faces(store, source);
            assert_eq!(caps.len(), 2);
            let cap_face = caps[usize::from(top_cap)];
            let cap_plane = match store
                .geometry()
                .surface(store.get(cap_face).unwrap().surface())
                .unwrap()
            {
                SurfaceGeom::Plane(plane) => *plane,
                _ => unreachable!(),
            };
            let opposing_frame =
                Frame::new(cap_plane.frame().origin() - frame.x(), frame.x(), frame.y()).unwrap();
            let opposing = ktopo::make::cylinder(store, &opposing_frame, 0.5, 2.0).unwrap();
            let opposing_face = store
                .faces_of_body(opposing)
                .unwrap()
                .into_iter()
                .find(|face| {
                    matches!(
                        store
                            .geometry()
                            .surface(store.get(*face).unwrap().surface()),
                        Some(SurfaceGeom::Cylinder(_))
                    )
                })
                .unwrap();
            (cap_face, opposing_face)
        };

        let part = session.part(part_id.clone()).unwrap();
        let store = &part.state.store;
        let raw_faces = if swapped {
            [opposing_face, cap_face]
        } else {
            [cap_face, opposing_face]
        };
        let face_data = raw_faces.map(|face| store.get(face).unwrap());
        let surfaces = face_data.map(|face| store.geometry().surface(face.surface()).unwrap());
        let senses = face_data.map(|face| face.sense());
        let domains = face_data.map(|face| {
            let domain = face.domain().unwrap();
            [domain.u, domain.v]
        });
        let facades = raw_faces.map(|face| FaceId::new(part_id.clone(), face));
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let error = GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: EXACT_ZERO_GAP,
            },
        );
        recover(
            store, raw_faces, &facades, surfaces, senses, domains, &error, &mut scope,
        )
        .unwrap()
        .expect("topology-owned cap must recover two strict secant rulings")
    }

    #[test]
    fn whole_circle_caps_recover_world_and_rounded_oblique_rulings_under_swap() {
        let frames = [
            Frame::world(),
            Frame::new(
                Point3::new(2.5, -1.75, 0.625),
                Vec3::new(0.48, 0.64, 0.6),
                Vec3::new(0.8, -0.6, 0.0),
            )
            .unwrap(),
        ];
        assert!(!exactly_perpendicular(frames[1].z(), frames[1].x()));
        for frame in frames {
            for top_cap in [false, true] {
                for swapped in [false, true] {
                    let branches = recover_cap_rulings(frame, top_cap, swapped);
                    assert_eq!(branches.len(), 2);
                    assert!(branches.iter().all(|branch| {
                        branch.topology == SectionBranchTopology::Open
                            && matches!(branch.carrier, SectionCarrier::Line { .. })
                            && branch
                                .evidence
                                .residual_bounds
                                .into_iter()
                                .all(|bound| bound <= branch.evidence.tolerance)
                    }));
                }
            }
        }
    }

    #[test]
    fn circular_cap_proof_refuses_malformed_winding_and_nonperpendicular_axes() {
        let mut store = Store::new();
        let frame = Frame::world();
        let body = ktopo::make::cylinder(&mut store, &frame, 1.0, 0.1).unwrap();
        let cap = cap_faces(&store, body)[0];
        let plane = match store
            .geometry()
            .surface(store.get(cap).unwrap().surface())
            .unwrap()
        {
            SurfaceGeom::Plane(plane) => *plane,
            _ => unreachable!(),
        };
        assert!(
            !certify_semantic_axis_containing_face(
                &store,
                cap,
                plane,
                frame.x() + frame.z(),
                TEST_TOLERANCE,
                Tolerances::default().angular(),
            )
            .unwrap()
        );

        let loop_id = store.get(cap).unwrap().loops()[0];
        let fin_id = store.get(loop_id).unwrap().fins()[0];
        let old = store.get(fin_id).unwrap().pcurve().unwrap();
        let mut transaction = store.transaction().unwrap();
        let mut assembly = transaction.assembly();
        assembly.get_mut(fin_id).unwrap().pcurve =
            Some(FinPcurve::new(old.curve(), old.range(), old.edge_to_pcurve()).unwrap());
        assert!(
            !certify_semantic_axis_containing_face(
                &assembly,
                cap,
                plane,
                frame.x(),
                TEST_TOLERANCE,
                Tolerances::default().angular(),
            )
            .unwrap()
        );
    }

    #[test]
    fn signed_axis_filter_accepts_outward_roundoff_but_rejects_tilt() {
        let axis = Vec3::new(0.48, 0.64, 0.6);
        let rounded = Vec3::new(0.48_f64.next_down(), 0.64_f64.next_up(), 0.6_f64.next_up());
        assert!(certifiably_signed_axis(rounded, axis, 1e-11));
        assert!(certifiably_signed_axis(-rounded, axis, 1e-11));

        let tilted = Vec3::new(0.48, 0.64, 0.6001).normalized().unwrap();
        assert!(!certifiably_signed_axis(tilted, axis, 1e-11));
        assert!(!certifiably_signed_axis(rounded, axis, -1.0));
    }
}
