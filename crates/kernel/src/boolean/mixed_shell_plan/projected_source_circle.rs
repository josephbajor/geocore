//! Exact projection evidence for reusing one cylinder ring span on a Plane.
//!
//! The periodic source span remains the 3D carrier authority.  This proof only
//! supplies a second, face-local pcurve for the same physical edge after
//! binding the retained ring topology and proving that its complete circle is
//! contained in the target Plane.  Materialization re-runs this certification
//! against the live store before emitting the pcurve.

use kcore::math;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, CylinderLongitudeTrace, PlaneCircleTrace, PlaneCylinderCircleTrace,
    certify_paired_plane_cylinder_circle_residuals,
};
use ktopo::analytic_shell::{AnalyticPcurveUse, AnalyticShellPcurve};
use ktopo::entity::{EdgeId as RawEdgeId, FinId as RawFinId, LoopId as RawLoopId};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use super::{MixedBoundedSourceSpanPlan, MixedSourceFaceKey, MixedSourceSpanKey};
use crate::FaceId;
use crate::boolean::mixed_cap_boundary::MixedCylinderCapRing;
use crate::boolean::mixed_periodic_arrangement::PeriodicSourceLoopKey;

/// First failed obligation while certifying a projected source-circle pcurve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ProjectedSourceCircleOnPlaneError {
    StoreRead,
    FacePartMismatch,
    FaceKeyMismatch,
    SourceTopologyMismatch,
    SourceFaceNotCylinder,
    SourceCurveNotCircle,
    CircleNotOnCylinder,
    TargetFaceNotPlane,
    TargetFaceNotCylinder,
    CircleNotOnPlane,
    InvalidProjection,
}

/// Certified target-chart pcurve for one endpoint-free source ring.
#[derive(Debug, Clone, PartialEq, Eq)]
enum ProjectedEndpointFreePcurve {
    Plane {
        center_bits: [u64; 2],
        radius_bits: u64,
        projected_x_bits: [u64; 2],
        circle_x_bits: [u64; 2],
        parameter_scale: i8,
        parameter_offset_bits: u64,
    },
    Cylinder {
        origin_bits: [u64; 2],
        direction_bits: [u64; 2],
        parameter_scale: i8,
        parameter_offset_bits: u64,
    },
}

/// Exact projection evidence for reusing a whole cylinder ring on another
/// selected face.
///
/// The raw source ring remains the 3D carrier and lineage authority. This
/// proof only supplies the target face's pcurve. It supports the two generic
/// cases needed by arrangement plans: a circle on a coincident plane and the
/// same circle on an exactly common cylindrical support.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ProjectedEndpointFreeSourceCircle {
    source: MixedSourceFaceKey,
    source_face: FaceId,
    target: MixedSourceFaceKey,
    target_face: FaceId,
    loop_key: PeriodicSourceLoopKey,
    edge: RawEdgeId,
    pcurve: ProjectedEndpointFreePcurve,
    residual_bound_bits: [u64; 2],
    tolerance_bits: u64,
}

impl ProjectedEndpointFreeSourceCircle {
    pub(crate) fn certify(
        store: &Store,
        ring: &MixedCylinderCapRing,
        target: MixedSourceFaceKey,
        target_face: &FaceId,
        tolerance: f64,
    ) -> Result<Self, ProjectedSourceCircleOnPlaneError> {
        if ring.side_face().part() != target_face.part() {
            return Err(ProjectedSourceCircleOnPlaneError::FacePartMismatch);
        }
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
        }
        let (source_cylinder, circle) = endpoint_free_source_circle(store, ring)?;
        let target_value = store
            .get(target_face.raw())
            .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
        let (pcurve, residual_bounds) = match store
            .surface(target_value.surface())
            .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
        {
            SurfaceGeom::Plane(plane) => {
                let local_center = plane.frame().to_local(circle.frame().origin());
                let projected_x = project_direction(circle.frame().x(), *plane);
                let circle_2d = Circle2d::new(
                    Point2::new(local_center.x, local_center.y),
                    circle.radius(),
                    projected_x,
                )
                .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
                let source_trace =
                    endpoint_free_source_cylinder_trace(store, ring, source_cylinder, circle)?;
                let mut certified = [-1_i8, 1_i8].into_iter().filter_map(|parameter_scale| {
                    let map = AffineParamMap1d::new(parameter_scale as f64, 0.0).ok()?;
                    let traces = [
                        PlaneCylinderCircleTrace::Cylinder(source_trace),
                        PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(
                            *plane, circle_2d, map,
                        )),
                    ];
                    certify_paired_plane_cylinder_circle_residuals(
                        circle,
                        circle.param_range(),
                        traces,
                        tolerance,
                    )
                    .ok()
                    .map(|proof| (parameter_scale, proof.residual_bounds()))
                });
                let Some((parameter_scale, residual_bounds)) = certified.next() else {
                    return Err(ProjectedSourceCircleOnPlaneError::CircleNotOnPlane);
                };
                if certified.next().is_some() {
                    return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
                }
                (
                    ProjectedEndpointFreePcurve::Plane {
                        center_bits: [local_center.x.to_bits(), local_center.y.to_bits()],
                        radius_bits: circle_2d.radius().to_bits(),
                        projected_x_bits: [projected_x.x.to_bits(), projected_x.y.to_bits()],
                        circle_x_bits: [
                            circle_2d.x_dir().x.to_bits(),
                            circle_2d.x_dir().y.to_bits(),
                        ],
                        parameter_scale,
                        parameter_offset_bits: 0.0_f64.to_bits(),
                    },
                    residual_bounds,
                )
            }
            SurfaceGeom::Cylinder(cylinder) => {
                let local = cylinder.frame().to_local(circle.frame().origin());
                let phase = math::atan2(
                    circle.frame().x().dot(cylinder.frame().y()),
                    circle.frame().x().dot(cylinder.frame().x()),
                );
                let line = Line2d::new(Point2::new(phase, local.z), Vec2::new(1.0, 0.0))
                    .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
                let source_trace = endpoint_free_source_plane_trace(store, ring)?;
                let mut certified = [-1_i8, 1_i8].into_iter().filter_map(|parameter_scale| {
                    let map = AffineParamMap1d::new(parameter_scale as f64, 0.0).ok()?;
                    let traces = [
                        PlaneCylinderCircleTrace::Plane(source_trace),
                        PlaneCylinderCircleTrace::Cylinder(CylinderLongitudeTrace::new(
                            *cylinder, line, map,
                        )),
                    ];
                    certify_paired_plane_cylinder_circle_residuals(
                        circle,
                        circle.param_range(),
                        traces,
                        tolerance,
                    )
                    .ok()
                    .map(|proof| (parameter_scale, proof.residual_bounds()))
                });
                let Some((parameter_scale, residual_bounds)) = certified.next() else {
                    return Err(ProjectedSourceCircleOnPlaneError::CircleNotOnCylinder);
                };
                if certified.next().is_some() {
                    return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
                }
                (
                    ProjectedEndpointFreePcurve::Cylinder {
                        origin_bits: [line.origin().x.to_bits(), line.origin().y.to_bits()],
                        direction_bits: [line.dir().x.to_bits(), line.dir().y.to_bits()],
                        parameter_scale,
                        parameter_offset_bits: 0.0_f64.to_bits(),
                    },
                    residual_bounds,
                )
            }
            _ => return Err(ProjectedSourceCircleOnPlaneError::TargetFaceNotPlane),
        };
        Ok(Self {
            source: ring.side_source(),
            source_face: ring.side_face().clone(),
            target,
            target_face: target_face.clone(),
            loop_key: ring.side_loop_key(),
            edge: ring.edge(),
            pcurve,
            residual_bound_bits: residual_bounds.map(f64::to_bits),
            tolerance_bits: tolerance.to_bits(),
        })
    }

    pub(crate) const fn source(&self) -> MixedSourceFaceKey {
        self.source
    }

    pub(crate) const fn source_face(&self) -> &FaceId {
        &self.source_face
    }

    pub(crate) const fn target(&self) -> MixedSourceFaceKey {
        self.target
    }

    pub(crate) const fn target_face(&self) -> &FaceId {
        &self.target_face
    }

    pub(crate) const fn loop_key(&self) -> PeriodicSourceLoopKey {
        self.loop_key
    }

    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn tolerance(&self) -> f64 {
        f64::from_bits(self.tolerance_bits)
    }

    pub(crate) fn pcurve(&self) -> Result<AnalyticPcurveUse, ProjectedSourceCircleOnPlaneError> {
        match self.pcurve {
            ProjectedEndpointFreePcurve::Plane {
                center_bits,
                radius_bits,
                projected_x_bits,
                circle_x_bits,
                parameter_scale,
                parameter_offset_bits,
            } => {
                let circle = Circle2d::new(
                    Point2::new(
                        f64::from_bits(center_bits[0]),
                        f64::from_bits(center_bits[1]),
                    ),
                    f64::from_bits(radius_bits),
                    Vec2::new(
                        f64::from_bits(projected_x_bits[0]),
                        f64::from_bits(projected_x_bits[1]),
                    ),
                )
                .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
                if [circle.x_dir().x.to_bits(), circle.x_dir().y.to_bits()] != circle_x_bits {
                    return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
                }
                let map = AffineParamMap1d::new(
                    parameter_scale as f64,
                    f64::from_bits(parameter_offset_bits),
                )
                .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
                Ok(
                    AnalyticPcurveUse::new(AnalyticShellPcurve::Circle(circle), map)
                        .with_closure_winding([0, 0]),
                )
            }
            ProjectedEndpointFreePcurve::Cylinder {
                origin_bits,
                direction_bits,
                parameter_scale,
                parameter_offset_bits,
            } => {
                let line = Line2d::new(
                    Point2::new(
                        f64::from_bits(origin_bits[0]),
                        f64::from_bits(origin_bits[1]),
                    ),
                    Vec2::new(
                        f64::from_bits(direction_bits[0]),
                        f64::from_bits(direction_bits[1]),
                    ),
                )
                .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
                let map = AffineParamMap1d::new(
                    parameter_scale as f64,
                    f64::from_bits(parameter_offset_bits),
                )
                .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
                Ok(AnalyticPcurveUse::new(AnalyticShellPcurve::Line(line), map)
                    .with_closure_winding([parameter_scale.into(), 0]))
            }
        }
    }
}

/// Immutable coefficients and topology bindings for one projected pcurve.
///
/// Floating-point values are retained as bits so equality is exact and a
/// copied or altered proof cannot silently change the emitted chart.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ProjectedSourceCircleOnPlane {
    source: MixedSourceFaceKey,
    source_face: FaceId,
    target: MixedSourceFaceKey,
    target_face: FaceId,
    span: MixedSourceSpanKey,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    center_bits: [u64; 2],
    radius_bits: u64,
    projected_x_bits: [u64; 2],
    circle_x_bits: [u64; 2],
    parameter_scale: i8,
    parameter_offset_bits: u64,
    residual_bound_bits: [u64; 2],
    tolerance_bits: u64,
}

impl ProjectedSourceCircleOnPlane {
    /// Prove that a retained cylinder ring can be evaluated in `target_face`.
    ///
    /// The source span provides the virtual bounded-arc identity; its raw edge
    /// remains a vertexless complete Circle.  Both face identities are kept so
    /// later materialization can reject cross-part, cross-face, and stale-span
    /// substitutions before constructing an analytic shell.
    pub(crate) fn certify(
        store: &Store,
        source_face: &FaceId,
        source_span: &MixedBoundedSourceSpanPlan,
        target: MixedSourceFaceKey,
        target_face: &FaceId,
        tolerance: f64,
    ) -> Result<Self, ProjectedSourceCircleOnPlaneError> {
        if source_face.part() != target_face.part() {
            return Err(ProjectedSourceCircleOnPlaneError::FacePartMismatch);
        }
        if (source_span.source() == target) != (source_face == target_face) {
            return Err(ProjectedSourceCircleOnPlaneError::FaceKeyMismatch);
        }

        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
        }
        let (cylinder, circle) = source_circle(store, source_face, source_span)?;
        if circle.radius() != cylinder.radius() {
            return Err(ProjectedSourceCircleOnPlaneError::CircleNotOnCylinder);
        }

        let target_value = store
            .get(target_face.raw())
            .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
        let SurfaceGeom::Plane(plane) = store
            .surface(target_value.surface())
            .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
        else {
            return Err(ProjectedSourceCircleOnPlaneError::TargetFaceNotPlane);
        };
        let local_center = plane.frame().to_local(circle.frame().origin());
        let projected_x = project_direction(circle.frame().x(), *plane);
        let circle_2d = Circle2d::new(
            Point2::new(local_center.x, local_center.y),
            circle.radius(),
            projected_x,
        )
        .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
        if !local_center.x.is_finite() || !local_center.y.is_finite() {
            return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
        }
        let cylinder_trace = source_cylinder_trace(store, source_span, cylinder, circle)?;
        let mut certified = [-1_i8, 1_i8].into_iter().filter_map(|parameter_scale| {
            let map = AffineParamMap1d::new(parameter_scale as f64, 0.0).ok()?;
            let traces = [
                PlaneCylinderCircleTrace::Cylinder(cylinder_trace),
                PlaneCylinderCircleTrace::Plane(PlaneCircleTrace::new(*plane, circle_2d, map)),
            ];
            certify_paired_plane_cylinder_circle_residuals(
                circle,
                circle.param_range(),
                traces,
                tolerance,
            )
            .ok()
            .map(|proof| (parameter_scale, proof.residual_bounds()))
        });
        let Some((parameter_scale, residual_bounds)) = certified.next() else {
            return Err(ProjectedSourceCircleOnPlaneError::CircleNotOnPlane);
        };
        if certified.next().is_some() {
            return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
        }

        Ok(Self {
            source: source_span.source(),
            source_face: source_face.clone(),
            target,
            target_face: target_face.clone(),
            span: source_span.span().clone(),
            loop_id: source_span.loop_id(),
            fin: source_span.fin(),
            edge: source_span.edge(),
            center_bits: [local_center.x.to_bits(), local_center.y.to_bits()],
            radius_bits: circle_2d.radius().to_bits(),
            projected_x_bits: [projected_x.x.to_bits(), projected_x.y.to_bits()],
            circle_x_bits: [circle_2d.x_dir().x.to_bits(), circle_2d.x_dir().y.to_bits()],
            parameter_scale,
            parameter_offset_bits: 0.0_f64.to_bits(),
            residual_bound_bits: residual_bounds.map(f64::to_bits),
            tolerance_bits: tolerance.to_bits(),
        })
    }

    pub(crate) const fn source(&self) -> MixedSourceFaceKey {
        self.source
    }

    pub(crate) const fn source_face(&self) -> &FaceId {
        &self.source_face
    }

    pub(crate) const fn target(&self) -> MixedSourceFaceKey {
        self.target
    }

    pub(crate) const fn target_face(&self) -> &FaceId {
        &self.target_face
    }

    pub(crate) const fn span(&self) -> &MixedSourceSpanKey {
        &self.span
    }

    pub(crate) const fn loop_id(&self) -> RawLoopId {
        self.loop_id
    }

    pub(crate) const fn fin(&self) -> RawFinId {
        self.fin
    }

    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn center(&self) -> Point2 {
        Point2::new(
            f64::from_bits(self.center_bits[0]),
            f64::from_bits(self.center_bits[1]),
        )
    }

    pub(crate) fn x_direction(&self) -> Result<Vec2, ProjectedSourceCircleOnPlaneError> {
        self.circle().map(|_| {
            Vec2::new(
                f64::from_bits(self.circle_x_bits[0]),
                f64::from_bits(self.circle_x_bits[1]),
            )
        })
    }

    pub(crate) fn circle(&self) -> Result<Circle2d, ProjectedSourceCircleOnPlaneError> {
        let circle = Circle2d::new(
            self.center(),
            f64::from_bits(self.radius_bits),
            Vec2::new(
                f64::from_bits(self.projected_x_bits[0]),
                f64::from_bits(self.projected_x_bits[1]),
            ),
        )
        .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
        let expected = self.circle_x_bits.map(f64::from_bits);
        if circle.x_dir().x.to_bits() != expected[0].to_bits()
            || circle.x_dir().y.to_bits() != expected[1].to_bits()
        {
            return Err(ProjectedSourceCircleOnPlaneError::InvalidProjection);
        }
        Ok(circle)
    }

    pub(crate) const fn parameter_scale(&self) -> f64 {
        self.parameter_scale as f64
    }

    pub(crate) const fn parameter_offset(&self) -> f64 {
        f64::from_bits(self.parameter_offset_bits)
    }

    pub(crate) const fn tolerance(&self) -> f64 {
        f64::from_bits(self.tolerance_bits)
    }
}

fn endpoint_free_source_circle(
    store: &Store,
    ring: &MixedCylinderCapRing,
) -> Result<(Cylinder, Circle), ProjectedSourceCircleOnPlaneError> {
    let face = store
        .get(ring.side_face().raw())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let SurfaceGeom::Cylinder(cylinder) = store
        .surface(face.surface())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceFaceNotCylinder);
    };
    let loop_ = store
        .get(ring.side_loop())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let fin = store
        .get(ring.side_fin())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let edge = store
        .get(ring.edge())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    if !face.loops().contains(&ring.side_loop())
        || loop_.face() != ring.side_face().raw()
        || loop_.fins() != [ring.side_fin()]
        || fin.parent() != ring.side_loop()
        || fin.edge() != ring.edge()
        || !edge.fins().contains(&ring.side_fin())
        || !edge.fins().contains(&ring.cap_fin())
        || edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
    {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    }
    let Some(curve) = edge.curve() else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceCurveNotCircle);
    };
    let CurveGeom::Circle(circle) = store
        .curve(curve)
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceCurveNotCircle);
    };
    Ok((*cylinder, *circle))
}

fn endpoint_free_source_cylinder_trace(
    store: &Store,
    ring: &MixedCylinderCapRing,
    cylinder: Cylinder,
    circle: Circle,
) -> Result<CylinderLongitudeTrace, ProjectedSourceCircleOnPlaneError> {
    let fin = store
        .get(ring.side_fin())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    cylinder_trace_from_fin(store, fin, cylinder, circle)
}

fn cylinder_trace_from_fin(
    store: &Store,
    fin: &ktopo::entity::Fin,
    cylinder: Cylinder,
    circle: Circle,
) -> Result<CylinderLongitudeTrace, ProjectedSourceCircleOnPlaneError> {
    let pcurve = fin
        .pcurve()
        .ok_or(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch)?;
    if pcurve.range() != circle.param_range()
        || !pcurve.chart().is_identity()
        || pcurve.seam().is_some()
        || pcurve.closure_winding().is_none()
    {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    }
    let Curve2dGeom::Line(line) = store
        .pcurve(pcurve.curve())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    };
    let raw_map = pcurve.edge_to_pcurve();
    let map = AffineParamMap1d::new(raw_map.scale(), raw_map.offset())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
    Ok(CylinderLongitudeTrace::new(cylinder, *line, map))
}

fn endpoint_free_source_plane_trace(
    store: &Store,
    ring: &MixedCylinderCapRing,
) -> Result<PlaneCircleTrace, ProjectedSourceCircleOnPlaneError> {
    let face = store
        .get(ring.cap_face().raw())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let SurfaceGeom::Plane(plane) = store
        .surface(face.surface())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::TargetFaceNotPlane);
    };
    let fin = store
        .get(ring.cap_fin())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let pcurve = fin
        .pcurve()
        .ok_or(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch)?;
    let Curve2dGeom::Circle(circle) = store
        .pcurve(pcurve.curve())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    };
    let raw_map = pcurve.edge_to_pcurve();
    let map = AffineParamMap1d::new(raw_map.scale(), raw_map.offset())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
    Ok(PlaneCircleTrace::new(*plane, *circle, map))
}

fn source_circle(
    store: &Store,
    source_face: &FaceId,
    source_span: &MixedBoundedSourceSpanPlan,
) -> Result<(Cylinder, Circle), ProjectedSourceCircleOnPlaneError> {
    let face = store
        .get(source_face.raw())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let SurfaceGeom::Cylinder(cylinder) = store
        .surface(face.surface())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceFaceNotCylinder);
    };
    let loop_ = store
        .get(source_span.loop_id())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let fin = store
        .get(source_span.fin())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let edge = store
        .get(source_span.edge())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    if !face.loops().contains(&source_span.loop_id())
        || loop_.face() != source_face.raw()
        || loop_.fins() != [source_span.fin()]
        || fin.parent() != source_span.loop_id()
        || fin.edge() != source_span.edge()
        || fin.pcurve().is_none()
        || !edge.fins().contains(&source_span.fin())
        || edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
    {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    }
    let Some(curve) = edge.curve() else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceCurveNotCircle);
    };
    let CurveGeom::Circle(circle) = store
        .curve(curve)
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceCurveNotCircle);
    };
    Ok((*cylinder, *circle))
}

fn source_cylinder_trace(
    store: &Store,
    source_span: &MixedBoundedSourceSpanPlan,
    cylinder: Cylinder,
    circle: Circle,
) -> Result<CylinderLongitudeTrace, ProjectedSourceCircleOnPlaneError> {
    let fin = store
        .get(source_span.fin())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?;
    let pcurve = fin
        .pcurve()
        .ok_or(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch)?;
    if pcurve.range() != circle.param_range()
        || !pcurve.chart().is_identity()
        || pcurve.seam().is_some()
        || pcurve.closure_winding() != Some([1, 0])
    {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    }
    let Curve2dGeom::Line(line) = store
        .pcurve(pcurve.curve())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::StoreRead)?
    else {
        return Err(ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch);
    };
    let raw_map = pcurve.edge_to_pcurve();
    let map = AffineParamMap1d::new(raw_map.scale(), raw_map.offset())
        .map_err(|_| ProjectedSourceCircleOnPlaneError::InvalidProjection)?;
    Ok(CylinderLongitudeTrace::new(cylinder, *line, map))
}

fn project_direction(direction: Vec3, plane: Plane) -> Vec2 {
    Vec2::new(
        direction.dot(plane.frame().x()),
        direction.dot(plane.frame().y()),
    )
}
