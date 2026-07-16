//! Deterministic complete-body rigid copy inside one checked transaction.

use core::fmt;

use crate::entity::{
    Body, BodyId, Curve2dId, CurveId, Edge, EdgeId, EntityRef, Face, Fin, FinId, FinPcurve, Loop,
    LoopId, PointId, Region, Shell, ShellId, SurfaceId, Vertex, VertexId,
};
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::store::Store;
use kcore::error::{CapabilityId, ClassifiedError, Error, ErrorClass, ErrorCode};
use kcore::operation::LimitSnapshot;
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances, check_in_size_box};
use kgeom::curve::{Circle, Ellipse, Line};
use kgeom::curve2d::{Curve2d, Line2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Dir, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    EvalLimits, ExactSurfaceField, IntersectionCertificateError, OffsetSurfaceDescriptor,
    PairedTrace, PlaneCircleTrace, PlaneSphereCircleTrace, SphereLatitudeTrace,
    SphericalCirclePcurve, TransmittedNurbsIntersectionCertificate, TransmittedOffsetNurbsTrace,
    TransmittedOffsetPlaneTrace, TransmittedPlaneIntersectionCertificate,
    VerifiedIntersectionCertificate, VerifiedNurbsIntersectionCertificate,
    certify_paired_plane_line_residuals, certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
    certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals,
    reissue_verified_nurbs_intersection_residuals,
    transmitted_nurbs_intersection_has_rigid_copy_recertifier,
};
use std::collections::HashMap;

/// Failure to copy a complete body under a rigid placement.
///
/// Existing topology, input, and store failures remain kernel errors. Graph
/// certificate reissuance failures retain their exact typed source so callers
/// can distinguish unsupported proof boundaries from rejected geometry.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum BodyCopyError {
    /// Existing topology, input, geometry-construction, or store failure.
    Kernel(Error),
    /// A retained intersection certificate could not be reissued.
    Certificate(IntersectionCertificateError),
}

impl BodyCopyError {
    /// Broad semantic class without erasing the concrete source.
    pub fn class(&self) -> ErrorClass {
        match self {
            Self::Kernel(error) => error.class(),
            Self::Certificate(error) => error.class(),
        }
    }

    /// Stable failure identity delegated from the concrete source.
    pub fn code(&self) -> ErrorCode {
        match self {
            Self::Kernel(error) => error.code(),
            Self::Certificate(error) => error.code(),
        }
    }

    /// Finite capability whose absence caused the failure, when applicable.
    pub fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::Kernel(error) => error.capability(),
            Self::Certificate(error) => error.capability(),
        }
    }

    /// Structured deterministic limit data delegated from the source.
    pub fn limit(&self) -> Option<LimitSnapshot> {
        match self {
            Self::Kernel(error) => error.limit(),
            Self::Certificate(error) => error.limit(),
        }
    }

    pub(crate) fn into_legacy(self) -> Error {
        match self {
            Self::Kernel(error) => error,
            Self::Certificate(_) => Error::InvalidGeometry {
                reason: "rigid body copy could not reissue an intersection certificate",
            },
        }
    }
}

impl fmt::Display for BodyCopyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Kernel(error) => error.fmt(formatter),
            Self::Certificate(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for BodyCopyError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Kernel(error) => Some(error),
            Self::Certificate(error) => Some(error),
        }
    }
}

impl ClassifiedError for BodyCopyError {
    fn class(&self) -> ErrorClass {
        self.class()
    }

    fn code(&self) -> ErrorCode {
        self.code()
    }

    fn capability(&self) -> Option<CapabilityId> {
        self.capability()
    }

    fn limit(&self) -> Option<LimitSnapshot> {
        self.limit()
    }
}

impl From<Error> for BodyCopyError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

impl From<IntersectionCertificateError> for BodyCopyError {
    fn from(error: IntersectionCertificateError) -> Self {
        Self::Certificate(error)
    }
}

/// Result returned by rigid whole-body copy operations.
pub type BodyCopyResult<T> = core::result::Result<T, BodyCopyError>;

pub(crate) struct BodyCopy {
    pub(crate) body: BodyId,
    pub(crate) lineage: Vec<(EntityRef, EntityRef)>,
}

struct Copier<'a> {
    store: &'a mut Store,
    placement: Frame,
    lineage: Vec<(EntityRef, EntityRef)>,
    points: HashMap<PointId, PointId>,
    curves: HashMap<CurveId, CurveId>,
    surfaces: HashMap<SurfaceId, SurfaceId>,
    pcurves: HashMap<Curve2dId, Curve2dId>,
    shells: HashMap<ShellId, ShellId>,
    shell_order: Vec<ShellId>,
    loops: HashMap<LoopId, LoopId>,
    loop_order: Vec<LoopId>,
    fins: HashMap<FinId, FinId>,
    edges: HashMap<EdgeId, EdgeId>,
    edge_order: Vec<EdgeId>,
    vertices: HashMap<VertexId, VertexId>,
}

pub(crate) fn copy_body_rigid(
    store: &mut Store,
    source: BodyId,
    placement: Frame,
) -> BodyCopyResult<BodyCopy> {
    let mut copier = Copier {
        store,
        placement,
        lineage: Vec::new(),
        points: HashMap::new(),
        curves: HashMap::new(),
        surfaces: HashMap::new(),
        pcurves: HashMap::new(),
        shells: HashMap::new(),
        shell_order: Vec::new(),
        loops: HashMap::new(),
        loop_order: Vec::new(),
        fins: HashMap::new(),
        edges: HashMap::new(),
        edge_order: Vec::new(),
        vertices: HashMap::new(),
    };
    copier.copy(source)
}

impl Copier<'_> {
    fn copy(&mut self, source: BodyId) -> BodyCopyResult<BodyCopy> {
        let source_body = self.store.get(source)?.clone();
        let body = self.store.add(Body {
            kind: source_body.kind,
            regions: Vec::new(),
        });
        self.derived(EntityRef::Body(body), EntityRef::Body(source));

        for source_region in source_body.regions {
            let region_value = self.store.get(source_region)?.clone();
            let region = self.store.add(Region {
                body,
                kind: region_value.kind,
                shells: Vec::new(),
            });
            self.store.get_mut(body)?.regions.push(region);
            self.derived(EntityRef::Region(region), EntityRef::Region(source_region));

            for source_shell in region_value.shells {
                let shell_value = self.store.get(source_shell)?.clone();
                let shell = self.store.add(Shell {
                    region,
                    faces: Vec::new(),
                    edges: Vec::new(),
                    vertex: None,
                });
                self.shells.insert(source_shell, shell);
                self.shell_order.push(source_shell);
                self.store.get_mut(region)?.shells.push(shell);
                self.derived(EntityRef::Shell(shell), EntityRef::Shell(source_shell));

                for source_face in shell_value.faces {
                    let face_value = self.store.get(source_face)?.clone();
                    let surface = self.copy_surface(face_value.surface)?;
                    let face = self.store.add(Face {
                        shell,
                        loops: Vec::new(),
                        surface,
                        sense: face_value.sense,
                        domain: face_value.domain,
                        tolerance: face_value.tolerance,
                    });
                    self.store.get_mut(shell)?.faces.push(face);
                    self.derived(EntityRef::Face(face), EntityRef::Face(source_face));

                    for source_loop in face_value.loops {
                        let loop_ = self.store.add(Loop {
                            face,
                            fins: Vec::new(),
                        });
                        self.loops.insert(source_loop, loop_);
                        self.loop_order.push(source_loop);
                        self.store.get_mut(face)?.loops.push(loop_);
                        self.derived(EntityRef::Loop(loop_), EntityRef::Loop(source_loop));
                    }
                }
            }
        }

        for source_vertex in self.store.vertices_of_body(source)? {
            let value = self.store.get(source_vertex)?.clone();
            let point = self.copy_point(value.point)?;
            let vertex = self.store.add(Vertex {
                point,
                tolerance: value.tolerance,
            });
            self.vertices.insert(source_vertex, vertex);
            self.derived(EntityRef::Vertex(vertex), EntityRef::Vertex(source_vertex));
        }

        self.edge_order = self.store.edges_of_body(source)?;
        for source_edge in self.edge_order.clone() {
            let value = self.store.get(source_edge)?.clone();
            let curve = value
                .curve
                .map(|curve| self.copy_curve(curve))
                .transpose()?;
            let edge = self.store.add(Edge {
                curve,
                vertices: value
                    .vertices
                    .map(|vertex| vertex.map(|vertex| self.vertices[&vertex])),
                bounds: value.bounds,
                fins: Vec::new(),
                tolerance: value.tolerance,
            });
            self.edges.insert(source_edge, edge);
            self.derived(EntityRef::Edge(edge), EntityRef::Edge(source_edge));
        }

        for source_loop in self.loop_order.clone() {
            let loop_ = self.loops[&source_loop];
            let source_fins = self.store.get(source_loop)?.fins.clone();
            for source_fin in source_fins {
                let value = self.store.get(source_fin)?.clone();
                let pcurve = value
                    .pcurve
                    .map(|use_| self.copy_pcurve_use(use_))
                    .transpose()?;
                let fin = self.store.add(Fin {
                    parent: loop_,
                    edge: self.edges[&value.edge],
                    sense: value.sense,
                    pcurve,
                });
                self.fins.insert(source_fin, fin);
                self.store.get_mut(loop_)?.fins.push(fin);
                self.derived(EntityRef::Fin(fin), EntityRef::Fin(source_fin));
            }
        }

        for source_edge in self.edge_order.clone() {
            let edge = self.edges[&source_edge];
            let source_fins = self.store.get(source_edge)?.fins.clone();
            self.store.get_mut(edge)?.fins =
                source_fins.into_iter().map(|fin| self.fins[&fin]).collect();
        }
        for source_shell in self.shell_order.clone() {
            let shell = self.shells[&source_shell];
            let value = self.store.get(source_shell)?.clone();
            let mapped_edges = value
                .edges
                .into_iter()
                .map(|edge| self.edges[&edge])
                .collect();
            let mapped_vertex = value.vertex.map(|vertex| self.vertices[&vertex]);
            let target = self.store.get_mut(shell)?;
            target.edges = mapped_edges;
            target.vertex = mapped_vertex;
        }

        Ok(BodyCopy {
            body,
            lineage: core::mem::take(&mut self.lineage),
        })
    }

    fn copy_point(&mut self, source: PointId) -> BodyCopyResult<PointId> {
        if let Some(&point) = self.points.get(&source) {
            return Ok(point);
        }
        let transformed = self.checked_point(*self.store.get(source)?)?;
        let point = self.store.add(transformed);
        self.points.insert(source, point);
        self.derived(EntityRef::Point(point), EntityRef::Point(source));
        Ok(point)
    }

    fn copy_curve(&mut self, source: CurveId) -> BodyCopyResult<CurveId> {
        if let Some(&curve) = self.curves.get(&source) {
            return Ok(curve);
        }
        let descriptor = self.store.get(source)?.clone();
        if let CurveGeom::Intersection(intersection) = &descriptor {
            return match intersection.certificate() {
                VerifiedIntersectionCertificate::PlaneLine(certificate) => self
                    .copy_plane_line_intersection(
                        source,
                        intersection.source_surfaces(),
                        intersection.pcurves(),
                        certificate,
                    ),
                VerifiedIntersectionCertificate::PlaneSphereCircle(certificate) => self
                    .copy_plane_sphere_intersection(
                        source,
                        intersection.source_surfaces(),
                        intersection.pcurves(),
                        certificate,
                    ),
            };
        }
        if let CurveGeom::VerifiedNurbsIntersection(intersection) = &descriptor {
            return self.copy_verified_nurbs_intersection(
                source,
                intersection.source_surfaces(),
                intersection.pcurves(),
                intersection.certificate(),
            );
        }
        if let CurveGeom::TransmittedIntersection(intersection) = &descriptor {
            return self.copy_transmitted_plane_intersection(
                source,
                intersection.source_surfaces(),
                intersection.pcurves(),
                intersection.certificate(),
            );
        }
        if let CurveGeom::TransmittedNurbsIntersection(intersection) = &descriptor {
            return self.copy_transmitted_nurbs_intersection(
                source,
                intersection.source_surfaces(),
                intersection.pcurves(),
                intersection.certificate(),
            );
        }
        let transformed = match descriptor {
            CurveGeom::Line(line) => CurveGeom::Line(Line::new(
                self.checked_point(line.origin())?,
                self.vector(line.dir()),
            )?),
            CurveGeom::Circle(circle) => {
                CurveGeom::Circle(Circle::new(self.frame(*circle.frame())?, circle.radius())?)
            }
            CurveGeom::Ellipse(ellipse) => CurveGeom::Ellipse(Ellipse::new(
                self.frame(*ellipse.frame())?,
                ellipse.major_radius(),
                ellipse.minor_radius(),
            )?),
            CurveGeom::Nurbs(nurbs) => CurveGeom::Nurbs(NurbsCurve::new(
                nurbs.degree(),
                nurbs.knots().as_slice().to_vec(),
                nurbs
                    .points()
                    .iter()
                    .map(|&point| self.checked_point(point))
                    .collect::<BodyCopyResult<Vec<_>>>()?,
                nurbs.weights().map(<[f64]>::to_vec),
            )?),
            CurveGeom::Intersection(_) => unreachable!("handled above"),
            CurveGeom::VerifiedNurbsIntersection(_) => unreachable!("handled above"),
            CurveGeom::TransmittedIntersection(_) | CurveGeom::TransmittedNurbsIntersection(_) => {
                unreachable!("handled above")
            }
            _ => {
                return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "rigid body copy does not support this curve descriptor",
                }));
            }
        };
        let curve = self.store.insert_curve(transformed)?;
        self.curves.insert(source, curve);
        self.derived(EntityRef::Curve(curve), EntityRef::Curve(source));
        Ok(curve)
    }

    fn copy_verified_nurbs_intersection(
        &mut self,
        source: CurveId,
        source_surfaces: [SurfaceId; 2],
        source_pcurves: [Curve2dId; 2],
        certificate: &VerifiedNurbsIntersectionCertificate,
    ) -> BodyCopyResult<CurveId> {
        let copied_surfaces = [
            self.copy_surface(source_surfaces[0])?,
            self.copy_surface(source_surfaces[1])?,
        ];
        let copied_pcurves = [
            self.copy_pcurve(source_pcurves[0])?,
            self.copy_pcurve(source_pcurves[1])?,
        ];
        let copied_nurbs_pcurve = |store: &Store, pcurve: Curve2dId| {
            store
                .get(pcurve)?
                .as_nurbs()
                .cloned()
                .ok_or(Error::InvalidGeometry {
                    reason: "verified NURBS intersection must retain paired NURBS pcurves",
                })
        };
        let pcurves = [
            copied_nurbs_pcurve(self.store, copied_pcurves[0])?,
            copied_nurbs_pcurve(self.store, copied_pcurves[1])?,
        ];
        let carrier = self.transform_nurbs_curve(certificate.carrier())?;
        let traces = [
            self.copied_nurbs_trace(&certificate.traces()[0], copied_surfaces[0])?,
            self.copied_nurbs_trace(&certificate.traces()[1], copied_surfaces[1])?,
        ];
        let certificate = reissue_verified_nurbs_intersection_residuals(
            carrier,
            traces,
            pcurves,
            certificate.tolerance(),
        )
        .map_err(BodyCopyError::Certificate)?;
        let curve = self.store.insert_verified_nurbs_intersection_curve(
            copied_surfaces,
            copied_pcurves,
            certificate,
        )?;
        self.register_curve_copy(source, curve);
        Ok(curve)
    }

    fn copy_transmitted_plane_intersection(
        &mut self,
        source: CurveId,
        source_surfaces: [SurfaceId; 2],
        source_pcurves: [Curve2dId; 2],
        certificate: &TransmittedPlaneIntersectionCertificate,
    ) -> BodyCopyResult<CurveId> {
        let copied_surfaces = [
            self.copy_surface(source_surfaces[0])?,
            self.copy_surface(source_surfaces[1])?,
        ];
        let copied_pcurves = [
            self.copy_pcurve(source_pcurves[0])?,
            self.copy_pcurve(source_pcurves[1])?,
        ];
        let pcurves = self.copied_nurbs_pcurves(copied_pcurves)?;
        let surfaces = [
            self.exact_plane(copied_surfaces[0])?,
            self.exact_plane(copied_surfaces[1])?,
        ];
        let certificate = certify_transmitted_plane_intersection_residuals(
            self.transform_nurbs_curve(certificate.carrier())?,
            surfaces,
            pcurves,
            certificate.metadata(),
            certificate.tolerance(),
        )
        .map_err(BodyCopyError::Certificate)?;
        let curve = self
            .store
            .insert_verified_transmitted_plane_intersection_curve(
                copied_surfaces,
                copied_pcurves,
                certificate,
            )?;
        self.register_curve_copy(source, curve);
        Ok(curve)
    }

    fn copy_transmitted_nurbs_intersection(
        &mut self,
        source: CurveId,
        source_surfaces: [SurfaceId; 2],
        source_pcurves: [Curve2dId; 2],
        certificate: &TransmittedNurbsIntersectionCertificate,
    ) -> BodyCopyResult<CurveId> {
        if !transmitted_nurbs_intersection_has_rigid_copy_recertifier(certificate)
            || !transmitted_nurbs_intersection_sources_are_rigid_copy_supported(
                self.store,
                source_surfaces,
                certificate,
            )?
        {
            return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "rigid body copy cannot rerun this transmitted NURBS certificate family",
            }));
        }
        let copied_surfaces = [
            self.copy_surface(source_surfaces[0])?,
            self.copy_surface(source_surfaces[1])?,
        ];
        let copied_pcurves = [
            self.copy_pcurve(source_pcurves[0])?,
            self.copy_pcurve(source_pcurves[1])?,
        ];
        let pcurves = self.copied_nurbs_pcurves(copied_pcurves)?;
        let quadratic_witnesses = certificate.quadratic_interpolation_witnesses();
        let cubic_witnesses = certificate.cubic_interpolation_witnesses();
        let witness_free_control_count = (quadratic_witnesses.is_none()
            && cubic_witnesses.is_none())
        .then(|| certificate.carrier().points().len());
        let transformed_quadratic_positions = match quadratic_witnesses {
            Some(witnesses) => Some(self.transform_points(witnesses.positions())?),
            None => None,
        };
        let transformed_cubic_positions = match cubic_witnesses {
            Some(witnesses) => Some(self.transform_points(witnesses.positions())?),
            None => None,
        };
        let carrier = match (transformed_quadratic_positions, transformed_cubic_positions) {
            (Some(positions), None) => NurbsCurve::new(
                certificate.carrier().degree(),
                certificate.carrier().knots().as_slice().to_vec(),
                vec![
                    positions[0],
                    positions[1] * 2.0 - (positions[0] + positions[2]) * 0.5,
                    positions[2],
                ],
                None,
            )?,
            (None, Some(positions)) => {
                let first = positions[1] * 27.0 - positions[0] * 8.0 - positions[3];
                let second = positions[2] * 27.0 - positions[0] - positions[3] * 8.0;
                NurbsCurve::new(
                    certificate.carrier().degree(),
                    certificate.carrier().knots().as_slice().to_vec(),
                    vec![
                        positions[0],
                        (first * 2.0 - second) / 18.0,
                        (second * 2.0 - first) / 18.0,
                        positions[3],
                    ],
                    None,
                )?
            }
            (None, None) => self.transform_nurbs_curve(certificate.carrier())?,
            (Some(_), Some(_)) => unreachable!(
                "rigid-copy structural preflight excludes overlapping interpolation witnesses"
            ),
        };
        let traces = [
            self.copied_nurbs_trace(&certificate.traces()[0], copied_surfaces[0])?,
            self.copied_nurbs_trace(&certificate.traces()[1], copied_surfaces[1])?,
        ];
        let metadata = certificate.metadata();
        let tolerance = certificate.tolerance();
        let reissued = match &traces {
            [kgraph::NurbsIntersectionTrace::Plane(_), kgraph::NurbsIntersectionTrace::Nurbs(_)]
            | [kgraph::NurbsIntersectionTrace::Nurbs(_), kgraph::NurbsIntersectionTrace::Plane(_)] => {
                certify_transmitted_plane_nurbs_intersection_residuals(
                    carrier, traces, pcurves, metadata, tolerance,
                )
            }
            [kgraph::NurbsIntersectionTrace::Nurbs(_), kgraph::NurbsIntersectionTrace::Nurbs(_)] => {
                certify_transmitted_nurbs_nurbs_intersection_residuals(
                    carrier, traces, pcurves, metadata, tolerance,
                )
            }
            [kgraph::NurbsIntersectionTrace::OffsetNurbs(_), kgraph::NurbsIntersectionTrace::Nurbs(_)]
            | [kgraph::NurbsIntersectionTrace::Nurbs(_), kgraph::NurbsIntersectionTrace::OffsetNurbs(_)]
            | [kgraph::NurbsIntersectionTrace::OffsetNurbs(_), kgraph::NurbsIntersectionTrace::Plane(_)]
            | [kgraph::NurbsIntersectionTrace::Plane(_), kgraph::NurbsIntersectionTrace::OffsetNurbs(_)] => {
                certify_transmitted_offset_nurbs_intersection_residuals(
                    carrier, traces, pcurves, metadata, tolerance,
                )
            }
            [kgraph::NurbsIntersectionTrace::OffsetNurbs(_), kgraph::NurbsIntersectionTrace::OffsetNurbs(_)] => {
                if let Some(witnesses) = quadratic_witnesses {
                    certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals(
                        carrier,
                        traces,
                        pcurves,
                        transformed_quadratic_positions.expect(
                            "quadratic witness positions were transformed with the carrier",
                        ),
                        witnesses.canonicalized_pcurve_points(),
                        metadata,
                        tolerance,
                    )
                } else if let Some(witnesses) = cubic_witnesses {
                    certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals(
                        carrier,
                        traces,
                        pcurves,
                        transformed_cubic_positions.expect(
                            "cubic witness positions were transformed with the carrier",
                        ),
                        witnesses.canonicalized_pcurve_points(),
                        metadata,
                        tolerance,
                    )
                } else {
                    match witness_free_control_count {
                        Some(5) => certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals(
                            carrier, traces, pcurves, metadata, tolerance,
                        ),
                        Some(7) => certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals(
                            carrier, traces, pcurves, metadata, tolerance,
                        ),
                        _ => certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals(
                            carrier, traces, pcurves, metadata, tolerance,
                        ),
                    }
                }
            }
            _ => return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "rigid body copy cannot rerun this transmitted NURBS certificate family",
            })),
        }
        .map_err(BodyCopyError::Certificate)?;
        let curve = self
            .store
            .insert_verified_transmitted_nurbs_intersection_curve(
                copied_surfaces,
                copied_pcurves,
                reissued,
            )?;
        self.register_curve_copy(source, curve);
        Ok(curve)
    }

    fn copied_nurbs_pcurves(
        &self,
        copied: [Curve2dId; 2],
    ) -> BodyCopyResult<[kgeom::curve2d::NurbsCurve2d; 2]> {
        copied
            .map(|pcurve| {
                self.store
                    .get(pcurve)
                    .map_err(BodyCopyError::Kernel)?
                    .as_nurbs()
                    .cloned()
                    .ok_or(BodyCopyError::Kernel(Error::InvalidGeometry {
                        reason: "transmitted intersection must retain paired NURBS pcurves",
                    }))
            })
            .into_iter()
            .collect::<BodyCopyResult<Vec<_>>>()?
            .try_into()
            .map_err(|_| {
                BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "paired transmitted pcurves must contain two curves",
                })
            })
    }

    fn transform_nurbs_curve(&self, curve: &NurbsCurve) -> BodyCopyResult<NurbsCurve> {
        Ok(NurbsCurve::new(
            curve.degree(),
            curve.knots().as_slice().to_vec(),
            curve
                .points()
                .iter()
                .map(|&point| self.checked_point(point))
                .collect::<BodyCopyResult<Vec<_>>>()?,
            curve.weights().map(<[f64]>::to_vec),
        )?)
    }

    fn transform_points<const N: usize>(
        &self,
        mut points: [Point3; N],
    ) -> BodyCopyResult<[Point3; N]> {
        for point in &mut points {
            *point = self.checked_point(*point)?;
        }
        Ok(points)
    }

    fn transform_nurbs_surface(&self, surface: &NurbsSurface) -> BodyCopyResult<NurbsSurface> {
        let periodic = surface.periodicity().map(|period| period.is_some());
        let transformed = NurbsSurface::new(
            surface.degree_u(),
            surface.degree_v(),
            surface.knots(Dir::U).as_slice().to_vec(),
            surface.knots(Dir::V).as_slice().to_vec(),
            surface
                .points()
                .iter()
                .map(|&point| self.checked_point(point))
                .collect::<BodyCopyResult<Vec<_>>>()?,
            surface.weights().map(<[f64]>::to_vec),
        )?;
        Ok(transformed.with_certified_periodicity(periodic, LINEAR_RESOLUTION)?)
    }

    fn copied_nurbs_trace(
        &self,
        trace: &kgraph::NurbsIntersectionTrace,
        copied_root: SurfaceId,
    ) -> BodyCopyResult<kgraph::NurbsIntersectionTrace> {
        Ok(match trace {
            kgraph::NurbsIntersectionTrace::Plane(_) => {
                kgraph::NurbsIntersectionTrace::Plane(self.exact_plane(copied_root)?)
            }
            kgraph::NurbsIntersectionTrace::Sphere(_) => {
                kgraph::NurbsIntersectionTrace::Sphere(self.exact_sphere(copied_root)?)
            }
            kgraph::NurbsIntersectionTrace::Nurbs(_) => {
                kgraph::NurbsIntersectionTrace::Nurbs(
                    self.store.get(copied_root)?.as_nurbs().cloned().ok_or(
                        Error::InvalidGeometry {
                            reason: "verified direct NURBS trace must retain a direct NURBS source",
                        },
                    )?,
                )
            }
            kgraph::NurbsIntersectionTrace::OffsetNurbs(offset) => {
                let mut basis = copied_root;
                let copied_basis = loop {
                    match self.store.get(basis)? {
                        SurfaceGeom::Offset(descriptor) => basis = descriptor.basis(),
                        SurfaceGeom::Nurbs(surface) => break surface.clone(),
                        _ => {
                            return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                                reason: "verified offset-NURBS trace must retain a complete NURBS basis chain",
                            }));
                        }
                    }
                };
                kgraph::NurbsIntersectionTrace::OffsetNurbs(
                    TransmittedOffsetNurbsTrace::from_descriptor_signed_distances(
                        copied_basis,
                        offset.descriptor_signed_distances(),
                    )
                    .ok_or(Error::InvalidGeometry {
                        reason: "verified offset-NURBS trace has an invalid retained descriptor chain",
                    })?,
                )
            }
            kgraph::NurbsIntersectionTrace::OffsetPlane(offset) => {
                let descriptor = self.store.get(copied_root)?.as_offset().copied().ok_or(
                    Error::InvalidGeometry {
                        reason: "verified offset-Plane trace must retain one offset descriptor",
                    },
                )?;
                let basis = self
                    .store
                    .get(descriptor.basis())?
                    .as_plane()
                    .copied()
                    .ok_or(Error::InvalidGeometry {
                        reason: "verified offset-Plane trace must retain a direct Plane basis",
                    })?;
                kgraph::NurbsIntersectionTrace::OffsetPlane(TransmittedOffsetPlaneTrace::new(
                    basis,
                    offset.signed_distance(),
                ))
            }
        })
    }

    fn copy_plane_line_intersection(
        &mut self,
        source: CurveId,
        source_surfaces: [SurfaceId; 2],
        source_pcurves: [Curve2dId; 2],
        certificate: kgraph::PairedPlaneLineResidualCertificate,
    ) -> BodyCopyResult<CurveId> {
        let source_fields = [
            self.exact_surface_field(source_surfaces[0])?,
            self.exact_surface_field(source_surfaces[1])?,
        ];
        if !source_fields
            .into_iter()
            .zip(certificate.surfaces())
            .all(|(field, certified)| field == ExactSurfaceField::Plane(certified))
        {
            return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "Plane/Plane certificate must retain safe exact Plane fields",
            }));
        }
        let copied_surfaces = [
            self.copy_surface(source_surfaces[0])?,
            self.copy_surface(source_surfaces[1])?,
        ];
        let surfaces = [
            self.exact_plane(copied_surfaces[0])?,
            self.exact_plane(copied_surfaces[1])?,
        ];
        let copied_pcurves = [
            self.copy_pcurve(source_pcurves[0])?,
            self.copy_pcurve(source_pcurves[1])?,
        ];
        let copied_line = |store: &Store, pcurve: Curve2dId| -> BodyCopyResult<Line2d> {
            store
                .get(pcurve)
                .map_err(BodyCopyError::Kernel)?
                .as_line()
                .copied()
                .ok_or(BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "Plane/Plane certificate must retain line pcurves",
                }))
        };
        let pcurves = [
            copied_line(self.store, copied_pcurves[0])?,
            copied_line(self.store, copied_pcurves[1])?,
        ];
        let carrier = certificate.carrier();
        let carrier = Line::new(
            self.checked_point(carrier.origin())?,
            self.vector(carrier.dir()),
        )?;
        let certificate = certify_paired_plane_line_residuals(
            carrier,
            certificate.carrier_range(),
            surfaces,
            pcurves,
            certificate.parameter_maps(),
            certificate.tolerance(),
        )
        .map_err(BodyCopyError::Certificate)?;
        let curve = self.store.insert_verified_plane_intersection_curve(
            copied_surfaces,
            copied_pcurves,
            certificate,
        )?;
        self.register_curve_copy(source, curve);
        Ok(curve)
    }

    fn copy_plane_sphere_intersection(
        &mut self,
        source: CurveId,
        source_surfaces: [SurfaceId; 2],
        source_pcurves: [Curve2dId; 2],
        certificate: kgraph::PairedPlaneSphereCircleResidualCertificate,
    ) -> BodyCopyResult<CurveId> {
        for (surface, trace) in source_surfaces.into_iter().zip(certificate.traces()) {
            let field = self.exact_surface_field(surface)?;
            let matches = match trace {
                PlaneSphereCircleTrace::Plane(trace) => {
                    field == ExactSurfaceField::Plane(trace.surface())
                }
                PlaneSphereCircleTrace::Sphere(trace) => {
                    field == ExactSurfaceField::Sphere(trace.surface())
                }
                PlaneSphereCircleTrace::SphereOblique(trace) => {
                    field == ExactSurfaceField::Sphere(trace.surface())
                }
            };
            if !matches {
                return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "Plane/Sphere certificate must retain safe exact Plane/Sphere fields",
                }));
            }
        }
        let copied_surfaces = [
            self.copy_surface(source_surfaces[0])?,
            self.copy_surface(source_surfaces[1])?,
        ];
        let carrier = certificate.carrier();
        let carrier = Circle::new(self.frame(*carrier.frame())?, carrier.radius())?;
        let traces = certificate.traces();

        let (copied_pcurves, certificate) = match traces {
            [
                PlaneSphereCircleTrace::Plane(_),
                PlaneSphereCircleTrace::Sphere(_),
            ]
            | [
                PlaneSphereCircleTrace::Sphere(_),
                PlaneSphereCircleTrace::Plane(_),
            ] => {
                let copied_pcurves = [
                    self.copy_pcurve(source_pcurves[0])?,
                    self.copy_pcurve(source_pcurves[1])?,
                ];
                let traces = [
                    self.copied_plane_sphere_trace(
                        traces[0],
                        copied_surfaces[0],
                        copied_pcurves[0],
                    )?,
                    self.copied_plane_sphere_trace(
                        traces[1],
                        copied_surfaces[1],
                        copied_pcurves[1],
                    )?,
                ];
                let certificate = certify_paired_plane_sphere_circle_residuals(
                    carrier,
                    certificate.carrier_range(),
                    traces,
                    certificate.tolerance(),
                )
                .map_err(BodyCopyError::Certificate)?;
                (copied_pcurves, certificate)
            }
            [
                PlaneSphereCircleTrace::Plane(_),
                PlaneSphereCircleTrace::SphereOblique(sphere),
            ]
            | [
                PlaneSphereCircleTrace::SphereOblique(sphere),
                PlaneSphereCircleTrace::Plane(_),
            ] => {
                let plane_first = matches!(traces[0], PlaneSphereCircleTrace::Plane(_));
                let plane_index = usize::from(!plane_first);
                let sphere_index = usize::from(plane_first);
                let copied_plane = self.exact_plane(copied_surfaces[plane_index])?;
                let copied_sphere = self.exact_sphere(copied_surfaces[sphere_index])?;
                let copied_plane_pcurve = self.copy_pcurve(source_pcurves[plane_index])?;
                let plane_pcurve = self
                    .store
                    .get(copied_plane_pcurve)?
                    .as_circle()
                    .copied()
                    .ok_or(Error::InvalidGeometry {
                        reason: "Plane/Sphere certificate must retain a circle plane pcurve",
                    })?;
                let source_sphere_pcurve = sphere.pcurve();
                let range = certificate.carrier_range();
                let endpoint_longitudes = [
                    source_sphere_pcurve.eval(range.lo).x,
                    source_sphere_pcurve.eval(range.hi).x,
                ];
                let (sphere_pcurve, certificate) =
                    certify_paired_plane_sphere_oblique_circle_residuals(
                        carrier,
                        range,
                        copied_plane,
                        plane_pcurve,
                        copied_sphere,
                        source_sphere_pcurve.chart_window(),
                        endpoint_longitudes,
                        if plane_first {
                            PairedTrace::First
                        } else {
                            PairedTrace::Second
                        },
                        certificate.tolerance(),
                    )
                    .map_err(BodyCopyError::Certificate)?;
                let copied_sphere_pcurve =
                    self.copy_spherical_pcurve(source_pcurves[sphere_index], sphere_pcurve)?;
                let mut copied_pcurves = [copied_plane_pcurve; 2];
                copied_pcurves[sphere_index] = copied_sphere_pcurve;
                copied_pcurves[plane_index] = copied_plane_pcurve;
                (copied_pcurves, certificate)
            }
            _ => {
                return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "Plane/Sphere certificate must retain one Plane and one Sphere trace",
                }));
            }
        };

        let curve = self.store.insert_verified_plane_sphere_intersection_curve(
            copied_surfaces,
            copied_pcurves,
            certificate,
        )?;
        self.register_curve_copy(source, curve);
        Ok(curve)
    }

    fn copied_plane_sphere_trace(
        &self,
        source: PlaneSphereCircleTrace,
        copied_surface: SurfaceId,
        copied_pcurve: Curve2dId,
    ) -> BodyCopyResult<PlaneSphereCircleTrace> {
        match source {
            PlaneSphereCircleTrace::Plane(trace) => {
                Ok(PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(
                    self.exact_plane(copied_surface)?,
                    self.store.get(copied_pcurve)?.as_circle().copied().ok_or(
                        Error::InvalidGeometry {
                            reason: "Plane/Sphere certificate must retain a circle plane pcurve",
                        },
                    )?,
                    trace.parameter_map(),
                )))
            }
            PlaneSphereCircleTrace::Sphere(trace) => {
                Ok(PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(
                    self.exact_sphere(copied_surface)?,
                    self.store.get(copied_pcurve)?.as_line().copied().ok_or(
                        Error::InvalidGeometry {
                            reason: "Plane/Sphere certificate must retain a line sphere pcurve",
                        },
                    )?,
                    trace.parameter_map(),
                )))
            }
            PlaneSphereCircleTrace::SphereOblique(_) => {
                Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "oblique Plane/Sphere traces require a regenerated spherical pcurve",
                }))
            }
        }
    }

    fn exact_surface_field(&self, surface: SurfaceId) -> BodyCopyResult<ExactSurfaceField> {
        let mut evaluator = self
            .store
            .eval_context(EvalLimits::default(), Tolerances::default());
        evaluator
            .surface_exact_field(surface)
            .map_err(|_| Error::InvalidGeometry {
                reason: "verified intersection source exceeds the supported safe offset-field boundary",
            })?
            .ok_or(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "verified intersection source is not a safe exact Plane/Sphere field",
            }))
    }

    fn exact_plane(&self, surface: SurfaceId) -> BodyCopyResult<Plane> {
        match self.exact_surface_field(surface)? {
            ExactSurfaceField::Plane(plane) => Ok(plane),
            ExactSurfaceField::Sphere(_) => Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "verified intersection source must retain an exact Plane field",
            })),
        }
    }

    fn exact_sphere(&self, surface: SurfaceId) -> BodyCopyResult<Sphere> {
        match self.exact_surface_field(surface)? {
            ExactSurfaceField::Sphere(sphere) => Ok(sphere),
            ExactSurfaceField::Plane(_) => Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "Plane/Sphere certificate must retain an exact Sphere field",
            })),
        }
    }

    fn copy_spherical_pcurve(
        &mut self,
        source: Curve2dId,
        pcurve: SphericalCirclePcurve,
    ) -> BodyCopyResult<Curve2dId> {
        if let Some(&copied) = self.pcurves.get(&source) {
            if self.store.get(copied)?.as_spherical_circle().copied() == Some(pcurve) {
                return Ok(copied);
            }
            return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                reason: "shared spherical pcurve copy does not match the reissued certificate",
            }));
        }
        let copied = self
            .store
            .insert_pcurve(crate::geom::Curve2dGeom::SphericalCircle(pcurve))?;
        self.pcurves.insert(source, copied);
        self.derived(EntityRef::Curve2d(copied), EntityRef::Curve2d(source));
        Ok(copied)
    }

    fn register_curve_copy(&mut self, source: CurveId, copied: CurveId) {
        self.curves.insert(source, copied);
        self.derived(EntityRef::Curve(copied), EntityRef::Curve(source));
    }

    fn copy_surface(&mut self, source: SurfaceId) -> BodyCopyResult<SurfaceId> {
        if let Some(&surface) = self.surfaces.get(&source) {
            return Ok(surface);
        }
        let descriptor = self.store.get(source)?.clone();
        let transformed = match descriptor {
            SurfaceGeom::Plane(surface) => {
                SurfaceGeom::Plane(Plane::new(self.frame(*surface.frame())?))
            }
            SurfaceGeom::Cylinder(surface) => SurfaceGeom::Cylinder(Cylinder::new(
                self.frame(*surface.frame())?,
                surface.radius(),
            )?),
            SurfaceGeom::Cone(surface) => SurfaceGeom::Cone(Cone::new(
                self.frame(*surface.frame())?,
                surface.radius(),
                surface.half_angle(),
            )?),
            SurfaceGeom::Sphere(surface) => SurfaceGeom::Sphere(Sphere::new(
                self.frame(*surface.frame())?,
                surface.radius(),
            )?),
            SurfaceGeom::Torus(surface) => SurfaceGeom::Torus(Torus::new(
                self.frame(*surface.frame())?,
                surface.major_radius(),
                surface.minor_radius(),
            )?),
            SurfaceGeom::Nurbs(surface) => {
                SurfaceGeom::Nurbs(self.transform_nurbs_surface(&surface)?)
            }
            SurfaceGeom::Offset(offset) => {
                let basis = self.copy_surface(offset.basis())?;
                SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
                    basis,
                    offset.signed_distance(),
                ))
            }
            _ => {
                return Err(BodyCopyError::Kernel(Error::InvalidGeometry {
                    reason: "rigid body copy does not support this surface descriptor",
                }));
            }
        };
        let surface = self.store.insert_surface(transformed)?;
        self.surfaces.insert(source, surface);
        self.derived(EntityRef::Surface(surface), EntityRef::Surface(source));
        Ok(surface)
    }

    fn copy_pcurve_use(&mut self, source: FinPcurve) -> BodyCopyResult<FinPcurve> {
        let curve = self.copy_pcurve(source.curve())?;
        let mut copy = FinPcurve::new(curve, source.range(), source.edge_to_pcurve())?
            .with_chart(source.chart())
            .with_endpoint_kinds(source.endpoint_kinds());
        if let Some(winding) = source.closure_winding() {
            copy = copy.with_closure_winding(winding);
        }
        if let Some(seam) = source.seam() {
            copy = copy.with_seam(seam);
        }
        Ok(copy)
    }

    fn copy_pcurve(&mut self, source: Curve2dId) -> BodyCopyResult<Curve2dId> {
        if let Some(&curve) = self.pcurves.get(&source) {
            return Ok(curve);
        }
        let curve = self.store.insert_pcurve(self.store.get(source)?.clone())?;
        self.pcurves.insert(source, curve);
        self.derived(EntityRef::Curve2d(curve), EntityRef::Curve2d(source));
        Ok(curve)
    }

    fn point(&self, point: Point3) -> Point3 {
        self.placement.point_at(point.x, point.y, point.z)
    }

    fn checked_point(&self, point: Point3) -> BodyCopyResult<Point3> {
        let transformed = self.point(point);
        check_in_size_box(transformed.to_array())?;
        Ok(transformed)
    }

    fn vector(&self, vector: Vec3) -> Vec3 {
        self.placement.x() * vector.x
            + self.placement.y() * vector.y
            + self.placement.z() * vector.z
    }

    fn frame(&self, frame: Frame) -> BodyCopyResult<Frame> {
        let origin = self.point(frame.origin());
        check_in_size_box(origin.to_array())?;
        Ok(Frame::new(
            origin,
            self.vector(frame.z()),
            self.vector(frame.x()),
        )?)
    }

    fn derived(&mut self, derived: EntityRef, source: EntityRef) {
        self.lineage.push((derived, source));
    }
}

// Keep this live-root predicate synchronized with the kernel facade preflight.
// Sharing it would expose ktopo's private raw Store/body-copy seam publicly.
fn transmitted_nurbs_intersection_sources_are_rigid_copy_supported(
    store: &Store,
    source_surfaces: [SurfaceId; 2],
    certificate: &TransmittedNurbsIntersectionCertificate,
) -> BodyCopyResult<bool> {
    if !certificate
        .traces()
        .iter()
        .any(|trace| matches!(trace, kgraph::NurbsIntersectionTrace::OffsetNurbs(_)))
    {
        return Ok(true);
    }
    let mut terminal_offset_bases = [None; 2];
    for (index, (source, trace)) in source_surfaces
        .into_iter()
        .zip(certificate.traces())
        .enumerate()
    {
        let source = store.get(source)?;
        let matches = match trace {
            kgraph::NurbsIntersectionTrace::OffsetNurbs(offset) => {
                let Some(terminal) =
                    matched_offset_nurbs_terminal(store, source_surfaces[index], offset)?
                else {
                    return Ok(false);
                };
                terminal_offset_bases[index] = Some(terminal);
                true
            }
            kgraph::NurbsIntersectionTrace::Plane(plane) => {
                source.as_plane().is_some_and(|actual| actual == plane)
            }
            kgraph::NurbsIntersectionTrace::Nurbs(nurbs) => {
                source.as_nurbs().is_some_and(|actual| actual == nurbs)
            }
            _ => false,
        };
        if !matches {
            return Ok(false);
        }
    }
    Ok(match terminal_offset_bases {
        [Some(first), Some(second)] => source_surfaces[0] != source_surfaces[1] && first != second,
        _ => true,
    })
}

fn matched_offset_nurbs_terminal(
    store: &Store,
    source: SurfaceId,
    trace: &TransmittedOffsetNurbsTrace,
) -> BodyCopyResult<Option<SurfaceId>> {
    let mut current = source;
    for &expected in trace.descriptor_signed_distances() {
        let Some(descriptor) = store.get(current)?.as_offset().copied() else {
            return Ok(None);
        };
        if descriptor.signed_distance().to_bits() != expected.to_bits() {
            return Ok(None);
        }
        current = descriptor.basis();
    }
    Ok(store
        .get(current)?
        .as_nurbs()
        .is_some_and(|basis| basis == trace.basis())
        .then_some(current))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::error::Error as _;

    #[test]
    fn certificate_failures_delegate_classification_and_source() {
        let cases = [
            (
                IntersectionCertificateError::HarmonicRootClassification,
                ErrorClass::Unsupported,
                "kgraph.intersection-certificate.harmonic-root-classification",
                Some("kgraph.intersection-certificate.harmonic-root-classification"),
            ),
            (
                IntersectionCertificateError::SingularSphereChart {
                    squared_pole_clearance: 0.0,
                },
                ErrorClass::Unsupported,
                "kgraph.intersection-certificate.singular-sphere-chart",
                Some("kgraph.intersection-certificate.regular-sphere-chart"),
            ),
        ];

        for (certificate, class, code, capability) in cases {
            let error = BodyCopyError::from(certificate.clone());
            assert_eq!(error, BodyCopyError::Certificate(certificate.clone()));
            assert_eq!(ClassifiedError::class(&error), class);
            assert_eq!(ClassifiedError::code(&error).as_str(), code);
            assert_eq!(
                ClassifiedError::capability(&error).map(CapabilityId::as_str),
                capability
            );
            assert_eq!(ClassifiedError::limit(&error), None);
            assert_eq!(
                error
                    .source()
                    .and_then(|source| source.downcast_ref::<IntersectionCertificateError>()),
                Some(&certificate)
            );
        }
    }

    #[test]
    fn kernel_failures_retain_their_variant_and_source() {
        let source = Error::StaleHandle;
        let error = BodyCopyError::from(source.clone());
        assert_eq!(error, BodyCopyError::Kernel(source.clone()));
        assert_eq!(ClassifiedError::class(&error), source.class());
        assert_eq!(ClassifiedError::code(&error), source.code());
        assert_eq!(ClassifiedError::capability(&error), source.capability());
        assert_eq!(
            error
                .source()
                .and_then(|source| source.downcast_ref::<Error>()),
            Some(&source)
        );
    }
}
