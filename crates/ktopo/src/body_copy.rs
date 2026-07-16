//! Deterministic complete-body rigid copy inside one checked transaction.

use crate::entity::{
    Body, BodyId, Curve2dId, CurveId, Edge, EdgeId, EntityRef, Face, Fin, FinId, FinPcurve, Loop,
    LoopId, PointId, Region, Shell, ShellId, SurfaceId, Vertex, VertexId,
};
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances, check_in_size_box};
use kgeom::curve::{Circle, Ellipse, Line};
use kgeom::curve2d::{Curve2d, Line2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Dir, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    EvalLimits, ExactSurfaceField, OffsetSurfaceDescriptor, PairedTrace, PlaneCircleTrace,
    PlaneSphereCircleTrace, SphereLatitudeTrace, SphericalCirclePcurve,
    TransmittedNurbsIntersectionCertificate, TransmittedOffsetNurbsTrace,
    TransmittedOffsetPlaneTrace, TransmittedPlaneIntersectionCertificate,
    VerifiedIntersectionCertificate, VerifiedNurbsIntersectionCertificate,
    certify_paired_plane_line_residuals, certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
    certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals,
    reissue_verified_nurbs_intersection_residuals,
    transmitted_nurbs_intersection_has_rigid_copy_recertifier,
};
use std::collections::HashMap;

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
) -> Result<BodyCopy> {
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
    fn copy(&mut self, source: BodyId) -> Result<BodyCopy> {
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

    fn copy_point(&mut self, source: PointId) -> Result<PointId> {
        if let Some(&point) = self.points.get(&source) {
            return Ok(point);
        }
        let transformed = self.checked_point(*self.store.get(source)?)?;
        let point = self.store.add(transformed);
        self.points.insert(source, point);
        self.derived(EntityRef::Point(point), EntityRef::Point(source));
        Ok(point)
    }

    fn copy_curve(&mut self, source: CurveId) -> Result<CurveId> {
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
                    .collect::<Result<Vec<_>>>()?,
                nurbs.weights().map(<[f64]>::to_vec),
            )?),
            CurveGeom::Intersection(_) => unreachable!("handled above"),
            CurveGeom::VerifiedNurbsIntersection(_) => unreachable!("handled above"),
            CurveGeom::TransmittedIntersection(_) | CurveGeom::TransmittedNurbsIntersection(_) => {
                unreachable!("handled above")
            }
            _ => {
                return Err(Error::InvalidGeometry {
                    reason: "rigid body copy does not support this curve descriptor",
                });
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
    ) -> Result<CurveId> {
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
        .map_err(|_| Error::InvalidGeometry {
            reason: "rigid body copy could not reissue the verified NURBS intersection certificate",
        })?;
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
    ) -> Result<CurveId> {
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
        .map_err(|_| Error::InvalidGeometry {
            reason: "rigid body copy could not reissue the transmitted Plane intersection certificate",
        })?;
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
    ) -> Result<CurveId> {
        if !transmitted_nurbs_intersection_has_rigid_copy_recertifier(certificate)
            || !transmitted_nurbs_intersection_sources_are_rigid_copy_supported(
                self.store,
                source_surfaces,
                certificate,
            )?
        {
            return Err(Error::InvalidGeometry {
                reason: "rigid body copy cannot rerun this transmitted NURBS certificate family",
            });
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
        let carrier = self.transform_nurbs_curve(certificate.carrier())?;
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
                certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals(
                    carrier, traces, pcurves, metadata, tolerance,
                )
            }
            _ => return Err(Error::InvalidGeometry {
                reason: "rigid body copy cannot rerun this transmitted NURBS certificate family",
            }),
        }
        .map_err(|_| Error::InvalidGeometry {
            reason: "rigid body copy could not reissue the transmitted NURBS intersection certificate",
        })?;
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
    ) -> Result<[kgeom::curve2d::NurbsCurve2d; 2]> {
        copied
            .map(|pcurve| {
                self.store
                    .get(pcurve)?
                    .as_nurbs()
                    .cloned()
                    .ok_or(Error::InvalidGeometry {
                        reason: "transmitted intersection must retain paired NURBS pcurves",
                    })
            })
            .into_iter()
            .collect::<Result<Vec<_>>>()?
            .try_into()
            .map_err(|_| Error::InvalidGeometry {
                reason: "paired transmitted pcurves must contain two curves",
            })
    }

    fn transform_nurbs_curve(&self, curve: &NurbsCurve) -> Result<NurbsCurve> {
        NurbsCurve::new(
            curve.degree(),
            curve.knots().as_slice().to_vec(),
            curve
                .points()
                .iter()
                .map(|&point| self.checked_point(point))
                .collect::<Result<Vec<_>>>()?,
            curve.weights().map(<[f64]>::to_vec),
        )
    }

    fn transform_nurbs_surface(&self, surface: &NurbsSurface) -> Result<NurbsSurface> {
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
                .collect::<Result<Vec<_>>>()?,
            surface.weights().map(<[f64]>::to_vec),
        )?;
        transformed.with_certified_periodicity(periodic, LINEAR_RESOLUTION)
    }

    fn copied_nurbs_trace(
        &self,
        trace: &kgraph::NurbsIntersectionTrace,
        copied_root: SurfaceId,
    ) -> Result<kgraph::NurbsIntersectionTrace> {
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
                            return Err(Error::InvalidGeometry {
                                reason: "verified offset-NURBS trace must retain a complete NURBS basis chain",
                            });
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
    ) -> Result<CurveId> {
        let source_fields = [
            self.exact_surface_field(source_surfaces[0])?,
            self.exact_surface_field(source_surfaces[1])?,
        ];
        if !source_fields
            .into_iter()
            .zip(certificate.surfaces())
            .all(|(field, certified)| field == ExactSurfaceField::Plane(certified))
        {
            return Err(Error::InvalidGeometry {
                reason: "Plane/Plane certificate must retain safe exact Plane fields",
            });
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
        let copied_line = |store: &Store, pcurve: Curve2dId| -> Result<Line2d> {
            store
                .get(pcurve)?
                .as_line()
                .copied()
                .ok_or(Error::InvalidGeometry {
                    reason: "Plane/Plane certificate must retain line pcurves",
                })
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
        .map_err(|_| Error::InvalidGeometry {
            reason: "rigid body copy could not reissue the Plane/Plane intersection certificate",
        })?;
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
    ) -> Result<CurveId> {
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
                return Err(Error::InvalidGeometry {
                    reason: "Plane/Sphere certificate must retain safe exact Plane/Sphere fields",
                });
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
                .map_err(|_| Error::InvalidGeometry {
                    reason: "rigid body copy could not reissue the Plane/Sphere intersection certificate",
                })?;
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
                    .map_err(|_| Error::InvalidGeometry {
                        reason: "rigid body copy could not reissue the oblique Plane/Sphere intersection certificate",
                    })?;
                let copied_sphere_pcurve =
                    self.copy_spherical_pcurve(source_pcurves[sphere_index], sphere_pcurve)?;
                let mut copied_pcurves = [copied_plane_pcurve; 2];
                copied_pcurves[sphere_index] = copied_sphere_pcurve;
                copied_pcurves[plane_index] = copied_plane_pcurve;
                (copied_pcurves, certificate)
            }
            _ => {
                return Err(Error::InvalidGeometry {
                    reason: "Plane/Sphere certificate must retain one Plane and one Sphere trace",
                });
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
    ) -> Result<PlaneSphereCircleTrace> {
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
            PlaneSphereCircleTrace::SphereOblique(_) => Err(Error::InvalidGeometry {
                reason: "oblique Plane/Sphere traces require a regenerated spherical pcurve",
            }),
        }
    }

    fn exact_surface_field(&self, surface: SurfaceId) -> Result<ExactSurfaceField> {
        let mut evaluator = self
            .store
            .eval_context(EvalLimits::default(), Tolerances::default());
        evaluator
            .surface_exact_field(surface)
            .map_err(|_| Error::InvalidGeometry {
                reason: "verified intersection source exceeds the supported safe offset-field boundary",
            })?
            .ok_or(Error::InvalidGeometry {
                reason: "verified intersection source is not a safe exact Plane/Sphere field",
            })
    }

    fn exact_plane(&self, surface: SurfaceId) -> Result<Plane> {
        match self.exact_surface_field(surface)? {
            ExactSurfaceField::Plane(plane) => Ok(plane),
            ExactSurfaceField::Sphere(_) => Err(Error::InvalidGeometry {
                reason: "verified intersection source must retain an exact Plane field",
            }),
        }
    }

    fn exact_sphere(&self, surface: SurfaceId) -> Result<Sphere> {
        match self.exact_surface_field(surface)? {
            ExactSurfaceField::Sphere(sphere) => Ok(sphere),
            ExactSurfaceField::Plane(_) => Err(Error::InvalidGeometry {
                reason: "Plane/Sphere certificate must retain an exact Sphere field",
            }),
        }
    }

    fn copy_spherical_pcurve(
        &mut self,
        source: Curve2dId,
        pcurve: SphericalCirclePcurve,
    ) -> Result<Curve2dId> {
        if let Some(&copied) = self.pcurves.get(&source) {
            if self.store.get(copied)?.as_spherical_circle().copied() == Some(pcurve) {
                return Ok(copied);
            }
            return Err(Error::InvalidGeometry {
                reason: "shared spherical pcurve copy does not match the reissued certificate",
            });
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

    fn copy_surface(&mut self, source: SurfaceId) -> Result<SurfaceId> {
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
                return Err(Error::InvalidGeometry {
                    reason: "rigid body copy does not support this surface descriptor",
                });
            }
        };
        let surface = self.store.insert_surface(transformed)?;
        self.surfaces.insert(source, surface);
        self.derived(EntityRef::Surface(surface), EntityRef::Surface(source));
        Ok(surface)
    }

    fn copy_pcurve_use(&mut self, source: FinPcurve) -> Result<FinPcurve> {
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

    fn copy_pcurve(&mut self, source: Curve2dId) -> Result<Curve2dId> {
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

    fn checked_point(&self, point: Point3) -> Result<Point3> {
        let transformed = self.point(point);
        check_in_size_box(transformed.to_array())?;
        Ok(transformed)
    }

    fn vector(&self, vector: Vec3) -> Vec3 {
        self.placement.x() * vector.x
            + self.placement.y() * vector.y
            + self.placement.z() * vector.z
    }

    fn frame(&self, frame: Frame) -> Result<Frame> {
        let origin = self.point(frame.origin());
        check_in_size_box(origin.to_array())?;
        Frame::new(origin, self.vector(frame.z()), self.vector(frame.x()))
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
) -> Result<bool> {
    if !certificate
        .traces()
        .iter()
        .any(|trace| matches!(trace, kgraph::NurbsIntersectionTrace::OffsetNurbs(_)))
    {
        return Ok(true);
    }
    if matches!(
        certificate.traces(),
        [
            kgraph::NurbsIntersectionTrace::OffsetNurbs(_),
            kgraph::NurbsIntersectionTrace::OffsetNurbs(_)
        ]
    ) {
        if source_surfaces[0] == source_surfaces[1] {
            return Ok(false);
        }
        let Some(first) = store.get(source_surfaces[0])?.as_offset().copied() else {
            return Ok(false);
        };
        let Some(second) = store.get(source_surfaces[1])?.as_offset().copied() else {
            return Ok(false);
        };
        if first.basis() == second.basis() {
            return Ok(false);
        }
    }
    for (source, trace) in source_surfaces.into_iter().zip(certificate.traces()) {
        let source = store.get(source)?;
        let matches = match trace {
            kgraph::NurbsIntersectionTrace::OffsetNurbs(offset) => {
                let distances = offset.descriptor_signed_distances();
                let Some(descriptor) = source.as_offset().copied() else {
                    return Ok(false);
                };
                distances.len() == 1
                    && descriptor.signed_distance() == distances[0]
                    && store
                        .get(descriptor.basis())?
                        .as_nurbs()
                        .is_some_and(|basis| basis == offset.basis())
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
    Ok(true)
}
