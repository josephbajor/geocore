//! Deterministic complete-body rigid copy inside one checked transaction.

use crate::entity::{
    Body, BodyId, Curve2dId, CurveId, Edge, EdgeId, EntityRef, Face, Fin, FinId, FinPcurve, Loop,
    LoopId, PointId, Region, Shell, ShellId, SurfaceId, Vertex, VertexId,
};
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::curve::{Circle, Ellipse, Line};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::surface::{Cone, Cylinder, Dir, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Point3, Vec3};
use kgraph::OffsetSurfaceDescriptor;
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
            CurveGeom::Intersection(_)
            | CurveGeom::VerifiedNurbsIntersection(_)
            | CurveGeom::TransmittedIntersection(_)
            | CurveGeom::TransmittedNurbsIntersection(_) => {
                return Err(Error::InvalidGeometry {
                    reason: "rigid body copy does not yet reissue verified intersection certificates",
                });
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
                SurfaceGeom::Nurbs(
                    transformed.with_certified_periodicity(periodic, LINEAR_RESOLUTION)?,
                )
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
