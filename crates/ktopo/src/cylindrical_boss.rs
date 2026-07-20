//! Semantic assembly of one capped cylindrical segment on a convex planar host.
//!
//! The host is supplied through [`PlanarSolidInput`]. One designated convex
//! face is punctured by a strictly contained circular loop. Exactly one axial
//! endpoint must lie on that face. The other endpoint may be on its outward
//! side, forming a boss, or its inward side, forming a blind pocket. This
//! distinction is derived from exact endpoint incidence and the oriented host
//! support plane; it is not a caller-supplied operation tag. All
//! caller-controlled geometry, convexity, incidence, containment, size-box,
//! and optional lineage data are preflighted before a body scaffold is
//! allocated.

use std::collections::BTreeMap;

use crate::entity::{
    BodyId, Edge, EdgeId, EntityRef, Face, FaceDomain, FaceId, Fin, FinPcurve, Loop, LoopId,
    ParamMap1d, Sense, ShellId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::loop_proof::certify_convex_polygon_circle_containment;
use crate::planar::{PlanarSolidInput, PreparedSolid};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3};
use kcore::tolerance::check_in_size_box;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};

/// Complete semantic input for one capped cylindrical host feature.
///
/// `port_face` indexes `host.faces()`. Exactly one endpoint of `axial_range`
/// must be incident with that face. The other endpoint's exact position
/// relative to the face's oriented support plane determines whether the
/// feature is an outward boss or an inward blind pocket. Optional source faces
/// are semantic lineage for the cylindrical side and closing cap; planar host
/// lineage remains owned by [`PlanarSolidInput`].
#[derive(Debug, Clone, PartialEq)]
pub struct CappedCylinderSolidInput {
    host: PlanarSolidInput,
    port_face: usize,
    frame: Frame,
    radius: f64,
    axial_range: ParamRange,
    side_source: Option<FaceId>,
    cap_source: Option<FaceId>,
}

impl CappedCylinderSolidInput {
    /// Describe one capped cylinder segment attached at either axial endpoint.
    pub const fn new(
        host: PlanarSolidInput,
        port_face: usize,
        frame: Frame,
        radius: f64,
        axial_range: ParamRange,
    ) -> Self {
        Self {
            host,
            port_face,
            frame,
            radius,
            axial_range,
            side_source: None,
            cap_source: None,
        }
    }

    /// Attach the selected source of the cylindrical side region.
    pub const fn with_side_source(mut self, source: FaceId) -> Self {
        self.side_source = Some(source);
        self
    }

    /// Attach the selected source of the closing cap region.
    pub const fn with_cap_source(mut self, source: FaceId) -> Self {
        self.cap_source = Some(source);
        self
    }

    /// Convex planar host proposal.
    pub const fn host(&self) -> &PlanarSolidInput {
        &self.host
    }

    /// Host face receiving the circular attachment.
    pub const fn port_face(&self) -> usize {
        self.port_face
    }

    /// Cylinder parameter frame.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Positive cylinder radius.
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// Finite increasing cylinder interval.
    pub const fn axial_range(&self) -> ParamRange {
        self.axial_range
    }

    /// Optional source of the side face.
    pub const fn side_source(&self) -> Option<FaceId> {
        self.side_source
    }

    /// Optional source of the closing cap.
    pub const fn cap_source(&self) -> Option<FaceId> {
        self.cap_source
    }
}

/// Backward-compatible name for an outward capped-cylinder input.
///
/// The aliased input now also admits a semantically proven inward blind
/// pocket. New neutral adapters should prefer [`CappedCylinderSolidInput`].
pub type CylindricalBossSolidInput = CappedCylinderSolidInput;

/// Stable handles produced by capped-cylinder assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CappedCylinderSolidOutput {
    body: BodyId,
    shell: ShellId,
    host_faces: Vec<FaceId>,
    port_face: FaceId,
    side_face: FaceId,
    cap_face: FaceId,
    ring_edges: [EdgeId; 2],
}

impl CappedCylinderSolidOutput {
    /// Newly assembled solid body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Positive connected boundary shell.
    pub const fn shell(&self) -> ShellId {
        self.shell
    }

    /// Planar host faces in input order.
    pub fn host_faces(&self) -> &[FaceId] {
        &self.host_faces
    }

    /// Punctured planar attachment face.
    pub const fn port_face(&self) -> FaceId {
        self.port_face
    }

    /// Cylindrical side face.
    pub const fn side_face(&self) -> FaceId {
        self.side_face
    }

    /// Closing planar cap face opposite the attachment endpoint.
    pub const fn cap_face(&self) -> FaceId {
        self.cap_face
    }

    /// Attachment and cap ring edges in axial order.
    pub const fn ring_edges(&self) -> [EdgeId; 2] {
        self.ring_edges
    }
}

/// Backward-compatible output name for capped-cylinder assembly.
pub type CylindricalBossSolidOutput = CappedCylinderSolidOutput;

#[derive(Debug, Clone, Copy)]
struct PreparedPlanarRingUse {
    curve: Circle2d,
    map: ParamMap1d,
}

#[derive(Debug, Clone, Copy)]
struct PreparedRing {
    circle: Circle,
    side_sense: Sense,
    side_pcurve: Line2d,
    planar: PreparedPlanarRingUse,
}

#[derive(Debug)]
struct PreparedCappedCylinder {
    host: PreparedSolid,
    port_face: usize,
    cylinder: Cylinder,
    side_domain: FaceDomain,
    side_sense: Sense,
    cap_plane: Plane,
    cap_domain: FaceDomain,
    cap_endpoint: usize,
    rings: [PreparedRing; 2],
    side_source: Option<FaceId>,
    cap_source: Option<FaceId>,
}

impl PreparedCappedCylinder {
    fn new(input: &CappedCylinderSolidInput, store: &crate::store::Store) -> Result<Self> {
        if !input.radius.is_finite() || input.radius <= 0.0 {
            return invalid("capped-cylinder radius must be finite and positive");
        }
        if !input.axial_range.is_finite() || input.axial_range.lo >= input.axial_range.hi {
            return invalid("capped-cylinder axial range must be finite and increasing");
        }
        for source in [input.side_source, input.cap_source].into_iter().flatten() {
            if !store.contains(source) {
                return Err(Error::StaleHandle);
            }
        }

        let host = PreparedSolid::new(&input.host, store)?;
        let Some((port_plane, port_sense)) = host.face_plane(input.port_face, store)? else {
            return invalid("capped-cylinder port face index is invalid");
        };
        certify_convex_host(&input.host, &host, store)?;

        check_in_size_box(input.frame.origin().to_array())?;
        let low_center = input.frame.origin() + input.frame.z() * input.axial_range.lo;
        let frame = input.frame.with_origin(low_center);
        let height = input.axial_range.hi - input.axial_range.lo;
        let cylinder = Cylinder::new(frame, input.radius)?;
        let high_center = frame.origin() + frame.z() * height;
        check_in_size_box(high_center.to_array())?;

        let zero = [0.0; 3];
        if exact_dot(port_plane.frame().x(), frame.z(), zero)? != Orientation::Zero
            || exact_dot(port_plane.frame().y(), frame.z(), zero)? != Orientation::Zero
        {
            return invalid("capped-cylinder axis must be exactly perpendicular to its port plane");
        }
        let outward = port_plane.frame().z() * if port_sense.is_forward() { 1.0 } else { -1.0 };
        let [low_incidence, high_incidence] = [low_center, high_center].map(|center| {
            exact_affine(
                port_plane.frame().z(),
                center,
                port_plane.frame().origin(),
                0.0,
            )
        });
        let endpoint_incidence = [low_incidence?, high_incidence?];
        let port_endpoint = match endpoint_incidence {
            [Orientation::Zero, nonzero] if nonzero != Orientation::Zero => 0,
            [nonzero, Orientation::Zero] if nonzero != Orientation::Zero => 1,
            _ => {
                return invalid(
                    "exactly one capped-cylinder endpoint must be incident with its port plane",
                );
            }
        };
        let cap_endpoint = 1 - port_endpoint;
        let endpoints = [low_center, high_center];
        let feature_direction = exact_affine(
            outward,
            endpoints[cap_endpoint],
            endpoints[port_endpoint],
            0.0,
        )?;
        let side_sense = match feature_direction {
            Orientation::Positive => Sense::Forward,
            Orientation::Negative => Sense::Reversed,
            Orientation::Zero => {
                return invalid("capped-cylinder segment must have nonzero signed height");
            }
        };

        let positions = input
            .host
            .vertices()
            .iter()
            .map(|vertex| (vertex.key(), vertex.position()))
            .collect::<BTreeMap<_, _>>();
        let port = &input.host.faces()[input.port_face];
        let polygon = port
            .vertices()
            .iter()
            .map(|key| {
                let point = positions.get(key).copied().ok_or(Error::InvalidGeometry {
                    reason: "capped-cylinder port references an unknown host vertex",
                })?;
                Ok(frame_uv(port_plane.frame(), point))
            })
            .collect::<Result<Vec<_>>>()?;
        let circles = [
            Circle::new(frame, input.radius)?,
            Circle::new(frame.with_origin(high_center), input.radius)?,
        ];
        preflight_circle_extent(circles[0])?;
        preflight_circle_extent(circles[1])?;
        let port_planar = planar_ring_use(port_plane.frame(), circles[port_endpoint])?;
        if !certify_convex_polygon_circle_containment(&polygon, port_planar.curve) {
            return invalid(
                "capped-cylinder attachment disk must lie strictly inside its convex port face",
            );
        }

        if side_sense == Sense::Reversed {
            certify_inward_segment_containment(
                input,
                &host,
                store,
                frame,
                input.radius,
                endpoints,
                input.port_face,
            )?;
        }

        let cap_frame = Frame::new(endpoints[cap_endpoint], outward, frame.x())?;
        let cap_plane = Plane::new(cap_frame);
        let cap_domain =
            FaceDomain::from_bounds(-input.radius, input.radius, -input.radius, input.radius)?;
        let side_domain = FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, height)?;
        let mut rings = Vec::with_capacity(2);
        for (index, side_v) in [0.0, height].into_iter().enumerate() {
            let conventional = if index == 0 {
                Sense::Forward
            } else {
                Sense::Reversed
            };
            let ring_sense = if side_sense == Sense::Forward {
                conventional
            } else {
                conventional.flipped()
            };
            let planar = if index == port_endpoint {
                port_planar
            } else {
                planar_ring_use(cap_plane.frame(), circles[index])?
            };
            rings.push(PreparedRing {
                circle: circles[index],
                side_sense: ring_sense,
                side_pcurve: Line2d::new(Point2::new(0.0, side_v), Vec2::new(1.0, 0.0))?,
                planar,
            });
        }
        let rings = rings
            .try_into()
            .expect("two axial endpoints prepare exactly two capped-cylinder rings");

        Ok(Self {
            host,
            port_face: input.port_face,
            cylinder,
            side_domain,
            side_sense,
            cap_plane,
            cap_domain,
            cap_endpoint,
            rings,
            side_source: input.side_source,
            cap_source: input.cap_source,
        })
    }
}

impl Transaction<'_> {
    /// Assemble one capped cylindrical boss or pocket on a convex planar host.
    ///
    /// Complete semantic preflight occurs before topology allocation. The
    /// caller owns the eventual checked or Full commit.
    pub fn assemble_capped_cylinder_solid(
        &mut self,
        input: &CappedCylinderSolidInput,
    ) -> Result<CappedCylinderSolidOutput> {
        let prepared = PreparedCappedCylinder::new(input, self.store())?;
        let (body, shell) = crate::make::solid_body_scaffold(self.store_mut());
        let host = self.allocate_prepared_planar_shell(prepared.host, shell)?;
        let port_face = host.faces[prepared.port_face];

        let side_surface = self
            .store_mut()
            .insert_surface(SurfaceGeom::Cylinder(prepared.cylinder))?;
        let side_face = self.store_mut().add(Face {
            shell,
            loops: Vec::new(),
            surface: side_surface,
            sense: prepared.side_sense,
            domain: Some(prepared.side_domain),
            tolerance: None,
        });
        self.store_mut().get_mut(shell)?.faces.push(side_face);

        let port_endpoint = 1 - prepared.cap_endpoint;
        let mut ring_edges = [None, None];
        ring_edges[port_endpoint] = Some(self.allocate_boss_port_ring(
            side_face,
            port_face,
            prepared.rings[port_endpoint],
        )?);
        let (cap_face, cap_edge) = self.allocate_boss_cap_ring(
            shell,
            side_face,
            prepared.rings[prepared.cap_endpoint],
            prepared.cap_plane,
            prepared.cap_domain,
        )?;
        ring_edges[prepared.cap_endpoint] = Some(cap_edge);

        if let Some(source) = prepared.side_source {
            self.record_derived_from(EntityRef::Face(side_face), EntityRef::Face(source));
        }
        if let Some(source) = prepared.cap_source {
            self.record_derived_from(EntityRef::Face(cap_face), EntityRef::Face(source));
        }

        Ok(CappedCylinderSolidOutput {
            body,
            shell,
            host_faces: host.faces,
            port_face,
            side_face,
            cap_face,
            ring_edges: ring_edges.map(|edge| {
                edge.expect("port and cap allocation cover both capped-cylinder endpoints")
            }),
        })
    }

    /// Backward-compatible spelling for capped-cylinder assembly.
    pub fn assemble_cylindrical_boss_solid(
        &mut self,
        input: &CylindricalBossSolidInput,
    ) -> Result<CylindricalBossSolidOutput> {
        self.assemble_capped_cylinder_solid(input)
    }

    fn allocate_boss_port_ring(
        &mut self,
        side_face: FaceId,
        port_face: FaceId,
        ring: PreparedRing,
    ) -> Result<EdgeId> {
        let (edge, side_fin) = self.allocate_boss_side_ring(side_face, ring)?;
        let store = self.store_mut();
        let loop_id = store.add(Loop {
            face: port_face,
            fins: Vec::new(),
        });
        store.get_mut(port_face)?.loops.push(loop_id);
        let curve = store.insert_pcurve(Curve2dGeom::Circle(ring.planar.curve))?;
        let use_ = FinPcurve::new(
            curve,
            ParamRange::new(0.0, core::f64::consts::TAU),
            ring.planar.map,
        )?
        .with_closure_winding([0, 0]);
        let planar_fin = store.add(Fin {
            parent: loop_id,
            edge,
            sense: ring.side_sense.flipped(),
            pcurve: Some(use_),
        });
        store.get_mut(loop_id)?.fins.push(planar_fin);
        store.get_mut(edge)?.fins = vec![side_fin, planar_fin];
        Ok(edge)
    }

    fn allocate_boss_cap_ring(
        &mut self,
        shell: ShellId,
        side_face: FaceId,
        ring: PreparedRing,
        plane: Plane,
        domain: FaceDomain,
    ) -> Result<(FaceId, EdgeId)> {
        let (edge, side_fin) = self.allocate_boss_side_ring(side_face, ring)?;
        let store = self.store_mut();
        let surface = store.insert_surface(SurfaceGeom::Plane(plane))?;
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
            domain: Some(domain),
            tolerance: None,
        });
        store.get_mut(shell)?.faces.push(face);
        let loop_id = store.add(Loop {
            face,
            fins: Vec::new(),
        });
        store.get_mut(face)?.loops.push(loop_id);
        let curve = store.insert_pcurve(Curve2dGeom::Circle(ring.planar.curve))?;
        let use_ = FinPcurve::new(
            curve,
            ParamRange::new(0.0, core::f64::consts::TAU),
            ring.planar.map,
        )?
        .with_closure_winding([0, 0]);
        let cap_fin = store.add(Fin {
            parent: loop_id,
            edge,
            sense: ring.side_sense.flipped(),
            pcurve: Some(use_),
        });
        store.get_mut(loop_id)?.fins.push(cap_fin);
        store.get_mut(edge)?.fins = vec![side_fin, cap_fin];
        Ok((face, edge))
    }

    fn allocate_boss_side_ring(
        &mut self,
        side_face: FaceId,
        ring: PreparedRing,
    ) -> Result<(EdgeId, crate::entity::FinId)> {
        let store = self.store_mut();
        let curve = store.insert_curve(CurveGeom::Circle(ring.circle))?;
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [None, None],
            bounds: None,
            fins: Vec::new(),
            tolerance: None,
        });
        let loop_id: LoopId = store.add(Loop {
            face: side_face,
            fins: Vec::new(),
        });
        store.get_mut(side_face)?.loops.push(loop_id);
        let curve = store.insert_pcurve(Curve2dGeom::Line(ring.side_pcurve))?;
        let use_ = FinPcurve::new(
            curve,
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamMap1d::identity(),
        )?
        .with_closure_winding([1, 0]);
        let fin = store.add(Fin {
            parent: loop_id,
            edge,
            sense: ring.side_sense,
            pcurve: Some(use_),
        });
        store.get_mut(loop_id)?.fins.push(fin);
        Ok((edge, fin))
    }
}

fn certify_convex_host(
    input: &PlanarSolidInput,
    prepared: &PreparedSolid,
    store: &crate::store::Store,
) -> Result<()> {
    for index in 0..input.faces().len() {
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "capped-cylinder host face disappeared during preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        let mut strictly_inside = false;
        for vertex in input.vertices() {
            match exact_affine(outward, vertex.position(), plane.frame().origin(), 0.0)? {
                Orientation::Negative => strictly_inside = true,
                Orientation::Zero => {}
                Orientation::Positive => {
                    return invalid("capped-cylinder host must be globally convex");
                }
            }
        }
        if !strictly_inside {
            return invalid("capped-cylinder host face must support a bounded convex solid");
        }
    }
    Ok(())
}

/// Prove that every point of an inward finite cylinder segment lies in the
/// virtual convex host. For a linear axial sweep, the maximum of each host
/// half-space form occurs on one of the two endpoint circles. The port support
/// is handled exactly (the port ring lies on it and every other section is
/// strictly inward); every other support receives an outward-rounded interval
/// proof that both complete endpoint circles are strictly inward.
fn certify_inward_segment_containment(
    input: &CappedCylinderSolidInput,
    prepared: &PreparedSolid,
    store: &crate::store::Store,
    cylinder_frame: Frame,
    radius: f64,
    endpoints: [Point3; 2],
    port_face: usize,
) -> Result<()> {
    for index in 0..input.host.faces().len() {
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "capped-cylinder host face disappeared during containment preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        if index == port_face {
            let [first, second] =
                endpoints.map(|center| exact_affine(outward, center, plane.frame().origin(), 0.0));
            let endpoint_signs = [first?, second?];
            if endpoint_signs
                .into_iter()
                .filter(|sign| *sign == Orientation::Zero)
                .count()
                != 1
                || !endpoint_signs.contains(&Orientation::Negative)
                || exact_dot(outward, cylinder_frame.x(), [0.0; 3])? != Orientation::Zero
                || exact_dot(outward, cylinder_frame.y(), [0.0; 3])? != Orientation::Zero
            {
                return invalid(
                    "inward capped-cylinder segment must leave its port strictly into the host",
                );
            }
            continue;
        }

        for center in endpoints {
            if exact_affine(outward, center, plane.frame().origin(), 0.0)? != Orientation::Negative
                || !certify_circle_strictly_inside_halfspace(
                    outward,
                    plane.frame().origin(),
                    cylinder_frame,
                    center,
                    radius,
                )
            {
                return invalid(
                    "complete inward capped-cylinder segment must lie strictly inside the convex host",
                );
            }
        }
    }
    Ok(())
}

fn certify_circle_strictly_inside_halfspace(
    outward: Vec3,
    support_origin: Point3,
    cylinder_frame: Frame,
    center: Point3,
    radius: f64,
) -> bool {
    let offset = center - support_origin;
    let signed = interval_dot(outward, offset);
    if signed.hi() >= 0.0 {
        return false;
    }
    let radius = Interval::point(radius);
    let radial_x = interval_dot(outward, cylinder_frame.x()) * radius;
    let radial_y = interval_dot(outward, cylinder_frame.y()) * radius;
    let radial_squared = radial_x.square() + radial_y.square();
    radial_squared.hi() < signed.square().lo()
}

fn interval_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
}

fn planar_ring_use(frame: &Frame, circle: Circle) -> Result<PreparedPlanarRingUse> {
    let center = frame_uv(frame, circle.frame().origin());
    let x = frame_uv_vector(frame, circle.frame().x());
    let y = frame_uv_vector(frame, circle.frame().y());
    let curve = Circle2d::new(center, circle.radius(), x)?;
    let map = if y.dot(curve.x_dir().perp()) >= 0.0 {
        ParamMap1d::identity()
    } else {
        ParamMap1d::affine(-1.0, core::f64::consts::TAU)?
    };
    Ok(PreparedPlanarRingUse { curve, map })
}

fn frame_uv(frame: &Frame, point: Point3) -> Point2 {
    let offset = point - frame.origin();
    Point2::new(offset.dot(frame.x()), offset.dot(frame.y()))
}

fn frame_uv_vector(frame: &Frame, vector: Vec3) -> Vec2 {
    Vec2::new(vector.dot(frame.x()), vector.dot(frame.y()))
}

fn exact_dot(normal: Vec3, vector: Vec3, origin: [f64; 3]) -> Result<Orientation> {
    affine_dot3(normal.to_array(), vector.to_array(), origin, 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "capped-cylinder exact vector predicate is indeterminate",
        })
}

fn exact_affine(normal: Vec3, point: Point3, origin: Point3, bias: f64) -> Result<Orientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), bias)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "capped-cylinder exact affine predicate is indeterminate",
        })
}

fn preflight_circle_extent(circle: Circle) -> Result<()> {
    let bounds = circle.bounding_box(circle.param_range());
    check_in_size_box(bounds.min.to_array())?;
    check_in_size_box(bounds.max.to_array())?;
    Ok(())
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::{
        Body, Edge as RawEdge, Face as RawFace, Fin as RawFin, Loop as RawLoop, Region, Shell,
        Vertex as RawVertex,
    };
    use crate::geom::SurfaceGeom;
    use crate::planar::{PlanarSolidFace, PlanarSolidVertex, PlanarVertexKey};
    use crate::store::Store;
    use crate::transaction::FullCommitRequirement;

    fn cube() -> PlanarSolidInput {
        let points = [
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let keys = core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(index as u64));
        let vertices = keys
            .into_iter()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
            .collect();
        let faces = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
        .into_iter()
        .map(|ring| PlanarSolidFace::new(ring.map(|index| keys[index]).to_vec()))
        .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn input(radius: f64) -> CylindricalBossSolidInput {
        CylindricalBossSolidInput::new(
            cube(),
            1,
            Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
            radius,
            ParamRange::new(0.0, 1.5),
        )
    }

    fn pocket(radius: f64) -> CappedCylinderSolidInput {
        CappedCylinderSolidInput::new(cube(), 1, Frame::world(), radius, ParamRange::new(0.0, 1.0))
    }

    fn topology_counts(store: &Store) -> [usize; 8] {
        [
            store.count::<Body>(),
            store.count::<Region>(),
            store.count::<Shell>(),
            store.count::<RawFace>(),
            store.count::<RawLoop>(),
            store.count::<RawFin>(),
            store.count::<RawEdge>(),
            store.count::<RawVertex>(),
        ]
    }

    #[test]
    fn convex_host_boss_is_full_valid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_boss_solid(&input(0.5))
            .unwrap();
        assert_eq!(output.host_faces().len(), 6);
        assert_eq!(
            transaction
                .store()
                .faces_of_body(output.body())
                .unwrap()
                .len(),
            8
        );
        assert_eq!(
            transaction
                .store()
                .edges_of_body(output.body())
                .unwrap()
                .len(),
            14
        );
        assert_eq!(
            transaction
                .store()
                .vertices_of_body(output.body())
                .unwrap()
                .len(),
            8
        );
        assert_eq!(
            transaction
                .store()
                .get(output.port_face())
                .unwrap()
                .loops()
                .len(),
            2
        );
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
        assert!(decision.checks().iter().all(|check| {
            check.report().outcome() == CheckOutcome::Valid && check.report().gaps.is_empty()
        }));
        assert_eq!(
            check_body_report(&store, output.body(), CheckLevel::Full)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        );
    }

    #[test]
    fn convex_host_blind_pocket_is_full_valid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_capped_cylinder_solid(&pocket(0.75))
            .unwrap();
        let faces = transaction.store().faces_of_body(output.body()).unwrap();
        assert_eq!(faces.len(), 8);
        assert_eq!(
            transaction
                .store()
                .edges_of_body(output.body())
                .unwrap()
                .len(),
            14
        );
        assert_eq!(
            transaction
                .store()
                .vertices_of_body(output.body())
                .unwrap()
                .len(),
            8
        );
        assert_eq!(
            transaction
                .store()
                .get(output.body())
                .unwrap()
                .regions()
                .len(),
            2
        );
        assert_eq!(
            transaction
                .store()
                .get(output.shell())
                .unwrap()
                .faces()
                .len(),
            8
        );
        assert_eq!(
            transaction
                .store()
                .get(transaction.store().get(output.shell()).unwrap().region())
                .unwrap()
                .shells(),
            [output.shell()]
        );
        assert_eq!(
            transaction
                .store()
                .get(output.body())
                .unwrap()
                .regions()
                .iter()
                .map(|region| transaction.store().get(*region).unwrap().shells().len())
                .sum::<usize>(),
            1
        );
        let mut loop_counts = faces
            .iter()
            .map(|face| transaction.store().get(*face).unwrap().loops().len())
            .collect::<Vec<_>>();
        loop_counts.sort_unstable();
        assert_eq!(loop_counts, [1, 1, 1, 1, 1, 1, 2, 2]);
        let (planes, cylinders) = faces.iter().fold((0, 0), |(planes, cylinders), face| {
            match transaction
                .store()
                .get(transaction.store().get(*face).unwrap().surface())
                .unwrap()
            {
                SurfaceGeom::Plane(_) => (planes + 1, cylinders),
                SurfaceGeom::Cylinder(_) => (planes, cylinders + 1),
                _ => (planes, cylinders),
            }
        });
        assert_eq!((planes, cylinders), (7, 1));
        assert_eq!(
            transaction.store().get(output.side_face()).unwrap().sense(),
            Sense::Reversed
        );
        assert_eq!(
            transaction
                .store()
                .get(output.port_face())
                .unwrap()
                .loops()
                .len(),
            2
        );
        let [low_ring, high_ring] = output.ring_edges();
        assert_ne!(low_ring, high_ring);

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
        assert!(decision.checks().iter().all(|check| {
            check.report().outcome() == CheckOutcome::Valid && check.report().gaps.is_empty()
        }));
        assert_eq!(
            check_body_report(&store, output.body(), CheckLevel::Full)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        );
    }

    #[test]
    fn blind_pocket_wrong_side_sense_is_full_invalid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_capped_cylinder_solid(&pocket(0.75))
            .unwrap();
        transaction
            .store_mut()
            .get_mut(output.side_face())
            .unwrap()
            .sense = Sense::Forward;
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(!decision.is_committed());
        assert!(
            decision
                .checks()
                .iter()
                .any(|check| check.report().outcome() == CheckOutcome::Invalid)
        );
        assert_eq!(store.count::<Body>(), 0);
    }

    #[test]
    fn malformed_inputs_fail_before_topology_allocation() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = topology_counts(transaction.store());
        let no_incident_endpoint = CappedCylinderSolidInput::new(
            cube(),
            1,
            Frame::world(),
            0.5,
            ParamRange::new(0.0, 0.5),
        );
        let escapes_convex_host = CappedCylinderSolidInput::new(
            cube(),
            1,
            Frame::world(),
            0.5,
            ParamRange::new(-2.0, 1.0),
        );
        for proposal in [
            input(-0.5),
            input(1.0),
            input(1.25),
            no_incident_endpoint,
            escapes_convex_host,
        ] {
            assert!(matches!(
                transaction.assemble_capped_cylinder_solid(&proposal),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(topology_counts(transaction.store()), before);
        }
    }
}
