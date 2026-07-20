//! Semantic assembly of one capped cylindrical boss on a convex planar host.
//!
//! The host is supplied through [`PlanarSolidInput`]. One designated convex
//! face is punctured by a strictly contained circular loop, and a positive
//! finite cylinder band is attached on the face's outward side. The other end
//! of the band is closed by one planar cap. All caller-controlled geometry,
//! convexity, incidence, size-box, and optional lineage data are preflighted
//! before a body scaffold is allocated.

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
use kcore::predicates::{Orientation, affine_dot3};
use kcore::tolerance::check_in_size_box;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};

/// Complete semantic input for one positive cylindrical boss.
///
/// `port_face` indexes `host.faces()`. `axial_range.lo` is the attachment
/// plane and `frame.z()` points from the host into the boss. Optional source
/// faces are semantic lineage for the cylindrical side and high cap; planar
/// host lineage remains owned by [`PlanarSolidInput`].
#[derive(Debug, Clone, PartialEq)]
pub struct CylindricalBossSolidInput {
    host: PlanarSolidInput,
    port_face: usize,
    frame: Frame,
    radius: f64,
    axial_range: ParamRange,
    side_source: Option<FaceId>,
    cap_source: Option<FaceId>,
}

impl CylindricalBossSolidInput {
    /// Describe a capped boss attached at the low end of `axial_range`.
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

    /// Positive boss radius.
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// Finite increasing boss interval.
    pub const fn axial_range(&self) -> ParamRange {
        self.axial_range
    }

    /// Optional source of the side face.
    pub const fn side_source(&self) -> Option<FaceId> {
        self.side_source
    }

    /// Optional source of the high cap.
    pub const fn cap_source(&self) -> Option<FaceId> {
        self.cap_source
    }
}

/// Stable handles produced by cylindrical-boss assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CylindricalBossSolidOutput {
    body: BodyId,
    shell: ShellId,
    host_faces: Vec<FaceId>,
    port_face: FaceId,
    side_face: FaceId,
    cap_face: FaceId,
    ring_edges: [EdgeId; 2],
}

impl CylindricalBossSolidOutput {
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

    /// High planar cap face.
    pub const fn cap_face(&self) -> FaceId {
        self.cap_face
    }

    /// Attachment and cap ring edges in axial order.
    pub const fn ring_edges(&self) -> [EdgeId; 2] {
        self.ring_edges
    }
}

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
struct PreparedBoss {
    host: PreparedSolid,
    port_face: usize,
    cylinder: Cylinder,
    side_domain: FaceDomain,
    cap_plane: Plane,
    cap_domain: FaceDomain,
    rings: [PreparedRing; 2],
    side_source: Option<FaceId>,
    cap_source: Option<FaceId>,
}

impl PreparedBoss {
    fn new(input: &CylindricalBossSolidInput, store: &crate::store::Store) -> Result<Self> {
        if !input.radius.is_finite() || input.radius <= 0.0 {
            return invalid("cylindrical-boss radius must be finite and positive");
        }
        if !input.axial_range.is_finite() || input.axial_range.lo >= input.axial_range.hi {
            return invalid("cylindrical-boss axial range must be finite and increasing");
        }
        for source in [input.side_source, input.cap_source].into_iter().flatten() {
            if !store.contains(source) {
                return Err(Error::StaleHandle);
            }
        }

        let host = PreparedSolid::new(&input.host, store)?;
        let Some((port_plane, port_sense)) = host.face_plane(input.port_face, store)? else {
            return invalid("cylindrical-boss port face index is invalid");
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
            || exact_affine(
                port_plane.frame().z(),
                low_center,
                port_plane.frame().origin(),
                0.0,
            )? != Orientation::Zero
        {
            return invalid(
                "cylindrical-boss axis and port plane must be exactly perpendicular and incident",
            );
        }
        let outward = port_plane.frame().z() * if port_sense.is_forward() { 1.0 } else { -1.0 };
        if exact_affine(outward, high_center, low_center, 0.0)? != Orientation::Positive {
            return invalid("cylindrical-boss must extend from the outward side of its port face");
        }

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
                    reason: "cylindrical-boss port references an unknown host vertex",
                })?;
                Ok(frame_uv(port_plane.frame(), point))
            })
            .collect::<Result<Vec<_>>>()?;
        let low_circle = Circle::new(frame, input.radius)?;
        preflight_circle_extent(low_circle)?;
        let low_planar = planar_ring_use(port_plane.frame(), low_circle)?;
        if !certify_convex_polygon_circle_containment(&polygon, low_planar.curve) {
            return invalid(
                "cylindrical-boss attachment disk must lie strictly inside its convex port face",
            );
        }

        let high_frame = frame.with_origin(high_center);
        let high_circle = Circle::new(high_frame, input.radius)?;
        preflight_circle_extent(high_circle)?;
        let cap_plane = Plane::new(high_frame);
        let cap_domain =
            FaceDomain::from_bounds(-input.radius, input.radius, -input.radius, input.radius)?;
        let side_domain = FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, height)?;
        let rings = [
            PreparedRing {
                circle: low_circle,
                side_sense: Sense::Forward,
                side_pcurve: Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0))?,
                planar: low_planar,
            },
            PreparedRing {
                circle: high_circle,
                side_sense: Sense::Reversed,
                side_pcurve: Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0))?,
                planar: planar_ring_use(cap_plane.frame(), high_circle)?,
            },
        ];

        Ok(Self {
            host,
            port_face: input.port_face,
            cylinder,
            side_domain,
            cap_plane,
            cap_domain,
            rings,
            side_source: input.side_source,
            cap_source: input.cap_source,
        })
    }
}

impl Transaction<'_> {
    /// Assemble one positive capped cylindrical boss on a convex planar host.
    ///
    /// Complete semantic preflight occurs before topology allocation. The
    /// caller owns the eventual checked or Full commit.
    pub fn assemble_cylindrical_boss_solid(
        &mut self,
        input: &CylindricalBossSolidInput,
    ) -> Result<CylindricalBossSolidOutput> {
        let prepared = PreparedBoss::new(input, self.store())?;
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
            sense: Sense::Forward,
            domain: Some(prepared.side_domain),
            tolerance: None,
        });
        self.store_mut().get_mut(shell)?.faces.push(side_face);

        let low_edge = self.allocate_boss_port_ring(side_face, port_face, prepared.rings[0])?;
        let (cap_face, high_edge) = self.allocate_boss_cap_ring(
            shell,
            side_face,
            prepared.rings[1],
            prepared.cap_plane,
            prepared.cap_domain,
        )?;

        if let Some(source) = prepared.side_source {
            self.record_derived_from(EntityRef::Face(side_face), EntityRef::Face(source));
        }
        if let Some(source) = prepared.cap_source {
            self.record_derived_from(EntityRef::Face(cap_face), EntityRef::Face(source));
        }

        Ok(CylindricalBossSolidOutput {
            body,
            shell,
            host_faces: host.faces,
            port_face,
            side_face,
            cap_face,
            ring_edges: [low_edge, high_edge],
        })
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
                reason: "cylindrical-boss host face disappeared during preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        let mut strictly_inside = false;
        for vertex in input.vertices() {
            match exact_affine(outward, vertex.position(), plane.frame().origin(), 0.0)? {
                Orientation::Negative => strictly_inside = true,
                Orientation::Zero => {}
                Orientation::Positive => {
                    return invalid("cylindrical-boss host must be globally convex");
                }
            }
        }
        if !strictly_inside {
            return invalid("cylindrical-boss host face must support a bounded convex solid");
        }
    }
    Ok(())
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
            reason: "cylindrical-boss exact vector predicate is indeterminate",
        })
}

fn exact_affine(normal: Vec3, point: Point3, origin: Point3, bias: f64) -> Result<Orientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), bias)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "cylindrical-boss exact affine predicate is indeterminate",
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
    use crate::entity::{Body, Face as RawFace, Region, Shell};
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
    fn malformed_inputs_fail_before_topology_allocation() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = (
            transaction.store().count::<Body>(),
            transaction.store().count::<Region>(),
            transaction.store().count::<Shell>(),
            transaction.store().count::<RawFace>(),
        );
        let inward = CylindricalBossSolidInput::new(
            cube(),
            0,
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
        );
        for proposal in [input(-0.5), input(1.0), input(1.25), inward] {
            assert!(matches!(
                transaction.assemble_cylindrical_boss_solid(&proposal),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(
                (
                    transaction.store().count::<Body>(),
                    transaction.store().count::<Region>(),
                    transaction.store().count::<Shell>(),
                    transaction.store().count::<RawFace>(),
                ),
                before
            );
        }
    }
}
