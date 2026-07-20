//! Semantic assembly of one finite cylinder band joining two convex-host ports.
//!
//! This is an operation-neutral endpoint-topology class, not a Boolean or
//! fixture layout. The caller supplies a convex planar host, two distinct host
//! face identities, and one finite analytic cylinder interval. Exact support
//! incidence maps the unordered port identities onto the low and high axial
//! endpoints. Both complete rings must be strictly contained by their port
//! polygons, and outward-rounded support proofs cover the complete swept
//! circle before any topology is allocated.
//!
//! The result retains the virtual host boundary with both port faces punctured
//! and joins those holes by one reversed full-period cylindrical face. One
//! face suffices: its two constant-height periodic loops bound the complete
//! finite band without an artificial seam or axial split.

use std::collections::BTreeMap;

use crate::entity::{
    BodyId, EdgeId, EntityRef, Face, FaceDomain, FaceId, ParamMap1d, Sense, ShellId,
};
use crate::geom::SurfaceGeom;
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
use kgeom::surface::Cylinder;
use kgeom::vec::{Point2, Point3, Vec2, Vec3};

/// Complete semantic input for one finite cylinder joining two host ports.
///
/// `port_faces` is an unordered pair of distinct indices into `host.faces()`.
/// Exact preflight maps each face onto exactly one axial endpoint. The side
/// source is optional semantic lineage for the resulting reversed cylinder;
/// planar-host lineage remains owned by [`PlanarSolidInput`].
#[derive(Debug, Clone, PartialEq)]
pub struct TwoPortCylinderSolidInput {
    host: PlanarSolidInput,
    port_faces: [usize; 2],
    frame: Frame,
    radius: f64,
    axial_range: ParamRange,
    side_source: Option<FaceId>,
}

impl TwoPortCylinderSolidInput {
    /// Describe a finite axial cylinder joining two convex-host faces.
    pub const fn new(
        host: PlanarSolidInput,
        port_faces: [usize; 2],
        frame: Frame,
        radius: f64,
        axial_range: ParamRange,
    ) -> Self {
        Self {
            host,
            port_faces,
            frame,
            radius,
            axial_range,
            side_source: None,
        }
    }

    /// Attach the live source face selected for the cylindrical side.
    pub const fn with_side_source(mut self, source: FaceId) -> Self {
        self.side_source = Some(source);
        self
    }

    /// Convex planar host proposal.
    pub const fn host(&self) -> &PlanarSolidInput {
        &self.host
    }

    /// Unordered input pair of host-port face indices.
    pub const fn port_faces(&self) -> [usize; 2] {
        self.port_faces
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

    /// Optional source of the cylindrical side face.
    pub const fn side_source(&self) -> Option<FaceId> {
        self.side_source
    }
}

/// Stable handles produced by two-port cylinder assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TwoPortCylinderSolidOutput {
    body: BodyId,
    shell: ShellId,
    host_faces: Vec<FaceId>,
    port_faces: [FaceId; 2],
    side_face: FaceId,
    ring_edges: [EdgeId; 2],
}

impl TwoPortCylinderSolidOutput {
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

    /// Punctured port faces in `[low, high]` axial order.
    pub const fn port_faces(&self) -> [FaceId; 2] {
        self.port_faces
    }

    /// Reversed cylindrical side face joining both ports.
    pub const fn side_face(&self) -> FaceId {
        self.side_face
    }

    /// Vertexless circle edges in `[low, high]` axial order.
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
struct PreparedPortRing {
    circle: Circle,
    side_sense: Sense,
    side_pcurve: Line2d,
    planar: PreparedPlanarRingUse,
}

#[derive(Debug)]
struct PreparedTwoPortCylinder {
    host: PreparedSolid,
    port_faces: [usize; 2],
    cylinder: Cylinder,
    side_domain: FaceDomain,
    rings: [PreparedPortRing; 2],
    side_source: Option<FaceId>,
}

impl PreparedTwoPortCylinder {
    fn new(input: &TwoPortCylinderSolidInput, store: &crate::store::Store) -> Result<Self> {
        if !input.radius.is_finite() || input.radius <= 0.0 {
            return invalid("two-port cylinder radius must be finite and positive");
        }
        if !input.axial_range.is_finite() || input.axial_range.lo >= input.axial_range.hi {
            return invalid("two-port cylinder axial range must be finite and increasing");
        }
        if input.port_faces[0] == input.port_faces[1] {
            return invalid("two-port cylinder requires two distinct host faces");
        }
        if input
            .side_source
            .is_some_and(|source| !store.contains(source))
        {
            return Err(Error::StaleHandle);
        }

        let host = PreparedSolid::new(&input.host, store)?;
        certify_convex_host(&input.host, &host, store)?;
        let port_planes = input.port_faces.map(|index| host.face_plane(index, store));
        let [first_plane, second_plane] = port_planes;
        let Some(first_plane) = first_plane? else {
            return invalid("two-port cylinder port face index is invalid");
        };
        let Some(second_plane) = second_plane? else {
            return invalid("two-port cylinder port face index is invalid");
        };
        let port_planes = [first_plane, second_plane];

        check_in_size_box(input.frame.origin().to_array())?;
        let low_center = input.frame.origin() + input.frame.z() * input.axial_range.lo;
        let frame = input.frame.with_origin(low_center);
        let height = input.axial_range.hi - input.axial_range.lo;
        let high_center = frame.origin() + frame.z() * height;
        check_in_size_box(low_center.to_array())?;
        check_in_size_box(high_center.to_array())?;
        let cylinder = Cylinder::new(frame, input.radius)?;
        let endpoints = [low_center, high_center];
        let circles = [
            Circle::new(frame, input.radius)?,
            Circle::new(frame.with_origin(high_center), input.radius)?,
        ];
        preflight_circle_extent(circles[0])?;
        preflight_circle_extent(circles[1])?;

        let positions = input
            .host
            .vertices()
            .iter()
            .map(|vertex| (vertex.key(), vertex.position()))
            .collect::<BTreeMap<_, _>>();
        let mut endpoint_ports = [None, None];
        let mut planar_rings = [None, None];
        for (input_port, (plane, sense)) in port_planes.into_iter().enumerate() {
            if exact_dot(plane.frame().x(), frame.z())? != Orientation::Zero
                || exact_dot(plane.frame().y(), frame.z())? != Orientation::Zero
            {
                return invalid("two-port cylinder axis must be perpendicular to both port planes");
            }
            let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
            let signs =
                endpoints.map(|center| exact_affine(outward, center, plane.frame().origin()));
            let [low_sign, high_sign] = signs;
            let signs = [low_sign?, high_sign?];
            let endpoint = match signs {
                [Orientation::Zero, Orientation::Negative] => 0,
                [Orientation::Negative, Orientation::Zero] => 1,
                _ => {
                    return invalid(
                        "each two-port cylinder face must support one endpoint with the other strictly inward",
                    );
                }
            };
            let expected_outward = if endpoint == 0 {
                Orientation::Negative
            } else {
                Orientation::Positive
            };
            if exact_dot(outward, frame.z())? != expected_outward
                || endpoint_ports[endpoint]
                    .replace(input.port_faces[input_port])
                    .is_some()
            {
                return invalid(
                    "two-port cylinder supports must outwardly oppose at distinct endpoints",
                );
            }

            let port = &input.host.faces()[input.port_faces[input_port]];
            let polygon = port
                .vertices()
                .iter()
                .map(|key| {
                    let point = positions.get(key).copied().ok_or(Error::InvalidGeometry {
                        reason: "two-port cylinder port references an unknown host vertex",
                    })?;
                    Ok(frame_uv(plane.frame(), point))
                })
                .collect::<Result<Vec<_>>>()?;
            let planar = planar_ring_use(plane.frame(), circles[endpoint])?;
            if !certify_convex_polygon_circle_containment(&polygon, planar.curve) {
                return invalid(
                    "two-port cylinder ring must lie strictly inside its convex port face",
                );
            }
            planar_rings[endpoint] = Some(planar);
        }
        let port_faces = endpoint_ports.map(|port| {
            port.expect("two distinct exact endpoint supports fill both axial port slots")
        });
        let planar_rings = planar_rings.map(|ring| {
            ring.expect("two distinct exact endpoint supports prepare both planar ring uses")
        });
        certify_complete_sweep_containment(input, &host, store, frame, endpoints, port_faces)?;

        let side_domain = FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, height)?;
        let rings = [
            PreparedPortRing {
                circle: circles[0],
                side_sense: Sense::Reversed,
                side_pcurve: Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0))?,
                planar: planar_rings[0],
            },
            PreparedPortRing {
                circle: circles[1],
                side_sense: Sense::Forward,
                side_pcurve: Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0))?,
                planar: planar_rings[1],
            },
        ];
        Ok(Self {
            host,
            port_faces,
            cylinder,
            side_domain,
            rings,
            side_source: input.side_source,
        })
    }
}

impl Transaction<'_> {
    /// Assemble one reversed finite cylinder band joining two convex-host ports.
    ///
    /// Complete semantic preflight occurs before topology allocation. The
    /// caller owns the eventual checked or Full commit.
    pub fn assemble_two_port_cylinder_solid(
        &mut self,
        input: &TwoPortCylinderSolidInput,
    ) -> Result<TwoPortCylinderSolidOutput> {
        let prepared = PreparedTwoPortCylinder::new(input, self.store())?;
        let (body, shell) = crate::make::solid_body_scaffold(self.store_mut());
        let host = self.allocate_prepared_planar_shell(prepared.host, shell)?;
        let port_faces = prepared.port_faces.map(|index| host.faces[index]);

        let side_surface = self
            .store_mut()
            .insert_surface(SurfaceGeom::Cylinder(prepared.cylinder))?;
        let side_face = self.store_mut().add(Face {
            shell,
            loops: Vec::new(),
            surface: side_surface,
            sense: Sense::Reversed,
            domain: Some(prepared.side_domain),
            tolerance: None,
        });
        self.store_mut().get_mut(shell)?.faces.push(side_face);

        let mut ring_edges = Vec::with_capacity(2);
        for (endpoint, ring) in prepared.rings.into_iter().enumerate() {
            ring_edges.push(self.allocate_two_port_ring(side_face, port_faces[endpoint], ring)?);
        }
        if let Some(source) = prepared.side_source {
            self.record_derived_from(EntityRef::Face(side_face), EntityRef::Face(source));
        }

        Ok(TwoPortCylinderSolidOutput {
            body,
            shell,
            host_faces: host.faces,
            port_faces,
            side_face,
            ring_edges: ring_edges
                .try_into()
                .expect("two prepared port rings allocate exactly two edges"),
        })
    }

    fn allocate_two_port_ring(
        &mut self,
        side_face: FaceId,
        port_face: FaceId,
        ring: PreparedPortRing,
    ) -> Result<EdgeId> {
        self.allocate_cylindrical_host_port_ring(
            side_face,
            port_face,
            crate::cylindrical_boss::PreparedRing {
                circle: ring.circle,
                side_sense: ring.side_sense,
                side_pcurve: ring.side_pcurve,
                planar: crate::cylindrical_boss::PreparedPlanarRingUse {
                    curve: ring.planar.curve,
                    map: ring.planar.map,
                },
            },
        )
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
                reason: "two-port cylinder host face disappeared during preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        let mut strictly_inside = false;
        for vertex in input.vertices() {
            match exact_affine(outward, vertex.position(), plane.frame().origin())? {
                Orientation::Negative => strictly_inside = true,
                Orientation::Zero => {}
                Orientation::Positive => {
                    return invalid("two-port cylinder host must be globally convex");
                }
            }
        }
        if !strictly_inside {
            return invalid("two-port cylinder host face must support a bounded convex solid");
        }
    }
    Ok(())
}

fn certify_complete_sweep_containment(
    input: &TwoPortCylinderSolidInput,
    prepared: &PreparedSolid,
    store: &crate::store::Store,
    frame: Frame,
    endpoints: [Point3; 2],
    port_faces: [usize; 2],
) -> Result<()> {
    for index in 0..input.host.faces().len() {
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "two-port cylinder host face disappeared during sweep preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        if let Some(endpoint) = port_faces.iter().position(|port| *port == index) {
            let other = 1 - endpoint;
            if exact_affine(outward, endpoints[endpoint], plane.frame().origin())?
                != Orientation::Zero
                || exact_affine(outward, endpoints[other], plane.frame().origin())?
                    != Orientation::Negative
                || exact_dot(outward, frame.x())? != Orientation::Zero
                || exact_dot(outward, frame.y())? != Orientation::Zero
            {
                return invalid(
                    "two-port cylinder sweep must leave each port strictly into the host",
                );
            }
            continue;
        }
        for center in endpoints {
            if exact_affine(outward, center, plane.frame().origin())? != Orientation::Negative
                || !certify_circle_inside_support(
                    outward,
                    plane.frame().origin(),
                    frame,
                    center,
                    input.radius,
                )
            {
                return invalid(
                    "complete two-port cylinder sweep must lie strictly inside non-port host supports",
                );
            }
        }
    }
    Ok(())
}

fn certify_circle_inside_support(
    outward: Vec3,
    support_origin: Point3,
    frame: Frame,
    center: Point3,
    radius: f64,
) -> bool {
    let signed = interval_dot(outward, center - support_origin);
    if signed.hi() >= 0.0 {
        return false;
    }
    let radius = Interval::point(radius);
    let radial_x = interval_dot(outward, frame.x()) * radius;
    let radial_y = interval_dot(outward, frame.y()) * radius;
    let radial_squared = radial_x.square() + radial_y.square();
    radial_squared.hi() < signed.square().lo()
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

fn exact_dot(left: Vec3, right: Vec3) -> Result<Orientation> {
    affine_dot3(left.to_array(), right.to_array(), [0.0; 3], 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "two-port cylinder exact vector predicate is indeterminate",
        })
}

fn exact_affine(normal: Vec3, point: Point3, origin: Point3) -> Result<Orientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "two-port cylinder exact affine predicate is indeterminate",
        })
}

fn interval_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
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

    fn input(radius: f64) -> TwoPortCylinderSolidInput {
        TwoPortCylinderSolidInput::new(
            cube(),
            [1, 0],
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            radius,
            ParamRange::new(0.0, 2.0),
        )
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
    fn convex_host_two_port_cylinder_is_full_valid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_two_port_cylinder_solid(&input(0.75))
            .unwrap();
        let faces = transaction.store().faces_of_body(output.body()).unwrap();
        assert_eq!(
            topology_counts(transaction.store()),
            [1, 2, 1, 7, 10, 28, 14, 8]
        );
        assert_eq!(faces.len(), 7);
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
            output.port_faces(),
            [output.host_faces()[0], output.host_faces()[1]],
            "unordered input ports map into low/high axial order"
        );
        assert_eq!(
            transaction.store().get(output.side_face()).unwrap().sense(),
            Sense::Reversed
        );
        let mut loop_counts = faces
            .iter()
            .map(|face| transaction.store().get(*face).unwrap().loops().len())
            .collect::<Vec<_>>();
        loop_counts.sort_unstable();
        assert_eq!(loop_counts, [1, 1, 1, 1, 2, 2, 2]);
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
        assert_eq!((planes, cylinders), (6, 1));
        for ring in output.ring_edges() {
            let edge = transaction.store().get(ring).unwrap();
            assert_eq!(edge.vertices(), [None, None]);
            assert!(edge.bounds().is_none());
            assert_eq!(edge.fins().len(), 2);
        }

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
    fn wrong_cylinder_sense_is_full_invalid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_two_port_cylinder_solid(&input(0.75))
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
    fn malformed_two_port_inputs_fail_before_allocation() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = topology_counts(transaction.store());
        let duplicate_ports = TwoPortCylinderSolidInput::new(
            cube(),
            [0, 0],
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            0.5,
            ParamRange::new(0.0, 2.0),
        );
        let non_endpoint_port = TwoPortCylinderSolidInput::new(
            cube(),
            [0, 2],
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            0.5,
            ParamRange::new(0.0, 2.0),
        );
        let incomplete_span = TwoPortCylinderSolidInput::new(
            cube(),
            [0, 1],
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            0.5,
            ParamRange::new(0.0, 1.5),
        );
        for proposal in [
            input(1.0),
            duplicate_ports,
            non_endpoint_port,
            incomplete_span,
        ] {
            assert!(matches!(
                transaction.assemble_two_port_cylinder_solid(&proposal),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(topology_counts(transaction.store()), before);
        }
    }
}
