//! Primitive body constructors (PK_BODY_create_solid_* analogs).
//!
//! Every public constructor is failure-atomic, checker-gated, and returns a
//! clean body positioned by a caller [`Frame`]. A corresponding
//! `*_with_journal` variant exposes the deterministic committed mutation
//! journal:
//!
//! - [`block`]: centered at the frame origin, sides along the frame axes.
//! - [`cylinder`], [`cone`] (frustum): base disc in the frame's origin
//!   plane, axis `frame.z`; bounded by two vertex-less **ring edges** on
//!   circles (see `entity.rs` — a loop of one fin over a ring edge is
//!   closed by definition).
//! - [`sphere`], [`torus`]: a single face with **zero loops** covering the
//!   closed surface — no edges, no vertices.
//! - [`cylindrical_sheet`]: a full-period cylindrical face whose shared
//!   longitudinal edge has explicit paired lower/upper seam roles.
//! - [`planar_sheet`]: one simple polygonal profile on a plane, with an
//!   explicit line pcurve on every boundary use.
//! - [`wire_polyline`]: an open or closed chain of bounded line edges.
//! - [`acorn`]: one isolated model-space point.
//!
//! Full cones running to the apex (a degenerate vertex-only boundary) are
//! deferred; [`cone`] builds frustums with two positive radii.

use crate::entity::{
    Body, BodyId, BodyKind, Edge, EdgeId, Face, FaceDomain, FaceId, Fin, FinPcurve, Loop, LoopId,
    ParamMap1d, PcurveChart, PcurveSeam, Region, RegionKind, SeamSide, Sense, Shell, ShellId,
    SurfaceParameter, Vertex, VertexId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::profile::PlanarProfile;
use crate::store::Store;
use crate::transaction::Journal;
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::curve::{Circle, Line};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Torus};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};

fn frame_uv(frame: &Frame, point: Point3) -> Point2 {
    let relative = point - frame.origin();
    Point2::new(relative.dot(frame.x()), relative.dot(frame.y()))
}

fn frame_uv_vector(frame: &Frame, vector: Vec3) -> Vec2 {
    Vec2::new(vector.dot(frame.x()), vector.dot(frame.y()))
}

fn point_domain(points: impl IntoIterator<Item = Point2>) -> Result<FaceDomain> {
    let mut points = points.into_iter();
    let first = points.next().ok_or(Error::InvalidGeometry {
        reason: "cannot build a face domain from no points",
    })?;
    let (mut u_min, mut u_max, mut v_min, mut v_max) = (first.x, first.x, first.y, first.y);
    for point in points {
        u_min = u_min.min(point.x);
        u_max = u_max.max(point.x);
        v_min = v_min.min(point.y);
        v_max = v_max.max(point.y);
    }
    FaceDomain::from_bounds(u_min, u_max, v_min, v_max)
}

fn line_pcurve(
    store: &mut Store,
    surface_frame: &Frame,
    edge_start: Point3,
    edge_end: Point3,
    edge_length: f64,
) -> Result<FinPcurve> {
    let start = frame_uv(surface_frame, edge_start);
    let end = frame_uv(surface_frame, edge_end);
    let curve = store.add(Curve2dGeom::Line(Line2d::new(start, end - start)?));
    FinPcurve::new(
        curve,
        ParamRange::new(0.0, edge_length),
        ParamMap1d::identity(),
    )
}

/// Create the standard body scaffold: a solid body with its infinite void
/// exterior region, one solid region, and one shell owned by the solid
/// region. Returns `(body, shell)`.
pub(crate) fn solid_body_scaffold(store: &mut Store) -> (BodyId, ShellId) {
    let body = store.add(Body {
        kind: BodyKind::Solid,
        regions: Vec::new(),
    });
    let void = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    let solid = store.add(Region {
        body,
        kind: RegionKind::Solid,
        shells: Vec::new(),
    });
    let shell = store.add(Shell {
        region: solid,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store
        .get_mut(solid)
        .expect("fresh handle")
        .shells
        .push(shell);
    store.get_mut(body).expect("fresh handle").regions = vec![void, solid];
    (body, shell)
}

/// Validate a primitive dimension: finite and strictly positive.
pub(crate) fn positive_dimension(value: f64, what: &'static str) -> Result<f64> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        let _ = what;
        Err(Error::InvalidGeometry {
            reason: "primitive dimension must be finite and positive",
        })
    }
}

/// A failure-atomic body creation and its deterministic mutation journal.
#[derive(Debug)]
pub struct BodyCreation {
    body: BodyId,
    journal: Journal,
}

impl BodyCreation {
    /// Created body handle.
    pub fn body(&self) -> BodyId {
        self.body
    }

    /// Raw and semantic mutations committed by the creation operation.
    pub fn journal(&self) -> &Journal {
        &self.journal
    }

    /// Consume the result into its body and journal.
    pub fn into_parts(self) -> (BodyId, Journal) {
        (self.body, self.journal)
    }
}

fn checked_body_creation(
    store: &mut Store,
    create: impl FnOnce(&mut Store) -> Result<BodyId>,
) -> Result<BodyCreation> {
    let mut transaction = store.transaction()?;
    let body = create(transaction.store_mut())?;
    let journal = transaction.commit_checked_body(body)?;
    Ok(BodyCreation { body, journal })
}

/// Create a non-solid body with its void region and one empty shell.
fn non_solid_body_scaffold(store: &mut Store, kind: BodyKind) -> (BodyId, ShellId) {
    debug_assert!(kind != BodyKind::Solid);
    let body = store.add(Body {
        kind,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store
        .get_mut(region)
        .expect("fresh handle")
        .shells
        .push(shell);
    store
        .get_mut(body)
        .expect("fresh handle")
        .regions
        .push(region);
    (body, shell)
}

/// Create a failure-atomic sheet body from one simple planar polygon.
///
/// `polygon` is expressed in the `frame` XY plane. Clockwise input is
/// normalized; repeated, collinear-consecutive, sub-resolution, non-finite,
/// outside-size-box, and self-intersecting boundaries are rejected. Holes are
/// intentionally deferred to the profile-region builder.
pub fn planar_sheet(store: &mut Store, frame: &Frame, polygon: &[Point2]) -> Result<BodyId> {
    Ok(planar_sheet_with_journal(store, frame, polygon)?.body)
}

/// Failure-atomic [`planar_sheet`] creation with its deterministic journal.
pub fn planar_sheet_with_journal(
    store: &mut Store,
    frame: &Frame,
    polygon: &[Point2],
) -> Result<BodyCreation> {
    let profile = PlanarProfile::from_polygon(*frame, polygon)?;
    planar_sheet_from_profile_with_journal(store, &profile)
}

/// Create a sheet from a previously validated reusable planar profile.
pub fn planar_sheet_from_profile(store: &mut Store, profile: &PlanarProfile) -> Result<BodyId> {
    Ok(planar_sheet_from_profile_with_journal(store, profile)?.body)
}

/// Failure-atomic [`planar_sheet_from_profile`] creation with its journal.
pub fn planar_sheet_from_profile_with_journal(
    store: &mut Store,
    profile: &PlanarProfile,
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| planar_sheet_in(store, profile))
}

fn planar_sheet_in(store: &mut Store, profile: &PlanarProfile) -> Result<BodyId> {
    let frame = profile.frame();
    let polygon = profile.outer();
    let positions: Vec<_> = polygon
        .iter()
        .map(|point| frame.point_at(point.x, point.y, 0.0))
        .collect();
    let (body, shell) = non_solid_body_scaffold(store, BodyKind::Sheet);
    let surface = store.add(SurfaceGeom::Plane(Plane::new(*frame)));
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense: Sense::Forward,
        domain: Some(point_domain(polygon.iter().copied())?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(face);
    let loop_id = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    store.get_mut(face)?.loops.push(loop_id);
    let vertices: Vec<_> = positions
        .iter()
        .map(|&position| {
            let point = store.add(position);
            store.add(Vertex {
                point,
                tolerance: None,
            })
        })
        .collect();
    for index in 0..positions.len() {
        let next = (index + 1) % positions.len();
        let start = positions[index];
        let end = positions[next];
        let length = (end - start).norm();
        let curve = store.add(CurveGeom::Line(Line::new(start, end - start)?));
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(vertices[index]), Some(vertices[next])],
            bounds: Some((0.0, length)),
            fins: Vec::new(),
            tolerance: None,
        });
        let pcurve = line_pcurve(store, frame, start, end, length)?;
        let fin = store.add(Fin {
            parent: loop_id,
            edge,
            sense: Sense::Forward,
            pcurve: Some(pcurve),
        });
        store.get_mut(loop_id)?.fins.push(fin);
        store.get_mut(edge)?.fins.push(fin);
    }
    Ok(body)
}

/// Create a failure-atomic line-segment wire from ordered model-space points.
///
/// A closed wire adds the final edge back to the first point; callers must
/// not repeat the first point at the end. The current Full checker still
/// reports global wire self-intersection as `Indeterminate`.
pub fn wire_polyline(store: &mut Store, points: &[Point3], closed: bool) -> Result<BodyId> {
    Ok(wire_polyline_with_journal(store, points, closed)?.body)
}

/// Failure-atomic [`wire_polyline`] creation with its deterministic journal.
pub fn wire_polyline_with_journal(
    store: &mut Store,
    points: &[Point3],
    closed: bool,
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| wire_polyline_in(store, points, closed))
}

fn wire_polyline_in(store: &mut Store, points: &[Point3], closed: bool) -> Result<BodyId> {
    let minimum = if closed { 3 } else { 2 };
    if points.len() < minimum {
        return Err(Error::InvalidGeometry {
            reason: "wire polyline has too few points for its closure mode",
        });
    }
    for &point in points {
        check_in_size_box(point.to_array())?;
    }
    let edge_count = if closed {
        points.len()
    } else {
        points.len() - 1
    };
    for index in 0..edge_count {
        let next = (index + 1) % points.len();
        if (points[next] - points[index]).norm() <= LINEAR_RESOLUTION {
            return Err(Error::InvalidGeometry {
                reason: "wire polyline has a zero-length edge",
            });
        }
    }
    let (body, shell) = non_solid_body_scaffold(store, BodyKind::Wire);
    let vertices: Vec<_> = points
        .iter()
        .map(|&position| {
            let point = store.add(position);
            store.add(Vertex {
                point,
                tolerance: None,
            })
        })
        .collect();
    for index in 0..edge_count {
        let next = (index + 1) % points.len();
        let direction = points[next] - points[index];
        let length = direction.norm();
        let curve = store.add(CurveGeom::Line(Line::new(points[index], direction)?));
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(vertices[index]), Some(vertices[next])],
            bounds: Some((0.0, length)),
            fins: Vec::new(),
            tolerance: None,
        });
        store.get_mut(shell)?.edges.push(edge);
    }
    Ok(body)
}

/// Create a failure-atomic acorn body containing one isolated point.
pub fn acorn(store: &mut Store, position: Point3) -> Result<BodyId> {
    Ok(acorn_with_journal(store, position)?.body)
}

/// Failure-atomic [`acorn`] creation with its deterministic journal.
pub fn acorn_with_journal(store: &mut Store, position: Point3) -> Result<BodyCreation> {
    checked_body_creation(store, |store| acorn_in(store, position))
}

fn acorn_in(store: &mut Store, position: Point3) -> Result<BodyId> {
    check_in_size_box(position.to_array())?;
    let (body, shell) = non_solid_body_scaffold(store, BodyKind::Acorn);
    let point = store.add(position);
    let vertex = store.add(Vertex {
        point,
        tolerance: None,
    });
    store.get_mut(shell)?.vertex = Some(vertex);
    Ok(body)
}

/// Create a solid block centered at `frame`'s origin with side lengths
/// `extents = [dx, dy, dz]` along the frame's axes.
///
/// Topology: 8 vertices, 12 line edges, 6 planar faces (one loop of four
/// fins each), one shell, solid + void regions. Face normals point
/// outward; loops run counterclockwise seen from outside.
pub fn block(store: &mut Store, frame: &Frame, extents: [f64; 3]) -> Result<BodyId> {
    Ok(block_with_journal(store, frame, extents)?.body)
}

/// Failure-atomic [`block`] creation with its deterministic journal.
pub fn block_with_journal(
    store: &mut Store,
    frame: &Frame,
    extents: [f64; 3],
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| block_in(store, frame, extents))
}

fn block_in(store: &mut Store, frame: &Frame, extents: [f64; 3]) -> Result<BodyId> {
    let [dx, dy, dz] = extents;
    let hx = positive_dimension(dx, "block dx")? / 2.0;
    let hy = positive_dimension(dy, "block dy")? / 2.0;
    let hz = positive_dimension(dz, "block dz")? / 2.0;

    let (body, shell) = solid_body_scaffold(store);

    // Corner i has local coordinate bits (x, y, z) = (i&1, i&2, i&4).
    let mut vertices: Vec<VertexId> = Vec::with_capacity(8);
    let mut corners: Vec<Point3> = Vec::with_capacity(8);
    for i in 0..8u8 {
        let sx = if i & 1 == 0 { -hx } else { hx };
        let sy = if i & 2 == 0 { -hy } else { hy };
        let sz = if i & 4 == 0 { -hz } else { hz };
        let p = frame.point_at(sx, sy, sz);
        corners.push(p);
        let point = store.add(p);
        vertices.push(store.add(Vertex {
            point,
            tolerance: None,
        }));
    }

    // Each face: four corner indices, counterclockwise seen from outside,
    // and the outward normal in local frame coordinates.
    let faces: [([usize; 4], Vec3); 6] = [
        ([0, 2, 3, 1], Vec3::new(0.0, 0.0, -1.0)), // -Z
        ([4, 5, 7, 6], Vec3::new(0.0, 0.0, 1.0)),  // +Z
        ([0, 1, 5, 4], Vec3::new(0.0, -1.0, 0.0)), // -Y
        ([2, 6, 7, 3], Vec3::new(0.0, 1.0, 0.0)),  // +Y
        ([0, 4, 6, 2], Vec3::new(-1.0, 0.0, 0.0)), // -X
        ([1, 3, 7, 5], Vec3::new(1.0, 0.0, 0.0)),  // +X
    ];

    // Shared edges, keyed by (low corner, high corner); the edge's curve
    // always runs low → high so the second face using it gets a reversed
    // fin.
    let mut edge_of: Vec<((usize, usize), EdgeId)> = Vec::with_capacity(12);

    for (ring, local_normal) in faces {
        let center_local = ring.iter().fold(Vec3::new(0.0, 0.0, 0.0), |acc, &i| {
            let s = |bit: u8, h: f64| if i as u8 & bit == 0 { -h } else { h };
            acc + Vec3::new(s(1, hx), s(2, hy), s(4, hz))
        }) * 0.25;
        let origin = frame.point_at(center_local.x, center_local.y, center_local.z);
        let normal =
            frame.x() * local_normal.x + frame.y() * local_normal.y + frame.z() * local_normal.z;
        // Deterministic in-plane x: direction of the loop's first edge.
        let x_hint = corners[ring[1]] - corners[ring[0]];
        let plane = Plane::new(Frame::new(origin, normal, x_hint)?);
        let surface = store.add(SurfaceGeom::Plane(plane));
        let domain = point_domain(
            ring.iter()
                .map(|&index| frame_uv(plane.frame(), corners[index])),
        )?;

        let face: FaceId = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
            domain: Some(domain),
            tolerance: None,
        });
        store.get_mut(shell)?.faces.push(face);

        let lp = store.add(Loop {
            face,
            fins: Vec::new(),
        });
        store.get_mut(face)?.loops.push(lp);

        let mut fins = Vec::with_capacity(4);
        for k in 0..4 {
            let a = ring[k];
            let b = ring[(k + 1) % 4];
            let key = (a.min(b), a.max(b));
            let existing = edge_of.iter().find(|(k2, _)| *k2 == key).map(|&(_, e)| e);
            let edge = match existing {
                Some(e) => e,
                None => {
                    let (lo, hi) = key;
                    let dir = corners[hi] - corners[lo];
                    let len = dir.norm();
                    let line = Line::new(corners[lo], dir)?;
                    let curve = store.add(CurveGeom::Line(line));
                    let e = store.add(Edge {
                        curve: Some(curve),
                        vertices: [Some(vertices[lo]), Some(vertices[hi])],
                        bounds: Some((0.0, len)),
                        fins: Vec::new(),
                        tolerance: None,
                    });
                    edge_of.push((key, e));
                    e
                }
            };
            let sense = if a < b {
                Sense::Forward
            } else {
                Sense::Reversed
            };
            let (lo, hi) = (key.0, key.1);
            let pcurve = line_pcurve(
                store,
                plane.frame(),
                corners[lo],
                corners[hi],
                (corners[hi] - corners[lo]).norm(),
            )?;
            let fin = store.add(Fin {
                parent: lp,
                edge,
                sense,
                pcurve: Some(pcurve),
            });
            store.get_mut(edge)?.fins.push(fin);
            fins.push(fin);
        }
        store.get_mut(lp)?.fins = fins;
    }

    Ok(body)
}

/// Add one ring-edge boundary of a revolved side face: the vertex-less
/// ring edge on `circle`, a single-fin loop on the side face traversing it
/// with `side_sense`, and a planar cap face (frame `cap_frame`, normal =
/// `cap_frame.z` pointing out of the material) whose single-fin loop
/// traverses the ring edge with the opposite sense.
///
/// Fin-sense derivation: with the side face's normal outward and its
/// `u` parameter running with the circle, the band's low-`v` boundary is
/// traversed in `+u` (`Forward`) and the high-`v` boundary in `−u`
/// (`Reversed`) to keep the interior on the left; the cap, seeing the
/// edge from the other side, always uses the complement.
fn ring_boundary(
    store: &mut Store,
    shell: ShellId,
    side_face: FaceId,
    circle: Circle,
    side_sense: Sense,
    side_v: f64,
    cap_frame: Frame,
) -> Result<EdgeId> {
    let curve = store.add(CurveGeom::Circle(circle));
    let edge = store.add(Edge {
        curve: Some(curve),
        vertices: [None, None],
        bounds: None,
        fins: Vec::new(),
        tolerance: None,
    });

    let range = ParamRange::new(0.0, core::f64::consts::TAU);
    let side_curve = store.add(Curve2dGeom::Line(Line2d::new(
        Point2::new(0.0, side_v),
        Vec2::new(1.0, 0.0),
    )?));
    let side_pcurve =
        FinPcurve::new(side_curve, range, ParamMap1d::identity())?.with_closure_winding([1, 0]);
    let side_loop: LoopId = store.add(Loop {
        face: side_face,
        fins: Vec::new(),
    });
    store.get_mut(side_face)?.loops.push(side_loop);
    let side_fin = store.add(Fin {
        parent: side_loop,
        edge,
        sense: side_sense,
        pcurve: Some(side_pcurve),
    });
    store.get_mut(side_loop)?.fins.push(side_fin);

    let plane = Plane::new(cap_frame);
    let cap_surface = store.add(SurfaceGeom::Plane(plane));
    let cap = store.add(Face {
        shell,
        loops: Vec::new(),
        surface: cap_surface,
        sense: Sense::Forward,
        domain: Some(FaceDomain::from_bounds(
            -circle.radius(),
            circle.radius(),
            -circle.radius(),
            circle.radius(),
        )?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(cap);
    let cap_loop = store.add(Loop {
        face: cap,
        fins: Vec::new(),
    });
    store.get_mut(cap)?.loops.push(cap_loop);

    // Express the edge circle directly in the cap's parameter space. The
    // cap frame can reverse handedness relative to the edge circle, so the
    // affine map carries that seam-safe orientation explicitly.
    let circle_x = frame_uv_vector(&cap_frame, circle.frame().x());
    let circle_y = frame_uv_vector(&cap_frame, circle.frame().y());
    let cap_curve = Circle2d::new(Point2::new(0.0, 0.0), circle.radius(), circle_x)?;
    let map = if circle_y.dot(cap_curve.x_dir().perp()) >= 0.0 {
        ParamMap1d::identity()
    } else {
        ParamMap1d::affine(-1.0, core::f64::consts::TAU)?
    };
    let cap_curve = store.add(Curve2dGeom::Circle(cap_curve));
    let cap_pcurve = FinPcurve::new(cap_curve, range, map)?.with_closure_winding([0, 0]);
    let cap_fin = store.add(Fin {
        parent: cap_loop,
        edge,
        sense: side_sense.flipped(),
        pcurve: Some(cap_pcurve),
    });
    store.get_mut(cap_loop)?.fins.push(cap_fin);

    store.get_mut(edge)?.fins = vec![side_fin, cap_fin];
    Ok(edge)
}

/// Create a solid cylinder: base disc in `frame`'s origin plane, axis
/// `frame.z`, extending to `height`.
///
/// Topology: one side face on the cylinder surface with two single-fin
/// ring loops, two planar caps, two ring edges — no vertices. The side
/// face's `u` runs with both circles; the base circle (low `v`) is
/// traversed `Forward`, the top (high `v`) `Reversed`.
pub fn cylinder(store: &mut Store, frame: &Frame, radius: f64, height: f64) -> Result<BodyId> {
    Ok(cylinder_with_journal(store, frame, radius, height)?.body)
}

/// Failure-atomic [`cylinder`] creation with its deterministic journal.
pub fn cylinder_with_journal(
    store: &mut Store,
    frame: &Frame,
    radius: f64,
    height: f64,
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| cylinder_in(store, frame, radius, height))
}

fn cylinder_in(store: &mut Store, frame: &Frame, radius: f64, height: f64) -> Result<BodyId> {
    let radius = positive_dimension(radius, "cylinder radius")?;
    let height = positive_dimension(height, "cylinder height")?;

    let (body, shell) = solid_body_scaffold(store);
    let surface = Cylinder::new(*frame, radius)?;
    let side_surf = store.add(SurfaceGeom::Cylinder(surface));
    let side = store.add(Face {
        shell,
        loops: Vec::new(),
        surface: side_surf,
        sense: Sense::Forward, // cylinder normal is radially outward
        domain: Some(FaceDomain::from_bounds(
            0.0,
            core::f64::consts::TAU,
            0.0,
            height,
        )?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(side);

    let top_origin = frame.origin() + frame.z() * height;
    let bottom_circle = Circle::new(*frame, radius)?;
    let top_circle = Circle::new(Frame::new(top_origin, frame.z(), frame.x())?, radius)?;
    let bottom_cap = Frame::new(frame.origin(), -frame.z(), frame.x())?;
    let top_cap = Frame::new(top_origin, frame.z(), frame.x())?;

    ring_boundary(
        store,
        shell,
        side,
        bottom_circle,
        Sense::Forward,
        0.0,
        bottom_cap,
    )?;
    ring_boundary(
        store,
        shell,
        side,
        top_circle,
        Sense::Reversed,
        height,
        top_cap,
    )?;
    Ok(body)
}

/// Create a full-period cylindrical sheet with an explicit longitudinal
/// seam, extending from `frame.origin()` through `height * frame.z()`.
///
/// The seam is one shared 3D edge used twice by the same face. Its two fins
/// carry complementary lower/upper [`PcurveSeam`] roles; the upper use
/// selects the next integer-period chart without duplicating pcurve
/// geometry. The circular boundaries are bounded closed edges so they can
/// participate in the four-fin chart-boundary loop.
pub fn cylindrical_sheet(
    store: &mut Store,
    frame: &Frame,
    radius: f64,
    height: f64,
) -> Result<BodyId> {
    Ok(cylindrical_sheet_with_journal(store, frame, radius, height)?.body)
}

/// Failure-atomic [`cylindrical_sheet`] creation with its deterministic journal.
pub fn cylindrical_sheet_with_journal(
    store: &mut Store,
    frame: &Frame,
    radius: f64,
    height: f64,
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| {
        cylindrical_sheet_in(store, frame, radius, height)
    })
}

fn cylindrical_sheet_in(
    store: &mut Store,
    frame: &Frame,
    radius: f64,
    height: f64,
) -> Result<BodyId> {
    let radius = positive_dimension(radius, "cylindrical sheet radius")?;
    let height = positive_dimension(height, "cylindrical sheet height")?;
    let tau = core::f64::consts::TAU;

    let body = store.add(Body {
        kind: BodyKind::Sheet,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body)?.regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region)?.shells.push(shell);

    let surface = store.add(SurfaceGeom::Cylinder(Cylinder::new(*frame, radius)?));
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense: Sense::Forward,
        domain: Some(FaceDomain::from_bounds(0.0, tau, 0.0, height)?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(face);
    let loop_id = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    store.get_mut(face)?.loops.push(loop_id);

    let bottom = frame.origin() + frame.x() * radius;
    let top_origin = frame.origin() + frame.z() * height;
    let top = top_origin + frame.x() * radius;
    let vertices = [bottom, top].map(|position| {
        let point = store.add(position);
        store.add(Vertex {
            point,
            tolerance: None,
        })
    });
    let seam_curve = store.add(CurveGeom::Line(Line::new(bottom, frame.z())?));
    let seam_edge = store.add(Edge {
        curve: Some(seam_curve),
        vertices: [Some(vertices[0]), Some(vertices[1])],
        bounds: Some((0.0, height)),
        fins: Vec::new(),
        tolerance: None,
    });
    let bottom_curve = store.add(CurveGeom::Circle(Circle::new(*frame, radius)?));
    let bottom_edge = store.add(Edge {
        curve: Some(bottom_curve),
        vertices: [Some(vertices[0]), Some(vertices[0])],
        bounds: Some((0.0, tau)),
        fins: Vec::new(),
        tolerance: None,
    });
    let top_frame = Frame::new(top_origin, frame.z(), frame.x())?;
    let top_curve = store.add(CurveGeom::Circle(Circle::new(top_frame, radius)?));
    let top_edge = store.add(Edge {
        curve: Some(top_curve),
        vertices: [Some(vertices[1]), Some(vertices[1])],
        bounds: Some((0.0, tau)),
        fins: Vec::new(),
        tolerance: None,
    });

    let horizontal = |store: &mut Store, v: f64| -> Result<_> {
        Ok(store.add(Curve2dGeom::Line(Line2d::new(
            Point2::new(0.0, v),
            Vec2::new(1.0, 0.0),
        )?)))
    };
    let vertical = |store: &mut Store| -> Result<_> {
        Ok(store.add(Curve2dGeom::Line(Line2d::new(
            Point2::new(0.0, 0.0),
            Vec2::new(0.0, 1.0),
        )?)))
    };
    let bottom_pcurve = FinPcurve::new(
        horizontal(store, 0.0)?,
        ParamRange::new(0.0, tau),
        ParamMap1d::identity(),
    )?
    .with_closure_winding([1, 0]);
    let seam_upper = FinPcurve::new(
        vertical(store)?,
        ParamRange::new(0.0, height),
        ParamMap1d::identity(),
    )?
    .with_chart(PcurveChart::shifted([1, 0]))
    .with_seam(PcurveSeam::new(SurfaceParameter::U, SeamSide::Upper));
    let top_pcurve = FinPcurve::new(
        horizontal(store, height)?,
        ParamRange::new(0.0, tau),
        ParamMap1d::identity(),
    )?
    .with_closure_winding([1, 0]);
    let seam_lower = FinPcurve::new(
        vertical(store)?,
        ParamRange::new(0.0, height),
        ParamMap1d::identity(),
    )?
    .with_seam(PcurveSeam::new(SurfaceParameter::U, SeamSide::Lower));

    for (edge, sense, pcurve) in [
        (bottom_edge, Sense::Forward, bottom_pcurve),
        (seam_edge, Sense::Forward, seam_upper),
        (top_edge, Sense::Reversed, top_pcurve),
        (seam_edge, Sense::Reversed, seam_lower),
    ] {
        let fin = store.add(Fin {
            parent: loop_id,
            edge,
            sense,
            pcurve: Some(pcurve),
        });
        store.get_mut(edge)?.fins.push(fin);
        store.get_mut(loop_id)?.fins.push(fin);
    }
    Ok(body)
}

/// Create a solid cone frustum: base disc of radius `base_radius` in
/// `frame`'s origin plane, top disc of radius `top_radius` at
/// `height` along `frame.z`. The radii must differ (use [`cylinder`]
/// otherwise); full cones ending at the apex are deferred.
///
/// The supporting [`Cone`] expands along its own `+z`, so a shrinking
/// frustum (`top_radius < base_radius`) is built on a flipped surface
/// frame (`z' = −frame.z`); the base circle then sits at the band's
/// high-`v` end and the fin senses swap accordingly.
pub fn cone(
    store: &mut Store,
    frame: &Frame,
    base_radius: f64,
    top_radius: f64,
    height: f64,
) -> Result<BodyId> {
    Ok(cone_with_journal(store, frame, base_radius, top_radius, height)?.body)
}

/// Failure-atomic [`cone`] creation with its deterministic journal.
pub fn cone_with_journal(
    store: &mut Store,
    frame: &Frame,
    base_radius: f64,
    top_radius: f64,
    height: f64,
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| {
        cone_in(store, frame, base_radius, top_radius, height)
    })
}

fn cone_in(
    store: &mut Store,
    frame: &Frame,
    base_radius: f64,
    top_radius: f64,
    height: f64,
) -> Result<BodyId> {
    let base_radius = positive_dimension(base_radius, "cone base radius")?;
    let top_radius = positive_dimension(top_radius, "cone top radius")?;
    let height = positive_dimension(height, "cone height")?;
    if base_radius == top_radius {
        return Err(Error::InvalidGeometry {
            reason: "cone radii are equal; use cylinder",
        });
    }
    let dr = top_radius - base_radius;
    // tan α = |Δr| / h with α ∈ (0, π/2): no trig needed beyond atan2.
    // (The band then spans slant length √(Δr² + h²) along v.)
    let alpha = math::atan2(dr.abs(), height);

    let expanding = dr > 0.0;
    let surf_frame = if expanding {
        *frame
    } else {
        Frame::new(frame.origin(), -frame.z(), frame.x())?
    };
    let surface = Cone::new(surf_frame, base_radius, alpha)?;
    let slant = (dr * dr + height * height).sqrt();
    let top_v = if expanding { slant } else { -slant };

    let (body, shell) = solid_body_scaffold(store);
    let side_surf = store.add(SurfaceGeom::Cone(surface));
    let side = store.add(Face {
        shell,
        loops: Vec::new(),
        surface: side_surf,
        // du × dv points out of the material for both frame choices.
        sense: Sense::Forward,
        domain: Some(FaceDomain::from_bounds(
            0.0,
            core::f64::consts::TAU,
            top_v.min(0.0),
            top_v.max(0.0),
        )?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(side);

    let top_origin = frame.origin() + frame.z() * height;
    // Circles use the *surface* frame's axes so their parameter runs with
    // the surface's u in both cases.
    let bottom_circle = Circle::new(
        Frame::new(frame.origin(), surf_frame.z(), surf_frame.x())?,
        base_radius,
    )?;
    let top_circle = Circle::new(
        Frame::new(top_origin, surf_frame.z(), surf_frame.x())?,
        top_radius,
    )?;
    let bottom_cap = Frame::new(frame.origin(), -frame.z(), frame.x())?;
    let top_cap = Frame::new(top_origin, frame.z(), frame.x())?;

    // Expanding: base at v = 0 (low), top at v = +slant (high).
    // Shrinking: base at v = 0 (high), top at v = −slant (low).
    let (bottom_sense, top_sense) = if expanding {
        (Sense::Forward, Sense::Reversed)
    } else {
        (Sense::Reversed, Sense::Forward)
    };
    ring_boundary(
        store,
        shell,
        side,
        bottom_circle,
        bottom_sense,
        0.0,
        bottom_cap,
    )?;
    ring_boundary(store, shell, side, top_circle, top_sense, top_v, top_cap)?;
    Ok(body)
}

/// Create a solid sphere centered at `frame`'s origin: a single face with
/// zero loops covering the closed surface — no edges, no vertices.
pub fn sphere(store: &mut Store, frame: &Frame, radius: f64) -> Result<BodyId> {
    Ok(sphere_with_journal(store, frame, radius)?.body)
}

/// Failure-atomic [`sphere`] creation with its deterministic journal.
pub fn sphere_with_journal(store: &mut Store, frame: &Frame, radius: f64) -> Result<BodyCreation> {
    checked_body_creation(store, |store| sphere_in(store, frame, radius))
}

fn sphere_in(store: &mut Store, frame: &Frame, radius: f64) -> Result<BodyId> {
    let radius = positive_dimension(radius, "sphere radius")?;
    let (body, shell) = solid_body_scaffold(store);
    let surface = store.add(SurfaceGeom::Sphere(Sphere::new(*frame, radius)?));
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense: Sense::Forward, // sphere normal is radially outward
        domain: Some(FaceDomain::from_bounds(
            0.0,
            core::f64::consts::TAU,
            -core::f64::consts::FRAC_PI_2,
            core::f64::consts::FRAC_PI_2,
        )?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(face);
    Ok(body)
}

/// Create a solid torus centered at `frame`'s origin, spine in the
/// origin plane around `frame.z`, with `major_radius > minor_radius > 0`:
/// a single zero-loop face like [`sphere`].
pub fn torus(
    store: &mut Store,
    frame: &Frame,
    major_radius: f64,
    minor_radius: f64,
) -> Result<BodyId> {
    Ok(torus_with_journal(store, frame, major_radius, minor_radius)?.body)
}

/// Failure-atomic [`torus`] creation with its deterministic journal.
pub fn torus_with_journal(
    store: &mut Store,
    frame: &Frame,
    major_radius: f64,
    minor_radius: f64,
) -> Result<BodyCreation> {
    checked_body_creation(store, |store| {
        torus_in(store, frame, major_radius, minor_radius)
    })
}

fn torus_in(
    store: &mut Store,
    frame: &Frame,
    major_radius: f64,
    minor_radius: f64,
) -> Result<BodyId> {
    let major_radius = positive_dimension(major_radius, "torus major radius")?;
    let minor_radius = positive_dimension(minor_radius, "torus minor radius")?;
    let (body, shell) = solid_body_scaffold(store);
    let surface = store.add(SurfaceGeom::Torus(Torus::new(
        *frame,
        major_radius,
        minor_radius,
    )?));
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense: Sense::Forward, // torus normal points out of the tube
        domain: Some(FaceDomain::from_bounds(
            0.0,
            core::f64::consts::TAU,
            0.0,
            core::f64::consts::TAU,
        )?),
        tolerance: None,
    });
    store.get_mut(shell)?.faces.push(face);
    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::btess::{TessOptions, tessellate_body};
    use crate::check::check_body;
    use crate::entity::{Edge, Face, Fin, Loop, Region, Shell, Vertex};
    use kgeom::curve::Curve;
    use kgeom::surface::Surface;

    fn tilted() -> Frame {
        Frame::new(
            Point3::new(0.3, -1.2, 2.1),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn block_has_expected_entity_counts() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        assert_eq!(store.count::<Vertex>(), 8);
        assert_eq!(store.count::<Edge>(), 12);
        assert_eq!(store.count::<Face>(), 6);
        assert_eq!(store.count::<Loop>(), 6);
        assert_eq!(store.count::<Fin>(), 24);
        assert_eq!(store.count::<Shell>(), 1);
        assert_eq!(store.count::<Region>(), 2);
        assert_eq!(store.faces_of_body(body).unwrap().len(), 6);
        assert_eq!(store.edges_of_body(body).unwrap().len(), 12);
        assert_eq!(store.vertices_of_body(body).unwrap().len(), 8);
        assert!(
            store
                .faces_of_body(body)
                .unwrap()
                .into_iter()
                .all(|face| store.get(face).unwrap().domain.is_some())
        );
    }

    #[test]
    fn block_loops_are_closed_rings_and_edges_manifold() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        for face in store.faces_of_body(body).unwrap() {
            for &lp in &store.get(face).unwrap().loops {
                let fins = &store.get(lp).unwrap().fins;
                assert_eq!(fins.len(), 4);
                for (k, &fin) in fins.iter().enumerate() {
                    let next = fins[(k + 1) % fins.len()];
                    assert_eq!(
                        store.fin_head(fin).unwrap(),
                        store.fin_tail(next).unwrap(),
                        "loop ring must close"
                    );
                }
            }
        }
        for edge in store.edges_of_body(body).unwrap() {
            let e = store.get(edge).unwrap();
            assert_eq!(e.fins.len(), 2, "manifold interior edge has two fins");
            let s0 = store.get(e.fins[0]).unwrap().sense;
            let s1 = store.get(e.fins[1]).unwrap().sense;
            assert_ne!(s0, s1, "the two fins traverse the edge oppositely");
        }
    }

    #[test]
    fn block_rejects_bad_extents() {
        let mut store = Store::new();
        assert!(block(&mut store, &Frame::world(), [0.0, 1.0, 1.0]).is_err());
        assert!(block(&mut store, &Frame::world(), [1.0, -1.0, 1.0]).is_err());
        assert!(block(&mut store, &Frame::world(), [1.0, 1.0, f64::NAN]).is_err());
    }

    /// Shared structural assertions for cylinder/cone bodies.
    fn assert_revolved_frustum_shape(store: &Store, body: BodyId) {
        assert_eq!(store.faces_of_body(body).unwrap().len(), 3);
        assert_eq!(store.edges_of_body(body).unwrap().len(), 2);
        assert_eq!(store.vertices_of_body(body).unwrap().len(), 0);
        let faces = store.faces_of_body(body).unwrap();
        // Side face first (construction order), then the two caps.
        assert_eq!(store.get(faces[0]).unwrap().loops.len(), 2);
        assert_eq!(store.get(faces[1]).unwrap().loops.len(), 1);
        assert_eq!(store.get(faces[2]).unwrap().loops.len(), 1);
        for edge in store.edges_of_body(body).unwrap() {
            let e = store.get(edge).unwrap();
            assert_eq!(e.bounds, None, "ring edge");
            assert_eq!(e.vertices, [None, None], "ring edge has no vertices");
            assert_eq!(e.fins.len(), 2, "manifold ring edge has two fins");
            let s0 = store.get(e.fins[0]).unwrap().sense;
            let s1 = store.get(e.fins[1]).unwrap().sense;
            assert_ne!(s0, s1, "fins traverse the ring edge oppositely");
        }
        for face in faces {
            for &lp in &store.get(face).unwrap().loops {
                let fins = &store.get(lp).unwrap().fins;
                assert_eq!(fins.len(), 1, "single-fin ring loop");
                // A single fin over a ring edge closes trivially: both
                // ends are None.
                assert_eq!(store.fin_head(fins[0]).unwrap(), None);
                assert_eq!(store.fin_tail(fins[0]).unwrap(), None);
            }
        }
    }

    /// The two ring edges of a revolved body, in construction order
    /// (bottom, top), with their circles.
    fn ring_circles(store: &Store, body: BodyId) -> Vec<Circle> {
        store
            .edges_of_body(body)
            .unwrap()
            .iter()
            .map(|&e| {
                let curve = store.get(e).unwrap().curve.unwrap();
                match store.get(curve).unwrap() {
                    CurveGeom::Circle(c) => *c,
                    other => panic!("ring edge curve should be a circle, got {other:?}"),
                }
            })
            .collect()
    }

    #[test]
    fn cylinder_counts_and_ring_structure() {
        for frame in [Frame::world(), tilted()] {
            let mut store = Store::new();
            let body = cylinder(&mut store, &frame, 1.5, 4.0).unwrap();
            assert_revolved_frustum_shape(&store, body);
        }
    }

    #[test]
    fn cylindrical_sheet_has_paired_seam_roles() {
        for frame in [Frame::world(), tilted()] {
            let mut store = Store::new();
            let body = cylindrical_sheet(&mut store, &frame, 1.5, 4.0).unwrap();
            let faults = check_body(&store, body).unwrap();
            assert!(faults.is_empty(), "cylindrical sheet faults: {faults:?}");
            assert_eq!(store.faces_of_body(body).unwrap().len(), 1);
            assert_eq!(store.edges_of_body(body).unwrap().len(), 3);
            assert_eq!(store.vertices_of_body(body).unwrap().len(), 2);
            let seam_edge = store
                .edges_of_body(body)
                .unwrap()
                .into_iter()
                .find(|&edge| store.get(edge).unwrap().fins.len() == 2)
                .unwrap();
            let mut sides: Vec<_> = store
                .get(seam_edge)
                .unwrap()
                .fins
                .iter()
                .map(|&fin| {
                    store
                        .get(fin)
                        .unwrap()
                        .pcurve
                        .unwrap()
                        .seam()
                        .unwrap()
                        .side()
                })
                .collect();
            sides.sort_by_key(|side| match side {
                SeamSide::Lower => 0,
                SeamSide::Upper => 1,
            });
            assert_eq!(sides, vec![SeamSide::Lower, SeamSide::Upper]);
            let mesh = tessellate_body(
                &store,
                body,
                &TessOptions {
                    chord_tol: 1e-2,
                    max_edge_len: Some(0.5),
                },
            )
            .unwrap();
            assert!(!mesh.triangles.is_empty());
        }
    }

    #[test]
    fn cylinder_circles_lie_on_side_surface_and_caps() {
        let frame = tilted();
        let mut store = Store::new();
        let body = cylinder(&mut store, &frame, 1.5, 4.0).unwrap();
        let faces = store.faces_of_body(body).unwrap();
        let side_surface = store.get(store.get(faces[0]).unwrap().surface).unwrap();
        let SurfaceGeom::Cylinder(cyl) = side_surface else {
            panic!("side face must sit on the cylinder");
        };
        let circles = ring_circles(&store, body);
        for (circle, v) in [(circles[0], 0.0), (circles[1], 4.0)] {
            for k in 0..7 {
                let u = k as f64;
                let on_surface = cyl.eval([u, v]);
                let on_circle = circle.eval(u);
                assert!(
                    (on_surface - on_circle).norm() < 1e-12,
                    "circle parameter must run with surface u"
                );
                // Outward normal: radially away from the axis.
                let radial = on_circle - (frame.origin() + frame.z() * v);
                let n = cyl.normal([u, v]).unwrap();
                assert!(n.dot(radial) > 0.0, "side normal points outward");
            }
        }
        // Caps: circles lie in the cap planes, normals point out of the
        // material (down at the base, up at the top).
        for (face, expected_n, v) in [(faces[1], -frame.z(), 0.0), (faces[2], frame.z(), 4.0)] {
            let SurfaceGeom::Plane(plane) = store.get(store.get(face).unwrap().surface).unwrap()
            else {
                panic!("cap must be planar");
            };
            assert!((plane.frame().z() - expected_n).norm() < 1e-15);
            let circle = &ring_circles(&store, body)[if v == 0.0 { 0 } else { 1 }];
            for k in 0..7 {
                let p = circle.eval(k as f64);
                let d = (p - plane.frame().origin()).dot(plane.frame().z());
                assert!(d.abs() < 1e-12, "circle lies in the cap plane");
            }
        }
    }

    #[test]
    fn cone_expanding_and_shrinking_frustums() {
        for (rb, rt) in [(1.0, 2.0), (2.0, 0.8)] {
            for frame in [Frame::world(), tilted()] {
                let mut store = Store::new();
                let body = cone(&mut store, &frame, rb, rt, 3.0).unwrap();
                assert_revolved_frustum_shape(&store, body);

                let faces = store.faces_of_body(body).unwrap();
                let SurfaceGeom::Cone(surf) =
                    store.get(store.get(faces[0]).unwrap().surface).unwrap()
                else {
                    panic!("side face must sit on the cone");
                };
                let dr: f64 = rt - rb;
                let slant = (dr * dr + 9.0).sqrt();
                let (v_bottom, v_top) = if dr > 0.0 {
                    (0.0, slant)
                } else {
                    (0.0, -slant)
                };
                let circles = ring_circles(&store, body);
                for (circle, v, radius, z) in [
                    (circles[0], v_bottom, rb, 0.0),
                    (circles[1], v_top, rt, 3.0),
                ] {
                    for k in 0..7 {
                        let u = k as f64;
                        let on_surface = surf.eval([u, v]);
                        let on_circle = circle.eval(u);
                        assert!(
                            (on_surface - on_circle).norm() < 1e-12,
                            "circle parameter must run with surface u (dr={dr})"
                        );
                        // Radius and axial position are as requested.
                        let center = frame.origin() + frame.z() * z;
                        let radial = on_circle - center;
                        assert!((radial.norm() - radius).abs() < 1e-12);
                        assert!(radial.dot(frame.z()).abs() < 1e-12);
                        // Outward normal.
                        let n = surf.normal([u, v]).unwrap();
                        assert!(n.dot(radial) > 0.0, "side normal points outward");
                    }
                }
            }
        }
    }

    #[test]
    fn sphere_and_torus_are_single_closed_faces() {
        let mut store = Store::new();
        let s = sphere(&mut store, &tilted(), 2.0).unwrap();
        assert_eq!(store.faces_of_body(s).unwrap().len(), 1);
        assert_eq!(store.edges_of_body(s).unwrap().len(), 0);
        assert_eq!(store.vertices_of_body(s).unwrap().len(), 0);
        let face = store.faces_of_body(s).unwrap()[0];
        assert!(store.get(face).unwrap().loops.is_empty());
        assert_eq!(store.get(face).unwrap().sense, Sense::Forward);

        let t = torus(&mut store, &Frame::world(), 3.0, 0.8).unwrap();
        let face = store.faces_of_body(t).unwrap()[0];
        assert!(store.get(face).unwrap().loops.is_empty());
        // Both bodies share the store without interference.
        assert_eq!(store.count::<Face>(), 2);
        assert_eq!(store.count::<Region>(), 4);
    }

    #[test]
    fn revolved_primitives_reject_bad_dimensions() {
        let mut store = Store::new();
        let f = Frame::world();
        assert!(cylinder(&mut store, &f, 0.0, 1.0).is_err());
        assert!(cylinder(&mut store, &f, 1.0, -1.0).is_err());
        assert!(cone(&mut store, &f, 1.0, 1.0, 1.0).is_err(), "equal radii");
        assert!(cone(&mut store, &f, 0.0, 1.0, 1.0).is_err());
        assert!(cone(&mut store, &f, 1.0, 2.0, f64::INFINITY).is_err());
        assert!(sphere(&mut store, &f, f64::NAN).is_err());
        assert!(torus(&mut store, &f, 1.0, 2.0).is_err(), "r >= R");
        assert!(torus(&mut store, &f, 1.0, 0.0).is_err());
    }
}
