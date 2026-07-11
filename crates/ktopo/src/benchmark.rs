//! Read-only benchmark observations.
//!
//! This module exists only with `benchmark-internals`. Its values summarize
//! ordinary checked commits and index audits without exposing topology
//! collections, index maps, or mutable benchmark-only operations.

use crate::entity::{
    Body, BodyKind, Curve2dId, CurveId, Edge, EntityRef, Face, Fin, Loop, PcurveEndpointKind,
    Region, RegionKind, SeamSide, Sense, Shell, SurfaceId, SurfaceParameter, Vertex,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::index::StoreIndex;
use crate::store::Store;
use core::hash::Hasher;
use kcore::arena::Handle;
use kgeom::vec::Point3;
use std::collections::HashMap;

/// Immutable counters captured from the most recent checked-commit attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommitObservation {
    /// Whether validation succeeded and the mutation was installed.
    pub committed: bool,
    /// Bodies in the store at validation time.
    pub body_count: usize,
    /// Bodies selected from old/new ownership and dependency indexes.
    pub affected_bodies: usize,
    /// Body footprints refreshed in the candidate index.
    pub refreshed_bodies: usize,
    /// Body checker obligations actually started.
    pub checked_bodies: usize,
    /// Deterministic net mutations presented to checked commit.
    pub mutations: usize,
    /// Stable digest of the ordered affected-body sequence.
    pub affected_order_digest: u64,
}

/// Immutable structural summary of an ownership/dependency index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IndexSnapshot {
    /// Indexed bodies.
    pub bodies: usize,
    /// Body-owned topology entries.
    pub ownership_entries: usize,
    /// Body-to-geometry dependency entries.
    pub dependency_entries: usize,
    /// Ownership-closure faults found while building the index.
    pub ownership_faults: usize,
    /// Explicitly stable digest of deterministic body footprints.
    pub digest: u64,
}

/// Stable live-entity counts and deterministic store digest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StoreSnapshot {
    /// Live body count.
    pub bodies: usize,
    /// Live region count.
    pub regions: usize,
    /// Live shell count.
    pub shells: usize,
    /// Live face count.
    pub faces: usize,
    /// Live loop count.
    pub loops: usize,
    /// Live fin count.
    pub fins: usize,
    /// Live edge count.
    pub edges: usize,
    /// Live vertex count.
    pub vertices: usize,
    /// Live point count.
    pub points: usize,
    /// Live 3D curve count.
    pub curves: usize,
    /// Live surface count.
    pub surfaces: usize,
    /// Live pcurve count.
    pub pcurves: usize,
    /// Explicitly stable digest of slot-ordered entity values.
    pub digest: u64,
}

/// Full-rebuild comparison against the installed incremental index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IndexAudit {
    /// Installed committed index summary.
    pub committed: IndexSnapshot,
    /// Independently rebuilt index summary.
    pub rebuilt: IndexSnapshot,
    /// Structural equality, including private maps and deterministic order.
    pub structurally_equal: bool,
}

/// Return the latest checked-commit observation, if any commit was attempted.
pub fn last_commit(store: &Store) -> Option<CommitObservation> {
    store.benchmark_observation()
}

/// Snapshot the installed index without exposing its representation.
pub fn index_snapshot(store: &Store) -> IndexSnapshot {
    store.committed_index().benchmark_snapshot(store)
}

/// Build an independent reference index and compare it structurally.
pub fn audit_full_rebuild(store: &Store) -> IndexAudit {
    let rebuilt = StoreIndex::build(store);
    IndexAudit {
        committed: store.committed_index().benchmark_snapshot(store),
        rebuilt: rebuilt.benchmark_snapshot(store),
        structurally_equal: &rebuilt == store.committed_index(),
    }
}

/// Snapshot deterministic model contents for rollback checks.
pub fn store_snapshot(store: &Store) -> StoreSnapshot {
    fn ordinals<T: crate::store::Entity>(store: &Store) -> HashMap<Handle<T>, u64> {
        store
            .iter::<T>()
            .enumerate()
            .map(|(ordinal, (handle, _))| (handle, ordinal as u64))
            .collect()
    }
    fn write_ref<T>(
        digest: &mut StableHasher,
        handle: Handle<T>,
        ordinals: &HashMap<Handle<T>, u64>,
    ) {
        digest.write_ordinal(ordinals.get(&handle).copied());
    }
    fn write_refs<T>(
        digest: &mut StableHasher,
        handles: &[Handle<T>],
        ordinals: &HashMap<Handle<T>, u64>,
    ) {
        digest.write_count(handles.len());
        for &handle in handles {
            write_ref(digest, handle, ordinals);
        }
    }
    fn write_tolerance(
        digest: &mut StableHasher,
        tolerance: Option<crate::tolerance::EntityTolerance>,
    ) {
        use crate::tolerance::ToleranceOrigin;
        let Some(tolerance) = tolerance else {
            digest.write_tag(0);
            return;
        };
        digest.write_tag(1);
        digest.write_f64(tolerance.value());
        match tolerance.origin() {
            ToleranceOrigin::ImportedXt => digest.write_tag(0),
            ToleranceOrigin::Operation(operation) => {
                digest.write_tag(1);
                digest.write_bytes(operation.as_bytes());
            }
        }
        digest.write_f64(tolerance.origin_value());
        digest.write_f64(tolerance.accumulated_growth());
        if let Some(operation) = tolerance.last_operation() {
            digest.write_tag(1);
            digest.write_bytes(operation.as_bytes());
        } else {
            digest.write_tag(0);
        }
    }
    fn write_point2(digest: &mut StableHasher, point: kgeom::vec::Point2) {
        digest.write_f64(point.x);
        digest.write_f64(point.y);
    }
    fn write_point3(digest: &mut StableHasher, point: Point3) {
        digest.write_f64(point.x);
        digest.write_f64(point.y);
        digest.write_f64(point.z);
    }
    fn write_vec2(digest: &mut StableHasher, vector: kgeom::vec::Vec2) {
        digest.write_f64(vector.x);
        digest.write_f64(vector.y);
    }
    fn write_vec3(digest: &mut StableHasher, vector: kgeom::vec::Vec3) {
        digest.write_f64(vector.x);
        digest.write_f64(vector.y);
        digest.write_f64(vector.z);
    }
    fn write_frame(digest: &mut StableHasher, frame: &kgeom::frame::Frame) {
        write_point3(digest, frame.origin());
        write_vec3(digest, frame.x());
        write_vec3(digest, frame.y());
        write_vec3(digest, frame.z());
    }
    fn write_f64s(digest: &mut StableHasher, values: &[f64]) {
        digest.write_count(values.len());
        for &value in values {
            digest.write_f64(value);
        }
    }
    fn write_points2(digest: &mut StableHasher, values: &[kgeom::vec::Point2]) {
        digest.write_count(values.len());
        for &value in values {
            write_point2(digest, value);
        }
    }
    fn write_points3(digest: &mut StableHasher, values: &[Point3]) {
        digest.write_count(values.len());
        for &value in values {
            write_point3(digest, value);
        }
    }
    fn write_weights(digest: &mut StableHasher, weights: Option<&[f64]>) {
        if let Some(weights) = weights {
            digest.write_tag(1);
            write_f64s(digest, weights);
        } else {
            digest.write_tag(0);
        }
    }
    fn write_knots(digest: &mut StableHasher, knots: &kgeom::nurbs::KnotVector) {
        digest.write_count(knots.degree());
        write_f64s(digest, knots.as_slice());
    }
    fn write_curve(digest: &mut StableHasher, curve: &CurveGeom) {
        digest.write_bytes(curve.class_key().as_str().as_bytes());
        match curve {
            CurveGeom::Line(value) => {
                digest.write_tag(0);
                write_point3(digest, value.origin());
                write_vec3(digest, value.dir());
            }
            CurveGeom::Circle(value) => {
                digest.write_tag(1);
                write_frame(digest, value.frame());
                digest.write_f64(value.radius());
            }
            CurveGeom::Ellipse(value) => {
                digest.write_tag(2);
                write_frame(digest, value.frame());
                digest.write_f64(value.major_radius());
                digest.write_f64(value.minor_radius());
            }
            CurveGeom::Nurbs(value) => {
                digest.write_tag(3);
                write_knots(digest, value.knots());
                write_points3(digest, value.points());
                write_weights(digest, value.weights());
            }
            _ => digest.write_tag(u8::MAX),
        }
    }
    fn write_surface(
        digest: &mut StableHasher,
        surface: &SurfaceGeom,
        ordinals: &HashMap<SurfaceId, u64>,
    ) {
        digest.write_bytes(surface.class_key().as_str().as_bytes());
        match surface {
            SurfaceGeom::Plane(value) => {
                digest.write_tag(0);
                write_frame(digest, value.frame());
            }
            SurfaceGeom::Cylinder(value) => {
                digest.write_tag(1);
                write_frame(digest, value.frame());
                digest.write_f64(value.radius());
            }
            SurfaceGeom::Cone(value) => {
                digest.write_tag(2);
                write_frame(digest, value.frame());
                digest.write_f64(value.radius());
                digest.write_f64(value.half_angle());
            }
            SurfaceGeom::Sphere(value) => {
                digest.write_tag(3);
                write_frame(digest, value.frame());
                digest.write_f64(value.radius());
            }
            SurfaceGeom::Torus(value) => {
                digest.write_tag(4);
                write_frame(digest, value.frame());
                digest.write_f64(value.major_radius());
                digest.write_f64(value.minor_radius());
            }
            SurfaceGeom::Nurbs(value) => {
                digest.write_tag(5);
                write_knots(digest, value.knots(kgeom::surface::Dir::U));
                write_knots(digest, value.knots(kgeom::surface::Dir::V));
                let (u_count, v_count) = value.net_size();
                digest.write_count(u_count);
                digest.write_count(v_count);
                write_points3(digest, value.points());
                write_weights(digest, value.weights());
            }
            SurfaceGeom::Offset(value) => {
                digest.write_tag(6);
                write_ref(digest, value.basis(), ordinals);
                digest.write_f64(value.signed_distance());
            }
            _ => digest.write_tag(u8::MAX),
        }
    }
    fn write_pcurve(digest: &mut StableHasher, curve: &Curve2dGeom) {
        digest.write_bytes(curve.class_key().as_str().as_bytes());
        match curve {
            Curve2dGeom::Line(value) => {
                digest.write_tag(0);
                write_point2(digest, value.origin());
                write_vec2(digest, value.dir());
            }
            Curve2dGeom::Circle(value) => {
                digest.write_tag(1);
                write_point2(digest, value.center());
                write_vec2(digest, value.x_dir());
                digest.write_f64(value.radius());
            }
            Curve2dGeom::Nurbs(value) => {
                digest.write_tag(2);
                write_knots(digest, value.knots());
                write_points2(digest, value.points());
                write_weights(digest, value.weights());
            }
            _ => digest.write_tag(u8::MAX),
        }
    }

    let body_ordinals = ordinals::<Body>(store);
    let region_ordinals = ordinals::<Region>(store);
    let shell_ordinals = ordinals::<Shell>(store);
    let face_ordinals = ordinals::<Face>(store);
    let loop_ordinals = ordinals::<Loop>(store);
    let fin_ordinals = ordinals::<Fin>(store);
    let edge_ordinals = ordinals::<Edge>(store);
    let vertex_ordinals = ordinals::<Vertex>(store);
    let point_ordinals = ordinals::<Point3>(store);
    let curve_ordinals = ordinals::<CurveGeom>(store);
    let surface_ordinals = ordinals::<SurfaceGeom>(store);
    let pcurve_ordinals = ordinals::<Curve2dGeom>(store);
    let mut digest = StableHasher::new();
    digest.write_tag(0x52);
    digest.write_count(store.count::<Body>());
    digest.write_count(store.count::<Region>());
    digest.write_count(store.count::<Shell>());
    digest.write_count(store.count::<Face>());
    digest.write_count(store.count::<Loop>());
    digest.write_count(store.count::<Fin>());
    digest.write_count(store.count::<Edge>());
    digest.write_count(store.count::<Vertex>());
    digest.write_count(store.count::<Point3>());
    digest.write_count(store.count::<CurveGeom>());
    digest.write_count(store.count::<SurfaceGeom>());
    digest.write_count(store.count::<Curve2dGeom>());
    for (_, body) in store.iter::<Body>() {
        digest.write_tag(match body.kind {
            BodyKind::Solid => 0,
            BodyKind::Sheet => 1,
            BodyKind::Wire => 2,
            BodyKind::Acorn => 3,
        });
        write_refs(&mut digest, &body.regions, &region_ordinals);
    }
    for (_, region) in store.iter::<Region>() {
        write_ref(&mut digest, region.body, &body_ordinals);
        digest.write_tag(match region.kind {
            RegionKind::Solid => 0,
            RegionKind::Void => 1,
        });
        write_refs(&mut digest, &region.shells, &shell_ordinals);
    }
    for (_, shell) in store.iter::<Shell>() {
        write_ref(&mut digest, shell.region, &region_ordinals);
        write_refs(&mut digest, &shell.faces, &face_ordinals);
        write_refs(&mut digest, &shell.edges, &edge_ordinals);
        digest.write_ordinal(
            shell
                .vertex
                .and_then(|id| vertex_ordinals.get(&id).copied()),
        );
    }
    for (_, face) in store.iter::<Face>() {
        write_ref(&mut digest, face.shell, &shell_ordinals);
        write_refs(&mut digest, &face.loops, &loop_ordinals);
        write_ref(&mut digest, face.surface, &surface_ordinals);
        digest.write_tag(match face.sense {
            Sense::Forward => 0,
            Sense::Reversed => 1,
        });
        if let Some(domain) = face.domain {
            digest.write_tag(1);
            digest.write_f64(domain.u.lo);
            digest.write_f64(domain.u.hi);
            digest.write_f64(domain.v.lo);
            digest.write_f64(domain.v.hi);
        } else {
            digest.write_tag(0);
        }
        write_tolerance(&mut digest, face.tolerance);
    }
    for (_, loop_) in store.iter::<Loop>() {
        write_ref(&mut digest, loop_.face, &face_ordinals);
        write_refs(&mut digest, &loop_.fins, &fin_ordinals);
    }
    for (_, fin) in store.iter::<Fin>() {
        write_ref(&mut digest, fin.parent, &loop_ordinals);
        write_ref(&mut digest, fin.edge, &edge_ordinals);
        digest.write_tag(match fin.sense {
            Sense::Forward => 0,
            Sense::Reversed => 1,
        });
        if let Some(pcurve) = fin.pcurve {
            digest.write_tag(1);
            write_ref(&mut digest, pcurve.curve(), &pcurve_ordinals);
            digest.write_f64(pcurve.range().lo);
            digest.write_f64(pcurve.range().hi);
            digest.write_f64(pcurve.edge_to_pcurve().scale());
            digest.write_f64(pcurve.edge_to_pcurve().offset());
            for shift in pcurve.chart().period_shifts() {
                digest.write_i32(shift);
            }
            for kind in pcurve.endpoint_kinds() {
                digest.write_tag(match kind {
                    PcurveEndpointKind::Regular => 0,
                    PcurveEndpointKind::SurfaceSingularity => 1,
                });
            }
            if let Some(winding) = pcurve.closure_winding() {
                digest.write_tag(1);
                digest.write_i32(winding[0]);
                digest.write_i32(winding[1]);
            } else {
                digest.write_tag(0);
            }
            if let Some(seam) = pcurve.seam() {
                digest.write_tag(1);
                digest.write_tag(match seam.direction() {
                    SurfaceParameter::U => 0,
                    SurfaceParameter::V => 1,
                });
                digest.write_tag(match seam.side() {
                    SeamSide::Lower => 0,
                    SeamSide::Upper => 1,
                });
            } else {
                digest.write_tag(0);
            }
        } else {
            digest.write_tag(0);
        }
    }
    for (_, edge) in store.iter::<Edge>() {
        digest.write_ordinal(edge.curve.and_then(|id| curve_ordinals.get(&id).copied()));
        for vertex in edge.vertices {
            digest.write_ordinal(vertex.and_then(|id| vertex_ordinals.get(&id).copied()));
        }
        if let Some((lo, hi)) = edge.bounds {
            digest.write_tag(1);
            digest.write_f64(lo);
            digest.write_f64(hi);
        } else {
            digest.write_tag(0);
        }
        write_refs(&mut digest, &edge.fins, &fin_ordinals);
        write_tolerance(&mut digest, edge.tolerance);
    }
    for (_, vertex) in store.iter::<Vertex>() {
        write_ref(&mut digest, vertex.point, &point_ordinals);
        write_tolerance(&mut digest, vertex.tolerance);
    }
    for (_, point) in store.iter::<Point3>() {
        write_point3(&mut digest, *point);
    }
    for (_, curve) in store.iter::<CurveGeom>() {
        write_curve(&mut digest, curve);
    }
    for (_, surface) in store.iter::<SurfaceGeom>() {
        write_surface(&mut digest, surface, &surface_ordinals);
    }
    for (_, pcurve) in store.iter::<Curve2dGeom>() {
        write_pcurve(&mut digest, pcurve);
    }
    let index = index_snapshot(store);
    digest.write_u64(index.digest);
    StoreSnapshot {
        bodies: store.count::<Body>(),
        regions: store.count::<Region>(),
        shells: store.count::<Shell>(),
        faces: store.count::<Face>(),
        loops: store.count::<Loop>(),
        fins: store.count::<Fin>(),
        edges: store.count::<Edge>(),
        vertices: store.count::<Vertex>(),
        points: store.count::<Point3>(),
        curves: store.count::<CurveGeom>(),
        surfaces: store.count::<SurfaceGeom>(),
        pcurves: store.count::<Curve2dGeom>(),
        digest: digest.finish_stable(),
    }
}

pub(crate) fn affected_digest(store: &Store, affected: &[crate::entity::BodyId]) -> u64 {
    let ordinals: HashMap<_, _> = store
        .iter::<Body>()
        .enumerate()
        .map(|(ordinal, (body, _))| (body, ordinal as u64))
        .collect();
    let mut digest = StableHasher::new();
    digest.write_tag(0x53);
    digest.write_count(affected.len());
    for affected_body in affected {
        digest.write_ordinal(ordinals.get(affected_body).copied());
    }
    digest.finish_stable()
}

/// Fixed FNV-1a hasher used only for reproducible benchmark evidence.
pub(crate) struct StableHasher(u64);

impl StableHasher {
    pub(crate) const fn new() -> Self {
        Self(14_695_981_039_346_656_037)
    }

    pub(crate) fn write_tag(&mut self, tag: u8) {
        self.write(&[tag]);
    }

    pub(crate) fn write_u64(&mut self, value: u64) {
        self.write(&value.to_le_bytes());
    }

    pub(crate) fn write_i32(&mut self, value: i32) {
        self.write(&value.to_le_bytes());
    }

    pub(crate) fn write_count(&mut self, value: usize) {
        self.write_u64(value as u64);
    }

    pub(crate) fn write_ordinal(&mut self, ordinal: Option<u64>) {
        match ordinal {
            Some(value) => {
                self.write_tag(1);
                self.write_u64(value);
            }
            None => self.write_tag(0),
        }
    }

    pub(crate) fn write_f64(&mut self, value: f64) {
        self.write_u64(value.to_bits());
    }

    pub(crate) fn write_bytes(&mut self, bytes: &[u8]) {
        self.write_count(bytes.len());
        self.write(bytes);
    }

    pub(crate) const fn finish_stable(&self) -> u64 {
        self.0
    }
}

impl Hasher for StableHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(1_099_511_628_211);
        }
    }
}

// Keep the public seam opaque even if imports above evolve.
const _: Option<(CurveId, SurfaceId, Curve2dId, EntityRef)> = None;
