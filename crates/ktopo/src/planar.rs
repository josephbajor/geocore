//! Assembly of a precomputed, planar, connected solid boundary.
//!
//! This module is the topology half of semantic polyhedral builders. Callers
//! retain their own [`Transaction`], provide stable combinatorial vertex keys
//! and outward-oriented convex face loops, then decide how to Full-check and
//! commit the resulting body. The assembler shares vertices and edges by key,
//! creates complete analytic geometry and pcurves, and records optional face
//! lineage. It does not classify, heal, or persist a candidate on its own.

use crate::entity::{
    BodyId, Edge, EdgeId, EntityRef, Face, FaceDomain, FaceId, Fin, FinPcurve, Loop, ParamMap1d,
    Sense, ShellId, SurfaceId, Vertex, VertexId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::predicates::{Orientation, orient2d};
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3};
use kgraph::{
    AffineParamMap1d, PairedPlaneLineResidualCertificate, certify_paired_plane_line_residuals,
};
use std::collections::{BTreeMap, BTreeSet};

/// Stable combinatorial identity of one assembled vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PlanarVertexKey(u64);

impl PlanarVertexKey {
    /// Construct a key. Its numeric value has no geometric meaning.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Numeric value supplied by the semantic builder.
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Canonical unordered pair of distinct vertex keys.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PlanarEdgeKey {
    first: PlanarVertexKey,
    second: PlanarVertexKey,
}

impl PlanarEdgeKey {
    /// Canonicalize two distinct endpoint keys.
    pub fn new(a: PlanarVertexKey, b: PlanarVertexKey) -> Option<Self> {
        (a != b).then(|| {
            let (first, second) = if a < b { (a, b) } else { (b, a) };
            Self { first, second }
        })
    }

    /// Lower endpoint key.
    pub const fn first(self) -> PlanarVertexKey {
        self.first
    }

    /// Higher endpoint key.
    pub const fn second(self) -> PlanarVertexKey {
        self.second
    }
}

/// Representative model-space position for one combinatorial vertex.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarSolidVertex {
    key: PlanarVertexKey,
    position: Point3,
}

impl PlanarSolidVertex {
    /// Pair a stable key with its representative position.
    pub const fn new(key: PlanarVertexKey, position: Point3) -> Self {
        Self { key, position }
    }

    /// Stable combinatorial key.
    pub const fn key(self) -> PlanarVertexKey {
        self.key
    }

    /// Representative position.
    pub const fn position(self) -> Point3 {
        self.position
    }
}

/// One outward-oriented, strictly convex planar face loop.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarSolidFace {
    vertices: Vec<PlanarVertexKey>,
    source: Option<EntityRef>,
    plane_binding: Option<PlanarFacePlaneBinding>,
}

/// Semantic planes retained by one assembled planar face and all of its edges.
///
/// The supporting plane is reused as the resulting face surface. Each carrier
/// is paired with that support for the directed edge at the same index as the
/// face's vertex ring. Validation is all-or-nothing across a complete solid.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanarFacePlaneBinding {
    support: SurfaceId,
    edge_carriers: Vec<SurfaceId>,
}

impl PlanarFacePlaneBinding {
    /// Bind a face support and one cutting plane to every directed edge.
    pub fn new(support: SurfaceId, edge_carriers: Vec<SurfaceId>) -> Self {
        Self {
            support,
            edge_carriers,
        }
    }

    /// Existing plane surface supporting the complete face.
    pub const fn support(&self) -> SurfaceId {
        self.support
    }

    /// Existing cutting-plane identities in directed-edge order.
    pub fn edge_carriers(&self) -> &[SurfaceId] {
        &self.edge_carriers
    }
}

impl PlanarSolidFace {
    /// Construct a face without persistent-name lineage.
    pub fn new(vertices: Vec<PlanarVertexKey>) -> Self {
        Self {
            vertices,
            source: None,
            plane_binding: None,
        }
    }

    /// Attach the source face retained in the transaction journal.
    pub fn with_source(mut self, source: EntityRef) -> Self {
        self.source = Some(source);
        self
    }

    /// Vertex keys in outward-oriented loop order.
    pub fn vertices(&self) -> &[PlanarVertexKey] {
        &self.vertices
    }

    /// Optional source face reference.
    pub const fn source(&self) -> Option<EntityRef> {
        self.source
    }

    /// Retain complete semantic plane evidence for this face and its edges.
    pub fn with_plane_binding(mut self, binding: PlanarFacePlaneBinding) -> Self {
        self.plane_binding = Some(binding);
        self
    }

    /// Optional complete semantic plane binding.
    pub const fn plane_binding(&self) -> Option<&PlanarFacePlaneBinding> {
        self.plane_binding.as_ref()
    }
}

/// Complete combinatorial and geometric description of one connected shell.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarSolidInput {
    vertices: Vec<PlanarSolidVertex>,
    faces: Vec<PlanarSolidFace>,
}

impl PlanarSolidInput {
    /// Construct an input. Validation occurs before assembly.
    pub fn new(vertices: Vec<PlanarSolidVertex>, faces: Vec<PlanarSolidFace>) -> Self {
        Self { vertices, faces }
    }

    /// Keyed representative vertices.
    pub fn vertices(&self) -> &[PlanarSolidVertex] {
        &self.vertices
    }

    /// Outward-oriented face loops, retained in this order in the shell.
    pub fn faces(&self) -> &[PlanarSolidFace] {
        &self.faces
    }
}

/// Handles and stable-key mappings produced by planar-solid assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanarSolidOutput {
    body: BodyId,
    shell: ShellId,
    vertices: Vec<(PlanarVertexKey, VertexId)>,
    edges: Vec<(PlanarEdgeKey, EdgeId)>,
    faces: Vec<FaceId>,
}

impl PlanarSolidOutput {
    /// Assembled solid body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Body's single connected boundary shell.
    pub const fn shell(&self) -> ShellId {
        self.shell
    }

    /// Vertex mappings in ascending key order.
    pub fn vertices(&self) -> &[(PlanarVertexKey, VertexId)] {
        &self.vertices
    }

    /// Edge mappings in ascending canonical endpoint-key order.
    pub fn edges(&self) -> &[(PlanarEdgeKey, EdgeId)] {
        &self.edges
    }

    /// Face handles in input order.
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    /// Look up an assembled vertex by its stable key.
    pub fn vertex(&self, key: PlanarVertexKey) -> Option<VertexId> {
        self.vertices
            .binary_search_by_key(&key, |(candidate, _)| *candidate)
            .ok()
            .map(|index| self.vertices[index].1)
    }

    /// Look up an assembled edge by its canonical endpoint keys.
    pub fn edge(&self, key: PlanarEdgeKey) -> Option<EdgeId> {
        self.edges
            .binary_search_by_key(&key, |(candidate, _)| *candidate)
            .ok()
            .map(|index| self.edges[index].1)
    }
}

#[derive(Debug, Clone, Copy)]
struct EdgeUse {
    face: usize,
    from: PlanarVertexKey,
    to: PlanarVertexKey,
    planes: Option<[SurfaceId; 2]>,
}

#[derive(Debug, Clone)]
struct PreparedEdge {
    line: Line,
    length: f64,
    binding: Option<PreparedEdgeBinding>,
}

#[derive(Debug, Clone)]
struct PreparedEdgeBinding {
    surfaces: [SurfaceId; 2],
    pcurves: [Line2d; 2],
    certificate: PairedPlaneLineResidualCertificate,
}

#[derive(Debug, Clone, Copy)]
struct PreparedFin {
    edge: PlanarEdgeKey,
    sense: Sense,
    pcurve: PreparedFinPcurve,
    length: f64,
}

#[derive(Debug, Clone, Copy)]
enum PreparedFinPcurve {
    New(Line2d),
    Bound(usize),
}

#[derive(Debug)]
struct PreparedFace {
    surface: PreparedFaceSurface,
    sense: Sense,
    domain: FaceDomain,
    fins: Vec<PreparedFin>,
    source: Option<EntityRef>,
}

#[derive(Debug, Clone, Copy)]
enum PreparedFaceSurface {
    New(Plane),
    Reuse(SurfaceId),
}

#[derive(Debug)]
struct PreparedSolid {
    vertices: BTreeMap<PlanarVertexKey, Point3>,
    edges: BTreeMap<PlanarEdgeKey, PreparedEdge>,
    faces: Vec<PreparedFace>,
}

#[derive(Debug, Clone, Copy)]
struct AllocatedEdge {
    edge: EdgeId,
    bound_pcurves: Option<[crate::entity::Curve2dId; 2]>,
}

impl Transaction<'_> {
    /// Assemble one prevalidated connected planar solid in this transaction.
    ///
    /// All caller-controlled combinatorics and geometry are checked before
    /// entity allocation. Every edge must have exactly two face uses in
    /// opposite directions, every vertex link must be a single cycle, every
    /// face loop must be planar and strictly convex, the face graph must be
    /// connected, and outward orientation must certify positive signed volume.
    /// The caller owns the eventual checked or Full commit.
    pub fn assemble_planar_solid(&mut self, input: &PlanarSolidInput) -> Result<PlanarSolidOutput> {
        let prepared = PreparedSolid::new(input, self.store())?;
        let (body, shell) = crate::make::solid_body_scaffold(self.store_mut());

        let mut vertex_handles = BTreeMap::new();
        let mut edge_handles = BTreeMap::new();
        let mut face_handles = Vec::with_capacity(prepared.faces.len());
        let mut lineage = Vec::new();

        {
            let store = self.store_mut();
            for (&key, &position) in &prepared.vertices {
                let point = store.add(position);
                let vertex = store.add(Vertex {
                    point,
                    tolerance: None,
                });
                vertex_handles.insert(key, vertex);
            }

            for (&key, edge) in &prepared.edges {
                let (curve, bound_pcurves) = if let Some(binding) = &edge.binding {
                    let pcurves = [
                        store.insert_pcurve(Curve2dGeom::Line(binding.pcurves[0]))?,
                        store.insert_pcurve(Curve2dGeom::Line(binding.pcurves[1]))?,
                    ];
                    let curve = store.insert_verified_plane_intersection_curve(
                        binding.surfaces,
                        pcurves,
                        binding.certificate,
                    )?;
                    (curve, Some(pcurves))
                } else {
                    (store.insert_curve(CurveGeom::Line(edge.line))?, None)
                };
                let value = store.add(Edge {
                    curve: Some(curve),
                    vertices: [
                        Some(vertex_handles[&key.first]),
                        Some(vertex_handles[&key.second]),
                    ],
                    bounds: Some((0.0, edge.length)),
                    fins: Vec::new(),
                    tolerance: None,
                });
                edge_handles.insert(
                    key,
                    AllocatedEdge {
                        edge: value,
                        bound_pcurves,
                    },
                );
            }

            for face in prepared.faces {
                let surface = match face.surface {
                    PreparedFaceSurface::New(plane) => {
                        store.insert_surface(SurfaceGeom::Plane(plane))?
                    }
                    PreparedFaceSurface::Reuse(surface) => surface,
                };
                let face_handle = store.add(Face {
                    shell,
                    loops: Vec::new(),
                    surface,
                    sense: face.sense,
                    domain: Some(face.domain),
                    tolerance: None,
                });
                store.get_mut(shell)?.faces.push(face_handle);

                let loop_handle = store.add(Loop {
                    face: face_handle,
                    fins: Vec::new(),
                });
                store.get_mut(face_handle)?.loops.push(loop_handle);

                let mut fins = Vec::with_capacity(face.fins.len());
                for prepared_fin in face.fins {
                    let allocated = edge_handles[&prepared_fin.edge];
                    let curve = match prepared_fin.pcurve {
                        PreparedFinPcurve::New(pcurve) => {
                            store.insert_pcurve(Curve2dGeom::Line(pcurve))?
                        }
                        PreparedFinPcurve::Bound(index) => {
                            allocated.bound_pcurves.ok_or(Error::InvalidGeometry {
                                reason: "bound planar fin lost its paired pcurve evidence",
                            })?[index]
                        }
                    };
                    let pcurve = FinPcurve::new(
                        curve,
                        ParamRange::new(0.0, prepared_fin.length),
                        ParamMap1d::identity(),
                    )?;
                    let fin = store.add(Fin {
                        parent: loop_handle,
                        edge: allocated.edge,
                        sense: prepared_fin.sense,
                        pcurve: Some(pcurve),
                    });
                    store.get_mut(allocated.edge)?.fins.push(fin);
                    fins.push(fin);
                }
                store.get_mut(loop_handle)?.fins = fins;
                face_handles.push(face_handle);
                if let Some(source) = face.source {
                    lineage.push((EntityRef::Face(face_handle), source));
                }
            }
        }

        for (derived, source) in lineage {
            self.record_derived_from(derived, source);
        }

        Ok(PlanarSolidOutput {
            body,
            shell,
            vertices: vertex_handles.into_iter().collect(),
            edges: edge_handles
                .into_iter()
                .map(|(key, allocated)| (key, allocated.edge))
                .collect(),
            faces: face_handles,
        })
    }
}

impl PreparedSolid {
    fn new(input: &PlanarSolidInput, store: &crate::store::Store) -> Result<Self> {
        if input.vertices.len() < 4 || input.faces.len() < 4 {
            return invalid("a planar solid requires at least four vertices and four faces");
        }
        let bound_faces = input
            .faces
            .iter()
            .filter(|face| face.plane_binding.is_some())
            .count();
        if bound_faces != 0 && bound_faces != input.faces.len() {
            return invalid("planar-solid semantic plane bindings must cover every face");
        }

        let mut vertices = BTreeMap::new();
        for vertex in &input.vertices {
            check_in_size_box(vertex.position.to_array())?;
            if vertices.insert(vertex.key, vertex.position).is_some() {
                return invalid("planar-solid vertex keys must be unique");
            }
        }

        let mut uses: BTreeMap<PlanarEdgeKey, Vec<EdgeUse>> = BTreeMap::new();
        let mut referenced = BTreeSet::new();
        let mut face_keys = Vec::with_capacity(input.faces.len());
        let mut face_geometry = Vec::with_capacity(input.faces.len());
        let mut face_domains = Vec::with_capacity(input.faces.len());

        for (face_index, face) in input.faces.iter().enumerate() {
            validate_source(store, face.source)?;
            let (frame, surface, sense, domain) = prepare_face(face, &vertices, store)?;
            face_geometry.push((frame, surface, sense));
            face_domains.push(domain);
            face_keys.push(face.vertices.clone());
            for index in 0..face.vertices.len() {
                let from = face.vertices[index];
                let to = face.vertices[(index + 1) % face.vertices.len()];
                let edge = PlanarEdgeKey::new(from, to).ok_or(Error::InvalidGeometry {
                    reason: "a face boundary edge must have distinct endpoint keys",
                })?;
                uses.entry(edge).or_default().push(EdgeUse {
                    face: face_index,
                    from,
                    to,
                    planes: face
                        .plane_binding
                        .as_ref()
                        .map(|binding| [binding.support, binding.edge_carriers[index]]),
                });
                referenced.insert(from);
                referenced.insert(to);
            }
        }

        if referenced.len() != vertices.len() {
            return invalid("every planar-solid vertex must be referenced by a face");
        }
        validate_edge_uses(&uses)?;
        validate_face_connectivity(input.faces.len(), &uses)?;
        validate_vertex_links(&face_keys)?;
        validate_positive_volume(&vertices, &face_keys)?;

        let mut edges = BTreeMap::new();
        for (&key, edge_uses) in &uses {
            let start = vertices[&key.first];
            let end = vertices[&key.second];
            let direction = end - start;
            let length = direction.norm();
            let line = Line::new(start, direction)?;
            if !length.is_finite() || length <= LINEAR_RESOLUTION {
                return invalid("planar-solid edges must exceed linear resolution");
            }
            let binding = prepare_edge_binding(store, line, start, end, length, edge_uses)?;
            edges.insert(
                key,
                PreparedEdge {
                    line,
                    length,
                    binding,
                },
            );
        }

        let mut faces = Vec::with_capacity(input.faces.len());
        for (face_index, face) in input.faces.iter().enumerate() {
            let (frame, surface, sense) = face_geometry[face_index];
            let mut fins = Vec::with_capacity(face.vertices.len());
            for index in 0..face.vertices.len() {
                let from = face.vertices[index];
                let to = face.vertices[(index + 1) % face.vertices.len()];
                let edge = PlanarEdgeKey::new(from, to).expect("face preflight rejected self-edge");
                let prepared_edge = &edges[&edge];
                let pcurve = if let Some(binding) = &prepared_edge.binding {
                    let support = face
                        .plane_binding
                        .as_ref()
                        .expect("all-or-nothing binding was preflighted")
                        .support;
                    let index = binding
                        .surfaces
                        .iter()
                        .position(|candidate| *candidate == support)
                        .ok_or(Error::InvalidGeometry {
                            reason: "bound face support is absent from its edge plane pair",
                        })?;
                    PreparedFinPcurve::Bound(index)
                } else {
                    let start = frame_uv(&frame, vertices[&edge.first]);
                    let end = frame_uv(&frame, vertices[&edge.second]);
                    PreparedFinPcurve::New(Line2d::new(start, end - start)?)
                };
                fins.push(PreparedFin {
                    edge,
                    sense: if from == edge.first {
                        Sense::Forward
                    } else {
                        Sense::Reversed
                    },
                    pcurve,
                    length: prepared_edge.length,
                });
            }
            faces.push(PreparedFace {
                surface,
                sense,
                domain: face_domains[face_index],
                fins,
                source: face.source,
            });
        }

        Ok(Self {
            vertices,
            edges,
            faces,
        })
    }
}

fn validate_source(store: &crate::store::Store, source: Option<EntityRef>) -> Result<()> {
    let Some(source) = source else {
        return Ok(());
    };
    match source {
        EntityRef::Face(face) if store.contains(face) => Ok(()),
        EntityRef::Face(_) => Err(Error::StaleHandle),
        _ => invalid("planar-solid lineage sources must reference faces"),
    }
}

fn prepare_face(
    face: &PlanarSolidFace,
    vertices: &BTreeMap<PlanarVertexKey, Point3>,
    store: &crate::store::Store,
) -> Result<(Frame, PreparedFaceSurface, Sense, FaceDomain)> {
    if face.vertices.len() < 3 {
        return invalid("a planar-solid face requires at least three vertices");
    }
    let mut unique = BTreeSet::new();
    let mut points = Vec::with_capacity(face.vertices.len());
    for &key in &face.vertices {
        if !unique.insert(key) {
            return invalid("a planar-solid face must not repeat a vertex key");
        }
        points.push(*vertices.get(&key).ok_or(Error::InvalidGeometry {
            reason: "a planar-solid face references an unknown vertex key",
        })?);
    }

    let (frame, surface, sense, projected) = if let Some(binding) = &face.plane_binding {
        if binding.edge_carriers.len() != face.vertices.len() {
            return invalid("a planar face binding requires one carrier per directed edge");
        }
        let plane = live_plane(store, binding.support)?;
        let mut carriers = Vec::with_capacity(binding.edge_carriers.len());
        for &carrier in &binding.edge_carriers {
            if carrier == binding.support {
                return invalid("a planar edge support and carrier plane must be distinct");
            }
            live_plane(store, carrier)?;
            if carriers.contains(&carrier) {
                return invalid("a planar face must not repeat an edge carrier plane");
            }
            carriers.push(carrier);
        }
        let frame = *plane.frame();
        let projected: Vec<_> = points
            .iter()
            .copied()
            .map(|point| {
                let point = frame_uv(&frame, point);
                [point.x, point.y]
            })
            .collect();
        let sense = match orient2d(projected[0], projected[1], projected[2]) {
            Orientation::Positive => Sense::Forward,
            Orientation::Negative => Sense::Reversed,
            Orientation::Zero => {
                return invalid("a bound planar face must have nonzero orientation on its support");
            }
        };
        (
            frame,
            PreparedFaceSurface::Reuse(binding.support),
            sense,
            projected,
        )
    } else {
        let mut frame = None;
        for index in 0..points.len() {
            let a = points[index];
            let b = points[(index + 1) % points.len()];
            let c = points[(index + 2) % points.len()];
            if let Ok(candidate) = Frame::new(a, (b - a).cross(c - a), b - a) {
                frame = Some(candidate);
                break;
            }
        }
        let frame = frame.ok_or(Error::InvalidGeometry {
            reason: "a planar-solid face must contain a stable non-collinear corner",
        })?;
        let normal = frame.z();
        let dominant_axis = if normal.x.abs() >= normal.y.abs() && normal.x.abs() >= normal.z.abs()
        {
            0
        } else if normal.y.abs() >= normal.z.abs() {
            1
        } else {
            2
        };
        let projected = points
            .iter()
            .copied()
            .map(|point| dominant_projection(point, dominant_axis))
            .collect();
        (
            frame,
            PreparedFaceSurface::New(Plane::new(frame)),
            Sense::Forward,
            projected,
        )
    };
    let expected_turn = orient2d(projected[0], projected[1], projected[2]);
    if expected_turn == Orientation::Zero {
        return invalid("a planar-solid face loop must have nonzero convex turns");
    }
    for index in 1..projected.len() {
        let a = projected[index];
        let b = projected[(index + 1) % projected.len()];
        let c = projected[(index + 2) % projected.len()];
        if orient2d(a, b, c) != expected_turn {
            return invalid("a planar-solid face loop must be strictly convex and oriented");
        }
    }

    let mut uv = Vec::with_capacity(points.len());
    for point in points {
        let residual = (point - frame.origin()).dot(frame.z());
        if !residual.is_finite() || residual.abs() > LINEAR_RESOLUTION {
            return invalid("a planar-solid face exceeds the planar tolerance");
        }
        uv.push(frame_uv(&frame, point));
    }
    Ok((frame, surface, sense, point_domain(uv)?))
}

fn live_plane(store: &crate::store::Store, surface: SurfaceId) -> Result<Plane> {
    match store.get(surface)? {
        SurfaceGeom::Plane(plane) => Ok(*plane),
        _ => invalid("planar semantic evidence must reference Plane surfaces"),
    }
}

fn prepare_edge_binding(
    store: &crate::store::Store,
    line: Line,
    start: Point3,
    end: Point3,
    length: f64,
    edge_uses: &[EdgeUse],
) -> Result<Option<PreparedEdgeBinding>> {
    let [first, second] = edge_uses else {
        return invalid("every planar-solid edge must have exactly two face uses");
    };
    let (Some(first), Some(second)) = (first.planes, second.planes) else {
        if first.planes.is_none() && second.planes.is_none() {
            return Ok(None);
        }
        return invalid("both uses of a planar edge must carry semantic plane evidence");
    };
    if !same_unordered_plane_pair(first, second) {
        return invalid("both uses of a planar edge must claim the same plane pair");
    }

    let surfaces = first;
    let planes = [
        live_plane(store, surfaces[0])?,
        live_plane(store, surfaces[1])?,
    ];
    let [first_pcurve, second_pcurve] = planes.map(|plane| {
        let start = frame_uv(plane.frame(), start);
        let end = frame_uv(plane.frame(), end);
        Line2d::new(start, end - start)
    });
    let pcurves = [first_pcurve?, second_pcurve?];
    let parameter_maps = [
        AffineParamMap1d::new(1.0, 0.0).expect("identity graph parameter map is valid"),
        AffineParamMap1d::new(1.0, 0.0).expect("identity graph parameter map is valid"),
    ];
    let certificate = certify_paired_plane_line_residuals(
        line,
        ParamRange::new(0.0, length),
        planes,
        pcurves,
        parameter_maps,
        LINEAR_RESOLUTION,
    )
    .map_err(|_| Error::InvalidGeometry {
        reason: "bound planar edge lacks a whole-range paired plane certificate",
    })?;
    Ok(Some(PreparedEdgeBinding {
        surfaces,
        pcurves,
        certificate,
    }))
}

fn same_unordered_plane_pair(first: [SurfaceId; 2], second: [SurfaceId; 2]) -> bool {
    first == second || first == [second[1], second[0]]
}

fn dominant_projection(point: Point3, dropped_axis: usize) -> [f64; 2] {
    match dropped_axis {
        0 => [point.y, point.z],
        1 => [point.x, point.z],
        _ => [point.x, point.y],
    }
}

fn validate_edge_uses(uses: &BTreeMap<PlanarEdgeKey, Vec<EdgeUse>>) -> Result<()> {
    for edge_uses in uses.values() {
        if edge_uses.len() != 2 {
            return invalid("every planar-solid edge must have exactly two face uses");
        }
        let [first, second] = [edge_uses[0], edge_uses[1]];
        if first.from != second.to || first.to != second.from || first.face == second.face {
            return invalid("the two uses of a planar-solid edge must be opposed");
        }
    }
    Ok(())
}

fn validate_face_connectivity(
    face_count: usize,
    uses: &BTreeMap<PlanarEdgeKey, Vec<EdgeUse>>,
) -> Result<()> {
    let mut neighbors = vec![Vec::new(); face_count];
    for edge_uses in uses.values() {
        let [first, second] = [edge_uses[0].face, edge_uses[1].face];
        neighbors[first].push(second);
        neighbors[second].push(first);
    }
    let mut seen = vec![false; face_count];
    let mut pending = vec![0];
    while let Some(face) = pending.pop() {
        if core::mem::replace(&mut seen[face], true) {
            continue;
        }
        pending.extend(neighbors[face].iter().copied());
    }
    if seen.into_iter().all(|value| value) {
        Ok(())
    } else {
        invalid("a planar-solid shell must have one connected face component")
    }
}

fn validate_vertex_links(faces: &[Vec<PlanarVertexKey>]) -> Result<()> {
    let mut corners: BTreeMap<PlanarVertexKey, Vec<(PlanarVertexKey, PlanarVertexKey)>> =
        BTreeMap::new();
    for face in faces {
        for index in 0..face.len() {
            let vertex = face[index];
            let previous = face[(index + face.len() - 1) % face.len()];
            let next = face[(index + 1) % face.len()];
            corners.entry(vertex).or_default().push((previous, next));
        }
    }

    for incident_corners in corners.values() {
        let mut link: BTreeMap<PlanarVertexKey, BTreeSet<PlanarVertexKey>> = BTreeMap::new();
        for &(first, second) in incident_corners {
            if first == second
                || !link.entry(first).or_default().insert(second)
                || !link.entry(second).or_default().insert(first)
            {
                return invalid("a planar-solid vertex link must be a simple cycle");
            }
        }
        if link.len() < 3 || link.values().any(|neighbors| neighbors.len() != 2) {
            return invalid("a planar-solid vertex link must be a simple cycle");
        }
        let start = *link.keys().next().expect("nonempty link");
        let mut seen = BTreeSet::new();
        let mut pending = vec![start];
        while let Some(vertex) = pending.pop() {
            if seen.insert(vertex) {
                pending.extend(link[&vertex].iter().copied());
            }
        }
        if seen.len() != link.len() {
            return invalid("a planar-solid vertex link must be connected");
        }
    }
    Ok(())
}

fn validate_positive_volume(
    vertices: &BTreeMap<PlanarVertexKey, Point3>,
    faces: &[Vec<PlanarVertexKey>],
) -> Result<()> {
    let reference = *vertices.values().next().expect("minimum input was checked");
    let mut six_volume = Interval::point(0.0);
    for face in faces {
        let a = vertices[&face[0]] - reference;
        for index in 1..face.len() - 1 {
            let b = vertices[&face[index]] - reference;
            let c = vertices[&face[index + 1]] - reference;
            six_volume = six_volume + determinant(a, b, c);
        }
    }
    if six_volume.lo() > 0.0 {
        Ok(())
    } else {
        invalid("outward planar-solid faces must certify positive enclosed volume")
    }
}

fn determinant(a: Point3, b: Point3, c: Point3) -> Interval {
    let ix = Interval::point;
    ix(a.x) * (ix(b.y) * ix(c.z) - ix(b.z) * ix(c.y))
        - ix(a.y) * (ix(b.x) * ix(c.z) - ix(b.z) * ix(c.x))
        + ix(a.z) * (ix(b.x) * ix(c.y) - ix(b.y) * ix(c.x))
}

fn frame_uv(frame: &Frame, point: Point3) -> Point2 {
    let relative = point - frame.origin();
    Point2::new(relative.dot(frame.x()), relative.dot(frame.y()))
}

fn point_domain(points: Vec<Point2>) -> Result<FaceDomain> {
    let first = points[0];
    let (mut u_min, mut u_max, mut v_min, mut v_max) = (first.x, first.x, first.y, first.y);
    for point in points.into_iter().skip(1) {
        u_min = u_min.min(point.x);
        u_max = u_max.max(point.x);
        v_min = v_min.min(point.y);
        v_max = v_max.max(point.y);
    }
    FaceDomain::from_bounds(u_min, u_max, v_min, v_max)
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{Body, Edge, Fin, Loop, Region, Shell};
    use crate::incidence::{
        IncidenceCertification, certify_edge_surface_incidence, certify_pcurve_incidence,
    };
    use crate::store::Store;
    use crate::transaction::{FullCommitRequirement, Journal, LineageEvent};
    use kgeom::surface::Sphere;
    use kgeom::vec::Vec3;

    const KEYS: [PlanarVertexKey; 8] = [
        PlanarVertexKey::new(101),
        PlanarVertexKey::new(307),
        PlanarVertexKey::new(211),
        PlanarVertexKey::new(503),
        PlanarVertexKey::new(109),
        PlanarVertexKey::new(401),
        PlanarVertexKey::new(223),
        PlanarVertexKey::new(509),
    ];

    fn keyed_box(sources: Option<&[FaceId]>) -> PlanarSolidInput {
        let points = [
            Point3::new(-1.0, -1.5, -2.0),
            Point3::new(1.0, -1.5, -2.0),
            Point3::new(-1.0, 1.5, -2.0),
            Point3::new(1.0, 1.5, -2.0),
            Point3::new(-1.0, -1.5, 2.0),
            Point3::new(1.0, -1.5, 2.0),
            Point3::new(-1.0, 1.5, 2.0),
            Point3::new(1.0, 1.5, 2.0),
        ];
        let vertices = [3, 0, 6, 2, 7, 1, 5, 4]
            .into_iter()
            .map(|index| PlanarSolidVertex::new(KEYS[index], points[index]))
            .collect();
        let rings = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        let faces = rings
            .into_iter()
            .enumerate()
            .map(|(index, ring)| {
                let face = PlanarSolidFace::new(ring.map(|vertex| KEYS[vertex]).to_vec());
                sources.map_or(face.clone(), |sources| {
                    face.with_source(EntityRef::Face(sources[index]))
                })
            })
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn box_rings() -> [[usize; 4]; 6] {
        [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
    }

    fn source_surfaces(store: &mut Store) -> Vec<SurfaceId> {
        let source = crate::make::block(store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        store
            .faces_of_body(source)
            .unwrap()
            .into_iter()
            .map(|face| store.get(face).unwrap().surface())
            .collect()
    }

    fn bind_keyed_box(input: &mut PlanarSolidInput, surfaces: &[SurfaceId]) {
        let rings = box_rings();
        assert_eq!(surfaces.len(), rings.len());
        for (face_index, ring) in rings.iter().enumerate() {
            let mut carriers = Vec::with_capacity(ring.len());
            for edge_index in 0..ring.len() {
                let first = ring[edge_index];
                let second = ring[(edge_index + 1) % ring.len()];
                let other = rings
                    .iter()
                    .enumerate()
                    .find_map(|(candidate_index, candidate)| {
                        (candidate_index != face_index
                            && (0..candidate.len()).any(|index| {
                                let a = candidate[index];
                                let b = candidate[(index + 1) % candidate.len()];
                                a == first && b == second || a == second && b == first
                            }))
                        .then_some(candidate_index)
                    })
                    .expect("every box edge has a second face use");
                carriers.push(surfaces[other]);
            }
            input.faces[face_index].plane_binding =
                Some(PlanarFacePlaneBinding::new(surfaces[face_index], carriers));
        }
    }

    fn bound_keyed_box(store: &mut Store) -> (PlanarSolidInput, Vec<SurfaceId>) {
        let surfaces = source_surfaces(store);
        let mut input = keyed_box(None);
        bind_keyed_box(&mut input, &surfaces);
        (input, surfaces)
    }

    fn rotated_off_origin_box() -> PlanarSolidInput {
        let mut input = keyed_box(None);
        let frame = Frame::new(
            Point3::new(17.0, -23.0, 31.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        for vertex in &mut input.vertices {
            let local = vertex.position;
            vertex.position = frame.point_at(local.x, local.y, local.z);
        }
        input
    }

    fn assemble_full(input: &PlanarSolidInput) -> (PlanarSolidOutput, Journal) {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(input).unwrap();
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "{decision:#?}");
        assert!(
            decision
                .checks()
                .iter()
                .all(|check| check.report().faults.is_empty() && check.report().gaps.is_empty())
        );
        (output, decision.journal().unwrap().clone())
    }

    #[test]
    fn assembles_one_shared_checker_clean_box_shell() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&keyed_box(None)).unwrap();

        assert_eq!(output.vertices().len(), 8);
        assert_eq!(output.edges().len(), 12);
        assert_eq!(output.faces().len(), 6);
        assert_eq!(transaction.store().count::<Body>(), 1);
        assert_eq!(transaction.store().count::<Region>(), 2);
        assert_eq!(transaction.store().count::<Shell>(), 1);
        assert_eq!(transaction.store().count::<Face>(), 6);
        assert_eq!(transaction.store().count::<Loop>(), 6);
        assert_eq!(transaction.store().count::<Fin>(), 24);
        assert_eq!(transaction.store().count::<Edge>(), 12);
        assert_eq!(transaction.store().count::<Vertex>(), 8);
        for &(_, edge) in output.edges() {
            let edge = transaction.store().get(edge).unwrap();
            assert_eq!(edge.fins().len(), 2);
            let first = transaction.store().get(edge.fins()[0]).unwrap();
            let second = transaction.store().get(edge.fins()[1]).unwrap();
            assert_eq!(first.sense(), second.sense().flipped());
        }

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
        assert!(
            decision
                .checks()
                .iter()
                .all(|check| check.report().faults.is_empty() && check.report().gaps.is_empty())
        );
    }

    #[test]
    fn construction_is_deterministic_and_rollback_restores_future_ids() {
        let input = keyed_box(None);
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let rolled_back = transaction.assemble_planar_solid(&input).unwrap();
        transaction.rollback().unwrap();
        assert_eq!(store.count::<Body>(), 0);
        assert_eq!(store.count::<Vertex>(), 0);

        let mut transaction = store.transaction().unwrap();
        let after_rollback = transaction.assemble_planar_solid(&input).unwrap();
        assert_eq!(after_rollback, rolled_back);
        let journal = transaction
            .commit_checked(&[after_rollback.body()])
            .unwrap();

        let mut fresh = Store::new();
        let mut transaction = fresh.transaction().unwrap();
        let fresh_output = transaction.assemble_planar_solid(&input).unwrap();
        let fresh_journal = transaction.commit_checked(&[fresh_output.body()]).unwrap();
        assert_eq!(fresh_output, after_rollback);
        assert_eq!(fresh_journal, journal);
    }

    #[test]
    fn rotated_off_origin_box_is_full_valid_and_deterministic() {
        let input = rotated_off_origin_box();
        let first = assemble_full(&input);
        let second = assemble_full(&input);
        assert_eq!(second, first);
    }

    #[test]
    fn records_source_face_lineage_in_input_order() {
        let mut store = Store::new();
        let source = crate::make::block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let source_faces = store.faces_of_body(source).unwrap();
        let input = keyed_box(Some(&source_faces));

        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        let journal = transaction.commit_checked(&[output.body()]).unwrap();
        let expected: Vec<_> = output
            .faces()
            .iter()
            .copied()
            .zip(source_faces)
            .map(|(derived, source)| LineageEvent::DerivedFrom {
                derived: EntityRef::Face(derived),
                source: EntityRef::Face(source),
            })
            .collect();
        assert_eq!(journal.lineage(), expected);
    }

    #[test]
    fn bound_faces_reuse_source_surfaces_and_edges_retain_verified_plane_pairs() {
        let mut store = Store::new();
        let (input, surfaces) = bound_keyed_box(&mut store);
        let surface_count = store.count::<SurfaceGeom>();
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();

        assert_eq!(transaction.store().count::<SurfaceGeom>(), surface_count);
        for (index, &face) in output.faces().iter().enumerate() {
            assert_eq!(
                transaction.store().get(face).unwrap().surface(),
                surfaces[index]
            );
        }
        for &(key, edge) in output.edges() {
            let expected = input
                .faces()
                .iter()
                .find_map(|face| {
                    let binding = face.plane_binding()?;
                    (0..face.vertices().len()).find_map(|index| {
                        let candidate = PlanarEdgeKey::new(
                            face.vertices()[index],
                            face.vertices()[(index + 1) % face.vertices().len()],
                        )?;
                        (candidate == key)
                            .then_some([binding.support(), binding.edge_carriers()[index]])
                    })
                })
                .unwrap();
            let edge_value = transaction.store().get(edge).unwrap();
            let descriptor = transaction
                .store()
                .get(edge_value.curve().unwrap())
                .unwrap()
                .as_intersection()
                .expect("bound planar edges use verified intersection descriptors");
            assert!(same_unordered_plane_pair(
                descriptor.source_surfaces(),
                expected
            ));
            assert!(descriptor.certificate().as_plane_line().is_some());
            let paired = descriptor.pcurves();
            assert!(edge_value.fins().iter().all(|fin| {
                let pcurve = transaction
                    .store()
                    .get(*fin)
                    .unwrap()
                    .pcurve()
                    .unwrap()
                    .curve();
                paired.contains(&pcurve)
            }));
        }

        for &face in output.faces() {
            let face = transaction.store().get(face).unwrap();
            for &loop_id in face.loops() {
                for &fin_id in transaction.store().get(loop_id).unwrap().fins() {
                    let fin = transaction.store().get(fin_id).unwrap();
                    assert_eq!(
                        certify_edge_surface_incidence(
                            transaction.store(),
                            fin.edge(),
                            face.surface(),
                            LINEAR_RESOLUTION,
                        )
                        .unwrap(),
                        IncidenceCertification::Certified,
                    );
                    assert_eq!(
                        certify_pcurve_incidence(
                            transaction.store(),
                            fin.edge(),
                            face.surface(),
                            fin.pcurve().unwrap(),
                            LINEAR_RESOLUTION,
                        )
                        .unwrap(),
                        IncidenceCertification::Certified,
                    );
                }
            }
        }

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "{decision:#?}");
        assert!(
            decision.checks().iter().all(|check| {
                check.report().faults.is_empty() && check.report().gaps.is_empty()
            })
        );
    }

    #[test]
    fn bound_face_sense_tracks_reversed_support_frame_orientation() {
        let mut store = Store::new();
        let mut surfaces = source_surfaces(&mut store);
        let original = *store.get(surfaces[0]).unwrap().as_plane().unwrap();
        let frame = original.frame();
        let reversed = Plane::new(
            Frame::new(frame.origin(), -frame.z(), frame.x()).expect("reversed plane frame"),
        );
        surfaces[0] = store.insert_surface(SurfaceGeom::Plane(reversed)).unwrap();
        let mut input = keyed_box(None);
        bind_keyed_box(&mut input, &surfaces);

        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        assert_eq!(
            transaction.store().get(output.faces()[0]).unwrap().sense(),
            Sense::Reversed
        );
        for &face in &output.faces()[1..] {
            assert_eq!(
                transaction.store().get(face).unwrap().sense(),
                Sense::Forward
            );
        }
        transaction.rollback().unwrap();
    }

    #[test]
    fn malformed_plane_bindings_refuse_before_topology_allocation() {
        let mut store = Store::new();
        let (valid, surfaces) = bound_keyed_box(&mut store);
        let non_plane = store
            .insert_surface(SurfaceGeom::Sphere(
                Sphere::new(Frame::world(), 1.0).unwrap(),
            ))
            .unwrap();

        let mut mixed = valid.clone();
        mixed.faces[0].plane_binding = None;
        let mut wrong_class = valid.clone();
        wrong_class.faces[0].plane_binding.as_mut().unwrap().support = non_plane;
        let mut mismatched = valid.clone();
        let opposite = surfaces
            .iter()
            .copied()
            .find(|surface| {
                *surface != mismatched.faces[0].plane_binding.as_ref().unwrap().support
                    && !mismatched.faces[0]
                        .plane_binding
                        .as_ref()
                        .unwrap()
                        .edge_carriers
                        .contains(surface)
            })
            .unwrap();
        mismatched.faces[0]
            .plane_binding
            .as_mut()
            .unwrap()
            .edge_carriers[0] = opposite;

        let expected = {
            let mut transaction = store.transaction().unwrap();
            let output = transaction.assemble_planar_solid(&valid).unwrap();
            transaction.rollback().unwrap();
            output
        };
        let mut transaction = store.transaction().unwrap();
        let stale = transaction
            .assembly()
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        transaction.assembly().remove_surface(stale).unwrap();
        let mut stale_input = valid.clone();
        stale_input.faces[0].plane_binding.as_mut().unwrap().support = stale;
        let before = (
            transaction.store().count::<Body>(),
            transaction.store().count::<Face>(),
            transaction.store().count::<Edge>(),
            transaction.store().count::<Vertex>(),
            transaction.store().count::<CurveGeom>(),
            transaction.store().count::<crate::geom::Curve2dGeom>(),
        );
        for input in [&mixed, &wrong_class, &mismatched, &stale_input] {
            assert!(transaction.assemble_planar_solid(input).is_err());
            assert_eq!(
                (
                    transaction.store().count::<Body>(),
                    transaction.store().count::<Face>(),
                    transaction.store().count::<Edge>(),
                    transaction.store().count::<Vertex>(),
                    transaction.store().count::<CurveGeom>(),
                    transaction.store().count::<crate::geom::Curve2dGeom>(),
                ),
                before
            );
        }
        let after_refusals = transaction.assemble_planar_solid(&valid).unwrap();
        assert_eq!(after_refusals, expected);
        transaction.rollback().unwrap();
    }

    #[test]
    fn malformed_and_nonmanifold_shells_are_rejected_before_allocation() {
        let input = keyed_box(None);
        let mut open = input.clone();
        open.faces.pop();
        let mut nonmanifold = input;
        nonmanifold.faces.push(nonmanifold.faces[0].clone());
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = (
            transaction.store().count::<Body>(),
            transaction.store().count::<Vertex>(),
        );
        assert!(matches!(
            transaction.assemble_planar_solid(&open),
            Err(Error::InvalidGeometry { .. })
        ));
        assert!(matches!(
            transaction.assemble_planar_solid(&nonmanifold),
            Err(Error::InvalidGeometry { .. })
        ));
        assert_eq!(
            (
                transaction.store().count::<Body>(),
                transaction.store().count::<Vertex>(),
            ),
            before
        );
        transaction.rollback().unwrap();
    }
}
