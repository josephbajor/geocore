//! Certified extraction of a convex planar source body for the symbolic BSP.
//!
//! This is an internal preflight boundary, not a Boolean operation. It accepts
//! one Fast-clean, connected convex polyhedral shell with strictly convex
//! single-loop faces, straight edges, unique supporting planes, and simple
//! three-plane vertices. Everything outside that proof envelope is refused
//! before a symbolic fragment is returned. The exact half-space, planarity,
//! convexity, and incidence proofs below discharge the global obligations
//! needed by this narrower class; unrelated checker-v2 gaps do not become an
//! authority gate for the BSP.

use std::collections::BTreeMap;

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation, orient3d, oriented_plane_triple_intersection_side};
use kgeom::surface::Plane;
use kgeom::vec::Point3;
use kgraph::{AffineParamMap1d, certify_paired_plane_line_residuals};
use ktopo::check::{CheckLevel, CheckOutcome, CheckReport, check_body_report_in_scope};
use ktopo::entity::{
    BodyKind, EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, PcurveEndpointKind,
    RegionKind, SurfaceId as RawSurfaceId, VertexId as RawVertexId,
};
use ktopo::store::Store;

use super::planar_bsp::{ConvexPlanarFragment, PlaneTripleVertexKey, SourcePlane, SourcePlaneRef};
use crate::session::Part;
use crate::{BodyId, EdgeId, FaceId, VertexId};

/// Cumulative topology scans and exact predicates used by source extraction.
pub(crate) const PLANAR_SOURCE_EXTRACTION_WORK: StageId =
    known_stage("kernel.boolean.planar-source-extraction-work");

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in planar source extraction stage identifier"),
    }
}

/// Version-1 bounded work allowance for one extracted operand.
pub(crate) struct PlanarSourceExtractionBudgetProfile;

impl PlanarSourceExtractionBudgetProfile {
    pub(crate) fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([LimitSpec::new(
            PLANAR_SOURCE_EXTRACTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            1_000_000,
        )])
        .expect("built-in planar source extraction budget is valid")
    }
}

/// A valid input outside the bounded source-body class supported by the BSP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PlanarSourceGap {
    NonSolidBody,
    RegionLayout,
    ShellLayout,
    TolerantEntity,
    NonPlanarFace,
    FaceLoopLayout,
    NonLineEdge,
    CoplanarFacetPartition,
    NonSimpleVertex,
}

/// An exact invariant required by the symbolic representation was not proven.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PlanarSourceProofFailure {
    NonFiniteInteriorSample,
    DegenerateSupportingPlane,
    NonPlanarBoundary,
    NonConvexFace,
    NonConvexBody,
    InconsistentAdjacency,
    FragmentContract,
    WorkCountOverflow,
}

/// Fail-closed result of source-body preflight.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum PlanarSourceExtractionError {
    InvalidOperand,
    WrongPart,
    Topology(kcore::error::Error),
    NotFastValid(CheckReport),
    Unsupported(PlanarSourceGap),
    Uncertified(PlanarSourceProofFailure),
}

/// Original topological face corresponding to one symbolic plane.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ExtractedSourceFace {
    plane: SourcePlaneRef,
    face: FaceId,
    surface: RawSurfaceId,
}

impl ExtractedSourceFace {
    pub(crate) const fn plane(&self) -> SourcePlaneRef {
        self.plane
    }

    pub(crate) fn face(&self) -> FaceId {
        self.face.clone()
    }

    pub(crate) const fn surface(&self) -> RawSurfaceId {
        self.surface
    }
}

/// Original edge at the intersection of two source planes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ExtractedSourceEdge {
    planes: [SourcePlaneRef; 2],
    edge: EdgeId,
}

impl ExtractedSourceEdge {
    pub(crate) const fn planes(&self) -> [SourcePlaneRef; 2] {
        self.planes
    }

    pub(crate) fn edge(&self) -> EdgeId {
        self.edge.clone()
    }
}

/// Stored representative for an exact simple three-plane vertex.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ExtractedSourceVertex {
    key: PlaneTripleVertexKey,
    vertex: VertexId,
    position: Point3,
}

impl ExtractedSourceVertex {
    pub(crate) const fn key(&self) -> PlaneTripleVertexKey {
        self.key
    }

    pub(crate) fn vertex(&self) -> VertexId {
        self.vertex.clone()
    }

    pub(crate) const fn position(&self) -> Point3 {
        self.position
    }
}

/// Deterministic semantic input for one operand of the exact planar BSP.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ExtractedPlanarSourceBody {
    interior_sample: Point3,
    planes: Vec<SourcePlane>,
    faces: Vec<ExtractedSourceFace>,
    edges: Vec<ExtractedSourceEdge>,
    vertices: Vec<ExtractedSourceVertex>,
    fragments: Vec<ConvexPlanarFragment>,
}

impl ExtractedPlanarSourceBody {
    /// Strict material-side witness shared by cross-operand support proofs.
    pub(crate) const fn interior_sample(&self) -> Point3 {
        self.interior_sample
    }

    pub(crate) fn planes(&self) -> &[SourcePlane] {
        &self.planes
    }

    pub(crate) fn plane_ids(&self) -> impl ExactSizeIterator<Item = SourcePlaneRef> + '_ {
        self.planes.iter().map(|plane| plane.id())
    }

    pub(crate) fn faces(&self) -> &[ExtractedSourceFace] {
        &self.faces
    }

    pub(crate) fn edges(&self) -> &[ExtractedSourceEdge] {
        &self.edges
    }

    pub(crate) fn vertices(&self) -> &[ExtractedSourceVertex] {
        &self.vertices
    }

    pub(crate) fn fragments(&self) -> &[ConvexPlanarFragment] {
        &self.fragments
    }
}

#[derive(Debug)]
struct FaceSeed {
    raw: RawFaceId,
    surface: RawSurfaceId,
    plane: Plane,
    id: SourcePlaneRef,
    vertices: Vec<RawVertexId>,
    edges: Vec<RawEdgeId>,
}

type EdgePlaneLookup = Vec<((SourcePlaneRef, RawEdgeId), SourcePlaneRef)>;
type ExtractedEdges = (Vec<ExtractedSourceEdge>, EdgePlaneLookup);

/// Extract a certified convex planar body without allocating or mutating.
///
/// The caller supplies the operation scope so Fast checking shares the
/// enclosing Boolean's accounting. The scope must include the checker-v2 Fast
/// budget family; a missing or exhausted allowance is returned as its typed
/// lower error through [`PlanarSourceExtractionError::Topology`].
pub(crate) fn extract_planar_source_body(
    part: &Part<'_>,
    body: BodyId,
    operand: u8,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ExtractedPlanarSourceBody, PlanarSourceExtractionError> {
    if operand > 1 {
        return Err(PlanarSourceExtractionError::InvalidOperand);
    }
    if body.part() != &part.id {
        return Err(PlanarSourceExtractionError::WrongPart);
    }

    scope
        .ledger()
        .require_limit(
            PLANAR_SOURCE_EXTRACTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )
        .map_err(|source| PlanarSourceExtractionError::Topology(source.into()))?;

    let store = &part.state.store;
    store
        .get(body.raw())
        .map_err(PlanarSourceExtractionError::Topology)?;
    let report = check_body_report_in_scope(store, body.raw(), CheckLevel::Fast, scope)
        .map_err(PlanarSourceExtractionError::Topology)?;
    if report.outcome() != CheckOutcome::Valid {
        return Err(PlanarSourceExtractionError::NotFastValid(report));
    }

    let (raw_faces, body_vertices) = preflight_body_layout(store, body.raw(), scope)?;
    let seeds = prepare_face_seeds(store, operand, &raw_faces, scope)?;
    if seeds.len() < 4 || body_vertices.len() < 4 {
        return unsupported(PlanarSourceGap::ShellLayout);
    }
    let interior_sample = strict_interior_sample(store, &body_vertices)?;
    charge_exact_work(scope, &seeds, body_vertices.len())?;
    let planes = build_source_planes(&seeds, interior_sample)?;
    certify_unique_planes(&planes)?;
    let incidence = collect_vertex_plane_incidence(&seeds)?;
    let vertices = extract_vertices(part, store, &incidence)?;
    let linear_tolerance = scope.context().session().precision().linear_resolution();
    let (edges, edge_planes) = extract_edge_planes(part, store, &seeds, linear_tolerance)?;
    certify_source_plane_complex(&seeds, &body_vertices, &incidence, &edge_planes, &planes)?;
    let fragments = build_fragments(&seeds, &incidence, &edge_planes)?;
    let faces = seeds
        .iter()
        .map(|seed| ExtractedSourceFace {
            plane: seed.id,
            face: FaceId::new(part.id.clone(), seed.raw),
            surface: seed.surface,
        })
        .collect();

    Ok(ExtractedPlanarSourceBody {
        interior_sample,
        planes,
        faces,
        edges,
        vertices,
        fragments,
    })
}

fn preflight_body_layout(
    store: &Store,
    body_id: ktopo::entity::BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<(Vec<RawFaceId>, Vec<RawVertexId>), PlanarSourceExtractionError> {
    charge(scope, 1)?;
    let body = store
        .get(body_id)
        .map_err(PlanarSourceExtractionError::Topology)?;
    if body.kind() != BodyKind::Solid {
        return unsupported(PlanarSourceGap::NonSolidBody);
    }
    if body.regions().len() != 2 {
        return unsupported(PlanarSourceGap::RegionLayout);
    }
    let exterior = store
        .get(body.regions()[0])
        .map_err(PlanarSourceExtractionError::Topology)?;
    let material = store
        .get(body.regions()[1])
        .map_err(PlanarSourceExtractionError::Topology)?;
    if exterior.kind() != RegionKind::Void
        || !exterior.shells().is_empty()
        || material.kind() != RegionKind::Solid
        || material.shells().len() != 1
    {
        return unsupported(PlanarSourceGap::RegionLayout);
    }
    let shell = store
        .get(material.shells()[0])
        .map_err(PlanarSourceExtractionError::Topology)?;
    if !shell.edges().is_empty() || shell.vertex().is_some() || shell.faces().is_empty() {
        return unsupported(PlanarSourceGap::ShellLayout);
    }
    if shell.faces().len() > u32::MAX as usize {
        return unsupported(PlanarSourceGap::ShellLayout);
    }
    charge(scope, count(shell.faces().len())?)?;
    let vertices = store
        .vertices_of_body(body_id)
        .map_err(PlanarSourceExtractionError::Topology)?;
    charge(scope, count(vertices.len())?)?;
    Ok((shell.faces().to_vec(), vertices))
}

fn strict_interior_sample(
    store: &Store,
    vertices: &[RawVertexId],
) -> Result<Point3, PlanarSourceExtractionError> {
    let mut sum = [0.0; 3];
    for &vertex in vertices {
        let entity = store
            .get(vertex)
            .map_err(PlanarSourceExtractionError::Topology)?;
        if entity.tolerance().is_some() {
            return unsupported(PlanarSourceGap::TolerantEntity);
        }
        let point = store
            .vertex_position(vertex)
            .map_err(PlanarSourceExtractionError::Topology)?;
        let coordinates = point.to_array();
        if coordinates.iter().any(|coordinate| !coordinate.is_finite()) {
            return uncertified(PlanarSourceProofFailure::NonFiniteInteriorSample);
        }
        for axis in 0..3 {
            sum[axis] += coordinates[axis];
        }
    }
    let denominator = vertices.len() as f64;
    let point = Point3::new(
        sum[0] / denominator,
        sum[1] / denominator,
        sum[2] / denominator,
    );
    if point
        .to_array()
        .iter()
        .any(|coordinate| !coordinate.is_finite())
    {
        return uncertified(PlanarSourceProofFailure::NonFiniteInteriorSample);
    }
    Ok(point)
}

fn prepare_face_seeds(
    store: &Store,
    operand: u8,
    raw_faces: &[RawFaceId],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Vec<FaceSeed>, PlanarSourceExtractionError> {
    let mut seeds = Vec::with_capacity(raw_faces.len());
    for (face_index, &raw) in raw_faces.iter().enumerate() {
        charge(scope, 1)?;
        let face = store
            .get(raw)
            .map_err(PlanarSourceExtractionError::Topology)?;
        if face.tolerance().is_some() {
            return unsupported(PlanarSourceGap::TolerantEntity);
        }
        let Some(plane) = store
            .geometry()
            .surface(face.surface())
            .and_then(|surface| surface.as_plane())
            .copied()
        else {
            return unsupported(PlanarSourceGap::NonPlanarFace);
        };
        if face.loops().len() != 1 {
            return unsupported(PlanarSourceGap::FaceLoopLayout);
        }
        let loop_value = store
            .get(face.loops()[0])
            .map_err(PlanarSourceExtractionError::Topology)?;
        if loop_value.fins().len() < 3 {
            return unsupported(PlanarSourceGap::FaceLoopLayout);
        }

        let mut vertices = Vec::with_capacity(loop_value.fins().len());
        let mut edges = Vec::with_capacity(loop_value.fins().len());
        for &fin_id in loop_value.fins() {
            charge(scope, 1)?;
            let fin = store
                .get(fin_id)
                .map_err(PlanarSourceExtractionError::Topology)?;
            let edge = store
                .get(fin.edge())
                .map_err(PlanarSourceExtractionError::Topology)?;
            if edge.tolerance().is_some()
                || edge.vertices().iter().any(Option::is_none)
                || edge.bounds().is_none()
                || edge
                    .curve()
                    .and_then(|curve| store.geometry().curve(curve))
                    .and_then(|curve| curve.as_line())
                    .is_none()
            {
                return unsupported(PlanarSourceGap::NonLineEdge);
            }
            let tail = store
                .fin_tail(fin_id)
                .map_err(PlanarSourceExtractionError::Topology)?
                .ok_or(PlanarSourceExtractionError::Unsupported(
                    PlanarSourceGap::NonLineEdge,
                ))?;
            vertices.push(tail);
            edges.push(fin.edge());
        }
        seeds.push(FaceSeed {
            raw,
            surface: face.surface(),
            plane,
            id: SourcePlaneRef::new(operand, face_index as u32),
            vertices,
            edges,
        });
    }
    Ok(seeds)
}

/// Charge a conservative, input-size-exact upper bound for every remaining
/// nonconstant scan and exact predicate. Charging once before those phases
/// makes a limit refusal read-only and independent of early-exit geometry.
fn charge_exact_work(
    scope: &mut OperationScope<'_, '_>,
    seeds: &[FaceSeed],
    body_vertex_count: usize,
) -> Result<(), PlanarSourceExtractionError> {
    let faces = count(seeds.len())?;
    let vertices = count(body_vertex_count)?;
    let ring_uses = seeds.iter().try_fold(0_u64, |total, seed| {
        total.checked_add(count(seed.vertices.len())?).ok_or(
            PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::WorkCountOverflow),
        )
    })?;
    let plane_pairs = faces
        .checked_mul(faces.saturating_sub(1))
        .and_then(|value| value.checked_div(2))
        .ok_or(PlanarSourceExtractionError::Uncertified(
            PlanarSourceProofFailure::WorkCountOverflow,
        ))?;
    let point_membership_and_side =
        checked_mul(checked_mul(faces, ring_uses)?, checked_add(vertices, 1)?)?;
    let terms = [
        checked_mul(2, ring_uses)?,
        point_membership_and_side,
        checked_mul(3, plane_pairs)?,
        checked_mul(ring_uses, checked_add(vertices, 3)?)?,
        vertices,
        checked_mul(ring_uses, faces)?,
        checked_mul(3, ring_uses)?,
        checked_mul(ring_uses, checked_add(vertices, ring_uses)?)?,
        // Every undirected source edge is visited twice through its two face
        // rings. This deliberately conservative term covers both complete
        // lifted-pcurve residual enclosures plus their descriptor checks.
        checked_mul(32, ring_uses)?,
    ];
    let work = terms.into_iter().try_fold(0_u64, checked_add)?;
    charge(scope, work)
}

fn build_source_planes(
    seeds: &[FaceSeed],
    interior_sample: Point3,
) -> Result<Vec<SourcePlane>, PlanarSourceExtractionError> {
    let interior = interior_sample.to_array();
    let mut planes = Vec::with_capacity(seeds.len());
    for seed in seeds {
        let witness = semantic_plane_witness(seed.plane);
        let plane = SourcePlane::from_interior_sample(seed.id, witness, interior).ok_or(
            PlanarSourceExtractionError::Uncertified(
                PlanarSourceProofFailure::DegenerateSupportingPlane,
            ),
        )?;
        planes.push(plane);
    }
    Ok(planes)
}

fn certify_source_plane_complex(
    seeds: &[FaceSeed],
    body_vertices: &[RawVertexId],
    incidence: &[(RawVertexId, PlaneTripleVertexKey)],
    edge_planes: &EdgePlaneLookup,
    planes: &[SourcePlane],
) -> Result<(), PlanarSourceExtractionError> {
    if incidence.len() != body_vertices.len()
        || body_vertices
            .iter()
            .any(|vertex| !incidence.iter().any(|(candidate, _)| candidate == vertex))
    {
        return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
    }

    // A topology vertex is represented by its three exact semantic support
    // planes. Rounded B-rep positions are representatives only; all BSP side
    // decisions below classify the ideal plane-triple intersection.
    for (_, key) in incidence {
        for plane in planes {
            let side = ideal_vertex_side(*key, plane, planes)?;
            if key.planes().contains(&plane.id()) {
                if side != Orientation::Zero {
                    return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
                }
            } else if side != plane.interior_side() {
                return uncertified(PlanarSourceProofFailure::NonConvexBody);
            }
        }
    }

    // Every directed face edge is the intersection with one neighboring
    // support. Both endpoints must contain that support, while every other
    // ideal face vertex lies strictly on its material side. This exact
    // half-space proof replaces rounded-point turn tests and certifies a
    // strictly convex face independently of its frame orientation.
    for seed in seeds {
        for edge_index in 0..seed.edges.len() {
            let edge = seed.edges[edge_index];
            let other_id = edge_planes
                .iter()
                .find(|((face, candidate), _)| *face == seed.id && *candidate == edge)
                .map(|(_, other)| *other)
                .ok_or(PlanarSourceExtractionError::Uncertified(
                    PlanarSourceProofFailure::InconsistentAdjacency,
                ))?;
            let other = source_plane(planes, other_id)?;
            let next = (edge_index + 1) % seed.vertices.len();
            for (vertex_index, raw_vertex) in seed.vertices.iter().enumerate() {
                let key = incidence
                    .iter()
                    .find(|(candidate, _)| candidate == raw_vertex)
                    .map(|(_, key)| *key)
                    .ok_or(PlanarSourceExtractionError::Uncertified(
                        PlanarSourceProofFailure::InconsistentAdjacency,
                    ))?;
                if !key.planes().contains(&seed.id) {
                    return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
                }
                let side = ideal_vertex_side(key, other, planes)?;
                if vertex_index == edge_index || vertex_index == next {
                    if side != Orientation::Zero {
                        return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
                    }
                } else if side != other.interior_side() {
                    return uncertified(PlanarSourceProofFailure::NonConvexFace);
                }
            }
        }
    }
    Ok(())
}

fn semantic_plane_witness(plane: Plane) -> [[f64; 3]; 3] {
    let frame = plane.frame();
    [
        frame.origin().to_array(),
        frame.point_at(1.0, 0.0, 0.0).to_array(),
        frame.point_at(0.0, 1.0, 0.0).to_array(),
    ]
}

fn source_plane(
    planes: &[SourcePlane],
    id: SourcePlaneRef,
) -> Result<&SourcePlane, PlanarSourceExtractionError> {
    planes
        .iter()
        .find(|plane| plane.id() == id)
        .ok_or(PlanarSourceExtractionError::Uncertified(
            PlanarSourceProofFailure::InconsistentAdjacency,
        ))
}

fn ideal_vertex_side(
    key: PlaneTripleVertexKey,
    query: &SourcePlane,
    planes: &[SourcePlane],
) -> Result<Orientation, PlanarSourceExtractionError> {
    // Source-plane identity is the exact boundary authority. Re-running the
    // general four-plane expansion predicate for an incident query adds no
    // geometric information and can exceed that predicate's conservative
    // arithmetic envelope after otherwise harmless frame rounding.
    if key.planes().contains(&query.id()) {
        return Ok(Orientation::Zero);
    }
    let [first, second, third] = key.planes();
    let defining = [
        source_plane(planes, first)?.points(),
        source_plane(planes, second)?.points(),
        source_plane(planes, third)?.points(),
    ];
    oriented_plane_triple_intersection_side(defining, query.points())
        .map(|side| side.sign())
        .ok_or(PlanarSourceExtractionError::Uncertified(
            PlanarSourceProofFailure::DegenerateSupportingPlane,
        ))
}

fn certify_unique_planes(planes: &[SourcePlane]) -> Result<(), PlanarSourceExtractionError> {
    for first in 0..planes.len() {
        let witness = planes[first].points();
        for second in planes.iter().skip(first + 1) {
            if second.points().iter().all(|point| {
                orient3d(witness[0], witness[1], witness[2], *point) == Orientation::Zero
            }) {
                return unsupported(PlanarSourceGap::CoplanarFacetPartition);
            }
        }
    }
    Ok(())
}

fn collect_vertex_plane_incidence(
    seeds: &[FaceSeed],
) -> Result<Vec<(RawVertexId, PlaneTripleVertexKey)>, PlanarSourceExtractionError> {
    let mut incident: Vec<(RawVertexId, Vec<SourcePlaneRef>)> = Vec::new();
    for seed in seeds {
        for &vertex in &seed.vertices {
            let planes = match incident
                .iter_mut()
                .find(|(candidate, _)| *candidate == vertex)
            {
                Some((_, planes)) => planes,
                None => {
                    incident.push((vertex, Vec::new()));
                    &mut incident.last_mut().expect("an entry was just pushed").1
                }
            };
            if !planes.contains(&seed.id) {
                planes.push(seed.id);
            }
        }
    }
    incident
        .into_iter()
        .map(|(vertex, planes)| {
            let values: [SourcePlaneRef; 3] = planes.try_into().map_err(|_| {
                PlanarSourceExtractionError::Unsupported(PlanarSourceGap::NonSimpleVertex)
            })?;
            let key = PlaneTripleVertexKey::new(values).ok_or(
                PlanarSourceExtractionError::Uncertified(
                    PlanarSourceProofFailure::InconsistentAdjacency,
                ),
            )?;
            Ok((vertex, key))
        })
        .collect()
}

fn extract_vertices(
    part: &Part<'_>,
    store: &Store,
    incidence: &[(RawVertexId, PlaneTripleVertexKey)],
) -> Result<Vec<ExtractedSourceVertex>, PlanarSourceExtractionError> {
    incidence
        .iter()
        .map(|(raw, key)| {
            Ok(ExtractedSourceVertex {
                key: *key,
                vertex: VertexId::new(part.id.clone(), *raw),
                position: store
                    .vertex_position(*raw)
                    .map_err(PlanarSourceExtractionError::Topology)?,
            })
        })
        .collect()
}

fn extract_edge_planes(
    part: &Part<'_>,
    store: &Store,
    seeds: &[FaceSeed],
    linear_tolerance: f64,
) -> Result<ExtractedEdges, PlanarSourceExtractionError> {
    let face_ids = seeds
        .iter()
        .map(|seed| (seed.raw, seed.id))
        .collect::<Vec<_>>();
    let mut adjacency = Vec::new();
    let mut source_edges: BTreeMap<[SourcePlaneRef; 2], RawEdgeId> = BTreeMap::new();
    let mut certified_edges = Vec::new();
    for seed in seeds {
        for &edge_id in &seed.edges {
            let edge = store
                .get(edge_id)
                .map_err(PlanarSourceExtractionError::Topology)?;
            if edge.fins().len() != 2 {
                return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
            }
            let other = other_face(store, edge.fins(), seed.raw)?;
            let other_plane = face_ids
                .iter()
                .find(|(face, _)| *face == other)
                .map(|(_, plane)| *plane)
                .ok_or(PlanarSourceExtractionError::Uncertified(
                    PlanarSourceProofFailure::InconsistentAdjacency,
                ))?;
            if other_plane == seed.id {
                return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
            }
            if !certified_edges.contains(&edge_id) {
                certify_semantic_edge_incidence(store, edge_id, seeds, linear_tolerance)?;
                certified_edges.push(edge_id);
            }
            adjacency.push(((seed.id, edge_id), other_plane));
            let planes = if seed.id < other_plane {
                [seed.id, other_plane]
            } else {
                [other_plane, seed.id]
            };
            match source_edges.insert(planes, edge_id) {
                Some(previous) if previous != edge_id => {
                    return unsupported(PlanarSourceGap::NonSimpleVertex);
                }
                _ => {}
            }
        }
    }
    let edges = source_edges
        .into_iter()
        .map(|(planes, raw)| ExtractedSourceEdge {
            planes,
            edge: EdgeId::new(part.id.clone(), raw),
        })
        .collect();
    Ok((edges, adjacency))
}

/// License one rounded line carrier as the ideal intersection of its two
/// topology-owning Plane fields. The whole finite carrier and both authored
/// line pcurves are enclosed by the graph certifier before topology incidence
/// is allowed to stand in for rounded point-on-plane equality.
fn certify_semantic_edge_incidence(
    store: &Store,
    edge_id: RawEdgeId,
    seeds: &[FaceSeed],
    linear_tolerance: f64,
) -> Result<(), PlanarSourceExtractionError> {
    let edge = store
        .get(edge_id)
        .map_err(PlanarSourceExtractionError::Topology)?;
    let (Some(curve_id), Some((lo, hi)), [Some(first), Some(second)]) =
        (edge.curve(), edge.bounds(), edge.vertices())
    else {
        return uncertified(PlanarSourceProofFailure::NonPlanarBoundary);
    };
    if first == second || edge.fins().len() != 2 {
        return uncertified(PlanarSourceProofFailure::NonPlanarBoundary);
    }
    let line = store
        .geometry()
        .curve(curve_id)
        .and_then(|curve| curve.as_line())
        .copied()
        .ok_or(PlanarSourceExtractionError::Uncertified(
            PlanarSourceProofFailure::NonPlanarBoundary,
        ))?;

    let mut surfaces = Vec::with_capacity(2);
    let mut pcurves = Vec::with_capacity(2);
    let mut maps = Vec::with_capacity(2);
    for &fin_id in edge.fins() {
        let fin = store
            .get(fin_id)
            .map_err(PlanarSourceExtractionError::Topology)?;
        let loop_value = store
            .get(fin.parent())
            .map_err(PlanarSourceExtractionError::Topology)?;
        let seed = seeds
            .iter()
            .find(|seed| seed.raw == loop_value.face())
            .ok_or(PlanarSourceExtractionError::Uncertified(
                PlanarSourceProofFailure::NonPlanarBoundary,
            ))?;
        let use_ = fin
            .pcurve()
            .ok_or(PlanarSourceExtractionError::Uncertified(
                PlanarSourceProofFailure::NonPlanarBoundary,
            ))?;
        if !use_.chart().is_identity()
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
            || use_.endpoint_kinds() != [PcurveEndpointKind::Regular; 2]
        {
            return uncertified(PlanarSourceProofFailure::NonPlanarBoundary);
        }
        let topology_map = use_.edge_to_pcurve();
        let mapped = [topology_map.map(lo), topology_map.map(hi)];
        let expected_range = if mapped[0] < mapped[1] {
            [mapped[0], mapped[1]]
        } else {
            [mapped[1], mapped[0]]
        };
        let active = use_.range();
        if expected_range != [active.lo, active.hi] {
            return uncertified(PlanarSourceProofFailure::NonPlanarBoundary);
        }
        let pcurve = store
            .geometry()
            .curve2d(use_.curve())
            .and_then(|curve| curve.as_line())
            .copied()
            .ok_or(PlanarSourceExtractionError::Uncertified(
                PlanarSourceProofFailure::NonPlanarBoundary,
            ))?;
        let map =
            AffineParamMap1d::new(topology_map.scale(), topology_map.offset()).map_err(|_| {
                PlanarSourceExtractionError::Uncertified(
                    PlanarSourceProofFailure::NonPlanarBoundary,
                )
            })?;
        surfaces.push(seed.plane);
        pcurves.push(pcurve);
        maps.push(map);
    }
    let surfaces: [Plane; 2] = surfaces.try_into().map_err(|_| {
        PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::NonPlanarBoundary)
    })?;
    let pcurves = pcurves.try_into().map_err(|_| {
        PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::NonPlanarBoundary)
    })?;
    let maps = maps.try_into().map_err(|_| {
        PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::NonPlanarBoundary)
    })?;
    certify_paired_plane_line_residuals(
        line,
        kgeom::param::ParamRange::new(lo, hi),
        surfaces,
        pcurves,
        maps,
        linear_tolerance,
    )
    .map_err(|_| {
        PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::NonPlanarBoundary)
    })?;
    Ok(())
}

fn other_face(
    store: &Store,
    fins: &[RawFinId],
    current: RawFaceId,
) -> Result<RawFaceId, PlanarSourceExtractionError> {
    let mut other = None;
    for &fin_id in fins {
        let fin = store
            .get(fin_id)
            .map_err(PlanarSourceExtractionError::Topology)?;
        let loop_value = store
            .get(fin.parent())
            .map_err(PlanarSourceExtractionError::Topology)?;
        if loop_value.face() != current && other.replace(loop_value.face()).is_some() {
            return uncertified(PlanarSourceProofFailure::InconsistentAdjacency);
        }
    }
    other.ok_or(PlanarSourceExtractionError::Uncertified(
        PlanarSourceProofFailure::InconsistentAdjacency,
    ))
}

fn build_fragments(
    seeds: &[FaceSeed],
    incidence: &[(RawVertexId, PlaneTripleVertexKey)],
    edge_planes: &EdgePlaneLookup,
) -> Result<Vec<ConvexPlanarFragment>, PlanarSourceExtractionError> {
    seeds
        .iter()
        .map(|seed| {
            let vertices = seed
                .vertices
                .iter()
                .map(|vertex| {
                    incidence
                        .iter()
                        .find(|(candidate, _)| *candidate == *vertex)
                        .map(|(_, key)| *key)
                        .ok_or(PlanarSourceExtractionError::Uncertified(
                            PlanarSourceProofFailure::InconsistentAdjacency,
                        ))
                })
                .collect::<Result<Vec<_>, _>>()?;
            let edges = seed
                .edges
                .iter()
                .map(|edge| {
                    edge_planes
                        .iter()
                        .find(|((plane, candidate), _)| *plane == seed.id && candidate == edge)
                        .map(|(_, other)| *other)
                        .ok_or(PlanarSourceExtractionError::Uncertified(
                            PlanarSourceProofFailure::InconsistentAdjacency,
                        ))
                })
                .collect::<Result<Vec<_>, _>>()?;
            ConvexPlanarFragment::new(seed.id, vertices, edges).map_err(|_| {
                PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::FragmentContract)
            })
        })
        .collect()
}

fn unsupported<T>(gap: PlanarSourceGap) -> Result<T, PlanarSourceExtractionError> {
    Err(PlanarSourceExtractionError::Unsupported(gap))
}

fn uncertified<T>(failure: PlanarSourceProofFailure) -> Result<T, PlanarSourceExtractionError> {
    Err(PlanarSourceExtractionError::Uncertified(failure))
}

fn charge(
    scope: &mut OperationScope<'_, '_>,
    amount: u64,
) -> Result<(), PlanarSourceExtractionError> {
    scope
        .ledger_mut()
        .charge(PLANAR_SOURCE_EXTRACTION_WORK, amount)
        .map_err(|source| PlanarSourceExtractionError::Topology(source.into()))
}

fn count(value: usize) -> Result<u64, PlanarSourceExtractionError> {
    u64::try_from(value).map_err(|_| {
        PlanarSourceExtractionError::Uncertified(PlanarSourceProofFailure::WorkCountOverflow)
    })
}

fn checked_mul(first: u64, second: u64) -> Result<u64, PlanarSourceExtractionError> {
    first
        .checked_mul(second)
        .ok_or(PlanarSourceExtractionError::Uncertified(
            PlanarSourceProofFailure::WorkCountOverflow,
        ))
}

fn checked_add(first: u64, second: u64) -> Result<u64, PlanarSourceExtractionError> {
    first
        .checked_add(second)
        .ok_or(PlanarSourceExtractionError::Uncertified(
            PlanarSourceProofFailure::WorkCountOverflow,
        ))
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationContext, OperationScope,
        ResourceKind,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point2, Vec2, Vec3};
    use ktopo::check::{CheckBudgetProfile, CheckLevel};
    use ktopo::geom::Curve2dGeom;
    use ktopo::planar::{PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey};
    use ktopo::profile::PlanarProfile;

    use super::*;
    use crate::{Kernel, PartId, Session};

    fn add_body(
        session: &mut Session,
        part_id: &PartId,
        build: impl FnOnce(&mut Store) -> ktopo::entity::BodyId,
    ) -> BodyId {
        let raw = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            build(edit.store_mut_for_test())
        };
        BodyId::new(part_id.clone(), raw)
    }

    fn extract(
        session: &Session,
        part_id: &PartId,
        body: BodyId,
        operand: u8,
    ) -> Result<ExtractedPlanarSourceBody, PlanarSourceExtractionError> {
        let part = session.part(part_id.clone()).unwrap();
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                CheckBudgetProfile::v1_defaults(CheckLevel::Fast)
                    .overlaid(&PlanarSourceExtractionBudgetProfile::v1_defaults()),
            );
        let mut scope = OperationScope::new(&context);
        extract_planar_source_body(&part, body, operand, &mut scope)
    }

    fn extract_with_work_limit(
        session: &Session,
        part_id: &PartId,
        body: BodyId,
        allowed: u64,
    ) -> (
        Result<ExtractedPlanarSourceBody, PlanarSourceExtractionError>,
        Vec<LimitSnapshot>,
    ) {
        let part = session.part(part_id.clone()).unwrap();
        let extraction = BudgetPlan::new([LimitSpec::new(
            PLANAR_SOURCE_EXTRACTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                CheckBudgetProfile::v1_defaults(CheckLevel::Fast).overlaid(&extraction),
            );
        let mut scope = OperationScope::new(&context);
        let result = extract_planar_source_body(&part, body, 0, &mut scope);
        let usage = scope.ledger().snapshots();
        (result, usage)
    }

    fn tetrahedron() -> PlanarSolidInput {
        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(0.0, 0.0, 4.0),
        ];
        let keys = [
            PlanarVertexKey::new(41),
            PlanarVertexKey::new(17),
            PlanarVertexKey::new(89),
            PlanarVertexKey::new(5),
        ];
        let vertices = keys
            .into_iter()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
            .collect();
        let faces = [[1, 2, 3], [0, 3, 2], [0, 1, 3], [0, 2, 1]]
            .into_iter()
            .map(|ring| PlanarSolidFace::new(ring.map(|index| keys[index]).to_vec()))
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn assemble_input(store: &mut Store, input: &PlanarSolidInput) -> ktopo::entity::BodyId {
        let mut transaction = store.transaction().unwrap();
        let body = transaction.assemble_planar_solid(input).unwrap().body();
        transaction.commit_checked(&[body]).unwrap();
        body
    }

    #[test]
    fn rotated_off_origin_block_extracts_deterministically() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let frame = Frame::new(
            Point3::new(13.0, -17.0, 23.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        let body = add_body(&mut session, &part_id, |store| {
            ktopo::make::block(store, &frame, [2.0, 3.0, 5.0]).unwrap()
        });

        let first = extract(&session, &part_id, body.clone(), 0).unwrap();
        let second = extract(&session, &part_id, body, 0).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.planes().len(), 6);
        assert_eq!(first.faces().len(), 6);
        assert_eq!(first.edges().len(), 12);
        assert_eq!(first.vertices().len(), 8);
        assert_eq!(first.fragments().len(), 6);
        assert!(
            first
                .fragments()
                .iter()
                .all(|face| face.vertices().len() == 4)
        );
        assert!(
            first
                .planes()
                .iter()
                .all(|plane| plane.interior_side() != Orientation::Zero)
        );
        let part = session.part(part_id).unwrap();
        assert!(first.faces().iter().all(|face| {
            part.state
                .store
                .get(face.face().raw())
                .is_ok_and(|source| source.surface() == face.surface())
                && part
                    .state
                    .store
                    .geometry()
                    .surface(face.surface())
                    .is_some_and(|surface| surface.as_plane().is_some())
        }));
    }

    #[test]
    fn incident_semantic_plane_identity_owns_exact_boundary_classification() {
        let ids = [
            SourcePlaneRef::new(0, 0),
            SourcePlaneRef::new(0, 2),
            SourcePlaneRef::new(0, 4),
        ];
        // These are the non-dyadic translated witnesses from the arbitrary-
        // angle Boolean regression. The bounded general predicate may refuse
        // their cancellation when the query repeats a defining plane, but
        // symbolic incidence already proves the exact zero relation.
        let witnesses = [
            [[-0.3, 0.2, -1.6], [-0.3, 1.2, -1.6], [0.7, 0.2, -1.6]],
            [[-0.3, -1.55, -0.1], [0.7, -1.55, -0.1], [-0.3, -1.55, 0.9]],
            [[-2.3, 0.2, -0.1], [-2.3, 0.2, 0.9], [-2.3, 1.2, -0.1]],
        ];
        let interior = [0.0, 0.0, 0.0];
        let planes = ids
            .into_iter()
            .zip(witnesses)
            .map(|(id, witness)| SourcePlane::from_interior_sample(id, witness, interior).unwrap())
            .collect::<Vec<_>>();
        let key = PlaneTripleVertexKey::new(ids).unwrap();

        for query in &planes {
            assert_eq!(
                ideal_vertex_side(key, query, &planes),
                Ok(Orientation::Zero)
            );
        }
    }

    #[test]
    fn all_nonzero_oblique_profile_extrusion_uses_semantic_plane_incidence() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap();
        let profile = PlanarProfile::from_polygon(
            frame,
            &[
                Point2::new(-1.25, -0.75),
                Point2::new(1.25, -0.75),
                Point2::new(1.25, 0.75),
                Point2::new(-1.25, 0.75),
            ],
        )
        .unwrap();
        let body = add_body(&mut session, &part_id, |store| {
            ktopo::make::extrude_profile(store, &profile, 2.0).unwrap()
        });

        let first = extract(&session, &part_id, body.clone(), 0).unwrap();
        let second = extract(&session, &part_id, body, 0).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.planes().len(), 6);
        assert_eq!(first.faces().len(), 6);
        assert_eq!(first.edges().len(), 12);
        assert_eq!(first.vertices().len(), 8);
        assert_eq!(first.fragments().len(), 6);

        // The fixture intentionally contains rounded representatives that
        // are not exact orient3d-zero on every semantic support plane. Their
        // exact BSP positions are instead the certified plane triples.
        let has_rounded_incidence_mismatch = first.vertices().iter().any(|vertex| {
            vertex.key().planes().into_iter().any(|plane_id| {
                let plane = first
                    .planes()
                    .iter()
                    .find(|plane| plane.id() == plane_id)
                    .unwrap();
                let points = plane.points();
                orient3d(
                    points[0],
                    points[1],
                    points[2],
                    vertex.position().to_array(),
                ) != Orientation::Zero
            })
        });
        assert!(has_rounded_incidence_mismatch);

        for vertex in first.vertices() {
            let defining = vertex.key().planes().map(|plane_id| {
                first
                    .planes()
                    .iter()
                    .find(|plane| plane.id() == plane_id)
                    .unwrap()
                    .points()
            });
            for plane_id in vertex.key().planes() {
                let plane = first
                    .planes()
                    .iter()
                    .find(|plane| plane.id() == plane_id)
                    .unwrap();
                assert_eq!(
                    oriented_plane_triple_intersection_side(defining, plane.points())
                        .unwrap()
                        .sign(),
                    Orientation::Zero
                );
            }
        }
    }

    #[test]
    fn perturbed_unlicensed_planar_incidence_is_refused() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = add_body(&mut session, &part_id, |store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 3.0, 5.0]).unwrap()
        });
        let (seeds, edge, pcurve) = {
            let part = session.part(part_id.clone()).unwrap();
            let context = OperationContext::new(part.policy(), Tolerances::default())
                .unwrap()
                .with_family_budget_defaults(
                    CheckBudgetProfile::v1_defaults(CheckLevel::Fast)
                        .overlaid(&PlanarSourceExtractionBudgetProfile::v1_defaults()),
                );
            let mut scope = OperationScope::new(&context);
            let (faces, _) =
                preflight_body_layout(&part.state.store, body.raw(), &mut scope).unwrap();
            let seeds = prepare_face_seeds(&part.state.store, 0, &faces, &mut scope).unwrap();
            let edge = seeds[0].edges[0];
            let fin = part.state.store.get(edge).unwrap().fins()[0];
            let pcurve = part.state.store.get(fin).unwrap().pcurve().unwrap().curve();
            (seeds, edge, pcurve)
        };

        let mut edit = session.edit_part(part_id).unwrap();
        let mut transaction = edit.store_mut_for_test().transaction().unwrap();
        transaction
            .assembly()
            .replace_pcurve(
                pcurve,
                Curve2dGeom::Line(
                    Line2d::new(Point2::new(100.0, 100.0), Vec2::new(1.0, 0.0)).unwrap(),
                ),
            )
            .unwrap();
        assert_eq!(
            certify_semantic_edge_incidence(
                transaction.store(),
                edge,
                &seeds,
                Tolerances::default().linear(),
            ),
            Err(PlanarSourceExtractionError::Uncertified(
                PlanarSourceProofFailure::NonPlanarBoundary
            ))
        );
    }

    #[test]
    fn general_simple_convex_polyhedron_is_not_a_block_case_table() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let input = tetrahedron();
        let body = add_body(&mut session, &part_id, |store| {
            assemble_input(store, &input)
        });
        let extracted = extract(&session, &part_id, body, 1).unwrap();

        assert_eq!(extracted.planes().len(), 4);
        assert_eq!(extracted.fragments().len(), 4);
        assert_eq!(extracted.vertices().len(), 4);
        assert_eq!(extracted.edges().len(), 6);
        assert!(
            extracted
                .fragments()
                .iter()
                .all(|face| face.vertices().len() == 3)
        );
        for vertex in extracted.vertices() {
            let planes = vertex.key().planes();
            let defining = planes.map(|plane_id| {
                extracted
                    .planes()
                    .iter()
                    .find(|plane| plane.id() == plane_id)
                    .unwrap()
                    .points()
            });
            for plane_id in planes {
                let plane = extracted
                    .planes()
                    .iter()
                    .find(|plane| plane.id() == plane_id)
                    .unwrap();
                assert_eq!(
                    oriented_plane_triple_intersection_side(defining, plane.points())
                        .unwrap()
                        .sign(),
                    Orientation::Zero
                );
            }
        }
    }

    #[test]
    fn full_valid_coplanar_facet_partition_is_honestly_unsupported() {
        let points = [
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(0.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(0.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(0.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let keys = (0..points.len())
            .map(|index| PlanarVertexKey::new(index as u64 + 1))
            .collect::<Vec<_>>();
        let vertices = keys
            .iter()
            .copied()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
            .collect();
        let rings = [
            [0, 3, 4, 1],
            [1, 4, 5, 2],
            [6, 7, 10, 9],
            [7, 8, 11, 10],
            [0, 1, 7, 6],
            [1, 2, 8, 7],
            [3, 9, 10, 4],
            [4, 10, 11, 5],
            [0, 6, 9, 3],
            [2, 5, 11, 8],
        ];
        let faces = rings
            .into_iter()
            .map(|ring| PlanarSolidFace::new(ring.into_iter().map(|index| keys[index]).collect()))
            .collect();
        let input = PlanarSolidInput::new(vertices, faces);
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = add_body(&mut session, &part_id, |store| {
            assemble_input(store, &input)
        });

        assert_eq!(
            extract(&session, &part_id, body, 0),
            Err(PlanarSourceExtractionError::Unsupported(
                PlanarSourceGap::CoplanarFacetPartition
            ))
        );
    }

    #[test]
    fn extraction_work_limit_has_an_exact_acceptance_boundary() {
        // Independent count for a block: preflight 15, face/fin preparation
        // 30, and the documented conservative exact-phase bound 3,413.
        const REQUIRED: u64 = 3_458;

        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = add_body(&mut session, &part_id, |store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 3.0, 5.0]).unwrap()
        });
        let (denied, denied_usage) =
            extract_with_work_limit(&session, &part_id, body.clone(), REQUIRED - 1);
        let Err(PlanarSourceExtractionError::Topology(error)) = denied else {
            panic!("expected a typed extraction-work limit, got {denied:?}");
        };
        assert_eq!(
            error.limit(),
            Some(LimitSnapshot {
                stage: PLANAR_SOURCE_EXTRACTION_WORK,
                resource: ResourceKind::Work,
                consumed: REQUIRED,
                allowed: REQUIRED - 1,
            })
        );
        assert!(denied_usage.contains(&LimitSnapshot {
            stage: PLANAR_SOURCE_EXTRACTION_WORK,
            resource: ResourceKind::Work,
            consumed: 45,
            allowed: REQUIRED - 1,
        }));

        let (accepted, accepted_usage) =
            extract_with_work_limit(&session, &part_id, body, REQUIRED);
        assert!(accepted.is_ok());
        assert!(accepted_usage.contains(&LimitSnapshot {
            stage: PLANAR_SOURCE_EXTRACTION_WORK,
            resource: ResourceKind::Work,
            consumed: REQUIRED,
            allowed: REQUIRED,
        }));
    }

    #[test]
    fn wrong_part_curved_and_non_solid_inputs_fail_closed() {
        let mut session = Kernel::new().create_session();
        let first_part = session.create_part();
        let second_part = session.create_part();
        let block = add_body(&mut session, &first_part, |store| {
            ktopo::make::block(store, &Frame::world(), [2.0; 3]).unwrap()
        });
        let sphere = add_body(&mut session, &first_part, |store| {
            ktopo::make::sphere(store, &Frame::world(), 1.0).unwrap()
        });
        let sheet = add_body(&mut session, &first_part, |store| {
            ktopo::make::planar_sheet(
                store,
                &Frame::world(),
                &[
                    kgeom::vec::Point2::new(-1.0, -1.0),
                    kgeom::vec::Point2::new(1.0, -1.0),
                    kgeom::vec::Point2::new(1.0, 1.0),
                    kgeom::vec::Point2::new(-1.0, 1.0),
                ],
            )
            .unwrap()
        });

        let wrong_part = session.part(second_part).unwrap();
        let context = OperationContext::new(wrong_part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(
                CheckBudgetProfile::v1_defaults(CheckLevel::Fast)
                    .overlaid(&PlanarSourceExtractionBudgetProfile::v1_defaults()),
            );
        let mut scope = OperationScope::new(&context);
        assert_eq!(
            extract_planar_source_body(&wrong_part, block, 0, &mut scope),
            Err(PlanarSourceExtractionError::WrongPart)
        );
        let sphere_result = extract(&session, &first_part, sphere, 0);
        assert_eq!(
            sphere_result,
            Err(PlanarSourceExtractionError::Unsupported(
                PlanarSourceGap::NonPlanarFace
            ))
        );
        assert_eq!(
            extract(&session, &first_part, sheet, 0),
            Err(PlanarSourceExtractionError::Unsupported(
                PlanarSourceGap::NonSolidBody
            ))
        );
    }
}
