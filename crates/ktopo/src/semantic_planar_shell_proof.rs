//! Semantic ideal-vertex evidence for Boolean-produced planar shells.
//!
//! This stage does not replay rounded B-rep coordinates as exact geometry.
//! Every accepted edge must be a graph-verified Plane/Plane intersection,
//! and every topological vertex must collect exactly three live source Plane
//! identities from its incident edges. The ideal vertex is the intersection
//! of those surfaces, enclosed by the shared `kcore` plane-triple solver.
//! Rounded vertex positions are used only as tolerance-bounded associations
//! with the ideal points.
//!
//! The prepared evidence is deliberately immutable and contains no shell
//! embedding or volume claim. Pairwise facet separation and orientation are
//! separate proof stages; unsupported or ambiguous input fails closed here.

use crate::entity::{EdgeId, FaceId, Sense, ShellId, SurfaceId, VertexId};
use crate::geom::SurfaceGeom;
use crate::semantic_planar_pair_proof::{
    SemanticFacetPairRelation, certify_semantic_facet_pair, semantic_facet_pair_work,
    semantic_signed_volume_interval, semantic_signed_volume_work,
};
#[cfg(test)]
pub(crate) use crate::semantic_planar_region_proof::SEMANTIC_PLANAR_REGION_WORK;
pub(crate) use crate::semantic_planar_region_proof::{
    SemanticPlanarRegionCertification, certify_semantic_planar_region_in_scope,
};
use crate::shell_proof::{ShellCertification, ShellEmbedding, ShellOrientation};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::plane_triple::enclose_plane_triple_intersection;
use kcore::predicates::{
    Orientation, OrientedPlanePoints, oriented_plane_triple_intersection_side,
};
/// Cumulative work for semantic plane binding and ideal-vertex preparation.
pub(crate) const SEMANTIC_PLANAR_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.semantic-planar-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid semantic planar shell work stage"),
    };

const DEFAULT_SEMANTIC_PLANAR_SHELL_WORK: u64 = 1_048_576;

/// Version-1 deterministic budget for semantic planar shell preparation.
pub(crate) fn semantic_planar_shell_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        SEMANTIC_PLANAR_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_SEMANTIC_PLANAR_SHELL_WORK,
    )])
    .expect("built-in semantic planar shell proof budget is valid")
    .overlaid(&crate::semantic_planar_region_proof::semantic_planar_region_proof_budget())
}

/// Certify embedding and orientation of one bound semantic planar shell.
///
/// Preparation establishes the ideal source-plane complex. Every facet pair
/// must then be strictly separated or confined to a shared topological
/// feature, and the complete interval volume must exclude zero before an
/// orientation claim is made.
pub(crate) fn certify_semantic_planar_shell_in_scope(
    store: &Store,
    shell_id: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ShellCertification> {
    let SemanticPlanarShellPreparation::Certified(evidence) =
        prepare_semantic_planar_shell_in_scope(store, shell_id, scope)?
    else {
        return Ok(indeterminate_shell());
    };

    certify_prepared_semantic_planar_shell_in_scope(&evidence, scope)
}

pub(crate) fn certify_prepared_semantic_planar_shell_in_scope(
    evidence: &SemanticPlanarShellEvidence,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ShellCertification> {
    let Some(pair_and_volume_work) = semantic_pair_and_volume_work(evidence) else {
        return Ok(indeterminate_shell());
    };
    charge(scope, pair_and_volume_work)?;
    for left in 0..evidence.facets().len() {
        for right in left + 1..evidence.facets().len() {
            if certify_semantic_facet_pair(
                evidence,
                &evidence.facets()[left],
                &evidence.facets()[right],
            ) == SemanticFacetPairRelation::Ambiguous
            {
                return Ok(indeterminate_shell());
            }
        }
    }

    let orientation = if evidence.sense_mismatch() {
        ShellOrientation::Invalid
    } else {
        semantic_signed_volume_interval(evidence).map_or(
            ShellOrientation::Indeterminate,
            |volume| {
                if volume.lo() > 0.0 {
                    ShellOrientation::Positive
                } else if volume.hi() < 0.0 {
                    ShellOrientation::Negative
                } else {
                    ShellOrientation::Indeterminate
                }
            },
        )
    };
    Ok(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation,
    })
}

fn semantic_pair_and_volume_work(evidence: &SemanticPlanarShellEvidence) -> Option<u64> {
    let mut work = semantic_signed_volume_work(evidence)?;
    for left in 0..evidence.facets().len() {
        for right in left + 1..evidence.facets().len() {
            work = work.checked_add(semantic_facet_pair_work(
                &evidence.facets()[left],
                &evidence.facets()[right],
            )?)?;
        }
    }
    Some(work)
}

fn indeterminate_shell() -> ShellCertification {
    ShellCertification {
        embedding: ShellEmbedding::Indeterminate,
        orientation: ShellOrientation::Indeterminate,
    }
}

/// Fail-closed result of semantic planar shell preparation.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SemanticPlanarShellPreparation {
    /// Complete ideal-vertex and strict facet evidence is available.
    Certified(SemanticPlanarShellEvidence),
    /// The representation or a numeric proof obligation was not certified.
    Indeterminate,
}

/// Immutable prepared input for pairwise embedding and volume proof stages.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SemanticPlanarShellEvidence {
    facets: Vec<SemanticFacetEvidence>,
    vertices: Vec<SemanticVertexEvidence>,
    plane_witnesses: Vec<(SurfaceId, OrientedPlanePoints)>,
    sense_mismatch: bool,
}

impl SemanticPlanarShellEvidence {
    /// Prepared facets in stable shell order.
    pub(crate) fn facets(&self) -> &[SemanticFacetEvidence] {
        &self.facets
    }

    /// Ideal vertices in deterministic first-incidence order.
    pub(crate) fn vertices(&self) -> &[SemanticVertexEvidence] {
        &self.vertices
    }

    /// Look up an ideal vertex by topology identity.
    pub(crate) fn vertex(&self, vertex: VertexId) -> Option<SemanticVertexEvidence> {
        self.vertices
            .iter()
            .copied()
            .find(|candidate| candidate.vertex == vertex)
    }

    /// Ordered-point witness defining one retained live Plane surface.
    pub(crate) fn plane_witness(&self, surface: SurfaceId) -> Option<OrientedPlanePoints> {
        self.plane_witnesses
            .iter()
            .find_map(|(candidate, witness)| (*candidate == surface).then_some(*witness))
    }

    /// Whether a face sense disagrees with its certified ideal loop winding.
    pub(crate) const fn sense_mismatch(&self) -> bool {
        self.sense_mismatch
    }
}

/// One strictly convex ideal planar facet.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SemanticFacetEvidence {
    face: FaceId,
    support: SurfaceId,
    vertices: Vec<VertexId>,
    edges: Vec<SemanticEdgeEvidence>,
    normal: [Interval; 3],
}

impl SemanticFacetEvidence {
    pub(crate) const fn face(&self) -> FaceId {
        self.face
    }

    pub(crate) const fn support(&self) -> SurfaceId {
        self.support
    }

    pub(crate) fn vertices(&self) -> &[VertexId] {
        &self.vertices
    }

    pub(crate) fn edges(&self) -> &[SemanticEdgeEvidence] {
        &self.edges
    }

    pub(crate) const fn normal(&self) -> [Interval; 3] {
        self.normal
    }
}

/// One directed facet use of a graph-verified Plane/Plane edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SemanticEdgeEvidence {
    edge: EdgeId,
    endpoints: [VertexId; 2],
    source_surfaces: [SurfaceId; 2],
}

impl SemanticEdgeEvidence {
    pub(crate) const fn edge(self) -> EdgeId {
        self.edge
    }

    pub(crate) const fn endpoints(self) -> [VertexId; 2] {
        self.endpoints
    }

    pub(crate) const fn source_surfaces(self) -> [SurfaceId; 2] {
        self.source_surfaces
    }
}

/// Certified association between one topological vertex and an ideal triple.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SemanticVertexEvidence {
    vertex: VertexId,
    surfaces: [SurfaceId; 3],
    coordinates: [Interval; 3],
}

impl SemanticVertexEvidence {
    pub(crate) const fn vertex(self) -> VertexId {
        self.vertex
    }

    pub(crate) const fn surfaces(self) -> [SurfaceId; 3] {
        self.surfaces
    }

    pub(crate) const fn coordinates(self) -> [Interval; 3] {
        self.coordinates
    }
}

#[derive(Debug)]
struct RawFacet {
    face: FaceId,
    support: SurfaceId,
    sense: Sense,
    vertices: Vec<VertexId>,
    edges: Vec<SemanticEdgeEvidence>,
}

type DirectedEdgeUse = (usize, [VertexId; 2]);
type EdgeUses = (EdgeId, Vec<DirectedEdgeUse>);

/// Prepare proof-bearing ideal geometry for one connected planar shell.
///
/// Certification requires one loop per face; verified PlaneLine descriptors
/// whose live source pair contains the owning face support; exactly three
/// source planes at every vertex; a conditioned, size-box-contained triple
/// enclosure no wider than the session linear resolution; a rounded topology
/// point within that resolution of the enclosure; and strict exact
/// source-plane halfspace evidence for every facet edge.
pub(crate) fn prepare_semantic_planar_shell_in_scope(
    store: &Store,
    shell_id: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<SemanticPlanarShellPreparation> {
    scope.ledger().require_limit(
        SEMANTIC_PLANAR_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let Some(raw) = prepare_raw_facets(store, shell_id, scope)? else {
        return Ok(SemanticPlanarShellPreparation::Indeterminate);
    };
    if !closed_connected_manifold(&raw) {
        return Ok(SemanticPlanarShellPreparation::Indeterminate);
    }

    let mut incident_planes: Vec<(VertexId, Vec<SurfaceId>)> = Vec::new();
    for facet in &raw {
        for edge in &facet.edges {
            for vertex in edge.endpoints {
                let planes = if let Some((_, planes)) = incident_planes
                    .iter_mut()
                    .find(|(candidate, _)| *candidate == vertex)
                {
                    planes
                } else {
                    incident_planes.push((vertex, Vec::new()));
                    &mut incident_planes.last_mut().expect("just inserted").1
                };
                for surface in edge.source_surfaces {
                    if !planes.contains(&surface) {
                        planes.push(surface);
                    }
                }
            }
        }
    }

    let mut witnesses: Vec<(SurfaceId, OrientedPlanePoints)> = Vec::new();
    for (_, planes) in &incident_planes {
        for &surface in planes {
            if !witnesses.iter().any(|(candidate, _)| *candidate == surface) {
                let Some(witness) = plane_witness(store, surface)? else {
                    return Ok(SemanticPlanarShellPreparation::Indeterminate);
                };
                witnesses.push((surface, witness));
            }
        }
    }

    let precision = scope.context().session().precision();
    let numerical = scope.context().session().numerical();
    let linear = precision.linear_resolution();
    let mut vertices = Vec::with_capacity(incident_planes.len());
    for &(vertex, ref planes) in &incident_planes {
        charge(scope, 16)?;
        let Ok(mut surfaces): core::result::Result<[SurfaceId; 3], _> =
            planes.as_slice().try_into()
        else {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        };
        surfaces.sort_by(|first, second| {
            compare_witnesses(witness(&witnesses, *first), witness(&witnesses, *second))
        });
        let defining = surfaces.map(|surface| witness(&witnesses, surface));
        let Ok(enclosure) = enclose_plane_triple_intersection(
            defining,
            precision.size_box_half(),
            f64::MIN_POSITIVE,
        ) else {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        };
        if !numerical.reciprocal_condition_is_usable(enclosure.reciprocal_condition_lower_bound()) {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        }
        let coordinates = enclosure.coordinates();
        if interval_vector_width(coordinates) > linear
            || point_enclosure_distance_upper(
                store.vertex_position(vertex)?.to_array(),
                coordinates,
            ) > linear
        {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        }
        vertices.push(SemanticVertexEvidence {
            vertex,
            surfaces,
            coordinates,
        });
    }
    let mut facets = Vec::with_capacity(raw.len());
    let mut sense_mismatch = false;
    for facet in raw {
        if facet.vertices.iter().any(|vertex| {
            !ideal_vertex(&vertices, *vertex)
                .surfaces
                .contains(&facet.support)
        }) {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        }
        let normal = plane_normal(witness(&witnesses, facet.support));
        let Some(loop_sense) = ideal_loop_sense(&facet.vertices, &vertices, normal) else {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        };
        sense_mismatch |= loop_sense != facet.sense;
        if !strict_facet_halfspaces(&facet, &vertices, &witnesses, scope)? {
            return Ok(SemanticPlanarShellPreparation::Indeterminate);
        }
        facets.push(SemanticFacetEvidence {
            face: facet.face,
            support: facet.support,
            vertices: facet.vertices,
            edges: facet.edges,
            normal,
        });
    }

    Ok(SemanticPlanarShellPreparation::Certified(
        SemanticPlanarShellEvidence {
            facets,
            vertices,
            plane_witnesses: witnesses,
            sense_mismatch,
        },
    ))
}

fn prepare_raw_facets(
    store: &Store,
    shell_id: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<Vec<RawFacet>>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 4 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let linear = scope.context().session().precision().linear_resolution();
    let mut facets = Vec::with_capacity(shell.faces.len());
    for &face_id in &shell.faces {
        charge(scope, 1)?;
        let face = store.get(face_id)?;
        if face.loops.len() != 1 || face.tolerance.is_some() {
            return Ok(None);
        }
        if !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
            return Ok(None);
        }
        let loop_ = store.get(face.loops[0])?;
        if loop_.fins.len() < 3 {
            return Ok(None);
        }
        let mut vertices = Vec::with_capacity(loop_.fins.len());
        let mut edges = Vec::with_capacity(loop_.fins.len());
        for &fin_id in &loop_.fins {
            charge(scope, 4)?;
            let fin = store.get(fin_id)?;
            let edge = store.get(fin.edge)?;
            let (Some(curve_id), Some(bounds), [Some(first), Some(second)]) =
                (edge.curve, edge.bounds, edge.vertices)
            else {
                return Ok(None);
            };
            if first == second || edge.fins.len() != 2 || edge.tolerance.is_some() {
                return Ok(None);
            }
            let Some(descriptor) = store.get(curve_id)?.as_intersection() else {
                return Ok(None);
            };
            let Some(certificate) = descriptor.certificate().as_plane_line() else {
                return Ok(None);
            };
            let sources = descriptor.source_surfaces();
            if sources[0] == sources[1]
                || !sources.contains(&face.surface)
                || certificate.carrier_range().lo != bounds.0
                || certificate.carrier_range().hi != bounds.1
                || certificate.tolerance() > linear
                || certificate
                    .residual_bounds()
                    .into_iter()
                    .any(|bound| !bound.is_finite() || bound > linear)
            {
                return Ok(None);
            }
            for source in sources {
                if !matches!(store.get(source)?, SurfaceGeom::Plane(_)) {
                    return Ok(None);
                }
            }
            let Some(source_index) = sources.iter().position(|source| *source == face.surface)
            else {
                return Ok(None);
            };
            let Some(pcurve) = fin.pcurve else {
                return Ok(None);
            };
            let graph_map = certificate.parameter_maps()[source_index];
            let topology_map = pcurve.edge_to_pcurve();
            if descriptor.pcurves()[source_index] != pcurve.curve()
                || pcurve.range().lo != bounds.0
                || pcurve.range().hi != bounds.1
                || topology_map.scale() != graph_map.scale()
                || topology_map.offset() != graph_map.offset()
                || !pcurve.chart().is_identity()
                || pcurve.closure_winding().is_some()
                || pcurve.seam().is_some()
            {
                return Ok(None);
            }
            let (Some(tail), Some(head)) = (store.fin_tail(fin_id)?, store.fin_head(fin_id)?)
            else {
                return Ok(None);
            };
            if tail == head || ![first, second].contains(&tail) || ![first, second].contains(&head)
            {
                return Ok(None);
            }
            let mut source_surfaces = sources;
            source_surfaces.sort_by(|first, second| {
                let first = plane_witness(store, *first).ok().flatten();
                let second = plane_witness(store, *second).ok().flatten();
                match (first, second) {
                    (Some(first), Some(second)) => compare_witnesses(first, second),
                    _ => core::cmp::Ordering::Equal,
                }
            });
            vertices.push(tail);
            edges.push(SemanticEdgeEvidence {
                edge: fin.edge,
                endpoints: [tail, head],
                source_surfaces,
            });
        }
        if vertices
            .iter()
            .enumerate()
            .any(|(index, vertex)| vertices[..index].contains(vertex))
        {
            return Ok(None);
        }
        facets.push(RawFacet {
            face: face_id,
            support: face.surface,
            sense: face.sense,
            vertices,
            edges,
        });
    }
    Ok(Some(facets))
}

fn closed_connected_manifold(facets: &[RawFacet]) -> bool {
    let mut uses: Vec<EdgeUses> = Vec::new();
    for (face_index, facet) in facets.iter().enumerate() {
        for edge in &facet.edges {
            if let Some((_, edge_uses)) = uses.iter_mut().find(|(id, _)| *id == edge.edge) {
                edge_uses.push((face_index, edge.endpoints));
            } else {
                uses.push((edge.edge, vec![(face_index, edge.endpoints)]));
            }
        }
    }
    if uses.iter().any(|(_, uses)| {
        uses.len() != 2
            || uses[0].0 == uses[1].0
            || uses[0].1 != [uses[1].1[1], uses[1].1[0]]
            || uses[0].1 == uses[1].1
    }) {
        return false;
    }
    let mut seen = vec![false; facets.len()];
    let mut pending = vec![0_usize];
    while let Some(face) = pending.pop() {
        if core::mem::replace(&mut seen[face], true) {
            continue;
        }
        for (_, uses) in uses
            .iter()
            .filter(|(_, uses)| uses.iter().any(|use_| use_.0 == face))
        {
            pending.extend(uses.iter().map(|use_| use_.0));
        }
    }
    seen.into_iter().all(|value| value) && vertex_links_are_cycles(facets)
}

fn vertex_links_are_cycles(facets: &[RawFacet]) -> bool {
    let mut vertices = Vec::new();
    for facet in facets {
        for &vertex in &facet.vertices {
            if !vertices.contains(&vertex) {
                vertices.push(vertex);
            }
        }
    }
    for vertex in vertices {
        let mut links: Vec<(VertexId, Vec<VertexId>)> = Vec::new();
        for facet in facets {
            let Some(index) = facet
                .vertices
                .iter()
                .position(|candidate| *candidate == vertex)
            else {
                continue;
            };
            let previous =
                facet.vertices[(index + facet.vertices.len() - 1) % facet.vertices.len()];
            let next = facet.vertices[(index + 1) % facet.vertices.len()];
            if previous == next
                || !insert_vertex_link(&mut links, previous, next)
                || !insert_vertex_link(&mut links, next, previous)
            {
                return false;
            }
        }
        if links.len() < 3 || links.iter().any(|(_, neighbors)| neighbors.len() != 2) {
            return false;
        }
        let mut seen = Vec::new();
        let mut pending = vec![links[0].0];
        while let Some(current) = pending.pop() {
            if seen.contains(&current) {
                continue;
            }
            seen.push(current);
            let Some((_, neighbors)) = links.iter().find(|(candidate, _)| *candidate == current)
            else {
                return false;
            };
            pending.extend(neighbors.iter().copied());
        }
        if seen.len() != links.len() {
            return false;
        }
    }
    true
}

fn insert_vertex_link(
    links: &mut Vec<(VertexId, Vec<VertexId>)>,
    vertex: VertexId,
    neighbor: VertexId,
) -> bool {
    if let Some((_, neighbors)) = links.iter_mut().find(|(candidate, _)| *candidate == vertex) {
        if neighbors.contains(&neighbor) {
            return false;
        }
        neighbors.push(neighbor);
    } else {
        links.push((vertex, vec![neighbor]));
    }
    true
}

fn plane_witness(store: &Store, surface: SurfaceId) -> Result<Option<OrientedPlanePoints>> {
    let SurfaceGeom::Plane(plane) = store.get(surface)? else {
        return Ok(None);
    };
    let frame = plane.frame();
    Ok(Some([
        frame.origin().to_array(),
        frame.point_at(1.0, 0.0, 0.0).to_array(),
        frame.point_at(0.0, 1.0, 0.0).to_array(),
    ]))
}

fn plane_normal(witness: OrientedPlanePoints) -> [Interval; 3] {
    let first = point_box(witness[1]);
    let second = point_box(witness[2]);
    let origin = point_box(witness[0]);
    cross(subtract(first, origin), subtract(second, origin))
}

fn witness(
    witnesses: &[(SurfaceId, OrientedPlanePoints)],
    surface: SurfaceId,
) -> OrientedPlanePoints {
    witnesses
        .iter()
        .find_map(|(candidate, witness)| (*candidate == surface).then_some(*witness))
        .expect("prepared surface witness")
}

fn compare_witnesses(
    first: OrientedPlanePoints,
    second: OrientedPlanePoints,
) -> core::cmp::Ordering {
    first
        .iter()
        .flatten()
        .zip(second.iter().flatten())
        .find_map(|(first, second)| {
            let ordering = first.total_cmp(second);
            (ordering != core::cmp::Ordering::Equal).then_some(ordering)
        })
        .unwrap_or(core::cmp::Ordering::Equal)
}

fn ideal_vertex(vertices: &[SemanticVertexEvidence], vertex: VertexId) -> SemanticVertexEvidence {
    vertices
        .iter()
        .copied()
        .find(|candidate| candidate.vertex == vertex)
        .expect("prepared ideal vertex")
}

fn strict_facet_halfspaces(
    facet: &RawFacet,
    vertices: &[SemanticVertexEvidence],
    witnesses: &[(SurfaceId, OrientedPlanePoints)],
    scope: &mut OperationScope<'_, '_>,
) -> Result<bool> {
    for edge in &facet.edges {
        let Some(carrier) = edge
            .source_surfaces
            .into_iter()
            .find(|surface| *surface != facet.support)
        else {
            return Ok(false);
        };
        let mut expected = None;
        for &vertex_id in &facet.vertices {
            charge(scope, 1)?;
            let vertex = ideal_vertex(vertices, vertex_id);
            if edge.endpoints.contains(&vertex_id) {
                if !vertex.surfaces.contains(&carrier) {
                    return Ok(false);
                }
                continue;
            }
            let defining = vertex.surfaces.map(|surface| witness(witnesses, surface));
            let Some(side) =
                oriented_plane_triple_intersection_side(defining, witness(witnesses, carrier))
                    .map(|side| side.sign())
            else {
                return Ok(false);
            };
            if side == Orientation::Zero || expected.is_some_and(|candidate| candidate != side) {
                return Ok(false);
            } else {
                expected = Some(side);
            }
        }
        if expected.is_none() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn ideal_loop_sense(
    ring: &[VertexId],
    vertices: &[SemanticVertexEvidence],
    normal: [Interval; 3],
) -> Option<Sense> {
    let mut expected = None;
    for index in 0..ring.len() {
        let a = ideal_vertex(vertices, ring[index]).coordinates;
        let b = ideal_vertex(vertices, ring[(index + 1) % ring.len()]).coordinates;
        let c = ideal_vertex(vertices, ring[(index + 2) % ring.len()]).coordinates;
        let turn = dot(cross(subtract(b, a), subtract(c, b)), normal).sign()?;
        if turn == 0 || expected.is_some_and(|value| value != turn) {
            return None;
        }
        expected = Some(turn);
    }
    match expected? {
        1 => Some(Sense::Forward),
        -1 => Some(Sense::Reversed),
        _ => None,
    }
}

fn point_enclosure_distance_upper(point: [f64; 3], enclosure: [Interval; 3]) -> f64 {
    let squared = enclosure
        .into_iter()
        .zip(point)
        .fold(Interval::point(0.0), |sum, (coordinate, value)| {
            sum + (coordinate - Interval::point(value)).square()
        });
    squared.sqrt().map_or(f64::INFINITY, Interval::hi)
}

fn interval_vector_width(vector: [Interval; 3]) -> f64 {
    vector
        .into_iter()
        .fold(Interval::point(0.0), |sum, coordinate| {
            sum + Interval::point(coordinate.width()).square()
        })
        .sqrt()
        .map_or(f64::INFINITY, Interval::hi)
}

fn point_box(point: [f64; 3]) -> [Interval; 3] {
    point.map(Interval::point)
}

fn subtract(left: [Interval; 3], right: [Interval; 3]) -> [Interval; 3] {
    core::array::from_fn(|axis| left[axis] - right[axis])
}

fn cross(left: [Interval; 3], right: [Interval; 3]) -> [Interval; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn dot(left: [Interval; 3], right: [Interval; 3]) -> Interval {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SEMANTIC_PLANAR_SHELL_WORK, amount)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::RegionId;
    use crate::make::block;
    use crate::planar::{
        PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarSolidOutput,
        PlanarSolidVertex, PlanarVertexKey,
    };
    use kcore::operation::{
        ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
        SessionPrecision,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    const KEYS: [PlanarVertexKey; 8] = [
        PlanarVertexKey::new(1),
        PlanarVertexKey::new(2),
        PlanarVertexKey::new(3),
        PlanarVertexKey::new(4),
        PlanarVertexKey::new(5),
        PlanarVertexKey::new(6),
        PlanarVertexKey::new(7),
        PlanarVertexKey::new(8),
    ];

    fn rings() -> [[usize; 4]; 6] {
        [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
    }

    fn bound_box(store: &mut Store) -> PlanarSolidInput {
        bound_box_at(store, [0.0; 3], [1.0, 1.5, 2.0])
    }

    fn bound_box_at(
        store: &mut Store,
        center: [f64; 3],
        half_extents: [f64; 3],
    ) -> PlanarSolidInput {
        let frame = Frame::new(
            Point3::new(center[0], center[1], center[2]),
            kgeom::vec::Vec3::new(0.0, 0.0, 1.0),
            kgeom::vec::Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let source = block(
            store,
            &frame,
            half_extents.map(|half_extent| 2.0 * half_extent),
        )
        .unwrap();
        let surfaces: Vec<_> = store
            .faces_of_body(source)
            .unwrap()
            .into_iter()
            .map(|face| store.get(face).unwrap().surface)
            .collect();
        let [cx, cy, cz] = center;
        let [hx, hy, hz] = half_extents;
        let points = [
            Point3::new(cx - hx, cy - hy, cz - hz),
            Point3::new(cx + hx, cy - hy, cz - hz),
            Point3::new(cx - hx, cy + hy, cz - hz),
            Point3::new(cx + hx, cy + hy, cz - hz),
            Point3::new(cx - hx, cy - hy, cz + hz),
            Point3::new(cx + hx, cy - hy, cz + hz),
            Point3::new(cx - hx, cy + hy, cz + hz),
            Point3::new(cx + hx, cy + hy, cz + hz),
        ];
        let vertices = points
            .into_iter()
            .enumerate()
            .map(|(index, point)| PlanarSolidVertex::new(KEYS[index], point))
            .collect();
        let all_rings = rings();
        let faces = all_rings
            .iter()
            .enumerate()
            .map(|(face_index, ring)| {
                let carriers = (0..ring.len())
                    .map(|edge_index| {
                        let a = ring[edge_index];
                        let b = ring[(edge_index + 1) % ring.len()];
                        let other = all_rings
                            .iter()
                            .enumerate()
                            .find(|(candidate_index, candidate)| {
                                *candidate_index != face_index
                                    && (0..candidate.len()).any(|index| {
                                        let c = candidate[index];
                                        let d = candidate[(index + 1) % candidate.len()];
                                        a == c && b == d || a == d && b == c
                                    })
                            })
                            .unwrap()
                            .0;
                        surfaces[other]
                    })
                    .collect();
                PlanarSolidFace::new(ring.map(|index| KEYS[index]).to_vec())
                    .with_plane_binding(PlanarFacePlaneBinding::new(surfaces[face_index], carriers))
            })
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn context() -> (SessionPolicy, Tolerances) {
        context_with_budget(semantic_planar_shell_proof_budget())
    }

    fn context_with_budget(budget: BudgetPlan) -> (SessionPolicy, Tolerances) {
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        );
        (session, Tolerances::default())
    }

    fn assemble_box(
        store: &mut Store,
        center: [f64; 3],
        half_extents: [f64; 3],
    ) -> PlanarSolidOutput {
        let input = bound_box_at(store, center, half_extents);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        transaction.commit_checked_body(output.body()).unwrap();
        output
    }

    fn reverse_shell(store: &mut Store, output: &PlanarSolidOutput) {
        for &face_id in output.faces() {
            let face = store.get(face_id).unwrap().clone();
            store.get_mut(face_id).unwrap().sense = face.sense.flipped();
            for loop_id in face.loops {
                let mut fins = store.get(loop_id).unwrap().fins.clone();
                fins.reverse();
                for &fin_id in &fins {
                    let sense = store.get(fin_id).unwrap().sense;
                    store.get_mut(fin_id).unwrap().sense = sense.flipped();
                }
                store.get_mut(loop_id).unwrap().fins = fins;
            }
        }
    }

    fn attach_second_shell(
        store: &mut Store,
        outer: &PlanarSolidOutput,
        inner: &PlanarSolidOutput,
    ) -> RegionId {
        let outer_body = store.get(outer.body()).unwrap();
        let outer_region = *outer_body
            .regions()
            .iter()
            .find(|&&region| store.get(region).unwrap().kind() == crate::entity::RegionKind::Solid)
            .unwrap();
        let inner_region = store.get(inner.shell()).unwrap().region;
        store
            .get_mut(inner_region)
            .unwrap()
            .shells
            .retain(|shell| *shell != inner.shell());
        store.get_mut(inner.shell()).unwrap().region = outer_region;
        store
            .get_mut(outer_region)
            .unwrap()
            .shells
            .push(inner.shell());
        let cavity_void = store.add(crate::entity::Region {
            body: outer.body(),
            kind: crate::entity::RegionKind::Void,
            shells: Vec::new(),
        });
        store
            .get_mut(outer.body())
            .unwrap()
            .regions
            .push(cavity_void);
        outer_region
    }

    #[test]
    fn bound_box_prepares_exact_plane_triples_and_strict_facets() {
        let mut store = Store::new();
        let input = bound_box(&mut store);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        let (session, tolerances) = context();
        let operation = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&operation);
        let SemanticPlanarShellPreparation::Certified(evidence) =
            prepare_semantic_planar_shell_in_scope(transaction.store(), output.shell(), &mut scope)
                .unwrap()
        else {
            panic!("bound semantic box was not prepared");
        };
        assert_eq!(evidence.facets().len(), 6);
        assert_eq!(evidence.vertices().len(), 8);
        assert!(!evidence.sense_mismatch());
        assert!(evidence.vertices().iter().all(|vertex| {
            let position = transaction
                .store()
                .vertex_position(vertex.vertex())
                .unwrap()
                .to_array();
            let defining = vertex
                .surfaces()
                .map(|surface| evidence.plane_witness(surface).unwrap());
            vertex
                .coordinates()
                .into_iter()
                .zip(position)
                .all(|(coordinate, expected)| {
                    coordinate.contains(expected) && coordinate.width() <= 1.0e-12
                })
                && vertex.surfaces().into_iter().all(|surface| {
                    oriented_plane_triple_intersection_side(
                        defining,
                        evidence.plane_witness(surface).unwrap(),
                    )
                    .is_some_and(|side| side.sign() == Orientation::Zero)
                })
        }));
        assert!(evidence.facets().iter().all(|facet| {
            facet.vertices().len() == 4
                && facet.edges().len() == 4
                && facet
                    .edges()
                    .iter()
                    .all(|edge| edge.source_surfaces().contains(&facet.support()))
        }));
    }

    #[test]
    fn bound_box_has_certified_semantic_embedding_and_orientation() {
        let mut store = Store::new();
        let input = bound_box(&mut store);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        let (session, tolerances) = context();
        let operation = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&operation);
        assert_eq!(
            certify_semantic_planar_shell_in_scope(
                transaction.store(),
                output.shell(),
                &mut scope,
            )
            .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );
    }

    #[test]
    fn plain_line_substitution_and_far_vertex_fail_closed() {
        let mut store = Store::new();
        let input = bound_box(&mut store);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        let original = transaction.store().clone();
        let edge_id = output.edges()[0].1;
        let curve_id = original.get(edge_id).unwrap().curve.unwrap();
        let line = original
            .get(curve_id)
            .unwrap()
            .as_intersection()
            .unwrap()
            .carrier()
            .as_line()
            .unwrap();
        let mut plain = original.clone();
        let plain_curve = plain
            .insert_curve(crate::geom::CurveGeom::Line(line))
            .unwrap();
        plain.get_mut(edge_id).unwrap().curve = Some(plain_curve);

        let (session, tolerances) = context();
        let operation = OperationContext::new(&session, tolerances).unwrap();
        let mut plain_scope = OperationScope::new(&operation);
        assert_eq!(
            prepare_semantic_planar_shell_in_scope(&plain, output.shell(), &mut plain_scope)
                .unwrap(),
            SemanticPlanarShellPreparation::Indeterminate
        );

        let mut displaced = original;
        let vertex = output.vertices()[0].1;
        let point_id = displaced.get(vertex).unwrap().point;
        displaced.get_mut(point_id).unwrap().x += 1.0e-4;
        let mut displaced_scope = OperationScope::new(&operation);
        assert_eq!(
            prepare_semantic_planar_shell_in_scope(
                &displaced,
                output.shell(),
                &mut displaced_scope
            )
            .unwrap(),
            SemanticPlanarShellPreparation::Indeterminate
        );
    }

    #[test]
    fn face_sense_mismatch_is_retained_as_orientation_evidence() {
        let mut store = Store::new();
        let input = bound_box(&mut store);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        let mut flipped = transaction.store().clone();
        let face = output.faces()[0];
        flipped.get_mut(face).unwrap().sense = flipped.get(face).unwrap().sense.flipped();
        let (session, tolerances) = context();
        let operation = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&operation);
        let SemanticPlanarShellPreparation::Certified(evidence) =
            prepare_semantic_planar_shell_in_scope(&flipped, output.shell(), &mut scope).unwrap()
        else {
            panic!("sense mismatch should not erase embedding inputs");
        };
        assert!(evidence.sense_mismatch());
        let mut certification_scope = OperationScope::new(&operation);
        assert_eq!(
            certify_semantic_planar_shell_in_scope(
                &flipped,
                output.shell(),
                &mut certification_scope,
            )
            .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            }
        );
    }

    #[test]
    fn positive_convex_outer_strictly_contains_negative_cavity() {
        let mut store = Store::new();
        let outer = assemble_box(&mut store, [0.0; 3], [3.0, 2.5, 2.0]);
        let inner = assemble_box(&mut store, [0.25, -0.2, 0.1], [0.75, 0.5, 0.4]);
        reverse_shell(&mut store, &inner);
        let region = attach_second_shell(&mut store, &outer, &inner);
        let (session, tolerances) = context();
        let operation = OperationContext::new(&session, tolerances).unwrap();
        let mut negative_scope = OperationScope::new(&operation);
        assert_eq!(
            certify_semantic_planar_shell_in_scope(&store, inner.shell(), &mut negative_scope)
                .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Negative,
            }
        );
        let mut scope = OperationScope::new(&operation);
        assert_eq!(
            certify_semantic_planar_region_in_scope(&store, region, &mut scope).unwrap(),
            SemanticPlanarRegionCertification::Certified
        );

        let report =
            crate::check::check_body_report(&store, outer.body(), crate::check::CheckLevel::Full)
                .unwrap();
        assert_eq!(
            report.outcome(),
            crate::check::CheckOutcome::Valid,
            "{report:?}"
        );
    }

    #[test]
    fn cavity_full_check_requires_exact_reduced_void_region_ownership() {
        let mut store = Store::new();
        let outer = assemble_box(&mut store, [0.0; 3], [3.0, 2.5, 2.0]);
        let inner = assemble_box(&mut store, [0.25, -0.2, 0.1], [0.75, 0.5, 0.4]);
        reverse_shell(&mut store, &inner);
        let _ = attach_second_shell(&mut store, &outer, &inner);
        let cavity_void = *store.get(outer.body()).unwrap().regions().last().unwrap();

        let assert_region_layout_fault = |candidate: &Store| {
            let report = crate::check::check_body_report(
                candidate,
                outer.body(),
                crate::check::CheckLevel::Full,
            )
            .unwrap();
            assert_eq!(report.outcome(), crate::check::CheckOutcome::Invalid);
            assert!(
                report
                    .faults
                    .iter()
                    .any(|fault| fault.kind == crate::check::FaultKind::RegionShellLayout),
                "{report:?}"
            );
        };

        let mut missing = store.clone();
        missing.get_mut(outer.body()).unwrap().regions.pop();
        assert_region_layout_fault(&missing);

        let mut extra = store.clone();
        let extra_void = extra.add(crate::entity::Region {
            body: outer.body(),
            kind: crate::entity::RegionKind::Void,
            shells: Vec::new(),
        });
        extra
            .get_mut(outer.body())
            .unwrap()
            .regions
            .push(extra_void);
        assert_region_layout_fault(&extra);

        let mut wrong_kind = store.clone();
        wrong_kind.get_mut(cavity_void).unwrap().kind = crate::entity::RegionKind::Solid;
        assert_region_layout_fault(&wrong_kind);

        let mut nonempty = store;
        let void_shell = nonempty.add(crate::entity::Shell {
            region: cavity_void,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        nonempty
            .get_mut(cavity_void)
            .unwrap()
            .shells
            .push(void_shell);
        let report = crate::check::check_body_report(
            &nonempty,
            outer.body(),
            crate::check::CheckLevel::Full,
        )
        .unwrap();
        assert_eq!(report.outcome(), crate::check::CheckOutcome::Invalid);
        assert!(
            report
                .faults
                .iter()
                .any(|fault| fault.kind == crate::check::FaultKind::KindMismatch),
            "{report:?}"
        );
    }

    #[test]
    fn two_shell_region_refuses_same_sign_outside_and_contact() {
        let run = |center, half_extents, reverse| {
            let mut store = Store::new();
            let outer = assemble_box(&mut store, [0.0; 3], [3.0, 2.5, 2.0]);
            let inner = assemble_box(&mut store, center, half_extents);
            if reverse {
                reverse_shell(&mut store, &inner);
            }
            let region = attach_second_shell(&mut store, &outer, &inner);
            let (session, tolerances) = context();
            let operation = OperationContext::new(&session, tolerances).unwrap();
            let mut scope = OperationScope::new(&operation);
            certify_semantic_planar_region_in_scope(&store, region, &mut scope).unwrap()
        };

        assert_eq!(
            run([0.0; 3], [0.75, 0.5, 0.4], false),
            SemanticPlanarRegionCertification::Invalid
        );
        assert_eq!(
            run([5.0, 0.0, 0.0], [0.75, 0.5, 0.4], true),
            SemanticPlanarRegionCertification::Invalid
        );
        assert_eq!(
            run([2.25, 0.0, 0.0], [0.75, 0.5, 0.4], true),
            SemanticPlanarRegionCertification::Indeterminate
        );
    }

    #[test]
    fn semantic_region_work_budget_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let outer = assemble_box(&mut store, [0.0; 3], [3.0, 2.5, 2.0]);
        let inner = assemble_box(&mut store, [0.25, -0.2, 0.1], [0.75, 0.5, 0.4]);
        reverse_shell(&mut store, &inner);
        let region = attach_second_shell(&mut store, &outer, &inner);

        let (default_session, tolerances) = context();
        let default_operation = OperationContext::new(&default_session, tolerances).unwrap();
        let mut default_scope = OperationScope::new(&default_operation);
        assert_eq!(
            certify_semantic_planar_region_in_scope(&store, region, &mut default_scope).unwrap(),
            SemanticPlanarRegionCertification::Certified
        );
        let required = default_scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == SEMANTIC_PLANAR_REGION_WORK)
            .unwrap()
            .consumed;
        assert!(required > 0);

        let budget = |allowed| {
            semantic_planar_shell_proof_budget().overlaid(
                &BudgetPlan::new([LimitSpec::new(
                    SEMANTIC_PLANAR_REGION_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            )
        };
        let (exact_session, tolerances) = context_with_budget(budget(required));
        let exact_operation = OperationContext::new(&exact_session, tolerances).unwrap();
        let mut exact_scope = OperationScope::new(&exact_operation);
        assert_eq!(
            certify_semantic_planar_region_in_scope(&store, region, &mut exact_scope).unwrap(),
            SemanticPlanarRegionCertification::Certified
        );

        let (short_session, tolerances) = context_with_budget(budget(required - 1));
        let short_operation = OperationContext::new(&short_session, tolerances).unwrap();
        let mut short_scope = OperationScope::new(&short_operation);
        let error =
            certify_semantic_planar_region_in_scope(&store, region, &mut short_scope).unwrap_err();
        assert_eq!(
            error.limit().map(|limit| limit.stage),
            Some(SEMANTIC_PLANAR_REGION_WORK)
        );
    }

    #[test]
    fn semantic_work_budget_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let input = bound_box(&mut store);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        let (default_session, tolerances) = context();
        let default_operation = OperationContext::new(&default_session, tolerances).unwrap();
        let mut default_scope = OperationScope::new(&default_operation);
        assert_eq!(
            certify_semantic_planar_shell_in_scope(
                transaction.store(),
                output.shell(),
                &mut default_scope
            )
            .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );
        let required = default_scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == SEMANTIC_PLANAR_SHELL_WORK)
            .unwrap()
            .consumed;

        let budget = |allowed| {
            BudgetPlan::new([LimitSpec::new(
                SEMANTIC_PLANAR_SHELL_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap()
        };
        let (exact_session, tolerances) = context_with_budget(budget(required));
        let exact_operation = OperationContext::new(&exact_session, tolerances).unwrap();
        let mut exact_scope = OperationScope::new(&exact_operation);
        assert_eq!(
            certify_semantic_planar_shell_in_scope(
                transaction.store(),
                output.shell(),
                &mut exact_scope
            )
            .unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );

        let (short_session, tolerances) = context_with_budget(budget(required - 1));
        let short_operation = OperationContext::new(&short_session, tolerances).unwrap();
        let mut short_scope = OperationScope::new(&short_operation);
        let error = certify_semantic_planar_shell_in_scope(
            transaction.store(),
            output.shell(),
            &mut short_scope,
        )
        .unwrap_err();
        assert!(error.limit().is_some());
    }
}
