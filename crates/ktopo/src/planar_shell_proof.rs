//! General exact embedding proof for connected planar polyhedral shells.
//!
//! The supported representation is deliberately geometric rather than tied
//! to a primitive or Boolean layout: one connected, closed vertex-manifold
//! shell whose faces are strictly convex planar polygons bounded by exact line
//! edges.  Facets may form a non-convex solid and a supporting plane may be
//! partitioned into several faces.
//!
//! Global embedding follows from a finite exact proof for every facet pair.
//! Positive-gap disjoint polygons are separated on the complete face/edge
//! axis set of infinitesimally thickened polygonal prisms. Non-coplanar
//! shared edges use their exact supporting-plane intersection; shared
//! vertices use strict exact sidedness when available. Remaining zero-gap
//! separators are accepted only when their exposed segments intersect in the
//! authorized shared topological vertex or edge. Every other contact remains
//! indeterminate. Closed manifold incidence and simple vertex links then make
//! the pairwise-embedded complex an embedded connected 2-manifold.
//!
//! Opposed edge uses give the complex one global orientation.  Its direction
//! is decided by the exact sign of the oriented tetrahedral volume sum.  The
//! declared planar surface senses are also checked against the exact loop
//! normal, so a locally flipped face is reported as invalid rather than being
//! hidden by the global sum.

use crate::entity::{EdgeId, FaceId, Sense, ShellId, VertexId};
use crate::geom::SurfaceGeom;
use crate::incidence::{
    IncidenceCertification, certify_edge_surface_incidence, exact_line_carrier,
};
use crate::loop_proof::{LoopSimplicity, certify_loop_simplicity};
use crate::shell_proof::{ShellCertification, ShellEmbedding, ShellOrientation};
use crate::store::Store;
use kcore::error::Result;
use kcore::expansion;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation, orient2d, orient3d};
use kcore::tolerance::LINEAR_RESOLUTION;

/// Cumulative exact projection work for general planar facet-pair proofs.
pub(crate) const PLANAR_SHELL_PAIR_WORK: StageId =
    match StageId::new("ktopo.check.planar-shell-pair-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid planar shell pair-work stage"),
    };

const DEFAULT_PLANAR_SHELL_PAIR_WORK: u64 = 200_000;

/// Version-1 deterministic budget for general planar shell pair proofs.
pub(crate) fn planar_shell_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        PLANAR_SHELL_PAIR_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_PLANAR_SHELL_PAIR_WORK,
    )])
    .expect("built-in planar shell proof budget is valid")
}

/// Certify a general connected closed shell of convex exact planar facets.
///
/// This is a conservative fallback after the cheaper representation-specific
/// shell proofs.  Unsupported geometry and unrecognized contacts return an
/// indeterminate certificate; operation-policy failures remain typed errors.
pub(crate) fn certify_general_planar_shell_in_scope(
    store: &Store,
    shell_id: ShellId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ShellCertification> {
    scope.ledger().require_limit(
        PLANAR_SHELL_PAIR_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    let Some(shell) = PreparedShell::new(store, shell_id)? else {
        return Ok(indeterminate());
    };

    for left in 0..shell.facets.len() {
        for right in left + 1..shell.facets.len() {
            let first = &shell.facets[left];
            let second = &shell.facets[right];
            let Some(work) = pair_work(first, second) else {
                return Ok(indeterminate());
            };
            scope.ledger_mut().charge(PLANAR_SHELL_PAIR_WORK, work)?;
            if facet_pair_relation(first, second) == PairRelation::Ambiguous {
                return Ok(indeterminate());
            }
        }
    }

    let embedding = ShellEmbedding::Certified;
    let orientation = if shell.sense_mismatch {
        ShellOrientation::Invalid
    } else {
        match exact_signed_volume_sign(&shell.facets) {
            Orientation::Positive => ShellOrientation::Positive,
            Orientation::Negative => ShellOrientation::Invalid,
            Orientation::Zero => ShellOrientation::Indeterminate,
        }
    };
    Ok(ShellCertification {
        embedding,
        orientation,
    })
}

fn indeterminate() -> ShellCertification {
    ShellCertification {
        embedding: ShellEmbedding::Indeterminate,
        orientation: ShellOrientation::Indeterminate,
    }
}

#[derive(Debug)]
struct PreparedShell {
    facets: Vec<Facet>,
    sense_mismatch: bool,
}

impl PreparedShell {
    fn new(store: &Store, shell_id: ShellId) -> Result<Option<Self>> {
        let shell = store.get(shell_id)?;
        if shell.faces.len() < 4 || !shell.edges.is_empty() || shell.vertex.is_some() {
            return Ok(None);
        }
        let mut facets = Vec::with_capacity(shell.faces.len());
        let mut sense_mismatch = false;
        for &face in &shell.faces {
            let Some(facet) = Facet::new(store, face)? else {
                return Ok(None);
            };
            sense_mismatch |= !facet.sense_matches_loop;
            facets.push(facet);
        }
        if !validate_closed_connected_manifold(&facets) {
            return Ok(None);
        }
        Ok(Some(Self {
            facets,
            sense_mismatch,
        }))
    }
}

#[derive(Debug)]
struct Facet {
    face: FaceId,
    vertices: Vec<FacetVertex>,
    edges: Vec<FacetEdge>,
    normal: ExactVec3,
    sense_matches_loop: bool,
}

impl Facet {
    fn new(store: &Store, face_id: FaceId) -> Result<Option<Self>> {
        let face = store.get(face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(None);
        };
        if face.loops.len() != 1
            || certify_loop_simplicity(store, face.loops[0])? != LoopSimplicity::Certified
        {
            return Ok(None);
        }
        let loop_ = store.get(face.loops[0])?;
        if loop_.fins.len() < 3 {
            return Ok(None);
        }

        let mut vertices = Vec::with_capacity(loop_.fins.len());
        let mut edges = Vec::with_capacity(loop_.fins.len());
        for &fin_id in &loop_.fins {
            let fin = store.get(fin_id)?;
            let edge = store.get(fin.edge)?;
            let Some(curve) = edge.curve else {
                return Ok(None);
            };
            if edge.tolerance.is_some()
                || edge.bounds.is_none()
                || exact_line_carrier(store.get(curve)?).is_none()
                || certify_edge_surface_incidence(store, fin.edge, face.surface, LINEAR_RESOLUTION)?
                    != IncidenceCertification::Certified
            {
                return Ok(None);
            }
            let Some(vertex) = store.fin_tail(fin_id)? else {
                return Ok(None);
            };
            if vertices
                .iter()
                .any(|candidate: &FacetVertex| candidate.id == vertex)
            {
                return Ok(None);
            }
            vertices.push(FacetVertex {
                id: vertex,
                point: store.vertex_position(vertex)?.to_array(),
            });
            edges.push(FacetEdge {
                id: fin.edge,
                from: vertex,
                to: vertex,
                direction: ExactVec3::zero(),
            });
        }
        for index in 0..vertices.len() {
            let next = (index + 1) % vertices.len();
            edges[index].to = vertices[next].id;
            edges[index].direction =
                ExactVec3::difference(vertices[next].point, vertices[index].point);
        }

        let witness = [vertices[0].point, vertices[1].point, vertices[2].point];
        if vertices[3..].iter().any(|vertex| {
            orient3d(witness[0], witness[1], witness[2], vertex.point) != Orientation::Zero
        }) {
            return Ok(None);
        }
        let normal = ExactVec3::difference(witness[1], witness[0])
            .cross(&ExactVec3::difference(witness[2], witness[0]));
        if normal.is_zero() {
            return Ok(None);
        }
        let Some(dropped_axis) = nondegenerate_projection(&vertices) else {
            return Ok(None);
        };
        if !strictly_convex_projected(&vertices, dropped_axis) {
            return Ok(None);
        }

        let surface_normal = ExactVec3::from_scalars(plane.frame().z().to_array());
        let alignment = normal.dot_exact(&surface_normal);
        let sense_matches_loop = match exact_sign(&alignment) {
            Orientation::Positive => face.sense == Sense::Forward,
            Orientation::Negative => face.sense == Sense::Reversed,
            Orientation::Zero => false,
        };
        Ok(Some(Self {
            face: face_id,
            vertices,
            edges,
            normal,
            sense_matches_loop,
        }))
    }
}

#[derive(Debug, Clone, Copy)]
struct FacetVertex {
    id: VertexId,
    point: [f64; 3],
}

#[derive(Debug)]
struct FacetEdge {
    id: EdgeId,
    from: VertexId,
    to: VertexId,
    direction: ExactVec3,
}

fn nondegenerate_projection(vertices: &[FacetVertex]) -> Option<usize> {
    (0..3).find(|&dropped| {
        orient2d(
            project(vertices[0].point, dropped),
            project(vertices[1].point, dropped),
            project(vertices[2].point, dropped),
        ) != Orientation::Zero
    })
}

fn strictly_convex_projected(vertices: &[FacetVertex], dropped: usize) -> bool {
    let mut expected = None;
    for index in 0..vertices.len() {
        let turn = orient2d(
            project(vertices[index].point, dropped),
            project(vertices[(index + 1) % vertices.len()].point, dropped),
            project(vertices[(index + 2) % vertices.len()].point, dropped),
        );
        if turn == Orientation::Zero || expected.is_some_and(|value| value != turn) {
            return false;
        }
        expected = Some(turn);
    }
    true
}

fn project(point: [f64; 3], dropped: usize) -> [f64; 2] {
    match dropped {
        0 => [point[1], point[2]],
        1 => [point[0], point[2]],
        _ => [point[0], point[1]],
    }
}

#[derive(Debug, Clone, Copy)]
struct EdgeUse {
    edge: EdgeId,
    face: FaceId,
    from: VertexId,
    to: VertexId,
}

fn validate_closed_connected_manifold(facets: &[Facet]) -> bool {
    let mut uses = Vec::new();
    for facet in facets {
        for edge in &facet.edges {
            uses.push(EdgeUse {
                edge: edge.id,
                face: facet.face,
                from: edge.from,
                to: edge.to,
            });
        }
    }
    for index in 0..uses.len() {
        if uses[..index]
            .iter()
            .any(|use_| use_.edge == uses[index].edge)
        {
            continue;
        }
        let matching: Vec<_> = uses
            .iter()
            .copied()
            .filter(|use_| use_.edge == uses[index].edge)
            .collect();
        if matching.len() != 2
            || matching[0].face == matching[1].face
            || matching[0].from != matching[1].to
            || matching[0].to != matching[1].from
        {
            return false;
        }
    }
    face_graph_connected(facets, &uses) && vertex_links_are_cycles(facets)
}

fn face_graph_connected(facets: &[Facet], uses: &[EdgeUse]) -> bool {
    let mut seen = vec![false; facets.len()];
    let mut pending = vec![0_usize];
    while let Some(index) = pending.pop() {
        if core::mem::replace(&mut seen[index], true) {
            continue;
        }
        let face = facets[index].face;
        for use_ in uses.iter().filter(|use_| use_.face == face) {
            for neighbor in uses
                .iter()
                .filter(|candidate| candidate.edge == use_.edge && candidate.face != face)
            {
                if let Some(position) = facets.iter().position(|facet| facet.face == neighbor.face)
                {
                    pending.push(position);
                }
            }
        }
    }
    seen.into_iter().all(|value| value)
}

fn vertex_links_are_cycles(facets: &[Facet]) -> bool {
    let mut vertices = Vec::new();
    for facet in facets {
        for vertex in &facet.vertices {
            if !vertices.contains(&vertex.id) {
                vertices.push(vertex.id);
            }
        }
    }
    for vertex in vertices {
        let mut links: Vec<(VertexId, Vec<VertexId>)> = Vec::new();
        for facet in facets {
            let Some(index) = facet
                .vertices
                .iter()
                .position(|candidate| candidate.id == vertex)
            else {
                continue;
            };
            let previous =
                facet.vertices[(index + facet.vertices.len() - 1) % facet.vertices.len()].id;
            let next = facet.vertices[(index + 1) % facet.vertices.len()].id;
            if previous == next
                || !insert_link(&mut links, previous, next)
                || !insert_link(&mut links, next, previous)
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

fn insert_link(
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

fn pair_work(left: &Facet, right: &Facet) -> Option<u64> {
    let left_edges = u64::try_from(left.edges.len()).ok()?;
    let right_edges = u64::try_from(right.edges.len()).ok()?;
    let vertices = u64::try_from(left.vertices.len())
        .ok()?
        .checked_add(u64::try_from(right.vertices.len()).ok()?)?;
    let axes = 3_u64
        .checked_add(left_edges.checked_mul(2)?)?
        .checked_add(right_edges.checked_mul(2)?)?
        .checked_add(left_edges.checked_mul(right_edges)?)?;
    axes.checked_mul(vertices)?.checked_add(1)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PairRelation {
    Disjoint,
    AuthorizedContact,
    Ambiguous,
}

fn facet_pair_relation(left: &Facet, right: &Facet) -> PairRelation {
    let common_vertices: Vec<_> = left
        .vertices
        .iter()
        .filter(|vertex| {
            right
                .vertices
                .iter()
                .any(|candidate| candidate.id == vertex.id)
        })
        .map(|vertex| vertex.id)
        .collect();
    let common_edges: Vec<_> = left
        .edges
        .iter()
        .filter(|edge| right.edges.iter().any(|candidate| candidate.id == edge.id))
        .collect();
    if common_edges.len() > 1 || common_vertices.len() > 2 {
        return PairRelation::Ambiguous;
    }
    if let [edge] = common_edges.as_slice() {
        if common_vertices.len() != 2
            || !common_vertices.contains(&edge.from)
            || !common_vertices.contains(&edge.to)
        {
            return PairRelation::Ambiguous;
        }
        if planes_are_distinct(left, right) {
            // The two supporting planes meet in the line of the common
            // edge. Strict convexity makes that edge the complete
            // intersection of either facet with this line.
            return PairRelation::AuthorizedContact;
        }
    } else if let [vertex] = common_vertices.as_slice()
        && planes_are_distinct(left, right)
        && facet_touches_plane_only_at(left, right, *vertex)
        && facet_touches_plane_only_at(right, left, *vertex)
    {
        // Each convex facet meets the other's plane only at the shared
        // vertex, so their intersection is exactly that vertex.
        return PairRelation::AuthorizedContact;
    }

    let mut authorized_contact = false;
    for axis in separating_axes(left, right) {
        if axis.is_zero() {
            continue;
        }
        let left_projection = Projection::new(left, &axis);
        let right_projection = Projection::new(right, &axis);
        match compare_exact(&left_projection.max, &right_projection.min) {
            core::cmp::Ordering::Less => return PairRelation::Disjoint,
            core::cmp::Ordering::Equal => {
                authorized_contact |= exposed_contact_is_authorized(
                    left,
                    &left_projection.max_vertices,
                    right,
                    &right_projection.min_vertices,
                );
            }
            core::cmp::Ordering::Greater => {}
        }
        match compare_exact(&right_projection.max, &left_projection.min) {
            core::cmp::Ordering::Less => return PairRelation::Disjoint,
            core::cmp::Ordering::Equal => {
                authorized_contact |= exposed_contact_is_authorized(
                    right,
                    &right_projection.max_vertices,
                    left,
                    &left_projection.min_vertices,
                );
            }
            core::cmp::Ordering::Greater => {}
        }
    }
    if authorized_contact {
        PairRelation::AuthorizedContact
    } else {
        PairRelation::Ambiguous
    }
}

fn planes_are_distinct(left: &Facet, right: &Facet) -> bool {
    let witness = [
        left.vertices[0].point,
        left.vertices[1].point,
        left.vertices[2].point,
    ];
    right.vertices.iter().any(|vertex| {
        orient3d(witness[0], witness[1], witness[2], vertex.point) != Orientation::Zero
    })
}

fn facet_touches_plane_only_at(facet: &Facet, plane: &Facet, shared: VertexId) -> bool {
    let witness = [
        plane.vertices[0].point,
        plane.vertices[1].point,
        plane.vertices[2].point,
    ];
    let mut side = None;
    for vertex in facet.vertices.iter().filter(|vertex| vertex.id != shared) {
        let current = orient3d(witness[0], witness[1], witness[2], vertex.point);
        if current == Orientation::Zero || side.is_some_and(|expected| expected != current) {
            return false;
        }
        side = Some(current);
    }
    side.is_some()
}

fn exposed_contact_is_authorized(
    left: &Facet,
    left_exposed: &[VertexId],
    right: &Facet,
    right_exposed: &[VertexId],
) -> bool {
    for vertex in &left.vertices {
        if right
            .vertices
            .iter()
            .any(|candidate| candidate.id == vertex.id)
            && exposed_segments_meet_only_at_vertex(
                left,
                left_exposed,
                right,
                right_exposed,
                vertex.id,
            )
        {
            return true;
        }
    }
    left.edges.iter().any(|edge| {
        let Some(other) = right.edges.iter().find(|candidate| candidate.id == edge.id) else {
            return false;
        };
        let endpoints = [edge.from, edge.to];
        edge_has_vertices(other, endpoints[0], endpoints[1])
            && ((same_vertex_set(left_exposed, &endpoints)
                && endpoints
                    .iter()
                    .all(|vertex| right_exposed.contains(vertex)))
                || (same_vertex_set(right_exposed, &endpoints)
                    && endpoints.iter().all(|vertex| left_exposed.contains(vertex))))
    })
}

fn exposed_segments_meet_only_at_vertex(
    left: &Facet,
    left_exposed: &[VertexId],
    right: &Facet,
    right_exposed: &[VertexId],
    shared: VertexId,
) -> bool {
    if !(left_exposed.contains(&shared) && right_exposed.contains(&shared))
        || left_exposed.is_empty()
        || right_exposed.is_empty()
        || left_exposed.len() > 2
        || right_exposed.len() > 2
    {
        return false;
    }
    let Some(left_other) = left_exposed
        .iter()
        .copied()
        .find(|vertex| *vertex != shared)
    else {
        return true;
    };
    let Some(right_other) = right_exposed
        .iter()
        .copied()
        .find(|vertex| *vertex != shared)
    else {
        return true;
    };
    let Some(origin) = left
        .vertices
        .iter()
        .find(|vertex| vertex.id == shared)
        .map(|vertex| vertex.point)
    else {
        return false;
    };
    let Some(left_point) = left
        .vertices
        .iter()
        .find(|vertex| vertex.id == left_other)
        .map(|vertex| vertex.point)
    else {
        return false;
    };
    let Some(right_point) = right
        .vertices
        .iter()
        .find(|vertex| vertex.id == right_other)
        .map(|vertex| vertex.point)
    else {
        return false;
    };
    let left_direction = ExactVec3::difference(left_point, origin);
    let right_direction = ExactVec3::difference(right_point, origin);
    let cross = left_direction.cross(&right_direction);
    !cross.is_zero()
        || exact_sign(&left_direction.dot_exact(&right_direction)) == Orientation::Negative
}

fn edge_has_vertices(edge: &FacetEdge, first: VertexId, second: VertexId) -> bool {
    (edge.from == first && edge.to == second) || (edge.from == second && edge.to == first)
}

fn same_vertex_set(left: &[VertexId], right: &[VertexId]) -> bool {
    left.len() == right.len()
        && left.iter().all(|vertex| right.contains(vertex))
        && right.iter().all(|vertex| left.contains(vertex))
}

fn separating_axes(left: &Facet, right: &Facet) -> Vec<ExactVec3> {
    // Pair-work preflight runs before this allocation and bounds every loop.
    // Avoid unchecked capacity arithmetic even for impossible arena sizes.
    let mut axes = Vec::new();
    axes.push(left.normal.clone());
    axes.push(right.normal.clone());
    for edge in &left.edges {
        axes.push(left.normal.cross(&edge.direction));
    }
    for edge in &right.edges {
        axes.push(right.normal.cross(&edge.direction));
    }
    for left_edge in &left.edges {
        for right_edge in &right.edges {
            axes.push(left_edge.direction.cross(&right_edge.direction));
        }
    }
    for edge in &left.edges {
        axes.push(edge.direction.cross(&right.normal));
    }
    for edge in &right.edges {
        axes.push(left.normal.cross(&edge.direction));
    }
    axes.push(left.normal.cross(&right.normal));
    axes
}

struct Projection {
    min: Vec<f64>,
    max: Vec<f64>,
    min_vertices: Vec<VertexId>,
    max_vertices: Vec<VertexId>,
}

impl Projection {
    fn new(facet: &Facet, axis: &ExactVec3) -> Self {
        let first = axis.dot_point(facet.vertices[0].point);
        let mut value = Self {
            min: first.clone(),
            max: first,
            min_vertices: vec![facet.vertices[0].id],
            max_vertices: vec![facet.vertices[0].id],
        };
        for vertex in &facet.vertices[1..] {
            let projection = axis.dot_point(vertex.point);
            match compare_exact(&projection, &value.min) {
                core::cmp::Ordering::Less => {
                    value.min = projection.clone();
                    value.min_vertices.clear();
                    value.min_vertices.push(vertex.id);
                }
                core::cmp::Ordering::Equal => value.min_vertices.push(vertex.id),
                core::cmp::Ordering::Greater => {}
            }
            match compare_exact(&projection, &value.max) {
                core::cmp::Ordering::Greater => {
                    value.max = projection;
                    value.max_vertices.clear();
                    value.max_vertices.push(vertex.id);
                }
                core::cmp::Ordering::Equal => value.max_vertices.push(vertex.id),
                core::cmp::Ordering::Less => {}
            }
        }
        value
    }
}

#[derive(Debug, Clone)]
struct ExactVec3 {
    coordinates: [Vec<f64>; 3],
}

impl ExactVec3 {
    fn zero() -> Self {
        Self::from_scalars([0.0; 3])
    }

    fn from_scalars(values: [f64; 3]) -> Self {
        Self {
            coordinates: values.map(|value| vec![value]),
        }
    }

    fn difference(left: [f64; 3], right: [f64; 3]) -> Self {
        Self {
            coordinates: core::array::from_fn(|index| {
                let (value, error) = expansion::two_diff(left[index], right[index]);
                expansion::from_two(value, error)
            }),
        }
    }

    fn cross(&self, other: &Self) -> Self {
        Self {
            coordinates: [
                exact_difference(
                    &expansion::mul(&self.coordinates[1], &other.coordinates[2]),
                    &expansion::mul(&self.coordinates[2], &other.coordinates[1]),
                ),
                exact_difference(
                    &expansion::mul(&self.coordinates[2], &other.coordinates[0]),
                    &expansion::mul(&self.coordinates[0], &other.coordinates[2]),
                ),
                exact_difference(
                    &expansion::mul(&self.coordinates[0], &other.coordinates[1]),
                    &expansion::mul(&self.coordinates[1], &other.coordinates[0]),
                ),
            ],
        }
    }

    fn dot_exact(&self, other: &Self) -> Vec<f64> {
        let mut sum = vec![0.0];
        for index in 0..3 {
            sum = expansion::sum(
                &sum,
                &expansion::mul(&self.coordinates[index], &other.coordinates[index]),
            );
        }
        sum
    }

    fn dot_point(&self, point: [f64; 3]) -> Vec<f64> {
        let mut sum = vec![0.0];
        for (coordinate, scalar) in self.coordinates.iter().zip(point) {
            sum = expansion::sum(&sum, &expansion::scale(coordinate, scalar));
        }
        sum
    }

    fn is_zero(&self) -> bool {
        self.coordinates
            .iter()
            .all(|value| expansion::sign(value) == 0)
    }
}

fn exact_difference(left: &[f64], right: &[f64]) -> Vec<f64> {
    expansion::sum(left, &expansion::negate(right))
}

fn compare_exact(left: &[f64], right: &[f64]) -> core::cmp::Ordering {
    match expansion::sign(&exact_difference(left, right)) {
        value if value < 0 => core::cmp::Ordering::Less,
        value if value > 0 => core::cmp::Ordering::Greater,
        _ => core::cmp::Ordering::Equal,
    }
}

fn exact_sign(value: &[f64]) -> Orientation {
    match expansion::sign(value) {
        value if value < 0 => Orientation::Negative,
        value if value > 0 => Orientation::Positive,
        _ => Orientation::Zero,
    }
}

fn exact_signed_volume_sign(facets: &[Facet]) -> Orientation {
    let mut six_volume = vec![0.0];
    for facet in facets {
        let origin = ExactVec3::from_scalars(facet.vertices[0].point);
        for index in 1..facet.vertices.len() - 1 {
            let second = ExactVec3::from_scalars(facet.vertices[index].point);
            let third = ExactVec3::from_scalars(facet.vertices[index + 1].point);
            six_volume = expansion::sum(&six_volume, &origin.dot_exact(&second.cross(&third)));
        }
    }
    exact_sign(&six_volume)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::{CheckLevel, CheckOutcome, check_body_report_with_context};
    use crate::make::block;
    use crate::planar::{PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey};
    use crate::store::Store;
    use crate::transaction::FullCommitRequirement;
    use kcore::operation::{
        ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
        SessionPrecision,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    fn rotated_l_prism() -> PlanarSolidInput {
        let profile = [
            [0.0, 0.0],
            [1.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
            [0.0, 1.0],
        ];
        let keys: Vec<_> = (0..16)
            .map(|index| PlanarVertexKey::new(index + 1))
            .collect();
        let mut vertices = Vec::new();
        for z in [0.0, 1.0] {
            for [x, y] in profile {
                // Axis permutation plus translation is an exact rigid motion;
                // it prevents an axis-specific fixture from passing by luck.
                let point = Point3::new(z + 3.0, x - 1.0, y + 2.0);
                vertices.push(PlanarSolidVertex::new(keys[vertices.len()], point));
            }
        }
        let top = [[0, 1, 4, 7], [1, 2, 3, 4], [7, 4, 5, 6]];
        let mut rings = Vec::new();
        for rectangle in top {
            rings.push(rectangle.map(|index| index + profile.len()).to_vec());
        }
        for mut rectangle in top {
            rectangle.reverse();
            rings.push(rectangle.to_vec());
        }
        for index in 0..profile.len() {
            let next = (index + 1) % profile.len();
            rings.push(vec![
                index,
                next,
                next + profile.len(),
                index + profile.len(),
            ]);
        }
        let faces = rings
            .into_iter()
            .map(|ring| PlanarSolidFace::new(ring.into_iter().map(|index| keys[index]).collect()))
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn context_with_limit(allowed: u64) -> (SessionPolicy, Tolerances) {
        let budget = BudgetPlan::new([LimitSpec::new(
            PLANAR_SHELL_PAIR_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        (
            SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                budget,
                PolicyVersion::V1,
            ),
            Tolerances::default(),
        )
    }

    fn solid_shell(store: &Store, body: crate::entity::BodyId) -> ShellId {
        let body = store.get(body).unwrap();
        let region = body
            .regions
            .iter()
            .copied()
            .find(|region| store.get(*region).unwrap().kind == crate::entity::RegionKind::Solid)
            .unwrap();
        store.get(region).unwrap().shells[0]
    }

    #[test]
    fn rotated_nonconvex_shell_has_exact_embedding_and_outward_orientation_proof() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_planar_solid(&rotated_l_prism())
            .unwrap();
        let shell = output.shell();
        assert_eq!(transaction.store().get(shell).unwrap().faces.len(), 14);

        let (session, tolerances) = context_with_limit(DEFAULT_PLANAR_SHELL_PAIR_WORK);
        let context = OperationContext::new(&session, tolerances).unwrap();
        let mut scope = OperationScope::new(&context);
        assert_eq!(
            certify_general_planar_shell_in_scope(transaction.store(), shell, &mut scope).unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            }
        );
        let consumed = scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|snapshot| snapshot.stage == PLANAR_SHELL_PAIR_WORK)
            .unwrap()
            .consumed;
        assert_eq!(consumed, 25_571);

        let mut flipped = transaction.store().clone();
        let face = flipped.get(shell).unwrap().faces[0];
        flipped.get_mut(face).unwrap().sense = flipped.get(face).unwrap().sense.flipped();
        let mut flipped_scope = OperationScope::new(&context);
        assert_eq!(
            certify_general_planar_shell_in_scope(&flipped, shell, &mut flipped_scope)
                .unwrap()
                .orientation,
            ShellOrientation::Invalid
        );

        let full_session = SessionPolicy::v1();
        let full_context = OperationContext::new(&full_session, Tolerances::default()).unwrap();
        let full = check_body_report_with_context(
            transaction.store(),
            output.body(),
            CheckLevel::Full,
            &full_context,
        )
        .unwrap();
        assert_eq!(
            full.result().as_ref().unwrap().outcome(),
            CheckOutcome::Valid
        );
        assert!(
            full.report()
                .usage()
                .contains(&kcore::operation::LimitSnapshot {
                    stage: PLANAR_SHELL_PAIR_WORK,
                    resource: ResourceKind::Work,
                    consumed: 25_571,
                    allowed: DEFAULT_PLANAR_SHELL_PAIR_WORK,
                })
        );

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
        assert!(
            decision
                .checks()
                .iter()
                .all(|check| check.report().outcome() == CheckOutcome::Valid)
        );
    }

    #[test]
    fn planar_pair_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_planar_solid(&rotated_l_prism())
            .unwrap();
        let shell = solid_shell(transaction.store(), output.body());
        let required = 25_571;

        let (accepted_session, accepted_tolerances) = context_with_limit(required);
        let accepted_context =
            OperationContext::new(&accepted_session, accepted_tolerances).unwrap();
        let mut accepted_scope = OperationScope::new(&accepted_context);
        assert_eq!(
            certify_general_planar_shell_in_scope(transaction.store(), shell, &mut accepted_scope,)
                .unwrap()
                .embedding,
            ShellEmbedding::Certified
        );

        let (denied_session, denied_tolerances) = context_with_limit(required - 1);
        let denied_context = OperationContext::new(&denied_session, denied_tolerances).unwrap();
        let mut denied_scope = OperationScope::new(&denied_context);
        let error =
            certify_general_planar_shell_in_scope(transaction.store(), shell, &mut denied_scope)
                .unwrap_err();
        assert_eq!(
            error.limit(),
            Some(kcore::operation::LimitSnapshot {
                stage: PLANAR_SHELL_PAIR_WORK,
                resource: ResourceKind::Work,
                consumed: required,
                allowed: required - 1,
            })
        );
    }

    #[test]
    fn unauthorized_coincident_facets_remain_ambiguous() {
        let mut store = Store::new();
        let first_body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let second_body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let first_shell = solid_shell(&store, first_body);
        let second_shell = solid_shell(&store, second_body);
        let first_face = store.get(first_shell).unwrap().faces[0];
        let second_face = store.get(second_shell).unwrap().faces[0];
        let first = Facet::new(&store, first_face).unwrap().unwrap();
        let second = Facet::new(&store, second_face).unwrap().unwrap();

        assert!(
            first
                .vertices
                .iter()
                .all(|vertex| second.vertices.iter().all(|other| other.id != vertex.id))
        );
        assert!(first.vertices.iter().all(|vertex| {
            second
                .vertices
                .iter()
                .any(|other| other.point == vertex.point)
        }));
        assert_eq!(
            facet_pair_relation(&first, &second),
            PairRelation::Ambiguous
        );
    }
}
