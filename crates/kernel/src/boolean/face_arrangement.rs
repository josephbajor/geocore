//! Exact combinatorial core for bounded source-face arrangements.
//!
//! Geometry-specific code must first prove that every source span and cut
//! fragment is embedded without an unreported crossing, then provide the
//! counterclockwise rotation of outgoing darts at every exact endpoint.  A
//! rotation is topology evidence, not a metric sort performed here.  This
//! module turns that rotation system into canonical face cycles and refuses
//! incomplete, branched, disconnected, non-planar, or non-separating input.
//!
//! Source spans are directed with the admitted source-face domain on their
//! left.  Cut fragments have two opposed uses.  Those conventions let the
//! core distinguish exterior cycles, prove source-span conservation, and
//! construct the cut-induced dual adjacency without sampling geometry.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// Direction of one dart relative to its proof-owned bounded span.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum ArrangementDirection {
    Forward,
    Reverse,
}

impl ArrangementDirection {
    const fn opposite(self) -> Self {
        match self {
            Self::Forward => Self::Reverse,
            Self::Reverse => Self::Forward,
        }
    }
}

/// Exact identity of one physical arrangement edge.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum ArrangementEdgeKey<S, C> {
    Source(S),
    Cut(C),
}

/// Exact identity of one directed use of a physical arrangement edge.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct ArrangementDartKey<S, C> {
    edge: ArrangementEdgeKey<S, C>,
    direction: ArrangementDirection,
}

impl<S, C> ArrangementDartKey<S, C> {
    /// Forward or reverse use of one source-boundary span.
    pub(crate) const fn source(key: S, direction: ArrangementDirection) -> Self {
        Self {
            edge: ArrangementEdgeKey::Source(key),
            direction,
        }
    }

    /// Forward or reverse use of one section-cut fragment.
    pub(crate) const fn cut(key: C, direction: ArrangementDirection) -> Self {
        Self {
            edge: ArrangementEdgeKey::Cut(key),
            direction,
        }
    }

    /// Exact physical-edge identity.
    pub(crate) const fn edge(&self) -> &ArrangementEdgeKey<S, C> {
        &self.edge
    }

    /// Direction relative to the input span.
    pub(crate) const fn direction(&self) -> ArrangementDirection {
        self.direction
    }
}

impl<S: Clone, C: Clone> ArrangementDartKey<S, C> {
    fn opposite(&self) -> Self {
        Self {
            edge: self.edge.clone(),
            direction: self.direction.opposite(),
        }
    }
}

/// One directed, bounded span of the admitted source-face boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DirectedSourceSpan<S, V> {
    key: S,
    start: V,
    end: V,
    whole_loop: bool,
}

impl<S, V> DirectedSourceSpan<S, V> {
    pub(crate) const fn new(key: S, start: V, end: V) -> Self {
        Self {
            key,
            start,
            end,
            whole_loop: false,
        }
    }

    /// Endpoint-free source cycle represented with one proof-only seam key.
    /// The seam is combinatorial and must not be realized as a topology vertex.
    pub(crate) fn whole_loop(key: S, proof_seam: V) -> Self
    where
        V: Clone,
    {
        Self {
            key,
            start: proof_seam.clone(),
            end: proof_seam,
            whole_loop: true,
        }
    }

    pub(crate) const fn key(&self) -> &S {
        &self.key
    }

    pub(crate) const fn endpoints(&self) -> [&V; 2] {
        [&self.start, &self.end]
    }

    pub(crate) const fn is_whole_loop(&self) -> bool {
        self.whole_loop
    }
}

/// One directed, bounded fragment of a certified section cut.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DirectedCutFragment<C, V> {
    key: C,
    start: V,
    end: V,
    whole_loop: bool,
}

impl<C, V> DirectedCutFragment<C, V> {
    pub(crate) const fn new(key: C, start: V, end: V) -> Self {
        Self {
            key,
            start,
            end,
            whole_loop: false,
        }
    }

    /// Endpoint-free cut cycle represented with one proof-only seam key.
    pub(crate) fn whole_loop(key: C, proof_seam: V) -> Self
    where
        V: Clone,
    {
        Self {
            key,
            start: proof_seam.clone(),
            end: proof_seam,
            whole_loop: true,
        }
    }

    pub(crate) const fn key(&self) -> &C {
        &self.key
    }

    pub(crate) const fn endpoints(&self) -> [&V; 2] {
        [&self.start, &self.end]
    }

    pub(crate) const fn is_whole_loop(&self) -> bool {
        self.whole_loop
    }
}

/// Certified counterclockwise order of all outgoing darts at one endpoint.
///
/// Cyclic shifts are equivalent.  Reversal is not: it changes the embedding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedEndpointRotation<S, C, V> {
    endpoint: V,
    outgoing: Vec<ArrangementDartKey<S, C>>,
}

impl<S, C, V> CertifiedEndpointRotation<S, C, V> {
    pub(crate) const fn new(endpoint: V, outgoing: Vec<ArrangementDartKey<S, C>>) -> Self {
        Self { endpoint, outgoing }
    }

    pub(crate) const fn endpoint(&self) -> &V {
        &self.endpoint
    }

    pub(crate) fn outgoing(&self) -> &[ArrangementDartKey<S, C>] {
        &self.outgoing
    }
}

/// Complete exact-topology input for one source face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FaceArrangementInput<S, C, V> {
    source_spans: Vec<DirectedSourceSpan<S, V>>,
    cut_fragments: Vec<DirectedCutFragment<C, V>>,
    rotations: Vec<CertifiedEndpointRotation<S, C, V>>,
}

impl<S, C, V> FaceArrangementInput<S, C, V> {
    pub(crate) const fn new(
        source_spans: Vec<DirectedSourceSpan<S, V>>,
        cut_fragments: Vec<DirectedCutFragment<C, V>>,
        rotations: Vec<CertifiedEndpointRotation<S, C, V>>,
    ) -> Self {
        Self {
            source_spans,
            cut_fragments,
            rotations,
        }
    }
}

/// Exact degree certificate for one endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ArrangementEndpointDegree {
    source: usize,
    cut: usize,
}

impl ArrangementEndpointDegree {
    pub(crate) const fn source(self) -> usize {
        self.source
    }

    pub(crate) const fn cut(self) -> usize {
        self.cut
    }

    pub(crate) const fn total(self) -> usize {
        self.source + self.cut
    }
}

/// One canonical, closed, counterclockwise face-boundary walk.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct ArrangementCycle<S, C, V> {
    uses: Vec<ArrangementDartKey<S, C>>,
    vertices: Vec<V>,
}

impl<S, C, V> ArrangementCycle<S, C, V> {
    pub(crate) fn uses(&self) -> &[ArrangementDartKey<S, C>] {
        &self.uses
    }

    /// Traversed endpoints, including the repeated closing endpoint.
    pub(crate) fn vertices(&self) -> &[V] {
        &self.vertices
    }
}

/// One open source-face cell bounded by one canonical cycle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ArrangementCell<S, C, V> {
    key: usize,
    boundary: ArrangementCycle<S, C, V>,
}

impl<S, C, V> ArrangementCell<S, C, V> {
    pub(crate) const fn key(&self) -> usize {
        self.key
    }

    pub(crate) const fn boundary(&self) -> &ArrangementCycle<S, C, V> {
        &self.boundary
    }
}

/// The two distinct cells using one cut fragment in opposed directions.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ArrangementCutAdjacency<C> {
    cut: C,
    forward_cell: usize,
    reverse_cell: usize,
}

impl<C> ArrangementCutAdjacency<C> {
    pub(crate) const fn cut(&self) -> &C {
        &self.cut
    }

    pub(crate) const fn forward_cell(&self) -> usize {
        self.forward_cell
    }

    pub(crate) const fn reverse_cell(&self) -> usize {
        self.reverse_cell
    }
}

/// Auditable invariants established while constructing an arrangement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FaceArrangementProof<V> {
    endpoint_degrees: Vec<(V, ArrangementEndpointDegree)>,
    source_spans_conserved: usize,
    opposed_cut_pairs: usize,
    closed_cycles: usize,
    exterior_cycles: usize,
    dual_connected: bool,
    euler_characteristic: isize,
}

impl<V> FaceArrangementProof<V> {
    pub(crate) fn endpoint_degrees(&self) -> &[(V, ArrangementEndpointDegree)] {
        &self.endpoint_degrees
    }

    pub(crate) const fn source_spans_conserved(&self) -> usize {
        self.source_spans_conserved
    }

    pub(crate) const fn opposed_cut_pairs(&self) -> usize {
        self.opposed_cut_pairs
    }

    pub(crate) const fn closed_cycles(&self) -> usize {
        self.closed_cycles
    }

    pub(crate) const fn exterior_cycles(&self) -> usize {
        self.exterior_cycles
    }

    pub(crate) const fn dual_connected(&self) -> bool {
        self.dual_connected
    }

    pub(crate) const fn euler_characteristic(&self) -> isize {
        self.euler_characteristic
    }
}

/// Canonical exact-topology result for one source face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FaceArrangement<S, C, V> {
    source_spans: Vec<DirectedSourceSpan<S, V>>,
    cut_fragments: Vec<DirectedCutFragment<C, V>>,
    cells: Vec<ArrangementCell<S, C, V>>,
    adjacency: Vec<ArrangementCutAdjacency<C>>,
    proof: FaceArrangementProof<V>,
}

/// Proof-owned side of one derived boundary cycle on a source surface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum CertifiedCycleSide<K> {
    Exterior,
    Cell(K),
}

/// Exact assignment of a derived cycle to a cell or to the source exterior.
///
/// Any dart on the cycle is a valid anchor.  The arrangement core resolves
/// the complete cycle and rejects duplicate or missing assignments.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedCycleAssignment<S, C, K> {
    anchor: ArrangementDartKey<S, C>,
    side: CertifiedCycleSide<K>,
}

impl<S, C, K> CertifiedCycleAssignment<S, C, K> {
    pub(crate) const fn new(anchor: ArrangementDartKey<S, C>, side: CertifiedCycleSide<K>) -> Self {
        Self { anchor, side }
    }

    pub(crate) const fn anchor(&self) -> &ArrangementDartKey<S, C> {
        &self.anchor
    }

    pub(crate) const fn side(&self) -> &CertifiedCycleSide<K> {
        &self.side
    }
}

/// Proof-owned topology of one connected open cell closure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedCellTopology<K> {
    key: K,
    euler_characteristic: i64,
}

impl<K> CertifiedCellTopology<K> {
    pub(crate) const fn new(key: K, euler_characteristic: i64) -> Self {
        Self {
            key,
            euler_characteristic,
        }
    }

    pub(crate) const fn key(&self) -> &K {
        &self.key
    }

    pub(crate) const fn euler_characteristic(&self) -> i64 {
        self.euler_characteristic
    }
}

/// Exact embedding relation required when graph components are disconnected.
///
/// Cell Euler characteristics and the source-surface characteristic are
/// proof inputs.  They are checked against the derived boundary counts and
/// Euler additivity; they are never inferred from metric nesting.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedSurfaceEmbedding<S, C, K> {
    assignments: Vec<CertifiedCycleAssignment<S, C, K>>,
    cells: Vec<CertifiedCellTopology<K>>,
    surface_euler_characteristic: i64,
}

impl<S, C, K> CertifiedSurfaceEmbedding<S, C, K> {
    pub(crate) const fn new(
        assignments: Vec<CertifiedCycleAssignment<S, C, K>>,
        cells: Vec<CertifiedCellTopology<K>>,
        surface_euler_characteristic: i64,
    ) -> Self {
        Self {
            assignments,
            cells,
            surface_euler_characteristic,
        }
    }
}

/// One proof-bearing cell with all of its disconnected boundary cycles.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SurfaceArrangementCell<S, C, V, K> {
    key: K,
    boundaries: Vec<ArrangementCycle<S, C, V>>,
    euler_characteristic: i64,
    genus: u64,
}

impl<S, C, V, K> SurfaceArrangementCell<S, C, V, K> {
    pub(crate) const fn key(&self) -> &K {
        &self.key
    }

    pub(crate) fn boundaries(&self) -> &[ArrangementCycle<S, C, V>] {
        &self.boundaries
    }

    pub(crate) const fn euler_characteristic(&self) -> i64 {
        self.euler_characteristic
    }

    pub(crate) const fn genus(&self) -> u64 {
        self.genus
    }
}

/// Exact cells on the forward and reverse sides of one cut fragment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SurfaceCutAdjacency<C, K> {
    cut: C,
    forward_cell: K,
    reverse_cell: K,
}

impl<C, K> SurfaceCutAdjacency<C, K> {
    pub(crate) const fn cut(&self) -> &C {
        &self.cut
    }

    pub(crate) const fn forward_cell(&self) -> &K {
        &self.forward_cell
    }

    pub(crate) const fn reverse_cell(&self) -> &K {
        &self.reverse_cell
    }
}

/// Auditable invariants established for a general surface arrangement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SurfaceArrangementProof<V, K> {
    endpoint_degrees: Vec<(V, ArrangementEndpointDegree)>,
    directed_darts_conserved: usize,
    source_spans_conserved: usize,
    opposed_cut_pairs: usize,
    closed_cycles: usize,
    exterior_cycles: usize,
    primal_components: usize,
    source_boundary_components: usize,
    cell_genera: Vec<(K, u64)>,
    dual_connected: bool,
    surface_euler_characteristic: i64,
    surface_genus: u64,
}

impl<V, K> SurfaceArrangementProof<V, K> {
    pub(crate) fn endpoint_degrees(&self) -> &[(V, ArrangementEndpointDegree)] {
        &self.endpoint_degrees
    }

    pub(crate) const fn directed_darts_conserved(&self) -> usize {
        self.directed_darts_conserved
    }

    pub(crate) const fn source_spans_conserved(&self) -> usize {
        self.source_spans_conserved
    }

    pub(crate) const fn opposed_cut_pairs(&self) -> usize {
        self.opposed_cut_pairs
    }

    pub(crate) const fn closed_cycles(&self) -> usize {
        self.closed_cycles
    }

    pub(crate) const fn exterior_cycles(&self) -> usize {
        self.exterior_cycles
    }

    pub(crate) const fn primal_components(&self) -> usize {
        self.primal_components
    }

    pub(crate) const fn source_boundary_components(&self) -> usize {
        self.source_boundary_components
    }

    pub(crate) fn cell_genera(&self) -> &[(K, u64)] {
        &self.cell_genera
    }

    pub(crate) const fn dual_connected(&self) -> bool {
        self.dual_connected
    }

    pub(crate) const fn surface_euler_characteristic(&self) -> i64 {
        self.surface_euler_characteristic
    }

    pub(crate) const fn surface_genus(&self) -> u64 {
        self.surface_genus
    }
}

/// Canonical proof-bearing arrangement on a connected orientable surface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SurfaceFaceArrangement<S, C, V, K> {
    source_spans: Vec<DirectedSourceSpan<S, V>>,
    cut_fragments: Vec<DirectedCutFragment<C, V>>,
    cells: Vec<SurfaceArrangementCell<S, C, V, K>>,
    adjacency: Vec<SurfaceCutAdjacency<C, K>>,
    proof: SurfaceArrangementProof<V, K>,
}

impl<S, C, V, K> SurfaceFaceArrangement<S, C, V, K> {
    pub(crate) fn source_spans(&self) -> &[DirectedSourceSpan<S, V>] {
        &self.source_spans
    }

    pub(crate) fn cut_fragments(&self) -> &[DirectedCutFragment<C, V>] {
        &self.cut_fragments
    }

    pub(crate) fn cells(&self) -> &[SurfaceArrangementCell<S, C, V, K>] {
        &self.cells
    }

    pub(crate) fn adjacency(&self) -> &[SurfaceCutAdjacency<C, K>] {
        &self.adjacency
    }

    pub(crate) const fn proof(&self) -> &SurfaceArrangementProof<V, K> {
        &self.proof
    }
}

/// Typed refusals specific to proof-owned surface embeddings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum SurfaceArrangementError<S, C, V, K> {
    Graph(FaceArrangementError<S, C, V>),
    UnknownCycleAnchor(ArrangementDartKey<S, C>),
    DuplicateCycleAssignment(ArrangementDartKey<S, C>),
    MissingCycleAssignment(ArrangementDartKey<S, C>),
    DuplicateCellTopology(K),
    UnknownCell(K),
    UnusedCellTopology(K),
    SourceSideMismatch(S),
    CutTouchesExterior(C),
    CutDoesNotSeparateCells(C),
    CellTopologyInconsistent {
        cell: K,
        boundary_cycles: usize,
        euler_characteristic: i64,
    },
    SurfaceTopologyInconsistent {
        boundary_components: usize,
        euler_characteristic: i64,
    },
    SurfaceEulerMismatch {
        expected: i64,
        actual: i64,
    },
    TopologyArithmeticOverflow,
    DisconnectedDual,
}

type FaceResult<S, C, V, T> = Result<T, FaceArrangementError<S, C, V>>;
type SurfaceResult<S, C, V, K, T> = Result<T, SurfaceArrangementError<S, C, V, K>>;
type EndpointRotations<S, C, V> = BTreeMap<V, Vec<ArrangementDartKey<S, C>>>;
type SurfaceCells<S, C, V, K> = Vec<SurfaceArrangementCell<S, C, V, K>>;

impl<S, C, V> FaceArrangement<S, C, V> {
    pub(crate) fn source_spans(&self) -> &[DirectedSourceSpan<S, V>] {
        &self.source_spans
    }

    pub(crate) fn cut_fragments(&self) -> &[DirectedCutFragment<C, V>] {
        &self.cut_fragments
    }

    pub(crate) fn cells(&self) -> &[ArrangementCell<S, C, V>] {
        &self.cells
    }

    pub(crate) fn adjacency(&self) -> &[ArrangementCutAdjacency<C>] {
        &self.adjacency
    }

    pub(crate) const fn proof(&self) -> &FaceArrangementProof<V> {
        &self.proof
    }
}

/// Typed fail-closed refusals from exact combinatorial arrangement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum FaceArrangementError<S, C, V> {
    SourceBoundaryRequired,
    DuplicateSourceSpan(S),
    DuplicateCutFragment(C),
    DegenerateSourceSpan(S),
    DegenerateCutFragment(C),
    MalformedWholeSourceSpan(S),
    MalformedWholeCutFragment(C),
    OpenSourceBoundary(V),
    BranchedSourceBoundary(V),
    InconsistentSourceDirection(V),
    OpenCutEndpoint(V),
    BranchedCutEndpoint(V),
    InconsistentCutDirection(V),
    MissingEndpointRotation(V),
    UnexpectedEndpointRotation(V),
    DuplicateEndpointRotation(V),
    EndpointIncidenceMismatch(V),
    DisconnectedPrimal,
    EmptyCycle,
    CycleDoesNotClose(ArrangementDartKey<S, C>),
    NonPlanarRotationSystem { euler_characteristic: isize },
    SourceSidesShareCycle(S),
    CutTouchesExterior(C),
    CutDoesNotSeparateCells(C),
    DisconnectedDual,
}

#[derive(Debug, Clone)]
struct CanonicalInput<S, C, V> {
    sources: Vec<DirectedSourceSpan<S, V>>,
    cuts: Vec<DirectedCutFragment<C, V>>,
    rotations: BTreeMap<V, Vec<ArrangementDartKey<S, C>>>,
    endpoints: BTreeMap<ArrangementDartKey<S, C>, (V, V)>,
    degrees: BTreeMap<V, ArrangementEndpointDegree>,
}

/// Build and certify a bounded source-face arrangement.
pub(crate) fn arrange_bounded_face<S, C, V>(
    input: FaceArrangementInput<S, C, V>,
) -> Result<FaceArrangement<S, C, V>, FaceArrangementError<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let canonical = canonicalize_input(input)?;
    ensure_primal_connected(&canonical)?;

    let mut cycles = traverse_cycles(&canonical)?;
    cycles.sort_unstable();

    let vertex_count = canonical.degrees.len() as isize;
    let edge_count = (canonical.sources.len() + canonical.cuts.len()) as isize;
    let euler_characteristic = vertex_count - edge_count + cycles.len() as isize;
    if euler_characteristic != 2 {
        return Err(FaceArrangementError::NonPlanarRotationSystem {
            euler_characteristic,
        });
    }

    let cycle_of = index_cycle_uses(&cycles);
    let mut exterior = vec![false; cycles.len()];
    mark_and_validate_exterior(&canonical.sources, &cycle_of, &mut exterior)?;

    let mut cycle_to_cell = BTreeMap::new();
    let mut cells = Vec::new();
    for (cycle_index, cycle) in cycles.iter().enumerate() {
        if !exterior[cycle_index] {
            let key = cells.len();
            cycle_to_cell.insert(cycle_index, key);
            cells.push(ArrangementCell {
                key,
                boundary: cycle.clone(),
            });
        }
    }

    let adjacency = build_cut_adjacency(&canonical.cuts, &cycle_of, &cycle_to_cell, &exterior)?;
    if !dual_is_connected(cells.len(), &adjacency) {
        return Err(FaceArrangementError::DisconnectedDual);
    }

    let proof = FaceArrangementProof {
        endpoint_degrees: canonical.degrees.into_iter().collect(),
        source_spans_conserved: canonical.sources.len(),
        opposed_cut_pairs: canonical.cuts.len(),
        closed_cycles: cycles.len(),
        exterior_cycles: exterior.iter().filter(|value| **value).count(),
        dual_connected: true,
        euler_characteristic,
    };

    Ok(FaceArrangement {
        source_spans: canonical.sources,
        cut_fragments: canonical.cuts,
        cells,
        adjacency,
        proof,
    })
}

/// Build and certify an arrangement whose cells may have multiple boundary
/// cycles and whose embedded graph may have multiple connected components.
pub(crate) fn arrange_bounded_surface<S, C, V, K>(
    input: FaceArrangementInput<S, C, V>,
    embedding: CertifiedSurfaceEmbedding<S, C, K>,
) -> SurfaceResult<S, C, V, K, SurfaceFaceArrangement<S, C, V, K>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
    K: Clone + Ord,
{
    let canonical = canonicalize_input(input).map_err(SurfaceArrangementError::Graph)?;
    let mut cycles = traverse_cycles(&canonical).map_err(SurfaceArrangementError::Graph)?;
    cycles.sort_unstable();
    let cycle_of = index_cycle_uses(&cycles);
    let assignments = resolve_cycle_assignments(&cycles, &cycle_of, embedding.assignments)?;
    let cell_topology = collect_cell_topology(embedding.cells)?;

    validate_surface_source_sides(&canonical.sources, &cycle_of, &assignments)?;
    let adjacency =
        build_surface_cut_adjacency(&canonical.cuts, &cycle_of, &assignments, &cell_topology)?;
    let cells = build_surface_cells(&cycles, &assignments, &cell_topology)?;
    if !surface_dual_is_connected(&cells, &adjacency) {
        return Err(SurfaceArrangementError::DisconnectedDual);
    }

    let source_boundary_components =
        graph_component_count(source_edge_endpoints(&canonical.sources));
    let surface_genus = orientable_genus(
        source_boundary_components,
        embedding.surface_euler_characteristic,
    )
    .ok_or(SurfaceArrangementError::SurfaceTopologyInconsistent {
        boundary_components: source_boundary_components,
        euler_characteristic: embedding.surface_euler_characteristic,
    })?;
    let vertex_count = i64::try_from(canonical.degrees.len())
        .map_err(|_| SurfaceArrangementError::TopologyArithmeticOverflow)?;
    let edge_count = i64::try_from(canonical.sources.len() + canonical.cuts.len())
        .map_err(|_| SurfaceArrangementError::TopologyArithmeticOverflow)?;
    let cell_characteristic = cells.iter().try_fold(0_i64, |sum, cell| {
        sum.checked_add(cell.euler_characteristic)
            .ok_or(SurfaceArrangementError::TopologyArithmeticOverflow)
    })?;
    let actual_euler = vertex_count
        .checked_sub(edge_count)
        .and_then(|value| value.checked_add(cell_characteristic))
        .ok_or(SurfaceArrangementError::TopologyArithmeticOverflow)?;
    if actual_euler != embedding.surface_euler_characteristic {
        return Err(SurfaceArrangementError::SurfaceEulerMismatch {
            expected: embedding.surface_euler_characteristic,
            actual: actual_euler,
        });
    }

    let directed_darts = cycles.iter().try_fold(0_usize, |sum, cycle| {
        sum.checked_add(cycle.uses.len())
            .ok_or(SurfaceArrangementError::TopologyArithmeticOverflow)
    })?;
    if directed_darts != canonical.endpoints.len() {
        return Err(SurfaceArrangementError::TopologyArithmeticOverflow);
    }
    let proof = SurfaceArrangementProof {
        endpoint_degrees: canonical.degrees.into_iter().collect(),
        directed_darts_conserved: directed_darts,
        source_spans_conserved: canonical.sources.len(),
        opposed_cut_pairs: canonical.cuts.len(),
        closed_cycles: cycles.len(),
        exterior_cycles: assignments
            .values()
            .filter(|side| matches!(side, CertifiedCycleSide::Exterior))
            .count(),
        primal_components: graph_component_count(canonical.endpoints.values().cloned()),
        source_boundary_components,
        cell_genera: cells
            .iter()
            .map(|cell| (cell.key.clone(), cell.genus))
            .collect(),
        dual_connected: true,
        surface_euler_characteristic: embedding.surface_euler_characteristic,
        surface_genus,
    };

    Ok(SurfaceFaceArrangement {
        source_spans: canonical.sources,
        cut_fragments: canonical.cuts,
        cells,
        adjacency,
        proof,
    })
}

/// Derive the canonical cycles of a proof-owned rotation system before a
/// surface-specific theorem assigns them to cells or the source exterior.
pub(crate) fn preview_bounded_surface_cycles<S, C, V>(
    input: &FaceArrangementInput<S, C, V>,
) -> FaceResult<S, C, V, Vec<ArrangementCycle<S, C, V>>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let canonical = canonicalize_input(input.clone())?;
    let mut cycles = traverse_cycles(&canonical)?;
    cycles.sort_unstable();
    Ok(cycles)
}

fn resolve_cycle_assignments<S, C, V, K>(
    cycles: &[ArrangementCycle<S, C, V>],
    cycle_of: &BTreeMap<ArrangementDartKey<S, C>, usize>,
    assignments: Vec<CertifiedCycleAssignment<S, C, K>>,
) -> SurfaceResult<S, C, V, K, BTreeMap<usize, CertifiedCycleSide<K>>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    K: Clone,
{
    let mut resolved = BTreeMap::new();
    for assignment in assignments {
        let Some(&cycle) = cycle_of.get(&assignment.anchor) else {
            return Err(SurfaceArrangementError::UnknownCycleAnchor(
                assignment.anchor,
            ));
        };
        if resolved.insert(cycle, assignment.side).is_some() {
            let anchor = cycles
                .get(cycle)
                .and_then(|value| value.uses.first())
                .cloned()
                .ok_or(SurfaceArrangementError::Graph(
                    FaceArrangementError::EmptyCycle,
                ))?;
            return Err(SurfaceArrangementError::DuplicateCycleAssignment(anchor));
        }
    }
    for (cycle, boundary) in cycles.iter().enumerate() {
        if !resolved.contains_key(&cycle) {
            let anchor = boundary
                .uses
                .first()
                .cloned()
                .ok_or(SurfaceArrangementError::Graph(
                    FaceArrangementError::EmptyCycle,
                ))?;
            return Err(SurfaceArrangementError::MissingCycleAssignment(anchor));
        }
    }
    Ok(resolved)
}

fn collect_cell_topology<S, C, V, K>(
    cells: Vec<CertifiedCellTopology<K>>,
) -> Result<BTreeMap<K, i64>, SurfaceArrangementError<S, C, V, K>>
where
    K: Clone + Ord,
{
    let mut result = BTreeMap::new();
    for cell in cells {
        if result
            .insert(cell.key.clone(), cell.euler_characteristic)
            .is_some()
        {
            return Err(SurfaceArrangementError::DuplicateCellTopology(cell.key));
        }
    }
    Ok(result)
}

fn validate_surface_source_sides<S, C, V, K>(
    sources: &[DirectedSourceSpan<S, V>],
    cycle_of: &BTreeMap<ArrangementDartKey<S, C>, usize>,
    assignments: &BTreeMap<usize, CertifiedCycleSide<K>>,
) -> Result<(), SurfaceArrangementError<S, C, V, K>>
where
    S: Clone + Ord,
    C: Clone + Ord,
{
    for source in sources {
        let forward = ArrangementDartKey::source(source.key.clone(), ArrangementDirection::Forward);
        let reverse = ArrangementDartKey::source(source.key.clone(), ArrangementDirection::Reverse);
        let Some(forward_side) = cycle_of
            .get(&forward)
            .and_then(|cycle| assignments.get(cycle))
        else {
            return Err(SurfaceArrangementError::Graph(
                FaceArrangementError::CycleDoesNotClose(forward),
            ));
        };
        let Some(reverse_side) = cycle_of
            .get(&reverse)
            .and_then(|cycle| assignments.get(cycle))
        else {
            return Err(SurfaceArrangementError::Graph(
                FaceArrangementError::CycleDoesNotClose(reverse),
            ));
        };
        if !matches!(forward_side, CertifiedCycleSide::Cell(_))
            || !matches!(reverse_side, CertifiedCycleSide::Exterior)
        {
            return Err(SurfaceArrangementError::SourceSideMismatch(
                source.key.clone(),
            ));
        }
    }
    Ok(())
}

fn build_surface_cut_adjacency<S, C, V, K>(
    cuts: &[DirectedCutFragment<C, V>],
    cycle_of: &BTreeMap<ArrangementDartKey<S, C>, usize>,
    assignments: &BTreeMap<usize, CertifiedCycleSide<K>>,
    cell_topology: &BTreeMap<K, i64>,
) -> SurfaceResult<S, C, V, K, Vec<SurfaceCutAdjacency<C, K>>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    K: Clone + Ord,
{
    let mut result = Vec::with_capacity(cuts.len());
    for cut in cuts {
        let forward = ArrangementDartKey::cut(cut.key.clone(), ArrangementDirection::Forward);
        let reverse = ArrangementDartKey::cut(cut.key.clone(), ArrangementDirection::Reverse);
        let Some(forward_side) = cycle_of
            .get(&forward)
            .and_then(|cycle| assignments.get(cycle))
        else {
            return Err(SurfaceArrangementError::Graph(
                FaceArrangementError::CycleDoesNotClose(forward),
            ));
        };
        let Some(reverse_side) = cycle_of
            .get(&reverse)
            .and_then(|cycle| assignments.get(cycle))
        else {
            return Err(SurfaceArrangementError::Graph(
                FaceArrangementError::CycleDoesNotClose(reverse),
            ));
        };
        let (CertifiedCycleSide::Cell(forward_cell), CertifiedCycleSide::Cell(reverse_cell)) =
            (forward_side, reverse_side)
        else {
            return Err(SurfaceArrangementError::CutTouchesExterior(cut.key.clone()));
        };
        if forward_cell == reverse_cell {
            return Err(SurfaceArrangementError::CutDoesNotSeparateCells(
                cut.key.clone(),
            ));
        }
        for cell in [forward_cell, reverse_cell] {
            if !cell_topology.contains_key(cell) {
                return Err(SurfaceArrangementError::UnknownCell(cell.clone()));
            }
        }
        result.push(SurfaceCutAdjacency {
            cut: cut.key.clone(),
            forward_cell: forward_cell.clone(),
            reverse_cell: reverse_cell.clone(),
        });
    }
    Ok(result)
}

fn build_surface_cells<S, C, V, K>(
    cycles: &[ArrangementCycle<S, C, V>],
    assignments: &BTreeMap<usize, CertifiedCycleSide<K>>,
    cell_topology: &BTreeMap<K, i64>,
) -> SurfaceResult<S, C, V, K, SurfaceCells<S, C, V, K>>
where
    S: Clone,
    C: Clone,
    V: Clone,
    K: Clone + Ord,
{
    let mut boundaries: BTreeMap<K, Vec<ArrangementCycle<S, C, V>>> = BTreeMap::new();
    for (cycle, side) in assignments {
        let CertifiedCycleSide::Cell(cell) = side else {
            continue;
        };
        if !cell_topology.contains_key(cell) {
            return Err(SurfaceArrangementError::UnknownCell(cell.clone()));
        }
        let Some(boundary) = cycles.get(*cycle) else {
            return Err(SurfaceArrangementError::TopologyArithmeticOverflow);
        };
        boundaries
            .entry(cell.clone())
            .or_default()
            .push(boundary.clone());
    }

    let mut result = Vec::with_capacity(cell_topology.len());
    for (key, &euler_characteristic) in cell_topology {
        let Some(cell_boundaries) = boundaries.remove(key) else {
            return Err(SurfaceArrangementError::UnusedCellTopology(key.clone()));
        };
        let Some(genus) = orientable_genus(cell_boundaries.len(), euler_characteristic) else {
            return Err(SurfaceArrangementError::CellTopologyInconsistent {
                cell: key.clone(),
                boundary_cycles: cell_boundaries.len(),
                euler_characteristic,
            });
        };
        result.push(SurfaceArrangementCell {
            key: key.clone(),
            boundaries: cell_boundaries,
            euler_characteristic,
            genus,
        });
    }
    Ok(result)
}

fn orientable_genus(boundary_components: usize, euler_characteristic: i64) -> Option<u64> {
    let boundaries = i64::try_from(boundary_components).ok()?;
    let twice_genus = 2_i64
        .checked_sub(boundaries)?
        .checked_sub(euler_characteristic)?;
    if twice_genus < 0 || twice_genus % 2 != 0 {
        return None;
    }
    u64::try_from(twice_genus / 2).ok()
}

fn source_edge_endpoints<S, V>(sources: &[DirectedSourceSpan<S, V>]) -> Vec<(V, V)>
where
    V: Clone,
{
    sources
        .iter()
        .map(|source| (source.start.clone(), source.end.clone()))
        .collect()
}

fn graph_component_count<V>(edges: impl IntoIterator<Item = (V, V)>) -> usize
where
    V: Clone + Ord,
{
    let mut adjacency: BTreeMap<V, BTreeSet<V>> = BTreeMap::new();
    for (from, to) in edges {
        adjacency
            .entry(from.clone())
            .or_default()
            .insert(to.clone());
        adjacency.entry(to).or_default().insert(from);
    }
    let mut unvisited: BTreeSet<_> = adjacency.keys().cloned().collect();
    let mut components = 0;
    while let Some(start) = unvisited.iter().next().cloned() {
        components += 1;
        let mut pending = vec![start];
        while let Some(vertex) = pending.pop() {
            if !unvisited.remove(&vertex) {
                continue;
            }
            if let Some(neighbors) = adjacency.get(&vertex) {
                pending.extend(neighbors.iter().cloned());
            }
        }
    }
    components
}

fn surface_dual_is_connected<S, C, V, K>(
    cells: &[SurfaceArrangementCell<S, C, V, K>],
    adjacency: &[SurfaceCutAdjacency<C, K>],
) -> bool
where
    K: Clone + Ord,
{
    let Some(start) = cells.first().map(|cell| cell.key.clone()) else {
        return false;
    };
    let mut neighbors: BTreeMap<K, BTreeSet<K>> = cells
        .iter()
        .map(|cell| (cell.key.clone(), BTreeSet::new()))
        .collect();
    for edge in adjacency {
        let Some(forward) = neighbors.get_mut(&edge.forward_cell) else {
            return false;
        };
        forward.insert(edge.reverse_cell.clone());
        let Some(reverse) = neighbors.get_mut(&edge.reverse_cell) else {
            return false;
        };
        reverse.insert(edge.forward_cell.clone());
    }
    let mut pending = vec![start];
    let mut visited = BTreeSet::new();
    while let Some(cell) = pending.pop() {
        if !visited.insert(cell.clone()) {
            continue;
        }
        if let Some(next) = neighbors.get(&cell) {
            pending.extend(next.iter().cloned());
        }
    }
    visited.len() == cells.len()
}

fn canonicalize_input<S, C, V>(
    input: FaceArrangementInput<S, C, V>,
) -> Result<CanonicalInput<S, C, V>, FaceArrangementError<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    if input.source_spans.is_empty() {
        return Err(FaceArrangementError::SourceBoundaryRequired);
    }

    let mut sources = input.source_spans;
    sources.sort_unstable_by(|a, b| a.key.cmp(&b.key));
    for pair in sources.windows(2) {
        if pair[0].key == pair[1].key {
            return Err(FaceArrangementError::DuplicateSourceSpan(
                pair[0].key.clone(),
            ));
        }
    }

    let mut cuts = input.cut_fragments;
    cuts.sort_unstable_by(|a, b| a.key.cmp(&b.key));
    for pair in cuts.windows(2) {
        if pair[0].key == pair[1].key {
            return Err(FaceArrangementError::DuplicateCutFragment(
                pair[0].key.clone(),
            ));
        }
    }

    let mut endpoints = BTreeMap::new();
    let mut incidence: BTreeMap<V, BTreeSet<ArrangementDartKey<S, C>>> = BTreeMap::new();
    for span in &sources {
        if span.start == span.end && !span.whole_loop {
            return Err(FaceArrangementError::DegenerateSourceSpan(span.key.clone()));
        }
        if span.start != span.end && span.whole_loop {
            return Err(FaceArrangementError::MalformedWholeSourceSpan(
                span.key.clone(),
            ));
        }
        insert_edge(
            ArrangementEdgeKey::Source(span.key.clone()),
            span.start.clone(),
            span.end.clone(),
            &mut endpoints,
            &mut incidence,
        );
    }
    for fragment in &cuts {
        if fragment.start == fragment.end && !fragment.whole_loop {
            return Err(FaceArrangementError::DegenerateCutFragment(
                fragment.key.clone(),
            ));
        }
        if fragment.start != fragment.end && fragment.whole_loop {
            return Err(FaceArrangementError::MalformedWholeCutFragment(
                fragment.key.clone(),
            ));
        }
        insert_edge(
            ArrangementEdgeKey::Cut(fragment.key.clone()),
            fragment.start.clone(),
            fragment.end.clone(),
            &mut endpoints,
            &mut incidence,
        );
    }

    let mut degrees = BTreeMap::new();
    for (endpoint, outgoing) in &incidence {
        let source = outgoing
            .iter()
            .filter(|dart| matches!(dart.edge, ArrangementEdgeKey::Source(_)))
            .count();
        let cut = outgoing.len() - source;
        validate_degree(endpoint, source, cut)?;
        if source == 2 {
            let forward = outgoing
                .iter()
                .filter(|dart| {
                    matches!(dart.edge, ArrangementEdgeKey::Source(_))
                        && dart.direction == ArrangementDirection::Forward
                })
                .count();
            if forward != 1 {
                return Err(FaceArrangementError::InconsistentSourceDirection(
                    endpoint.clone(),
                ));
            }
        }
        if cut == 2 {
            let forward = outgoing
                .iter()
                .filter(|dart| {
                    matches!(dart.edge, ArrangementEdgeKey::Cut(_))
                        && dart.direction == ArrangementDirection::Forward
                })
                .count();
            if forward != 1 {
                return Err(FaceArrangementError::InconsistentCutDirection(
                    endpoint.clone(),
                ));
            }
        }
        degrees.insert(endpoint.clone(), ArrangementEndpointDegree { source, cut });
    }

    let rotations = canonicalize_rotations(input.rotations, &incidence)?;
    Ok(CanonicalInput {
        sources,
        cuts,
        rotations,
        endpoints,
        degrees,
    })
}

fn insert_edge<S, C, V>(
    edge: ArrangementEdgeKey<S, C>,
    start: V,
    end: V,
    endpoints: &mut BTreeMap<ArrangementDartKey<S, C>, (V, V)>,
    incidence: &mut BTreeMap<V, BTreeSet<ArrangementDartKey<S, C>>>,
) where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let forward = ArrangementDartKey {
        edge: edge.clone(),
        direction: ArrangementDirection::Forward,
    };
    let reverse = ArrangementDartKey {
        edge,
        direction: ArrangementDirection::Reverse,
    };
    endpoints.insert(forward.clone(), (start.clone(), end.clone()));
    endpoints.insert(reverse.clone(), (end.clone(), start.clone()));
    incidence.entry(start).or_default().insert(forward);
    incidence.entry(end).or_default().insert(reverse);
}

fn validate_degree<S, C, V>(
    endpoint: &V,
    source: usize,
    cut: usize,
) -> Result<(), FaceArrangementError<S, C, V>>
where
    V: Clone,
{
    match source {
        0 => match cut {
            1 => Err(FaceArrangementError::OpenCutEndpoint(endpoint.clone())),
            2 => Ok(()),
            _ => Err(FaceArrangementError::BranchedCutEndpoint(endpoint.clone())),
        },
        1 => Err(FaceArrangementError::OpenSourceBoundary(endpoint.clone())),
        2 => {
            if cut <= 2 {
                Ok(())
            } else {
                Err(FaceArrangementError::BranchedCutEndpoint(endpoint.clone()))
            }
        }
        _ => Err(FaceArrangementError::BranchedSourceBoundary(
            endpoint.clone(),
        )),
    }
}

fn canonicalize_rotations<S, C, V>(
    rotations: Vec<CertifiedEndpointRotation<S, C, V>>,
    incidence: &BTreeMap<V, BTreeSet<ArrangementDartKey<S, C>>>,
) -> FaceResult<S, C, V, EndpointRotations<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let mut result = BTreeMap::new();
    for rotation in rotations {
        if !incidence.contains_key(&rotation.endpoint) {
            return Err(FaceArrangementError::UnexpectedEndpointRotation(
                rotation.endpoint,
            ));
        }
        if result.contains_key(&rotation.endpoint) {
            return Err(FaceArrangementError::DuplicateEndpointRotation(
                rotation.endpoint,
            ));
        }
        let actual: BTreeSet<_> = rotation.outgoing.iter().cloned().collect();
        if actual.len() != rotation.outgoing.len()
            || incidence.get(&rotation.endpoint) != Some(&actual)
        {
            return Err(FaceArrangementError::EndpointIncidenceMismatch(
                rotation.endpoint,
            ));
        }
        let mut outgoing = rotation.outgoing;
        if let Some((offset, _)) = outgoing.iter().enumerate().min_by_key(|(_, dart)| *dart) {
            outgoing.rotate_left(offset);
        }
        result.insert(rotation.endpoint, outgoing);
    }
    for endpoint in incidence.keys() {
        if !result.contains_key(endpoint) {
            return Err(FaceArrangementError::MissingEndpointRotation(
                endpoint.clone(),
            ));
        }
    }
    Ok(result)
}

fn ensure_primal_connected<S, C, V>(
    input: &CanonicalInput<S, C, V>,
) -> Result<(), FaceArrangementError<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let Some(start) = input.degrees.keys().next().cloned() else {
        return Err(FaceArrangementError::SourceBoundaryRequired);
    };
    let mut adjacent: BTreeMap<V, BTreeSet<V>> = BTreeMap::new();
    for (dart, (from, to)) in &input.endpoints {
        if dart.direction == ArrangementDirection::Forward {
            adjacent.entry(from.clone()).or_default().insert(to.clone());
            adjacent.entry(to.clone()).or_default().insert(from.clone());
        }
    }
    let mut pending = vec![start];
    let mut visited = BTreeSet::new();
    while let Some(vertex) = pending.pop() {
        if !visited.insert(vertex.clone()) {
            continue;
        }
        if let Some(neighbors) = adjacent.get(&vertex) {
            pending.extend(neighbors.iter().cloned());
        }
    }
    if visited.len() == input.degrees.len() {
        Ok(())
    } else {
        Err(FaceArrangementError::DisconnectedPrimal)
    }
}

fn traverse_cycles<S, C, V>(
    input: &CanonicalInput<S, C, V>,
) -> FaceResult<S, C, V, Vec<ArrangementCycle<S, C, V>>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let mut unvisited: BTreeSet<_> = input.endpoints.keys().cloned().collect();
    let mut cycles = Vec::new();
    while let Some(first) = unvisited.iter().next().cloned() {
        let mut uses = Vec::new();
        let mut current = first.clone();
        loop {
            if !unvisited.remove(&current) {
                return Err(FaceArrangementError::CycleDoesNotClose(first));
            }
            uses.push(current.clone());
            let Some(next) = face_successor(&current, input) else {
                return Err(FaceArrangementError::CycleDoesNotClose(first));
            };
            if next == first {
                break;
            }
            current = next;
        }
        cycles.push(canonical_cycle(uses, &input.endpoints)?);
    }
    Ok(cycles)
}

fn face_successor<S, C, V>(
    dart: &ArrangementDartKey<S, C>,
    input: &CanonicalInput<S, C, V>,
) -> Option<ArrangementDartKey<S, C>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let (_, end) = input.endpoints.get(dart)?;
    let twin = dart.opposite();
    let rotation = input.rotations.get(end)?;
    let twin_index = rotation.iter().position(|candidate| candidate == &twin)?;
    let next_index = if twin_index == 0 {
        rotation.len().checked_sub(1)?
    } else {
        twin_index - 1
    };
    rotation.get(next_index).cloned()
}

fn canonical_cycle<S, C, V>(
    mut uses: Vec<ArrangementDartKey<S, C>>,
    endpoints: &BTreeMap<ArrangementDartKey<S, C>, (V, V)>,
) -> Result<ArrangementCycle<S, C, V>, FaceArrangementError<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
    V: Clone + Ord,
{
    let Some(first) = uses.first().cloned() else {
        return Err(FaceArrangementError::EmptyCycle);
    };
    let offset = uses
        .iter()
        .enumerate()
        .min_by_key(|(_, dart)| *dart)
        .map(|(index, _)| index)
        .ok_or_else(|| FaceArrangementError::CycleDoesNotClose(first.clone()))?;
    uses.rotate_left(offset);
    let mut vertices = Vec::with_capacity(uses.len() + 1);
    for dart in &uses {
        let Some((start, _)) = endpoints.get(dart) else {
            return Err(FaceArrangementError::CycleDoesNotClose(first));
        };
        vertices.push(start.clone());
    }
    let Some(last) = uses.last() else {
        return Err(FaceArrangementError::CycleDoesNotClose(first));
    };
    let Some((_, closing)) = endpoints.get(last) else {
        return Err(FaceArrangementError::CycleDoesNotClose(first));
    };
    if vertices.first() != Some(closing) {
        return Err(FaceArrangementError::CycleDoesNotClose(first));
    }
    vertices.push(closing.clone());
    Ok(ArrangementCycle { uses, vertices })
}

fn index_cycle_uses<S, C, V>(
    cycles: &[ArrangementCycle<S, C, V>],
) -> BTreeMap<ArrangementDartKey<S, C>, usize>
where
    S: Clone + Ord,
    C: Clone + Ord,
{
    let mut result = BTreeMap::new();
    for (cycle_index, cycle) in cycles.iter().enumerate() {
        for dart in &cycle.uses {
            result.insert(dart.clone(), cycle_index);
        }
    }
    result
}

fn mark_and_validate_exterior<S, C, V>(
    sources: &[DirectedSourceSpan<S, V>],
    cycle_of: &BTreeMap<ArrangementDartKey<S, C>, usize>,
    exterior: &mut [bool],
) -> Result<(), FaceArrangementError<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
{
    for source in sources {
        let forward = ArrangementDartKey::source(source.key.clone(), ArrangementDirection::Forward);
        let reverse = ArrangementDartKey::source(source.key.clone(), ArrangementDirection::Reverse);
        let Some(&forward_cycle) = cycle_of.get(&forward) else {
            return Err(FaceArrangementError::CycleDoesNotClose(forward));
        };
        let Some(&reverse_cycle) = cycle_of.get(&reverse) else {
            return Err(FaceArrangementError::CycleDoesNotClose(reverse));
        };
        if forward_cycle == reverse_cycle {
            return Err(FaceArrangementError::SourceSidesShareCycle(
                source.key.clone(),
            ));
        }
        let Some(flag) = exterior.get_mut(reverse_cycle) else {
            return Err(FaceArrangementError::CycleDoesNotClose(reverse));
        };
        *flag = true;
    }
    Ok(())
}

fn build_cut_adjacency<S, C, V>(
    cuts: &[DirectedCutFragment<C, V>],
    cycle_of: &BTreeMap<ArrangementDartKey<S, C>, usize>,
    cycle_to_cell: &BTreeMap<usize, usize>,
    exterior: &[bool],
) -> Result<Vec<ArrangementCutAdjacency<C>>, FaceArrangementError<S, C, V>>
where
    S: Clone + Ord,
    C: Clone + Ord,
{
    let mut adjacency = Vec::with_capacity(cuts.len());
    for cut in cuts {
        let forward = ArrangementDartKey::cut(cut.key.clone(), ArrangementDirection::Forward);
        let reverse = ArrangementDartKey::cut(cut.key.clone(), ArrangementDirection::Reverse);
        let Some(&forward_cycle) = cycle_of.get(&forward) else {
            return Err(FaceArrangementError::CycleDoesNotClose(forward));
        };
        let Some(&reverse_cycle) = cycle_of.get(&reverse) else {
            return Err(FaceArrangementError::CycleDoesNotClose(reverse));
        };
        if exterior.get(forward_cycle).copied().unwrap_or(true)
            || exterior.get(reverse_cycle).copied().unwrap_or(true)
        {
            return Err(FaceArrangementError::CutTouchesExterior(cut.key.clone()));
        }
        if forward_cycle == reverse_cycle {
            return Err(FaceArrangementError::CutDoesNotSeparateCells(
                cut.key.clone(),
            ));
        }
        let Some(&forward_cell) = cycle_to_cell.get(&forward_cycle) else {
            return Err(FaceArrangementError::CutTouchesExterior(cut.key.clone()));
        };
        let Some(&reverse_cell) = cycle_to_cell.get(&reverse_cycle) else {
            return Err(FaceArrangementError::CutTouchesExterior(cut.key.clone()));
        };
        adjacency.push(ArrangementCutAdjacency {
            cut: cut.key.clone(),
            forward_cell,
            reverse_cell,
        });
    }
    Ok(adjacency)
}

fn dual_is_connected<C>(cell_count: usize, adjacency: &[ArrangementCutAdjacency<C>]) -> bool {
    if cell_count == 0 {
        return false;
    }
    let mut neighbors = vec![Vec::new(); cell_count];
    for edge in adjacency {
        let Some(forward) = neighbors.get_mut(edge.forward_cell) else {
            return false;
        };
        forward.push(edge.reverse_cell);
        let Some(reverse) = neighbors.get_mut(edge.reverse_cell) else {
            return false;
        };
        reverse.push(edge.forward_cell);
    }
    let mut visited = vec![false; cell_count];
    let mut pending = VecDeque::from([0]);
    while let Some(cell) = pending.pop_front() {
        let Some(flag) = visited.get_mut(cell) else {
            return false;
        };
        if *flag {
            continue;
        }
        *flag = true;
        if let Some(next) = neighbors.get(cell) {
            pending.extend(next.iter().copied());
        }
    }
    visited.into_iter().all(|value| value)
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestInput = FaceArrangementInput<u8, u8, u8>;
    type TestDart = ArrangementDartKey<u8, u8>;

    fn source(key: u8, direction: ArrangementDirection) -> TestDart {
        ArrangementDartKey::source(key, direction)
    }

    fn cut(key: u8, direction: ArrangementDirection) -> TestDart {
        ArrangementDartKey::cut(key, direction)
    }

    /// Independent convex-polygon rotation oracle.  Chords must be directed
    /// from their first endpoint to their second and must not cross.
    fn polygon_with_chords(vertex_count: u8, chords: &[(u8, u8, u8)]) -> TestInput {
        let source_spans = (0..vertex_count)
            .map(|vertex| DirectedSourceSpan::new(vertex, vertex, (vertex + 1) % vertex_count))
            .collect();
        let cut_fragments = chords
            .iter()
            .map(|(key, start, end)| DirectedCutFragment::new(*key, *start, *end))
            .collect();
        let rotations = (0..vertex_count)
            .map(|vertex| {
                let mut outgoing = vec![source(vertex, ArrangementDirection::Forward)];
                for (key, start, end) in chords {
                    if *start == vertex {
                        outgoing.push(cut(*key, ArrangementDirection::Forward));
                    } else if *end == vertex {
                        outgoing.push(cut(*key, ArrangementDirection::Reverse));
                    }
                }
                outgoing.push(source(
                    (vertex + vertex_count - 1) % vertex_count,
                    ArrangementDirection::Reverse,
                ));
                CertifiedEndpointRotation::new(vertex, outgoing)
            })
            .collect();
        FaceArrangementInput::new(source_spans, cut_fragments, rotations)
    }

    fn annulus_with_closed_cuts(cut_loop_count: u8) -> TestInput {
        let source_spans = vec![
            DirectedSourceSpan::whole_loop(0, 0),
            DirectedSourceSpan::whole_loop(3, 3),
        ];
        let mut cut_fragments = Vec::new();
        let mut rotations = vec![
            CertifiedEndpointRotation::new(
                0,
                vec![
                    source(0, ArrangementDirection::Forward),
                    source(0, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                3,
                vec![
                    source(3, ArrangementDirection::Forward),
                    source(3, ArrangementDirection::Reverse),
                ],
            ),
        ];
        for loop_index in 0..cut_loop_count {
            let key_base = 10 + 3 * loop_index;
            let vertex_base = 20 + 3 * loop_index;
            for offset in 0..3 {
                let next_vertex = vertex_base + (offset + 1) % 3;
                let previous_key = key_base + (offset + 2) % 3;
                cut_fragments.push(DirectedCutFragment::new(
                    key_base + offset,
                    vertex_base + offset,
                    next_vertex,
                ));
                rotations.push(CertifiedEndpointRotation::new(
                    vertex_base + offset,
                    vec![
                        cut(key_base + offset, ArrangementDirection::Forward),
                        cut(previous_key, ArrangementDirection::Reverse),
                    ],
                ));
            }
        }
        FaceArrangementInput::new(source_spans, cut_fragments, rotations)
    }

    fn annulus_with_whole_ring() -> TestInput {
        let mut input = annulus_with_closed_cuts(0);
        input
            .cut_fragments
            .push(DirectedCutFragment::whole_loop(10, 10));
        input.rotations.push(CertifiedEndpointRotation::new(
            10,
            vec![
                cut(10, ArrangementDirection::Forward),
                cut(10, ArrangementDirection::Reverse),
            ],
        ));
        input
    }

    fn source_cycle_assignments(
        lower_cell: u8,
        upper_cell: u8,
    ) -> Vec<CertifiedCycleAssignment<u8, u8, u8>> {
        vec![
            CertifiedCycleAssignment::new(
                source(0, ArrangementDirection::Forward),
                CertifiedCycleSide::Cell(lower_cell),
            ),
            CertifiedCycleAssignment::new(
                source(0, ArrangementDirection::Reverse),
                CertifiedCycleSide::Exterior,
            ),
            CertifiedCycleAssignment::new(
                source(3, ArrangementDirection::Forward),
                CertifiedCycleSide::Cell(upper_cell),
            ),
            CertifiedCycleAssignment::new(
                source(3, ArrangementDirection::Reverse),
                CertifiedCycleSide::Exterior,
            ),
        ]
    }

    fn contractible_cut_embedding(cut_loop_count: u8) -> CertifiedSurfaceEmbedding<u8, u8, u8> {
        let mut assignments = source_cycle_assignments(0, 0);
        let mut cells = vec![CertifiedCellTopology::new(0, -i64::from(cut_loop_count))];
        for loop_index in 0..cut_loop_count {
            let key_base = 10 + 3 * loop_index;
            let disk = loop_index + 1;
            assignments.push(CertifiedCycleAssignment::new(
                cut(key_base, ArrangementDirection::Forward),
                CertifiedCycleSide::Cell(disk),
            ));
            assignments.push(CertifiedCycleAssignment::new(
                cut(key_base, ArrangementDirection::Reverse),
                CertifiedCycleSide::Cell(0),
            ));
            cells.push(CertifiedCellTopology::new(disk, 1));
        }
        CertifiedSurfaceEmbedding::new(assignments, cells, 0)
    }

    #[test]
    fn annulus_without_cuts_is_one_two_boundary_cell() {
        let embedding = CertifiedSurfaceEmbedding::new(
            source_cycle_assignments(0, 0),
            vec![CertifiedCellTopology::new(0, 0)],
            0,
        );
        assert!(matches!(
            embedding.assignments[0].side(),
            CertifiedCycleSide::Cell(0)
        ));
        assert_eq!(
            embedding.assignments[0].anchor(),
            &source(0, ArrangementDirection::Forward)
        );
        assert_eq!(*embedding.cells[0].key(), 0);
        assert_eq!(embedding.cells[0].euler_characteristic(), 0);

        let arrangement = arrange_bounded_surface(annulus_with_closed_cuts(0), embedding)
            .expect("an annulus is one cell bounded by two exact source cycles");

        assert_eq!(arrangement.source_spans().len(), 2);
        assert!(
            arrangement
                .source_spans()
                .iter()
                .all(DirectedSourceSpan::is_whole_loop)
        );
        assert!(arrangement.cut_fragments().is_empty());
        assert_eq!(arrangement.cells().len(), 1);
        assert_eq!(*arrangement.cells()[0].key(), 0);
        assert_eq!(arrangement.cells()[0].boundaries().len(), 2);
        assert_eq!(arrangement.cells()[0].euler_characteristic(), 0);
        assert_eq!(arrangement.cells()[0].genus(), 0);
        assert!(arrangement.adjacency().is_empty());

        let proof = arrangement.proof();
        assert_eq!(proof.endpoint_degrees().len(), 2);
        assert_eq!(proof.directed_darts_conserved(), 4);
        assert_eq!(proof.source_spans_conserved(), 2);
        assert_eq!(proof.opposed_cut_pairs(), 0);
        assert_eq!(proof.closed_cycles(), 4);
        assert_eq!(proof.exterior_cycles(), 2);
        assert_eq!(proof.primal_components(), 2);
        assert_eq!(proof.source_boundary_components(), 2);
        assert_eq!(proof.cell_genera(), &[(0, 0)]);
        assert!(proof.dual_connected());
        assert_eq!(proof.surface_euler_characteristic(), 0);
        assert_eq!(proof.surface_genus(), 0);
    }

    #[test]
    fn noncontractible_ring_splits_annulus_into_two_annular_cells() {
        let mut assignments = source_cycle_assignments(0, 1);
        assignments.extend([
            CertifiedCycleAssignment::new(
                cut(10, ArrangementDirection::Forward),
                CertifiedCycleSide::Cell(0),
            ),
            CertifiedCycleAssignment::new(
                cut(10, ArrangementDirection::Reverse),
                CertifiedCycleSide::Cell(1),
            ),
        ]);
        let embedding = CertifiedSurfaceEmbedding::new(
            assignments,
            vec![
                CertifiedCellTopology::new(0, 0),
                CertifiedCellTopology::new(1, 0),
            ],
            0,
        );

        let arrangement = arrange_bounded_surface(annulus_with_whole_ring(), embedding)
            .expect("an essential ring separates the two source boundaries");

        assert_eq!(arrangement.cells().len(), 2);
        assert!(
            arrangement
                .cells()
                .iter()
                .all(|cell| cell.boundaries().len() == 2 && cell.euler_characteristic() == 0)
        );
        assert_eq!(arrangement.cut_fragments().len(), 1);
        assert!(arrangement.cut_fragments()[0].is_whole_loop());
        assert_eq!(arrangement.adjacency().len(), 1);
        assert!(arrangement.adjacency().iter().all(|edge| {
            *edge.cut() >= 10 && edge.forward_cell() == &0 && edge.reverse_cell() == &1
        }));
        assert_eq!(arrangement.proof().primal_components(), 3);
        assert_eq!(arrangement.proof().opposed_cut_pairs(), 1);
    }

    #[test]
    fn one_and_two_contractible_cycles_form_valid_multicycle_cells() {
        for (cut_loops, expected_boundaries, expected_cells) in [(1, 3, 2), (2, 4, 3)] {
            let arrangement = arrange_bounded_surface(
                annulus_with_closed_cuts(cut_loops),
                contractible_cut_embedding(cut_loops),
            )
            .expect("contractible closed cuts have proof-owned nesting");

            assert_eq!(arrangement.cells().len(), expected_cells);
            let outer = arrangement
                .cells()
                .iter()
                .find(|cell| cell.key() == &0)
                .expect("outer cell");
            assert_eq!(outer.boundaries().len(), expected_boundaries);
            assert_eq!(outer.euler_characteristic(), -i64::from(cut_loops));
            assert_eq!(outer.genus(), 0);
            assert!(
                arrangement
                    .cells()
                    .iter()
                    .filter(|cell| cell.key() != &0)
                    .all(|cell| cell.boundaries().len() == 1 && cell.euler_characteristic() == 1)
            );
            assert_eq!(arrangement.adjacency().len(), usize::from(cut_loops) * 3);
            assert_eq!(
                arrangement.proof().primal_components(),
                usize::from(cut_loops) + 2
            );
            assert_eq!(arrangement.proof().surface_euler_characteristic(), 0);
        }
    }

    #[test]
    fn annular_arrangement_ignores_input_order_and_cycle_anchor_period_shift() {
        let input = annulus_with_closed_cuts(2);
        let embedding = contractible_cut_embedding(2);
        let expected = arrange_bounded_surface(input.clone(), embedding.clone())
            .expect("reference annular arrangement");

        let mut permuted_input = input;
        permuted_input.source_spans.reverse();
        permuted_input.cut_fragments.reverse();
        permuted_input.rotations.reverse();
        for rotation in &mut permuted_input.rotations {
            rotation.outgoing.rotate_left(1);
        }
        let mut shifted_embedding = embedding;
        shifted_embedding.assignments.reverse();
        shifted_embedding.cells.reverse();
        for assignment in &mut shifted_embedding.assignments {
            assignment.anchor = match &assignment.anchor.edge {
                ArrangementEdgeKey::Source(key) => source(*key, assignment.anchor.direction),
                ArrangementEdgeKey::Cut(key) => {
                    let base = 10 + ((*key - 10) / 3) * 3;
                    cut(base + (*key - base + 1) % 3, assignment.anchor.direction)
                }
            };
        }

        let actual = arrange_bounded_surface(permuted_input, shifted_embedding)
            .expect("period-shifted anchors identify the same exact cycles");
        assert_eq!(actual, expected);
    }

    #[test]
    fn wrong_nesting_and_disconnected_dual_proofs_are_refused() {
        let mut wrong_topology = contractible_cut_embedding(1);
        wrong_topology.cells[0].euler_characteristic = 0;
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(1), wrong_topology),
            Err(SurfaceArrangementError::CellTopologyInconsistent {
                cell: 0,
                boundary_cycles: 3,
                euler_characteristic: 0,
            })
        );

        let mut disconnected_assignments = source_cycle_assignments(0, 0);
        disconnected_assignments.extend([
            CertifiedCycleAssignment::new(
                cut(10, ArrangementDirection::Forward),
                CertifiedCycleSide::Cell(1),
            ),
            CertifiedCycleAssignment::new(
                cut(10, ArrangementDirection::Reverse),
                CertifiedCycleSide::Cell(2),
            ),
        ]);
        let disconnected_embedding = CertifiedSurfaceEmbedding::new(
            disconnected_assignments,
            vec![
                CertifiedCellTopology::new(0, 0),
                CertifiedCellTopology::new(1, 1),
                CertifiedCellTopology::new(2, 1),
            ],
            0,
        );
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(1), disconnected_embedding),
            Err(SurfaceArrangementError::DisconnectedDual)
        );

        let mut same_side = contractible_cut_embedding(1);
        same_side.assignments[4].side = CertifiedCycleSide::Cell(0);
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(1), same_side),
            Err(SurfaceArrangementError::CutDoesNotSeparateCells(10))
        );
    }

    #[test]
    fn missing_duplicate_and_globally_inconsistent_embedding_proofs_are_refused() {
        let mut missing = CertifiedSurfaceEmbedding::new(
            source_cycle_assignments(0, 0),
            vec![CertifiedCellTopology::new(0, 0)],
            0,
        );
        missing.assignments.pop();
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(0), missing),
            Err(SurfaceArrangementError::MissingCycleAssignment(source(
                3,
                ArrangementDirection::Reverse,
            )))
        );

        let mut duplicate = CertifiedSurfaceEmbedding::new(
            source_cycle_assignments(0, 0),
            vec![CertifiedCellTopology::new(0, 0)],
            0,
        );
        duplicate.assignments.push(CertifiedCycleAssignment::new(
            source(0, ArrangementDirection::Forward),
            CertifiedCycleSide::Cell(0),
        ));
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(0), duplicate),
            Err(SurfaceArrangementError::DuplicateCycleAssignment(source(
                0,
                ArrangementDirection::Forward,
            )))
        );

        let invalid_surface = CertifiedSurfaceEmbedding::new(
            source_cycle_assignments(0, 0),
            vec![CertifiedCellTopology::new(0, 0)],
            1,
        );
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(0), invalid_surface),
            Err(SurfaceArrangementError::SurfaceTopologyInconsistent {
                boundary_components: 2,
                euler_characteristic: 1,
            })
        );

        let genus_one_claim = CertifiedSurfaceEmbedding::new(
            source_cycle_assignments(0, 0),
            vec![CertifiedCellTopology::new(0, 0)],
            -2,
        );
        assert_eq!(
            arrange_bounded_surface(annulus_with_closed_cuts(0), genus_one_claim),
            Err(SurfaceArrangementError::SurfaceEulerMismatch {
                expected: -2,
                actual: 0,
            })
        );
    }

    #[test]
    fn diagonal_partitions_polygon_and_proves_conservation() {
        let input = polygon_with_chords(4, &[(9, 0, 2)]);
        assert_eq!(*input.source_spans[0].key(), 0);
        assert_eq!(input.source_spans[0].endpoints(), [&0, &1]);
        assert_eq!(*input.cut_fragments[0].key(), 9);
        assert_eq!(input.cut_fragments[0].endpoints(), [&0, &2]);
        assert_eq!(*input.rotations[0].endpoint(), 0);
        assert_eq!(input.rotations[0].outgoing().len(), 3);
        assert!(matches!(
            input.rotations[0].outgoing()[0].edge(),
            ArrangementEdgeKey::Source(0)
        ));
        assert_eq!(
            input.rotations[0].outgoing()[0].direction(),
            ArrangementDirection::Forward
        );

        let arrangement =
            arrange_bounded_face(input).expect("a certified diagonal partitions a convex polygon");

        assert_eq!(arrangement.source_spans().len(), 4);
        assert_eq!(arrangement.cut_fragments().len(), 1);
        assert_eq!(arrangement.cells().len(), 2);
        assert_eq!(arrangement.adjacency().len(), 1);
        assert_ne!(
            arrangement.adjacency()[0].forward_cell(),
            arrangement.adjacency()[0].reverse_cell()
        );
        assert_eq!(*arrangement.adjacency()[0].cut(), 9);

        let proof = arrangement.proof();
        assert_eq!(proof.source_spans_conserved(), 4);
        assert_eq!(proof.opposed_cut_pairs(), 1);
        assert_eq!(proof.closed_cycles(), 3);
        assert_eq!(proof.exterior_cycles(), 1);
        assert!(proof.dual_connected());
        assert_eq!(proof.euler_characteristic(), 2);
        assert_eq!(proof.endpoint_degrees().len(), 4);
        assert_eq!(proof.endpoint_degrees()[0].1.total(), 3);
        assert_eq!(proof.endpoint_degrees()[0].1.source(), 2);
        assert_eq!(proof.endpoint_degrees()[0].1.cut(), 1);

        for cell in arrangement.cells() {
            assert_eq!(cell.key(), arrangement.cells()[cell.key()].key());
            let vertices = cell.boundary().vertices();
            assert_eq!(vertices.first(), vertices.last());
            assert_eq!(vertices.len(), cell.boundary().uses().len() + 1);
        }
    }

    #[test]
    fn opposed_cut_continuation_at_boundary_root_is_not_a_branch() {
        let source_spans = (0..5)
            .map(|vertex| DirectedSourceSpan::new(vertex, vertex, (vertex + 1) % 5))
            .collect();
        let cuts = vec![
            DirectedCutFragment::new(8, 2, 0),
            DirectedCutFragment::new(9, 0, 3),
        ];
        let rotations = vec![
            CertifiedEndpointRotation::new(
                0,
                vec![
                    source(0, ArrangementDirection::Forward),
                    cut(8, ArrangementDirection::Reverse),
                    cut(9, ArrangementDirection::Forward),
                    source(4, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                1,
                vec![
                    source(1, ArrangementDirection::Forward),
                    source(0, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                2,
                vec![
                    source(2, ArrangementDirection::Forward),
                    cut(8, ArrangementDirection::Forward),
                    source(1, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                3,
                vec![
                    source(3, ArrangementDirection::Forward),
                    cut(9, ArrangementDirection::Reverse),
                    source(2, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                4,
                vec![
                    source(4, ArrangementDirection::Forward),
                    source(3, ArrangementDirection::Reverse),
                ],
            ),
        ];

        let arrangement =
            arrange_bounded_face(FaceArrangementInput::new(source_spans, cuts, rotations))
                .expect("opposed cut darts continue through the exact boundary root");
        assert_eq!(arrangement.cells().len(), 3);
        assert_eq!(arrangement.adjacency().len(), 2);
        assert_eq!(arrangement.proof().endpoint_degrees()[0].1.total(), 4);
    }

    #[test]
    fn two_non_crossing_chords_produce_three_dual_connected_cells() {
        let arrangement = arrange_bounded_face(polygon_with_chords(8, &[(10, 0, 3), (20, 4, 7)]))
            .expect("two disjoint chords partition a convex polygon");

        assert_eq!(arrangement.cells().len(), 3);
        assert_eq!(arrangement.adjacency().len(), 2);
        assert_eq!(arrangement.proof().closed_cycles(), 4);
        assert_eq!(arrangement.proof().source_spans_conserved(), 8);
        assert_eq!(arrangement.proof().opposed_cut_pairs(), 2);

        let incident_cells: BTreeSet<_> = arrangement
            .adjacency()
            .iter()
            .flat_map(|edge| [edge.forward_cell(), edge.reverse_cell()])
            .collect();
        assert_eq!(incident_cells, BTreeSet::from([0, 1, 2]));
    }

    #[test]
    fn input_permutation_and_cyclic_rotation_do_not_change_output() {
        let original = polygon_with_chords(8, &[(10, 0, 3), (20, 4, 7)]);
        let expected = arrange_bounded_face(original.clone()).expect("reference arrangement");

        let mut permuted = original;
        permuted.source_spans.reverse();
        permuted.cut_fragments.reverse();
        permuted.rotations.reverse();
        for rotation in &mut permuted.rotations {
            if !rotation.outgoing.is_empty() {
                rotation.outgoing.rotate_left(1);
            }
        }
        let actual = arrange_bounded_face(permuted).expect("permuted arrangement");

        assert_eq!(actual, expected);
    }

    #[test]
    fn disconnected_source_components_are_refused() {
        let first = polygon_with_chords(3, &[]);
        let second_sources = (3..6).map(|vertex| {
            DirectedSourceSpan::new(vertex, vertex, if vertex == 5 { 3 } else { vertex + 1 })
        });
        let second_rotations = (3..6).map(|vertex| {
            CertifiedEndpointRotation::new(
                vertex,
                vec![
                    source(vertex, ArrangementDirection::Forward),
                    source(
                        if vertex == 3 { 5 } else { vertex - 1 },
                        ArrangementDirection::Reverse,
                    ),
                ],
            )
        });
        let input = FaceArrangementInput::new(
            first
                .source_spans
                .into_iter()
                .chain(second_sources)
                .collect(),
            Vec::new(),
            first
                .rotations
                .into_iter()
                .chain(second_rotations)
                .collect(),
        );

        assert_eq!(
            arrange_bounded_face(input),
            Err(FaceArrangementError::DisconnectedPrimal)
        );
    }

    #[test]
    fn branched_cut_endpoint_is_refused_before_embedding() {
        let input = polygon_with_chords(4, &[(7, 0, 1), (8, 0, 2), (9, 0, 3)]);
        assert_eq!(
            arrange_bounded_face(input),
            Err(FaceArrangementError::BranchedCutEndpoint(0))
        );
    }

    #[test]
    fn incomplete_cut_endpoint_is_refused_before_embedding() {
        let mut input = polygon_with_chords(4, &[]);
        input.cut_fragments.push(DirectedCutFragment::new(9, 0, 10));
        input.rotations[0]
            .outgoing
            .insert(1, cut(9, ArrangementDirection::Forward));
        input.rotations.push(CertifiedEndpointRotation::new(
            10,
            vec![cut(9, ArrangementDirection::Reverse)],
        ));

        assert_eq!(
            arrange_bounded_face(input),
            Err(FaceArrangementError::OpenCutEndpoint(10))
        );
    }

    #[test]
    fn bridge_cut_that_lacks_two_interior_sides_is_refused() {
        let source_spans = vec![
            DirectedSourceSpan::new(0, 0, 1),
            DirectedSourceSpan::new(1, 1, 2),
            DirectedSourceSpan::new(2, 2, 0),
            DirectedSourceSpan::new(3, 3, 4),
            DirectedSourceSpan::new(4, 4, 5),
            DirectedSourceSpan::new(5, 5, 3),
        ];
        let cut_fragments = vec![DirectedCutFragment::new(9, 1, 3)];
        let rotations = vec![
            CertifiedEndpointRotation::new(
                0,
                vec![
                    source(0, ArrangementDirection::Forward),
                    source(2, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                1,
                vec![
                    cut(9, ArrangementDirection::Forward),
                    source(1, ArrangementDirection::Forward),
                    source(0, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                2,
                vec![
                    source(2, ArrangementDirection::Forward),
                    source(1, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                3,
                vec![
                    source(3, ArrangementDirection::Forward),
                    source(5, ArrangementDirection::Reverse),
                    cut(9, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                4,
                vec![
                    source(4, ArrangementDirection::Forward),
                    source(3, ArrangementDirection::Reverse),
                ],
            ),
            CertifiedEndpointRotation::new(
                5,
                vec![
                    source(5, ArrangementDirection::Forward),
                    source(4, ArrangementDirection::Reverse),
                ],
            ),
        ];

        assert_eq!(
            arrange_bounded_face(FaceArrangementInput::new(
                source_spans,
                cut_fragments,
                rotations,
            )),
            Err(FaceArrangementError::CutTouchesExterior(9))
        );
    }

    #[test]
    fn missing_and_mutated_rotation_evidence_is_refused() {
        let mut missing = polygon_with_chords(4, &[(9, 0, 2)]);
        missing.rotations.retain(|rotation| rotation.endpoint != 2);
        assert_eq!(
            arrange_bounded_face(missing),
            Err(FaceArrangementError::MissingEndpointRotation(2))
        );

        let mut duplicate_dart = polygon_with_chords(4, &[(9, 0, 2)]);
        duplicate_dart.rotations[0]
            .outgoing
            .push(source(0, ArrangementDirection::Forward));
        assert_eq!(
            arrange_bounded_face(duplicate_dart),
            Err(FaceArrangementError::EndpointIncidenceMismatch(0))
        );

        let mut duplicate_endpoint = polygon_with_chords(4, &[(9, 0, 2)]);
        duplicate_endpoint
            .rotations
            .push(duplicate_endpoint.rotations[0].clone());
        assert_eq!(
            arrange_bounded_face(duplicate_endpoint),
            Err(FaceArrangementError::DuplicateEndpointRotation(0))
        );
    }

    #[test]
    fn reversed_rotation_is_rejected_as_non_planar_evidence() {
        let mut input = polygon_with_chords(4, &[(9, 0, 2)]);
        input.rotations[0].outgoing.reverse();

        assert_eq!(
            arrange_bounded_face(input),
            Err(FaceArrangementError::NonPlanarRotationSystem {
                euler_characteristic: 0,
            })
        );
    }

    #[test]
    fn duplicate_and_degenerate_exact_identities_are_refused() {
        let mut duplicate_source = polygon_with_chords(3, &[]);
        duplicate_source
            .source_spans
            .push(duplicate_source.source_spans[0].clone());
        assert_eq!(
            arrange_bounded_face(duplicate_source),
            Err(FaceArrangementError::DuplicateSourceSpan(0))
        );

        let mut degenerate_cut = polygon_with_chords(3, &[]);
        degenerate_cut
            .cut_fragments
            .push(DirectedCutFragment::new(7, 0, 0));
        assert_eq!(
            arrange_bounded_face(degenerate_cut),
            Err(FaceArrangementError::DegenerateCutFragment(7))
        );
    }
}
