//! Proof-bearing two-dimensional source-face partitions.
//!
//! Boolean truth selection consumes open pieces of source boundary faces, not
//! the one-dimensional section curves that cut them.  This module supplies a
//! representation shared by planar and periodic analytic faces together with
//! the first two exact adapters needed by the block/cylinder ladder:
//!
//! - a convex planar face cut by pairwise-disjoint, endpoint-free circles, and
//! - a full-period cylinder band cut by constant-axial-parameter rings.
//!
//! The adapters are intentionally combinatorial.  Their inputs are minted by
//! an upstream exact certifier: a planar cut is already proven to be a simple
//! circle strictly inside the source face and disjoint from every peer, while
//! an axial cut carries a caller-owned exact order key.  Numeric circle and
//! axial values are retained only as later realization evidence.  They never
//! decide identity, order, incidence, or adjacency here.

use std::collections::{BTreeMap, BTreeSet};

/// Direction of one boundary use relative to its proof-owned source use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum FaceBoundaryOrientation {
    Forward,
    Reversed,
}

/// Exact identity of one complete boundary use.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum FaceBoundaryKey<S, C> {
    /// One use inherited from the source face boundary.
    Source(S),
    /// One use of a certified section cut.
    Cut(C),
}

/// One oriented occurrence of a proof-owned boundary use.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct FaceBoundaryUse<S, C> {
    key: FaceBoundaryKey<S, C>,
    orientation: FaceBoundaryOrientation,
}

impl<S, C> FaceBoundaryUse<S, C> {
    const fn source(key: S, orientation: FaceBoundaryOrientation) -> Self {
        Self {
            key: FaceBoundaryKey::Source(key),
            orientation,
        }
    }

    const fn cut(key: C, orientation: FaceBoundaryOrientation) -> Self {
        Self {
            key: FaceBoundaryKey::Cut(key),
            orientation,
        }
    }

    /// Exact source-boundary or section-cut identity.
    pub(crate) const fn key(&self) -> &FaceBoundaryKey<S, C> {
        &self.key
    }

    /// Direction relative to the proof-owned source use.
    pub(crate) const fn orientation(&self) -> FaceBoundaryOrientation {
        self.orientation
    }
}

/// Topological role of one closed cell-boundary cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum FaceBoundaryCycleRole {
    /// Positively oriented outer cycle of a planar cell.
    PlanarOuter,
    /// Negatively oriented hole cycle of a planar cell.
    PlanarHole,
    /// Lower constant-axial boundary of a periodic band cell.
    AxialLower,
    /// Upper constant-axial boundary of a periodic band cell.
    AxialUpper,
}

/// One closed cycle bounding a two-dimensional face cell.
///
/// A whole ring is one use in one cycle.  There is no vertex or chart-seam
/// field, so an endpoint-free periodic cut cannot accidentally acquire a
/// physical seam vertex in this representation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FaceBoundaryCycle<S, C> {
    role: FaceBoundaryCycleRole,
    uses: Vec<FaceBoundaryUse<S, C>>,
}

impl<S, C> FaceBoundaryCycle<S, C> {
    /// Surface-specific role of this cycle.
    pub(crate) const fn role(&self) -> FaceBoundaryCycleRole {
        self.role
    }

    /// Oriented proof-owned uses in traversal order.
    pub(crate) fn uses(&self) -> &[FaceBoundaryUse<S, C>] {
        &self.uses
    }
}

/// Exact lower or upper boundary of a periodic axial band.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum AxialBoundary<C> {
    LowerSource,
    Cut(C),
    UpperSource,
}

/// Stable identity of one open source-face cell.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum FaceRegionKey<C> {
    /// The planar source region outside every admitted circle.
    PlanarOuter,
    /// The open disk bounded by one admitted circle.
    PlanarDisk(C),
    /// One full-period cylinder band between consecutive exact boundaries.
    AxialBand {
        lower: AxialBoundary<C>,
        upper: AxialBoundary<C>,
    },
}

/// Stable source-face-qualified identity of one two-dimensional cell.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct FaceCellKey<F, C> {
    face: F,
    region: FaceRegionKey<C>,
}

impl<F, C> FaceCellKey<F, C> {
    fn new(face: F, region: FaceRegionKey<C>) -> Self {
        Self { face, region }
    }

    /// Source face carrying the cell.
    pub(crate) const fn face(&self) -> &F {
        &self.face
    }

    /// Exact region identity within the source face.
    pub(crate) const fn region(&self) -> &FaceRegionKey<C> {
        &self.region
    }
}

/// One proof-bearing open two-dimensional source-face cell.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FaceCell<F, S, C> {
    key: FaceCellKey<F, C>,
    boundary: Vec<FaceBoundaryCycle<S, C>>,
}

impl<F, S, C> FaceCell<F, S, C> {
    /// Stable source-face-qualified cell identity.
    pub(crate) const fn key(&self) -> &FaceCellKey<F, C> {
        &self.key
    }

    /// Closed cycles bounding the cell.
    pub(crate) fn boundary(&self) -> &[FaceBoundaryCycle<S, C>] {
        &self.boundary
    }
}

/// Two face cells separated by one certified transverse cut.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FaceCutAdjacency<F, C> {
    cut: C,
    cells: [FaceCellKey<F, C>; 2],
}

impl<F: Ord, C> FaceCutAdjacency<F, C>
where
    C: Ord,
{
    fn new(cut: C, mut cells: [FaceCellKey<F, C>; 2]) -> Self {
        cells.sort_unstable();
        Self { cut, cells }
    }

    /// Exact cut identity.
    pub(crate) const fn cut(&self) -> &C {
        &self.cut
    }

    /// Canonically ordered adjacent cell identities.
    pub(crate) const fn cells(&self) -> &[FaceCellKey<F, C>; 2] {
        &self.cells
    }
}

/// One canonical cut record retained for later realization.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FacePartitionCut<C, R> {
    key: C,
    representative: R,
}

impl<C, R> FacePartitionCut<C, R> {
    /// Exact caller-owned cut identity.
    pub(crate) const fn key(&self) -> &C {
        &self.key
    }

    /// Non-authoritative numeric realization evidence.
    pub(crate) const fn representative(&self) -> &R {
        &self.representative
    }
}

/// Complete proof-bearing partition of one source face.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FacePartition<F, S, C, R> {
    face: F,
    source_boundaries: Vec<S>,
    cuts: Vec<FacePartitionCut<C, R>>,
    cells: Vec<FaceCell<F, S, C>>,
    adjacency: Vec<FaceCutAdjacency<F, C>>,
}

impl<F, S, C, R> FacePartition<F, S, C, R> {
    /// Source face partitioned by the adapter.
    pub(crate) const fn face(&self) -> &F {
        &self.face
    }

    /// Canonical source-boundary identities.
    pub(crate) fn source_boundaries(&self) -> &[S] {
        &self.source_boundaries
    }

    /// Canonical cut records.  Adapter semantics define whether this is key
    /// order (planar circles) or certified axial order (periodic rings).
    pub(crate) fn cuts(&self) -> &[FacePartitionCut<C, R>] {
        &self.cuts
    }

    /// Cells in stable key order.
    pub(crate) fn cells(&self) -> &[FaceCell<F, S, C>] {
        &self.cells
    }

    /// Dual cut adjacency in stable cut-key order.
    pub(crate) fn adjacency(&self) -> &[FaceCutAdjacency<F, C>] {
        &self.adjacency
    }
}

/// Numeric plane-chart evidence for one certified whole circular cut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PlanarCircleRepresentative {
    center: [f64; 2],
    radius: f64,
}

impl PlanarCircleRepresentative {
    /// Retain a numeric circle representative.  Admission is checked by the
    /// partition adapter, but the values never decide topology.
    pub(crate) const fn new(center: [f64; 2], radius: f64) -> Self {
        Self { center, radius }
    }

    /// Plane-chart center.
    pub(crate) const fn center(self) -> [f64; 2] {
        self.center
    }

    /// Circle radius.
    pub(crate) const fn radius(self) -> f64 {
        self.radius
    }

    fn valid(self) -> bool {
        self.center.into_iter().all(f64::is_finite) && self.radius.is_finite() && self.radius > 0.0
    }
}

/// Certifier-minted endpoint-free circle strictly inside a convex face.
///
/// The upstream certifier owns the proof that all values supplied to one
/// adapter call are simple, mutually disjoint, and contained in the source
/// face.  This module consumes that proof shape and never reclassifies it from
/// the representative.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CertifiedPlanarCircleCut<C> {
    key: C,
    representative: PlanarCircleRepresentative,
}

impl<C> CertifiedPlanarCircleCut<C> {
    /// Pair an exact cut identity with non-authoritative realization data.
    pub(crate) const fn new(key: C, representative: PlanarCircleRepresentative) -> Self {
        Self {
            key,
            representative,
        }
    }
}

/// Numeric and exact-order evidence retained for one axial whole ring.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AxialRingRepresentative<O> {
    exact_order: O,
    axial_parameter: f64,
}

impl<O> AxialRingRepresentative<O> {
    /// Caller-owned exact ordering key.
    pub(crate) const fn exact_order(&self) -> &O {
        &self.exact_order
    }

    /// Non-authoritative axial-parameter representative.
    pub(crate) const fn axial_parameter(&self) -> f64 {
        self.axial_parameter
    }
}

/// Certifier-minted constant-axial-parameter whole ring.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CertifiedAxialRingCut<C, O> {
    key: C,
    exact_order: O,
    axial_parameter: f64,
}

impl<C, O> CertifiedAxialRingCut<C, O> {
    /// Pair exact identity and order evidence with a numeric representative.
    pub(crate) const fn new(key: C, exact_order: O, axial_parameter: f64) -> Self {
        Self {
            key,
            exact_order,
            axial_parameter,
        }
    }
}

/// Typed fail-closed outcome from face partition construction or validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FacePartitionError {
    EmptySourceBoundary,
    DuplicateSourceBoundaryKey,
    DuplicateCutKey,
    DuplicateCutOrder,
    InvalidCutRepresentative,
    DuplicateCellKey,
    EmptyCellBoundary,
    UnknownSourceBoundaryUse,
    UnknownCutBoundaryUse,
    SourceBoundaryUseCount {
        count: usize,
    },
    CutBoundaryUseCount {
        count: usize,
        forward: usize,
        reversed: usize,
    },
    DuplicateCutAdjacency,
    MissingCutAdjacency,
    UnknownAdjacencyCell,
    SelfAdjacency,
    AdjacencyDoesNotMatchBoundaryUses,
    NonCanonicalOrder,
}

/// Certified constant relation of one open face cell to the other solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FaceCellOpenClassification {
    Interior,
    Exterior,
}

impl FaceCellOpenClassification {
    const fn toggled(self) -> Self {
        match self {
            Self::Interior => Self::Exterior,
            Self::Exterior => Self::Interior,
        }
    }
}

/// Fail-closed outcome from dual-graph classification propagation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FaceCellClassificationError {
    InvalidPartition(FacePartitionError),
    UnknownAnchor,
    ContradictoryAdjacency,
    DisconnectedDualGraph,
}

/// Propagate one certified anchor classification across transverse cuts.
///
/// Every cut toggles open-set occupancy. The partition's exact dual graph is
/// the only propagation authority; representatives are never inspected. A
/// disconnected cell or incompatible parity cycle is an honest proof gap.
pub(crate) fn classify_face_partition_from_anchor<F, S, C, R>(
    partition: &FacePartition<F, S, C, R>,
    anchor: &FaceCellKey<F, C>,
    anchor_classification: FaceCellOpenClassification,
) -> Result<BTreeMap<FaceCellKey<F, C>, FaceCellOpenClassification>, FaceCellClassificationError>
where
    F: Clone + Ord,
    S: Clone + Ord,
    C: Clone + Ord,
{
    partition
        .validate()
        .map_err(FaceCellClassificationError::InvalidPartition)?;
    if !partition.cells.iter().any(|cell| &cell.key == anchor) {
        return Err(FaceCellClassificationError::UnknownAnchor);
    }

    let mut classifications = BTreeMap::from([(anchor.clone(), anchor_classification)]);
    loop {
        let before = classifications.len();
        for adjacency in &partition.adjacency {
            let [first, second] = &adjacency.cells;
            match (
                classifications.get(first).copied(),
                classifications.get(second).copied(),
            ) {
                (Some(first_class), Some(second_class)) => {
                    if first_class == second_class {
                        return Err(FaceCellClassificationError::ContradictoryAdjacency);
                    }
                }
                (Some(classification), None) => {
                    classifications.insert(second.clone(), classification.toggled());
                }
                (None, Some(classification)) => {
                    classifications.insert(first.clone(), classification.toggled());
                }
                (None, None) => {}
            }
        }
        if classifications.len() == before {
            break;
        }
    }
    if classifications.len() != partition.cells.len() {
        return Err(FaceCellClassificationError::DisconnectedDualGraph);
    }
    Ok(classifications)
}

/// Partition a certified convex planar face by endpoint-free circular cuts.
///
/// Every circle produces one disk cell and one oppositely oriented hole use
/// in the common outer cell.  No configuration count is special-cased: zero,
/// one, or any number of upstream-certified disjoint cuts use the same loop.
pub(crate) fn partition_convex_planar_face<F, S, C>(
    face: F,
    source_boundary: impl IntoIterator<Item = S>,
    cuts: impl IntoIterator<Item = CertifiedPlanarCircleCut<C>>,
) -> Result<FacePartition<F, S, C, PlanarCircleRepresentative>, FacePartitionError>
where
    F: Clone + Ord,
    S: Clone + Ord,
    C: Clone + Ord,
{
    let mut source_boundaries = source_boundary.into_iter().collect::<Vec<_>>();
    if source_boundaries.is_empty() {
        return Err(FacePartitionError::EmptySourceBoundary);
    }
    if has_duplicates(source_boundaries.iter()) {
        return Err(FacePartitionError::DuplicateSourceBoundaryKey);
    }
    canonicalize_cycle(&mut source_boundaries);

    let mut cuts = cuts.into_iter().collect::<Vec<_>>();
    if cuts.iter().any(|cut| !cut.representative.valid()) {
        return Err(FacePartitionError::InvalidCutRepresentative);
    }
    cuts.sort_by(|left, right| left.key.cmp(&right.key));
    if cuts.windows(2).any(|pair| pair[0].key == pair[1].key) {
        return Err(FacePartitionError::DuplicateCutKey);
    }

    let outer_key = FaceCellKey::new(face.clone(), FaceRegionKey::PlanarOuter);
    let mut outer_boundary = vec![FaceBoundaryCycle {
        role: FaceBoundaryCycleRole::PlanarOuter,
        uses: source_boundaries
            .iter()
            .cloned()
            .map(|source| FaceBoundaryUse::source(source, FaceBoundaryOrientation::Forward))
            .collect(),
    }];
    let mut cells = Vec::with_capacity(cuts.len() + 1);
    let mut adjacency = Vec::with_capacity(cuts.len());
    let mut records = Vec::with_capacity(cuts.len());

    for cut in cuts {
        let disk_key = FaceCellKey::new(face.clone(), FaceRegionKey::PlanarDisk(cut.key.clone()));
        outer_boundary.push(FaceBoundaryCycle {
            role: FaceBoundaryCycleRole::PlanarHole,
            uses: vec![FaceBoundaryUse::cut(
                cut.key.clone(),
                FaceBoundaryOrientation::Reversed,
            )],
        });
        cells.push(FaceCell {
            key: disk_key.clone(),
            boundary: vec![FaceBoundaryCycle {
                role: FaceBoundaryCycleRole::PlanarOuter,
                uses: vec![FaceBoundaryUse::cut(
                    cut.key.clone(),
                    FaceBoundaryOrientation::Forward,
                )],
            }],
        });
        adjacency.push(FaceCutAdjacency::new(
            cut.key.clone(),
            [outer_key.clone(), disk_key],
        ));
        records.push(FacePartitionCut {
            key: cut.key,
            representative: cut.representative,
        });
    }
    cells.push(FaceCell {
        key: outer_key,
        boundary: outer_boundary,
    });
    cells.sort_by(|left, right| left.key.cmp(&right.key));
    adjacency.sort_by(|left, right| left.cut.cmp(&right.cut));

    let partition = FacePartition {
        face,
        source_boundaries,
        cuts: records,
        cells,
        adjacency,
    };
    partition.validate()?;
    Ok(partition)
}

/// Partition a certified full-period cylinder band by constant-v whole rings.
///
/// The lower and upper source rings bound the sequence.  Every admitted cut
/// adds one axial band.  Sorting uses only `exact_order`; the f64 axial value
/// is retained but deliberately ignored even if its numeric order disagrees.
pub(crate) fn partition_periodic_cylinder_face<F, S, C, O>(
    face: F,
    lower_source: S,
    upper_source: S,
    cuts: impl IntoIterator<Item = CertifiedAxialRingCut<C, O>>,
) -> Result<FacePartition<F, S, C, AxialRingRepresentative<O>>, FacePartitionError>
where
    F: Clone + Ord,
    S: Clone + Ord,
    C: Clone + Ord,
    O: Clone + Ord,
{
    if lower_source == upper_source {
        return Err(FacePartitionError::DuplicateSourceBoundaryKey);
    }
    let mut cuts = cuts.into_iter().collect::<Vec<_>>();
    if cuts.iter().any(|cut| !cut.axial_parameter.is_finite()) {
        return Err(FacePartitionError::InvalidCutRepresentative);
    }
    let mut identities = BTreeSet::new();
    if cuts.iter().any(|cut| !identities.insert(cut.key.clone())) {
        return Err(FacePartitionError::DuplicateCutKey);
    }
    cuts.sort_by(|left, right| left.exact_order.cmp(&right.exact_order));
    if cuts
        .windows(2)
        .any(|pair| pair[0].exact_order == pair[1].exact_order)
    {
        return Err(FacePartitionError::DuplicateCutOrder);
    }

    let ordered_boundaries = std::iter::once(AxialBoundary::LowerSource)
        .chain(cuts.iter().map(|cut| AxialBoundary::Cut(cut.key.clone())))
        .chain(std::iter::once(AxialBoundary::UpperSource))
        .collect::<Vec<_>>();
    let mut cells = Vec::with_capacity(ordered_boundaries.len() - 1);
    for pair in ordered_boundaries.windows(2) {
        let [lower, upper] = pair else {
            unreachable!("windows(2) always yields pairs")
        };
        cells.push(FaceCell {
            key: FaceCellKey::new(
                face.clone(),
                FaceRegionKey::AxialBand {
                    lower: lower.clone(),
                    upper: upper.clone(),
                },
            ),
            boundary: vec![
                FaceBoundaryCycle {
                    role: FaceBoundaryCycleRole::AxialLower,
                    uses: vec![axial_boundary_use(
                        lower,
                        &lower_source,
                        &upper_source,
                        FaceBoundaryOrientation::Reversed,
                    )],
                },
                FaceBoundaryCycle {
                    role: FaceBoundaryCycleRole::AxialUpper,
                    uses: vec![axial_boundary_use(
                        upper,
                        &lower_source,
                        &upper_source,
                        FaceBoundaryOrientation::Forward,
                    )],
                },
            ],
        });
    }

    let cells_by_region = cells
        .iter()
        .map(|cell| (cell.key.region.clone(), cell.key.clone()))
        .collect::<BTreeMap<_, _>>();
    let mut adjacency = Vec::with_capacity(cuts.len());
    for index in 0..cuts.len() {
        let below = FaceRegionKey::AxialBand {
            lower: ordered_boundaries[index].clone(),
            upper: ordered_boundaries[index + 1].clone(),
        };
        let above = FaceRegionKey::AxialBand {
            lower: ordered_boundaries[index + 1].clone(),
            upper: ordered_boundaries[index + 2].clone(),
        };
        adjacency.push(FaceCutAdjacency::new(
            cuts[index].key.clone(),
            [
                cells_by_region
                    .get(&below)
                    .expect("every consecutive boundary pair made one band")
                    .clone(),
                cells_by_region
                    .get(&above)
                    .expect("every consecutive boundary pair made one band")
                    .clone(),
            ],
        ));
    }

    let records = cuts
        .into_iter()
        .map(|cut| FacePartitionCut {
            key: cut.key,
            representative: AxialRingRepresentative {
                exact_order: cut.exact_order,
                axial_parameter: cut.axial_parameter,
            },
        })
        .collect();
    cells.sort_by(|left, right| left.key.cmp(&right.key));
    adjacency.sort_by(|left, right| left.cut.cmp(&right.cut));
    let partition = FacePartition {
        face,
        source_boundaries: vec![lower_source, upper_source],
        cuts: records,
        cells,
        adjacency,
    };
    partition.validate()?;
    Ok(partition)
}

fn axial_boundary_use<S: Clone, C: Clone>(
    boundary: &AxialBoundary<C>,
    lower_source: &S,
    upper_source: &S,
    orientation: FaceBoundaryOrientation,
) -> FaceBoundaryUse<S, C> {
    match boundary {
        AxialBoundary::LowerSource => FaceBoundaryUse::source(lower_source.clone(), orientation),
        AxialBoundary::Cut(cut) => FaceBoundaryUse::cut(cut.clone(), orientation),
        AxialBoundary::UpperSource => FaceBoundaryUse::source(upper_source.clone(), orientation),
    }
}

impl<F, S, C, R> FacePartition<F, S, C, R>
where
    F: Clone + Ord,
    S: Clone + Ord,
    C: Clone + Ord,
{
    /// Recheck all representation-level conservation and adjacency proofs.
    ///
    /// Each source use must occur once.  Each cut must occur exactly twice,
    /// once in each direction, and its two owners must exactly match the one
    /// dual adjacency record.  The check never consults representatives.
    pub(crate) fn validate(&self) -> Result<(), FacePartitionError> {
        if self.source_boundaries.is_empty() {
            return Err(FacePartitionError::EmptySourceBoundary);
        }
        if has_duplicates(self.source_boundaries.iter()) {
            return Err(FacePartitionError::DuplicateSourceBoundaryKey);
        }
        if has_duplicates(self.cuts.iter().map(|cut| &cut.key)) {
            return Err(FacePartitionError::DuplicateCutKey);
        }
        if self.cells.windows(2).any(|pair| pair[0].key >= pair[1].key) {
            return if self.cells.windows(2).any(|pair| pair[0].key == pair[1].key) {
                Err(FacePartitionError::DuplicateCellKey)
            } else {
                Err(FacePartitionError::NonCanonicalOrder)
            };
        }
        if self
            .adjacency
            .windows(2)
            .any(|pair| pair[0].cut >= pair[1].cut)
        {
            return if self
                .adjacency
                .windows(2)
                .any(|pair| pair[0].cut == pair[1].cut)
            {
                Err(FacePartitionError::DuplicateCutAdjacency)
            } else {
                Err(FacePartitionError::NonCanonicalOrder)
            };
        }

        let sources = self
            .source_boundaries
            .iter()
            .cloned()
            .collect::<BTreeSet<_>>();
        let cuts = self
            .cuts
            .iter()
            .map(|cut| cut.key.clone())
            .collect::<BTreeSet<_>>();
        let cell_keys = self
            .cells
            .iter()
            .map(|cell| cell.key.clone())
            .collect::<BTreeSet<_>>();
        let mut source_counts = BTreeMap::<S, usize>::new();
        let mut cut_counts = BTreeMap::<C, [usize; 2]>::new();
        let mut cut_owners = BTreeMap::<C, BTreeSet<FaceCellKey<F, C>>>::new();
        for cell in &self.cells {
            if cell.key.face != self.face {
                return Err(FacePartitionError::UnknownAdjacencyCell);
            }
            if cell.boundary.is_empty() || cell.boundary.iter().any(|cycle| cycle.uses.is_empty()) {
                return Err(FacePartitionError::EmptyCellBoundary);
            }
            for use_ in cell.boundary.iter().flat_map(|cycle| &cycle.uses) {
                match &use_.key {
                    FaceBoundaryKey::Source(source) => {
                        if !sources.contains(source) {
                            return Err(FacePartitionError::UnknownSourceBoundaryUse);
                        }
                        *source_counts.entry(source.clone()).or_default() += 1;
                    }
                    FaceBoundaryKey::Cut(cut) => {
                        if !cuts.contains(cut) {
                            return Err(FacePartitionError::UnknownCutBoundaryUse);
                        }
                        let counts = cut_counts.entry(cut.clone()).or_default();
                        match use_.orientation {
                            FaceBoundaryOrientation::Forward => counts[0] += 1,
                            FaceBoundaryOrientation::Reversed => counts[1] += 1,
                        }
                        cut_owners
                            .entry(cut.clone())
                            .or_default()
                            .insert(cell.key.clone());
                    }
                }
            }
        }
        for source in sources {
            let count = source_counts.get(&source).copied().unwrap_or(0);
            if count != 1 {
                return Err(FacePartitionError::SourceBoundaryUseCount { count });
            }
        }
        for cut in &cuts {
            let [forward, reversed] = cut_counts.get(cut).copied().unwrap_or([0, 0]);
            let count = forward + reversed;
            if count != 2 || forward != 1 || reversed != 1 {
                return Err(FacePartitionError::CutBoundaryUseCount {
                    count,
                    forward,
                    reversed,
                });
            }
        }

        let mut adjacency = BTreeMap::new();
        for entry in &self.adjacency {
            if entry.cells[0] == entry.cells[1] {
                return Err(FacePartitionError::SelfAdjacency);
            }
            if entry.cells[0] > entry.cells[1] {
                return Err(FacePartitionError::NonCanonicalOrder);
            }
            if !cuts.contains(&entry.cut) {
                return Err(FacePartitionError::UnknownCutBoundaryUse);
            }
            if !cell_keys.contains(&entry.cells[0]) || !cell_keys.contains(&entry.cells[1]) {
                return Err(FacePartitionError::UnknownAdjacencyCell);
            }
            if adjacency
                .insert(entry.cut.clone(), entry.cells.clone())
                .is_some()
            {
                return Err(FacePartitionError::DuplicateCutAdjacency);
            }
        }
        for cut in cuts {
            let Some(adjacent) = adjacency.get(&cut) else {
                return Err(FacePartitionError::MissingCutAdjacency);
            };
            let owners = cut_owners.get(&cut).cloned().unwrap_or_default();
            let adjacent = adjacent.iter().cloned().collect::<BTreeSet<_>>();
            if owners.len() != 2 || owners != adjacent {
                return Err(FacePartitionError::AdjacencyDoesNotMatchBoundaryUses);
            }
        }
        Ok(())
    }
}

fn has_duplicates<'a, T: 'a + Ord>(values: impl IntoIterator<Item = &'a T>) -> bool {
    let mut seen = BTreeSet::new();
    values.into_iter().any(|value| !seen.insert(value))
}

fn canonicalize_cycle<T: Ord>(cycle: &mut [T]) {
    let Some(first) = cycle
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| left.cmp(right))
        .map(|(index, _)| index)
    else {
        return;
    };
    cycle.rotate_left(first);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle(key: char, center: [f64; 2], radius: f64) -> CertifiedPlanarCircleCut<char> {
        CertifiedPlanarCircleCut::new(key, PlanarCircleRepresentative::new(center, radius))
    }

    fn axial(key: char, exact_order: i32, representative: f64) -> CertifiedAxialRingCut<char, i32> {
        CertifiedAxialRingCut::new(key, exact_order, representative)
    }

    fn use_counts<F, S, C, R>(
        partition: &FacePartition<F, S, C, R>,
    ) -> (BTreeMap<S, usize>, BTreeMap<C, [usize; 2]>)
    where
        S: Clone + Ord,
        C: Clone + Ord,
    {
        let mut sources = BTreeMap::new();
        let mut cuts = BTreeMap::<C, [usize; 2]>::new();
        for use_ in partition
            .cells()
            .iter()
            .flat_map(|cell| cell.boundary().iter())
            .flat_map(FaceBoundaryCycle::uses)
        {
            match use_.key() {
                FaceBoundaryKey::Source(source) => {
                    *sources.entry(source.clone()).or_default() += 1;
                }
                FaceBoundaryKey::Cut(cut) => {
                    let count = cuts.entry(cut.clone()).or_default();
                    match use_.orientation() {
                        FaceBoundaryOrientation::Forward => count[0] += 1,
                        FaceBoundaryOrientation::Reversed => count[1] += 1,
                    }
                }
            }
        }
        (sources, cuts)
    }

    #[test]
    fn planar_three_cut_partition_is_canonical_and_conservative() {
        let first = partition_convex_planar_face(
            7_u8,
            [10_u8, 20, 30, 40],
            [
                circle('z', [3.0, 0.0], 0.5),
                circle('a', [-3.0, 0.0], 0.75),
                circle('m', [0.0, 2.0], 0.25),
            ],
        )
        .unwrap();
        let permuted = partition_convex_planar_face(
            7_u8,
            [30_u8, 40, 10, 20],
            [
                circle('m', [0.0, 2.0], 0.25),
                circle('z', [3.0, 0.0], 0.5),
                circle('a', [-3.0, 0.0], 0.75),
            ],
        )
        .unwrap();

        assert_eq!(first, permuted);
        assert_eq!(first.face(), &7);
        assert_eq!(first.cells().len(), 4);
        assert_eq!(first.adjacency().len(), 3);
        assert_eq!(first.source_boundaries(), &[10, 20, 30, 40]);
        assert_eq!(
            first
                .cuts()
                .iter()
                .map(|cut| *cut.key())
                .collect::<Vec<_>>(),
            vec!['a', 'm', 'z']
        );
        assert_eq!(first.cuts()[0].representative().center(), [-3.0, 0.0]);
        assert_eq!(first.cuts()[0].representative().radius(), 0.75);

        let outer = first
            .cells()
            .iter()
            .find(|cell| cell.key().region() == &FaceRegionKey::PlanarOuter)
            .unwrap();
        assert_eq!(outer.key().face(), &7);
        assert_eq!(outer.boundary().len(), 4);
        assert_eq!(
            outer.boundary()[0].role(),
            FaceBoundaryCycleRole::PlanarOuter
        );
        assert!(
            outer.boundary()[1..]
                .iter()
                .all(|cycle| cycle.role() == FaceBoundaryCycleRole::PlanarHole)
        );

        let (sources, cuts) = use_counts(&first);
        assert_eq!(sources.values().copied().collect::<Vec<_>>(), vec![1; 4]);
        assert_eq!(cuts.values().copied().collect::<Vec<_>>(), vec![[1, 1]; 3]);
        for entry in first.adjacency() {
            assert!(
                entry
                    .cells()
                    .iter()
                    .any(|cell| { cell.region() == &FaceRegionKey::PlanarOuter })
            );
            assert!(
                entry
                    .cells()
                    .iter()
                    .any(|cell| { cell.region() == &FaceRegionKey::PlanarDisk(*entry.cut()) })
            );
        }
        first.validate().unwrap();
    }

    #[test]
    fn four_axial_cuts_follow_exact_order_not_numeric_representatives() {
        let first = partition_periodic_cylinder_face(
            9_u8,
            100_u16,
            200_u16,
            [
                axial('z', 20, -500.0),
                axial('b', 100, -1_000.0),
                axial('a', -10, 900.0),
                axial('m', 7, 0.0),
            ],
        )
        .unwrap();
        let permuted = partition_periodic_cylinder_face(
            9_u8,
            100_u16,
            200_u16,
            [
                axial('m', 7, 0.0),
                axial('a', -10, 900.0),
                axial('b', 100, -1_000.0),
                axial('z', 20, -500.0),
            ],
        )
        .unwrap();

        assert_eq!(first, permuted);
        assert_eq!(first.cells().len(), 5);
        assert_eq!(first.adjacency().len(), 4);
        assert_eq!(
            first
                .cuts()
                .iter()
                .map(|cut| *cut.key())
                .collect::<Vec<_>>(),
            vec!['a', 'm', 'z', 'b']
        );
        assert_eq!(
            first
                .cuts()
                .iter()
                .map(|cut| *cut.representative().exact_order())
                .collect::<Vec<_>>(),
            vec![-10, 7, 20, 100]
        );
        assert_eq!(
            first
                .cuts()
                .iter()
                .map(|cut| cut.representative().axial_parameter())
                .collect::<Vec<_>>(),
            vec![900.0, 0.0, -500.0, -1_000.0]
        );

        // Five axial bands have two endpoint-free ring cycles each.  The
        // representation has no vertex or seam occurrence to count.
        assert!(first.cells().iter().all(|cell| {
            cell.boundary().len() == 2
                && cell.boundary().iter().all(|cycle| cycle.uses().len() == 1)
        }));
        let (sources, cuts) = use_counts(&first);
        assert_eq!(sources, BTreeMap::from([(100, 1), (200, 1)]));
        assert_eq!(cuts.values().copied().collect::<Vec<_>>(), vec![[1, 1]; 4]);

        let expected = [
            (AxialBoundary::LowerSource, AxialBoundary::Cut('a')),
            (AxialBoundary::Cut('a'), AxialBoundary::Cut('m')),
            (AxialBoundary::Cut('m'), AxialBoundary::Cut('z')),
            (AxialBoundary::Cut('z'), AxialBoundary::Cut('b')),
            (AxialBoundary::Cut('b'), AxialBoundary::UpperSource),
        ];
        for (lower, upper) in expected {
            assert!(first.cells().iter().any(|cell| {
                cell.key().region()
                    == &FaceRegionKey::AxialBand {
                        lower: lower.clone(),
                        upper: upper.clone(),
                    }
            }));
        }
        first.validate().unwrap();
    }

    #[test]
    fn exact_dual_graph_propagates_anchor_parity_without_representatives() {
        let planar = partition_convex_planar_face(
            7_u8,
            [10_u8, 20, 30, 40],
            [
                circle('z', [3.0, 0.0], 0.5),
                circle('a', [-3.0, 0.0], 0.75),
                circle('m', [0.0, 2.0], 0.25),
            ],
        )
        .unwrap();
        let planar_anchor = planar
            .cells()
            .iter()
            .find(|cell| cell.key().region() == &FaceRegionKey::PlanarOuter)
            .unwrap()
            .key()
            .clone();
        let planar_classes = classify_face_partition_from_anchor(
            &planar,
            &planar_anchor,
            FaceCellOpenClassification::Exterior,
        )
        .unwrap();
        assert_eq!(
            planar_classes[&planar_anchor],
            FaceCellOpenClassification::Exterior
        );
        assert!(planar_classes.iter().all(|(cell, classification)| {
            matches!(cell.region(), FaceRegionKey::PlanarOuter)
                || *classification == FaceCellOpenClassification::Interior
        }));

        let axial = partition_periodic_cylinder_face(
            9_u8,
            100_u16,
            200_u16,
            [
                axial('z', 20, -500.0),
                axial('b', 100, -1_000.0),
                axial('a', -10, 900.0),
                axial('m', 7, 0.0),
            ],
        )
        .unwrap();
        let axial_anchor = axial
            .cells()
            .iter()
            .find(|cell| {
                cell.key().region()
                    == &FaceRegionKey::AxialBand {
                        lower: AxialBoundary::LowerSource,
                        upper: AxialBoundary::Cut('a'),
                    }
            })
            .unwrap()
            .key()
            .clone();
        let axial_classes = classify_face_partition_from_anchor(
            &axial,
            &axial_anchor,
            FaceCellOpenClassification::Exterior,
        )
        .unwrap();
        let ordered = [
            FaceCellOpenClassification::Exterior,
            FaceCellOpenClassification::Interior,
            FaceCellOpenClassification::Exterior,
            FaceCellOpenClassification::Interior,
            FaceCellOpenClassification::Exterior,
        ];
        let regions = [
            (AxialBoundary::LowerSource, AxialBoundary::Cut('a')),
            (AxialBoundary::Cut('a'), AxialBoundary::Cut('m')),
            (AxialBoundary::Cut('m'), AxialBoundary::Cut('z')),
            (AxialBoundary::Cut('z'), AxialBoundary::Cut('b')),
            (AxialBoundary::Cut('b'), AxialBoundary::UpperSource),
        ];
        for ((lower, upper), expected) in regions.into_iter().zip(ordered) {
            let key = axial_classes
                .keys()
                .find(|cell| {
                    cell.region()
                        == &FaceRegionKey::AxialBand {
                            lower: lower.clone(),
                            upper: upper.clone(),
                        }
                })
                .unwrap();
            assert_eq!(axial_classes[key], expected);
        }
    }

    #[test]
    fn malformed_inputs_fail_closed_before_topology_is_returned() {
        assert_eq!(
            partition_convex_planar_face(
                0_u8,
                std::iter::empty::<u8>(),
                [circle('a', [0.0, 0.0], 1.0)],
            ),
            Err(FacePartitionError::EmptySourceBoundary)
        );
        assert_eq!(
            partition_convex_planar_face(
                0_u8,
                [1_u8, 1],
                std::iter::empty::<CertifiedPlanarCircleCut<char>>(),
            ),
            Err(FacePartitionError::DuplicateSourceBoundaryKey)
        );
        assert_eq!(
            partition_convex_planar_face(
                0_u8,
                [1_u8, 2, 3],
                [circle('a', [0.0, 0.0], 1.0), circle('a', [1.0, 0.0], 1.0)],
            ),
            Err(FacePartitionError::DuplicateCutKey)
        );
        assert_eq!(
            partition_convex_planar_face(0_u8, [1_u8, 2, 3], [circle('a', [f64::NAN, 0.0], 1.0)],),
            Err(FacePartitionError::InvalidCutRepresentative)
        );
        assert_eq!(
            partition_periodic_cylinder_face(
                0_u8,
                1_u8,
                2_u8,
                [axial('a', 4, 0.25), axial('b', 4, 0.75)],
            ),
            Err(FacePartitionError::DuplicateCutOrder)
        );
        assert_eq!(
            partition_periodic_cylinder_face(
                0_u8,
                1_u8,
                1_u8,
                std::iter::empty::<CertifiedAxialRingCut<char, i32>>(),
            ),
            Err(FacePartitionError::DuplicateSourceBoundaryKey)
        );
    }

    #[test]
    fn validation_detects_lost_source_use_and_wrong_cut_orientation() {
        let mut missing_source =
            partition_convex_planar_face(1_u8, [10_u8, 20, 30], [circle('a', [0.0, 0.0], 1.0)])
                .unwrap();
        let outer = missing_source
            .cells
            .iter_mut()
            .find(|cell| cell.key.region == FaceRegionKey::PlanarOuter)
            .unwrap();
        outer.boundary[0].uses.pop();
        assert_eq!(
            missing_source.validate(),
            Err(FacePartitionError::SourceBoundaryUseCount { count: 0 })
        );

        let mut wrong_orientation =
            partition_periodic_cylinder_face(1_u8, 10_u8, 20_u8, [axial('a', 0, 0.5)]).unwrap();
        for use_ in wrong_orientation
            .cells
            .iter_mut()
            .flat_map(|cell| &mut cell.boundary)
            .flat_map(|cycle| &mut cycle.uses)
        {
            if use_.key == FaceBoundaryKey::Cut('a') {
                use_.orientation = FaceBoundaryOrientation::Forward;
            }
        }
        assert_eq!(
            wrong_orientation.validate(),
            Err(FacePartitionError::CutBoundaryUseCount {
                count: 2,
                forward: 2,
                reversed: 0,
            })
        );
    }
}
