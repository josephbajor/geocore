//! Exact-topology assembly for finite non-wrapping skew-cylinder sheet spans.
//!
//! This module consumes four already-complete 64-work axial-bound queries. It
//! performs no new root solve and depends on no whole-sheet carrier
//! certificate. The merge is purely topological: projective root corridors
//! establish a strict cyclic order, exact open-cell relations establish finite
//! occupancy, and only two locally revalidated simple transverse roots may
//! bound a returned open span. Numeric range endpoints are the representable
//! inside sides of those exact-source corridors; the roots themselves remain
//! separate endpoint proofs.

use core::cmp::Ordering;

use super::{
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderAxialBoundProvenance,
    SkewCylinderAxialBoundTopology, SkewCylinderAxialBoundary, SkewCylinderAxialRelation,
    SkewCylinderAxialRoot, SkewCylinderHalfAngleChart, SkewCylinderHalfAngleRootBracket,
    SkewCylinderSheet,
};
use kcore::interval::Interval;
use kgeom::param::ParamRange;

const TAU: f64 = core::f64::consts::TAU;
// Deterministic interval-rounding headroom for both the stored and exact-source
// residual enclosures. The merged-corridor checks below still refuse any
// authored chart where these steps could cross another root.
const ENDPOINT_GUARD_STEPS: usize = 2 * SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS;

/// Four exact bound topologies and the authored windows they must describe.
///
/// `ranges` is in canonical solver order; provenance remains in caller order
/// through `canonical_to_source`.
#[derive(Debug, Clone, Copy)]
pub struct SkewCylinderOpenSpanTopologyInput<'a> {
    /// Four sealed exact bound outcomes.
    pub topologies: &'a [SkewCylinderAxialBoundTopology; 4],
    /// Exact authored windows in formula order.
    pub ranges: [[ParamRange; 2]; 2],
    /// Formula-slot to caller/source-slot permutation.
    pub canonical_to_source: [usize; 2],
}

/// The strict side of a source-root corridor retained by one finite span.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderRootInsideSide {
    /// Increasing-longitude side immediately before the exact root.
    Before,
    /// Increasing-longitude side immediately after the exact root.
    After,
}

/// Exact-source endpoint identity plus the representable parameter on its
/// proven finite-window side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderOpenSpanEndpointProof {
    /// Exact source-root event.
    pub root: SkewCylinderAxialRoot,
    /// Retained side of that root.
    pub inside_side: SkewCylinderRootInsideSide,
    /// Representable inside-side carrier parameter.
    pub carrier_parameter: f64,
}

/// One proper finite component in increasing authored carrier order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderOpenSpan {
    /// Ordered quadratic sheet.
    pub sheet: SkewCylinderSheet,
    /// Strict guarded open range.
    pub range: ParamRange,
    /// Lower canonical endpoint proof.
    pub start: SkewCylinderOpenSpanEndpointProof,
    /// Upper canonical endpoint proof.
    pub end: SkewCylinderOpenSpanEndpointProof,
}

impl SkewCylinderOpenSpan {
    /// Lift both exact-source projective root brackets into the canonical
    /// longitude chart that owns this non-wrapping span.
    ///
    /// The returned order is `[lower, upper]` in increasing carrier
    /// parameter, independent of caller operand order. The exact projective
    /// provenance remains attached to `start` and `end`; these intervals are
    /// only its monotone longitude image for pcurve corridor certification.
    pub fn root_longitude_intervals(self, authored_longitude: ParamRange) -> Option<[Interval; 2]> {
        if self.start.inside_side != SkewCylinderRootInsideSide::After
            || self.end.inside_side != SkewCylinderRootInsideSide::Before
            || self.start.carrier_parameter.to_bits() != self.range.lo.to_bits()
            || self.end.carrier_parameter.to_bits() != self.range.hi.to_bits()
        {
            return None;
        }
        let lower = lift_root_longitude_interval(self.start, authored_longitude)?;
        let upper = lift_root_longitude_interval(self.end, authored_longitude)?;
        (lower.hi() < self.range.lo && upper.lo() > self.range.hi).then_some([lower, upper])
    }
}

/// Complete finite occupancy for one ordered strict-positive sheet.
#[derive(Debug, Clone, PartialEq)]
pub enum SkewCylinderFiniteSheetTopology {
    /// No point of this sheet lies inside all four axial bounds.
    Outside,
    /// The complete full-period sheet lies inside all four axial bounds.
    Whole,
    /// Every retained non-wrapping finite component in increasing longitude.
    Open(Vec<SkewCylinderOpenSpan>),
}

/// Conservative refusal causes for the topology-only merge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderOpenSpanFailure {
    /// Formula slots do not map bijectively to the two caller/source slots.
    InvalidSourcePermutation,
    /// At least one authored range is nonfinite or reverses its endpoints.
    InvalidRange,
    /// A bound identity or full-period longitude differs from its header.
    RangeMismatch,
    /// The four lower/upper source bounds are not represented exactly once.
    DuplicateOrMissingBound,
    /// Sealed roots and open-cell relations do not form one valid cyclic sweep.
    InconsistentTopology,
    /// A repeated or non-transverse axial root cannot bound an open component.
    ContactRoot,
    /// A projective root bracket straddles a chart sector boundary.
    AmbiguousRootSector,
    /// Two exact-source root corridors cannot be strictly ordered.
    CoincidentOrOverlappingRoots,
    /// A root-to-guard continuation leaves the authored longitude chart.
    RootCorridorCrossesSeam,
    /// An occupied component wraps through the authored longitude seam.
    SeamWrappingSpan,
}

/// Sealed complete two-sheet finite-window topology.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderFiniteWindowTopologyCertificate {
    formula_cylinders: [kgeom::surface::Cylinder; 2],
    formula_ranges: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    bound_topologies: [SkewCylinderAxialBoundTopology; 4],
    sheets: [SkewCylinderFiniteSheetTopology; 2],
}

impl SkewCylinderFiniteWindowTopologyCertificate {
    /// Exact cylinders in formula/ruling order.
    pub const fn formula_cylinders(&self) -> [kgeom::surface::Cylinder; 2] {
        self.formula_cylinders
    }

    /// Exact authored windows in formula/ruling order.
    pub const fn formula_ranges(&self) -> [[ParamRange; 2]; 2] {
        self.formula_ranges
    }

    /// Formula-slot to caller/source-slot permutation.
    pub const fn formula_to_source(&self) -> [usize; 2] {
        self.formula_to_source
    }

    /// The four sealed bound outcomes used by the sweep.
    pub const fn bound_topologies(&self) -> &[SkewCylinderAxialBoundTopology; 4] {
        &self.bound_topologies
    }

    /// Complete occupancy for Lower or Upper.
    pub const fn sheet(&self, sheet: SkewCylinderSheet) -> &SkewCylinderFiniteSheetTopology {
        &self.sheets[sheet_index(sheet)]
    }
}

#[derive(Debug, Clone, Copy)]
struct RootCut {
    topology_index: usize,
    cyclic_ordinal: usize,
    bracket: SkewCylinderHalfAngleRootBracket,
    events: [Option<SkewCylinderAxialRoot>; 2],
}

#[derive(Debug, Clone, Copy)]
struct AuthoredRootCut {
    source: RootCut,
    root_parameter: f64,
    before_parameter: f64,
    after_parameter: f64,
}

#[derive(Debug, Clone, Copy)]
struct CutTransition {
    cut: AuthoredRootCut,
    before_inside: [bool; 2],
    after_inside: [bool; 2],
}

fn lift_root_longitude_interval(
    proof: SkewCylinderOpenSpanEndpointProof,
    authored_longitude: ParamRange,
) -> Option<Interval> {
    if !authored_longitude.is_finite() || authored_longitude.width() != TAU {
        return None;
    }
    let angular = proof.root.angular_bracket();
    let representative = angular.representative();
    let lifted_representative =
        fit_periodic_parameter(representative, authored_longitude, TAU, 0.0)?;
    let shift = lifted_representative - representative;
    let mut lo = angular.lo + shift;
    let mut hi = angular.hi + shift;
    match proof.inside_side {
        SkewCylinderRootInsideSide::After if hi >= proof.carrier_parameter => {
            lo -= TAU;
            hi -= TAU;
        }
        SkewCylinderRootInsideSide::Before if lo <= proof.carrier_parameter => {
            lo += TAU;
            hi += TAU;
        }
        SkewCylinderRootInsideSide::After | SkewCylinderRootInsideSide::Before => {}
    }
    if !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
        || lo <= authored_longitude.lo
        || hi >= authored_longitude.hi
    {
        return None;
    }
    match proof.inside_side {
        SkewCylinderRootInsideSide::After if hi < proof.carrier_parameter => {
            Some(Interval::new(lo, hi))
        }
        SkewCylinderRootInsideSide::Before if lo > proof.carrier_parameter => {
            Some(Interval::new(lo, hi))
        }
        SkewCylinderRootInsideSide::After | SkewCylinderRootInsideSide::Before => None,
    }
}

fn fit_periodic_parameter(
    candidate: f64,
    range: ParamRange,
    period: f64,
    tolerance: f64,
) -> Option<f64> {
    let k_min = ((range.lo - tolerance - candidate) / period).ceil() as i64;
    let k_max = ((range.hi + tolerance - candidate) / period).floor() as i64;
    if k_min > k_max {
        return None;
    }
    Some((candidate + k_min as f64 * period).clamp(range.lo, range.hi))
}

/// Merge four exact source-bound topologies into complete Lower/Upper finite
/// occupancy. The result order is always `[Lower, Upper]`.
pub fn classify_skew_cylinder_open_spans(
    input: SkewCylinderOpenSpanTopologyInput<'_>,
) -> Result<SkewCylinderFiniteWindowTopologyCertificate, SkewCylinderOpenSpanFailure> {
    validate_ranges(input.ranges, input.canonical_to_source)?;
    let formula_cylinders = input.topologies[0].formula_cylinders();
    if input.topologies.iter().any(|topology| {
        topology.formula_cylinders() != formula_cylinders
            || topology.formula_to_source() != input.canonical_to_source
    }) {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    let topologies = normalize_topologies(input)?;
    let mut cuts = Vec::new();
    for (topology_index, topology) in topologies.iter().enumerate() {
        cuts.extend(validate_topology(topology_index, topology)?);
    }
    sort_and_validate_projective_roots(&mut cuts)?;

    let carrier_range = input.ranges[0][0];
    let mut authored = cuts
        .into_iter()
        .map(|cut| contextualize_root_cut(cut, carrier_range))
        .collect::<Result<Vec<_>, _>>()?;
    authored.sort_by(|lhs, rhs| lhs.root_parameter.total_cmp(&rhs.root_parameter));
    validate_authored_root_order(&authored, carrier_range)?;

    let (initial_inside, transitions) = sweep_topologies(&topologies, &authored)?;
    let sheets = [
        classify_sheet(
            SkewCylinderSheet::Lower,
            initial_inside[0],
            &transitions,
            carrier_range,
        )?,
        classify_sheet(
            SkewCylinderSheet::Upper,
            initial_inside[1],
            &transitions,
            carrier_range,
        )?,
    ];
    Ok(SkewCylinderFiniteWindowTopologyCertificate {
        formula_cylinders,
        formula_ranges: input.ranges,
        formula_to_source: input.canonical_to_source,
        bound_topologies: core::array::from_fn(|index| topologies[index].clone()),
        sheets,
    })
}

fn validate_ranges(
    ranges: [[ParamRange; 2]; 2],
    canonical_to_source: [usize; 2],
) -> Result<(), SkewCylinderOpenSpanFailure> {
    if !matches!(canonical_to_source, [0, 1] | [1, 0]) {
        return Err(SkewCylinderOpenSpanFailure::InvalidSourcePermutation);
    }
    if ranges
        .iter()
        .flatten()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(SkewCylinderOpenSpanFailure::InvalidRange);
    }
    if ranges
        .iter()
        .any(|range| range[0].width() != TAU || range[1].width() < 0.0)
    {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    Ok(())
}

fn normalize_topologies<'a>(
    input: SkewCylinderOpenSpanTopologyInput<'a>,
) -> Result<[&'a SkewCylinderAxialBoundTopology; 4], SkewCylinderOpenSpanFailure> {
    let mut normalized: [Option<&SkewCylinderAxialBoundTopology>; 4] = [None; 4];
    for topology in input.topologies {
        let source_operand = topology.provenance().source_operand;
        let canonical_operand = input
            .canonical_to_source
            .iter()
            .position(|source| *source == source_operand)
            .ok_or(SkewCylinderOpenSpanFailure::RangeMismatch)?;
        let boundary_index = match topology.provenance().boundary {
            SkewCylinderAxialBoundary::Lower => 0,
            SkewCylinderAxialBoundary::Upper => 1,
        };
        let expected_value = if boundary_index == 0 {
            input.ranges[canonical_operand][1].lo
        } else {
            input.ranges[canonical_operand][1].hi
        };
        if topology.provenance().value.to_bits() != expected_value.to_bits() {
            return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
        }
        let slot = 2 * canonical_operand + boundary_index;
        if normalized[slot].replace(topology).is_some() {
            return Err(SkewCylinderOpenSpanFailure::DuplicateOrMissingBound);
        }
    }
    let [
        Some(first_lower),
        Some(first_upper),
        Some(second_lower),
        Some(second_upper),
    ] = normalized
    else {
        return Err(SkewCylinderOpenSpanFailure::DuplicateOrMissingBound);
    };
    Ok([first_lower, first_upper, second_lower, second_upper])
}

fn validate_topology(
    topology_index: usize,
    topology: &SkewCylinderAxialBoundTopology,
) -> Result<Vec<RootCut>, SkewCylinderOpenSpanFailure> {
    if topology.open_cell_relations().is_empty() {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    if topology.roots().is_empty() {
        return if topology.open_cell_relations().len() == 1 {
            Ok(Vec::new())
        } else {
            Err(SkewCylinderOpenSpanFailure::InconsistentTopology)
        };
    }

    let root_count = topology.open_cell_relations().len();
    let mut cuts = (0..root_count)
        .map(|cyclic_ordinal| RootCut {
            topology_index,
            cyclic_ordinal,
            bracket: SkewCylinderHalfAngleRootBracket {
                chart: SkewCylinderHalfAngleChart::Tangent,
                lo: 0.0,
                hi: 0.0,
            },
            events: [None; 2],
        })
        .collect::<Vec<_>>();
    let mut initialized = vec![false; root_count];

    for root in topology.roots() {
        validate_root(topology, *root)?;
        let cut = cuts
            .get_mut(root.cyclic_ordinal)
            .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
        if initialized[root.cyclic_ordinal] {
            if cut.bracket != root.bracket {
                return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
            }
        } else {
            validate_root_bracket(root.bracket)?;
            cut.bracket = root.bracket;
            initialized[root.cyclic_ordinal] = true;
        }
        let sheet = sheet_index(root.sheet);
        if cut.events[sheet].replace(*root).is_some() {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
    }
    if initialized.contains(&false) {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }

    for cut in &cuts {
        let before =
            topology.open_cell_relations()[(cut.cyclic_ordinal + root_count - 1) % root_count];
        let after = topology.open_cell_relations()[cut.cyclic_ordinal];
        for sheet in 0..2 {
            if (before[sheet] != after[sheet]) != cut.events[sheet].is_some() {
                return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
            }
        }
    }
    for pair in cuts.windows(2) {
        if compare_projective_corridors(pair[0].bracket, pair[1].bracket)? != Ordering::Less {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
    }
    Ok(cuts)
}

fn validate_root(
    topology: &SkewCylinderAxialBoundTopology,
    root: SkewCylinderAxialRoot,
) -> Result<(), SkewCylinderOpenSpanFailure> {
    if root.provenance != topology.provenance() {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    if root.repeated || root.before == root.after {
        return Err(SkewCylinderOpenSpanFailure::ContactRoot);
    }
    let root_count = topology.open_cell_relations().len();
    if root.cyclic_ordinal >= root_count {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let sheet = sheet_index(root.sheet);
    let before =
        topology.open_cell_relations()[(root.cyclic_ordinal + root_count - 1) % root_count][sheet];
    let after = topology.open_cell_relations()[root.cyclic_ordinal][sheet];
    if (root.before, root.after) != (before, after) {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok(())
}

fn validate_root_bracket(
    bracket: SkewCylinderHalfAngleRootBracket,
) -> Result<(), SkewCylinderOpenSpanFailure> {
    if !bracket.lo.is_finite() || !bracket.hi.is_finite() || bracket.lo > bracket.hi {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let owned = match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent => bracket.lo >= -1.0 && bracket.hi <= 1.0,
        SkewCylinderHalfAngleChart::Cotangent => bracket.lo > -1.0 && bracket.hi < 1.0,
    };
    if !owned {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    if bracket.lo < 0.0 && bracket.hi > 0.0 {
        return Err(SkewCylinderOpenSpanFailure::AmbiguousRootSector);
    }
    Ok(())
}

fn sort_and_validate_projective_roots(
    cuts: &mut [RootCut],
) -> Result<(), SkewCylinderOpenSpanFailure> {
    // At most sixteen cuts exist. Insertion sort keeps fallible exact-corridor
    // comparison explicit rather than hiding a refusal in an infallible sort.
    for index in 1..cuts.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare_projective_corridors(cuts[cursor - 1].bracket, cuts[cursor].bracket)? {
                Ordering::Less => break,
                Ordering::Equal => {
                    return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
                }
                Ordering::Greater => {
                    cuts.swap(cursor - 1, cursor);
                    cursor -= 1;
                }
            }
        }
    }
    Ok(())
}

fn compare_projective_corridors(
    lhs: SkewCylinderHalfAngleRootBracket,
    rhs: SkewCylinderHalfAngleRootBracket,
) -> Result<Ordering, SkewCylinderOpenSpanFailure> {
    let lhs_sector = root_sector(lhs)?;
    let rhs_sector = root_sector(rhs)?;
    if lhs_sector != rhs_sector {
        return Ok(lhs_sector.cmp(&rhs_sector));
    }
    let disjoint_order = if matches!(lhs_sector, 0 | 3) {
        if lhs.hi < rhs.lo {
            Some(Ordering::Less)
        } else if rhs.hi < lhs.lo {
            Some(Ordering::Greater)
        } else {
            None
        }
    } else if lhs.lo > rhs.hi {
        Some(Ordering::Less)
    } else if rhs.lo > lhs.hi {
        Some(Ordering::Greater)
    } else {
        None
    };
    disjoint_order.ok_or(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots)
}

fn root_sector(
    bracket: SkewCylinderHalfAngleRootBracket,
) -> Result<u8, SkewCylinderOpenSpanFailure> {
    validate_root_bracket(bracket)?;
    Ok(match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent if bracket.lo >= 0.0 => 0,
        SkewCylinderHalfAngleChart::Cotangent if bracket.lo >= 0.0 => 1,
        SkewCylinderHalfAngleChart::Cotangent => 2,
        SkewCylinderHalfAngleChart::Tangent => 3,
    })
}

fn contextualize_root_cut(
    source: RootCut,
    range: ParamRange,
) -> Result<AuthoredRootCut, SkewCylinderOpenSpanFailure> {
    let angular = source
        .events
        .iter()
        .flatten()
        .next()
        .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?
        .angular_bracket();
    let root = angular.representative();
    let root_parameter = fit_periodic_parameter(root, range, TAU, 0.0)
        .ok_or(SkewCylinderOpenSpanFailure::RangeMismatch)?;
    let before_distance = backward_cyclic_distance(root, angular.strict_before_side());
    let after_distance = forward_cyclic_distance(root, angular.strict_after_side());
    if !before_distance.is_finite()
        || !after_distance.is_finite()
        || before_distance < 0.0
        || after_distance < 0.0
        || before_distance + after_distance <= 0.0
    {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let mut before_parameter = root_parameter - before_distance;
    let mut after_parameter = root_parameter + after_distance;
    let collapsed = angular.lo == angular.hi;
    if before_parameter < range.lo || after_parameter > range.hi {
        if !collapsed || root_parameter != range.lo {
            return Err(SkewCylinderOpenSpanFailure::RootCorridorCrossesSeam);
        }
        before_parameter = range.hi - before_distance;
        after_parameter = range.lo + after_distance;
    }
    for _ in 0..ENDPOINT_GUARD_STEPS {
        before_parameter = before_parameter.next_down();
        after_parameter = after_parameter.next_up();
    }
    if !(range.lo < after_parameter
        && after_parameter < range.hi
        && range.lo < before_parameter
        && before_parameter < range.hi)
    {
        return Err(SkewCylinderOpenSpanFailure::RootCorridorCrossesSeam);
    }
    Ok(AuthoredRootCut {
        source,
        root_parameter,
        before_parameter,
        after_parameter,
    })
}

fn backward_cyclic_distance(root: f64, before: f64) -> f64 {
    if before <= root {
        root - before
    } else {
        root + (TAU - before)
    }
}

fn forward_cyclic_distance(root: f64, after: f64) -> f64 {
    if after >= root {
        after - root
    } else {
        (TAU - root) + after
    }
}

fn validate_authored_root_order(
    cuts: &[AuthoredRootCut],
    range: ParamRange,
) -> Result<(), SkewCylinderOpenSpanFailure> {
    for pair in cuts.windows(2) {
        if pair[0].root_parameter == pair[1].root_parameter
            || pair[0].after_parameter >= pair[1].before_parameter
        {
            return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
        }
    }
    if cuts.iter().any(|cut| {
        !range.contains(cut.root_parameter)
            || !range.contains(cut.before_parameter)
            || !range.contains(cut.after_parameter)
    }) {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    Ok(())
}

fn sweep_topologies(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    cuts: &[AuthoredRootCut],
) -> Result<([bool; 2], Vec<CutTransition>), SkewCylinderOpenSpanFailure> {
    let mut states = [[SkewCylinderAxialRelation::Below; 2]; 4];
    for (topology_index, topology) in topologies.iter().enumerate() {
        states[topology_index] = if topology.roots().is_empty() {
            topology.open_cell_relations()[0]
        } else {
            let first = cuts
                .iter()
                .find(|cut| cut.source.topology_index == topology_index)
                .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
            let count = topology.open_cell_relations().len();
            topology.open_cell_relations()[(first.source.cyclic_ordinal + count - 1) % count]
        };
    }
    let initial_states = states;
    let initial_inside = sheet_inside(topologies, &states);
    let mut transitions = Vec::with_capacity(cuts.len());
    for cut in cuts {
        let before_inside = sheet_inside(topologies, &states);
        let topology = topologies[cut.source.topology_index];
        let count = topology.open_cell_relations().len();
        let expected_before =
            topology.open_cell_relations()[(cut.source.cyclic_ordinal + count - 1) % count];
        if states[cut.source.topology_index] != expected_before {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
        states[cut.source.topology_index] =
            topology.open_cell_relations()[cut.source.cyclic_ordinal];
        let after_inside = sheet_inside(topologies, &states);
        transitions.push(CutTransition {
            cut: *cut,
            before_inside,
            after_inside,
        });
    }
    if states != initial_states {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok((initial_inside, transitions))
}

fn sheet_inside(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    states: &[[SkewCylinderAxialRelation; 2]; 4],
) -> [bool; 2] {
    core::array::from_fn(|sheet| {
        topologies.iter().enumerate().all(|(index, topology)| {
            states[index][sheet] == required_relation(topology.provenance())
        })
    })
}

fn classify_sheet(
    sheet: SkewCylinderSheet,
    initial_inside: bool,
    transitions: &[CutTransition],
    carrier_range: ParamRange,
) -> Result<SkewCylinderFiniteSheetTopology, SkewCylinderOpenSpanFailure> {
    let sheet_index = sheet_index(sheet);
    let changed = transitions
        .iter()
        .enumerate()
        .filter(|(_, transition)| {
            transition.before_inside[sheet_index] != transition.after_inside[sheet_index]
        })
        .collect::<Vec<_>>();
    if changed.is_empty() {
        return Ok(if initial_inside {
            SkewCylinderFiniteSheetTopology::Whole
        } else {
            SkewCylinderFiniteSheetTopology::Outside
        });
    }
    if changed.len() % 2 != 0 {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    for (current, next) in changed
        .iter()
        .zip(changed.iter().cycle().skip(1))
        .take(changed.len())
    {
        if current.1.after_inside[sheet_index] == next.1.after_inside[sheet_index] {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
    }

    let mut spans = Vec::with_capacity(changed.len() / 2);
    for (position, (start_index, start_transition)) in changed.iter().enumerate() {
        if start_transition.before_inside[sheet_index]
            || !start_transition.after_inside[sheet_index]
        {
            continue;
        }
        let (end_index, end_transition) = changed[(position + 1) % changed.len()];
        if !end_transition.before_inside[sheet_index] || end_transition.after_inside[sheet_index] {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
        let start_root = start_transition.cut.source.events[sheet_index]
            .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
        let end_root = end_transition.cut.source.events[sheet_index]
            .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
        if start_root.after != required_relation(start_root.provenance)
            || end_root.before != required_relation(end_root.provenance)
        {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
        let start_parameter = start_transition.cut.after_parameter;
        let end_parameter = end_transition.cut.before_parameter;
        if end_index <= *start_index && end_transition.cut.root_parameter != carrier_range.lo {
            return Err(SkewCylinderOpenSpanFailure::SeamWrappingSpan);
        }
        if !start_parameter.is_finite()
            || !end_parameter.is_finite()
            || start_parameter >= end_parameter
        {
            return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
        }
        let range = ParamRange::new(start_parameter, end_parameter);
        if !range.is_finite() || range.width() <= 0.0 || range.width() >= TAU {
            return Err(SkewCylinderOpenSpanFailure::SeamWrappingSpan);
        }
        spans.push(SkewCylinderOpenSpan {
            sheet,
            range,
            start: SkewCylinderOpenSpanEndpointProof {
                root: start_root,
                inside_side: SkewCylinderRootInsideSide::After,
                carrier_parameter: start_parameter,
            },
            end: SkewCylinderOpenSpanEndpointProof {
                root: end_root,
                inside_side: SkewCylinderRootInsideSide::Before,
                carrier_parameter: end_parameter,
            },
        });
    }
    spans.sort_by(|lhs, rhs| lhs.range.lo.total_cmp(&rhs.range.lo));
    if spans.is_empty() {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok(SkewCylinderFiniteSheetTopology::Open(spans))
}

fn required_relation(provenance: SkewCylinderAxialBoundProvenance) -> SkewCylinderAxialRelation {
    match provenance.boundary {
        SkewCylinderAxialBoundary::Lower => SkewCylinderAxialRelation::Above,
        SkewCylinderAxialBoundary::Upper => SkewCylinderAxialRelation::Below,
    }
}

const fn sheet_index(sheet: SkewCylinderSheet) -> usize {
    match sheet {
        SkewCylinderSheet::Lower => 0,
        SkewCylinderSheet::Upper => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::super::{SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, classify_skew_cylinder_axial_bound};
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::surface::Cylinder;
    use kgeom::vec::{Point3, Vec3};

    fn perpendicular_pair(offset: f64) -> [Cylinder; 2] {
        [
            Cylinder::new(Frame::world(), 1.0).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(0.0, offset, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
                2.0,
            )
            .unwrap(),
        ]
    }

    fn topologies(
        cylinders: [Cylinder; 2],
        ranges: [[ParamRange; 2]; 2],
    ) -> [SkewCylinderAxialBoundTopology; 4] {
        [
            (0, SkewCylinderAxialBoundary::Lower, ranges[0][1].lo),
            (0, SkewCylinderAxialBoundary::Upper, ranges[0][1].hi),
            (1, SkewCylinderAxialBoundary::Lower, ranges[1][1].lo),
            (1, SkewCylinderAxialBoundary::Upper, ranges[1][1].hi),
        ]
        .map(|(source_operand, boundary, value)| {
            classify_skew_cylinder_axial_bound(
                cylinders,
                [0, 1],
                SkewCylinderAxialBoundProvenance {
                    source_operand,
                    boundary,
                    value,
                },
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )
            .unwrap()
        })
    }

    fn clipped_ranges(longitude: ParamRange) -> [[ParamRange; 2]; 2] {
        [
            [longitude, ParamRange::new(1.8, 2.1)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 0.0)],
        ]
    }

    fn classify(
        topologies: &[SkewCylinderAxialBoundTopology; 4],
        ranges: [[ParamRange; 2]; 2],
    ) -> Result<SkewCylinderFiniteWindowTopologyCertificate, SkewCylinderOpenSpanFailure> {
        classify_skew_cylinder_open_spans(SkewCylinderOpenSpanTopologyInput {
            topologies,
            ranges,
            canonical_to_source: [0, 1],
        })
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= 1.0e-12,
            "{actual:.17e} != {expected:.17e}"
        );
    }

    #[test]
    fn clipped_perpendicular_window_has_one_upper_nonwrapping_span() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = clipped_ranges(ParamRange::new(0.0, TAU));
        let source = topologies(cylinders, ranges);
        let result = classify(&source, ranges).unwrap();

        assert_eq!(
            result.sheet(SkewCylinderSheet::Lower),
            &SkewCylinderFiniteSheetTopology::Outside
        );
        let SkewCylinderFiniteSheetTopology::Open(spans) = result.sheet(SkewCylinderSheet::Upper)
        else {
            panic!("{result:?}");
        };
        assert_eq!(spans.len(), 1);
        let span = spans[0];
        assert_eq!(span.sheet, SkewCylinderSheet::Upper);
        assert_close(span.range.lo, 2.082769014844373);
        assert_close(span.range.hi, 4.200416292335213);
        assert_eq!(
            span.start.root.provenance,
            SkewCylinderAxialBoundProvenance {
                source_operand: 0,
                boundary: SkewCylinderAxialBoundary::Lower,
                value: 1.8,
            }
        );
        assert_eq!(span.end.root.provenance, span.start.root.provenance);
        assert_eq!(span.start.inside_side, SkewCylinderRootInsideSide::After);
        assert_eq!(span.end.inside_side, SkewCylinderRootInsideSide::Before);
        assert_eq!(span.start.carrier_parameter, span.range.lo);
        assert_eq!(span.end.carrier_parameter, span.range.hi);
        assert!(!span.start.root.repeated && !span.end.root.repeated);
        assert_ne!(span.start.root.before, span.start.root.after);
        assert_ne!(span.end.root.before, span.end.root.after);
        let [lower_root, upper_root] = span
            .root_longitude_intervals(ranges[0][0])
            .expect("both projective roots must lift into the retained longitude chart");
        assert!(ranges[0][0].lo < lower_root.lo());
        assert!(lower_root.hi() < span.range.lo);
        assert!(span.range.hi < upper_root.lo());
        assert!(upper_root.hi() < ranges[0][0].hi);
    }

    #[test]
    fn two_active_axial_windows_have_four_upper_nonwrapping_spans() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(1.8, 1.9)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 1.25)],
        ];
        let mut source = topologies(cylinders, ranges);
        let result = classify(&source, ranges).unwrap();
        source.reverse();
        assert_eq!(classify(&source, ranges).unwrap(), result);

        assert_eq!(
            result.sheet(SkewCylinderSheet::Lower),
            &SkewCylinderFiniteSheetTopology::Outside
        );
        let SkewCylinderFiniteSheetTopology::Open(spans) = result.sheet(SkewCylinderSheet::Upper)
        else {
            panic!("{result:?}");
        };
        assert_eq!(spans.len(), 4);
        for (ordinal, span) in spans.iter().enumerate() {
            assert_eq!(span.sheet, SkewCylinderSheet::Upper);
            assert!(0.0 < span.range.lo && span.range.hi < TAU);
            assert!(0.0 < span.range.width() && span.range.width() < TAU);
            assert_eq!(span.start.root.cyclic_ordinal, ordinal);
            assert_eq!(span.end.root.cyclic_ordinal, ordinal);
            assert_eq!(
                span.start.root.provenance.source_operand,
                span.end.root.provenance.source_operand
            );
            assert_eq!(span.start.root.provenance.source_operand, 0);
            assert_ne!(
                span.start.root.provenance.boundary,
                span.end.root.provenance.boundary
            );
            assert_eq!(span.start.carrier_parameter, span.range.lo);
            assert_eq!(span.end.carrier_parameter, span.range.hi);
            assert_eq!(span.start.inside_side, SkewCylinderRootInsideSide::After);
            assert_eq!(span.end.inside_side, SkewCylinderRootInsideSide::Before);
            assert!(!span.start.root.repeated && !span.end.root.repeated);
            assert_ne!(span.start.root.before, span.start.root.after);
            assert_ne!(span.end.root.before, span.end.root.after);
            let [lower_root, upper_root] = span
                .root_longitude_intervals(ranges[0][0])
                .expect("each projective root pair must lift without wrapping");
            assert!(lower_root.hi() < span.range.lo);
            assert!(span.range.hi < upper_root.lo());
        }
        for pair in spans.windows(2) {
            assert!(pair[0].range.hi < pair[1].range.lo);
        }
    }

    #[test]
    fn occupied_component_crossing_the_authored_seam_is_refused() {
        let cylinders = perpendicular_pair(0.0);
        let canonical_ranges = clipped_ranges(ParamRange::new(0.0, TAU));
        let source = topologies(cylinders, canonical_ranges);
        let shifted_ranges = clipped_ranges(ParamRange::new(
            -core::f64::consts::PI,
            core::f64::consts::PI,
        ));

        assert_eq!(
            classify(&source, shifted_ranges),
            Err(SkewCylinderOpenSpanFailure::SeamWrappingSpan)
        );
    }

    #[test]
    fn repeated_axial_contact_root_is_refused_before_span_assembly() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(-3.0, 3.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.0, 1.0)],
        ];
        let source = topologies(cylinders, ranges);
        assert_eq!(
            classify(&source, ranges),
            Err(SkewCylinderOpenSpanFailure::ContactRoot)
        );
    }

    #[test]
    fn source_bound_range_mismatch_is_refused() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = clipped_ranges(ParamRange::new(0.0, TAU));
        let source = topologies(cylinders, ranges);
        let mut mismatched_ranges = ranges;
        mismatched_ranges[0][1].lo = 1.8_f64.next_up();
        assert_eq!(
            classify(&source, mismatched_ranges),
            Err(SkewCylinderOpenSpanFailure::RangeMismatch)
        );
    }

    #[test]
    fn coincident_cross_bound_root_corridors_are_refused() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(1.8, 1.8)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 0.0)],
        ];
        let source = topologies(cylinders, ranges);
        assert_eq!(
            classify(&source, ranges),
            Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots)
        );
    }

    #[test]
    fn expanded_guard_overlap_is_refused_before_range_construction() {
        let provenance = SkewCylinderAxialBoundProvenance {
            source_operand: 0,
            boundary: SkewCylinderAxialBoundary::Lower,
            value: 1.8,
        };
        let root = |cyclic_ordinal, before, after| SkewCylinderAxialRoot {
            provenance,
            sheet: SkewCylinderSheet::Upper,
            cyclic_ordinal,
            bracket: SkewCylinderHalfAngleRootBracket {
                chart: SkewCylinderHalfAngleChart::Tangent,
                lo: 0.25,
                hi: 0.25,
            },
            repeated: false,
            before,
            after,
        };
        let authored_cut =
            |root: SkewCylinderAxialRoot, root_parameter, before_parameter, after_parameter| {
                AuthoredRootCut {
                    source: RootCut {
                        topology_index: 0,
                        cyclic_ordinal: root.cyclic_ordinal,
                        bracket: root.bracket,
                        events: [None, Some(root)],
                    },
                    root_parameter,
                    before_parameter,
                    after_parameter,
                }
            };
        let transitions = [
            CutTransition {
                cut: authored_cut(
                    root(
                        0,
                        SkewCylinderAxialRelation::Below,
                        SkewCylinderAxialRelation::Above,
                    ),
                    0.9,
                    0.8,
                    2.0,
                ),
                before_inside: [false, false],
                after_inside: [false, true],
            },
            CutTransition {
                cut: authored_cut(
                    root(
                        1,
                        SkewCylinderAxialRelation::Above,
                        SkewCylinderAxialRelation::Below,
                    ),
                    2.1,
                    1.0,
                    2.2,
                ),
                before_inside: [false, true],
                after_inside: [false, false],
            },
        ];

        assert_eq!(
            classify_sheet(
                SkewCylinderSheet::Upper,
                false,
                &transitions,
                ParamRange::new(0.0, TAU),
            ),
            Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots)
        );
    }
}
