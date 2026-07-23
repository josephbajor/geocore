//! Pure exact-order sweep planning for two closed axial source intervals.
//!
//! Geometry and tolerance policy live upstream. This module accepts only the
//! six pair classifications of four topology-owned authored endpoints,
//! validates that they form one total preorder, and derives regularized CSG
//! spans from open-cell membership. Selected adjacent cells are closed and
//! coalesced; every returned boundary retains all source endpoint identities
//! in its exact equality class.

use core::cmp::Ordering;

use super::boundary_select::RegularizedBooleanOperation;

/// Ordered Boolean operand owning one axial interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AxialIntervalOperand {
    Left,
    Right,
}

impl AxialIntervalOperand {
    const fn index(self) -> usize {
        match self {
            Self::Left => 0,
            Self::Right => 1,
        }
    }
}

const OPERANDS: [AxialIntervalOperand; 2] =
    [AxialIntervalOperand::Left, AxialIntervalOperand::Right];

/// One endpoint in its source cylinder's authored-axis order.
///
/// `Start` and `End` are identities, not claims about physical low/high order.
/// An antiparallel authored axis therefore needs no special planner path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AuthoredAxialEndpoint {
    Start,
    End,
}

/// Stable identity of one topology-owned axial endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct AxialEndpointContributor {
    operand: AxialIntervalOperand,
    endpoint: AuthoredAxialEndpoint,
}

impl AxialEndpointContributor {
    pub(crate) const fn new(
        operand: AxialIntervalOperand,
        endpoint: AuthoredAxialEndpoint,
    ) -> Self {
        Self { operand, endpoint }
    }

    pub(crate) const fn operand(self) -> AxialIntervalOperand {
        self.operand
    }

    pub(crate) const fn endpoint(self) -> AuthoredAxialEndpoint {
        self.endpoint
    }

    const fn index(self) -> usize {
        match (self.operand, self.endpoint) {
            (AxialIntervalOperand::Left, AuthoredAxialEndpoint::Start) => 0,
            (AxialIntervalOperand::Left, AuthoredAxialEndpoint::End) => 1,
            (AxialIntervalOperand::Right, AuthoredAxialEndpoint::Start) => 2,
            (AxialIntervalOperand::Right, AuthoredAxialEndpoint::End) => 3,
        }
    }
}

const LEFT_START: AxialEndpointContributor =
    AxialEndpointContributor::new(AxialIntervalOperand::Left, AuthoredAxialEndpoint::Start);
const LEFT_END: AxialEndpointContributor =
    AxialEndpointContributor::new(AxialIntervalOperand::Left, AuthoredAxialEndpoint::End);
const RIGHT_START: AxialEndpointContributor =
    AxialEndpointContributor::new(AxialIntervalOperand::Right, AuthoredAxialEndpoint::Start);
const RIGHT_END: AxialEndpointContributor =
    AxialEndpointContributor::new(AxialIntervalOperand::Right, AuthoredAxialEndpoint::End);
const ENDPOINTS: [AxialEndpointContributor; 4] = [LEFT_START, LEFT_END, RIGHT_START, RIGHT_END];

/// One exact proof-fed pair classification.
///
/// The caller remains responsible for deriving `ordering` from certified
/// predicates rather than coordinates or tolerances. The preorder constructor
/// below validates completeness and consistency before planning can begin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct AxialEndpointComparison {
    first: AxialEndpointContributor,
    second: AxialEndpointContributor,
    ordering: Ordering,
}

impl AxialEndpointComparison {
    pub(crate) const fn new(
        first: AxialEndpointContributor,
        second: AxialEndpointContributor,
        ordering: Ordering,
    ) -> Self {
        Self {
            first,
            second,
            ordering,
        }
    }
}

/// Structural failure in the claimed four-endpoint total preorder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AxialEndpointPreorderError {
    SelfComparison,
    DuplicateComparison,
    IncompleteComparison,
    NonTransitive,
}

/// Structurally certified total preorder of all four authored endpoints.
///
/// Construction is the only route into this type. Equal endpoints share one
/// rank and every rank retains a contributor set in deterministic identity
/// order, independently of comparison presentation order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CertifiedAxialEndpointPreorder {
    ranks: [u8; 4],
    classes: [AxialEndpointContributors; 4],
    class_count: u8,
}

impl CertifiedAxialEndpointPreorder {
    /// Validate the complete six-pair classification as one total preorder.
    pub(crate) fn from_comparisons(
        comparisons: [AxialEndpointComparison; 6],
    ) -> Result<Self, AxialEndpointPreorderError> {
        let mut relations = [[None; 4]; 4];
        for index in 0..4 {
            relations[index][index] = Some(Ordering::Equal);
        }
        for comparison in comparisons {
            let first = comparison.first.index();
            let second = comparison.second.index();
            if first == second {
                return Err(AxialEndpointPreorderError::SelfComparison);
            }
            if relations[first][second].is_some() {
                return Err(AxialEndpointPreorderError::DuplicateComparison);
            }
            relations[first][second] = Some(comparison.ordering);
            relations[second][first] = Some(comparison.ordering.reverse());
        }
        if relations.iter().flatten().any(Option::is_none) {
            return Err(AxialEndpointPreorderError::IncompleteComparison);
        }

        for first in 0..4 {
            for second in 0..4 {
                for third in 0..4 {
                    let (Some(first_second), Some(second_third), Some(first_third)) = (
                        relations[first][second],
                        relations[second][third],
                        relations[first][third],
                    ) else {
                        return Err(AxialEndpointPreorderError::IncompleteComparison);
                    };
                    if first_second != Ordering::Greater
                        && second_third != Ordering::Greater
                        && first_third == Ordering::Greater
                    {
                        return Err(AxialEndpointPreorderError::NonTransitive);
                    }
                }
            }
        }

        let mut ranks = [0_u8; 4];
        let mut classes = [AxialEndpointContributors::empty(); 4];
        let mut remaining = [true; 4];
        let mut class_count = 0_u8;
        while remaining.into_iter().any(|present| present) {
            let minimum = (0..4).find(|&candidate| {
                remaining[candidate]
                    && (0..4).all(|other| {
                        !remaining[other] || relations[other][candidate] != Some(Ordering::Less)
                    })
            });
            let Some(minimum) = minimum else {
                return Err(AxialEndpointPreorderError::NonTransitive);
            };
            for endpoint in 0..4 {
                if remaining[endpoint] && relations[minimum][endpoint] == Some(Ordering::Equal) {
                    remaining[endpoint] = false;
                    ranks[endpoint] = class_count;
                    classes[class_count as usize].insert(ENDPOINTS[endpoint]);
                }
            }
            class_count += 1;
        }

        Ok(Self {
            ranks,
            classes,
            class_count,
        })
    }

    /// Compare two retained endpoint identities in the certified preorder.
    pub(crate) const fn compare(
        &self,
        first: AxialEndpointContributor,
        second: AxialEndpointContributor,
    ) -> Ordering {
        let first = self.ranks[first.index()];
        let second = self.ranks[second.index()];
        if first < second {
            Ordering::Less
        } else if first > second {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}

/// Exact-equality class of topology endpoint contributors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct AxialEndpointContributors(u8);

impl AxialEndpointContributors {
    const fn empty() -> Self {
        Self(0)
    }

    fn insert(&mut self, endpoint: AxialEndpointContributor) {
        self.0 |= 1 << endpoint.index();
    }

    /// Whether this exact boundary class contains `endpoint`.
    pub(crate) const fn contains(self, endpoint: AxialEndpointContributor) -> bool {
        self.0 & (1 << endpoint.index()) != 0
    }

    /// Contributors in stable Left-Start, Left-End, Right-Start, Right-End order.
    pub(crate) fn iter(self) -> impl Iterator<Item = AxialEndpointContributor> {
        ENDPOINTS
            .into_iter()
            .filter(move |endpoint| self.contains(*endpoint))
    }
}

/// Source intervals contributing side material to one selected span.
///
/// This is the union of exact source memberships over the span's selected
/// open cells. It is therefore direct lineage input: realization need not
/// reconstruct source-side provenance from the operation or interval shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct AxialOperandContributors(u8);

impl AxialOperandContributors {
    const fn empty() -> Self {
        Self(0)
    }

    fn insert(&mut self, operand: AxialIntervalOperand) {
        self.0 |= 1 << operand.index();
    }

    fn extend(&mut self, other: Self) {
        self.0 |= other.0;
    }

    /// Whether this span has selected open cells covered by `operand`.
    pub(crate) const fn contains(self, operand: AxialIntervalOperand) -> bool {
        self.0 & (1 << operand.index()) != 0
    }

    /// Contributors in stable Left, Right order.
    pub(crate) fn iter(self) -> impl Iterator<Item = AxialIntervalOperand> {
        OPERANDS
            .into_iter()
            .filter(move |operand| self.contains(*operand))
    }
}

/// One closed maximal selected axial span.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PlannedAxialSpan {
    low: AxialEndpointContributors,
    high: AxialEndpointContributors,
    side_operands: AxialOperandContributors,
}

impl PlannedAxialSpan {
    pub(crate) const fn low(&self) -> AxialEndpointContributors {
        self.low
    }

    pub(crate) const fn high(&self) -> AxialEndpointContributors {
        self.high
    }

    /// Source operands covering at least one selected open cell in this span.
    pub(crate) const fn side_operands(&self) -> AxialOperandContributors {
        self.side_operands
    }
}

/// Complete regularized interval plan; two source intervals yield at most two spans.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct AxialIntervalPlan {
    spans: Vec<PlannedAxialSpan>,
}

impl AxialIntervalPlan {
    pub(crate) fn spans(&self) -> &[PlannedAxialSpan] {
        &self.spans
    }
}

/// Apply regularized CSG truth to the certified four-endpoint sweep.
///
/// Only open cells carry material truth. Each maximal selected cell run is
/// closed at its bounding endpoint classes, and adjacent selected cells are
/// therefore coalesced across exact contacts and shared endpoints.
pub(crate) fn plan_axial_interval_sweep(
    operation: RegularizedBooleanOperation,
    preorder: &CertifiedAxialEndpointPreorder,
) -> AxialIntervalPlan {
    let cell_count = preorder.class_count.saturating_sub(1) as usize;
    let mut selected = [false; 3];
    let mut cell_contributors = [AxialOperandContributors::empty(); 3];
    let left = source_rank_interval(preorder, AxialIntervalOperand::Left);
    let right = source_rank_interval(preorder, AxialIntervalOperand::Right);
    for cell in 0..cell_count {
        let rank = cell as u8;
        let left_inside = left.0 <= rank && rank < left.1;
        let right_inside = right.0 <= rank && rank < right.1;
        if left_inside {
            cell_contributors[cell].insert(AxialIntervalOperand::Left);
        }
        if right_inside {
            cell_contributors[cell].insert(AxialIntervalOperand::Right);
        }
        selected[cell] = regularized_truth(operation, left_inside, right_inside);
    }

    let mut spans = Vec::with_capacity(2);
    let mut cell = 0;
    while cell < cell_count {
        if !selected[cell] {
            cell += 1;
            continue;
        }
        let first = cell;
        while cell + 1 < cell_count && selected[cell + 1] {
            cell += 1;
        }
        let mut side_contributors = AxialOperandContributors::empty();
        for contributors in &cell_contributors[first..=cell] {
            side_contributors.extend(*contributors);
        }
        spans.push(PlannedAxialSpan {
            low: preorder.classes[first],
            high: preorder.classes[cell + 1],
            side_operands: side_contributors,
        });
        cell += 1;
    }
    debug_assert!(spans.len() <= 2);
    AxialIntervalPlan { spans }
}

fn source_rank_interval(
    preorder: &CertifiedAxialEndpointPreorder,
    operand: AxialIntervalOperand,
) -> (u8, u8) {
    let start = AxialEndpointContributor::new(operand, AuthoredAxialEndpoint::Start).index();
    let end = AxialEndpointContributor::new(operand, AuthoredAxialEndpoint::End).index();
    let first = preorder.ranks[start];
    let second = preorder.ranks[end];
    (first.min(second), first.max(second))
}

const fn regularized_truth(
    operation: RegularizedBooleanOperation,
    left_inside: bool,
    right_inside: bool,
) -> bool {
    match operation {
        RegularizedBooleanOperation::Unite => left_inside || right_inside,
        RegularizedBooleanOperation::Intersect => left_inside && right_inside,
        RegularizedBooleanOperation::Subtract => left_inside && !right_inside,
    }
}

#[cfg(test)]
mod tests {
    use core::array;

    use super::*;

    const PAIRS: [(AxialEndpointContributor, AxialEndpointContributor); 6] = [
        (LEFT_START, LEFT_END),
        (LEFT_START, RIGHT_START),
        (LEFT_START, RIGHT_END),
        (LEFT_END, RIGHT_START),
        (LEFT_END, RIGHT_END),
        (RIGHT_START, RIGHT_END),
    ];
    const LEFT_ONLY: [AxialIntervalOperand; 1] = [AxialIntervalOperand::Left];
    const RIGHT_ONLY: [AxialIntervalOperand; 1] = [AxialIntervalOperand::Right];
    const BOTH_OPERANDS: [AxialIntervalOperand; 2] =
        [AxialIntervalOperand::Left, AxialIntervalOperand::Right];

    fn comparisons(ranks: [u8; 4]) -> [AxialEndpointComparison; 6] {
        PAIRS.map(|(first, second)| {
            AxialEndpointComparison::new(
                first,
                second,
                ranks[first.index()].cmp(&ranks[second.index()]),
            )
        })
    }

    fn preorder(ranks: [u8; 4]) -> CertifiedAxialEndpointPreorder {
        CertifiedAxialEndpointPreorder::from_comparisons(comparisons(ranks)).unwrap()
    }

    fn plan(ranks: [u8; 4], operation: RegularizedBooleanOperation) -> AxialIntervalPlan {
        plan_axial_interval_sweep(operation, &preorder(ranks))
    }

    fn contributors(endpoints: &[AxialEndpointContributor]) -> AxialEndpointContributors {
        let mut contributors = AxialEndpointContributors::empty();
        for endpoint in endpoints {
            contributors.insert(*endpoint);
        }
        contributors
    }

    fn span(
        low: &[AxialEndpointContributor],
        high: &[AxialEndpointContributor],
        side_operands: &[AxialIntervalOperand],
    ) -> PlannedAxialSpan {
        let mut side_contributors = AxialOperandContributors::empty();
        for operand in side_operands {
            side_contributors.insert(*operand);
        }
        PlannedAxialSpan {
            low: contributors(low),
            high: contributors(high),
            side_operands: side_contributors,
        }
    }

    fn assert_spans(
        ranks: [u8; 4],
        operation: RegularizedBooleanOperation,
        expected: &[PlannedAxialSpan],
    ) {
        assert_eq!(plan(ranks, operation).spans(), expected);
    }

    #[test]
    fn canonical_gap_contact_crossing_shared_nesting_and_equality_cases() {
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        let gap = [0, 1, 2, 3];
        assert_spans(
            gap,
            Unite,
            &[
                span(&[LEFT_START], &[LEFT_END], &LEFT_ONLY),
                span(&[RIGHT_START], &[RIGHT_END], &RIGHT_ONLY),
            ],
        );
        assert_spans(gap, Intersect, &[]);
        assert_spans(
            gap,
            Subtract,
            &[span(&[LEFT_START], &[LEFT_END], &LEFT_ONLY)],
        );

        let contact = [0, 1, 1, 2];
        assert_spans(
            contact,
            Unite,
            &[span(&[LEFT_START], &[RIGHT_END], &BOTH_OPERANDS)],
        );
        assert_spans(contact, Intersect, &[]);
        assert_spans(
            contact,
            Subtract,
            &[span(&[LEFT_START], &[LEFT_END, RIGHT_START], &LEFT_ONLY)],
        );

        let crossing = [0, 2, 1, 3];
        assert_spans(
            crossing,
            Unite,
            &[span(&[LEFT_START], &[RIGHT_END], &BOTH_OPERANDS)],
        );
        assert_spans(
            crossing,
            Intersect,
            &[span(&[RIGHT_START], &[LEFT_END], &BOTH_OPERANDS)],
        );
        assert_spans(
            crossing,
            Subtract,
            &[span(&[LEFT_START], &[RIGHT_START], &LEFT_ONLY)],
        );

        let shared_low = [0, 2, 0, 1];
        assert_spans(
            shared_low,
            Unite,
            &[span(
                &[LEFT_START, RIGHT_START],
                &[LEFT_END],
                &BOTH_OPERANDS,
            )],
        );
        assert_spans(
            shared_low,
            Intersect,
            &[span(
                &[LEFT_START, RIGHT_START],
                &[RIGHT_END],
                &BOTH_OPERANDS,
            )],
        );
        assert_spans(
            shared_low,
            Subtract,
            &[span(&[RIGHT_END], &[LEFT_END], &LEFT_ONLY)],
        );

        let shared_high = [0, 2, 1, 2];
        assert_spans(
            shared_high,
            Unite,
            &[span(&[LEFT_START], &[LEFT_END, RIGHT_END], &BOTH_OPERANDS)],
        );
        assert_spans(
            shared_high,
            Intersect,
            &[span(&[RIGHT_START], &[LEFT_END, RIGHT_END], &BOTH_OPERANDS)],
        );
        assert_spans(
            shared_high,
            Subtract,
            &[span(&[LEFT_START], &[RIGHT_START], &LEFT_ONLY)],
        );

        let nested = [0, 3, 1, 2];
        assert_spans(
            nested,
            Unite,
            &[span(&[LEFT_START], &[LEFT_END], &BOTH_OPERANDS)],
        );
        assert_spans(
            nested,
            Intersect,
            &[span(&[RIGHT_START], &[RIGHT_END], &BOTH_OPERANDS)],
        );
        assert_spans(
            nested,
            Subtract,
            &[
                span(&[LEFT_START], &[RIGHT_START], &LEFT_ONLY),
                span(&[RIGHT_END], &[LEFT_END], &LEFT_ONLY),
            ],
        );

        let equal = [0, 1, 0, 1];
        let equal_span = span(
            &[LEFT_START, RIGHT_START],
            &[LEFT_END, RIGHT_END],
            &BOTH_OPERANDS,
        );
        assert_spans(equal, Unite, &[equal_span]);
        assert_spans(equal, Intersect, &[equal_span]);
        assert_spans(equal, Subtract, &[]);
    }

    fn remap_contributors(
        source: AxialEndpointContributors,
        mapping: [AxialEndpointContributor; 4],
    ) -> AxialEndpointContributors {
        let mut mapped = AxialEndpointContributors::empty();
        for endpoint in source.iter() {
            mapped.insert(mapping[endpoint.index()]);
        }
        mapped
    }

    fn remap_plan(
        source: &AxialIntervalPlan,
        endpoint_mapping: [AxialEndpointContributor; 4],
        operand_mapping: [AxialIntervalOperand; 2],
    ) -> AxialIntervalPlan {
        AxialIntervalPlan {
            spans: source
                .spans()
                .iter()
                .map(|source| {
                    let mut side_contributors = AxialOperandContributors::empty();
                    for operand in source.side_operands().iter() {
                        side_contributors.insert(operand_mapping[operand.index()]);
                    }
                    PlannedAxialSpan {
                        low: remap_contributors(source.low(), endpoint_mapping),
                        high: remap_contributors(source.high(), endpoint_mapping),
                        side_operands: side_contributors,
                    }
                })
                .collect(),
        }
    }

    #[test]
    fn operand_swap_and_reversed_authored_axes_only_remap_identity() {
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        let ranks = [0, 3, 1, 2];
        let swapped_ranks = [ranks[2], ranks[3], ranks[0], ranks[1]];
        let swap_mapping = [RIGHT_START, RIGHT_END, LEFT_START, LEFT_END];
        let swapped_operands = [AxialIntervalOperand::Right, AxialIntervalOperand::Left];
        for operation in [Unite, Intersect] {
            assert_eq!(
                plan(swapped_ranks, operation),
                remap_plan(&plan(ranks, operation), swap_mapping, swapped_operands)
            );
        }
        assert_spans(swapped_ranks, Subtract, &[]);

        let reversed_ranks = [ranks[1], ranks[0], ranks[3], ranks[2]];
        let reverse_mapping = [LEFT_END, LEFT_START, RIGHT_END, RIGHT_START];
        for operation in [Unite, Intersect, Subtract] {
            assert_eq!(
                plan(reversed_ranks, operation),
                remap_plan(&plan(ranks, operation), reverse_mapping, OPERANDS)
            );
        }
    }

    fn comparison_permutations(
        values: &mut [usize; 6],
        index: usize,
        output: &mut Vec<[usize; 6]>,
    ) {
        if index == values.len() {
            output.push(*values);
            return;
        }
        for swap in index..values.len() {
            values.swap(index, swap);
            comparison_permutations(values, index + 1, output);
            values.swap(index, swap);
        }
    }

    fn reversed(comparison: AxialEndpointComparison) -> AxialEndpointComparison {
        AxialEndpointComparison::new(
            comparison.second,
            comparison.first,
            comparison.ordering.reverse(),
        )
    }

    #[test]
    fn every_comparison_permutation_and_direction_has_one_deterministic_plan() {
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        let base = comparisons([0, 2, 1, 2]);
        let expected = [Unite, Intersect, Subtract].map(|operation| {
            plan_axial_interval_sweep(
                operation,
                &CertifiedAxialEndpointPreorder::from_comparisons(base).unwrap(),
            )
        });
        let mut permutations = Vec::new();
        comparison_permutations(&mut [0, 1, 2, 3, 4, 5], 0, &mut permutations);
        assert_eq!(permutations.len(), 720);
        for permutation in permutations {
            let presented = array::from_fn(|index| {
                let comparison = base[permutation[index]];
                if (index + permutation[index]) % 2 == 0 {
                    comparison
                } else {
                    reversed(comparison)
                }
            });
            let preorder = CertifiedAxialEndpointPreorder::from_comparisons(presented).unwrap();
            for (index, operation) in [Unite, Intersect, Subtract].into_iter().enumerate() {
                assert_eq!(
                    plan_axial_interval_sweep(operation, &preorder),
                    expected[index]
                );
            }
        }
    }

    fn boundary_rank(boundary: AxialEndpointContributors, ranks: [u8; 4]) -> u8 {
        let mut endpoints = boundary.iter();
        let rank = ranks[endpoints.next().unwrap().index()];
        assert!(endpoints.all(|endpoint| ranks[endpoint.index()] == rank));
        let expected = ENDPOINTS
            .into_iter()
            .filter(|endpoint| ranks[endpoint.index()] == rank)
            .collect::<Vec<_>>();
        assert_eq!(boundary, contributors(&expected));
        rank
    }

    #[test]
    fn all_seventy_five_four_endpoint_weak_orders_match_open_cell_truth() {
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        let mut weak_orders = 0;
        for encoded in 0_u16..256 {
            let ranks = array::from_fn(|index| ((encoded >> (2 * index)) & 3) as u8);
            let maximum = *ranks.iter().max().unwrap();
            if (0..=maximum).any(|rank| !ranks.contains(&rank)) {
                continue;
            }
            weak_orders += 1;
            let class_count = maximum + 1;
            for operation in [Unite, Intersect, Subtract] {
                let plan = plan(ranks, operation);
                assert!(plan.spans().len() <= 2);
                for cell in 0..class_count.saturating_sub(1) {
                    let left_low = ranks[0].min(ranks[1]);
                    let left_high = ranks[0].max(ranks[1]);
                    let right_low = ranks[2].min(ranks[3]);
                    let right_high = ranks[2].max(ranks[3]);
                    let left_inside = left_low <= cell && cell < left_high;
                    let right_inside = right_low <= cell && cell < right_high;
                    let expected = regularized_truth(operation, left_inside, right_inside);
                    let planned = plan.spans().iter().any(|span| {
                        boundary_rank(span.low(), ranks) <= cell
                            && cell < boundary_rank(span.high(), ranks)
                    });
                    assert_eq!(planned, expected, "ranks={ranks:?} cell={cell}");
                }
                for span in plan.spans() {
                    let low = boundary_rank(span.low(), ranks);
                    let high = boundary_rank(span.high(), ranks);
                    let mut expected_contributors = AxialOperandContributors::empty();
                    for cell in low..high {
                        let left_low = ranks[0].min(ranks[1]);
                        let left_high = ranks[0].max(ranks[1]);
                        let right_low = ranks[2].min(ranks[3]);
                        let right_high = ranks[2].max(ranks[3]);
                        if left_low <= cell && cell < left_high {
                            expected_contributors.insert(AxialIntervalOperand::Left);
                        }
                        if right_low <= cell && cell < right_high {
                            expected_contributors.insert(AxialIntervalOperand::Right);
                        }
                    }
                    assert_eq!(
                        span.side_operands(),
                        expected_contributors,
                        "ranks={ranks:?} operation={operation:?}"
                    );
                }
            }
        }
        assert_eq!(weak_orders, 75);
    }

    #[test]
    fn malformed_pair_classifications_are_rejected() {
        let valid = comparisons([0, 1, 2, 3]);

        let mut self_comparison = valid;
        self_comparison[0] = AxialEndpointComparison::new(LEFT_START, LEFT_START, Ordering::Equal);
        assert_eq!(
            CertifiedAxialEndpointPreorder::from_comparisons(self_comparison),
            Err(AxialEndpointPreorderError::SelfComparison)
        );

        let mut duplicate = valid;
        duplicate[5] = reversed(valid[0]);
        assert_eq!(
            CertifiedAxialEndpointPreorder::from_comparisons(duplicate),
            Err(AxialEndpointPreorderError::DuplicateComparison)
        );

        let cycle = [
            AxialEndpointComparison::new(LEFT_START, LEFT_END, Ordering::Less),
            AxialEndpointComparison::new(LEFT_START, RIGHT_START, Ordering::Greater),
            AxialEndpointComparison::new(LEFT_START, RIGHT_END, Ordering::Less),
            AxialEndpointComparison::new(LEFT_END, RIGHT_START, Ordering::Less),
            AxialEndpointComparison::new(LEFT_END, RIGHT_END, Ordering::Less),
            AxialEndpointComparison::new(RIGHT_START, RIGHT_END, Ordering::Less),
        ];
        assert_eq!(
            CertifiedAxialEndpointPreorder::from_comparisons(cycle),
            Err(AxialEndpointPreorderError::NonTransitive)
        );

        let inconsistent_equality = [
            AxialEndpointComparison::new(LEFT_START, LEFT_END, Ordering::Equal),
            AxialEndpointComparison::new(LEFT_START, RIGHT_START, Ordering::Greater),
            AxialEndpointComparison::new(LEFT_START, RIGHT_END, Ordering::Less),
            AxialEndpointComparison::new(LEFT_END, RIGHT_START, Ordering::Less),
            AxialEndpointComparison::new(LEFT_END, RIGHT_END, Ordering::Less),
            AxialEndpointComparison::new(RIGHT_START, RIGHT_END, Ordering::Less),
        ];
        assert_eq!(
            CertifiedAxialEndpointPreorder::from_comparisons(inconsistent_equality),
            Err(AxialEndpointPreorderError::NonTransitive)
        );
    }
}
