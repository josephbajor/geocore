//! Exact truth selection for classified Boolean boundary fragments.
//!
//! This module deliberately knows nothing about a fragment's carrier or
//! topology representation. Planar polygons, complete-period rings, and
//! future bounded curved arcs all present the same proof obligation after
//! splitting: the fragment must have one canonical identity and a certified
//! constant relation to the other operand. Selection follows only from the
//! regularized set truth on the source-interior and source-exterior sides.
//! Boundary contact, incomplete classification, and unsupported
//! classification are propagated before any payload can reach realization.

use std::collections::BTreeMap;

/// One regularized Boolean set operation over ordered operands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RegularizedBooleanOperation {
    Unite,
    Intersect,
    Subtract,
}

/// Operand that owns a source-boundary fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum OperandSide {
    Left,
    Right,
}

/// Orientation of a retained fragment relative to its source boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum SelectedOrientation {
    Preserved,
    Reversed,
}

/// Relative orientation of two source boundaries certified to be coincident.
///
/// `Aligned` means their source-boundary orientations describe the same
/// physical direction. `Opposed` means one source-boundary orientation must
/// be reversed to describe the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CoincidentSourceOrientation {
    Aligned,
    Opposed,
}

/// Proof-fed correspondence between one Left and one Right boundary cell.
///
/// The operand qualification is encoded by the two distinct fields: callers
/// cannot accidentally present a Left/Left or Right/Right pair. Keys remain
/// local to their owning operands, exactly as they are for classified
/// fragments. This value certifies correspondence only; the selector still
/// validates membership, contact occupancy, and physical orientation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CoincidentBoundaryPairEvidence<K> {
    left_key: K,
    right_key: K,
    source_orientation: CoincidentSourceOrientation,
}

impl<K> CoincidentBoundaryPairEvidence<K> {
    pub(crate) const fn new(
        left_key: K,
        right_key: K,
        source_orientation: CoincidentSourceOrientation,
    ) -> Self {
        Self {
            left_key,
            right_key,
            source_orientation,
        }
    }
}

/// Complete relation of one open fragment to the other operand.
///
/// `Interior`, `Exterior`, and `TwoSided` are decisive classes. The remaining
/// variants preserve why the classifier could not supply an open-set
/// relation; selection never converts them into a guessed truth value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BoundaryFragmentClassification {
    Interior,
    Exterior,
    /// Certified occupancy of the other operand on both source sides.
    ///
    /// This represents a coincident boundary cell without guessing a single
    /// open-set class. The first value is adjacent to the source interior;
    /// the second is adjacent to the source exterior.
    TwoSided {
        other_on_source_interior: bool,
        other_on_source_exterior: bool,
    },
    Boundary,
    Indeterminate {
        reason: &'static str,
    },
    Unsupported {
        reason: &'static str,
    },
}

/// A representation-independent boundary fragment awaiting truth selection.
///
/// `K` is a caller-owned canonical identity within the source operand. `F`
/// is opaque proof-bearing fragment data; the selector neither inspects nor
/// mutates it. The selector combines `operand` and `key` for global identity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ClassifiedBoundaryFragment<K, F> {
    key: K,
    operand: OperandSide,
    fragment: F,
    classification: BoundaryFragmentClassification,
}

struct CanonicalBoundaryFragment<F> {
    operand: OperandSide,
    fragment: F,
    classification: BoundaryFragmentClassification,
    pair_disposition: Option<CoincidentPairDisposition>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CoincidentPairDisposition {
    Retain(SelectedOrientation),
    Omit,
}

impl<K, F> ClassifiedBoundaryFragment<K, F> {
    pub(crate) const fn new(
        key: K,
        operand: OperandSide,
        fragment: F,
        classification: BoundaryFragmentClassification,
    ) -> Self {
        Self {
            key,
            operand,
            fragment,
            classification,
        }
    }

    /// Canonical identity within the owning source operand.
    pub(crate) const fn key(&self) -> &K {
        &self.key
    }

    /// Source operand that owns this boundary fragment.
    pub(crate) const fn operand(&self) -> OperandSide {
        self.operand
    }
}

/// One representation-independent fragment retained by the set truth.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SelectedBoundaryFragment<K, F> {
    key: K,
    operand: OperandSide,
    fragment: F,
    orientation: SelectedOrientation,
}

impl<K, F> SelectedBoundaryFragment<K, F> {
    /// Canonical identity within the owning source operand.
    pub(crate) const fn key(&self) -> &K {
        &self.key
    }

    /// Source operand that owns this retained boundary fragment.
    pub(crate) const fn operand(&self) -> OperandSide {
        self.operand
    }

    /// Retained orientation relative to the source boundary.
    pub(crate) const fn orientation(&self) -> SelectedOrientation {
        self.orientation
    }

    /// Representation payload retained for topology recognition.
    pub(crate) const fn fragment(&self) -> &F {
        &self.fragment
    }

    pub(crate) fn into_parts(self) -> (K, OperandSide, F, SelectedOrientation) {
        (self.key, self.operand, self.fragment, self.orientation)
    }
}

/// Honest refusal from representation-independent truth selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BoundarySelectionError {
    DuplicateFragmentKey,
    BoundaryContact,
    Indeterminate { reason: &'static str },
    Unsupported { reason: &'static str },
}

const MISSING_LEFT_COINCIDENT_PAIR_MEMBER: &str =
    "coincident boundary pair is missing its Left member";
const MISSING_RIGHT_COINCIDENT_PAIR_MEMBER: &str =
    "coincident boundary pair is missing its Right member";
const REUSED_LEFT_COINCIDENT_PAIR_MEMBER: &str =
    "Left boundary fragment belongs to more than one coincident pair";
const REUSED_RIGHT_COINCIDENT_PAIR_MEMBER: &str =
    "Right boundary fragment belongs to more than one coincident pair";
const MALFORMED_LEFT_COINCIDENT_PAIR_MEMBER: &str =
    "Left coincident pair member has incompatible two-sided occupancy";
const MALFORMED_RIGHT_COINCIDENT_PAIR_MEMBER: &str =
    "Right coincident pair member has incompatible two-sided occupancy";
const COINCIDENT_PAIR_TRUTH_MISMATCH: &str =
    "coincident boundary pair has incompatible regularized truth";
const COINCIDENT_PAIR_ORIENTATION_MISMATCH: &str =
    "coincident boundary pair has incompatible final physical orientation";

const fn result_contains(
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BoundaryTruthSelection {
    orientation: Option<SelectedOrientation>,
    is_contact: bool,
}

fn boundary_truth_selection(
    operation: RegularizedBooleanOperation,
    operand: OperandSide,
    classification: BoundaryFragmentClassification,
) -> Result<BoundaryTruthSelection, BoundarySelectionError> {
    let (other_on_source_interior, other_on_source_exterior, is_contact) = match classification {
        BoundaryFragmentClassification::Interior => (true, true, false),
        BoundaryFragmentClassification::Exterior => (false, false, false),
        BoundaryFragmentClassification::TwoSided {
            other_on_source_interior,
            other_on_source_exterior,
        } => (
            other_on_source_interior,
            other_on_source_exterior,
            other_on_source_interior != other_on_source_exterior,
        ),
        BoundaryFragmentClassification::Boundary => {
            return Err(BoundarySelectionError::BoundaryContact);
        }
        BoundaryFragmentClassification::Indeterminate { reason } => {
            return Err(BoundarySelectionError::Indeterminate { reason });
        }
        BoundaryFragmentClassification::Unsupported { reason } => {
            return Err(BoundarySelectionError::Unsupported { reason });
        }
    };

    let (source_interior, source_exterior) = match operand {
        OperandSide::Left => (
            result_contains(operation, true, other_on_source_interior),
            result_contains(operation, false, other_on_source_exterior),
        ),
        OperandSide::Right => (
            result_contains(operation, other_on_source_interior, true),
            result_contains(operation, other_on_source_exterior, false),
        ),
    };
    let orientation = match (source_interior, source_exterior) {
        (true, false) => Some(SelectedOrientation::Preserved),
        (false, true) => Some(SelectedOrientation::Reversed),
        _ => None,
    };
    Ok(BoundaryTruthSelection {
        orientation,
        is_contact,
    })
}

fn selected_orientation(
    operation: RegularizedBooleanOperation,
    operand: OperandSide,
    classification: BoundaryFragmentClassification,
) -> Result<Option<SelectedOrientation>, BoundarySelectionError> {
    let selection = boundary_truth_selection(operation, operand, classification)?;
    if selection.is_contact && selection.orientation.is_some() {
        // A retained coincident boundary needs pairwise coalescing before one
        // canonical source can own it. Until that proof exists, fail closed.
        return Err(BoundarySelectionError::BoundaryContact);
    }
    Ok(selection.orientation)
}

struct CanonicalBoundaryFragments<K, F> {
    left: BTreeMap<K, CanonicalBoundaryFragment<F>>,
    right: BTreeMap<K, CanonicalBoundaryFragment<F>>,
}

fn canonicalize_boundary_fragments<K: Ord, F>(
    fragments: impl IntoIterator<Item = ClassifiedBoundaryFragment<K, F>>,
) -> Result<CanonicalBoundaryFragments<K, F>, BoundarySelectionError> {
    let mut canonical = CanonicalBoundaryFragments {
        left: BTreeMap::new(),
        right: BTreeMap::new(),
    };
    for fragment in fragments {
        let ClassifiedBoundaryFragment {
            key,
            operand,
            fragment,
            classification,
        } = fragment;
        let fragment = CanonicalBoundaryFragment {
            operand,
            fragment,
            classification,
            pair_disposition: None,
        };
        let operand_fragments = match operand {
            OperandSide::Left => &mut canonical.left,
            OperandSide::Right => &mut canonical.right,
        };
        if operand_fragments.insert(key, fragment).is_some() {
            return Err(BoundarySelectionError::DuplicateFragmentKey);
        }
    }
    Ok(canonical)
}

const fn expected_coincident_occupancy(
    source_orientation: CoincidentSourceOrientation,
) -> (bool, bool) {
    match source_orientation {
        CoincidentSourceOrientation::Aligned => (true, false),
        CoincidentSourceOrientation::Opposed => (false, true),
    }
}

const fn has_coincident_occupancy(
    classification: BoundaryFragmentClassification,
    expected: (bool, bool),
) -> bool {
    matches!(
        classification,
        BoundaryFragmentClassification::TwoSided {
            other_on_source_interior,
            other_on_source_exterior,
        } if other_on_source_interior == expected.0
            && other_on_source_exterior == expected.1
    )
}

const fn final_orientations_are_compatible(
    source_orientation: CoincidentSourceOrientation,
    left: SelectedOrientation,
    right: SelectedOrientation,
) -> bool {
    matches!(
        (source_orientation, left, right),
        (
            CoincidentSourceOrientation::Aligned,
            SelectedOrientation::Preserved,
            SelectedOrientation::Preserved,
        ) | (
            CoincidentSourceOrientation::Aligned,
            SelectedOrientation::Reversed,
            SelectedOrientation::Reversed,
        ) | (
            CoincidentSourceOrientation::Opposed,
            SelectedOrientation::Preserved,
            SelectedOrientation::Reversed,
        ) | (
            CoincidentSourceOrientation::Opposed,
            SelectedOrientation::Reversed,
            SelectedOrientation::Preserved,
        )
    )
}

fn coincident_pair_dispositions(
    operation: RegularizedBooleanOperation,
    source_orientation: CoincidentSourceOrientation,
    left_classification: BoundaryFragmentClassification,
    right_classification: BoundaryFragmentClassification,
) -> Result<(CoincidentPairDisposition, CoincidentPairDisposition), BoundarySelectionError> {
    let expected = expected_coincident_occupancy(source_orientation);
    if !has_coincident_occupancy(left_classification, expected) {
        return Err(BoundarySelectionError::Unsupported {
            reason: MALFORMED_LEFT_COINCIDENT_PAIR_MEMBER,
        });
    }
    if !has_coincident_occupancy(right_classification, expected) {
        return Err(BoundarySelectionError::Unsupported {
            reason: MALFORMED_RIGHT_COINCIDENT_PAIR_MEMBER,
        });
    }

    let left = boundary_truth_selection(operation, OperandSide::Left, left_classification)?;
    let right = boundary_truth_selection(operation, OperandSide::Right, right_classification)?;
    match (left.orientation, right.orientation) {
        (None, None) => Ok((
            CoincidentPairDisposition::Omit,
            CoincidentPairDisposition::Omit,
        )),
        (Some(left), Some(right)) => {
            if !final_orientations_are_compatible(source_orientation, left, right) {
                return Err(BoundarySelectionError::Unsupported {
                    reason: COINCIDENT_PAIR_ORIENTATION_MISMATCH,
                });
            }
            Ok((
                CoincidentPairDisposition::Retain(left),
                CoincidentPairDisposition::Omit,
            ))
        }
        _ => Err(BoundarySelectionError::Unsupported {
            reason: COINCIDENT_PAIR_TRUTH_MISMATCH,
        }),
    }
}

fn apply_coincident_pair_evidence<K: Ord, F>(
    operation: RegularizedBooleanOperation,
    canonical: &mut CanonicalBoundaryFragments<K, F>,
    pairs: impl IntoIterator<Item = CoincidentBoundaryPairEvidence<K>>,
) -> Result<(), BoundarySelectionError> {
    for pair in pairs {
        let Some(left) = canonical.left.get(&pair.left_key) else {
            return Err(BoundarySelectionError::Unsupported {
                reason: MISSING_LEFT_COINCIDENT_PAIR_MEMBER,
            });
        };
        let left_classification = left.classification;
        if left.pair_disposition.is_some() {
            return Err(BoundarySelectionError::Unsupported {
                reason: REUSED_LEFT_COINCIDENT_PAIR_MEMBER,
            });
        }

        let Some(right) = canonical.right.get(&pair.right_key) else {
            return Err(BoundarySelectionError::Unsupported {
                reason: MISSING_RIGHT_COINCIDENT_PAIR_MEMBER,
            });
        };
        let right_classification = right.classification;
        if right.pair_disposition.is_some() {
            return Err(BoundarySelectionError::Unsupported {
                reason: REUSED_RIGHT_COINCIDENT_PAIR_MEMBER,
            });
        }

        let (left_disposition, right_disposition) = coincident_pair_dispositions(
            operation,
            pair.source_orientation,
            left_classification,
            right_classification,
        )?;
        canonical
            .left
            .get_mut(&pair.left_key)
            .expect("pair membership was validated")
            .pair_disposition = Some(left_disposition);
        canonical
            .right
            .get_mut(&pair.right_key)
            .expect("pair membership was validated")
            .pair_disposition = Some(right_disposition);
    }
    Ok(())
}

fn select_canonical_boundary_fragments<K: Ord, F>(
    operation: RegularizedBooleanOperation,
    canonical: CanonicalBoundaryFragments<K, F>,
) -> Result<Vec<SelectedBoundaryFragment<K, F>>, BoundarySelectionError> {
    let mut selected = Vec::new();
    for (operand, fragments) in [
        (OperandSide::Left, canonical.left),
        (OperandSide::Right, canonical.right),
    ] {
        for (key, fragment) in fragments {
            let orientation = match fragment.pair_disposition {
                Some(CoincidentPairDisposition::Retain(orientation)) => Some(orientation),
                Some(CoincidentPairDisposition::Omit) => None,
                None => selected_orientation(operation, fragment.operand, fragment.classification)?,
            };
            let Some(orientation) = orientation else {
                continue;
            };
            selected.push(SelectedBoundaryFragment {
                key,
                operand,
                fragment: fragment.fragment,
                orientation,
            });
        }
    }
    Ok(selected)
}

/// Canonicalize, truth-select, and orient classified boundary fragments.
///
/// Duplicate identities are rejected before truth filtering, including when
/// both copies would be omitted. Output order is canonical operand/key order
/// and is independent of classifier discovery order. The payload remains
/// unchanged; its adapter must apply the returned orientation without
/// discarding its proof data.
pub(crate) fn select_boundary_fragments<K: Ord, F>(
    operation: RegularizedBooleanOperation,
    fragments: impl IntoIterator<Item = ClassifiedBoundaryFragment<K, F>>,
) -> Result<Vec<SelectedBoundaryFragment<K, F>>, BoundarySelectionError> {
    let canonical = canonicalize_boundary_fragments(fragments)?;
    select_canonical_boundary_fragments(operation, canonical)
}

/// Canonicalize, truth-select, and coalesce proof-paired coincident cells.
///
/// Pair evidence must name one existing Left and one existing Right fragment,
/// and each fragment may occur in at most one pair. Both classifications must
/// be decisive `TwoSided` contacts matching the certified relative source
/// orientation. A pair is retained only when both source views select the
/// same physical boundary orientation; the already-canonical Left owner wins.
/// An unpaired contact retains the fail-closed behavior of
/// [`select_boundary_fragments`].
pub(crate) fn select_boundary_fragments_with_coincident_pairs<K: Ord, F>(
    operation: RegularizedBooleanOperation,
    fragments: impl IntoIterator<Item = ClassifiedBoundaryFragment<K, F>>,
    pairs: impl IntoIterator<Item = CoincidentBoundaryPairEvidence<K>>,
) -> Result<Vec<SelectedBoundaryFragment<K, F>>, BoundarySelectionError> {
    let mut canonical = canonicalize_boundary_fragments(fragments)?;
    apply_coincident_pair_evidence(operation, &mut canonical, pairs)?;
    select_canonical_boundary_fragments(operation, canonical)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALIGNED_CONTACT: BoundaryFragmentClassification =
        BoundaryFragmentClassification::TwoSided {
            other_on_source_interior: true,
            other_on_source_exterior: false,
        };
    const OPPOSED_CONTACT: BoundaryFragmentClassification =
        BoundaryFragmentClassification::TwoSided {
            other_on_source_interior: false,
            other_on_source_exterior: true,
        };

    fn contact_pair(
        classification: BoundaryFragmentClassification,
    ) -> [ClassifiedBoundaryFragment<u8, &'static str>; 2] {
        [
            ClassifiedBoundaryFragment::new(1, OperandSide::Left, "left", classification),
            ClassifiedBoundaryFragment::new(2, OperandSide::Right, "right", classification),
        ]
    }

    fn pair_evidence(
        source_orientation: CoincidentSourceOrientation,
    ) -> [CoincidentBoundaryPairEvidence<u8>; 1] {
        [CoincidentBoundaryPairEvidence::new(
            1,
            2,
            source_orientation,
        )]
    }

    #[test]
    fn all_truth_tables_follow_boundary_occupancy() {
        use BoundaryFragmentClassification::{Exterior, Interior};
        use OperandSide::{Left, Right};
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};
        use SelectedOrientation::{Preserved, Reversed};

        let cases = [
            (Unite, Left, Exterior, Some(Preserved)),
            (Unite, Left, Interior, None),
            (Unite, Right, Exterior, Some(Preserved)),
            (Unite, Right, Interior, None),
            (Intersect, Left, Exterior, None),
            (Intersect, Left, Interior, Some(Preserved)),
            (Intersect, Right, Exterior, None),
            (Intersect, Right, Interior, Some(Preserved)),
            (Subtract, Left, Exterior, Some(Preserved)),
            (Subtract, Left, Interior, None),
            (Subtract, Right, Exterior, None),
            (Subtract, Right, Interior, Some(Reversed)),
        ];
        for (operation, operand, classification, expected) in cases {
            assert_eq!(
                selected_orientation(operation, operand, classification),
                Ok(expected)
            );
        }
    }

    #[test]
    fn opposed_contact_is_omitted_or_refused_from_generic_side_truth() {
        use BoundaryFragmentClassification::{Exterior, Interior};
        use OperandSide::{Left, Right};
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        let contact = BoundaryFragmentClassification::TwoSided {
            other_on_source_interior: false,
            other_on_source_exterior: true,
        };
        for operand in [Left, Right] {
            assert_eq!(selected_orientation(Unite, operand, contact), Ok(None));
            assert_eq!(selected_orientation(Intersect, operand, contact), Ok(None));
            assert_eq!(
                selected_orientation(Subtract, operand, contact),
                Err(BoundarySelectionError::BoundaryContact)
            );
        }

        for operation in [Unite, Intersect, Subtract] {
            for operand in [Left, Right] {
                for (other_inside, expected) in [(false, Exterior), (true, Interior)] {
                    let equal = BoundaryFragmentClassification::TwoSided {
                        other_on_source_interior: other_inside,
                        other_on_source_exterior: other_inside,
                    };
                    assert_eq!(
                        selected_orientation(operation, operand, equal),
                        selected_orientation(operation, operand, expected)
                    );
                }
            }
        }
    }

    #[test]
    fn incomplete_classes_propagate_without_truth_defaults() {
        let classes = [
            (
                BoundaryFragmentClassification::Boundary,
                BoundarySelectionError::BoundaryContact,
            ),
            (
                BoundaryFragmentClassification::Indeterminate {
                    reason: "uncertain classifier",
                },
                BoundarySelectionError::Indeterminate {
                    reason: "uncertain classifier",
                },
            ),
            (
                BoundaryFragmentClassification::Unsupported {
                    reason: "unsupported classifier",
                },
                BoundarySelectionError::Unsupported {
                    reason: "unsupported classifier",
                },
            ),
        ];
        for operation in [
            RegularizedBooleanOperation::Unite,
            RegularizedBooleanOperation::Intersect,
            RegularizedBooleanOperation::Subtract,
        ] {
            for operand in [OperandSide::Left, OperandSide::Right] {
                for (classification, expected) in classes {
                    assert_eq!(
                        select_boundary_fragments(
                            operation,
                            [ClassifiedBoundaryFragment::new(
                                0_u8,
                                operand,
                                (),
                                classification,
                            )]
                        ),
                        Err(expected)
                    );
                }
            }
        }
    }

    #[test]
    fn canonical_order_and_payload_are_representation_independent() {
        let selected = select_boundary_fragments(
            RegularizedBooleanOperation::Unite,
            [
                ClassifiedBoundaryFragment::new(
                    7_u8,
                    OperandSide::Right,
                    "ring",
                    BoundaryFragmentClassification::Exterior,
                ),
                ClassifiedBoundaryFragment::new(
                    2_u8,
                    OperandSide::Left,
                    "planar face",
                    BoundaryFragmentClassification::Exterior,
                ),
            ],
        )
        .unwrap()
        .into_iter()
        .map(SelectedBoundaryFragment::into_parts)
        .collect::<Vec<_>>();
        assert_eq!(
            selected,
            vec![
                (
                    2,
                    OperandSide::Left,
                    "planar face",
                    SelectedOrientation::Preserved,
                ),
                (
                    7,
                    OperandSide::Right,
                    "ring",
                    SelectedOrientation::Preserved,
                ),
            ]
        );
    }

    #[test]
    fn duplicate_identity_is_rejected_before_an_omitting_truth() {
        let fragment = ClassifiedBoundaryFragment::new(
            3_u8,
            OperandSide::Left,
            (),
            BoundaryFragmentClassification::Interior,
        );
        assert_eq!(
            select_boundary_fragments(
                RegularizedBooleanOperation::Unite,
                [fragment.clone(), fragment]
            ),
            Err(BoundarySelectionError::DuplicateFragmentKey)
        );
    }

    #[test]
    fn matching_local_keys_from_different_operands_remain_distinct() {
        let selected = select_boundary_fragments(
            RegularizedBooleanOperation::Intersect,
            [
                ClassifiedBoundaryFragment::new(
                    4_u8,
                    OperandSide::Right,
                    "right",
                    BoundaryFragmentClassification::Interior,
                ),
                ClassifiedBoundaryFragment::new(
                    4_u8,
                    OperandSide::Left,
                    "left",
                    BoundaryFragmentClassification::Interior,
                ),
            ],
        )
        .unwrap()
        .into_iter()
        .map(SelectedBoundaryFragment::into_parts)
        .collect::<Vec<_>>();
        assert_eq!(
            selected,
            vec![
                (4, OperandSide::Left, "left", SelectedOrientation::Preserved,),
                (
                    4,
                    OperandSide::Right,
                    "right",
                    SelectedOrientation::Preserved,
                ),
            ]
        );
    }

    #[test]
    fn aligned_pairs_coalesce_unite_and_intersect_but_subtract_omits_them() {
        use CoincidentSourceOrientation::Aligned;
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        for operation in [Unite, Intersect] {
            let selected = select_boundary_fragments_with_coincident_pairs(
                operation,
                contact_pair(ALIGNED_CONTACT),
                pair_evidence(Aligned),
            )
            .unwrap()
            .into_iter()
            .map(SelectedBoundaryFragment::into_parts)
            .collect::<Vec<_>>();
            assert_eq!(
                selected,
                vec![(1, OperandSide::Left, "left", SelectedOrientation::Preserved,)]
            );
        }

        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                Subtract,
                contact_pair(ALIGNED_CONTACT),
                pair_evidence(Aligned),
            ),
            Ok(Vec::new())
        );
    }

    #[test]
    fn opposed_pairs_omit_unite_and_intersect_but_subtract_coalesces() {
        use CoincidentSourceOrientation::Opposed;
        use RegularizedBooleanOperation::{Intersect, Subtract, Unite};

        for operation in [Unite, Intersect] {
            assert_eq!(
                select_boundary_fragments_with_coincident_pairs(
                    operation,
                    contact_pair(OPPOSED_CONTACT),
                    pair_evidence(Opposed),
                ),
                Ok(Vec::new())
            );
        }

        let selected = select_boundary_fragments_with_coincident_pairs(
            Subtract,
            contact_pair(OPPOSED_CONTACT),
            pair_evidence(Opposed),
        )
        .unwrap()
        .into_iter()
        .map(SelectedBoundaryFragment::into_parts)
        .collect::<Vec<_>>();
        assert_eq!(
            selected,
            vec![(1, OperandSide::Left, "left", SelectedOrientation::Preserved,)]
        );
    }

    #[test]
    fn retained_contact_without_pair_evidence_still_fails_closed() {
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                contact_pair(ALIGNED_CONTACT),
                [],
            ),
            Err(BoundarySelectionError::BoundaryContact)
        );
    }

    #[test]
    fn pair_evidence_requires_present_members() {
        use CoincidentSourceOrientation::Aligned;

        let right_only = [ClassifiedBoundaryFragment::new(
            2,
            OperandSide::Right,
            "right",
            ALIGNED_CONTACT,
        )];
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                right_only,
                pair_evidence(Aligned),
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: MISSING_LEFT_COINCIDENT_PAIR_MEMBER,
            })
        );

        let left_only = [ClassifiedBoundaryFragment::new(
            1,
            OperandSide::Left,
            "left",
            ALIGNED_CONTACT,
        )];
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                left_only,
                pair_evidence(Aligned),
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: MISSING_RIGHT_COINCIDENT_PAIR_MEMBER,
            })
        );
    }

    #[test]
    fn pair_membership_is_one_use() {
        use CoincidentSourceOrientation::Aligned;

        let duplicate_pair = CoincidentBoundaryPairEvidence::new(1, 2, Aligned);
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                contact_pair(ALIGNED_CONTACT),
                [duplicate_pair.clone(), duplicate_pair],
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: REUSED_LEFT_COINCIDENT_PAIR_MEMBER,
            })
        );

        let fragments = [
            ClassifiedBoundaryFragment::new(1, OperandSide::Left, "left", ALIGNED_CONTACT),
            ClassifiedBoundaryFragment::new(2, OperandSide::Right, "right", ALIGNED_CONTACT),
            ClassifiedBoundaryFragment::new(3, OperandSide::Right, "other right", ALIGNED_CONTACT),
        ];
        let pairs = [
            CoincidentBoundaryPairEvidence::new(1, 2, Aligned),
            CoincidentBoundaryPairEvidence::new(1, 3, Aligned),
        ];
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                fragments,
                pairs,
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: REUSED_LEFT_COINCIDENT_PAIR_MEMBER,
            })
        );

        let fragments = [
            ClassifiedBoundaryFragment::new(1, OperandSide::Left, "left", ALIGNED_CONTACT),
            ClassifiedBoundaryFragment::new(3, OperandSide::Left, "other left", ALIGNED_CONTACT),
            ClassifiedBoundaryFragment::new(2, OperandSide::Right, "right", ALIGNED_CONTACT),
        ];
        let pairs = [
            CoincidentBoundaryPairEvidence::new(1, 2, Aligned),
            CoincidentBoundaryPairEvidence::new(3, 2, Aligned),
        ];
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                fragments,
                pairs,
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: REUSED_RIGHT_COINCIDENT_PAIR_MEMBER,
            })
        );
    }

    #[test]
    fn malformed_pair_evidence_never_coalesces_contact() {
        use CoincidentSourceOrientation::{Aligned, Opposed};

        let malformed_left = [
            ClassifiedBoundaryFragment::new(
                1,
                OperandSide::Left,
                "left",
                BoundaryFragmentClassification::Exterior,
            ),
            ClassifiedBoundaryFragment::new(2, OperandSide::Right, "right", ALIGNED_CONTACT),
        ];
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                malformed_left,
                pair_evidence(Aligned),
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: MALFORMED_LEFT_COINCIDENT_PAIR_MEMBER,
            })
        );

        let malformed_right = [
            ClassifiedBoundaryFragment::new(1, OperandSide::Left, "left", ALIGNED_CONTACT),
            ClassifiedBoundaryFragment::new(
                2,
                OperandSide::Right,
                "right",
                BoundaryFragmentClassification::TwoSided {
                    other_on_source_interior: true,
                    other_on_source_exterior: true,
                },
            ),
        ];
        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                malformed_right,
                pair_evidence(Aligned),
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: MALFORMED_RIGHT_COINCIDENT_PAIR_MEMBER,
            })
        );

        assert_eq!(
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                contact_pair(ALIGNED_CONTACT),
                pair_evidence(Opposed),
            ),
            Err(BoundarySelectionError::Unsupported {
                reason: MALFORMED_LEFT_COINCIDENT_PAIR_MEMBER,
            })
        );
    }

    #[test]
    fn physical_orientation_compatibility_is_explicit() {
        use CoincidentSourceOrientation::{Aligned, Opposed};
        use SelectedOrientation::{Preserved, Reversed};

        assert!(final_orientations_are_compatible(
            Aligned, Preserved, Preserved
        ));
        assert!(final_orientations_are_compatible(
            Aligned, Reversed, Reversed
        ));
        assert!(!final_orientations_are_compatible(
            Aligned, Preserved, Reversed
        ));
        assert!(final_orientations_are_compatible(
            Opposed, Preserved, Reversed
        ));
        assert!(final_orientations_are_compatible(
            Opposed, Reversed, Preserved
        ));
        assert!(!final_orientations_are_compatible(
            Opposed, Preserved, Preserved
        ));
    }

    #[test]
    fn pair_and_fragment_discovery_order_do_not_change_canonical_ownership() {
        use CoincidentSourceOrientation::Aligned;

        let run = |reverse: bool| {
            let mut fragments = vec![
                ClassifiedBoundaryFragment::new(
                    30,
                    OperandSide::Right,
                    "right 30",
                    ALIGNED_CONTACT,
                ),
                ClassifiedBoundaryFragment::new(
                    10,
                    OperandSide::Right,
                    "right 10",
                    ALIGNED_CONTACT,
                ),
                ClassifiedBoundaryFragment::new(3, OperandSide::Left, "left 3", ALIGNED_CONTACT),
                ClassifiedBoundaryFragment::new(1, OperandSide::Left, "left 1", ALIGNED_CONTACT),
            ];
            let mut pairs = vec![
                CoincidentBoundaryPairEvidence::new(3, 30, Aligned),
                CoincidentBoundaryPairEvidence::new(1, 10, Aligned),
            ];
            if reverse {
                fragments.reverse();
                pairs.reverse();
            }
            select_boundary_fragments_with_coincident_pairs(
                RegularizedBooleanOperation::Intersect,
                fragments,
                pairs,
            )
            .unwrap()
            .into_iter()
            .map(SelectedBoundaryFragment::into_parts)
            .collect::<Vec<_>>()
        };

        let expected = vec![
            (
                1,
                OperandSide::Left,
                "left 1",
                SelectedOrientation::Preserved,
            ),
            (
                3,
                OperandSide::Left,
                "left 3",
                SelectedOrientation::Preserved,
            ),
        ];
        assert_eq!(run(false), expected);
        assert_eq!(run(true), expected);
    }
}
