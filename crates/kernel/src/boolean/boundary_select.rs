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

/// Complete relation of one open fragment to the other operand.
///
/// `Interior` and `Exterior` are the only decisive classes. The remaining
/// variants preserve why the classifier could not supply an open-set
/// relation; selection never converts them into a guessed truth value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BoundaryFragmentClassification {
    Interior,
    Exterior,
    Boundary,
    Indeterminate { reason: &'static str },
    Unsupported { reason: &'static str },
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

fn selected_orientation(
    operation: RegularizedBooleanOperation,
    operand: OperandSide,
    classification: BoundaryFragmentClassification,
) -> Result<Option<SelectedOrientation>, BoundarySelectionError> {
    let other_inside = match classification {
        BoundaryFragmentClassification::Interior => true,
        BoundaryFragmentClassification::Exterior => false,
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
            result_contains(operation, true, other_inside),
            result_contains(operation, false, other_inside),
        ),
        OperandSide::Right => (
            result_contains(operation, other_inside, true),
            result_contains(operation, other_inside, false),
        ),
    };
    Ok(match (source_interior, source_exterior) {
        (true, false) => Some(SelectedOrientation::Preserved),
        (false, true) => Some(SelectedOrientation::Reversed),
        _ => None,
    })
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
    let mut canonical = BTreeMap::new();
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
        };
        if canonical.insert((operand, key), fragment).is_some() {
            return Err(BoundarySelectionError::DuplicateFragmentKey);
        }
    }

    let mut selected = Vec::new();
    for ((operand, key), fragment) in canonical {
        let Some(orientation) =
            selected_orientation(operation, fragment.operand, fragment.classification)?
        else {
            continue;
        };
        selected.push(SelectedBoundaryFragment {
            key,
            operand,
            fragment: fragment.fragment,
            orientation,
        });
    }
    Ok(selected)
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
