//! Exact identity decisions for roots of bounded low-degree polynomials.
//!
//! Each input bracket is re-certified as owning exactly one distinct root.
//! Disjoint brackets prove separation directly. Overlapping brackets are
//! decided by exact pseudo-remainder GCDs, never by numeric proximity.

use super::bounded_polynomial::{
    ExactPolynomial, RootBracket, RootIsolation, RootIsolationFailure, common_root_polynomial,
};

/// Exact relation between the unique roots owned by two certified brackets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ExactRootRelation {
    /// Both brackets own the same mathematical root.
    Same,
    /// The brackets own different mathematical roots.
    Distinct,
}

/// Stable fail-closed causes for an exact root-relation query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ExactRootRelationFailure {
    /// A bracket was non-finite or reversed.
    InvalidBracket,
    /// A bracket did not contain exactly one distinct root of its polynomial.
    BracketDoesNotIsolateOneRoot,
    /// Exact construction or isolation left its fixed safe envelope.
    ExactArithmetic(RootIsolationFailure),
    /// Two independently exact proof paths produced incompatible decisions.
    InconsistentExactProof,
}

/// Decide whether two certified low-degree polynomial roots are identical.
///
/// Both polynomials must use the same exact parameter coordinate; callers
/// that use projective charts must convert them to a common chart first.
/// Both closed brackets are independently re-isolated before any result is
/// published. Pseudo-remainder paths are evaluated in both argument orders;
/// one sound proof may discharge an arithmetic refusal in the other order,
/// while conflicting proofs fail closed.
pub(crate) fn classify_exact_root_relation(
    first: &ExactPolynomial,
    first_bracket: RootBracket,
    second: &ExactPolynomial,
    second_bracket: RootBracket,
) -> Result<ExactRootRelation, ExactRootRelationFailure> {
    reconcile_validations(
        validate_root_owner(first, first_bracket),
        validate_root_owner(second, second_bracket),
    )?;

    if first_bracket.hi < second_bracket.lo || second_bracket.hi < first_bracket.lo {
        return Ok(ExactRootRelation::Distinct);
    }
    let overlap_lo = first_bracket.lo.max(second_bracket.lo);
    let overlap_hi = first_bracket.hi.min(second_bracket.hi);
    if overlap_lo == overlap_hi {
        return classify_touching_boundary(first, second, overlap_lo);
    }

    reconcile_proofs(
        relation_from_common_root(first, second, overlap_lo, overlap_hi),
        relation_from_common_root(second, first, overlap_lo, overlap_hi),
    )
}

fn validate_root_owner(
    polynomial: &ExactPolynomial,
    bracket: RootBracket,
) -> Result<(), ExactRootRelationFailure> {
    if !bracket.lo.is_finite() || !bracket.hi.is_finite() || bracket.lo > bracket.hi {
        return Err(ExactRootRelationFailure::InvalidBracket);
    }
    match polynomial.isolate(bracket.lo, bracket.hi) {
        RootIsolation::Complete(roots) if roots.len() == 1 => Ok(()),
        RootIsolation::Complete(_) => Err(ExactRootRelationFailure::BracketDoesNotIsolateOneRoot),
        RootIsolation::Ambiguous(failure) => {
            Err(ExactRootRelationFailure::ExactArithmetic(failure))
        }
    }
}

fn classify_touching_boundary(
    first: &ExactPolynomial,
    second: &ExactPolynomial,
    parameter: f64,
) -> Result<ExactRootRelation, ExactRootRelationFailure> {
    let first_zero = first
        .evaluate(parameter)
        .map(|value| value.is_zero())
        .map_err(ExactRootRelationFailure::ExactArithmetic);
    let second_zero = second
        .evaluate(parameter)
        .map(|value| value.is_zero())
        .map_err(ExactRootRelationFailure::ExactArithmetic);
    match (first_zero, second_zero) {
        (Ok(true), Ok(true)) => Ok(ExactRootRelation::Same),
        (Ok(_), Ok(_)) => Ok(ExactRootRelation::Distinct),
        (Err(first), Err(second)) => Err(preferred_failure(first, second)),
        (Err(failure), Ok(_)) | (Ok(_), Err(failure)) => Err(failure),
    }
}

fn relation_from_common_root(
    first: &ExactPolynomial,
    second: &ExactPolynomial,
    overlap_lo: f64,
    overlap_hi: f64,
) -> Result<ExactRootRelation, ExactRootRelationFailure> {
    let Some(common) =
        common_root_polynomial(first, second).map_err(ExactRootRelationFailure::ExactArithmetic)?
    else {
        return Ok(ExactRootRelation::Distinct);
    };
    match common.isolate(overlap_lo, overlap_hi) {
        RootIsolation::Complete(roots) if roots.is_empty() => Ok(ExactRootRelation::Distinct),
        RootIsolation::Complete(roots) if roots.len() == 1 => Ok(ExactRootRelation::Same),
        RootIsolation::Complete(_) => Err(ExactRootRelationFailure::InconsistentExactProof),
        RootIsolation::Ambiguous(failure) => {
            Err(ExactRootRelationFailure::ExactArithmetic(failure))
        }
    }
}

fn reconcile_validations(
    first: Result<(), ExactRootRelationFailure>,
    second: Result<(), ExactRootRelationFailure>,
) -> Result<(), ExactRootRelationFailure> {
    match (first, second) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(first), Err(second)) => Err(preferred_failure(first, second)),
        (Err(failure), Ok(())) | (Ok(()), Err(failure)) => Err(failure),
    }
}

fn reconcile_proofs(
    first: Result<ExactRootRelation, ExactRootRelationFailure>,
    second: Result<ExactRootRelation, ExactRootRelationFailure>,
) -> Result<ExactRootRelation, ExactRootRelationFailure> {
    match (first, second) {
        (Ok(first), Ok(second)) if first == second => Ok(first),
        (Ok(_), Ok(_)) => Err(ExactRootRelationFailure::InconsistentExactProof),
        (Ok(relation), Err(_)) | (Err(_), Ok(relation)) => Ok(relation),
        (Err(first), Err(second)) => Err(preferred_failure(first, second)),
    }
}

fn preferred_failure(
    first: ExactRootRelationFailure,
    second: ExactRootRelationFailure,
) -> ExactRootRelationFailure {
    if failure_key(first) <= failure_key(second) {
        first
    } else {
        second
    }
}

fn failure_key(failure: ExactRootRelationFailure) -> (u8, u8) {
    match failure {
        ExactRootRelationFailure::InvalidBracket => (0, 0),
        ExactRootRelationFailure::BracketDoesNotIsolateOneRoot => (1, 0),
        ExactRootRelationFailure::ExactArithmetic(failure) => (2, arithmetic_key(failure)),
        ExactRootRelationFailure::InconsistentExactProof => (3, 0),
    }
}

fn arithmetic_key(failure: RootIsolationFailure) -> u8 {
    match failure {
        RootIsolationFailure::NonFiniteInput => 0,
        RootIsolationFailure::DegreeLimit => 1,
        RootIsolationFailure::ZeroPolynomial => 2,
        RootIsolationFailure::UnsafeArithmeticEnvelope => 3,
        RootIsolationFailure::ExpansionLimit => 4,
        RootIsolationFailure::InvalidRange => 5,
        RootIsolationFailure::SturmChainLimit => 6,
        RootIsolationFailure::IsolationLimit => 7,
        RootIsolationFailure::ParameterResolution => 8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::bounded_polynomial::ExactScalar;

    fn scalar(value: f64) -> ExactScalar {
        ExactScalar::from_f64(value).unwrap()
    }

    fn polynomial(coefficients: &[f64]) -> ExactPolynomial {
        ExactPolynomial::new(coefficients.iter().copied().map(scalar).collect()).unwrap()
    }

    fn assert_symmetric_relation(
        first: &ExactPolynomial,
        first_bracket: RootBracket,
        second: &ExactPolynomial,
        second_bracket: RootBracket,
        expected: ExactRootRelation,
    ) {
        assert_eq!(
            classify_exact_root_relation(first, first_bracket, second, second_bracket),
            Ok(expected)
        );
        assert_eq!(
            classify_exact_root_relation(second, second_bracket, first, first_bracket),
            Ok(expected)
        );
    }

    fn assert_symmetric_failure(
        first: &ExactPolynomial,
        first_bracket: RootBracket,
        second: &ExactPolynomial,
        second_bracket: RootBracket,
        expected: ExactRootRelationFailure,
    ) {
        assert_eq!(
            classify_exact_root_relation(first, first_bracket, second, second_bracket),
            Err(expected)
        );
        assert_eq!(
            classify_exact_root_relation(second, second_bracket, first, first_bracket),
            Err(expected)
        );
    }

    #[test]
    fn same_irrational_root_is_proven_across_nonproportional_polynomials() {
        // P = x^2 - 2; Q = (x^2 - 2)(x - 3).
        let first = polynomial(&[-2.0, 0.0, 1.0]);
        let second = polynomial(&[6.0, -2.0, -3.0, 1.0]);
        assert_symmetric_relation(
            &first,
            RootBracket { lo: 1.0, hi: 2.0 },
            &second,
            RootBracket { lo: 1.0, hi: 2.0 },
            ExactRootRelation::Same,
        );
    }

    #[test]
    fn overlapping_brackets_around_distinct_roots_are_separated_exactly() {
        // sqrt(2) and sqrt(3) lie in different owning brackets whose numeric
        // interiors overlap.
        let first = polynomial(&[-2.0, 0.0, 1.0]);
        let second = polynomial(&[-3.0, 0.0, 1.0]);
        assert_symmetric_relation(
            &first,
            RootBracket { lo: 1.0, hi: 1.6 },
            &second,
            RootBracket { lo: 1.5, hi: 2.0 },
            ExactRootRelation::Distinct,
        );
    }

    #[test]
    fn touching_brackets_use_exact_endpoint_membership() {
        let exact_one = polynomial(&[-1.0, 1.0]);
        let also_one = polynomial(&[-2.0, 1.0, 1.0]);
        assert_symmetric_relation(
            &exact_one,
            RootBracket { lo: 1.0, hi: 1.0 },
            &also_one,
            RootBracket { lo: 1.0, hi: 1.0 },
            ExactRootRelation::Same,
        );

        let exact_two = polynomial(&[-2.0, 1.0]);
        assert_symmetric_relation(
            &exact_one,
            RootBracket { lo: 1.0, hi: 1.0 },
            &exact_two,
            RootBracket { lo: 1.0, hi: 2.0 },
            ExactRootRelation::Distinct,
        );
    }

    #[test]
    fn common_root_outside_selected_overlap_does_not_merge_distinct_roots() {
        // P = (x - 1)(x - 5), Q = (x - 2)(x - 5). The selected roots are
        // 1 and 2; the common root 5 lies outside both owning brackets.
        let first = polynomial(&[5.0, -6.0, 1.0]);
        let second = polynomial(&[10.0, -7.0, 1.0]);
        assert_symmetric_relation(
            &first,
            RootBracket { lo: 0.0, hi: 1.75 },
            &second,
            RootBracket { lo: 1.5, hi: 3.0 },
            ExactRootRelation::Distinct,
        );
    }

    #[test]
    fn equal_and_exact_scalar_multiple_polynomials_preserve_root_identity() {
        let first = polynomial(&[-2.0, 0.0, 1.0]);
        let equal = first.clone();
        let scaled = polynomial(&[-14.0, 0.0, 7.0]);
        let bracket = RootBracket { lo: 1.0, hi: 2.0 };
        assert_symmetric_relation(&first, bracket, &equal, bracket, ExactRootRelation::Same);
        assert_symmetric_relation(&first, bracket, &scaled, bracket, ExactRootRelation::Same);
    }

    #[test]
    fn malformed_and_nonisolating_brackets_fail_closed_symmetrically() {
        let first = polynomial(&[-1.0, 0.0, 1.0]);
        let second = polynomial(&[-1.0, 1.0]);
        let valid = RootBracket { lo: 1.0, hi: 1.0 };

        assert_symmetric_failure(
            &first,
            RootBracket {
                lo: f64::NAN,
                hi: 2.0,
            },
            &second,
            valid,
            ExactRootRelationFailure::InvalidBracket,
        );
        assert_symmetric_failure(
            &first,
            RootBracket { lo: 2.0, hi: 1.0 },
            &second,
            valid,
            ExactRootRelationFailure::InvalidBracket,
        );
        assert_symmetric_failure(
            &first,
            RootBracket { lo: -2.0, hi: 2.0 },
            &second,
            valid,
            ExactRootRelationFailure::BracketDoesNotIsolateOneRoot,
        );
    }

    #[test]
    fn ambiguous_exact_isolation_never_publishes_a_relation() {
        let first = polynomial(&[1.0, 1.0]);
        let second = first.clone();
        let bracket = RootBracket {
            lo: -1.0e308,
            hi: 1.0e308,
        };
        assert_symmetric_failure(
            &first,
            bracket,
            &second,
            bracket,
            ExactRootRelationFailure::ExactArithmetic(
                RootIsolationFailure::UnsafeArithmeticEnvelope,
            ),
        );
    }
}
