//! Deterministic candidate deduplication and first-emission semantics.

use kgeom::param::ParamRange;
use kgeom::vec::Point3;

/// Emits a candidate only when no previously emitted item is equivalent.
///
/// This deliberately retains the first representative and preserves discovery
/// order. Result constructors remain responsible for final canonical sorting.
pub(super) fn emit_distinct_by<T>(
    emitted: &mut Vec<T>,
    candidate: T,
    equivalent: impl Fn(&T, &T) -> bool,
) -> bool {
    if emitted
        .iter()
        .any(|existing| equivalent(existing, &candidate))
    {
        false
    } else {
        emitted.push(candidate);
        true
    }
}

/// Emits the first candidate at each model-space point within `tolerance`.
pub(super) fn emit_distinct_spatial<T>(
    emitted: &mut Vec<T>,
    candidate: T,
    point: impl Fn(&T) -> Point3,
    tolerance: f64,
) -> bool {
    emit_distinct_by(emitted, candidate, |existing, candidate| {
        point(existing).dist(point(candidate)) <= tolerance
    })
}

/// Emits the first branch with each parameter interval within `tolerance`.
pub(super) fn emit_distinct_range<T>(
    emitted: &mut Vec<T>,
    candidate: T,
    range: impl Fn(&T) -> ParamRange,
    tolerance: f64,
) -> bool {
    emit_distinct_by(emitted, candidate, |existing, candidate| {
        let existing = range(existing);
        let candidate = range(candidate);
        (existing.lo - candidate.lo).abs() <= tolerance
            && (existing.hi - candidate.hi).abs() <= tolerance
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, PartialEq)]
    struct Candidate {
        point: Point3,
        range: ParamRange,
        label: &'static str,
    }

    #[test]
    fn spatial_emission_is_first_wins_and_discovery_ordered() {
        let mut emitted = Vec::new();
        assert!(emit_distinct_spatial(
            &mut emitted,
            Candidate {
                point: Point3::new(1.0, 0.0, 0.0),
                range: ParamRange::new(0.0, 1.0),
                label: "first",
            },
            |candidate| candidate.point,
            1e-6,
        ));
        assert!(!emit_distinct_spatial(
            &mut emitted,
            Candidate {
                point: Point3::new(1.0 + 5e-7, 0.0, 0.0),
                range: ParamRange::new(2.0, 3.0),
                label: "duplicate",
            },
            |candidate| candidate.point,
            1e-6,
        ));
        assert!(emit_distinct_spatial(
            &mut emitted,
            Candidate {
                point: Point3::new(2.0, 0.0, 0.0),
                range: ParamRange::new(4.0, 5.0),
                label: "second",
            },
            |candidate| candidate.point,
            1e-6,
        ));
        assert_eq!(
            emitted
                .iter()
                .map(|candidate| candidate.label)
                .collect::<Vec<_>>(),
            ["first", "second"]
        );
    }

    #[test]
    fn range_emission_retains_first_equivalent_branch() {
        let mut emitted = Vec::new();
        let make = |range, label| Candidate {
            point: Point3::new(0.0, 0.0, 0.0),
            range,
            label,
        };
        assert!(emit_distinct_range(
            &mut emitted,
            make(ParamRange::new(0.0, 1.0), "first"),
            |candidate| candidate.range,
            1e-6,
        ));
        assert!(!emit_distinct_range(
            &mut emitted,
            make(ParamRange::new(5e-7, 1.0 + 5e-7), "duplicate"),
            |candidate| candidate.range,
            1e-6,
        ));
        assert_eq!(emitted[0].label, "first");
    }
}
