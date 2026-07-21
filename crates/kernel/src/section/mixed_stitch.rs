//! Deterministic directed assembly of published analytic section fragments.
//!
//! Exact endpoint interning happens before this module runs.  Consequently an
//! endpoint index, rather than a rounded point or metric interval, is the sole
//! incidence authority.  The walk is independent of carrier family and
//! fragment count: every bounded fragment contributes one directed arrival and
//! departure, and only one-in/one-out components become certified cycles.

use super::{SectionCurveFragment, SectionCurveFragmentSpan};
use crate::error::{Error, Result};

/// One maximal directed component in deterministic first-fragment order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedStitchComponent {
    pub(crate) fragments: Vec<usize>,
    pub(crate) closed: bool,
}

/// Structural ambiguity found by exact directed-incidence assembly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MixedStitchDefect {
    /// A used endpoint does not have exactly one arriving fragment.
    IncomingDegree,
    /// A used endpoint does not have exactly one departing fragment.
    OutgoingDegree,
    /// A maximal directed walk did not return to its first endpoint.
    OpenChain,
}

/// Deterministic mixed-family component evidence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedStitchResult {
    pub(crate) components: Vec<MixedStitchComponent>,
    pub(crate) defects: Vec<MixedStitchDefect>,
}

/// Assemble all published whole, arc, and line fragments by exact endpoint ID.
pub(crate) fn stitch_curve_fragments(
    fragments: &[SectionCurveFragment],
    endpoint_count: usize,
) -> Result<MixedStitchResult> {
    let endpoints = fragments.iter().map(fragment_endpoints).collect::<Vec<_>>();
    stitch_endpoint_pairs(&endpoints, endpoint_count)
}

fn stitch_endpoint_pairs(
    endpoints: &[Option<[usize; 2]>],
    endpoint_count: usize,
) -> Result<MixedStitchResult> {
    let mut incoming = vec![Vec::new(); endpoint_count];
    let mut outgoing = vec![Vec::new(); endpoint_count];
    let mut touched = vec![false; endpoint_count];

    for (fragment_index, endpoint_pair) in endpoints.iter().copied().enumerate() {
        let Some(pair) = endpoint_pair else {
            continue;
        };
        if pair.into_iter().any(|endpoint| endpoint >= endpoint_count) {
            return Err(inconsistent_topology(
                "section curve fragment referenced an unknown endpoint",
            ));
        }
        outgoing[pair[0]].push(fragment_index);
        incoming[pair[1]].push(fragment_index);
        touched[pair[0]] = true;
        touched[pair[1]] = true;
    }

    let mut defects = Vec::new();
    let mut endpoint_valid = vec![true; endpoint_count];
    for endpoint in 0..endpoint_count {
        if !touched[endpoint] {
            continue;
        }
        if incoming[endpoint].len() != 1 {
            endpoint_valid[endpoint] = false;
            defects.push(MixedStitchDefect::IncomingDegree);
        }
        if outgoing[endpoint].len() != 1 {
            endpoint_valid[endpoint] = false;
            defects.push(MixedStitchDefect::OutgoingDegree);
        }
    }

    let mut used = vec![false; endpoints.len()];
    let mut components = Vec::new();
    for first in 0..endpoints.len() {
        if used[first] {
            continue;
        }
        let Some(first_endpoints) = endpoints[first] else {
            used[first] = true;
            components.push(MixedStitchComponent {
                fragments: vec![first],
                closed: true,
            });
            continue;
        };
        let component = walk_chain(
            first,
            first_endpoints,
            endpoints,
            &outgoing,
            &endpoint_valid,
            &mut used,
        )?;
        if !component.closed {
            defects.push(MixedStitchDefect::OpenChain);
        } else {
            components.push(component);
        }
    }

    Ok(MixedStitchResult {
        components,
        defects,
    })
}

fn fragment_endpoints(fragment: &SectionCurveFragment) -> Option<[usize; 2]> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => None,
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            Some(endpoints.each_ref().map(|endpoint| endpoint.endpoint()))
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            Some(endpoints.each_ref().map(|endpoint| endpoint.endpoint()))
        }
    }
}

fn walk_chain(
    first: usize,
    first_endpoints: [usize; 2],
    endpoints: &[Option<[usize; 2]>],
    outgoing: &[Vec<usize>],
    endpoint_valid: &[bool],
    used: &mut [bool],
) -> Result<MixedStitchComponent> {
    used[first] = true;
    let origin = first_endpoints[0];
    let mut at = first_endpoints[1];
    let mut ordered = vec![first];
    let mut unambiguous = endpoint_valid[origin] && endpoint_valid[at];
    let closed = loop {
        if at == origin {
            break unambiguous;
        }
        let [next] = outgoing[at].as_slice() else {
            break false;
        };
        if used[*next] {
            break false;
        }
        let next_endpoints = endpoints[*next].ok_or_else(|| {
            inconsistent_topology("whole section fragment appeared in endpoint incidence")
        })?;
        if next_endpoints[0] != at {
            return Err(inconsistent_topology(
                "section fragment departure disagreed with endpoint incidence",
            ));
        }
        used[*next] = true;
        ordered.push(*next);
        unambiguous &= endpoint_valid[next_endpoints[0]] && endpoint_valid[next_endpoints[1]];
        at = next_endpoints[1];
    };
    Ok(MixedStitchComponent {
        fragments: ordered,
        closed,
    })
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_and_whole_fragments_are_closed_without_endpoints() {
        assert_eq!(
            stitch_endpoint_pairs(&[], 0).unwrap(),
            MixedStitchResult {
                components: Vec::new(),
                defects: Vec::new(),
            }
        );
        assert_eq!(
            stitch_endpoint_pairs(&[None], 0).unwrap(),
            MixedStitchResult {
                components: vec![MixedStitchComponent {
                    fragments: vec![0],
                    closed: true,
                }],
                defects: Vec::new(),
            }
        );
    }

    #[test]
    fn directed_mixed_cycle_uses_deterministic_input_first_traversal() {
        let result =
            stitch_endpoint_pairs(&[Some([0, 1]), Some([2, 3]), Some([1, 2]), Some([3, 0])], 4)
                .unwrap();
        assert_eq!(result.defects, Vec::new());
        assert_eq!(
            result.components,
            vec![MixedStitchComponent {
                fragments: vec![0, 2, 1, 3],
                closed: true,
            }]
        );
    }

    #[test]
    fn open_and_degree_ambiguity_publish_no_component() {
        let open = stitch_endpoint_pairs(&[Some([0, 1])], 2).unwrap();
        assert!(open.components.is_empty());
        assert!(open.defects.contains(&MixedStitchDefect::IncomingDegree));
        assert!(open.defects.contains(&MixedStitchDefect::OutgoingDegree));
        assert!(open.defects.contains(&MixedStitchDefect::OpenChain));

        let branching =
            stitch_endpoint_pairs(&[Some([0, 1]), Some([0, 2]), Some([1, 0]), Some([2, 0])], 3)
                .unwrap();
        assert!(branching.components.is_empty());
        assert!(
            branching
                .defects
                .contains(&MixedStitchDefect::IncomingDegree)
        );
        assert!(
            branching
                .defects
                .contains(&MixedStitchDefect::OutgoingDegree)
        );
    }

    #[test]
    fn unknown_endpoint_is_a_typed_internal_error() {
        let error = stitch_endpoint_pairs(&[Some([0, 2])], 2).unwrap_err();
        assert!(matches!(error, Error::InconsistentTopology { .. }));
    }

    #[test]
    fn opposing_arrivals_are_not_implicitly_reversed() {
        let result = stitch_endpoint_pairs(&[Some([0, 1]), Some([2, 1])], 3).unwrap();
        assert!(result.components.is_empty());
        assert!(result.defects.contains(&MixedStitchDefect::IncomingDegree));
        assert!(result.defects.contains(&MixedStitchDefect::OutgoingDegree));
        assert!(result.defects.contains(&MixedStitchDefect::OpenChain));
    }
}
