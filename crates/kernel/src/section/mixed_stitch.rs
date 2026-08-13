//! Deterministic directed assembly of published section fragments.
//!
//! Exact endpoint interning happens before this module runs.  Consequently an
//! endpoint index, rather than a rounded point or metric interval, is the sole
//! incidence authority.  The walk is independent of carrier family and
//! fragment count: every bounded fragment contributes one directed arrival and
//! departure, and only one-in/one-out components become certified cycles.

use super::{
    SectionCurveEndpoint, SectionCurveEndpointTopology, SectionCurveFragment,
    SectionCurveFragmentSpan, SectionIsolatedContact,
};
use crate::error::{Error, Result};

/// One maximal directed component in deterministic first-fragment order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedStitchComponent {
    pub(crate) fragments: Vec<usize>,
    pub(crate) isolated_contacts: Vec<usize>,
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

/// Assemble every published fragment family by exact endpoint ID.
pub(crate) fn stitch_curve_fragments(
    fragments: &[SectionCurveFragment],
    isolated_contacts: &[SectionIsolatedContact],
    endpoints: &[SectionCurveEndpoint],
    through_contact_count: usize,
) -> Result<MixedStitchResult> {
    let occurrences = fragments
        .iter()
        .enumerate()
        .map(|(source, fragment)| EndpointPairOccurrence {
            source,
            endpoint_pair: fragment_endpoints(fragment),
        })
        .collect::<Vec<_>>();
    let contact_terminals = endpoints
        .iter()
        .map(|endpoint| match endpoint.topology() {
            SectionCurveEndpointTopology::ThroughContact { contact }
                if *contact < through_contact_count =>
            {
                Ok(true)
            }
            SectionCurveEndpointTopology::ThroughContact { .. } => Err(inconsistent_topology(
                "section endpoint referenced an unknown through-contact",
            )),
            _ => Ok(false),
        })
        .collect::<Result<Vec<_>>>()?;
    let stitched = stitch_endpoint_occurrences_with_terminals(
        &occurrences,
        endpoints.len(),
        &contact_terminals,
    )?;
    let mut components = stitched
        .components
        .into_iter()
        .map(|component| MixedStitchComponent {
            fragments: component.sources,
            isolated_contacts: Vec::new(),
            closed: component.closed,
        })
        .collect::<Vec<_>>();
    let mut endpoint_components = vec![None; endpoints.len()];
    for (component, stitched) in components.iter().enumerate() {
        for &fragment in &stitched.fragments {
            let Some(endpoint_pair) = fragments.get(fragment).and_then(fragment_endpoints) else {
                continue;
            };
            for endpoint in [endpoint_pair.departure, endpoint_pair.arrival] {
                let Some(slot) = endpoint_components.get_mut(endpoint) else {
                    return Err(inconsistent_topology(
                        "section fragment contact referenced an unknown endpoint",
                    ));
                };
                if slot.is_some_and(|current| current != component) {
                    return Err(inconsistent_topology(
                        "one section endpoint escaped into multiple mixed components",
                    ));
                }
                *slot = Some(component);
            }
        }
    }
    for (contact, isolated) in isolated_contacts.iter().enumerate() {
        let endpoint = isolated.endpoint();
        let Some(component) = endpoint_components.get(endpoint).copied() else {
            return Err(inconsistent_topology(
                "isolated section contact referenced an unknown endpoint",
            ));
        };
        if let Some(component) = component {
            components[component].isolated_contacts.push(contact);
        } else {
            endpoint_components[endpoint] = Some(components.len());
            components.push(MixedStitchComponent {
                fragments: Vec::new(),
                isolated_contacts: vec![contact],
                closed: true,
            });
        }
    }
    Ok(MixedStitchResult {
        components,
        defects: stitched.defects,
    })
}

/// One exact directed departure/arrival relation, independent of the
/// published curve family that supplied it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct DirectedEndpointPair {
    pub(super) departure: usize,
    pub(super) arrival: usize,
}

/// One published occurrence and its exact directed endpoint relation.  The
/// source token is intentionally opaque: independent publishers can identify
/// a family and family-local index without teaching this graph those families.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct EndpointPairOccurrence<Source> {
    pub(super) source: Source,
    pub(super) endpoint_pair: Option<DirectedEndpointPair>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct EndpointPairComponent<Source> {
    pub(super) sources: Vec<Source>,
    pub(super) closed: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct EndpointPairStitchResult<Source> {
    pub(super) components: Vec<EndpointPairComponent<Source>>,
    pub(super) defects: Vec<MixedStitchDefect>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct EndpointIncidence {
    incoming: usize,
    outgoing: usize,
    /// The unique departing fragment, cleared as soon as departure branches.
    successor: Option<usize>,
    /// The unique arriving fragment, cleared as soon as arrival branches.
    predecessor: Option<usize>,
}

impl EndpointIncidence {
    fn record_incoming(&mut self, fragment: usize) {
        self.incoming += 1;
        self.predecessor = if self.incoming == 1 {
            Some(fragment)
        } else {
            None
        };
    }

    fn record_outgoing(&mut self, fragment: usize) {
        self.outgoing += 1;
        self.successor = if self.outgoing == 1 {
            Some(fragment)
        } else {
            None
        };
    }

    fn touched(self) -> bool {
        self.incoming != 0 || self.outgoing != 0
    }

    fn is_one_in_one_out(self) -> bool {
        self.incoming == 1 && self.outgoing == 1
    }
}

/// Count-independent exact-incidence graph shared by chords, arcs, rulings,
/// and future bounded published fragment families.
struct EndpointPairGraph<'a, Source> {
    occurrences: &'a [EndpointPairOccurrence<Source>],
    incidence: Vec<EndpointIncidence>,
    endpoint_valid: Vec<bool>,
    contact_terminals: Vec<bool>,
    defects: Vec<MixedStitchDefect>,
}

impl<'a, Source: Copy> EndpointPairGraph<'a, Source> {
    fn build(
        occurrences: &'a [EndpointPairOccurrence<Source>],
        endpoint_count: usize,
        contact_terminals: &[bool],
    ) -> Result<Self> {
        if contact_terminals.len() != endpoint_count {
            return Err(inconsistent_topology(
                "section contact-terminal evidence was not endpoint-aligned",
            ));
        }
        let mut incidence = vec![EndpointIncidence::default(); endpoint_count];

        for (occurrence, published) in occurrences.iter().enumerate() {
            let Some(endpoint_pair) = published.endpoint_pair else {
                continue;
            };
            if endpoint_pair.departure >= endpoint_count || endpoint_pair.arrival >= endpoint_count
            {
                return Err(inconsistent_topology(
                    "section curve fragment referenced an unknown endpoint",
                ));
            }
            incidence[endpoint_pair.departure].record_outgoing(occurrence);
            incidence[endpoint_pair.arrival].record_incoming(occurrence);
        }

        let mut defects = Vec::new();
        let mut endpoint_valid = vec![true; endpoint_count];
        for (endpoint, counts) in incidence.iter().copied().enumerate() {
            if !counts.touched() {
                continue;
            }
            let valid_contact_terminal = contact_terminals[endpoint]
                && matches!((counts.incoming, counts.outgoing), (1, 0) | (0, 1));
            if !counts.is_one_in_one_out() && !valid_contact_terminal {
                endpoint_valid[endpoint] = false;
                if counts.incoming != 1 {
                    defects.push(MixedStitchDefect::IncomingDegree);
                }
                if counts.outgoing != 1 {
                    defects.push(MixedStitchDefect::OutgoingDegree);
                }
            }
            debug_assert_eq!(
                endpoint_valid[endpoint],
                counts.is_one_in_one_out() || valid_contact_terminal
            );
        }

        Ok(Self {
            occurrences,
            incidence,
            endpoint_valid,
            contact_terminals: contact_terminals.to_vec(),
            defects,
        })
    }

    fn stitch(mut self) -> Result<EndpointPairStitchResult<Source>> {
        let mut used = vec![false; self.occurrences.len()];
        let mut components = Vec::new();
        for first in 0..self.occurrences.len() {
            if used[first] {
                continue;
            }
            let published = self.occurrences[first];
            let Some(_) = published.endpoint_pair else {
                used[first] = true;
                components.push(EndpointPairComponent {
                    sources: vec![published.source],
                    closed: true,
                });
                continue;
            };
            let first = self.chain_start(first)?;
            let first_endpoints = self.occurrences[first]
                .endpoint_pair
                .ok_or_else(|| inconsistent_topology("chain start lost its endpoint pair"))?;
            let (component, admitted) = self.walk_chain(first, first_endpoints, &mut used)?;
            if !admitted {
                self.defects.push(MixedStitchDefect::OpenChain);
            } else {
                components.push(component);
            }
        }

        Ok(EndpointPairStitchResult {
            components,
            defects: self.defects,
        })
    }

    fn chain_start(&self, first: usize) -> Result<usize> {
        let mut current = first;
        for _ in 0..=self.occurrences.len() {
            let endpoints = self.occurrences[current].endpoint_pair.ok_or_else(|| {
                inconsistent_topology("bounded chain search found a whole fragment")
            })?;
            if self.contact_terminals[endpoints.departure] {
                return Ok(current);
            }
            let Some(previous) = self.incidence[endpoints.departure].predecessor else {
                return Ok(current);
            };
            if previous == first {
                return Ok(first);
            }
            current = previous;
        }
        Err(inconsistent_topology(
            "section endpoint predecessor search exceeded the fragment count",
        ))
    }

    fn walk_chain(
        &self,
        first: usize,
        first_endpoints: DirectedEndpointPair,
        used: &mut [bool],
    ) -> Result<(EndpointPairComponent<Source>, bool)> {
        used[first] = true;
        let origin = first_endpoints.departure;
        let mut at = first_endpoints.arrival;
        let mut ordered = vec![self.occurrences[first].source];
        let mut unambiguous = self.endpoint_valid[origin] && self.endpoint_valid[at];
        let closed = loop {
            if at == origin {
                break unambiguous;
            }
            let Some(next) = self.incidence[at].successor else {
                break false;
            };
            if used[next] {
                break false;
            }
            let next_published = self.occurrences[next];
            let next_endpoints = next_published.endpoint_pair.ok_or_else(|| {
                inconsistent_topology("whole section fragment appeared in endpoint incidence")
            })?;
            if next_endpoints.departure != at {
                return Err(inconsistent_topology(
                    "section fragment departure disagreed with endpoint incidence",
                ));
            }
            used[next] = true;
            ordered.push(next_published.source);
            unambiguous &= self.endpoint_valid[next_endpoints.departure]
                && self.endpoint_valid[next_endpoints.arrival];
            at = next_endpoints.arrival;
        };
        let admitted =
            closed || self.contact_terminals[origin] && self.contact_terminals[at] && unambiguous;
        Ok((
            EndpointPairComponent {
                sources: ordered,
                closed,
            },
            admitted,
        ))
    }
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn stitch_endpoint_occurrences<Source: Copy>(
    occurrences: &[EndpointPairOccurrence<Source>],
    endpoint_count: usize,
) -> Result<EndpointPairStitchResult<Source>> {
    stitch_endpoint_occurrences_with_terminals(
        occurrences,
        endpoint_count,
        &vec![false; endpoint_count],
    )
}

fn stitch_endpoint_occurrences_with_terminals<Source: Copy>(
    occurrences: &[EndpointPairOccurrence<Source>],
    endpoint_count: usize,
    contact_terminals: &[bool],
) -> Result<EndpointPairStitchResult<Source>> {
    EndpointPairGraph::build(occurrences, endpoint_count, contact_terminals)?.stitch()
}

fn fragment_endpoints(fragment: &SectionCurveFragment) -> Option<DirectedEndpointPair> {
    match fragment.span() {
        SectionCurveFragmentSpan::Whole => None,
        SectionCurveFragmentSpan::Arc { endpoints, .. } => Some(DirectedEndpointPair {
            departure: endpoints[0].endpoint(),
            arrival: endpoints[1].endpoint(),
        }),
        SectionCurveFragmentSpan::LineSegment { endpoints } => Some(DirectedEndpointPair {
            departure: endpoints[0].endpoint(),
            arrival: endpoints[1].endpoint(),
        }),
        SectionCurveFragmentSpan::BoundedProcedural { endpoints } => Some(DirectedEndpointPair {
            departure: endpoints[0].endpoint(),
            arrival: endpoints[1].endpoint(),
        }),
        SectionCurveFragmentSpan::FoldedSupport { endpoints } => Some(DirectedEndpointPair {
            departure: endpoints[0].endpoint(),
            arrival: endpoints[1].endpoint(),
        }),
        SectionCurveFragmentSpan::TouchingSupport { endpoints } => Some(DirectedEndpointPair {
            departure: endpoints[0].endpoint(),
            arrival: endpoints[1].endpoint(),
        }),
    }
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum SymbolicFamily {
        Chord,
        Ruling,
        Arc,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct SymbolicSource {
        family: SymbolicFamily,
        index: usize,
    }

    const CHORD: SymbolicSource = SymbolicSource {
        family: SymbolicFamily::Chord,
        index: 0,
    };
    const RULING_A: SymbolicSource = SymbolicSource {
        family: SymbolicFamily::Ruling,
        index: 0,
    };
    const RULING_B: SymbolicSource = SymbolicSource {
        family: SymbolicFamily::Ruling,
        index: 1,
    };
    const ARC: SymbolicSource = SymbolicSource {
        family: SymbolicFamily::Arc,
        index: 0,
    };

    #[derive(Debug, Clone, Copy)]
    #[repr(usize)]
    enum SymbolicEndpoint {
        A,
        B,
        C,
        D,
    }

    fn bounded(
        departure: SymbolicEndpoint,
        arrival: SymbolicEndpoint,
    ) -> Option<DirectedEndpointPair> {
        Some(DirectedEndpointPair {
            departure: departure as usize,
            arrival: arrival as usize,
        })
    }

    fn stitch_symbolic(
        fragments: &[(SymbolicSource, SymbolicEndpoint, SymbolicEndpoint)],
        endpoint_count: usize,
    ) -> EndpointPairStitchResult<SymbolicSource> {
        let occurrences = fragments
            .iter()
            .map(|(source, departure, arrival)| EndpointPairOccurrence {
                source: *source,
                endpoint_pair: bounded(*departure, *arrival),
            })
            .collect::<Vec<_>>();
        stitch_endpoint_occurrences(&occurrences, endpoint_count).unwrap()
    }

    fn ordered_symbolic_sources(
        result: &EndpointPairStitchResult<SymbolicSource>,
    ) -> Vec<SymbolicSource> {
        let component = result.components.first().expect("one closed component");
        component.sources.clone()
    }

    fn rotate_to_chord(mut sources: Vec<SymbolicSource>) -> Vec<SymbolicSource> {
        let chord = sources
            .iter()
            .position(|source| *source == CHORD)
            .expect("every symbolic cycle contains its planar chord");
        sources.rotate_left(chord);
        sources
    }

    #[test]
    fn empty_and_whole_fragments_are_closed_without_endpoints() {
        assert_eq!(
            stitch_endpoint_occurrences::<usize>(&[], 0).unwrap(),
            EndpointPairStitchResult {
                components: Vec::new(),
                defects: Vec::new(),
            }
        );
        assert_eq!(
            stitch_endpoint_occurrences(
                &[EndpointPairOccurrence {
                    source: 7,
                    endpoint_pair: None,
                }],
                0,
            )
            .unwrap(),
            EndpointPairStitchResult {
                components: vec![EndpointPairComponent {
                    sources: vec![7],
                    closed: true,
                }],
                defects: Vec::new(),
            }
        );
    }

    /// One named directed-cycle case: symbolic edges plus the expected order.
    type DirectedCycleCase<'a> = (
        &'a str,
        &'a [(SymbolicSource, SymbolicEndpoint, SymbolicEndpoint)],
        &'a [SymbolicSource],
    );

    #[test]
    fn mixed_family_cycles_share_one_directed_endpoint_graph() {
        let cases: &[DirectedCycleCase] = &[
            (
                "planar chord plus arc",
                &[
                    (CHORD, SymbolicEndpoint::A, SymbolicEndpoint::B),
                    (ARC, SymbolicEndpoint::B, SymbolicEndpoint::A),
                ],
                &[CHORD, ARC],
            ),
            (
                "planar chord plus two rulings plus arc",
                &[
                    (CHORD, SymbolicEndpoint::A, SymbolicEndpoint::B),
                    (RULING_B, SymbolicEndpoint::C, SymbolicEndpoint::D),
                    (RULING_A, SymbolicEndpoint::B, SymbolicEndpoint::C),
                    (ARC, SymbolicEndpoint::D, SymbolicEndpoint::A),
                ],
                &[CHORD, RULING_A, RULING_B, ARC],
            ),
        ];

        for (name, fragments, expected) in cases {
            let endpoint_count = fragments.len();
            let result = stitch_symbolic(fragments, endpoint_count);
            assert!(result.defects.is_empty(), "{name}: {result:?}");
            assert_eq!(result.components.len(), 1, "{name}: {result:?}");
            assert!(result.components[0].closed, "{name}: {result:?}");
            assert_eq!(
                rotate_to_chord(ordered_symbolic_sources(&result)).as_slice(),
                *expected,
                "{name}"
            );
        }
    }

    #[test]
    fn exact_contact_terminals_admit_open_stratified_chains_only_at_both_ends() {
        let fragments = [
            EndpointPairOccurrence {
                source: RULING_B,
                endpoint_pair: bounded(SymbolicEndpoint::B, SymbolicEndpoint::C),
            },
            EndpointPairOccurrence {
                source: RULING_A,
                endpoint_pair: bounded(SymbolicEndpoint::A, SymbolicEndpoint::B),
            },
        ];
        let admitted =
            stitch_endpoint_occurrences_with_terminals(&fragments, 3, &[true, false, true])
                .unwrap();
        assert!(admitted.defects.is_empty(), "{admitted:?}");
        assert_eq!(admitted.components.len(), 1);
        assert_eq!(admitted.components[0].sources, vec![RULING_A, RULING_B]);
        assert!(!admitted.components[0].closed);

        let refused =
            stitch_endpoint_occurrences_with_terminals(&fragments, 3, &[true, false, false])
                .unwrap();
        assert!(refused.components.is_empty());
        assert!(refused.defects.contains(&MixedStitchDefect::OpenChain));
    }

    #[test]
    fn fragment_permutation_changes_only_the_cycle_rotation() {
        let canonical = [
            (CHORD, SymbolicEndpoint::A, SymbolicEndpoint::B),
            (RULING_A, SymbolicEndpoint::B, SymbolicEndpoint::C),
            (RULING_B, SymbolicEndpoint::C, SymbolicEndpoint::D),
            (ARC, SymbolicEndpoint::D, SymbolicEndpoint::A),
        ];
        let permutations = [[0, 1, 2, 3], [2, 0, 3, 1], [3, 2, 1, 0], [1, 3, 0, 2]];
        let expected = vec![CHORD, RULING_A, RULING_B, ARC];

        for permutation in permutations {
            let fragments = permutation.map(|index| canonical[index]);
            let result = stitch_symbolic(&fragments, 4);
            assert!(result.defects.is_empty(), "{permutation:?}: {result:?}");
            assert_eq!(result.components.len(), 1, "{permutation:?}");
            assert_eq!(
                rotate_to_chord(ordered_symbolic_sources(&result)),
                expected,
                "{permutation:?}"
            );
        }
    }

    /// One named refusal case: occurrences, cycle count, expected defects.
    type RefusalCase<'a> = (
        &'a str,
        &'a [EndpointPairOccurrence<SymbolicSource>],
        usize,
        &'a [MixedStitchDefect],
    );

    #[test]
    fn open_and_branching_symbolic_incidence_are_refused() {
        let cases: &[RefusalCase] = &[
            (
                "open chord",
                &[EndpointPairOccurrence {
                    source: CHORD,
                    endpoint_pair: bounded(SymbolicEndpoint::A, SymbolicEndpoint::B),
                }],
                2,
                &[
                    MixedStitchDefect::IncomingDegree,
                    MixedStitchDefect::OutgoingDegree,
                    MixedStitchDefect::OpenChain,
                ],
            ),
            (
                "branching departure and arrival",
                &[
                    EndpointPairOccurrence {
                        source: CHORD,
                        endpoint_pair: bounded(SymbolicEndpoint::A, SymbolicEndpoint::B),
                    },
                    EndpointPairOccurrence {
                        source: RULING_A,
                        endpoint_pair: bounded(SymbolicEndpoint::A, SymbolicEndpoint::C),
                    },
                    EndpointPairOccurrence {
                        source: ARC,
                        endpoint_pair: bounded(SymbolicEndpoint::B, SymbolicEndpoint::A),
                    },
                    EndpointPairOccurrence {
                        source: RULING_B,
                        endpoint_pair: bounded(SymbolicEndpoint::C, SymbolicEndpoint::A),
                    },
                ],
                3,
                &[
                    MixedStitchDefect::IncomingDegree,
                    MixedStitchDefect::OutgoingDegree,
                    MixedStitchDefect::OpenChain,
                ],
            ),
        ];

        for (name, endpoint_pairs, endpoint_count, expected_defects) in cases {
            let result = stitch_endpoint_occurrences(endpoint_pairs, *endpoint_count).unwrap();
            assert!(result.components.is_empty(), "{name}: {result:?}");
            for defect in *expected_defects {
                assert!(result.defects.contains(defect), "{name}: {result:?}");
            }
        }
    }

    #[test]
    fn unknown_endpoint_is_a_typed_internal_error() {
        let error = stitch_endpoint_occurrences(
            &[EndpointPairOccurrence {
                source: CHORD,
                endpoint_pair: bounded(SymbolicEndpoint::A, SymbolicEndpoint::C),
            }],
            2,
        )
        .unwrap_err();
        assert!(matches!(error, Error::InconsistentTopology { .. }));
    }

    #[test]
    fn opposing_arrivals_are_not_implicitly_reversed() {
        let result = stitch_endpoint_occurrences(
            &[
                EndpointPairOccurrence {
                    source: CHORD,
                    endpoint_pair: bounded(SymbolicEndpoint::A, SymbolicEndpoint::B),
                },
                EndpointPairOccurrence {
                    source: RULING_A,
                    endpoint_pair: bounded(SymbolicEndpoint::C, SymbolicEndpoint::B),
                },
            ],
            3,
        )
        .unwrap();
        assert!(result.components.is_empty());
        assert!(result.defects.contains(&MixedStitchDefect::IncomingDegree));
        assert!(result.defects.contains(&MixedStitchDefect::OutgoingDegree));
        assert!(result.defects.contains(&MixedStitchDefect::OpenChain));
    }
}
