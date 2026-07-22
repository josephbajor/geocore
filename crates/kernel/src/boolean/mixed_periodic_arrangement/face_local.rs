use super::{BodySectionGraph, SectionCurveFragment, SectionCurveFragmentSpan};

pub(super) struct UnstitchedFragmentPaths {
    pub(super) paths: Vec<Vec<usize>>,
    pub(super) assigned: Vec<bool>,
}

/// Reconstruct the deterministic owner-local paths sealed by Section.
///
/// This is graph validation, not geometric recertification: endpoint
/// incidence proves that every retained fragment has one predecessor and one
/// successor at most, and path order assigns each bounded fragment once.
pub(super) fn collect_unstitched_fragment_paths(
    graph: &BodySectionGraph,
) -> UnstitchedFragmentPaths {
    let mut assigned = vec![false; graph.curve_fragments().len()];
    for component in graph.curve_components() {
        for &fragment in component.fragments() {
            if let Some(slot) = assigned.get_mut(fragment) {
                *slot = true;
            }
        }
    }

    let endpoint_count = graph.curve_endpoints().len();
    let mut endpoint_pairs = vec![None; graph.curve_fragments().len()];
    let mut incoming = vec![0_usize; endpoint_count];
    let mut outgoing = vec![0_usize; endpoint_count];
    let mut successor = vec![None; endpoint_count];
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        if assigned[fragment_index] {
            continue;
        }
        let Some([departure, arrival]) = bounded_fragment_endpoints(fragment) else {
            continue;
        };
        let (Some(incoming_slot), Some(outgoing_slot), Some(successor_slot)) = (
            incoming.get_mut(arrival),
            outgoing.get_mut(departure),
            successor.get_mut(departure),
        ) else {
            continue;
        };
        *incoming_slot = incoming_slot.saturating_add(1);
        *outgoing_slot = outgoing_slot.saturating_add(1);
        *successor_slot = if *outgoing_slot == 1 {
            Some(fragment_index)
        } else {
            None
        };
        endpoint_pairs[fragment_index] = Some([departure, arrival]);
    }

    let eligible = endpoint_pairs
        .iter()
        .map(|pair| {
            pair.is_some_and(|[departure, arrival]| {
                incoming[departure] <= 1
                    && outgoing[departure] == 1
                    && incoming[arrival] == 1
                    && outgoing[arrival] <= 1
            })
        })
        .collect::<Vec<_>>();
    let mut paths = Vec::new();
    for first in 0..graph.curve_fragments().len() {
        if assigned[first] || !eligible[first] {
            continue;
        }
        let [departure, _] = endpoint_pairs[first].expect("eligible fragment has endpoints");
        if incoming[departure] != 0 {
            continue;
        }
        let mut path = Vec::new();
        let mut at = first;
        loop {
            if assigned[at] || !eligible[at] {
                break;
            }
            assigned[at] = true;
            path.push(at);
            let [_, arrival] = endpoint_pairs[at].expect("eligible fragment has endpoints");
            let Some(next) = successor[arrival] else {
                break;
            };
            at = next;
        }
        if !path.is_empty() {
            paths.push(path);
        }
    }
    UnstitchedFragmentPaths { paths, assigned }
}

fn bounded_fragment_endpoints(fragment: &SectionCurveFragment) -> Option<[usize; 2]> {
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
