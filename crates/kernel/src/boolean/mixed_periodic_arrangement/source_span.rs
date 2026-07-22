//! Canonical sampling intervals for topology-owned periodic source spans.

use super::PeriodicSourceLoopKey;

/// Return the strict canonical-longitude interval represented by one bounded
/// source-span key.
///
/// Root chart shifts belong to the incident lifted Section trace. They do not
/// choose which source-ring span the key names, especially when the source fin
/// is reversed. The cyclic-span ordinal and canonical root enclosures are the
/// exact ownership authority; only the last canonical span adds one period.
pub(crate) fn canonical_source_span_open_interval(
    source: PeriodicSourceLoopKey,
) -> Option<[f64; 2]> {
    let roots = source.terminal_roots()?;
    let span = source.cyclic_span_ordinal()?;
    let start = roots.iter().find(|root| root.cyclic_order() == span)?;
    let end = roots.iter().find(|root| root.cyclic_order() != span)?;
    let start_enclosure = start.root_enclosure();
    let end_enclosure = end.root_enclosure();
    let wrap = if end.cyclic_order() < start.cyclic_order() {
        core::f64::consts::TAU
    } else {
        0.0
    };
    let open = [start_enclosure[1], end_enclosure[0] + wrap];
    (roots[0].cyclic_order() != roots[1].cyclic_order()
        && [open[0], open[1]].into_iter().all(f64::is_finite)
        && open[0] < open[1])
        .then_some(open)
}

#[cfg(test)]
mod tests {
    use super::super::PeriodicSourceRootKey;
    use super::*;
    use crate::boolean::face_arrangement::ArrangementDirection;

    fn root(cyclic_order: usize, parameter: f64, chart_shift: i64) -> PeriodicSourceRootKey {
        PeriodicSourceRootKey {
            endpoint: cyclic_order,
            cyclic_order,
            source_root_ordinal: cyclic_order,
            root_parameter_bits: parameter.to_bits(),
            root_enclosure_bits: [parameter.to_bits(), parameter.to_bits()],
            cylinder_chart_shift: chart_shift,
        }
    }

    #[test]
    fn canonical_span_sampling_ignores_reversed_trace_chart_shifts() {
        let low = root(0, 1.0, 1);
        let high = root(1, 5.0, 0);
        let key = |span, roots| PeriodicSourceLoopKey {
            topology_ordinal: 1,
            source_direction: ArrangementDirection::Reverse,
            cyclic_span_ordinal: Some(span),
            terminal_roots: Some(roots),
        };
        assert_eq!(
            canonical_source_span_open_interval(key(0, [high, low])),
            Some([1.0, 5.0])
        );
        assert_eq!(
            canonical_source_span_open_interval(key(1, [low, high])),
            Some([5.0, 1.0 + core::f64::consts::TAU])
        );
    }
}
