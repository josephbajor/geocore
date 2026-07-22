use super::*;

fn returning_specs(
    source_loop: usize,
    pairs: &[[usize; 2]],
    reversed: bool,
    fragment_counts: &[usize],
) -> (
    [Vec<PeriodicBoundaryRootSpec>; 2],
    Vec<PeriodicBoundaryTraceSpec>,
) {
    assert_eq!(pairs.len(), fragment_counts.len());
    let root_count = pairs.len() * 2;
    let mut roots: [Vec<PeriodicBoundaryRootSpec>; 2] = core::array::from_fn(|_| Vec::new());
    roots[source_loop] = (0..root_count)
        .map(|cyclic_order| {
            let parameter = cyclic_order as f64 + 0.25;
            PeriodicBoundaryRootSpec {
                key: PeriodicSourceRootKey {
                    endpoint: 10_000 + source_loop * 1_000 + cyclic_order,
                    cyclic_order,
                    source_root_ordinal: source_loop * 1_000 + cyclic_order,
                    root_parameter_bits: parameter.to_bits(),
                    root_enclosure_bits: [parameter.to_bits(), parameter.to_bits()],
                    cylinder_chart_shift: 0,
                },
                source_loop_ordinal: source_loop,
            }
        })
        .collect();
    let traces = pairs
        .iter()
        .zip(fragment_counts)
        .enumerate()
        .map(|(trace_ordinal, (pair, &fragment_count))| {
            assert!(fragment_count > 0);
            let orders = if reversed { [pair[1], pair[0]] } else { *pair };
            let terminals = [
                roots[source_loop][orders[0]].clone(),
                roots[source_loop][orders[1]].clone(),
            ];
            let key = PeriodicBoundaryTraceKey {
                component: 30_000 + source_loop * 1_000 + trace_ordinal,
                first_component_ordinal: trace_ordinal * 10,
            };
            let mut path = vec![terminals[0].key.endpoint];
            path.extend(
                (1..fragment_count)
                    .map(|internal| 50_000 + source_loop * 10_000 + trace_ordinal * 100 + internal),
            );
            path.push(terminals[1].key.endpoint);
            PeriodicBoundaryTraceSpec {
                key,
                fragments: (0..fragment_count)
                    .map(|ordinal| PeriodicFragmentSpec {
                        key: PeriodicCutFragmentKey {
                            component: key.component,
                            ordinal: key.first_component_ordinal + ordinal,
                            fragment: 70_000 + source_loop * 10_000 + trace_ordinal * 100 + ordinal,
                            cylinder_period_shift: ordinal as i64 - 1,
                        },
                        endpoints: [path[ordinal], path[ordinal + 1]],
                    })
                    .collect(),
                terminals,
            }
        })
        .collect();
    (roots, traces)
}

fn assert_returning_arrangement(
    arrangement: &MixedPeriodicFaceArrangement,
    roots: &[Vec<PeriodicBoundaryRootSpec>; 2],
    traces: &[PeriodicBoundaryTraceSpec],
    expected_source_spans: usize,
    expected_cycles: usize,
) {
    let fragment_count = traces
        .iter()
        .map(|trace| trace.fragments.len())
        .sum::<usize>();
    assert_eq!(arrangement.source_spans().len(), expected_source_spans);
    assert_eq!(arrangement.cut_fragments().len(), fragment_count);
    assert_eq!(arrangement.adjacency().len(), fragment_count);
    assert_eq!(arrangement.cells().len(), traces.len() + 1);
    assert_eq!(arrangement.proof().closed_cycles(), expected_cycles);
    assert_eq!(arrangement.proof().exterior_cycles(), 2);
    assert_eq!(arrangement.proof().primal_components(), 2);
    assert!(arrangement.proof().dual_connected());
    let remainder = arrangement
        .cells()
        .iter()
        .find(|cell| cell.key() == &PeriodicArrangementCellKey::AnnularRemainder)
        .unwrap();
    assert_eq!(remainder.boundaries().len(), 2);
    assert_eq!(remainder.euler_characteristic(), 0);
    for trace in traces {
        let source_loop = trace.terminals[0].source_loop_ordinal;
        let anchor_span = returning_disk_spans(trace, roots).unwrap()[0];
        let cell = arrangement
            .cells()
            .iter()
            .find(|cell| cell.key() == &PeriodicArrangementCellKey::TraceCell(trace.key))
            .unwrap();
        assert_eq!(cell.boundaries().len(), 1);
        assert_eq!(cell.euler_characteristic(), 1);
        assert!(cell.boundaries()[0].uses().iter().any(|use_| matches!(
            (use_.edge(), use_.direction()),
            (ArrangementEdgeKey::Source(key), ArrangementDirection::Forward)
                if key.topology_ordinal() == source_loop
                    && key.cyclic_span_ordinal() == Some(anchor_span)
        )));
    }
}

#[test]
fn disjoint_nested_and_mixed_returning_families_are_constructed_generically() {
    let families = [
        [[0, 1], [2, 3], [4, 5]],
        [[0, 5], [1, 4], [2, 3]],
        [[0, 3], [1, 2], [4, 5]],
    ];
    for pairs in families {
        for source_loop in 0..2 {
            for reversed in [false, true] {
                for directions in [
                    [ArrangementDirection::Forward, ArrangementDirection::Reverse],
                    [ArrangementDirection::Reverse, ArrangementDirection::Forward],
                ] {
                    let counts = [1, 2, 4];
                    let (roots, traces) = returning_specs(source_loop, &pairs, reversed, &counts);
                    assert_eq!(
                        traces
                            .iter()
                            .map(|trace| trace.fragments.len())
                            .collect::<Vec<_>>(),
                        counts
                    );
                    let arrangement =
                        arrange_boundary_trace_spec(traces.clone(), roots.clone(), directions)
                            .unwrap();
                    assert_returning_arrangement(&arrangement, &roots, &traces, 7, 7);
                    if pairs == [[0, 5], [1, 4], [2, 3]] {
                        assert!(arrangement.adjacency().iter().any(|adjacency| matches!(
                            (adjacency.forward_cell(), adjacency.reverse_cell()),
                            (
                                PeriodicArrangementCellKey::TraceCell(_),
                                PeriodicArrangementCellKey::TraceCell(_)
                            )
                        )));
                    }
                }
            }
        }
    }
}

#[test]
fn returning_families_on_both_source_rings_share_one_annular_remainder() {
    for reversed in [false, true] {
        for directions in [
            [ArrangementDirection::Forward, ArrangementDirection::Reverse],
            [ArrangementDirection::Reverse, ArrangementDirection::Forward],
        ] {
            let (mut roots, mut traces) = returning_specs(0, &[[0, 1], [2, 3]], reversed, &[1, 3]);
            let (second_roots, second_traces) =
                returning_specs(1, &[[0, 3], [1, 2]], reversed, &[2, 4]);
            roots[1] = second_roots[1].clone();
            traces.extend(second_traces);
            let arrangement =
                arrange_boundary_trace_spec(traces.clone(), roots.clone(), directions).unwrap();
            assert_returning_arrangement(&arrangement, &roots, &traces, 8, 8);
        }
    }
}

#[test]
fn alternating_returning_terminals_fail_closed() {
    let (roots, traces) = returning_specs(0, &[[0, 2], [1, 3]], false, &[2, 3]);
    assert!(matches!(
        arrange_boundary_trace_spec(
            traces,
            roots,
            [ArrangementDirection::Forward, ArrangementDirection::Reverse],
        ),
        Err(MixedPeriodicArrangementError::BoundaryTraceMatchingMismatch(_))
    ));
}

#[test]
fn complete_mixed_returning_and_transverse_family_is_explicitly_unsupported() {
    // One transverse trace plus one disk-cutting return on each ring covers
    // every retained root exactly once without alternating terminal pairs.
    let roots: [Vec<PeriodicBoundaryRootSpec>; 2] = core::array::from_fn(|source_loop| {
        (0..3)
            .map(|cyclic_order| {
                let parameter = (source_loop * 10 + cyclic_order) as f64 + 0.5;
                PeriodicBoundaryRootSpec {
                    key: PeriodicSourceRootKey {
                        endpoint: 90_000 + source_loop * 10 + cyclic_order,
                        cyclic_order,
                        source_root_ordinal: source_loop * 3 + cyclic_order,
                        root_parameter_bits: parameter.to_bits(),
                        root_enclosure_bits: [parameter.to_bits(), parameter.to_bits()],
                        cylinder_chart_shift: 0,
                    },
                    source_loop_ordinal: source_loop,
                }
            })
            .collect()
    });
    let terminal_pairs = [[(0, 0), (1, 0)], [(0, 1), (0, 2)], [(1, 1), (1, 2)]];
    let traces = terminal_pairs
        .into_iter()
        .enumerate()
        .map(|(trace_ordinal, terminals)| {
            let terminals = terminals.map(|(source_loop, order)| roots[source_loop][order].clone());
            let key = PeriodicBoundaryTraceKey {
                component: 91_000 + trace_ordinal,
                first_component_ordinal: trace_ordinal,
            };
            PeriodicBoundaryTraceSpec {
                key,
                fragments: vec![PeriodicFragmentSpec {
                    key: PeriodicCutFragmentKey {
                        component: key.component,
                        ordinal: key.first_component_ordinal,
                        fragment: 92_000 + trace_ordinal,
                        cylinder_period_shift: 0,
                    },
                    endpoints: [terminals[0].key.endpoint, terminals[1].key.endpoint],
                }],
                terminals,
            }
        })
        .collect::<Vec<_>>();
    let returning = traces[1].key;
    let transverse = traces[0].key;
    assert_eq!(
        arrange_boundary_trace_spec(
            traces,
            roots,
            [ArrangementDirection::Forward, ArrangementDirection::Reverse],
        ),
        Err(
            MixedPeriodicArrangementError::MixedBoundaryTraceFamiliesUnsupported {
                returning,
                transverse,
            }
        )
    );
}

#[test]
fn returning_trace_input_permutations_are_deterministic() {
    let (roots, traces) = returning_specs(0, &[[0, 5], [1, 4], [2, 3]], false, &[1, 2, 4]);
    let directions = [ArrangementDirection::Forward, ArrangementDirection::Reverse];
    let expected = arrange_boundary_trace_spec(traces.clone(), roots.clone(), directions).unwrap();
    for order in [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ] {
        let permuted = order.map(|index| traces[index].clone()).to_vec();
        assert_eq!(
            arrange_boundary_trace_spec(permuted, roots.clone(), directions),
            Ok(expected.clone())
        );
    }
}
