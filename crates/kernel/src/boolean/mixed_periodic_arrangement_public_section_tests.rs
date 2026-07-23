#[test]
fn public_section_evidence_adapts_deterministically_in_both_operand_orders() {
    let first_graph = public_graph(false);
    let (first_operand, first_face) = certified_face(&first_graph);
    let first =
        arrange_mixed_periodic_face(&first_graph, first_face.clone(), first_operand).unwrap();
    assert_eq!(
        arrange_mixed_periodic_face(&first_graph, first_face, first_operand).unwrap(),
        first
    );

    let second_graph = public_graph(true);
    let (second_operand, second_face) = certified_face(&second_graph);
    let second =
        arrange_mixed_periodic_face(&second_graph, second_face.clone(), second_operand).unwrap();
    assert_eq!(
        arrange_mixed_periodic_face(&second_graph, second_face, second_operand).unwrap(),
        second
    );

    // Swapping operands reverses the section carrier convention and can
    // therefore exchange forward/reverse cut sides. Compare graph-local
    // identities only through stable branch/source-ordinal lineage.
    assert_eq!(first.source_spans(), second.source_spans());
    let proof_signature = |arrangement: &MixedPeriodicFaceArrangement| {
        let proof = arrangement.proof();
        let mut degrees = proof
            .endpoint_degrees()
            .iter()
            .map(|(_, degree)| (degree.source(), degree.cut()))
            .collect::<Vec<_>>();
        degrees.sort_unstable();
        (
            degrees,
            proof.directed_darts_conserved(),
            proof.source_spans_conserved(),
            proof.opposed_cut_pairs(),
            proof.closed_cycles(),
            proof.exterior_cycles(),
            proof.primal_components(),
            proof.source_boundary_components(),
            proof.dual_connected(),
            proof.surface_euler_characteristic(),
            proof.surface_genus(),
        )
    };
    assert_eq!(proof_signature(&first), proof_signature(&second));
    let cell_signature =
        |graph: &BodySectionGraph, arrangement: &MixedPeriodicFaceArrangement| {
            arrangement
                .cells()
                .iter()
                .map(|cell| {
                    (
                        cell_role(graph, *cell.key()),
                        cell.boundaries().len(),
                        cell.euler_characteristic(),
                        cell.genus(),
                    )
                })
                .collect::<BTreeSet<_>>()
        };
    assert_eq!(
        cell_signature(&first_graph, &first),
        cell_signature(&second_graph, &second)
    );

    let first_orientations = component_orientations(&first_graph);
    let second_orientations = component_orientations(&second_graph);
    assert_eq!(first_orientations.len(), second_orientations.len());
    let second_adjacency = second
        .adjacency()
        .iter()
        .map(|adjacency| {
            (
                fragment_lineage(&second_graph, adjacency.cut().fragment()),
                adjacency,
            )
        })
        .collect::<BTreeMap<_, _>>();
    for first_edge in first.adjacency() {
        let lineage = fragment_lineage(&first_graph, first_edge.cut().fragment());
        let second_edge = second_adjacency[&lineage];
        let component = component_lineage(&first_graph, first_edge.cut().component());
        let first_sides = [
            cell_role(&first_graph, *first_edge.forward_cell()),
            cell_role(&first_graph, *first_edge.reverse_cell()),
        ];
        let second_sides = [
            cell_role(&second_graph, *second_edge.forward_cell()),
            cell_role(&second_graph, *second_edge.reverse_cell()),
        ];
        if first_orientations[&component] == second_orientations[&component] {
            assert_eq!(first_sides, second_sides);
        } else {
            assert_eq!(
                first_sides,
                [second_sides[1].clone(), second_sides[0].clone()]
            );
        }
    }
    assert_eq!(first.cells().len(), 3);
    assert_eq!(first.proof().surface_euler_characteristic(), 0);
    assert_eq!(first.proof().surface_genus(), 0);
    assert!(first.proof().dual_connected());
    assert_eq!(first.proof().opposed_cut_pairs(), 8);
}
