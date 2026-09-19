//! Folded-support graph publication and exact work-budget regressions.

use super::*;

#[test]
fn mixed_simple_repeated_contact_publishes_one_folded_support_component() {
    let rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = mixed_folded_support_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = mixed_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (direction, result, sources) in [
            ("forward", &forward, [first_handle, second_handle]),
            ("reversed", &reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(
                result.raw.is_complete(),
                "{name}/{direction}: {:#?}",
                result.raw
            );
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            assert!(result.skew_cylinder_support_contacts().is_empty());
            assert!(result.skew_cylinder_touching_support_curves().is_empty());
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}/{direction}: expected one mixed folded component")
            };
            assert_eq!(folded.certificate().topology().root_ordinals(), [0, 2]);
            assert_eq!(
                folded
                    .certificate()
                    .topology()
                    .interior_touching_root_ordinal(),
                Some(1)
            );
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportJoin { .. }
                    ))
                    .count(),
                2
            );
            let mut touching_ports = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::TouchingSupportRootJoin {
                        root_ordinal,
                        continuation,
                    } => Some((root_ordinal, continuation)),
                    _ => None,
                })
                .collect::<Vec<_>>();
            touching_ports.sort_unstable();
            assert_eq!(touching_ports, vec![(1, 0), (1, 1)]);
            for edge in &result.branch_graph.edges {
                assert_eq!(edge.topology, IntersectionBranchTopology::Open);
                assert!(edge.certificate.as_skew_cylinder_folded_support().is_some());
            }
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn mixed_simple_repeated_folded_support_owns_atomic_work() {
    let [first, second] = mixed_folded_support_pair(Frame::world());
    let windows = mixed_folded_support_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK);
    let result = exact.result().unwrap();
    assert_eq!(result.branch_graph.edges.len(), 4);
    assert_eq!(result.skew_cylinder_folded_support_curves().len(), 1);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn non_cardinal_seam_mixed_contact_publishes_six_folded_support_members() {
    let rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = non_cardinal_seam_mixed_folded_support_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = non_cardinal_seam_mixed_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (direction, result, sources) in [
            ("forward", &forward, [first_handle, second_handle]),
            ("reversed", &reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(
                result.raw.is_complete(),
                "{name}/{direction}: {:#?}",
                result.raw
            );
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 6);
            assert_eq!(result.branch_graph.edges.len(), 6);
            assert_eq!(result.branch_graph.vertices.len(), 6);
            assert!(result.skew_cylinder_support_contacts().is_empty());
            assert!(result.skew_cylinder_touching_support_curves().is_empty());
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}/{direction}: expected one seam-mixed folded component")
            };
            assert_eq!(folded.certificate().topology().root_ordinals(), [1, 2]);
            assert_eq!(
                folded
                    .certificate()
                    .topology()
                    .interior_touching_root_ordinal(),
                Some(0)
            );
            assert_eq!(folded.certificate().formula_residuals().len(), 6);
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportJoin { .. }
                    ))
                    .count(),
                2
            );
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportSeamJoin { .. }
                    ))
                    .count(),
                2
            );
            let mut touching_ports = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::TouchingSupportRootJoin {
                        root_ordinal,
                        continuation,
                    } => Some((root_ordinal, continuation)),
                    _ => None,
                })
                .collect::<Vec<_>>();
            touching_ports.sort_unstable();
            assert_eq!(touching_ports, vec![(0, 0), (0, 1)]);
            for edge in &result.branch_graph.edges {
                assert_eq!(edge.topology, IntersectionBranchTopology::Open);
                assert!(edge.certificate.as_skew_cylinder_folded_support().is_some());
            }
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn non_cardinal_seam_mixed_folded_support_owns_atomic_work() {
    let [first, second] = non_cardinal_seam_mixed_folded_support_pair(Frame::world());
    let windows = non_cardinal_seam_mixed_folded_support_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK);
    let result = exact.result().unwrap();
    assert_eq!(result.branch_graph.edges.len(), 6);
    assert_eq!(result.skew_cylinder_folded_support_curves().len(), 1);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn non_cardinal_seam_mixed_shifted_axial_charts_publish() {
    let frame = Frame::world();
    let second_axis = frame.x() * 0.8 - frame.y() * 0.6;
    let second_radial = frame.x() * 0.6 + frame.y() * 0.8;
    let first_radius = 0.078125;
    let second_radius = 0.0390625;
    let second_axis_offset = 0.21875;
    let first = Cylinder::new(frame, first_radius).unwrap();
    let second = Cylinder::new(
        Frame::new(
            frame.origin() + second_radial * second_radius - second_axis * second_axis_offset
                + frame.z() * (2.0 * second_radius),
            second_axis,
            second_radial,
        )
        .unwrap(),
        second_radius,
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 4.0 * second_radius)),
        cylinder_window(range(0.0, 2.0 * second_axis_offset)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let result = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    assert!(result.raw.is_complete(), "{result:#?}");
    assert_eq!(result.branch_graph.edges.len(), 6);
}

#[test]
fn four_simple_contact_cycle_publishes_two_folded_support_components() {
    let rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = perpendicular_axis_pair(frame, 0.0, 0.03125);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (direction, result, sources) in [
            ("forward", &forward, [first_handle, second_handle]),
            ("reversed", &reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(
                result.raw.is_complete(),
                "{name}/{direction}: {:#?}",
                result.raw
            );
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 6);
            assert_eq!(result.branch_graph.edges.len(), 6);
            assert_eq!(result.branch_graph.vertices.len(), 6);
            assert!(result.skew_cylinder_support_contacts().is_empty());
            assert!(result.skew_cylinder_touching_support_curves().is_empty());
            let folded = result.skew_cylinder_folded_support_curves();
            assert_eq!(folded.len(), 2);
            assert_eq!(
                folded
                    .iter()
                    .map(|component| component.certificate().topology().root_ordinals())
                    .collect::<Vec<_>>(),
                vec![[0, 3], [1, 2]]
            );
            assert_eq!(
                folded
                    .iter()
                    .map(|component| component.certificate().topology().positive_cell())
                    .collect::<Vec<_>>(),
                vec![
                    kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam,
                    kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots,
                ]
            );
            assert_eq!(
                folded
                    .iter()
                    .map(|component| component.certificate().formula_residuals().len())
                    .collect::<Vec<_>>(),
                vec![4, 2]
            );
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportJoin { .. }
                    ))
                    .count(),
                4
            );
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportSeamJoin { .. }
                    ))
                    .count(),
                2
            );
            for edge in &result.branch_graph.edges {
                assert_eq!(edge.topology, IntersectionBranchTopology::Open);
                assert!(edge.certificate.as_skew_cylinder_folded_support().is_some());
            }
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn four_simple_folded_support_components_own_atomic_combined_work() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 0.0, 0.03125);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let exact_work =
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK + SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK;
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(exact_work);
    let result = exact.result().unwrap();
    assert_eq!(result.branch_graph.edges.len(), 6);
    assert_eq!(result.skew_cylinder_folded_support_curves().len(), 2);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        exact_work
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(exact_work - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: exact_work,
        allowed: exact_work - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn authored_seam_support_contact_owns_existing_atomic_discriminant_work() {
    for offset in [3.0, -3.0] {
        let [first, second] = seam_perpendicular_axis_pair(Frame::world(), offset, 2.0);
        let windows = skew_windows();
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let session = SessionPolicy::v1();
        let tolerances = Tolerances::default();
        let run = |allowed| {
            let context = OperationContext::new(&session, tolerances)
                .unwrap()
                .with_budget_overrides(
                    BudgetPlan::new([LimitSpec::new(
                        SKEW_CYLINDER_DISCRIMINANT_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        allowed,
                    )])
                    .unwrap(),
                );
            intersect_bounded_graph_surfaces_with_context(
                &graph,
                first_handle,
                windows[0],
                second_handle,
                windows[1],
                &context,
            )
        };

        let exact = run(SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK);
        assert_eq!(
            exact
                .result()
                .unwrap()
                .skew_cylinder_support_contacts()
                .len(),
            1,
            "offset={offset}"
        );
        assert_eq!(
            observed_work(exact.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
            SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
        );
        assert!(exact.report().limit_events().is_empty());

        let denied = run(SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1);
        let expected = LimitSnapshot {
            stage: SKEW_CYLINDER_DISCRIMINANT_WORK,
            resource: ResourceKind::Work,
            consumed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
            allowed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1,
        };
        assert!(matches!(
            denied.result(),
            Err(GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(snapshot)
            )) if *snapshot == expected
        ));
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(
            observed_work(denied.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
            0
        );
    }
}

#[test]
fn folded_support_contact_owns_atomic_existing_open_span_work() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 3.0_f64.next_down(), 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                SKEW_CYLINDER_OPEN_SPAN_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
            )])
            .unwrap(),
        );
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &exact_context,
    );
    assert_folded_support_result(exact.result().unwrap(), [first_handle, second_handle]);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                SKEW_CYLINDER_OPEN_SPAN_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
            )])
            .unwrap(),
        );
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn seam_folded_support_publishes_four_members_and_four_exact_joins() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_perpendicular_axis_pair(frame, 3.0_f64.next_down(), 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one seam-folded component")
            };
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            let root_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportJoin { .. }
                    )
                })
                .count();
            let seam_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportSeamJoin { .. }
                    )
                })
                .count();
            assert_eq!((root_joins, seam_joins), (2, 2));
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn seam_folded_support_owns_atomic_four_member_work() {
    let [first, second] = seam_perpendicular_axis_pair(Frame::world(), 3.0_f64.next_down(), 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn seam_root_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_root_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one seam-root folded component")
            };
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            );
            let root_ordinals = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            let chart_sheets = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportChartJoin { sheet } => Some(sheet),
                    _ => None,
                })
                .collect::<Vec<_>>();
            assert_eq!(root_ordinals, vec![0, 1]);
            assert_eq!(
                chart_sheets,
                vec![SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
            );
            assert!(
                result
                    .branch_graph
                    .edges
                    .iter()
                    .flat_map(|edge| edge.endpoint_events)
                    .all(|event| matches!(
                        event,
                        IntersectionBranchEndpointEvent::FoldedSupportJoin { .. }
                            | IntersectionBranchEndpointEvent::FoldedSupportChartJoin { .. }
                    ))
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn seam_root_across_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_root_across_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one across-seam pole-pair folded component")
            };
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(3.0 * core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            assert!(folded.certificate().seam_points().is_none());
            let mut root_ordinals = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            let chart_sheets = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportChartJoin { sheet } => Some(sheet),
                    _ => None,
                })
                .collect::<Vec<_>>();
            root_ordinals.sort_unstable();
            assert_eq!(root_ordinals, vec![0, 1]);
            assert_eq!(
                chart_sheets,
                vec![SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
            );
            assert!(
                result
                    .branch_graph
                    .edges
                    .iter()
                    .flat_map(|edge| edge.endpoint_events)
                    .all(|event| matches!(
                        event,
                        IntersectionBranchEndpointEvent::FoldedSupportJoin { .. }
                            | IntersectionBranchEndpointEvent::FoldedSupportChartJoin { .. }
                    ))
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn short_seam_root_folded_support_publishes_two_root_joined_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = short_seam_root_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        assert_folded_support_result(&forward, [first_handle, second_handle]);
        assert_folded_support_result(&reversed, [second_handle, first_handle]);
        for result in [&forward, &reversed] {
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one short seam-root folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            );
            assert_eq!(folded.certificate().formula_residuals().len(), 2);
            assert_eq!(folded.certificate().chart_join_longitude(), None);
            assert!(folded.certificate().seam_points().is_none());
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn short_seam_root_folded_support_owns_atomic_two_member_work() {
    let [first, second] = short_seam_root_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert_folded_support_result(exact.result().unwrap(), [first_handle, second_handle]);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn short_seam_root_across_folded_support_publishes_two_root_joined_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = short_seam_root_across_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        assert_folded_support_result_in_root_order(&forward, [first_handle, second_handle], [1, 0]);
        assert_folded_support_result_in_root_order(
            &reversed,
            [second_handle, first_handle],
            [1, 0],
        );
        for result in [&forward, &reversed] {
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one short across-seam folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(
                angular[1].lo > core::f64::consts::PI && angular[1].hi < core::f64::consts::TAU
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            assert_eq!(folded.certificate().formula_residuals().len(), 2);
            assert_eq!(folded.certificate().chart_join_longitude(), None);
            assert!(folded.certificate().seam_points().is_none());
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn short_seam_root_across_folded_support_owns_atomic_two_member_work() {
    let [first, second] = short_seam_root_across_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert_folded_support_result_in_root_order(
        exact.result().unwrap(),
        [first_handle, second_handle],
        [1, 0],
    );
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn long_seam_root_across_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = long_seam_root_across_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one long across-seam folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(3.0 * core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            assert!(folded.certificate().seam_points().is_none());
            let mut roots = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            roots.sort_unstable();
            assert_eq!(roots, vec![0, 1]);
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn long_seam_root_across_folded_support_owns_atomic_subdivided_work() {
    let [first, second] = long_seam_root_across_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn long_seam_root_between_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = long_seam_root_between_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one long between-roots folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(angular[1].lo > core::f64::consts::PI);
            assert!(angular[1].hi < core::f64::consts::TAU);
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            );
            assert!(folded.certificate().seam_points().is_none());
            let mut roots = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            roots.sort_unstable();
            assert_eq!(roots, vec![0, 1]);
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn long_seam_root_between_folded_support_owns_atomic_subdivided_work() {
    let [first, second] = long_seam_root_between_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0
    );
}

#[test]
fn seam_root_folded_support_owns_atomic_four_member_work() {
    for [first, second] in [
        seam_root_folded_support_pair(Frame::world()),
        seam_root_across_folded_support_pair(Frame::world()),
    ] {
        let windows = skew_windows();
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let session = SessionPolicy::v1();
        let tolerances = Tolerances::default();
        let run = |allowed| {
            let context = OperationContext::new(&session, tolerances)
                .unwrap()
                .with_budget_overrides(
                    BudgetPlan::new([LimitSpec::new(
                        SKEW_CYLINDER_OPEN_SPAN_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        allowed,
                    )])
                    .unwrap(),
                );
            intersect_bounded_graph_surfaces_with_context(
                &graph,
                first_handle,
                windows[0],
                second_handle,
                windows[1],
                &context,
            )
        };

        let exact = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
        assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
        assert_eq!(
            observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
        );
        assert!(exact.report().limit_events().is_empty());

        let denied = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1);
        let expected = LimitSnapshot {
            stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
            resource: ResourceKind::Work,
            consumed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
            allowed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
        };
        assert!(matches!(
            denied.result(),
            Err(GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(snapshot)
            )) if *snapshot == expected
        ));
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(
            observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
            0
        );
    }
}
