use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};

use super::*;
use crate::{
    BodyId, CylinderRequest, Kernel, PartId, SectionBodiesRequest, SectionCompletion, Session,
};

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

struct Fixture {
    session: Session,
    part: PartId,
    outer: BodyId,
    inner: BodyId,
}

fn shared_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => Frame::new(
            Point3::new(2.5, -1.75, 0.625),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    }
}

fn fixture_with_axial_intervals(
    placement: Placement,
    first: (f64, f64),
    second: (f64, f64),
) -> Fixture {
    fixture_with_axial_intervals_and_parity(placement, first, second, false)
}

fn fixture_with_axial_intervals_and_parity(
    placement: Placement,
    first: (f64, f64),
    second: (f64, f64),
    reverse_second_axis: bool,
) -> Fixture {
    let frame = shared_frame(placement);
    let second_frame = if reverse_second_axis {
        Frame::new(
            frame.point_at(0.5, 0.0, second.0 + second.1),
            -frame.z(),
            frame.x(),
        )
        .unwrap()
    } else {
        frame.with_origin(frame.point_at(0.5, 0.0, second.0))
    };
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let outer = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(-0.5, 0.0, first.0)),
                1.0,
                first.1,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let inner = edit
            .create_cylinder(CylinderRequest::new(second_frame, 1.0, second.1))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (outer, inner)
    };
    Fixture {
        session,
        part,
        outer,
        inner,
    }
}

fn fixture(placement: Placement) -> Fixture {
    fixture_with_axial_intervals(placement, (-2.0, 4.0), (-1.0, 2.0))
}

fn antiparallel_nested_fixture(placement: Placement) -> Fixture {
    fixture_with_axial_intervals_and_parity(placement, (-2.0, 4.0), (-1.0, 2.0), true)
}

fn partial_overlap_fixture(placement: Placement) -> Fixture {
    fixture_with_axial_intervals(placement, (-1.0, 2.0), (0.0, 2.0))
}

fn antiparallel_partial_overlap_fixture(placement: Placement) -> Fixture {
    fixture_with_axial_intervals_and_parity(placement, (-1.0, 2.0), (0.0, 2.0), true)
}

fn section(fixture: &Fixture, swapped: bool) -> BodySectionGraph {
    let (first, second) = if swapped {
        (fixture.inner.clone(), fixture.outer.clone())
    } else {
        (fixture.outer.clone(), fixture.inner.clone())
    };
    fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first, second))
        .unwrap()
        .into_result()
        .unwrap()
}

fn extract_source(fixture: &Fixture, body: &BodyId) -> CertifiedCylinderSource {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let context = OperationContext::new(part.policy(), Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    match super::super::curved_source::extract_cylinder_source(
        &part.state.store,
        body.raw(),
        &mut scope,
    )
    .unwrap()
    {
        super::super::curved_source::CylinderSourceOutcome::Ready(source) => source,
        other => panic!("unexpected cylinder source outcome: {other:?}"),
    }
}

fn sources(fixture: &Fixture, swapped: bool) -> [CertifiedCylinderSource; 2] {
    let outer = extract_source(fixture, &fixture.outer);
    let inner = extract_source(fixture, &fixture.inner);
    if swapped {
        [inner, outer]
    } else {
        [outer, inner]
    }
}

fn certify(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    sources: &[CertifiedCylinderSource; 2],
    allowed: u64,
) -> Result<ParallelCylinderRelationOutcome> {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let overrides = BudgetPlan::new([LimitSpec::new(
        PLANAR_BOOLEAN_BSP_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    let context = OperationContext::new(part.policy(), Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults())
        .with_budget_overrides(overrides);
    let mut scope = OperationScope::new(&context);
    certify_parallel_cylinder_relation(graph, [&sources[0], &sources[1]], &mut scope)
}

fn certified(outcome: ParallelCylinderRelationOutcome) -> CertifiedParallelCylinderLensRelation {
    match outcome {
        ParallelCylinderRelationOutcome::Certified(certificate) => *certificate,
        other => panic!("expected certified relation, got {other:?}"),
    }
}

fn certified_relation(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    sources: &[CertifiedCylinderSource; 2],
) -> CertifiedParallelCylinderLensRelation {
    certified(certify(fixture, graph, sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap())
}

fn certified_coincident_caps(
    outcome: ParallelCylinderRelationOutcome,
) -> CertifiedParallelCylinderCoincidentCapRelation {
    match outcome {
        ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(certificate) => *certificate,
        other => panic!("expected certified coincident-cap relation, got {other:?}"),
    }
}

fn assert_coincident_cap_relation(
    fixture: &Fixture,
    swapped: bool,
    expected_shared_ends: usize,
    context: &str,
) {
    let graph = section(fixture, swapped);
    let replay = section(fixture, swapped);
    let sources = sources(fixture, swapped);
    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    let relation_outcome =
        certify(fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap();
    if !matches!(
        relation_outcome,
        ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(_)
    ) {
        panic!(
            "{context}: unexpected coincident-cap outcome {relation_outcome:?}; completion={:?}; gaps={:#?}",
            graph.completion(),
            graph.gaps()
        );
    }
    let relation = certified_coincident_caps(relation_outcome);
    let replay_relation = certified_coincident_caps(
        certify(fixture, &replay, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
    );
    assert_eq!(relation, replay_relation);
    assert_eq!(
        relation
            .overlap_ends()
            .iter()
            .filter(|end| end.is_shared())
            .count(),
        expected_shared_ends
    );
    assert_eq!(relation.unique_end_count(), 2 - expected_shared_ends);
    for (physical_end, end) in relation.overlap_ends().iter().enumerate() {
        assert_eq!(
            end.sources().iter().flatten().count() + usize::from(end.cap_arc().is_some()),
            2
        );
        for source in end.sources().iter().flatten() {
            assert!(source.operand() < 2);
            assert!(source.boundary() < 2);
            assert_eq!(
                source.cap_face(),
                sources[source.operand()].boundaries()[source.boundary()].cap_face()
            );
            assert_eq!(
                source.edge(),
                sources[source.operand()].boundaries()[source.boundary()].edge()
            );
            assert_eq!(source.roots().map(|root| root.root_ordinal()), [0, 1]);
            for root in source.roots() {
                assert!(root.endpoint() < graph.curve_endpoints().len());
                assert!(root.enclosure()[0] <= root.parameter());
                assert!(root.parameter() <= root.enclosure()[1]);
            }
        }
        if let Some(arc) = end.cap_arc() {
            assert!(arc.branch() < graph.branches().len());
            assert!(arc.fragment() < graph.curve_fragments().len());
            assert!(arc.endpoints().into_iter().all(|endpoint| endpoint < 4));
        }
        for ruling in relation.rulings() {
            assert!(ruling.endpoints()[physical_end] < graph.curve_endpoints().len());
        }
    }
    for ruling in relation.rulings() {
        assert!(ruling.branch() < graph.branches().len());
        assert!(ruling.fragment() < graph.curve_fragments().len());
        assert_ne!(ruling.endpoints()[0], ruling.endpoints()[1]);
    }
}

#[test]
fn coincident_cap_relations_certify_equal_and_shared_end_graphs_generically() {
    for placement in [Placement::World, Placement::Oblique] {
        for reverse_second_axis in [false, true] {
            let equal = fixture_with_axial_intervals_and_parity(
                placement,
                (-1.0, 2.0),
                (-1.0, 2.0),
                reverse_second_axis,
            );
            assert_coincident_cap_relation(&equal, false, 2, "equal forward");
            assert_coincident_cap_relation(&equal, true, 2, "equal swapped");

            let shared = fixture_with_axial_intervals_and_parity(
                placement,
                (-1.0, 3.0),
                (-1.0, 2.0),
                reverse_second_axis,
            );
            assert_coincident_cap_relation(&shared, false, 1, "shared-lower forward");
            assert_coincident_cap_relation(&shared, true, 1, "shared-lower swapped");

            let shared_upper = fixture_with_axial_intervals_and_parity(
                placement,
                (-2.0, 4.0),
                (-1.0, 3.0),
                reverse_second_axis,
            );
            let context = format!("shared-upper {placement:?} antiparallel={reverse_second_axis}");
            assert_coincident_cap_relation(&shared_upper, false, 1, &context);
            assert_coincident_cap_relation(&shared_upper, true, 1, &format!("{context} swapped"));
        }
    }
}

#[test]
fn strict_world_and_oblique_relations_are_replay_and_swap_deterministic() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = fixture(placement);
        let forward_graph = section(&fixture, false);
        let replay_graph = section(&fixture, false);
        let swapped_graph = section(&fixture, true);
        let forward_sources = sources(&fixture, false);
        let swapped_sources = sources(&fixture, true);

        let forward = certified_relation(&fixture, &forward_graph, &forward_sources);
        let replay = certified_relation(&fixture, &replay_graph, &forward_sources);
        let swapped = certified_relation(&fixture, &swapped_graph, &swapped_sources);
        assert_eq!(forward, replay);
        assert_eq!(forward.strict_nesting_operands(), Some([1, 0]));
        assert_eq!(swapped.strict_nesting_operands(), Some([0, 1]));
        assert_eq!(forward.component(), 0);
        assert_eq!(swapped.component(), 0);
        assert_eq!(
            forward.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals()
            )),
            swapped.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals()
            ))
        );
        assert_eq!(
            forward.rulings().map(|witness| witness.root_ordinals()),
            swapped.rulings().map(|witness| witness.root_ordinals())
        );
        for (boundary, witness) in forward.overlap_ends().iter().enumerate() {
            assert_eq!(witness.operand(), 1);
            assert_eq!(witness.boundary(), boundary);
            assert_eq!(witness.root_ordinals(), [0, 1]);
            assert!(witness.branch() < forward_graph.branches().len());
            assert!(witness.fragment() < forward_graph.curve_fragments().len());
        }
        for witness in forward.rulings() {
            assert!(witness.branch() < forward_graph.branches().len());
            assert!(witness.fragment() < forward_graph.curve_fragments().len());
            assert!(witness.endpoints().into_iter().all(|endpoint| endpoint < 4));
        }
    }
}

#[test]
fn antiparallel_strict_nesting_normalizes_world_oblique_replay_and_swap() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = antiparallel_nested_fixture(placement);
        let forward_graph = section(&fixture, false);
        let replay_graph = section(&fixture, false);
        let swapped_graph = section(&fixture, true);
        let forward_sources = sources(&fixture, false);
        let swapped_sources = sources(&fixture, true);

        let forward = certified_relation(&fixture, &forward_graph, &forward_sources);
        let replay = certified_relation(&fixture, &replay_graph, &forward_sources);
        let swapped = certified_relation(&fixture, &swapped_graph, &swapped_sources);

        assert_eq!(forward, replay);
        assert_eq!(forward.strict_nesting_operands(), Some([1, 0]));
        assert_eq!(swapped.strict_nesting_operands(), Some([0, 1]));
        assert_eq!(forward.component(), 0);
        assert_eq!(swapped.component(), 0);
        assert_eq!(
            forward.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals(),
            )),
            swapped.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals(),
            ))
        );
        assert_eq!(
            forward.overlap_ends().map(|witness| (
                witness.operand(),
                witness.boundary(),
                witness.root_ordinals(),
            )),
            [(1, 1, [0, 1]), (1, 0, [0, 1])]
        );
        assert_eq!(
            swapped.overlap_ends().map(|witness| (
                witness.operand(),
                witness.boundary(),
                witness.root_ordinals(),
            )),
            [(0, 1, [0, 1]), (0, 0, [0, 1])]
        );
        assert_eq!(
            forward.rulings().map(|witness| witness.root_ordinals()),
            swapped.rulings().map(|witness| witness.root_ordinals())
        );
        for witness in forward.rulings() {
            assert!(witness.branch() < forward_graph.branches().len());
            assert!(witness.fragment() < forward_graph.curve_fragments().len());
            assert!(witness.endpoints().into_iter().all(|endpoint| endpoint < 4));
        }
    }
}

#[test]
fn partial_overlap_ends_are_physical_replay_and_swap_deterministic() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = partial_overlap_fixture(placement);
        let forward_graph = section(&fixture, false);
        let replay_graph = section(&fixture, false);
        let swapped_graph = section(&fixture, true);
        let forward_sources = sources(&fixture, false);
        let swapped_sources = sources(&fixture, true);

        let forward = certified_relation(&fixture, &forward_graph, &forward_sources);
        let replay = certified_relation(&fixture, &replay_graph, &forward_sources);
        let swapped = certified_relation(&fixture, &swapped_graph, &swapped_sources);

        assert_eq!(forward, replay);
        assert_eq!(forward.strict_nesting_operands(), None);
        assert_eq!(swapped.strict_nesting_operands(), None);
        assert_eq!(
            forward.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals(),
            )),
            swapped.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals(),
            ))
        );
        assert_eq!(
            forward
                .overlap_ends()
                .map(|witness| (witness.operand(), witness.boundary())),
            [(1, 0), (0, 1)]
        );
        assert_eq!(
            swapped
                .overlap_ends()
                .map(|witness| (witness.operand(), witness.boundary())),
            [(0, 0), (1, 1)]
        );
        assert_eq!(
            forward.rulings().map(|witness| witness.root_ordinals()),
            swapped.rulings().map(|witness| witness.root_ordinals())
        );
        assert!(
            forward
                .overlap_ends()
                .iter()
                .all(|witness| witness.root_ordinals() == [0, 1])
        );
    }
}

#[test]
fn antiparallel_partial_overlap_normalizes_world_oblique_and_swap() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = antiparallel_partial_overlap_fixture(placement);
        let forward_graph = section(&fixture, false);
        let replay_graph = section(&fixture, false);
        let swapped_graph = section(&fixture, true);
        let forward_sources = sources(&fixture, false);
        let swapped_sources = sources(&fixture, true);

        let forward = certified_relation(&fixture, &forward_graph, &forward_sources);
        let replay = certified_relation(&fixture, &replay_graph, &forward_sources);
        let swapped = certified_relation(&fixture, &swapped_graph, &swapped_sources);

        assert_eq!(forward, replay);
        assert_eq!(forward.strict_nesting_operands(), None);
        assert_eq!(swapped.strict_nesting_operands(), None);
        assert_eq!(
            forward.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals(),
            )),
            swapped.overlap_ends().map(|witness| (
                witness.boundary(),
                witness.cap_face(),
                witness.edge(),
                witness.root_ordinals(),
            ))
        );
        assert_eq!(
            forward
                .overlap_ends()
                .map(|witness| (witness.operand(), witness.boundary())),
            [(1, 1), (0, 1)]
        );
        assert_eq!(
            swapped
                .overlap_ends()
                .map(|witness| (witness.operand(), witness.boundary())),
            [(0, 1), (1, 1)]
        );
        assert_eq!(
            forward.rulings().map(|witness| witness.root_ordinals()),
            swapped.rulings().map(|witness| witness.root_ordinals())
        );
    }
}

#[test]
fn relation_work_accepts_exact_n_and_refuses_n_minus_one() {
    for fixture in [
        fixture(Placement::World),
        antiparallel_nested_fixture(Placement::World),
    ] {
        let graph = section(&fixture, false);
        let sources = sources(&fixture, false);
        assert!(matches!(
            certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
            ParallelCylinderRelationOutcome::Certified(_)
        ));

        let error = certify(
            &fixture,
            &graph,
            &sources,
            PARALLEL_CYLINDER_RELATION_WORK - 1,
        )
        .unwrap_err();
        let snapshot = error
            .limit()
            .expect("relation must retain exact limit evidence");
        assert_eq!(snapshot.stage, PLANAR_BOOLEAN_BSP_WORK);
        assert_eq!(snapshot.resource, ResourceKind::Work);
        assert_eq!(snapshot.consumed, PARALLEL_CYLINDER_RELATION_WORK);
        assert_eq!(snapshot.allowed, PARALLEL_CYLINDER_RELATION_WORK - 1);
    }
}

#[test]
fn incomplete_layout_binding_and_endpoint_failures_are_typed() {
    let fixture = fixture(Placement::World);
    let graph = section(&fixture, false);
    let sources = sources(&fixture, false);

    let mut incomplete = graph.clone();
    incomplete.completion = SectionCompletion::Indeterminate;
    assert_eq!(
        certify(
            &fixture,
            &incomplete,
            &sources,
            PARALLEL_CYLINDER_RELATION_WORK,
        )
        .unwrap(),
        ParallelCylinderRelationOutcome::Indeterminate(
            ParallelCylinderRelationGap::SectionIncomplete,
        )
    );

    let mut truncated = graph.clone();
    truncated.curve_fragments.pop();
    assert_eq!(
        certify(
            &fixture,
            &truncated,
            &sources,
            PARALLEL_CYLINDER_RELATION_WORK,
        )
        .unwrap(),
        ParallelCylinderRelationOutcome::Indeterminate(ParallelCylinderRelationGap::SectionLayout,)
    );

    let reversed_sources = [sources[1].clone(), sources[0].clone()];
    assert_eq!(
        certify(
            &fixture,
            &graph,
            &reversed_sources,
            PARALLEL_CYLINDER_RELATION_WORK,
        )
        .unwrap(),
        ParallelCylinderRelationOutcome::Indeterminate(
            ParallelCylinderRelationGap::SectionOperandBinding,
        )
    );

    let mut mismatched_endpoints = graph;
    mismatched_endpoints.curve_endpoints.swap(0, 1);
    assert_eq!(
        certify(
            &fixture,
            &mismatched_endpoints,
            &sources,
            PARALLEL_CYLINDER_RELATION_WORK,
        )
        .unwrap(),
        ParallelCylinderRelationOutcome::Indeterminate(
            ParallelCylinderRelationGap::SectionEndpointProvenance,
        )
    );
}

fn analytic_sources(
    first_frame: Frame,
    first_height: f64,
    second_frame: Frame,
    second_height: f64,
) -> [CertifiedCylinderSource; 2] {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(first_frame, 1.0, first_height))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 1.0, second_height))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let part = session.part(part_id).unwrap();
    let context = OperationContext::new(part.policy(), Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    [first, second].map(|body| {
        match super::super::curved_source::extract_cylinder_source(
            &part.state.store,
            body.raw(),
            &mut scope,
        )
        .unwrap()
        {
            super::super::curved_source::CylinderSourceOutcome::Ready(source) => source,
            other => panic!("unexpected source extraction: {other:?}"),
        }
    })
}

#[test]
fn analytic_boundary_cases_have_distinct_typed_gaps() {
    let world = Frame::world();
    let equal = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
        2.0,
        world.with_origin(Point3::new(0.5, 0.0, -1.0)),
        2.0,
    );
    assert_eq!(
        certify_source_relation([&equal[0], &equal[1]]),
        Ok([
            SourceOverlapEnd {
                operand: 0,
                boundary: 0,
                peer_boundary: Some(0),
            },
            SourceOverlapEnd {
                operand: 0,
                boundary: 1,
                peer_boundary: Some(1),
            },
        ])
    );

    let partial = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
        2.0,
        world.with_origin(Point3::new(0.5, 0.0, 0.0)),
        2.0,
    );
    assert_eq!(
        certify_source_relation([&partial[0], &partial[1]]),
        Ok([
            SourceOverlapEnd {
                operand: 1,
                boundary: 0,
                peer_boundary: None,
            },
            SourceOverlapEnd {
                operand: 0,
                boundary: 1,
                peer_boundary: None,
            },
        ])
    );

    for second_low in [0.0, 0.25] {
        let no_overlap = analytic_sources(
            world.with_origin(Point3::new(-0.5, 0.0, -2.0)),
            2.0,
            world.with_origin(Point3::new(0.5, 0.0, second_low)),
            2.0,
        );
        assert_eq!(
            certify_source_relation([&no_overlap[0], &no_overlap[1]]),
            Err(ParallelCylinderRelationGap::AxialOverlapNotStrictlyPositive)
        );
    }

    let shared_low = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
        3.0,
        world.with_origin(Point3::new(0.5, 0.0, -1.0)),
        2.0,
    );
    assert_eq!(
        certify_source_relation([&shared_low[0], &shared_low[1]]),
        Ok([
            SourceOverlapEnd {
                operand: 0,
                boundary: 0,
                peer_boundary: Some(0),
            },
            SourceOverlapEnd {
                operand: 1,
                boundary: 1,
                peer_boundary: None,
            },
        ])
    );

    let tangent = analytic_sources(
        world.with_origin(Point3::new(-1.0, 0.0, -2.0)),
        4.0,
        world.with_origin(Point3::new(1.0, 0.0, -1.0)),
        2.0,
    );
    assert_eq!(
        certify_source_relation([&tangent[0], &tangent[1]]),
        Err(ParallelCylinderRelationGap::RadialSecancyNotStrict)
    );

    let reversed = Frame::new(Point3::new(0.5, 0.0, 1.0), -world.z(), world.x()).unwrap();
    let opposed = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -2.0)),
        4.0,
        reversed,
        2.0,
    );
    assert_eq!(
        certify_source_relation([&opposed[0], &opposed[1]]),
        Ok([
            SourceOverlapEnd {
                operand: 1,
                boundary: 1,
                peer_boundary: None,
            },
            SourceOverlapEnd {
                operand: 1,
                boundary: 0,
                peer_boundary: None,
            },
        ])
    );

    let antiparallel_equal = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
        2.0,
        reversed,
        2.0,
    );
    assert_eq!(
        certify_source_relation([&antiparallel_equal[0], &antiparallel_equal[1]]),
        Ok([
            SourceOverlapEnd {
                operand: 0,
                boundary: 0,
                peer_boundary: Some(1),
            },
            SourceOverlapEnd {
                operand: 0,
                boundary: 1,
                peer_boundary: Some(0),
            },
        ])
    );

    let antiparallel_shared_low = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
        3.0,
        reversed,
        2.0,
    );
    assert_eq!(
        certify_source_relation([&antiparallel_shared_low[0], &antiparallel_shared_low[1],]),
        Ok([
            SourceOverlapEnd {
                operand: 0,
                boundary: 0,
                peer_boundary: Some(1),
            },
            SourceOverlapEnd {
                operand: 1,
                boundary: 0,
                peer_boundary: None,
            },
        ])
    );

    let near_opposed_frame = Frame::new(
        Point3::new(0.5, 0.0, 1.0),
        Vec3::new(0.0, 1.0e-12, -1.0),
        world.x(),
    )
    .unwrap();
    let near_opposed = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -2.0)),
        4.0,
        near_opposed_frame,
        2.0,
    );
    assert_eq!(
        certify_source_relation([&near_opposed[0], &near_opposed[1]]),
        Err(ParallelCylinderRelationGap::AxesNotExactlyParallel)
    );

    let skew = Frame::new(
        Point3::new(0.5, 0.0, -1.0),
        Vec3::new(0.0, 1.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let nonparallel = analytic_sources(
        world.with_origin(Point3::new(-0.5, 0.0, -2.0)),
        4.0,
        skew,
        2.0,
    );
    assert_eq!(
        certify_source_relation([&nonparallel[0], &nonparallel[1]]),
        Err(ParallelCylinderRelationGap::AxesNotExactlyParallel)
    );
}
