use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
};
use kcore::tolerance::Tolerances;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3, Vec3};

use super::super::axial_interval_sweep::plan_axial_interval_sweep;
use super::super::boundary_select::RegularizedBooleanOperation;
use super::*;
use crate::{
    BodyId, CylinderRequest, FaceId, Kernel, PartId, SectionBodiesRequest, SectionCompletion,
    Session,
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

#[derive(Debug, Clone, Copy)]
enum CapPlanePerturbation {
    Shift,
    Tilt,
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
    fixture_with_geometry(
        placement,
        first,
        second,
        reverse_second_axis,
        [-0.5, 0.5],
        [1.0, 1.0],
    )
}

fn fixture_with_geometry(
    placement: Placement,
    first: (f64, f64),
    second: (f64, f64),
    reverse_second_axis: bool,
    radial_offsets: [f64; 2],
    radii: [f64; 2],
) -> Fixture {
    let frame = shared_frame(placement);
    let second_frame = if reverse_second_axis {
        Frame::new(
            frame.point_at(radial_offsets[1], 0.0, second.0 + second.1),
            -frame.z(),
            frame.x(),
        )
        .unwrap()
    } else {
        frame.with_origin(frame.point_at(radial_offsets[1], 0.0, second.0))
    };
    fixture_with_frames(
        frame.with_origin(frame.point_at(radial_offsets[0], 0.0, first.0)),
        first.1,
        radii[0],
        second_frame,
        second.1,
        radii[1],
    )
}

fn fixture_with_frames(
    first_frame: Frame,
    first_height: f64,
    first_radius: f64,
    second_frame: Frame,
    second_height: f64,
    second_radius: f64,
) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let outer = edit
            .create_cylinder(CylinderRequest::new(
                first_frame,
                first_radius,
                first_height,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let inner = edit
            .create_cylinder(CylinderRequest::new(
                second_frame,
                second_radius,
                second_height,
            ))
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

fn exact_axial_contact_fixture(
    placement: Placement,
    reverse_second_axis: bool,
    radial_scale: f64,
    radii: [f64; 2],
) -> Fixture {
    let frame = shared_frame(placement).with_origin(Point3::new(0.0, 0.0, 0.0));
    let axis = frame.z();
    let exact_perpendicular = if axis.x == 0.0 && axis.y == 0.0 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(axis.y, -axis.x, 0.0)
    };
    let radial = exact_perpendicular * radial_scale;
    let first_frame = frame.with_origin(Point3::new(-axis.x, -axis.y, -axis.z));
    let second_low = Point3::new(radial.x, radial.y, radial.z);
    let second_frame = if reverse_second_axis {
        Frame::new(second_low + axis, -axis, frame.x()).unwrap()
    } else {
        frame.with_origin(second_low)
    };
    fixture_with_frames(first_frame, 1.0, radii[0], second_frame, 1.0, radii[1])
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

fn axially_separated_fixture(placement: Placement, reverse_second_axis: bool) -> Fixture {
    fixture_with_geometry(
        placement,
        (-2.0, 2.0),
        (1.0, 2.0),
        reverse_second_axis,
        [-0.5, 0.5],
        [1.0, 1.0],
    )
}

fn common_support_fixture(
    first: (f64, f64),
    second: (f64, f64),
    reverse_second_axis: bool,
) -> Fixture {
    fixture_with_geometry(
        Placement::World,
        first,
        second,
        reverse_second_axis,
        [0.0, 0.0],
        [1.0, 1.0],
    )
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

fn perturb_cap_plane_within_full_tolerance(
    fixture: &mut Fixture,
    body: &BodyId,
    boundary: usize,
    perturbation: CapPlanePerturbation,
) {
    let source = extract_source(fixture, body);
    let cap_face = source.boundaries()[boundary].cap_face();
    let raw_body = body.raw();
    let linear = Tolerances::default().linear();
    let mut edit = fixture.session.edit_part(fixture.part.clone()).unwrap();
    let store = edit.store_mut_for_test();
    let face = store.get(cap_face).unwrap();
    let surface = face.surface();
    let SurfaceGeom::Plane(plane) = store.surface(surface).unwrap() else {
        panic!("cylinder cap must remain planar");
    };
    let frame = *plane.frame();
    let perturbed_frame = match perturbation {
        CapPlanePerturbation::Shift => {
            frame.with_origin(frame.origin() + frame.z() * (linear * 0.125))
        }
        CapPlanePerturbation::Tilt => Frame::new(
            frame.origin(),
            frame.z() + frame.x() * (linear * 0.0625),
            frame.x(),
        )
        .unwrap(),
    };
    let mut transaction = store.transaction().unwrap();
    transaction
        .assembly()
        .replace_surface(surface, SurfaceGeom::Plane(Plane::new(perturbed_frame)))
        .unwrap();
    transaction.commit_checked_body(raw_body).unwrap();
}

fn perturb_side_height_within_full_tolerance(
    fixture: &mut Fixture,
    body: &BodyId,
    boundary: usize,
) {
    let source = extract_source(fixture, body);
    let side_fin = source.boundaries()[boundary].side_fin();
    let raw_body = body.raw();
    let linear = Tolerances::default().linear();
    let mut edit = fixture.session.edit_part(fixture.part.clone()).unwrap();
    let store = edit.store_mut_for_test();
    let pcurve = store
        .get(side_fin)
        .unwrap()
        .pcurve()
        .expect("cylinder side fin must retain a pcurve");
    let Curve2dGeom::Line(line) = store.pcurve(pcurve.curve()).unwrap() else {
        panic!("cylinder side boundary must remain a line pcurve");
    };
    let origin = line.origin();
    let shifted =
        Line2d::new(Point2::new(origin.x, origin.y - linear * 0.125), line.dir()).unwrap();
    let mut transaction = store.transaction().unwrap();
    transaction
        .assembly()
        .replace_pcurve(pcurve.curve(), Curve2dGeom::Line(shifted))
        .unwrap();
    transaction.commit_checked_body(raw_body).unwrap();
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
    certify_parallel_cylinder_relation(
        &part.state.store,
        graph,
        [&sources[0], &sources[1]],
        &mut scope,
    )
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

fn certified_axial_separation(
    outcome: ParallelCylinderRelationOutcome,
) -> CertifiedParallelCylinderAxialSeparation {
    match outcome {
        ParallelCylinderRelationOutcome::CertifiedAxialSeparation(certificate) => *certificate,
        other => panic!("expected certified axial separation, got {other:?}"),
    }
}

fn certified_axial_contact(
    outcome: ParallelCylinderRelationOutcome,
    context: &str,
) -> CertifiedParallelCylinderAxialContact {
    match outcome {
        ParallelCylinderRelationOutcome::CertifiedAxialContact(certificate) => *certificate,
        other => panic!("{context}: expected certified axial contact, got {other:?}"),
    }
}

fn certified_common_support(
    outcome: ParallelCylinderRelationOutcome,
) -> CertifiedParallelCylinderCommonSupport {
    match outcome {
        ParallelCylinderRelationOutcome::CertifiedCommonSupport(certificate) => *certificate,
        other => panic!("expected certified common support, got {other:?}"),
    }
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
fn coincident_cap_periodic_projection_excludes_boundary_overlays_and_rejects_hostile_subsets() {
    assert_eq!(
        crate::section::periodic_face_fragment_subset_work(0),
        Some(2)
    );
    assert_eq!(
        crate::section::periodic_face_fragment_subset_work(1),
        Some(3)
    );
    assert_eq!(
        crate::section::periodic_face_fragment_subset_work(2),
        Some(5)
    );
    assert_eq!(
        crate::section::periodic_face_fragment_subset_work(3),
        Some(8)
    );
    if usize::BITS == 64 {
        assert_eq!(
            crate::section::periodic_face_fragment_subset_work(usize::MAX),
            None
        );
    }

    for fixture in [
        fixture_with_axial_intervals(Placement::World, (-1.0, 2.0), (-1.0, 2.0)),
        fixture_with_axial_intervals(Placement::World, (-1.0, 3.0), (-1.0, 2.0)),
    ] {
        let graph = section(&fixture, false);
        let sources = sources(&fixture, false);
        let relation = certified_coincident_caps(
            certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
        );
        let shared_ends = relation
            .overlap_ends()
            .iter()
            .filter(|end| end.is_shared())
            .count();
        let selected_union = (0..2)
            .flat_map(|operand| relation.periodic_fragment_subset(operand))
            .collect::<std::collections::BTreeSet<_>>();
        let overlay_fragments = (0..graph.curve_fragments().len())
            .filter(|fragment| !selected_union.contains(fragment))
            .collect::<Vec<_>>();
        assert_eq!(overlay_fragments.len(), 2 * shared_ends);

        let part = fixture.session.part(fixture.part.clone()).unwrap();
        for operand in 0..2 {
            let face = FaceId::new(fixture.part.clone(), sources[operand].side_face());
            let selected = relation.periodic_fragment_subset(operand);
            assert!(selected.windows(2).all(|pair| pair[0] < pair[1]));
            assert_eq!(
                selected
                    .iter()
                    .copied()
                    .collect::<std::collections::BTreeSet<_>>(),
                selected_union
                    .iter()
                    .copied()
                    .filter(|fragment| {
                        graph.branches()[graph.curve_fragments()[*fragment].branch()].faces()
                            [operand]
                            == face
                    })
                    .collect(),
            );
            crate::section::certify_periodic_face_fragment_subset(
                &part.state.store,
                face.part(),
                &graph,
                operand,
                face.clone(),
                &selected,
                Tolerances::default().linear(),
            )
            .unwrap();

            let overlay = overlay_fragments
                .iter()
                .copied()
                .find(|fragment| {
                    graph.branches()[graph.curve_fragments()[*fragment].branch()].faces()[operand]
                        == face
                })
                .expect("every shared end contributes one side-carried boundary overlay");
            let mut contaminated = selected.clone();
            contaminated.push(overlay);
            assert!(
                crate::section::certify_periodic_face_fragment_subset(
                    &part.state.store,
                    face.part(),
                    &graph,
                    operand,
                    face.clone(),
                    &contaminated,
                    Tolerances::default().linear(),
                )
                .is_err(),
                "a boundary-coincident overlay must not be promoted to an interior side cut",
            );

            let duplicate = [selected[0], selected[0]];
            assert_eq!(
                crate::section::certify_periodic_face_fragment_subset(
                    &part.state.store,
                    face.part(),
                    &graph,
                    operand,
                    face.clone(),
                    &duplicate,
                    Tolerances::default().linear(),
                ),
                Err(crate::SectionPeriodicEmbeddingGap::UnstitchedFragmentPath {
                    fragment: selected[0],
                }),
            );

            let wrong_face = FaceId::new(
                fixture.part.clone(),
                sources[operand].boundaries()[0].cap_face(),
            );
            assert_eq!(
                crate::section::certify_periodic_face_fragment_subset(
                    &part.state.store,
                    wrong_face.part(),
                    &graph,
                    operand,
                    wrong_face.clone(),
                    &selected,
                    Tolerances::default().linear(),
                ),
                Err(crate::SectionPeriodicEmbeddingGap::UnstitchedFragmentPath {
                    fragment: selected[0],
                }),
            );
        }

        let face = FaceId::new(fixture.part.clone(), sources[0].side_face());
        assert_eq!(
            crate::section::certify_periodic_face_fragment_subset(
                &part.state.store,
                face.part(),
                &graph,
                2,
                face.clone(),
                &[],
                Tolerances::default().linear(),
            ),
            Err(crate::SectionPeriodicEmbeddingGap::SourceFaceTopology),
        );
    }
}

#[test]
fn strict_axial_separation_is_radial_independent_replay_and_swap_deterministic() {
    let radial_relations = [
        ("strict secant", [-0.5, 0.5], [1.0, 1.0]),
        ("tangent", [-1.0, 1.0], [1.0, 1.0]),
        ("strict internal", [0.0, 0.25], [2.0, 0.5]),
        ("coincident", [0.0, 0.0], [1.0, 1.0]),
    ];
    for placement in [Placement::World, Placement::Oblique] {
        for reverse_second_axis in [false, true] {
            for (radial_relation, offsets, radii) in radial_relations {
                let fixture = fixture_with_geometry(
                    placement,
                    (-2.0, 2.0),
                    (1.0, 2.0),
                    reverse_second_axis,
                    offsets,
                    radii,
                );
                let forward_graph = section(&fixture, false);
                let replay_graph = section(&fixture, false);
                let swapped_graph = section(&fixture, true);
                let forward_sources = sources(&fixture, false);
                let swapped_sources = sources(&fixture, true);

                let forward_outcome = certify(
                    &fixture,
                    &forward_graph,
                    &forward_sources,
                    PARALLEL_CYLINDER_RELATION_WORK,
                )
                .unwrap();
                let replay_outcome = certify(
                    &fixture,
                    &replay_graph,
                    &forward_sources,
                    PARALLEL_CYLINDER_RELATION_WORK,
                )
                .unwrap();
                let swapped_outcome = certify(
                    &fixture,
                    &swapped_graph,
                    &swapped_sources,
                    PARALLEL_CYLINDER_RELATION_WORK,
                )
                .unwrap();
                let forward = certified_axial_separation(forward_outcome);
                let replay = certified_axial_separation(replay_outcome);
                let swapped = certified_axial_separation(swapped_outcome);

                assert_eq!(forward, replay, "{placement:?} {radial_relation}");
                assert_eq!(
                    forward.gap_boundaries().map(|witness| (
                        witness.boundary(),
                        witness.cap_face(),
                        witness.edge(),
                    )),
                    swapped.gap_boundaries().map(|witness| (
                        witness.boundary(),
                        witness.cap_face(),
                        witness.edge(),
                    )),
                    "{placement:?} {radial_relation} antiparallel={reverse_second_axis}",
                );
                assert_eq!(
                    forward.gap_boundaries().map(|witness| witness.operand()),
                    [0, 1],
                );
                assert_eq!(
                    swapped.gap_boundaries().map(|witness| witness.operand()),
                    [1, 0],
                );
                assert_eq!(
                    forward.gap_boundaries().map(|witness| witness.boundary()),
                    [1, usize::from(reverse_second_axis)],
                );
                for witness in forward.gap_boundaries() {
                    let source = &forward_sources[witness.operand()];
                    let boundary = source.boundaries()[witness.boundary()];
                    assert_eq!(witness.cap_face(), boundary.cap_face());
                    assert_eq!(witness.edge(), boundary.edge());
                }
            }
        }
    }
}

#[test]
fn exact_axial_contact_is_radial_independent_replay_and_swap_deterministic() {
    for placement in [Placement::World, Placement::Oblique] {
        let tangent_radii = match placement {
            Placement::World => [1.0, 1.0],
            Placement::Oblique => [0.8, 0.8],
        };
        let radial_relations = [
            ("strict secant", 2.0, [2.0, 2.0]),
            ("tangent", 2.0, tangent_radii),
            ("strict internal", 2.0, [3.0, 0.5]),
            ("coincident", 0.0, [1.0, 1.0]),
        ];
        for reverse_second_axis in [false, true] {
            for (radial_relation, radial_distance, radii) in radial_relations {
                let fixture = exact_axial_contact_fixture(
                    placement,
                    reverse_second_axis,
                    radial_distance,
                    radii,
                );
                let forward_graph = section(&fixture, false);
                let replay_graph = section(&fixture, false);
                let swapped_graph = section(&fixture, true);
                let forward_sources = sources(&fixture, false);
                let swapped_sources = sources(&fixture, true);
                let context =
                    format!("{placement:?} {radial_relation} antiparallel={reverse_second_axis}");

                let forward = certified_axial_contact(
                    certify(
                        &fixture,
                        &forward_graph,
                        &forward_sources,
                        PARALLEL_CYLINDER_RELATION_WORK,
                    )
                    .unwrap(),
                    &context,
                );
                let replay = certified_axial_contact(
                    certify(
                        &fixture,
                        &replay_graph,
                        &forward_sources,
                        PARALLEL_CYLINDER_RELATION_WORK,
                    )
                    .unwrap(),
                    &context,
                );
                let swapped = certified_axial_contact(
                    certify(
                        &fixture,
                        &swapped_graph,
                        &swapped_sources,
                        PARALLEL_CYLINDER_RELATION_WORK,
                    )
                    .unwrap(),
                    &context,
                );

                assert_eq!(forward, replay, "{context}");
                assert_eq!(
                    forward.contact_boundaries().map(|witness| (
                        witness.boundary(),
                        witness.cap_face(),
                        witness.edge(),
                    )),
                    swapped.contact_boundaries().map(|witness| (
                        witness.boundary(),
                        witness.cap_face(),
                        witness.edge(),
                    )),
                    "{context}",
                );
                assert_eq!(
                    forward
                        .contact_boundaries()
                        .map(|witness| witness.operand()),
                    [0, 1],
                    "{context}",
                );
                assert_eq!(
                    swapped
                        .contact_boundaries()
                        .map(|witness| witness.operand()),
                    [1, 0],
                    "{context}",
                );
                assert_eq!(
                    forward
                        .contact_boundaries()
                        .map(|witness| witness.boundary()),
                    [1, usize::from(reverse_second_axis)],
                    "{context}",
                );
                for witness in forward.contact_boundaries() {
                    let source = &forward_sources[witness.operand()];
                    let boundary = source.boundaries()[witness.boundary()];
                    assert_eq!(witness.cap_face(), boundary.cap_face(), "{context}");
                    assert_eq!(witness.edge(), boundary.edge(), "{context}");
                }
            }
        }
    }
}

#[test]
fn exact_common_support_retains_all_four_boundaries_and_endpoint_equalities() {
    let cases = [
        ("nested", (-2.0, 4.0), (-1.0, 2.0), [1, 1]),
        ("crossing", (-2.0, 3.0), (-1.0, 3.0), [1, 1]),
        ("shared low", (-2.0, 4.0), (-2.0, 2.0), [2, 1]),
        ("shared high", (-2.0, 2.0), (-1.0, 1.0), [1, 2]),
        ("equal", (-2.0, 2.0), (-2.0, 2.0), [2, 2]),
    ];
    for (name, first, second, intersection_end_contributors) in cases {
        for reverse_second_axis in [false, true] {
            let fixture = common_support_fixture(first, second, reverse_second_axis);
            for swapped in [false, true] {
                let graph = section(&fixture, swapped);
                let replay = section(&fixture, swapped);
                let sources = sources(&fixture, swapped);
                let relation = certified_common_support(
                    certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
                );
                let replayed = certified_common_support(
                    certify(&fixture, &replay, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
                );
                assert_eq!(relation, replayed, "{name} swapped={swapped}");

                for (index, witness) in relation.boundaries().iter().enumerate() {
                    let operand = index / 2;
                    let boundary = index % 2;
                    assert_eq!(witness.operand(), operand, "{name} swapped={swapped}");
                    assert_eq!(witness.boundary(), boundary, "{name} swapped={swapped}");
                    assert_eq!(
                        witness.cap_face(),
                        sources[operand].boundaries()[boundary].cap_face(),
                        "{name} swapped={swapped}",
                    );
                    assert_eq!(
                        witness.edge(),
                        sources[operand].boundaries()[boundary].edge(),
                        "{name} swapped={swapped}",
                    );
                }

                let plan = plan_axial_interval_sweep(
                    RegularizedBooleanOperation::Intersect,
                    relation.preorder(),
                );
                let [span] = plan.spans() else {
                    panic!("{name} swapped={swapped}: intersection must have one span")
                };
                assert_eq!(
                    [span.low().iter().count(), span.high().iter().count()],
                    intersection_end_contributors,
                    "{name} antiparallel={reverse_second_axis} swapped={swapped}",
                );
            }
        }
    }
}

#[test]
fn exact_common_support_preserves_gap_contact_precedence_and_rejects_ring_drift() {
    for reverse_second_axis in [false, true] {
        let gap = common_support_fixture((-2.0, 1.0), (1.0, 1.0), reverse_second_axis);
        let graph = section(&gap, false);
        let gap_sources = sources(&gap, false);
        assert!(matches!(
            certify(&gap, &graph, &gap_sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
            ParallelCylinderRelationOutcome::CertifiedAxialSeparation(_)
        ));

        let contact = common_support_fixture((-1.0, 1.0), (0.0, 1.0), reverse_second_axis);
        let graph = section(&contact, false);
        let contact_sources = sources(&contact, false);
        assert!(matches!(
            certify(
                &contact,
                &graph,
                &contact_sources,
                PARALLEL_CYLINDER_RELATION_WORK,
            )
            .unwrap(),
            ParallelCylinderRelationOutcome::CertifiedAxialContact(_)
        ));
    }

    let mut drifted = common_support_fixture((-2.0, 4.0), (-1.0, 2.0), false);
    let body = drifted.inner.clone();
    perturb_side_height_within_full_tolerance(&mut drifted, &body, 1);
    let graph = section(&drifted, false);
    let sources = sources(&drifted, false);
    assert_eq!(
        certify(&drifted, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
        ParallelCylinderRelationOutcome::Indeterminate(
            ParallelCylinderRelationGap::SourceBoundaryBinding,
        ),
    );
}

#[test]
fn exterior_radial_separation_precedes_exact_axial_contact() {
    for placement in [Placement::World, Placement::Oblique] {
        for reverse_second_axis in [false, true] {
            let fixture =
                exact_axial_contact_fixture(placement, reverse_second_axis, 4.0, [1.0, 1.0]);
            for swapped in [false, true] {
                let graph = section(&fixture, swapped);
                let sources = sources(&fixture, swapped);
                assert_eq!(
                    certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
                    ParallelCylinderRelationOutcome::CertifiedExteriorRadialSeparation,
                    "{placement:?} antiparallel={reverse_second_axis} swapped={swapped}",
                );
            }
        }
    }
}

#[test]
fn exterior_radial_witness_remains_available_without_an_axial_gap() {
    for placement in [Placement::World, Placement::Oblique] {
        for reverse_second_axis in [false, true] {
            let fixture = fixture_with_geometry(
                placement,
                (-2.0, 4.0),
                (-1.0, 2.0),
                reverse_second_axis,
                [-1.5, 1.5],
                [1.0, 1.0],
            );
            for swapped in [false, true] {
                let graph = section(&fixture, swapped);
                let sources = sources(&fixture, swapped);
                assert_eq!(
                    certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
                    ParallelCylinderRelationOutcome::CertifiedExteriorRadialSeparation,
                );
            }
        }
    }
}

#[test]
fn axial_separation_rejects_full_valid_subtolerance_cap_plane_drift() {
    for placement in [Placement::World, Placement::Oblique] {
        for perturbation in [CapPlanePerturbation::Shift, CapPlanePerturbation::Tilt] {
            let mut fixture = axially_separated_fixture(placement, false);
            let body = fixture.outer.clone();
            perturb_cap_plane_within_full_tolerance(&mut fixture, &body, 1, perturbation);

            let graph = section(&fixture, false);
            // Extraction repeats Full validation; reaching Ready demonstrates
            // why the stricter exact relation binding is independently needed.
            let sources = sources(&fixture, false);
            assert_eq!(
                certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
                ParallelCylinderRelationOutcome::Indeterminate(
                    ParallelCylinderRelationGap::SourceBoundaryBinding,
                ),
                "{placement:?} {perturbation:?}",
            );
        }
    }
}

#[test]
fn rounded_axis_evaluation_does_not_masquerade_as_an_exact_affine_identity() {
    let tiny = f64::EPSILON / 2.0;
    let origin = Point3::new(1.0, 1.0, 1.0);
    let axis = Vec3::new(tiny, 1.0, 0.0);
    let rounded = origin + axis;
    assert_eq!(rounded.x, origin.x);
    assert!(!axis_parameter_identity_is_exact(
        rounded, origin, axis, 1.0,
    ));

    let exact = origin + axis * 2.0;
    assert!(axis_parameter_identity_is_exact(exact, origin, axis, 2.0));
}

#[test]
fn tolerance_backed_side_height_cannot_certify_a_one_ulp_cap_gap() {
    let boundary = 1.0_f64;
    let mut fixture = fixture_with_geometry(
        Placement::World,
        (-1.0, 2.0),
        (next_up_positive(boundary), 0.5),
        false,
        [0.0, 0.25],
        [2.0, 0.5],
    );
    let body = fixture.outer.clone();
    perturb_side_height_within_full_tolerance(&mut fixture, &body, 1);

    let graph = section(&fixture, false);
    // Extraction repeats Full validation, but the tolerance-backed side lift
    // is wider than the exact one-ULP cap-plane gap.
    let sources = sources(&fixture, false);
    assert!(matches!(
        certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
        ParallelCylinderRelationOutcome::Indeterminate(_)
    ));
}

#[test]
fn tolerance_backed_side_height_cannot_certify_exact_axial_contact() {
    for placement in [Placement::World, Placement::Oblique] {
        for reverse_second_axis in [false, true] {
            let mut fixture =
                exact_axial_contact_fixture(placement, reverse_second_axis, 2.0, [2.0, 2.0]);
            let body = fixture.outer.clone();
            perturb_side_height_within_full_tolerance(&mut fixture, &body, 1);

            let graph = section(&fixture, false);
            let sources = sources(&fixture, false);
            assert!(matches!(
                certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap(),
                ParallelCylinderRelationOutcome::Indeterminate(_)
            ));
        }
    }
}

#[test]
fn exact_constructor_one_ulp_gap_contact_and_overlap_are_distinct() {
    let boundary = 1.0;
    for reverse_second_axis in [false, true] {
        for (second_low, expected, context) in [
            (next_up_positive(boundary), 0, "one-ulp gap"),
            (boundary, 1, "contact"),
            (next_down_positive(boundary), 2, "one-ulp overlap"),
        ] {
            let fixture = fixture_with_geometry(
                Placement::World,
                (-1.0, 2.0),
                (second_low, 1.5 - second_low),
                reverse_second_axis,
                [-0.5, 0.5],
                [1.0, 1.0],
            );
            let graph = section(&fixture, false);
            let sources = sources(&fixture, false);
            let outcome =
                certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK).unwrap();
            assert_eq!(
                match &outcome {
                    ParallelCylinderRelationOutcome::CertifiedAxialSeparation(_) => 0,
                    ParallelCylinderRelationOutcome::CertifiedAxialContact(_) => 1,
                    ParallelCylinderRelationOutcome::Indeterminate(_) => 2,
                    _ => usize::MAX,
                },
                expected,
                "{context} antiparallel={reverse_second_axis}: {outcome:?}",
            );
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
        axially_separated_fixture(Placement::World, false),
        axially_separated_fixture(Placement::World, true),
        exact_axial_contact_fixture(Placement::World, false, 1.0, [1.0, 1.0]),
        exact_axial_contact_fixture(Placement::World, true, 1.0, [1.0, 1.0]),
        common_support_fixture((-2.0, 4.0), (-1.0, 2.0), false),
        common_support_fixture((-2.0, 4.0), (-1.0, 2.0), true),
    ] {
        let graph = section(&fixture, false);
        let sources = sources(&fixture, false);
        assert!(matches!(
            certify(&fixture, &graph, &sources, PARALLEL_CYLINDER_RELATION_WORK,).unwrap(),
            ParallelCylinderRelationOutcome::Certified(_)
                | ParallelCylinderRelationOutcome::CertifiedAxialSeparation(_)
                | ParallelCylinderRelationOutcome::CertifiedAxialContact(_)
                | ParallelCylinderRelationOutcome::CertifiedCommonSupport(_)
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

fn next_up_positive(value: f64) -> f64 {
    assert!(value.is_finite() && value > 0.0);
    f64::from_bits(value.to_bits() + 1)
}

fn next_down_positive(value: f64) -> f64 {
    assert!(value.is_finite() && value > 0.0);
    f64::from_bits(value.to_bits() - 1)
}

#[test]
fn normalized_axial_intervals_accept_only_a_strict_one_ulp_gap() {
    let world = Frame::world();
    let first_high = 1.0;
    for reverse_second_axis in [false, true] {
        for (second_low, expected, context) in [
            (
                next_up_positive(first_high),
                Orientation::Positive,
                "one-ulp gap",
            ),
            (first_high, Orientation::Zero, "exact contact"),
            (
                next_down_positive(first_high),
                Orientation::Negative,
                "one-ulp overlap",
            ),
        ] {
            let second_high = 1.5;
            let second_height = second_high - second_low;
            let second_frame = if reverse_second_axis {
                Frame::new(Point3::new(0.5, 0.0, second_high), -world.z(), world.x()).unwrap()
            } else {
                world.with_origin(Point3::new(0.5, 0.0, second_low))
            };
            let sources = analytic_sources(
                world.with_origin(Point3::new(-0.5, 0.0, -1.0)),
                2.0,
                second_frame,
                second_height,
            );
            let supports = sources.each_ref().map(|source| {
                source
                    .boundaries()
                    .map(|boundary| AxialBoundarySupport::exact(boundary.center()))
            });
            let normalized = normalize_source_axial_intervals([&sources[0], &sources[1]], supports)
                .unwrap_or_else(|gap| {
                    panic!("{context} antiparallel={reverse_second_axis}: {gap:?}")
                });
            let first_upper = sources[0].boundaries()[normalized.sources[0].high].center();
            let second_lower = sources[1].boundaries()[normalized.sources[1].low].center();
            assert_eq!(
                axial_compare(normalized.common_axis, second_lower, first_upper).unwrap(),
                expected,
                "{context} antiparallel={reverse_second_axis}",
            );
            let gap = strict_axial_gap_boundaries(&normalized).unwrap();
            assert_eq!(
                gap.is_some(),
                expected == Orientation::Positive,
                "{context} antiparallel={reverse_second_axis}",
            );
        }
    }
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
