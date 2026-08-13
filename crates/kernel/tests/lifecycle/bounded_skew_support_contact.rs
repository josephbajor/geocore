//! Facade-only lifecycle evidence for an exact isolated skew support tangency.
//! Wall-time budget: less than 10 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodySectionGraph, SectionBranchTopology, SectionCurveEndpointTopology,
    SectionCurveFragmentSpan, SectionFoldedSupportSheet, SectionIsolatedContactKind,
    SectionPeriodicEmbeddingGap, SectionSite, SectionSkewCylinderAxialBoundary,
    SectionTouchingSupportSheet,
};

struct Fixture {
    session: Session,
    part: PartId,
    first: BodyId,
    second: BodyId,
    frame: Frame,
}

fn exact_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
    ]
}

fn folded_exact_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
    ]
}

fn seam_folded_exact_frames() -> [Frame; 2] {
    [
        Frame::world(),
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.0, 0.0),
        )
        .unwrap(),
    ]
}

fn fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -2.0)),
                1.0,
                4.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(-2.0, 3.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn boundary_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(frame, 1.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.point_at(-2.0, 3.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn corner_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(frame, 1.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame = Frame::new(frame.point_at(0.0, 3.0, 0.0), frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn folded_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 2.25),
                1.0,
                4.5,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_axis_origin = frame.origin() + frame.y() * 3.0_f64.next_down();
        let second_frame =
            Frame::new(second_axis_origin + frame.x() * 1.25, -frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 2.5))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn seam_folded_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 2.25),
                1.0,
                4.5,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_center = frame.origin() + frame.x() * 3.0_f64.next_down();
        let second_frame =
            Frame::new(second_center - frame.y() * 1.25, frame.y(), frame.x()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 2.0, 2.5))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn seam_root_folded_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 0.5),
                0.0625,
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_center = frame.origin() + frame.y() * 0.125;
        let second_frame =
            Frame::new(second_center - frame.x() * 0.5, frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 0.125, 1.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn seam_root_across_folded_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 0.5),
                0.0625,
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_center = frame.origin() - frame.y() * 0.125;
        let second_frame =
            Frame::new(second_center - frame.x() * 0.5, frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 0.125, 1.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn touching_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 0.5),
                0.25,
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_center = frame.origin() + frame.y() * 0.125;
        let second_frame =
            Frame::new(second_center - frame.x() * 0.5, frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 0.375, 1.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn seam_touching_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 0.5),
                0.25,
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_center = frame.origin() - frame.x() * 0.125;
        let second_frame =
            Frame::new(second_center - frame.y() * 0.5, frame.y(), frame.x()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 0.375, 1.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn opposite_pole_touching_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 0.5),
                0.25,
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_center = frame.origin() + frame.x() * 0.125;
        let second_frame =
            Frame::new(second_center - frame.y() * 0.5, frame.y(), frame.x()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 0.375, 1.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn double_touching_support_fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.origin() - frame.z() * 0.5),
                0.25,
                1.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame =
            Frame::new(frame.origin() - frame.x() * 0.5, frame.x(), frame.y()).unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(second_frame, 0.25, 1.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    Fixture {
        session,
        part,
        first,
        second,
        frame,
    }
}

fn section(fixture: &Fixture, swapped: bool) -> BodySectionGraph {
    let bodies = if swapped {
        [fixture.second.clone(), fixture.first.clone()]
    } else {
        [fixture.first.clone(), fixture.second.clone()]
    };
    fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(
            bodies[0].clone(),
            bodies[1].clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap()
}

fn assert_support_contact(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.branches().is_empty());
    assert!(graph.curve_fragments().is_empty());
    assert_eq!(graph.curve_endpoints().len(), 1);
    assert_eq!(graph.isolated_contacts().len(), 1);
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert!(component.fragments().is_empty());
    assert_eq!(component.isolated_contacts(), &[0]);

    let contact = &graph.isolated_contacts()[0];
    assert_eq!(contact.kind(), SectionIsolatedContactKind::SupportTangency);
    assert!(contact.roots().is_empty());
    assert_eq!(contact.endpoint(), 0);
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = graph.curve_endpoints()[0].topology()
    else {
        panic!("support tangency acquired a non-trim endpoint")
    };
    assert!(
        sites
            .iter()
            .all(|site| matches!(site, SectionSite::FaceInterior(_)))
    );
    assert!(source_parameters.iter().all(Option::is_none));
    assert!(contact.point().dist(fixture.frame.point_at(0.0, 1.0, 0.0)) <= 1.0e-12);
    assert!((contact.surface_parameters()[0][1] - 2.0).abs() <= 1.0e-12);
    assert!((contact.surface_parameters()[1][1] - 2.0).abs() <= 1.0e-12);

    let part = fixture.session.part(fixture.part.clone()).unwrap();
    for body in [fixture.first.clone(), fixture.second.clone()] {
        let classification = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body, contact.point()))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(matches!(
            classification.verdict(),
            kernel::PointBodyVerdict::Boundary { .. }
        ));
    }
}

fn assert_boundary_support_contact(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    boundary_operand: usize,
) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.branches().is_empty());
    assert!(graph.curve_fragments().is_empty());
    assert_eq!(graph.curve_endpoints().len(), 1);
    assert_eq!(graph.isolated_contacts().len(), 1);
    assert_eq!(graph.curve_components().len(), 1);

    let contact = &graph.isolated_contacts()[0];
    assert_eq!(contact.kind(), SectionIsolatedContactKind::SupportTangency);
    let [root] = contact.roots() else {
        panic!("boundary support contact did not retain one source root")
    };
    assert_eq!(root.operand(), boundary_operand);
    assert_eq!(
        root.axial_boundary(),
        SectionSkewCylinderAxialBoundary::Lower
    );
    assert_eq!(root.authored_bound().to_bits(), 0.0_f64.to_bits());
    assert_eq!(contact.endpoint(), 0);
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = graph.curve_endpoints()[0].topology()
    else {
        panic!("boundary support tangency acquired a non-trim endpoint")
    };
    for operand in 0..2 {
        if operand == boundary_operand {
            assert!(matches!(sites[operand], SectionSite::EdgeInterior(_)));
            assert!(source_parameters[operand].is_some());
        } else {
            assert!(matches!(sites[operand], SectionSite::FaceInterior(_)));
            assert!(source_parameters[operand].is_none());
        }
    }
    assert!(contact.point().dist(fixture.frame.point_at(0.0, 1.0, 0.0)) <= 1.0e-12);
    assert!(contact.surface_parameters()[boundary_operand][1].abs() <= 1.0e-12);
}

fn assert_corner_support_contact(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.branches().is_empty());
    assert!(graph.curve_fragments().is_empty());
    let [contact] = graph.isolated_contacts() else {
        panic!("expected one corner support contact: {graph:#?}")
    };
    assert_eq!(contact.kind(), SectionIsolatedContactKind::SupportTangency);
    assert!(contact.point().dist(fixture.frame.point_at(0.0, 1.0, 0.0)) <= 1.0e-12);
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = graph.curve_endpoints()[contact.endpoint()].topology()
    else {
        panic!("corner support tangency acquired a non-trim endpoint")
    };
    assert!(
        sites
            .iter()
            .all(|site| matches!(site, SectionSite::EdgeInterior(_)))
    );
    assert!(source_parameters.iter().all(Option::is_some));
    assert_eq!(contact.roots().len(), 2);
    assert!(contact.roots().iter().all(|root| {
        root.axial_boundary() == SectionSkewCylinderAxialBoundary::Lower
            && root.authored_bound() == 0.0
    }));
}

fn assert_folded_support_component(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert!(graph.isolated_contacts().is_empty());
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.branches().len(), 2);
    assert_eq!(graph.curve_fragments().len(), 2);
    assert_eq!(graph.curve_endpoints().len(), 2);
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert_eq!(component.fragments().len(), 2);
    assert!(component.isolated_contacts().is_empty());

    let mut endpoint_incidence = [0_usize; 2];
    for &fragment_index in component.fragments() {
        let fragment = &graph.curve_fragments()[fragment_index];
        assert_eq!(fragment.source_ordinal(), 0);
        let branch = &graph.branches()[fragment.branch()];
        assert_eq!(branch.topology(), SectionBranchTopology::Open);
        assert_eq!(branch.fragment_sites().len(), 2);
        assert_eq!(branch.endpoint_sites(), [0, 1]);
        assert!(branch.embedding_certificate().is_none());
        let SectionCurveFragmentSpan::FoldedSupport { endpoints } = fragment.span() else {
            panic!("folded support component retained a non-folded fragment")
        };
        assert!(endpoints[0].carrier_parameter() < endpoints[1].carrier_parameter());
        for endpoint in endpoints.iter() {
            endpoint_incidence[endpoint.endpoint()] += 1;
            let local = fixture.frame.to_local(endpoint.point());
            assert!((local.x * local.x + local.y * local.y - 1.0).abs() <= 1.0e-12);
            assert!(local.z.abs() <= 1.0e-12);
            assert!((local.y - (3.0_f64.next_down() - 2.0)).abs() <= 1.0e-12);
        }
    }
    assert_eq!(endpoint_incidence, [2, 2]);
    let mut root_ordinals = graph
        .curve_endpoints()
        .iter()
        .map(|endpoint| match endpoint.topology() {
            SectionCurveEndpointTopology::FoldedSupportJoin {
                faces,
                root_ordinal,
                root_interval,
                ..
            } => {
                assert_eq!(faces, graph.branches()[0].faces());
                assert!(root_interval.lo().is_finite());
                assert!(root_interval.hi().is_finite());
                assert!(root_interval.lo() <= root_interval.hi());
                *root_ordinal
            }
            topology => panic!("folded support acquired an unexpected endpoint: {topology:?}"),
        })
        .collect::<Vec<_>>();
    root_ordinals.sort_unstable();
    assert_eq!(root_ordinals, vec![0, 1]);
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    let mut embedding_operands = graph
        .periodic_face_embeddings()
        .iter()
        .map(|embedding| {
            assert!(
                matches!(
                embedding.gap(),
                Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { fragment })
                    if *fragment < graph.curve_fragments().len()
                ),
                "{embedding:?}"
            );
            embedding.operand()
        })
        .collect::<Vec<_>>();
    embedding_operands.sort_unstable();
    assert_eq!(embedding_operands, vec![0, 1]);
}

fn assert_seam_folded_support_component(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert!(graph.isolated_contacts().is_empty());
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.branches().len(), 4);
    assert_eq!(graph.curve_fragments().len(), 4);
    assert_eq!(graph.curve_endpoints().len(), 4);
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert_eq!(component.fragments().len(), 4);

    let mut endpoint_incidence = vec![0_usize; 4];
    for &fragment_index in component.fragments() {
        let fragment = &graph.curve_fragments()[fragment_index];
        let branch = &graph.branches()[fragment.branch()];
        assert_eq!(branch.topology(), SectionBranchTopology::Open);
        assert!(branch.embedding_certificate().is_none());
        let SectionCurveFragmentSpan::FoldedSupport { endpoints } = fragment.span() else {
            panic!("seam-folded support retained a non-folded fragment")
        };
        for endpoint in endpoints.iter() {
            endpoint_incidence[endpoint.endpoint()] += 1;
            let local = fixture.frame.to_local(endpoint.point());
            assert!((local.x * local.x + local.y * local.y - 1.0).abs() <= 1.0e-12);
            assert!((-2.25..=2.25).contains(&local.z));
        }
    }
    assert_eq!(endpoint_incidence, vec![2, 2, 2, 2]);

    let mut roots = Vec::new();
    let mut seams = Vec::new();
    for endpoint in graph.curve_endpoints() {
        match endpoint.topology() {
            SectionCurveEndpointTopology::FoldedSupportJoin { root_ordinal, .. } => {
                roots.push(*root_ordinal);
            }
            SectionCurveEndpointTopology::FoldedSupportSeamJoin { sheet, .. } => {
                seams.push(*sheet);
            }
            topology => panic!("seam-folded support acquired an unexpected endpoint: {topology:?}"),
        }
    }
    roots.sort_unstable();
    seams.sort_by_key(|sheet| match sheet {
        SectionFoldedSupportSheet::Lower => 0,
        SectionFoldedSupportSheet::Upper => 1,
    });
    assert_eq!(roots, vec![0, 1]);
    assert_eq!(
        seams,
        vec![
            SectionFoldedSupportSheet::Lower,
            SectionFoldedSupportSheet::Upper,
        ]
    );
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    assert!(graph.periodic_face_embeddings().iter().all(|embedding| {
        matches!(
            embedding.gap(),
            Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { .. })
        )
    }));
}

fn assert_seam_root_folded_support_component(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    chart_join_longitude: f64,
) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert!(graph.isolated_contacts().is_empty());
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.branches().len(), 4);
    assert_eq!(graph.curve_fragments().len(), 4);
    assert_eq!(graph.curve_endpoints().len(), 4);
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert_eq!(component.fragments().len(), 4);

    let mut endpoint_incidence = vec![0_usize; 4];
    for &fragment_index in component.fragments() {
        let fragment = &graph.curve_fragments()[fragment_index];
        let branch = &graph.branches()[fragment.branch()];
        assert_eq!(branch.topology(), SectionBranchTopology::Open);
        assert!(branch.embedding_certificate().is_none());
        let SectionCurveFragmentSpan::FoldedSupport { endpoints } = fragment.span() else {
            panic!("seam-root folded support retained a non-folded fragment")
        };
        for endpoint in endpoints.iter() {
            endpoint_incidence[endpoint.endpoint()] += 1;
            let local = fixture.frame.to_local(endpoint.point());
            assert!((local.x * local.x + local.y * local.y - 0.0625_f64.powi(2)).abs() <= 1.0e-12);
            assert!((-0.5..=0.5).contains(&local.z));
        }
    }
    assert_eq!(endpoint_incidence, vec![2, 2, 2, 2]);

    let mut roots = Vec::new();
    let mut charts = Vec::new();
    for endpoint in graph.curve_endpoints() {
        match endpoint.topology() {
            SectionCurveEndpointTopology::FoldedSupportJoin { root_ordinal, .. } => {
                roots.push(*root_ordinal);
            }
            SectionCurveEndpointTopology::FoldedSupportChartJoin {
                sheet, longitude, ..
            } => {
                assert_eq!(longitude.to_bits(), chart_join_longitude.to_bits());
                charts.push(*sheet);
            }
            topology => {
                panic!("seam-root folded support acquired an unexpected endpoint: {topology:?}")
            }
        }
    }
    roots.sort_unstable();
    charts.sort_by_key(|sheet| match sheet {
        SectionFoldedSupportSheet::Lower => 0,
        SectionFoldedSupportSheet::Upper => 1,
    });
    assert_eq!(roots, vec![0, 1]);
    assert_eq!(
        charts,
        vec![
            SectionFoldedSupportSheet::Lower,
            SectionFoldedSupportSheet::Upper,
        ]
    );
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    assert!(graph.periodic_face_embeddings().iter().all(|embedding| {
        matches!(
            embedding.gap(),
            Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { .. })
        )
    }));
}

fn assert_touching_support_component(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    expect_regular_seams: bool,
) {
    assert_touching_support_component_impl(fixture, graph, expect_regular_seams, false);
}

fn assert_opposite_pole_touching_support_component(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_touching_support_component_impl(fixture, graph, true, true);
}

fn assert_touching_support_component_impl(
    fixture: &Fixture,
    graph: &BodySectionGraph,
    expect_regular_seams: bool,
    expect_two_chart_joins: bool,
) {
    let expected_longitudes = if !expect_regular_seams || expect_two_chart_joins {
        vec![
            core::f64::consts::FRAC_PI_2,
            3.0 * core::f64::consts::FRAC_PI_2,
        ]
    } else {
        vec![core::f64::consts::FRAC_PI_2]
    };
    let expected_endpoint_count =
        2 + if expect_regular_seams { 2 } else { 0 } + 2 * expected_longitudes.len();
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert!(graph.isolated_contacts().is_empty());
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.branches().len(), expected_endpoint_count);
    assert_eq!(graph.curve_fragments().len(), expected_endpoint_count);
    assert_eq!(graph.curve_endpoints().len(), expected_endpoint_count);
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert_eq!(component.fragments().len(), expected_endpoint_count);

    let mut endpoint_incidence = vec![0_usize; expected_endpoint_count];
    for &fragment_index in component.fragments() {
        let fragment = &graph.curve_fragments()[fragment_index];
        let branch = &graph.branches()[fragment.branch()];
        assert_eq!(branch.topology(), SectionBranchTopology::Open);
        assert!(branch.embedding_certificate().is_none());
        let SectionCurveFragmentSpan::TouchingSupport { endpoints } = fragment.span() else {
            panic!("touching support retained a non-touching fragment")
        };
        for endpoint in endpoints.iter() {
            endpoint_incidence[endpoint.endpoint()] += 1;
            let local = fixture.frame.to_local(endpoint.point());
            assert!((local.x * local.x + local.y * local.y - 0.0625).abs() <= 1.0e-12);
            assert!((-0.375..=0.375).contains(&local.z));
        }
    }
    assert_eq!(endpoint_incidence, vec![2; expected_endpoint_count]);

    let mut root_ports = Vec::new();
    let mut seams = Vec::new();
    let mut charts = Vec::new();
    for endpoint in graph.curve_endpoints() {
        match endpoint.topology() {
            SectionCurveEndpointTopology::TouchingSupportRootJoin {
                continuation,
                root_interval,
                ..
            } => {
                assert!(root_interval.lo().is_finite());
                assert!(root_interval.hi().is_finite());
                assert!(root_interval.lo() <= root_interval.hi());
                root_ports.push(*continuation);
            }
            SectionCurveEndpointTopology::TouchingSupportSeamJoin { sheet, .. } => {
                seams.push(*sheet);
            }
            SectionCurveEndpointTopology::TouchingSupportChartJoin {
                sheet, longitude, ..
            } => {
                charts.push((*sheet, longitude.to_bits()));
            }
            topology => panic!("touching support acquired an unexpected endpoint: {topology:?}"),
        }
    }
    root_ports.sort_unstable();
    let sheet_key = |sheet: &SectionTouchingSupportSheet| match sheet {
        SectionTouchingSupportSheet::Lower => 0,
        SectionTouchingSupportSheet::Upper => 1,
    };
    seams.sort_by_key(sheet_key);
    charts.sort_by_key(|(sheet, longitude)| (sheet_key(sheet), *longitude));
    let expected_sheets = vec![
        SectionTouchingSupportSheet::Lower,
        SectionTouchingSupportSheet::Upper,
    ];
    assert_eq!(root_ports, vec![0, 1]);
    assert_eq!(
        seams,
        if expect_regular_seams {
            expected_sheets.clone()
        } else {
            Vec::new()
        }
    );
    let mut expected_charts = expected_sheets
        .into_iter()
        .flat_map(|sheet| {
            expected_longitudes
                .iter()
                .map(move |longitude| (sheet, longitude.to_bits()))
        })
        .collect::<Vec<_>>();
    expected_charts.sort_by_key(|(sheet, longitude)| (sheet_key(sheet), *longitude));
    assert_eq!(charts, expected_charts);
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    assert!(graph.periodic_face_embeddings().iter().all(|embedding| {
        matches!(
            embedding.gap(),
            Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { .. })
        )
    }));
}

fn assert_double_touching_support_components(fixture: &Fixture, graph: &BodySectionGraph) {
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{:#?}", graph.gaps());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert!(graph.isolated_contacts().is_empty());
    assert!(graph.through_contacts().is_empty());
    assert_eq!(graph.branches().len(), 6);
    assert_eq!(graph.curve_fragments().len(), 6);
    assert_eq!(graph.curve_endpoints().len(), 6);
    assert_eq!(graph.curve_components().len(), 2);

    let mut component_sizes = graph
        .curve_components()
        .iter()
        .map(|component| {
            assert!(component.closed());
            component.fragments().len()
        })
        .collect::<Vec<_>>();
    component_sizes.sort_unstable();
    assert_eq!(component_sizes, vec![2, 4]);

    let mut endpoint_incidence = vec![0_usize; 6];
    for component in graph.curve_components() {
        for &fragment_index in component.fragments() {
            let fragment = &graph.curve_fragments()[fragment_index];
            let branch = &graph.branches()[fragment.branch()];
            assert_eq!(branch.topology(), SectionBranchTopology::Open);
            assert!(branch.embedding_certificate().is_none());
            let SectionCurveFragmentSpan::TouchingSupport { endpoints } = fragment.span() else {
                panic!("double touching support retained a non-touching fragment")
            };
            for endpoint in endpoints.iter() {
                endpoint_incidence[endpoint.endpoint()] += 1;
                let local = fixture.frame.to_local(endpoint.point());
                assert!((local.x * local.x + local.y * local.y - 0.0625).abs() <= 1.0e-12);
                assert!((-0.25..=0.25).contains(&local.z));
            }
        }
    }
    assert_eq!(endpoint_incidence, vec![2, 2, 2, 2, 2, 2]);

    let mut root_ports = Vec::new();
    let mut seams = Vec::new();
    for endpoint in graph.curve_endpoints() {
        match endpoint.topology() {
            SectionCurveEndpointTopology::TouchingSupportRootJoin {
                root_ordinal,
                continuation,
                root_interval,
                ..
            } => {
                assert!(root_interval.lo().is_finite());
                assert!(root_interval.hi().is_finite());
                assert!(root_interval.lo() <= root_interval.hi());
                root_ports.push((*root_ordinal, *continuation));
            }
            SectionCurveEndpointTopology::TouchingSupportSeamJoin { sheet, .. } => {
                seams.push(*sheet);
            }
            topology => {
                panic!("double touching support acquired an unexpected endpoint: {topology:?}")
            }
        }
    }
    root_ports.sort_unstable();
    assert_eq!(root_ports, vec![(0, 0), (0, 1), (1, 0), (1, 1)]);
    seams.sort_by_key(|sheet| match sheet {
        SectionTouchingSupportSheet::Lower => 0,
        SectionTouchingSupportSheet::Upper => 1,
    });
    assert_eq!(
        seams,
        vec![
            SectionTouchingSupportSheet::Lower,
            SectionTouchingSupportSheet::Upper,
        ]
    );
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    assert!(graph.periodic_face_embeddings().iter().all(|embedding| {
        matches!(
            embedding.gap(),
            Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { .. })
        )
    }));
}

#[test]
fn isolated_support_contact_section_is_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_support_contact(&fixture, &forward);
        assert_support_contact(&fixture, &swapped);
        assert_eq!(
            swapped.bodies(),
            &[fixture.second.clone(), fixture.first.clone()]
        );
    }
}

#[test]
fn isolated_support_contact_boolean_refuses_distinctly_without_mutation() {
    for frame in exact_frames() {
        for swapped in [false, true] {
            let mut fixture = fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Subtract,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
        }
    }
}

#[test]
fn boundary_support_contact_section_is_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = boundary_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_boundary_support_contact(&fixture, &forward, 0);
        assert_boundary_support_contact(&fixture, &swapped, 1);
    }
}

#[test]
fn boundary_support_contact_boolean_refuses_distinctly_without_mutation() {
    for frame in exact_frames() {
        for swapped in [false, true] {
            let mut fixture = boundary_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            let graph = section(&fixture, swapped);
            assert_boundary_support_contact(&fixture, &graph, usize::from(swapped));
        }
    }
}

#[test]
fn corner_support_contact_section_is_complete_replay_swap_and_frame_stable() {
    for frame in exact_frames() {
        let fixture = corner_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_corner_support_contact(&fixture, &forward);
        assert_corner_support_contact(&fixture, &swapped);
    }
}

#[test]
fn corner_support_contact_boolean_refuses_distinctly_without_mutation() {
    for frame in exact_frames() {
        for swapped in [false, true] {
            let mut fixture = corner_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_corner_support_contact(&fixture, &section(&fixture, swapped));
        }
    }
}

#[test]
fn folded_support_section_is_complete_replay_swap_and_frame_stable() {
    for frame in folded_exact_frames() {
        let fixture = folded_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay);
        assert_eq!(swapped, swapped_replay);
        assert_folded_support_component(&fixture, &forward);
        assert_folded_support_component(&fixture, &swapped);
    }
}

#[test]
fn folded_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = folded_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_folded_support_component(&fixture, &section(&fixture, swapped));
        }
    }
}

#[test]
fn seam_folded_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in seam_folded_exact_frames().into_iter().enumerate() {
        let fixture = seam_folded_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_eq!(
            forward.completion(),
            SectionCompletion::Complete,
            "frame {frame_index} forward: {forward:#?}"
        );
        assert_eq!(
            swapped.completion(),
            SectionCompletion::Complete,
            "frame {frame_index} swapped: {swapped:#?}"
        );
        assert_seam_folded_support_component(&fixture, &forward);
        assert_seam_folded_support_component(&fixture, &swapped);
    }
}

#[test]
fn seam_folded_support_boolean_refuses_distinctly_without_mutation() {
    for frame in seam_folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = seam_folded_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_seam_folded_support_component(&fixture, &section(&fixture, swapped));
        }
    }
}

#[test]
fn seam_root_folded_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in folded_exact_frames().into_iter().enumerate() {
        let fixture = seam_root_folded_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_seam_root_folded_support_component(&fixture, &forward, core::f64::consts::FRAC_PI_2);
        assert_seam_root_folded_support_component(&fixture, &swapped, core::f64::consts::FRAC_PI_2);
    }
}

#[test]
fn seam_root_folded_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = seam_root_folded_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_seam_root_folded_support_component(
                &fixture,
                &section(&fixture, swapped),
                core::f64::consts::FRAC_PI_2,
            );
        }
    }
}

#[test]
fn seam_root_across_folded_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in folded_exact_frames().into_iter().enumerate() {
        let fixture = seam_root_across_folded_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_seam_root_folded_support_component(
            &fixture,
            &forward,
            3.0 * core::f64::consts::FRAC_PI_2,
        );
        assert_seam_root_folded_support_component(
            &fixture,
            &swapped,
            3.0 * core::f64::consts::FRAC_PI_2,
        );
    }
}

#[test]
fn seam_root_across_folded_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = seam_root_across_folded_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(matches!(
                outcome,
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_seam_root_folded_support_component(
                &fixture,
                &section(&fixture, swapped),
                3.0 * core::f64::consts::FRAC_PI_2,
            );
        }
    }
}

#[test]
fn touching_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in folded_exact_frames().into_iter().enumerate() {
        let fixture = touching_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_touching_support_component(&fixture, &forward, true);
        assert_touching_support_component(&fixture, &swapped, true);
    }
}

#[test]
fn touching_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = touching_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_touching_support_component(&fixture, &section(&fixture, swapped), true);
        }
    }
}

#[test]
fn seam_touching_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in folded_exact_frames().into_iter().enumerate() {
        let fixture = seam_touching_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_touching_support_component(&fixture, &forward, false);
        assert_touching_support_component(&fixture, &swapped, false);
    }
}

#[test]
fn seam_touching_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = seam_touching_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_touching_support_component(&fixture, &section(&fixture, swapped), false);
        }
    }
}

#[test]
fn opposite_pole_touching_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in folded_exact_frames().into_iter().enumerate() {
        let fixture = opposite_pole_touching_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_opposite_pole_touching_support_component(&fixture, &forward);
        assert_opposite_pole_touching_support_component(&fixture, &swapped);
    }
}

#[test]
fn opposite_pole_touching_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = opposite_pole_touching_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_opposite_pole_touching_support_component(&fixture, &section(&fixture, swapped));
        }
    }
}

#[test]
fn double_touching_support_section_is_complete_replay_swap_and_frame_stable() {
    for (frame_index, frame) in folded_exact_frames().into_iter().enumerate() {
        let fixture = double_touching_support_fixture(frame);
        let forward = section(&fixture, false);
        let replay = section(&fixture, false);
        let swapped = section(&fixture, true);
        let swapped_replay = section(&fixture, true);
        assert_eq!(forward, replay, "frame {frame_index} forward replay");
        assert_eq!(swapped, swapped_replay, "frame {frame_index} swap replay");
        assert_double_touching_support_components(&fixture, &forward);
        assert_double_touching_support_components(&fixture, &swapped);
    }
}

#[test]
fn double_touching_support_boolean_refuses_distinctly_without_mutation() {
    for frame in folded_exact_frames() {
        for swapped in [false, true] {
            let mut fixture = double_touching_support_fixture(frame);
            let bodies = if swapped {
                [fixture.second.clone(), fixture.first.clone()]
            } else {
                [fixture.first.clone(), fixture.second.clone()]
            };
            let outcome = fixture
                .session
                .edit_part(fixture.part.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    bodies[0].clone(),
                    bodies[1].clone(),
                ))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    outcome,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{outcome:?}"
            );
            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), 2);
            assert!(part.body(fixture.first.clone()).is_ok());
            assert!(part.body(fixture.second.clone()).is_ok());
            assert_double_touching_support_components(&fixture, &section(&fixture, swapped));
        }
    }
}
