use super::*;

use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};

const TEST_TOLERANCE: f64 = 1.0e-8;

struct FamilyFixture {
    admission: SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    topology: SkewCylinderFiniteWindowTopologyCertificate,
    members: Vec<PersistentSkewCylinderFiniteWindowMemberInput>,
    tolerance: f64,
}

fn perpendicular_pair(offset: f64) -> [Cylinder; 2] {
    [
        Cylinder::new(Frame::world(), 1.0).unwrap(),
        Cylinder::new(
            Frame::new(
                Point3::new(0.0, offset, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
            2.0,
        )
        .unwrap(),
    ]
}

fn formula_ranges() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(1.8, 1.9)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 1.25)],
    ]
}

fn axial_topologies(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
) -> [SkewCylinderAxialBoundTopology; 4] {
    [
        (0, SkewCylinderAxialBoundary::Lower, ranges[0][1].lo),
        (0, SkewCylinderAxialBoundary::Upper, ranges[0][1].hi),
        (1, SkewCylinderAxialBoundary::Lower, ranges[1][1].lo),
        (1, SkewCylinderAxialBoundary::Upper, ranges[1][1].hi),
    ]
    .map(|(formula_slot, boundary, value)| {
        classify_skew_cylinder_axial_bound(
            cylinders,
            formula_to_source,
            SkewCylinderAxialBoundProvenance {
                source_operand: formula_to_source[formula_slot],
                boundary,
                value,
            },
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
    })
}

fn finite_topology(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
) -> SkewCylinderFiniteWindowTopologyCertificate {
    let topologies = axial_topologies(cylinders, ranges, formula_to_source);
    classify_skew_cylinder_open_spans(SkewCylinderOpenSpanTopologyInput {
        topologies: &topologies,
        ranges,
        canonical_to_source: formula_to_source,
    })
    .unwrap()
}

fn strict_positive(
    cylinders: [Cylinder; 2],
) -> SkewCylinderStrictPositiveTwoSheetAdmissionCertificate {
    let SkewCylinderExactDiscriminantTopology::StrictPositive(admission) =
        classify_skew_cylinder_exact_discriminant(cylinders, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK)
            .unwrap()
    else {
        panic!("perpendicular fixture must have two strict-positive sheets");
    };
    admission
}

fn family_fixture() -> FamilyFixture {
    family_fixture_for(perpendicular_pair(0.0), formula_ranges(), TEST_TOLERANCE)
}

fn family_fixture_for(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
) -> FamilyFixture {
    let topology = finite_topology(cylinders, ranges, [0, 1]);
    let members = derived_open_members(&topology)
        .into_iter()
        .map(|span| {
            let residual = certify_paired_skew_cylinder_branch_subrange_residuals(
                cylinders, ranges, span.range, span.sheet, tolerance,
            )
            .unwrap();
            let roots = span.root_longitude_intervals(ranges[0][0]).unwrap();
            let root_corridors = [
                residual
                    .certify_lower_pcurve_root_corridor(roots[0])
                    .unwrap(),
                residual
                    .certify_upper_pcurve_root_corridor(roots[1])
                    .unwrap(),
            ];
            PersistentSkewCylinderFiniteWindowMemberInput {
                residual,
                root_corridors,
            }
        })
        .collect();
    FamilyFixture {
        admission: strict_positive(cylinders),
        topology,
        members,
        tolerance,
    }
}

fn certify_fixture(
    fixture: &FamilyFixture,
) -> Result<PersistentSkewCylinderFiniteWindowFamilyCertificate, IntersectionCertificateError> {
    certify_persistent_skew_cylinder_finite_window_family(
        fixture.admission,
        &fixture.topology,
        &fixture.members,
        fixture.tolerance,
    )
}

#[test]
fn exact_topology_mints_complete_deterministic_family_without_added_work() {
    let fixture = family_fixture();
    let family = certify_fixture(&fixture).unwrap();

    assert_eq!(
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS,
        4 * PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_CELLS_PER_BOUND
    );
    assert_eq!(
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_BOUND,
        2 * PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_CELLS_PER_BOUND
    );
    assert_eq!(family.version(), 1);
    assert_eq!(family.member_count(), 4);
    assert_eq!(
        family.work(),
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_FAMILY_BASE_WORK
            + 4 * PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
    );
    assert_eq!(family.work(), 896 + 260 * 4);
    assert_eq!(
        family.sheet_occupancy(SkewCylinderSheet::Lower),
        PersistentSkewCylinderFiniteWindowSheetOccupancy::Outside
    );
    assert_eq!(
        family.sheet_occupancy(SkewCylinderSheet::Upper),
        PersistentSkewCylinderFiniteWindowSheetOccupancy::Open {
            first_member_ordinal: 0,
            member_count: 4,
        }
    );
    for (ordinal, input) in fixture.members.iter().copied().enumerate() {
        let membership = family.membership(ordinal).unwrap();
        let member = membership.member();
        assert_eq!(membership.family(), family);
        assert_eq!(membership.ordinal(), ordinal);
        assert_eq!(member.ordinal(), ordinal);
        assert_eq!(member.sheet(), SkewCylinderSheet::Upper);
        assert_eq!(member.guarded_range(), input.residual.carrier_range());
        assert_eq!(
            member.root_parameter_enclosures(),
            input
                .root_corridors
                .map(|corridor| corridor.root_parameter())
        );
        validate_finite_window_family_membership(membership, input.residual, input.root_corridors)
            .unwrap();
    }
}

#[test]
fn missing_and_reordered_members_are_rejected() {
    let mut fixture = family_fixture();
    fixture.members.pop();
    assert_eq!(
        certify_fixture(&fixture),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );

    let mut fixture = family_fixture();
    fixture.members.swap(0, 1);
    assert_eq!(
        certify_fixture(&fixture),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );
}

#[test]
fn shifted_and_swapped_root_corridors_are_rejected() {
    let mut shifted = family_fixture();
    let lower = shifted.members[0].root_corridors[0].root_parameter();
    let shifted_root = Interval::new(lower.lo().next_down(), lower.hi());
    shifted.members[0].root_corridors[0] = shifted.members[0]
        .residual
        .certify_lower_pcurve_root_corridor(shifted_root)
        .unwrap();
    assert_eq!(
        certify_fixture(&shifted),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );

    let mut swapped = family_fixture();
    swapped.members[0].root_corridors = swapped.members[1].root_corridors;
    assert_eq!(
        certify_fixture(&swapped),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );
}

#[test]
fn cross_geometry_and_source_permutation_replay_are_rejected() {
    let fixture = family_fixture();
    let other_admission = strict_positive(perpendicular_pair(0.125));
    assert_eq!(
        certify_persistent_skew_cylinder_finite_window_family(
            other_admission,
            &fixture.topology,
            &fixture.members,
            TEST_TOLERANCE,
        ),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );

    let reversed_topology = finite_topology(perpendicular_pair(0.0), formula_ranges(), [1, 0]);
    assert_eq!(
        certify_persistent_skew_cylinder_finite_window_family(
            fixture.admission,
            &reversed_topology,
            &fixture.members,
            TEST_TOLERANCE,
        ),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );
}

#[test]
fn membership_replay_rejects_another_members_evidence() {
    let fixture = family_fixture();
    let family = certify_fixture(&fixture).unwrap();
    let first_membership = family.membership(0).unwrap();
    let second = fixture.members[1];

    assert_eq!(
        validate_finite_window_family_membership(
            first_membership,
            second.residual,
            second.root_corridors,
        ),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );
}

#[test]
fn complete_family_and_selected_composite_reissue_under_rigid_frames() {
    let fixture = family_fixture();
    let family = certify_fixture(&fixture).unwrap();
    let input = fixture.members[0];
    let canonical_endpoints = input.root_corridors.map(|corridor| {
        let root = corridor.root_parameter();
        input
            .residual
            .carrier()
            .eval(0.5 * root.lo() + 0.5 * root.hi())
    });

    for placement in [
        Frame::world().with_origin(Point3::new(3.0, -2.0, 1.0)),
        Frame::new(
            Point3::new(-1.25, 2.5, 0.75),
            Vec3::new(1.0, 2.0, 3.0).normalized().unwrap(),
            Vec3::new(2.0, -1.0, 0.0).normalized().unwrap(),
        )
        .unwrap(),
    ] {
        let transformed = family
            .formula_cylinders()
            .map(|cylinder| rigid_cylinder(cylinder, placement));
        let reissue =
            reissue_persistent_skew_cylinder_finite_window_family(family, transformed).unwrap();
        assert_eq!(reissue.source_family(), family);
        assert_eq!(reissue.certificate().formula_cylinders(), transformed);

        for orientation in [
            PersistentSkewCylinderOpenSpanOrientation::Forward,
            PersistentSkewCylinderOpenSpanOrientation::Reversed,
        ] {
            let source = certify_persistent_skew_cylinder_open_span_in_family(
                input.residual,
                input.root_corridors,
                canonical_endpoints,
                orientation,
                family.membership(0).unwrap(),
            )
            .unwrap();
            let transformed_logical_endpoints = source
                .endpoint_points()
                .map(|point| placement.point_at(point.x, point.y, point.z));
            let copied = reissue
                .reissue_member(source, transformed_logical_endpoints)
                .unwrap();
            assert_eq!(copied.orientation(), orientation);
            assert_eq!(copied.endpoint_points(), transformed_logical_endpoints);
            assert_eq!(
                copied.finite_window_family_membership().unwrap().family(),
                reissue.certificate()
            );
            assert_eq!(
                copied.residual_certificate().carrier().cylinders(),
                transformed
            );
        }
    }
}

#[test]
fn reissue_refuses_a_member_from_another_complete_family() {
    let fixture = family_fixture();
    let family = certify_fixture(&fixture).unwrap();
    let reissue =
        reissue_persistent_skew_cylinder_finite_window_family(family, family.formula_cylinders())
            .unwrap();
    let other = family_fixture_for(
        perpendicular_pair(0.0),
        formula_ranges(),
        2.0 * TEST_TOLERANCE,
    );
    let other_family = certify_fixture(&other).unwrap();
    let input = other.members[0];
    let endpoints = input.root_corridors.map(|corridor| {
        let root = corridor.root_parameter();
        input
            .residual
            .carrier()
            .eval(0.5 * root.lo() + 0.5 * root.hi())
    });
    let source = certify_persistent_skew_cylinder_open_span_in_family(
        input.residual,
        input.root_corridors,
        endpoints,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
        other_family.membership(0).unwrap(),
    )
    .unwrap();

    assert_eq!(
        reissue.reissue_member(source, endpoints),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );
}

fn rigid_cylinder(cylinder: Cylinder, placement: Frame) -> Cylinder {
    let frame = cylinder.frame();
    let map_vector = |vector: Vec3| {
        placement.x() * vector.x + placement.y() * vector.y + placement.z() * vector.z
    };
    Cylinder::new(
        Frame::new(
            placement.point_at(frame.origin().x, frame.origin().y, frame.origin().z),
            map_vector(frame.z()),
            map_vector(frame.x()),
        )
        .unwrap(),
        cylinder.radius(),
    )
    .unwrap()
}
