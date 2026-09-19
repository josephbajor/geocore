use kgeom::frame::Frame;
use kgeom::vec::Point3;

use super::*;
use crate::{
    SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, SkewCylinderExactDiscriminantTopology,
    SkewCylinderFoldedSupportCellLocation, certify_persistent_skew_cylinder_folded_support,
    certify_persistent_skew_cylinder_touching_support,
    certify_skew_cylinder_folded_support_topology, certify_skew_cylinder_touching_support_topology,
    classify_skew_cylinder_exact_discriminant,
};

fn cylinders(offset: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(0.0, offset, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    [first, second]
}

fn seam_cylinders(offset: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(offset, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    [first, second]
}

fn bounded_seam_cylinders(frame: Frame, offset: f64) -> [Cylinder; 2] {
    [
        Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 2.25), 1.0).unwrap(),
        Cylinder::new(
            Frame::new(
                frame.origin() + frame.x() * offset - frame.y() * 1.25,
                frame.y(),
                frame.x(),
            )
            .unwrap(),
            2.0,
        )
        .unwrap(),
    ]
}

fn touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame, 1.0).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() + frame.y() * 0.5, frame.x(), frame.y()).unwrap(),
            1.5,
        )
        .unwrap(),
    ]
}

fn seam_touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame, 1.0).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - frame.x() * 0.5, frame.y(), frame.x()).unwrap(),
            1.5,
        )
        .unwrap(),
    ]
}

fn bounded_opposite_pole_touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 0.5), 0.25).unwrap(),
        Cylinder::new(
            Frame::new(
                frame.origin() + frame.x() * 0.125 - frame.y() * 0.5,
                frame.y(),
                frame.x(),
            )
            .unwrap(),
            0.375,
        )
        .unwrap(),
    ]
}

fn double_touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame, 1.0).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin(), frame.x(), frame.y()).unwrap(),
            1.0,
        )
        .unwrap(),
    ]
}

fn mixed_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame, 0.25).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() + frame.y() * 0.125, frame.x(), frame.y()).unwrap(),
            0.125,
        )
        .unwrap(),
    ]
}

fn non_cardinal_seam_mixed_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    let second_axis = frame.x() * 0.8 - frame.y() * 0.6;
    let second_radial = frame.x() * 0.6 + frame.y() * 0.8;
    [
        Cylinder::new(frame, 0.078125).unwrap(),
        Cylinder::new(
            Frame::new(
                frame.origin() + second_radial * 0.0390625,
                second_axis,
                second_radial,
            )
            .unwrap(),
            0.0390625,
        )
        .unwrap(),
    ]
}

fn seam_root_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame, 1.0).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() + frame.y() * 2.0, frame.x(), frame.y()).unwrap(),
            2.0,
        )
        .unwrap(),
    ]
}

fn seam_root_across_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    [
        Cylinder::new(frame, 1.0).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - frame.y() * 2.0, frame.x(), frame.y()).unwrap(),
            2.0,
        )
        .unwrap(),
    ]
}

fn short_seam_root_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
    let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
    let first_radius = 1.0;
    let second_radius = 2.0;
    let offset = second_radial * second_radius - frame.x() * first_radius;
    [
        Cylinder::new(frame, first_radius).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
            second_radius,
        )
        .unwrap(),
    ]
}

fn bounded_short_seam_root_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
    let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    [
        Cylinder::new(frame, first_radius).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
            second_radius,
        )
        .unwrap(),
    ]
}

fn bounded_short_seam_root_across_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
    let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    let reversed_first = Frame::new(frame.origin() + frame.z(), -frame.z(), frame.x()).unwrap();
    [
        Cylinder::new(reversed_first, first_radius).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
            second_radius,
        )
        .unwrap(),
    ]
}

fn bounded_long_seam_root_across_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    let second_axis = frame.x() * -0.6 + frame.y() * 0.8;
    let second_radial = frame.x() * 0.8 + frame.y() * 0.6;
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    [
        Cylinder::new(frame, first_radius).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - offset, second_axis, second_radial).unwrap(),
            second_radius,
        )
        .unwrap(),
    ]
}

fn bounded_long_seam_root_between_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
    let second_axis = frame.x() * -0.6 + frame.y() * 0.8;
    let second_radial = frame.x() * 0.8 + frame.y() * 0.6;
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    let reversed_first = Frame::new(frame.origin(), -frame.z(), frame.x()).unwrap();
    [
        Cylinder::new(reversed_first, first_radius).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin() - offset, second_axis, second_radial).unwrap(),
            second_radius,
        )
        .unwrap(),
    ]
}

fn touching_support_windows() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(-2.0, 2.0)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-2.0, 2.0)],
    ]
}

fn mixed_folded_support_windows() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(-0.2, 0.2)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-0.3, 0.3)],
    ]
}

fn bounded_touching_support_windows() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 1.0)],
        [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 1.0)],
    ]
}

fn bounded_long_support_windows() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(-0.5, 0.75)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-0.5, 0.75)],
    ]
}

fn bounded_windows() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 4.5)],
        [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 2.5)],
    ]
}

fn windows() -> [[ParamRange; 2]; 2] {
    [
        [ParamRange::new(0.0, TAU), ParamRange::new(-1.0, 1.0)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-1.0, 1.0)],
    ]
}

#[test]
fn mixed_simple_repeated_folded_support_mints_four_guarded_members() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        mixed_folded_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected mixed contact topology, got {other:?}"),
    };
    let topology = crate::certify_skew_cylinder_folded_support_topologies(contact)
        .unwrap()
        .remove(0);
    assert_eq!(topology.root_ordinals(), [0, 2]);
    assert_eq!(topology.interior_touching_root_ordinal(), Some(1));
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            mixed_folded_support_windows(),
            [0, 1],
            1.0e-8,
            SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        mixed_folded_support_windows(),
        [0, 1],
        1.0e-8,
        SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.work(), SKEW_CYLINDER_MIXED_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(folded.formula_residuals().len(), 4);
    assert_eq!(folded.formula_branch_endpoints().len(), 4);
    let mut root_ports = folded
        .formula_branch_endpoints()
        .iter()
        .flatten()
        .filter_map(|endpoint| match endpoint {
            PersistentSkewCylinderFoldedSupportEndpoint::TouchingRoot {
                root_ordinal,
                continuation,
            } => Some((*root_ordinal, *continuation)),
            _ => None,
        })
        .collect::<Vec<_>>();
    root_ports.sort_unstable();
    assert_eq!(root_ports, vec![(1, 0), (1, 0), (1, 1), (1, 1)]);
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
}

#[test]
fn non_cardinal_seam_mixed_folded_support_mints_six_guarded_members() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        non_cardinal_seam_mixed_folded_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected non-cardinal mixed contact topology, got {other:?}"),
    };
    let topology = crate::certify_skew_cylinder_folded_support_topologies(contact)
        .unwrap()
        .remove(0);
    assert_eq!(topology.root_ordinals(), [1, 2]);
    assert_eq!(topology.interior_touching_root_ordinal(), Some(0));
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
    );
    let windows = [
        [ParamRange::new(0.0, TAU), ParamRange::new(-4.0, 4.0)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-4.0, 4.0)],
    ];
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            windows,
            [0, 1],
            1.0e-8,
            SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        windows,
        [0, 1],
        1.0e-8,
        SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(
        folded.work(),
        SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK
    );
    assert_eq!(folded.formula_residuals().len(), 6);
    assert_eq!(folded.formula_branch_endpoints().len(), 6);
    let mut root_ports = folded
        .formula_branch_endpoints()
        .iter()
        .flatten()
        .filter_map(|endpoint| match endpoint {
            PersistentSkewCylinderFoldedSupportEndpoint::TouchingRoot {
                root_ordinal,
                continuation,
            } => Some((*root_ordinal, *continuation)),
            _ => None,
        })
        .collect::<Vec<_>>();
    root_ports.sort_unstable();
    assert_eq!(root_ports, vec![(0, 0), (0, 0), (0, 1), (0, 1)]);
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
}

#[test]
fn mixed_folded_partition_follows_cyclic_root_order() {
    // Rotating the exact 3-4-5 radial direction moves the repeated root
    // through every quadrant. cos(u-alpha)=0 owns the two simple roots;
    // cos(u-alpha)=1 owns the touching root. No numerical root finder is
    // used to obtain these incidence counts or complete-cycle ordinals.
    for (x, y, simple, repeated, members) in [
        (0.6, 0.8, [1, 2], 0, 6),
        (-0.6, 0.8, [0, 2], 1, 4),
        (-0.6, -0.8, [0, 2], 1, 4),
        (0.6, -0.8, [0, 1], 2, 6),
    ] {
        let radial = Vec3::new(x, y, 0.0);
        let cylinders = [
            Cylinder::new(Frame::world(), 0.078125).unwrap(),
            Cylinder::new(
                Frame::new(radial * 0.0390625, Vec3::new(y, -x, 0.0), radial).unwrap(),
                0.0390625,
            )
            .unwrap(),
        ];
        let SkewCylinderExactDiscriminantTopology::Contact(contact) =
            classify_skew_cylinder_exact_discriminant(
                cylinders,
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )
            .unwrap()
        else {
            panic!("expected exact mixed topology at {x},{y}")
        };
        let topology = crate::certify_skew_cylinder_folded_support_topologies(*contact)
            .unwrap()
            .remove(0);
        assert_eq!(topology.root_ordinals(), simple);
        assert_eq!(topology.interior_touching_root_ordinal(), Some(repeated));
        let work = members
            * (PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
                + SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BOUND_WORK);
        assert_eq!(
            persistent_skew_cylinder_folded_support_exact_work(&topology),
            work
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                topology.clone(),
                touching_support_windows(),
                [0, 1],
                1.0e-8,
                work - 1
            )
            .is_err()
        );
        let certificate = certify_persistent_skew_cylinder_folded_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-8,
            work,
        )
        .unwrap_or_else(|error| panic!("mixed partition at {x},{y}: {error:?}"));
        assert_eq!(certificate.formula_residuals().len(), members as usize);
        assert_eq!(certificate.work(), work);
        assert!(certificate.required_edge_tolerance() <= 1.0e-8);
        let mut ports = certificate
            .formula_branch_endpoints()
            .iter()
            .flatten()
            .filter_map(|endpoint| {
                if let PersistentSkewCylinderFoldedSupportEndpoint::TouchingRoot {
                    root_ordinal,
                    continuation,
                } = endpoint
                {
                    Some((*root_ordinal, *continuation))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        ports.sort_unstable();
        assert_eq!(
            ports,
            [(repeated, 0), (repeated, 0), (repeated, 1), (repeated, 1)]
        );
    }
}

#[test]
fn non_cardinal_seam_mixed_endpoint_resolution_stays_fail_closed() {
    let frame = Frame::world();
    let second_axis = frame.x() * 0.8 - frame.y() * 0.6;
    let second_radial = frame.x() * 0.6 + frame.y() * 0.8;
    let cylinders = [
        Cylinder::new(frame, 1.25).unwrap(),
        Cylinder::new(
            Frame::new(
                frame.origin() + second_radial * 0.625,
                second_axis,
                second_radial,
            )
            .unwrap(),
            0.625,
        )
        .unwrap(),
    ];
    let contact = match classify_skew_cylinder_exact_discriminant(
        cylinders,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected scaled non-cardinal mixed topology, got {other:?}"),
    };
    let topology = crate::certify_skew_cylinder_folded_support_topologies(contact)
        .unwrap()
        .remove(0);
    let windows = [
        [ParamRange::new(0.0, TAU), ParamRange::new(-4.0, 4.0)],
        [ParamRange::new(0.0, TAU), ParamRange::new(-4.0, 4.0)],
    ];
    let session_resolution = certify_persistent_skew_cylinder_folded_support(
        topology.clone(),
        windows,
        [0, 1],
        1.0e-8,
        SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK,
    );
    assert!(matches!(
        session_resolution,
        Err(IntersectionCertificateError::InvalidTraceFamily)
    ));
    let tolerant = certify_persistent_skew_cylinder_folded_support(
        topology,
        windows,
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_SEAM_MIXED_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert!(tolerant.required_edge_tolerance() > 1.0e-8);
    assert!(tolerant.required_edge_tolerance() <= tolerant.tolerance());
}

#[test]
fn exact_isolated_support_root_mints_a_persistent_point_but_rooted_arc_does_not() {
    let exact = match classify_skew_cylinder_exact_discriminant(
        cylinders(3.0),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected contact topology, got {other:?}"),
    };
    assert_eq!(exact.roots().len(), 1);
    assert!(exact.roots()[0].repeated());
    let certified =
        certify_persistent_skew_cylinder_support_contact(exact, windows(), [0, 1], 1.0e-9, 0)
            .unwrap();
    assert!(certified.point().dist(Point3::new(0.0, 1.0, 0.0)) <= 1.0e-12);
    assert_eq!(certified.source_surface_parameters()[0][1], 0.0);

    let mut boundary_windows = windows();
    boundary_windows[0][1] = ParamRange::new(0.0, 1.0);
    let plan = plan_persistent_skew_cylinder_support_contact_boundaries(
        certified.topology(),
        boundary_windows,
        [0, 1],
        1.0e-9,
    );
    let plan = plan.unwrap();
    assert_eq!(plan.bits(), 1);
    assert_eq!(
        plan.work(),
        SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_support_contact(
            certified.topology().clone(),
            boundary_windows,
            [0, 1],
            1.0e-9,
            plan.work() - 1,
        )
        .is_err()
    );
    let boundary = certify_persistent_skew_cylinder_support_contact(
        certified.topology().clone(),
        boundary_windows,
        [0, 1],
        1.0e-9,
        plan.work(),
    )
    .unwrap();
    assert_eq!(
        boundary.source_axial_locations(),
        [
            PersistentSkewCylinderSupportContactAxialLocation::Lower,
            PersistentSkewCylinderSupportContactAxialLocation::Interior,
        ]
    );

    let rooted = match classify_skew_cylinder_exact_discriminant(
        cylinders(3.0_f64.next_down()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected rooted contact topology, got {other:?}"),
    };
    assert!(rooted.roots().len() >= 2);
    let folded = certify_skew_cylinder_folded_support_topology(rooted.clone()).unwrap();
    assert!(folded.roots().iter().all(|root| !root.repeated()));
    assert_eq!(folded.roots(), rooted.roots());
    assert_eq!(
        folded.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            folded.clone(),
            windows(),
            [0, 1],
            1.0e-9,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        folded,
        windows(),
        [0, 1],
        1.0e-9,
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.work(), SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert!(folded.guarded_ranges()[0].width() > 0.0);
    assert_eq!(
        folded
            .formula_residuals()
            .iter()
            .map(|residual| residual.sheet())
            .collect::<Vec<_>>(),
        vec![SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
    );
    assert!(folded.endpoint_points().iter().all(|point| point.y > 0.0));
    assert!(
        certify_persistent_skew_cylinder_support_contact(rooted, windows(), [0, 1], 1.0e-9, 0,)
            .is_err()
    );
}

#[test]
fn exact_isolated_support_root_on_authored_seam_mints_a_persistent_point() {
    let exact = match classify_skew_cylinder_exact_discriminant(
        seam_cylinders(3.0),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected seam contact topology, got {other:?}"),
    };
    let [root] = exact.roots() else {
        panic!("expected one isolated seam root")
    };
    let angular = root.angular_bracket();
    assert!(root.repeated());
    assert_eq!(angular.lo.to_bits(), 0.0_f64.to_bits());
    assert_eq!(angular.hi.to_bits(), 0.0_f64.to_bits());
    let certified =
        certify_persistent_skew_cylinder_support_contact(exact, windows(), [0, 1], 1.0e-9, 0)
            .unwrap();
    assert!(certified.point().dist(Point3::new(1.0, 0.0, 0.0)) <= 1.0e-12);
    assert_eq!(certified.carrier_parameter().to_bits(), 0.0_f64.to_bits());
    let longitudes = certified.source_longitude_enclosures();
    assert_eq!(longitudes[0].lo().to_bits(), 0.0_f64.to_bits());
    assert_eq!(longitudes[0].hi().to_bits(), 0.0_f64.to_bits());
    assert!(longitudes[1].lo() <= core::f64::consts::PI);
    assert!(longitudes[1].hi() >= core::f64::consts::PI);
}

#[test]
fn exact_isolated_support_root_on_opposite_authored_seam_mints_a_persistent_point() {
    let exact = match classify_skew_cylinder_exact_discriminant(
        seam_cylinders(-3.0),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected opposite-seam contact topology, got {other:?}"),
    };
    let [root] = exact.roots() else {
        panic!("expected one isolated opposite-seam root")
    };
    let angular = root.angular_bracket();
    assert!(root.repeated());
    assert_eq!(angular.lo.to_bits(), core::f64::consts::PI.to_bits());
    assert_eq!(angular.hi.to_bits(), core::f64::consts::PI.to_bits());
    let certified =
        certify_persistent_skew_cylinder_support_contact(exact, windows(), [0, 1], 1.0e-9, 0)
            .unwrap();
    assert!(certified.point().dist(Point3::new(-1.0, 0.0, 0.0)) <= 1.0e-12);
    assert_eq!(
        certified.carrier_parameter().to_bits(),
        core::f64::consts::PI.to_bits()
    );
    let longitudes = certified.source_longitude_enclosures();
    assert!(longitudes[0].lo() <= core::f64::consts::PI);
    assert!(longitudes[0].hi() >= core::f64::consts::PI);
    assert_eq!(longitudes[1].lo().to_bits(), 0.0_f64.to_bits());
    assert_eq!(longitudes[1].hi().to_bits(), TAU.to_bits());
    let parameters = certified.source_surface_parameters();
    assert_eq!(parameters[0][0].to_bits(), core::f64::consts::PI.to_bits());
    assert_eq!(parameters[1][0].to_bits(), 0.0_f64.to_bits());
}

#[test]
fn across_seam_folded_support_splits_into_four_exactly_joined_members() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        seam_cylinders(3.0_f64.next_down()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected rooted contact topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            windows(),
            [0, 1],
            1.0e-9,
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        windows(),
        [0, 1],
        1.0e-9,
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.formula_residuals().len(), 4);
    assert_eq!(folded.formula_branch_endpoints().len(), 4);
    assert_eq!(folded.work(), SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
    assert!(folded.seam_points().is_some());
    assert!(folded.source_seam_parameters().is_some());
    assert!(
        folded
            .guarded_ranges()
            .iter()
            .all(|range| range.width() > 0.0)
    );
}

#[test]
fn bounded_seam_folded_support_is_exact_rigid_frame_stable() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, 0.0),
    )
    .unwrap();
    for frame in [Frame::world(), rotated] {
        let direct = bounded_seam_cylinders(frame, 3.0_f64.next_down());
        for cylinders in [direct, [direct[1], direct[0]]] {
            let contact = match classify_skew_cylinder_exact_discriminant(
                cylinders,
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )
            .unwrap()
            {
                SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
                other => panic!("expected rooted contact topology, got {other:?}"),
            };
            let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
            let location = topology.positive_cell();
            let windows = if cylinders == direct {
                bounded_windows()
            } else {
                let windows = bounded_windows();
                [windows[1], windows[0]]
            };
            let roots = topology.roots();
            let root_evidence: [SupportRootEvidence; 2] = core::array::from_fn(|ordinal| {
                let root = roots[ordinal];
                root_evidence(topology.topology(), root, windows, false, true)
                    .unwrap_or_else(|error| {
                    panic!(
                        "reversed={} location={location:?} root {ordinal} {:?} evidence: {error:?}",
                        cylinders != direct,
                        root.angular_bracket(),
                    )
                    })
            });
            assert!(root_evidence.iter().all(|evidence| {
                evidence
                    .exact_heights
                    .into_iter()
                    .zip(windows)
                    .all(|(height, window)| strictly_inside(height, window[1]))
            }));
            let folded = certify_persistent_skew_cylinder_folded_support(
                topology,
                windows,
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "reversed={} location={location:?}: {error:?}",
                    cylinders != direct
                )
            });
            assert_eq!(
                folded.formula_residuals().len(),
                match location {
                    SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots => 2,
                    SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam => 4,
                }
            );
        }
    }
}

#[test]
fn repeated_positive_support_touch_mints_six_cross_sheet_members_atomically() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        touching_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected rooted contact topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
    assert!(topology.root().repeated());
    assert!(
        certify_persistent_skew_cylinder_touching_support(
            topology.clone(),
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let touching = certify_persistent_skew_cylinder_touching_support(
        topology,
        touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(touching.work(), SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK);
    assert_eq!(touching.formula_residuals().len(), 6);
    assert_eq!(touching.formula_branch_endpoints().len(), 6);
    assert!(
        touching
            .chart_join_longitude_for(PersistentSkewCylinderTouchingSupportChartJoin::First)
            .is_some()
    );
    assert_eq!(
        touching.chart_join_longitude_for(PersistentSkewCylinderTouchingSupportChartJoin::Second),
        None
    );
    let mut root_ports = touching
        .formula_branch_endpoints()
        .iter()
        .flatten()
        .filter_map(|endpoint| match endpoint {
            PersistentSkewCylinderTouchingSupportEndpoint::Root { continuation, .. } => {
                Some(*continuation)
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    root_ports.sort_unstable();
    assert_eq!(root_ports, vec![0, 0, 1, 1]);
    let first_root = PersistentSkewCylinderTouchingSupportEndpoint::Root {
        root: PersistentSkewCylinderTouchingSupportRoot::First,
        continuation: 0,
    };
    let unused_second_root = PersistentSkewCylinderTouchingSupportEndpoint::Root {
        root: PersistentSkewCylinderTouchingSupportRoot::Second,
        continuation: 0,
    };
    assert_eq!(
        touching.endpoint_point(unused_second_root),
        touching.endpoint_point(first_root)
    );
    assert_eq!(
        touching.source_parameters(unused_second_root),
        touching.source_parameters(first_root)
    );
    assert!(touching.required_edge_tolerance() <= touching.tolerance());
}

#[test]
fn repeated_positive_seam_touch_uses_two_chart_joins_per_sheet() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        seam_touching_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected seam-root contact topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
    assert_eq!(
        topology.root().angular_bracket(),
        crate::SkewCylinderAngularRootBracket { lo: 0.0, hi: 0.0 }
    );
    let touching = certify_persistent_skew_cylinder_touching_support(
        topology,
        touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(touching.formula_residuals().len(), 6);
    assert_eq!(
        touching.chart_join_longitudes(),
        &[
            core::f64::consts::FRAC_PI_2,
            3.0 * core::f64::consts::FRAC_PI_2,
        ]
    );
    assert_eq!(
        touching.chart_join_longitude_for(PersistentSkewCylinderTouchingSupportChartJoin::Second),
        Some(3.0 * core::f64::consts::FRAC_PI_2)
    );
    assert!(
        touching
            .formula_branch_endpoints()
            .iter()
            .flatten()
            .all(|endpoint| !matches!(
                endpoint,
                PersistentSkewCylinderTouchingSupportEndpoint::Seam(_)
            ))
    );
    assert!(touching.required_edge_tolerance() <= touching.tolerance());
}

#[test]
fn repeated_positive_opposite_pole_touch_uses_two_chart_joins_per_sheet() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        bounded_opposite_pole_touching_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected opposite-pole contact topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
    assert_eq!(
        topology.root().angular_bracket(),
        crate::SkewCylinderAngularRootBracket {
            lo: core::f64::consts::PI,
            hi: core::f64::consts::PI,
        }
    );
    assert!(
        certify_persistent_skew_cylinder_touching_support(
            topology.clone(),
            bounded_touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let touching = certify_persistent_skew_cylinder_touching_support(
        topology,
        bounded_touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(
        touching.work(),
        SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK
    );
    assert_eq!(touching.formula_residuals().len(), 8);
    assert_eq!(touching.formula_branch_endpoints().len(), 8);
    assert_eq!(
        touching.chart_join_longitudes(),
        &[
            core::f64::consts::FRAC_PI_2,
            3.0 * core::f64::consts::FRAC_PI_2,
        ]
    );
    let seam_endpoints = touching
        .formula_branch_endpoints()
        .iter()
        .flatten()
        .filter(|endpoint| {
            matches!(
                endpoint,
                PersistentSkewCylinderTouchingSupportEndpoint::Seam(_)
            )
        })
        .count();
    assert_eq!(seam_endpoints, 4);
    assert!(touching.required_edge_tolerance() <= touching.tolerance());
}

#[test]
fn two_repeated_positive_roots_mint_two_closed_crossing_curves() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        double_touching_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected double-touching contact topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
    assert_eq!(topology.roots().len(), 2);
    assert!(topology.roots().iter().all(|root| root.repeated()));
    let touching = certify_persistent_skew_cylinder_touching_support(
        topology,
        touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(touching.formula_residuals().len(), 6);
    assert_eq!(touching.formula_root_longitudes().len(), 2);
    assert!(touching.chart_join_longitudes().is_empty());
    let mut root_ports = touching
        .formula_branch_endpoints()
        .iter()
        .flatten()
        .filter_map(|endpoint| match endpoint {
            PersistentSkewCylinderTouchingSupportEndpoint::Root { root, continuation } => {
                Some((root.ordinal(), *continuation))
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    root_ports.sort_unstable();
    assert_eq!(
        root_ports,
        vec![
            (0, 0),
            (0, 0),
            (0, 1),
            (0, 1),
            (1, 0),
            (1, 0),
            (1, 1),
            (1, 1),
        ]
    );
    assert!(touching.required_edge_tolerance() <= touching.tolerance());
}

#[test]
fn simple_root_on_authored_seam_mints_four_chart_split_members() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        seam_root_folded_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected seam-root folded topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    assert_eq!(
        persistent_skew_cylinder_folded_support_exact_work(&topology),
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.work(), SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(folded.formula_residuals().len(), 4);
    assert_eq!(
        folded.formula_branch_endpoints(),
        [
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
            ],
        ]
    );
    assert_eq!(
        folded.chart_join_longitude(),
        Some(core::f64::consts::FRAC_PI_2)
    );
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
}

#[test]
fn seam_root_across_positive_cell_mints_four_chart_split_members() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        seam_root_across_folded_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected across-seam pole-pair topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
    );
    assert_eq!(
        persistent_skew_cylinder_folded_support_exact_work(&topology),
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.work(), SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(folded.formula_residuals().len(), 4);
    assert_eq!(
        folded.formula_branch_endpoints(),
        [
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
            ],
        ]
    );
    assert_eq!(
        folded.chart_join_longitude(),
        Some(3.0 * core::f64::consts::FRAC_PI_2)
    );
    assert!(folded.seam_points().is_none());
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
}

#[test]
fn short_positive_cell_from_simple_seam_root_mints_two_members() {
    let contact = match classify_skew_cylinder_exact_discriminant(
        short_seam_root_folded_support_cylinders(Frame::world()),
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected short seam-root folded topology, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    let angular = topology
        .roots()
        .map(SkewCylinderDiscriminantRoot::angular_bracket);
    assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
    assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
    assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
    );
    assert_eq!(
        persistent_skew_cylinder_folded_support_exact_work(&topology),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.work(), SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(folded.formula_residuals().len(), 2);
    assert_eq!(
        folded.formula_branch_endpoints(),
        [
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
            ],
        ]
    );
    assert_eq!(folded.chart_join_longitude(), None);
    assert!(folded.seam_points().is_none());
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
}

#[test]
fn perpendicular_semantic_discriminant_preserves_bounded_short_seam_root() {
    let cylinders = bounded_short_seam_root_folded_support_cylinders(Frame::world());
    let contact = match classify_skew_cylinder_exact_discriminant(
        cylinders,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected bounded short seam-root contact, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    let angular = topology
        .roots()
        .map(SkewCylinderDiscriminantRoot::angular_bracket);
    assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
    assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
    assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        bounded_touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.formula_residuals().len(), 2);
    assert!(
        folded
            .source_endpoint_parameters()
            .into_iter()
            .flatten()
            .all(|parameter| parameter[1] > 0.0 && parameter[1] < 1.0)
    );
}

#[test]
fn short_across_seam_non_pole_layout_mints_two_members() {
    let cylinders = bounded_short_seam_root_across_folded_support_cylinders(Frame::world());
    let contact = match classify_skew_cylinder_exact_discriminant(
        cylinders,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected bounded short across-seam contact, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    let angular = topology
        .roots()
        .map(SkewCylinderDiscriminantRoot::angular_bracket);
    assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
    assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
    assert!(angular[1].lo > core::f64::consts::PI && angular[1].hi < TAU);
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
    );
    assert_eq!(
        persistent_skew_cylinder_folded_support_exact_work(&topology),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            bounded_touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        bounded_touching_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(folded.work(), SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(folded.formula_residuals().len(), 2);
    assert_eq!(
        folded.formula_branch_endpoints(),
        [
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
            ],
        ]
    );
    assert_eq!(folded.chart_join_longitude(), None);
    assert!(folded.seam_points().is_none());
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
    assert!(
        folded
            .source_endpoint_parameters()
            .into_iter()
            .flatten()
            .all(|parameter| parameter[1] > 0.0 && parameter[1] < 1.0)
    );
}

#[test]
fn long_across_seam_non_pole_layout_mints_four_chart_split_members() {
    let cylinders = bounded_long_seam_root_across_folded_support_cylinders(Frame::world());
    let contact = match classify_skew_cylinder_exact_discriminant(
        cylinders,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected bounded long across-seam contact, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    let angular = topology
        .roots()
        .map(SkewCylinderDiscriminantRoot::angular_bracket);
    assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
    assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
    assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
    );
    assert_eq!(
        persistent_skew_cylinder_folded_support_exact_work(&topology),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            bounded_long_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        bounded_long_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(
        folded.work(),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert_eq!(folded.formula_residuals().len(), 4);
    assert_eq!(
        folded.formula_branch_endpoints(),
        [
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
            ],
        ]
    );
    assert_eq!(
        folded.chart_join_longitude(),
        Some(3.0 * core::f64::consts::FRAC_PI_2)
    );
    assert!(folded.seam_points().is_none());
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
    assert!(
        folded
            .source_endpoint_parameters()
            .into_iter()
            .flatten()
            .all(|parameter| parameter[1] > -0.5 && parameter[1] < 0.75)
    );
}

#[test]
fn long_between_seam_non_pole_layout_mints_four_chart_split_members() {
    let cylinders = bounded_long_seam_root_between_folded_support_cylinders(Frame::world());
    let contact = match classify_skew_cylinder_exact_discriminant(
        cylinders,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .unwrap()
    {
        SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
        other => panic!("expected bounded long between-roots contact, got {other:?}"),
    };
    let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
    let angular = topology
        .roots()
        .map(SkewCylinderDiscriminantRoot::angular_bracket);
    assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
    assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
    assert!(angular[1].lo > core::f64::consts::PI && angular[1].hi < TAU);
    assert_eq!(
        topology.positive_cell(),
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
    );
    assert_eq!(
        persistent_skew_cylinder_folded_support_exact_work(&topology),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(
        certify_persistent_skew_cylinder_folded_support(
            topology.clone(),
            bounded_long_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1,
        )
        .is_err()
    );
    let folded = certify_persistent_skew_cylinder_folded_support(
        topology,
        bounded_long_support_windows(),
        [0, 1],
        1.0e-7,
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
    )
    .unwrap();
    assert_eq!(
        folded.work(),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert_eq!(folded.formula_residuals().len(), 4);
    assert_eq!(
        folded.formula_branch_endpoints(),
        [
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Lower,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
            ],
            [
                PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(SkewCylinderSheet::Upper,),
                PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
            ],
        ]
    );
    assert_eq!(
        folded.chart_join_longitude(),
        Some(core::f64::consts::FRAC_PI_2)
    );
    assert!(folded.seam_points().is_none());
    assert!(folded.required_edge_tolerance() <= folded.tolerance());
    assert!(
        folded
            .source_endpoint_parameters()
            .into_iter()
            .flatten()
            .all(|parameter| parameter[1] > -0.5 && parameter[1] < 0.75)
    );
}
