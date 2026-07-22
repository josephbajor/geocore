//! Facade-only lifecycle evidence for separated and exact-contact finite cylinders.
//! Wall-time budget: less than 60 seconds for the rigid-frame/order matrix.

use super::*;

// Identity-copy precharge: body 1 + regions 2 + shell 1 + faces 6 +
// loops 4 + fin/pcurve pairs 8 + edge/curve pairs 4 = 26.
const ONE_CYLINDER_COPY_WORK: u64 = 26;
const TWO_CYLINDER_COPY_WORK: u64 = 2 * ONE_CYLINDER_COPY_WORK;
const ONE_CYLINDER_COPY_IDENTITIES: usize = 26;
const CYLINDER_RELATION_WORK: u64 = 64;

#[derive(Debug, Clone, Copy)]
struct CylinderSpec {
    radius: f64,
    radial_center: [f64; 2],
    axial: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
enum AxialRelationWitness {
    RadialSeparation,
    AxialSeparation,
    AxialContact,
    AxialOverlap,
}

#[derive(Debug, Clone, Copy)]
enum RadialRelation {
    Exterior,
    StrictSecant,
    StrictInternal,
    Tangent,
    RoundedTangent,
    InternalTangent,
    Coincident,
}

#[derive(Debug, Clone, Copy)]
struct CylinderRelationCase {
    name: &'static str,
    cylinders: [CylinderSpec; 2],
    witness: AxialRelationWitness,
    radial_relation: RadialRelation,
}

const RADIAL_DISJOINT: CylinderRelationCase = CylinderRelationCase {
    name: "exterior radial separation",
    cylinders: [
        CylinderSpec {
            radius: 0.75,
            radial_center: [0.0, 0.0],
            axial: [-1.0, 1.0],
        },
        CylinderSpec {
            radius: 1.25,
            radial_center: [1.5, 2.0],
            axial: [-1.0, 1.0],
        },
    ],
    witness: AxialRelationWitness::RadialSeparation,
    radial_relation: RadialRelation::Exterior,
};

const AXIAL_DISJOINT: [CylinderRelationCase; 4] = [
    CylinderRelationCase {
        name: "axial gap with strict-secant radial supports",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, -1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [1.0, 0.0],
                axial: [1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialSeparation,
        radial_relation: RadialRelation::StrictSecant,
    },
    CylinderRelationCase {
        name: "axial gap with strict-internal radial supports",
        cylinders: [
            CylinderSpec {
                radius: 2.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, -1.0],
            },
            CylinderSpec {
                radius: 0.5,
                radial_center: [0.3, 0.4],
                axial: [1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialSeparation,
        radial_relation: RadialRelation::StrictInternal,
    },
    CylinderRelationCase {
        name: "axial gap with tangent radial supports",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, -1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [2.0, 0.0],
                axial: [1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialSeparation,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "axial gap with coincident radial supports",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, -1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialSeparation,
        radial_relation: RadialRelation::Coincident,
    },
];

const AXIAL_CONTACT: [CylinderRelationCase; 4] = [
    CylinderRelationCase {
        name: "axial contact with strict-secant radial supports",
        cylinders: [
            CylinderSpec {
                radius: 2.0,
                radial_center: [0.0, 0.0],
                axial: [-1.0, 0.0],
            },
            CylinderSpec {
                radius: 2.0,
                radial_center: [2.0, 0.0],
                axial: [0.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialContact,
        radial_relation: RadialRelation::StrictSecant,
    },
    CylinderRelationCase {
        name: "axial contact with strict-internal radial supports",
        cylinders: [
            CylinderSpec {
                radius: 3.0,
                radial_center: [0.0, 0.0],
                axial: [-1.0, 0.0],
            },
            CylinderSpec {
                radius: 0.5,
                radial_center: [2.0, 0.0],
                axial: [0.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialContact,
        radial_relation: RadialRelation::StrictInternal,
    },
    CylinderRelationCase {
        name: "axial contact with tangent radial supports",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-1.0, 0.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [2.0, 0.0],
                axial: [0.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialContact,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "axial contact with coincident radial supports",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-1.0, 0.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [0.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialContact,
        radial_relation: RadialRelation::Coincident,
    },
];

const UNEQUAL_RADIUS_AXIAL_TANGENT: CylinderRelationCase = CylinderRelationCase {
    name: "axial contact with unequal-radius tangent supports",
    cylinders: [
        CylinderSpec {
            radius: 3.0,
            radial_center: [0.0, 0.0],
            axial: [-1.0, 0.0],
        },
        CylinderSpec {
            radius: 1.0,
            radial_center: [4.0, 0.0],
            axial: [0.0, 1.0],
        },
    ],
    witness: AxialRelationWitness::AxialContact,
    radial_relation: RadialRelation::Tangent,
};

// Positive-area contact relations are covered by the dedicated axial-contact
// shell suite. External tangency instead regularizes to two independently
// closed source-copy components because a point-connected shell is not a
// manifold boundary.
const AXIAL_TANGENT_CONTACTS: [CylinderRelationCase; 2] =
    [AXIAL_CONTACT[2], UNEQUAL_RADIUS_AXIAL_TANGENT];

const AXIAL_OVERLAP_TANGENT_CONTACTS: [CylinderRelationCase; 6] = [
    CylinderRelationCase {
        name: "equal-radius external tangency with nested axial overlap",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, 2.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [2.0, 0.0],
                axial: [-1.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "equal-radius external tangency with partial axial overlap",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, 1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [2.0, 0.0],
                axial: [-1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "equal-radius external tangency with one shared axial end",
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, 1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [2.0, 0.0],
                axial: [-1.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "unequal-radius external tangency with nested axial overlap",
        cylinders: [
            CylinderSpec {
                radius: 3.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, 2.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [4.0, 0.0],
                axial: [-1.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "unequal-radius external tangency with partial axial overlap",
        cylinders: [
            CylinderSpec {
                radius: 3.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, 1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [4.0, 0.0],
                axial: [-1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation: RadialRelation::Tangent,
    },
    CylinderRelationCase {
        name: "unequal-radius external tangency with one shared axial end",
        cylinders: [
            CylinderSpec {
                radius: 3.0,
                radial_center: [0.0, 0.0],
                axial: [-2.0, 1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [4.0, 0.0],
                axial: [-1.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation: RadialRelation::Tangent,
    },
];

const DISJOINT_CASES: [CylinderRelationCase; 5] = [
    RADIAL_DISJOINT,
    AXIAL_DISJOINT[0],
    AXIAL_DISJOINT[1],
    AXIAL_DISJOINT[2],
    AXIAL_DISJOINT[3],
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SetOperation {
    Intersect,
    Unite,
    Subtract,
}

impl SetOperation {
    const fn operation(self) -> BooleanOperation {
        match self {
            Self::Intersect => BooleanOperation::Intersect,
            Self::Unite => BooleanOperation::Unite,
            Self::Subtract => BooleanOperation::Subtract,
        }
    }

    const fn result_body_count(self) -> usize {
        match self {
            Self::Intersect => 0,
            Self::Unite => 2,
            Self::Subtract => 1,
        }
    }

    const fn realization_work(self) -> u64 {
        match self {
            Self::Intersect => 0,
            Self::Unite => TWO_CYLINDER_COPY_WORK,
            Self::Subtract => ONE_CYLINDER_COPY_WORK,
        }
    }
}

const SET_OPERATIONS: [SetOperation; 3] = [
    SetOperation::Intersect,
    SetOperation::Unite,
    SetOperation::Subtract,
];

struct OperationEvidence {
    exports: Vec<Vec<u8>>,
    report: kernel::OperationReport,
}

fn assert_radial_relation(
    name: &str,
    relation: RadialRelation,
    radii: [f64; 2],
    distance_squared: f64,
) {
    let sum_squared = (radii[0] + radii[1]).powi(2);
    let difference_squared = (radii[0] - radii[1]).powi(2);
    match relation {
        RadialRelation::Exterior => assert!(distance_squared > sum_squared, "{name}"),
        RadialRelation::StrictSecant => assert!(
            difference_squared < distance_squared && distance_squared < sum_squared,
            "{name}"
        ),
        RadialRelation::StrictInternal => assert!(
            distance_squared > 0.0 && distance_squared < difference_squared,
            "{name}"
        ),
        RadialRelation::Tangent => {
            assert_eq!(distance_squared.to_bits(), sum_squared.to_bits(), "{name}")
        }
        RadialRelation::RoundedTangent => {
            assert_eq!(distance_squared.to_bits(), sum_squared.to_bits(), "{name}");
            assert_ne!(
                kcore::expansion::two_sum(radii[0], radii[1]).1,
                0.0,
                "{name}: radius sum must retain a nonzero exact residual"
            );
        }
        RadialRelation::InternalTangent => {
            assert_eq!(
                distance_squared.to_bits(),
                difference_squared.to_bits(),
                "{name}"
            )
        }
        RadialRelation::Coincident => {
            assert_eq!(distance_squared.to_bits(), 0.0_f64.to_bits(), "{name}");
            assert_eq!(radii[0].to_bits(), radii[1].to_bits(), "{name}");
        }
    }
}

fn assert_transformed_radial_relation(
    name: &str,
    relation: RadialRelation,
    radii: [f64; 2],
    distance_squared: f64,
) {
    if matches!(relation, RadialRelation::Tangent) {
        let sum_squared = (radii[0] + radii[1]).powi(2);
        assert!(
            (distance_squared - sum_squared).abs() <= 1.0e-14,
            "{name}: transformed tangent distance={distance_squared}, expected={sum_squared}"
        );
    } else {
        assert_radial_relation(name, relation, radii, distance_squared);
    }
}

fn assert_certified_relation(case: CylinderRelationCase) {
    let [first, second] = case.cylinders;
    assert!(first.radius > 0.0 && second.radius > 0.0, "{}", case.name);
    assert!(first.axial[0] < first.axial[1], "{}", case.name);
    assert!(second.axial[0] < second.axial[1], "{}", case.name);
    match case.witness {
        AxialRelationWitness::RadialSeparation => {
            let dx = second.radial_center[0] - first.radial_center[0];
            let dy = second.radial_center[1] - first.radial_center[1];
            assert!(
                dx.powi(2) + dy.powi(2) > (first.radius + second.radius).powi(2),
                "{}",
                case.name
            );
        }
        AxialRelationWitness::AxialSeparation => assert!(
            first.axial[1] < second.axial[0] || second.axial[1] < first.axial[0],
            "{}",
            case.name
        ),
        AxialRelationWitness::AxialContact => assert_eq!(
            first.axial[1].to_bits(),
            second.axial[0].to_bits(),
            "{} must retain one exact shared axial endpoint",
            case.name
        ),
        AxialRelationWitness::AxialOverlap => assert!(
            first.axial[0].max(second.axial[0]) < first.axial[1].min(second.axial[1]),
            "{}",
            case.name
        ),
    }

    let dx = second.radial_center[0] - first.radial_center[0];
    let dy = second.radial_center[1] - first.radial_center[1];
    let distance_squared = dx.powi(2) + dy.powi(2);
    assert_radial_relation(
        case.name,
        case.radial_relation,
        [first.radius, second.radius],
        distance_squared,
    );
}

fn exact_contact_perpendicular(frame: Frame) -> Vec3 {
    let axis = frame.z();
    if axis.x == 0.0 && axis.y == 0.0 {
        Vec3::new(1.0, 0.0, 0.0)
    } else if axis.x == 0.0 {
        Vec3::new(1.0, 0.0, 0.0)
    } else if axis.y == 0.0 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(axis.y, -axis.x, 0.0)
    }
}

fn is_zero_or_power_of_two(value: f64) -> bool {
    let bits = value.abs().to_bits();
    let exponent = (bits >> 52) & 0x7ff;
    let significand = bits & ((1_u64 << 52) - 1);
    value == 0.0 || (exponent != 0 && exponent != 0x7ff && significand == 0)
}

fn authored_radius(case: CylinderRelationCase, frame: Frame, index: usize) -> f64 {
    let radius = case.cylinders[index].radius;
    if uses_exact_external_tangent_authorship(case) {
        radius * exact_contact_perpendicular(frame).norm()
    } else {
        radius
    }
}

fn uses_exact_external_tangent_authorship(case: CylinderRelationCase) -> bool {
    matches!(
        case.witness,
        AxialRelationWitness::AxialContact | AxialRelationWitness::AxialOverlap
    ) && matches!(case.radial_relation, RadialRelation::Tangent)
}

fn exact_tangent_axial_reference(case: CylinderRelationCase) -> f64 {
    match case.witness {
        AxialRelationWitness::AxialContact => case.cylinders[0].axial[1],
        AxialRelationWitness::AxialOverlap => 0.0,
        AxialRelationWitness::RadialSeparation | AxialRelationWitness::AxialSeparation => {
            unreachable!("exact tangent authorship requires contact or overlap")
        }
    }
}

fn authored_cylinders_with_directions(
    case: CylinderRelationCase,
    frame: Frame,
    reversed_axes: [bool; 2],
) -> [(Frame, f64, f64); 2] {
    if !uses_exact_external_tangent_authorship(case) {
        return core::array::from_fn(|index| {
            let cylinder = case.cylinders[index];
            let reversed = reversed_axes[index];
            let authored_start = cylinder.axial[usize::from(reversed)];
            let origin = frame.point_at(
                cylinder.radial_center[0],
                cylinder.radial_center[1],
                authored_start,
            );
            let cylinder_frame = if reversed {
                Frame::new(origin, -frame.z(), frame.x()).unwrap()
            } else {
                frame.with_origin(origin)
            };
            (
                cylinder_frame,
                cylinder.radius,
                cylinder.axial[1] - cylinder.axial[0],
            )
        });
    }

    let axial_reference = exact_tangent_axial_reference(case);
    if matches!(case.witness, AxialRelationWitness::AxialContact) {
        assert_eq!(
            axial_reference.to_bits(),
            case.cylinders[1].axial[0].to_bits(),
            "{}",
            case.name
        );
    }
    let axis = frame.z();
    let perpendicular = exact_contact_perpendicular(frame);
    let mut centers = [Point3::new(0.0, 0.0, 0.0); 2];
    let authored = core::array::from_fn(|index| {
        let cylinder = case.cylinders[index];
        assert_eq!(
            cylinder.radial_center[1].to_bits(),
            0.0_f64.to_bits(),
            "{} contact radial scale must use one exact perpendicular",
            case.name
        );
        if matches!(case.radial_relation, RadialRelation::Tangent) {
            assert!(
                is_zero_or_power_of_two(cylinder.radial_center[0]),
                "{} tangent radial scale must be zero or a power of two",
                case.name
            );
        }
        let radial = perpendicular * cylinder.radial_center[0];
        let frame_origin = frame.origin();
        centers[index] = Point3::new(
            frame_origin.x + radial.x,
            frame_origin.y + radial.y,
            frame_origin.z + radial.z,
        );
        let reversed = reversed_axes[index];
        let authored_start = cylinder.axial[usize::from(reversed)] - axial_reference;
        let origin = Point3::new(
            centers[index].x + axis.x * authored_start,
            centers[index].y + axis.y * authored_start,
            centers[index].z + axis.z * authored_start,
        );
        let cylinder_frame = if reversed {
            Frame::new(origin, -axis, frame.x()).unwrap()
        } else {
            frame.with_origin(origin)
        };
        (
            cylinder_frame,
            authored_radius(case, frame, index),
            cylinder.axial[1] - cylinder.axial[0],
        )
    });
    let displacement = centers[1] - centers[0];
    assert_transformed_radial_relation(
        case.name,
        case.radial_relation,
        [
            authored_radius(case, frame, 0),
            authored_radius(case, frame, 1),
        ],
        displacement.x.powi(2) + displacement.y.powi(2) + displacement.z.powi(2),
    );
    authored
}

fn fixture(case: CylinderRelationCase, placement: Placement, antiparallel: bool) -> Fixture {
    fixture_with_directions(case, placement, [false, antiparallel])
}

fn fixture_with_directions(
    case: CylinderRelationCase,
    placement: Placement,
    reversed_axes: [bool; 2],
) -> Fixture {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let frame = if uses_exact_external_tangent_authorship(case)
        && matches!(placement, Placement::Oblique)
    {
        // A genuinely tilted axis with an exact coordinate-basis radial
        // direction. The zero y component makes world y exactly
        // perpendicular even though the normalized x/z axis is non-dyadic.
        Frame::new(
            Point3::new(0.0, 8.0, 0.0),
            Vec3::new(0.6, 0.0, 0.8),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap()
    } else if uses_exact_external_tangent_authorship(case) {
        shared_frame(placement).with_origin(origin)
    } else {
        shared_frame(placement)
    };
    fixture_with_frame(case, frame, reversed_axes)
}

fn fixture_with_frame(
    case: CylinderRelationCase,
    frame: Frame,
    reversed_axes: [bool; 2],
) -> Fixture {
    let authored = authored_cylinders_with_directions(case, frame, reversed_axes);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let mut bodies = Vec::with_capacity(2);
        for (cylinder_frame, radius, height) in authored {
            bodies.push(
                edit.create_cylinder(CylinderRequest::new(cylinder_frame, radius, height))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body(),
            );
        }
        (bodies.remove(0), bodies.remove(0))
    };
    Fixture {
        session,
        part_id,
        outer,
        inner,
        frame,
    }
}

fn run_set_operation(
    fixture: &mut Fixture,
    operation: SetOperation,
    swapped: bool,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    let bodies = if swapped {
        [fixture.inner.clone(), fixture.outer.clone()]
    } else {
        [fixture.outer.clone(), fixture.inner.clone()]
    };
    fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(operation.operation(), bodies[0].clone(), bodies[1].clone())
                .with_settings(settings),
        )
        .unwrap()
}

fn usage_at(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    resource: ResourceKind,
) -> Option<u64> {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == resource)
        .map(|usage| usage.consumed)
}

fn assert_full_valid(created: &kernel::BooleanCreatedResult) {
    assert_eq!(created.reports().len(), created.bodies().len());
    for (report, body) in created.reports().iter().zip(created.bodies()) {
        assert_eq!(report.body(), *body);
        assert_eq!(report.report().level(), CheckLevel::Full);
        assert_eq!(report.report().outcome(), CheckOutcome::Valid);
        assert!(report.report().faults().is_empty());
        assert!(report.report().gaps().is_empty());
    }
}

fn source_copy_lineage(fixture: &Fixture, created: &kernel::BooleanCreatedResult) -> Vec<BodyId> {
    assert_eq!(created.journal().part(), fixture.part_id);
    let mutations = created.journal().mutations().collect::<Vec<_>>();
    assert!(!mutations.is_empty());
    assert!(
        mutations
            .iter()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    assert_eq!(created.journal().lineage_count(), mutations.len());
    assert_eq!(
        mutations.len(),
        ONE_CYLINDER_COPY_IDENTITIES * created.bodies().len()
    );

    let mut derived = Vec::with_capacity(mutations.len());
    let mut body_pairs = Vec::new();
    let mut face_pairs = Vec::new();
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: derived_entity,
            source,
        } = event
        else {
            panic!("whole-cylinder copy lineage must contain only DerivedFrom events")
        };
        assert!(!derived.contains(&derived_entity));
        assert_eq!(derived_entity.kind(), source.kind());
        derived.push(derived_entity.clone());
        match (derived_entity, source) {
            (JournalEntity::Body(result), JournalEntity::Body(source)) => {
                body_pairs.push((result, source));
            }
            (JournalEntity::Face(result), JournalEntity::Face(source)) => {
                face_pairs.push((result, source));
            }
            _ => {}
        }
    }
    assert!(
        mutations
            .iter()
            .all(|mutation| derived.contains(mutation.entity()))
    );
    for (kind, identities_per_source) in [
        (EntityKind::Body, 1),
        (EntityKind::Region, 2),
        (EntityKind::Shell, 1),
        (EntityKind::Face, 3),
        (EntityKind::Loop, 4),
        (EntityKind::Fin, 4),
        (EntityKind::Edge, 2),
        (EntityKind::Vertex, 0),
        (EntityKind::Curve, 2),
        (EntityKind::Surface, 3),
        (EntityKind::Point, 0),
        (EntityKind::Pcurve, 4),
    ] {
        assert_eq!(
            derived
                .iter()
                .filter(|entity| entity.kind() == kind)
                .count(),
            identities_per_source * created.bodies().len(),
            "unexpected {kind:?} copy inventory"
        );
    }
    assert_eq!(body_pairs.len(), created.bodies().len());
    assert_eq!(
        body_pairs
            .iter()
            .map(|(result, _)| result.clone())
            .collect::<Vec<_>>(),
        created.bodies()
    );

    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    for (result, source) in &body_pairs {
        assert_ne!(result, source);
        let result_faces = part
            .body(result.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        let source_faces = part
            .body(source.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        assert_eq!(result_faces.len(), source_faces.len());
        assert!(result_faces.iter().all(|result_face| {
            face_pairs
                .iter()
                .filter(|(derived, source)| derived == result_face && source_faces.contains(source))
                .count()
                == 1
        }));
    }
    body_pairs.into_iter().map(|(_, source)| source).collect()
}

fn assert_analytic_cylinder(
    fixture: &Fixture,
    case: CylinderRelationCase,
    body: BodyId,
    source: BodyId,
) {
    let (index, cylinder) = if source == fixture.outer {
        (0, case.cylinders[0])
    } else if source == fixture.inner {
        (1, case.cylinders[1])
    } else {
        panic!("whole-cylinder result escaped both source bodies")
    };
    let height = cylinder.axial[1] - cylinder.axial[0];
    let radius = authored_radius(case, fixture.frame, index);
    let centroid = if uses_exact_external_tangent_authorship(case) {
        let perpendicular = exact_contact_perpendicular(fixture.frame);
        let radial = perpendicular * cylinder.radial_center[0];
        let axial =
            (cylinder.axial[0] + cylinder.axial[1]) / 2.0 - exact_tangent_axial_reference(case);
        let origin = fixture.frame.origin();
        let axis = fixture.frame.z();
        Point3::new(
            origin.x + radial.x + axis.x * axial,
            origin.y + radial.y + axis.y * axial,
            origin.z + radial.z + axis.z * axial,
        )
    } else {
        fixture.frame.point_at(
            cylinder.radial_center[0],
            cylinder.radial_center[1],
            (cylinder.axial[0] + cylinder.axial[1]) / 2.0,
        )
    };
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, body.clone()), CYLINDER_TOPOLOGY);
    let outcome = part
        .body_properties(BodyPropertiesRequest::new(body))
        .unwrap();
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = outcome.into_result().unwrap()
    else {
        panic!("whole-cylinder copy properties were not certified")
    };
    assert_eq!(full_check.level(), CheckLevel::Full);
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert_scalar_matches_analytic(
        properties.volume(),
        core::f64::consts::PI * radius.powi(2) * height,
        "whole-cylinder volume",
    );
    assert_scalar_matches_analytic(
        properties.surface_area(),
        2.0 * core::f64::consts::PI * radius * height
            + 2.0 * core::f64::consts::PI * radius.powi(2),
        "whole-cylinder surface area",
    );
    assert_point_matches_analytic(properties.centroid(), centroid);
    let volume = core::f64::consts::PI * radius.powi(2) * height;
    let axial = volume * radius.powi(2) / 2.0;
    let transverse = volume * (3.0 * radius.powi(2) + height.powi(2)) / 12.0;
    let axis = fixture.frame.z().to_array();
    let inertia = core::array::from_fn(|row| {
        core::array::from_fn(|column| {
            let identity = if row == column { 1.0 } else { 0.0 };
            transverse * identity + (axial - transverse) * axis[row] * axis[column]
        })
    });
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        inertia,
    );
}

fn deterministic_exports(fixture: &mut Fixture, bodies: &[BodyId]) -> Vec<Vec<u8>> {
    let exports = {
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        bodies
            .iter()
            .map(|body| {
                let first = part
                    .export_xt(ExportXtRequest::new(body.clone()))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let second = part
                    .export_xt(ExportXtRequest::new(body.clone()))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(first.bytes(), second.bytes());
                first.bytes().to_vec()
            })
            .collect::<Vec<_>>()
    };
    for bytes in &exports {
        assert_fast_self_import(&mut fixture.session, bytes);
    }
    exports
}

fn assert_created_source_copies(
    fixture: &mut Fixture,
    case: CylinderRelationCase,
    operation: SetOperation,
    swapped: bool,
    created: kernel::BooleanCreatedResult,
) -> Vec<Vec<u8>> {
    assert_eq!(created.bodies().len(), operation.result_body_count());
    assert_full_valid(&created);
    let sources = source_copy_lineage(fixture, &created);
    match operation {
        SetOperation::Intersect => unreachable!(),
        SetOperation::Unite => {
            assert_eq!(sources, [fixture.outer.clone(), fixture.inner.clone()]);
        }
        SetOperation::Subtract if swapped => {
            assert_eq!(sources, [fixture.inner.clone()]);
        }
        SetOperation::Subtract => {
            assert_eq!(sources, [fixture.outer.clone()]);
        }
    }
    let bodies = created.bodies().to_vec();
    for (body, source) in bodies.iter().cloned().zip(sources) {
        assert_analytic_cylinder(fixture, case, body, source);
    }
    let exports = deterministic_exports(fixture, &bodies);
    assert_source_bodies_preserved(fixture, 2 + operation.result_body_count());
    exports
}

fn assert_success(
    fixture: &mut Fixture,
    case: CylinderRelationCase,
    operation: SetOperation,
    swapped: bool,
    before: FixtureSignature,
    outcome: OperationOutcome<BooleanOutcome>,
) -> OperationEvidence {
    assert_eq!(
        usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(CYLINDER_RELATION_WORK),
        "{} {operation:?}",
        case.name
    );
    let realization_work = usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work);
    let realized_vertices = usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items);
    let report = outcome.report().clone();
    let result = outcome.into_result().unwrap();
    if operation == SetOperation::Intersect {
        let BooleanOutcome::Success(BooleanResult::ProvenEmpty) = result else {
            panic!("{} Intersect returned {result:#?}", case.name)
        };
        assert_eq!(realization_work, Some(0), "{}", case.name);
        assert_eq!(realized_vertices, Some(0), "{}", case.name);
        assert_eq!(fixture_signature(fixture), before, "{}", case.name);
        assert_source_bodies_preserved(fixture, 2);
        return OperationEvidence {
            exports: Vec::new(),
            report,
        };
    }

    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("{} {operation:?} returned {result:#?}", case.name)
    };
    assert_eq!(
        realization_work,
        Some(operation.realization_work()),
        "{} {operation:?}",
        case.name
    );
    assert_eq!(realized_vertices, Some(0), "{} {operation:?}", case.name);
    let exports = assert_created_source_copies(fixture, case, operation, swapped, created);
    OperationEvidence { exports, report }
}

fn assert_tangent_axial_contact(
    fixture: &mut Fixture,
    case: CylinderRelationCase,
    operation: SetOperation,
    swapped: bool,
    before: FixtureSignature,
    outcome: OperationOutcome<BooleanOutcome>,
) -> OperationEvidence {
    assert_eq!(
        usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(CYLINDER_RELATION_WORK),
        "{} {operation:?}",
        case.name
    );
    let realization_work = usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work);
    let realized_vertices = usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items);
    let report = outcome.report().clone();
    let result = outcome.into_result().unwrap();
    let exports = match operation {
        SetOperation::Intersect => {
            let BooleanOutcome::Success(BooleanResult::ProvenEmpty) = result else {
                panic!(
                    "{} frame={:?} swapped={swapped} Intersect returned {result:#?}",
                    case.name, fixture.frame
                )
            };
            assert_eq!(realization_work, Some(0), "{}", case.name);
            assert_eq!(realized_vertices, Some(0), "{}", case.name);
            assert_eq!(fixture_signature(fixture), before, "{}", case.name);
            assert_source_bodies_preserved(fixture, 2);
            Vec::new()
        }
        SetOperation::Unite => {
            assert_eq!(
                realization_work,
                Some(TWO_CYLINDER_COPY_WORK),
                "{}",
                case.name
            );
            assert_eq!(realized_vertices, Some(0), "{}", case.name);
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!(
                    "{} frame={:?} swapped={swapped} Unite returned {result:#?}",
                    case.name, fixture.frame
                )
            };
            assert_created_source_copies(fixture, case, operation, swapped, created)
        }
        SetOperation::Subtract => {
            assert_eq!(
                realization_work,
                Some(ONE_CYLINDER_COPY_WORK),
                "{}",
                case.name
            );
            assert_eq!(realized_vertices, Some(0), "{}", case.name);
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("{} Subtract returned {result:#?}", case.name)
            };
            assert_created_source_copies(fixture, case, operation, swapped, created)
        }
    };
    OperationEvidence { exports, report }
}

fn assert_same_evidence(actual: &OperationEvidence, expected: &OperationEvidence, label: &str) {
    assert_eq!(actual.report, expected.report, "{label}: report changed");
    assert_eq!(actual.exports.len(), expected.exports.len(), "{label}");
    for (actual, expected) in actual.exports.iter().zip(&expected.exports) {
        assert_xt_equal(actual, expected, label);
    }
}

fn exercise_operation_matrix(
    case: CylinderRelationCase,
    placement: Placement,
    antiparallel: bool,
    assert_outcome: fn(
        &mut Fixture,
        CylinderRelationCase,
        SetOperation,
        bool,
        FixtureSignature,
        OperationOutcome<BooleanOutcome>,
    ) -> OperationEvidence,
) -> usize {
    exercise_operation_matrix_with_directions(
        case,
        placement,
        [false, antiparallel],
        assert_outcome,
    )
}

fn exercise_operation_matrix_with_directions(
    case: CylinderRelationCase,
    placement: Placement,
    reversed_axes: [bool; 2],
    assert_outcome: fn(
        &mut Fixture,
        CylinderRelationCase,
        SetOperation,
        bool,
        FixtureSignature,
        OperationOutcome<BooleanOutcome>,
    ) -> OperationEvidence,
) -> usize {
    assert_certified_relation(case);
    let mut executions = 0;
    for operation in SET_OPERATIONS {
        let mut canonical: [Option<OperationEvidence>; 2] = [None, None];
        for swapped in [false, true] {
            let canonical_index = usize::from(operation == SetOperation::Subtract && swapped);
            for repeat in 0..2 {
                let mut fixture = fixture_with_directions(case, placement, reversed_axes);
                let before = fixture_signature(&fixture);
                assert_source_bodies_preserved(&fixture, 2);
                let outcome =
                    run_set_operation(&mut fixture, operation, swapped, OperationSettings::new());
                let evidence =
                    assert_outcome(&mut fixture, case, operation, swapped, before, outcome);
                let label = format!(
                    "{} {placement:?} reversed_axes={reversed_axes:?} {operation:?} swapped={swapped} repeat={repeat}",
                    case.name
                );
                if let Some(expected) = canonical[canonical_index].as_ref() {
                    assert_same_evidence(&evidence, expected, &label);
                } else {
                    canonical[canonical_index] = Some(evidence);
                }
                executions += 1;
            }
        }
    }
    executions
}

#[test]
fn certified_radial_and_axial_disjointness_realize_the_same_deterministic_set_contract() {
    let mut executions = 0;
    for case in DISJOINT_CASES {
        for placement in [Placement::World, Placement::Oblique] {
            for antiparallel in [false, true] {
                executions +=
                    exercise_operation_matrix(case, placement, antiparallel, assert_success);
            }
        }
    }
    assert_eq!(executions, 240);
}

#[test]
fn exact_external_tangent_axial_contact_has_deterministic_regularized_set_semantics() {
    let mut executions = 0;
    for case in AXIAL_TANGENT_CONTACTS {
        for placement in [Placement::World, Placement::Oblique] {
            for reversed_axes in [[false, false], [false, true], [true, false], [true, true]] {
                executions += exercise_operation_matrix_with_directions(
                    case,
                    placement,
                    reversed_axes,
                    assert_tangent_axial_contact,
                );
            }
        }
    }
    assert_eq!(executions, 192);
}

#[test]
fn exact_external_tangent_positive_axial_overlap_has_deterministic_regularized_set_semantics() {
    let mut executions = 0;
    for case in AXIAL_OVERLAP_TANGENT_CONTACTS {
        for placement in [Placement::World, Placement::Oblique] {
            for reversed_axes in [[false, false], [false, true], [true, false], [true, true]] {
                executions += exercise_operation_matrix_with_directions(
                    case,
                    placement,
                    reversed_axes,
                    assert_tangent_axial_contact,
                );
            }
        }
    }
    assert_eq!(executions, 576);
}

fn axial_boundary_case(name: &'static str, second_lower: f64) -> CylinderRelationCase {
    let witness = if second_lower > 1.0 {
        AxialRelationWitness::AxialSeparation
    } else if second_lower.to_bits() == 1.0_f64.to_bits() {
        AxialRelationWitness::AxialContact
    } else {
        AxialRelationWitness::AxialOverlap
    };
    CylinderRelationCase {
        name,
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [0.0, 1.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [2.0, 0.0],
                axial: [second_lower, 1.5],
            },
        ],
        witness,
        radial_relation: RadialRelation::Tangent,
    }
}

fn assert_refusal_matrix_with_directions(
    case: CylinderRelationCase,
    reversed_axes: [bool; 2],
) -> usize {
    assert_certified_relation(case);
    let mut executions = 0;
    for operation in SET_OPERATIONS {
        let mut canonical: [Option<kernel::OperationReport>; 2] = [None, None];
        for swapped in [false, true] {
            let canonical_index = usize::from(operation == SetOperation::Subtract && swapped);
            for repeat in 0..2 {
                let mut fixture = fixture_with_directions(case, Placement::World, reversed_axes);
                let before = fixture_signature(&fixture);
                let outcome =
                    run_set_operation(&mut fixture, operation, swapped, OperationSettings::new());
                let report = outcome.report().clone();
                let result = outcome.into_result().unwrap();
                assert!(
                    matches!(result, BooleanOutcome::Refused(_)),
                    "{} reversed_axes={reversed_axes:?} {operation:?} swapped={swapped} repeat={repeat} returned {result:#?}",
                    case.name
                );
                assert_eq!(fixture_signature(&fixture), before, "{}", case.name);
                assert_source_bodies_preserved(&fixture, 2);
                if let Some(expected) = canonical[canonical_index].as_ref() {
                    assert_eq!(&report, expected, "{} report changed", case.name);
                } else {
                    canonical[canonical_index] = Some(report);
                }
                executions += 1;
            }
        }
    }
    executions
}

#[test]
fn exact_axial_boundary_preserves_gap_contact_and_positive_overlap_set_semantics() {
    let boundary = 1.0_f64;
    let gap = axial_boundary_case("one ULP axial gap", boundary.next_up());
    let contact = axial_boundary_case("exact axial contact", boundary);
    let overlap = axial_boundary_case("one ULP axial overlap", boundary.next_down());
    assert!(gap.cylinders[0].axial[1] < gap.cylinders[1].axial[0]);
    assert_eq!(contact.cylinders[0].axial[1], contact.cylinders[1].axial[0]);
    assert!(overlap.cylinders[0].axial[1] > overlap.cylinders[1].axial[0]);
    assert_certified_relation(contact);

    let mut executions = 0;
    for antiparallel in [false, true] {
        executions +=
            exercise_operation_matrix(gap, Placement::World, antiparallel, assert_success);
        executions += exercise_operation_matrix(
            contact,
            Placement::World,
            antiparallel,
            assert_tangent_axial_contact,
        );
        executions += exercise_operation_matrix(
            overlap,
            Placement::World,
            antiparallel,
            assert_tangent_axial_contact,
        );
    }
    assert_eq!(executions, 72);
}

fn radial_boundary_case(
    name: &'static str,
    second_radial: f64,
    radial_relation: RadialRelation,
) -> CylinderRelationCase {
    CylinderRelationCase {
        name,
        cylinders: [
            CylinderSpec {
                radius: 1.0,
                radial_center: [0.0, 0.0],
                axial: [-1.0, 0.0],
            },
            CylinderSpec {
                radius: 1.0,
                radial_center: [second_radial, 0.0],
                axial: [0.0, 1.0],
            },
        ],
        witness: AxialRelationWitness::AxialContact,
        radial_relation,
    }
}

fn radial_overlap_boundary_case(
    name: &'static str,
    radii: [f64; 2],
    second_radial: f64,
    radial_relation: RadialRelation,
) -> CylinderRelationCase {
    CylinderRelationCase {
        name,
        cylinders: [
            CylinderSpec {
                radius: radii[0],
                radial_center: [0.0, 0.0],
                axial: [-2.0, 1.0],
            },
            CylinderSpec {
                radius: radii[1],
                radial_center: [second_radial, 0.0],
                axial: [-1.0, 2.0],
            },
        ],
        witness: AxialRelationWitness::AxialOverlap,
        radial_relation,
    }
}

fn boundary_contact_unite_refusal(
    fixture: Fixture,
    settings: OperationSettings,
    label: &str,
) -> kernel::OperationReport {
    unite_refusal(fixture, settings, BooleanRefusal::BoundaryContact, label)
}

fn unite_refusal(
    mut fixture: Fixture,
    settings: OperationSettings,
    expected: BooleanRefusal,
    label: &str,
) -> kernel::OperationReport {
    let before = fixture_signature(&fixture);
    let outcome = run_set_operation(&mut fixture, SetOperation::Unite, false, settings);
    assert_eq!(
        usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(CYLINDER_RELATION_WORK),
        "{label}"
    );
    assert_eq!(
        usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(0),
        "{label}"
    );
    assert_eq!(
        usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items),
        Some(0),
        "{label}"
    );
    let report = outcome.report().clone();
    let BooleanOutcome::Refused(actual) = outcome.into_result().unwrap() else {
        panic!("{label}: Unite did not refuse")
    };
    assert_eq!(actual, expected, "{label}");
    assert_eq!(fixture_signature(&fixture), before, "{label}");
    assert_source_bodies_preserved(&fixture, 2);
    report
}

fn tangent_unite_evidence(
    case: CylinderRelationCase,
    placement: Placement,
    reversed_axes: [bool; 2],
    settings: OperationSettings,
) -> OperationEvidence {
    let mut fixture = fixture_with_directions(case, placement, reversed_axes);
    let before = fixture_signature(&fixture);
    let outcome = run_set_operation(&mut fixture, SetOperation::Unite, false, settings);
    assert_tangent_axial_contact(
        &mut fixture,
        case,
        SetOperation::Unite,
        false,
        before,
        outcome,
    )
}

#[test]
fn exact_external_tangency_is_not_inferred_from_one_ulp_or_resolution_near_supports() {
    let boundary = 2.0_f64;
    let exterior = radial_boundary_case(
        "one ULP exterior radial support",
        boundary.next_up(),
        RadialRelation::Exterior,
    );
    let interior = radial_boundary_case(
        "one ULP interior radial support",
        boundary.next_down(),
        RadialRelation::StrictSecant,
    );
    assert_certified_relation(exterior);
    assert_certified_relation(interior);

    let loose = OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap());
    let mut exterior_baseline = fixture_with_directions(exterior, Placement::World, [false, true]);
    let before = fixture_signature(&exterior_baseline);
    let outcome = run_set_operation(
        &mut exterior_baseline,
        SetOperation::Unite,
        false,
        OperationSettings::new(),
    );
    let exterior_baseline = assert_success(
        &mut exterior_baseline,
        exterior,
        SetOperation::Unite,
        false,
        before,
        outcome,
    );
    let mut exterior_loose = fixture_with_directions(exterior, Placement::World, [false, true]);
    let before = fixture_signature(&exterior_loose);
    let outcome = run_set_operation(
        &mut exterior_loose,
        SetOperation::Unite,
        false,
        loose.clone(),
    );
    let exterior_loose = assert_success(
        &mut exterior_loose,
        exterior,
        SetOperation::Unite,
        false,
        before,
        outcome,
    );
    assert_same_evidence(
        &exterior_loose,
        &exterior_baseline,
        "loose tolerance changed the one-ULP exterior result",
    );

    let interior_baseline = boundary_contact_unite_refusal(
        fixture_with_directions(interior, Placement::World, [false, true]),
        OperationSettings::new(),
        interior.name,
    );
    let interior_loose = boundary_contact_unite_refusal(
        fixture_with_directions(interior, Placement::World, [false, true]),
        loose.clone(),
        interior.name,
    );
    assert_eq!(interior_loose, interior_baseline);

    // The historical all-nonzero oblique construction is mathematically
    // tangent before f64 storage, but its exact dyadic geometry is slightly
    // overlapping. It must remain a fail-closed near case, not acquire the
    // exact tangent source-copy contract through operation tolerance.
    let all_nonzero = shared_frame(Placement::Oblique).with_origin(Point3::new(0.0, 0.0, 0.0));
    let near_baseline = boundary_contact_unite_refusal(
        fixture_with_frame(AXIAL_CONTACT[2], all_nonzero, [false, true]),
        OperationSettings::new(),
        "all-nonzero oblique near tangent",
    );
    let near_loose = boundary_contact_unite_refusal(
        fixture_with_frame(AXIAL_CONTACT[2], all_nonzero, [false, true]),
        loose,
        "all-nonzero oblique near tangent",
    );
    assert_eq!(near_loose, near_baseline);
}

#[test]
fn positive_overlap_external_tangency_is_not_inferred_from_near_or_internal_supports() {
    let boundary = 2.0_f64;
    let exterior = radial_overlap_boundary_case(
        "positive overlap with one ULP exterior radial support",
        [1.0, 1.0],
        boundary.next_up(),
        RadialRelation::Exterior,
    );
    let interior = radial_overlap_boundary_case(
        "positive overlap with one ULP interior radial support",
        [1.0, 1.0],
        boundary.next_down(),
        RadialRelation::StrictSecant,
    );
    let internal_tangent = radial_overlap_boundary_case(
        "positive overlap with exact internal tangency",
        [3.0, 1.0],
        2.0,
        RadialRelation::InternalTangent,
    );
    let rounded_radius_sum = 1.0 + 0.2;
    let rounded_sum = radial_overlap_boundary_case(
        "positive overlap with unrepresentable radius-sum equality",
        [1.0, 0.2],
        rounded_radius_sum,
        RadialRelation::RoundedTangent,
    );
    for case in [exterior, interior, internal_tangent, rounded_sum] {
        assert_certified_relation(case);
    }

    let mut executions = 0;
    for reversed_axes in [[false, false], [false, true], [true, false], [true, true]] {
        executions += exercise_operation_matrix_with_directions(
            exterior,
            Placement::World,
            reversed_axes,
            assert_success,
        );
        for refused in [interior, internal_tangent, rounded_sum] {
            executions += assert_refusal_matrix_with_directions(refused, reversed_axes);
        }
    }
    assert_eq!(executions, 192);

    // This all-nonzero authored frame is tangent over real arithmetic, but
    // normalization stores a rounded dyadic support with a tiny radial
    // overlap. Operation tolerance must not promote it to exact tangency.
    let all_nonzero = shared_frame(Placement::Oblique);
    let baseline = unite_refusal(
        fixture_with_frame(
            AXIAL_OVERLAP_TANGENT_CONTACTS[0],
            all_nonzero,
            [false, true],
        ),
        OperationSettings::new(),
        BooleanRefusal::CurvedResultTopologyUnsupported,
        "positive-overlap all-nonzero oblique near tangent",
    );
    let loose = unite_refusal(
        fixture_with_frame(
            AXIAL_OVERLAP_TANGENT_CONTACTS[0],
            all_nonzero,
            [false, true],
        ),
        OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap()),
        BooleanRefusal::CurvedResultTopologyUnsupported,
        "positive-overlap all-nonzero oblique near tangent",
    );
    assert_eq!(loose, baseline);
}

#[test]
fn exact_external_tangent_unite_ignores_loose_operation_tolerance() {
    for case in AXIAL_TANGENT_CONTACTS
        .into_iter()
        .chain(AXIAL_OVERLAP_TANGENT_CONTACTS)
    {
        let baseline = tangent_unite_evidence(
            case,
            Placement::Oblique,
            [true, false],
            OperationSettings::new(),
        );
        let loose = tangent_unite_evidence(
            case,
            Placement::Oblique,
            [true, false],
            OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap()),
        );
        assert_same_evidence(
            &loose,
            &baseline,
            &format!("{} loose tangent tolerance", case.name),
        );
    }
}

fn settings_at(stage: kernel::StageId, allowed: u64) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(
            stage,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap(),
    )
}

fn assert_limit(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    expected_work: u64,
) {
    let limit = *outcome
        .report()
        .limit_events()
        .first()
        .expect("disjoint-cylinder N-1 refusal recorded no limit event");
    assert_eq!(limit.stage, stage);
    assert_eq!(limit.resource, ResourceKind::Work);
    assert_eq!(limit.allowed, expected_work - 1);
    assert_eq!(limit.consumed, expected_work);
    assert_eq!(outcome.result().unwrap_err().limit(), Some(limit));
    assert_eq!(outcome.report().limit_events(), &[limit]);
}

#[test]
fn separated_and_contact_relation_work_accepts_n_and_refuses_n_minus_one_atomically() {
    for case in [RADIAL_DISJOINT, AXIAL_DISJOINT[1]]
        .into_iter()
        .chain(AXIAL_CONTACT)
        .chain([UNEQUAL_RADIUS_AXIAL_TANGENT])
        .chain(AXIAL_OVERLAP_TANGENT_CONTACTS)
    {
        assert_certified_relation(case);
        for antiparallel in [false, true] {
            let mut baseline = fixture(case, Placement::World, antiparallel);
            let before = fixture_signature(&baseline);
            let outcome = run_set_operation(
                &mut baseline,
                SetOperation::Intersect,
                false,
                OperationSettings::new(),
            );
            assert_eq!(
                usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
                Some(CYLINDER_RELATION_WORK),
                "{}",
                case.name
            );
            assert!(matches!(
                outcome.into_result().unwrap(),
                BooleanOutcome::Success(BooleanResult::ProvenEmpty)
            ));
            assert_eq!(fixture_signature(&baseline), before);

            let mut admitted = fixture(case, Placement::World, antiparallel);
            let before = fixture_signature(&admitted);
            let outcome = run_set_operation(
                &mut admitted,
                SetOperation::Intersect,
                false,
                settings_at(BOOLEAN_BSP_WORK, CYLINDER_RELATION_WORK),
            );
            assert!(matches!(
                outcome.into_result().unwrap(),
                BooleanOutcome::Success(BooleanResult::ProvenEmpty)
            ));
            assert_eq!(fixture_signature(&admitted), before);

            let mut denied = fixture(case, Placement::World, antiparallel);
            let before = fixture_signature(&denied);
            let outcome = run_set_operation(
                &mut denied,
                SetOperation::Intersect,
                false,
                settings_at(BOOLEAN_BSP_WORK, CYLINDER_RELATION_WORK - 1),
            );
            assert_limit(&outcome, BOOLEAN_BSP_WORK, CYLINDER_RELATION_WORK);
            assert_eq!(fixture_signature(&denied), before);
            assert_source_bodies_preserved(&denied, 2);
        }
    }
}

fn assert_copy_work_frontier(
    case: CylinderRelationCase,
    antiparallel: bool,
    operation: SetOperation,
    swapped: bool,
) {
    let expected_work = operation.realization_work();
    let mut baseline = fixture(case, Placement::World, antiparallel);
    let outcome = run_set_operation(&mut baseline, operation, swapped, OperationSettings::new());
    assert_eq!(
        usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(CYLINDER_RELATION_WORK),
        "{} {operation:?}",
        case.name
    );
    assert_eq!(
        usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(expected_work),
        "{} {operation:?}",
        case.name
    );
    assert_eq!(
        usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items),
        Some(0),
        "{} {operation:?}",
        case.name
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    assert_source_bodies_preserved(&baseline, 2 + operation.result_body_count());

    let mut admitted = fixture(case, Placement::World, antiparallel);
    let outcome = run_set_operation(
        &mut admitted,
        operation,
        swapped,
        settings_at(BOOLEAN_POST_SELECTION_WORK, expected_work),
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    assert_source_bodies_preserved(&admitted, 2 + operation.result_body_count());

    let mut denied = fixture(case, Placement::World, antiparallel);
    let before = fixture_signature(&denied);
    let outcome = run_set_operation(
        &mut denied,
        operation,
        swapped,
        settings_at(BOOLEAN_POST_SELECTION_WORK, expected_work - 1),
    );
    assert_limit(&outcome, BOOLEAN_POST_SELECTION_WORK, expected_work);
    assert_eq!(fixture_signature(&denied), before);
    assert_source_bodies_preserved(&denied, 2);
}

#[test]
fn whole_source_copy_work_accepts_n_and_refuses_n_minus_one_atomically() {
    for case in [RADIAL_DISJOINT, AXIAL_DISJOINT[1]] {
        for antiparallel in [false, true] {
            for operation in [SetOperation::Unite, SetOperation::Subtract] {
                for swapped in [false, true] {
                    if operation != SetOperation::Unite || !swapped {
                        assert_copy_work_frontier(case, antiparallel, operation, swapped);
                    }
                }
            }
        }
    }
    for case in AXIAL_CONTACT
        .into_iter()
        .chain([UNEQUAL_RADIUS_AXIAL_TANGENT])
        .chain(AXIAL_OVERLAP_TANGENT_CONTACTS)
    {
        for antiparallel in [false, true] {
            for swapped in [false, true] {
                assert_copy_work_frontier(case, antiparallel, SetOperation::Subtract, swapped);
            }
        }
    }
    for case in AXIAL_TANGENT_CONTACTS
        .into_iter()
        .chain(AXIAL_OVERLAP_TANGENT_CONTACTS)
    {
        for antiparallel in [false, true] {
            assert_copy_work_frontier(case, antiparallel, SetOperation::Unite, false);
        }
    }
}
