//! Facade-only lifecycle coverage for bounded-skew lobe properties.
//!
//! R7 wall-time budget: the complete world/oblique replay, including Boolean
//! construction and property budget crossings, should remain below 10 seconds
//! in an unoptimized local test run.

use kernel::{
    AccountingMode, BODY_PROPERTIES_ANALYTIC_WORK, BodyId, BodyProperties, BodyPropertiesOutcome,
    BodyPropertiesRequest, BooleanBodiesRequest, BooleanOperation, BooleanOutcome, BooleanResult,
    BudgetPlan, CylinderRequest, ErrorClass, Frame, Kernel, LimitSpec, OperationSettings, PartId,
    Point3, ResourceKind, Session, Vec3,
};

const BOUNDED_LOWER: f64 = 1.8;
const BOUNDED_UPPER: f64 = 1.9;
const TRANSVERSE_HALF_HEIGHT: f64 = 1.25;
const TRANSVERSE_RADIUS: f64 = 2.0;

/// Deterministic inverse cosine over the kernel-owned `atan2`.
fn deterministic_acos(value: f64) -> f64 {
    kcore::math::atan2((1.0 - value * value).sqrt(), value)
}

/// Deterministic inverse sine over the kernel-owned `atan2`.
fn deterministic_asin(value: f64) -> f64 {
    kcore::math::atan2(value, (1.0 - value * value).sqrt())
}

struct Fixture {
    session: Session,
    part: PartId,
    bounded: BodyId,
    transverse: BodyId,
}

fn fixture(frame: Frame) -> Fixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (bounded, transverse) = {
        let mut edit = session.edit_part(part.clone()).unwrap();
        let bounded = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, BOUNDED_LOWER)),
                1.0,
                BOUNDED_UPPER - BOUNDED_LOWER,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let transverse_frame = Frame::new(
            frame.point_at(-TRANSVERSE_HALF_HEIGHT, 0.0, 0.0),
            frame.x(),
            frame.y(),
        )
        .unwrap();
        let transverse = edit
            .create_cylinder(CylinderRequest::new(
                transverse_frame,
                TRANSVERSE_RADIUS,
                2.0 * TRANSVERSE_HALF_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (bounded, transverse)
    };
    Fixture {
        session,
        part,
        bounded,
        transverse,
    }
}

fn subtract(fixture: &mut Fixture) -> Vec<BodyId> {
    let outcome = fixture
        .session
        .edit_part(fixture.part.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Subtract,
            fixture.bounded.clone(),
            fixture.transverse.clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
        panic!("expected two public bounded-skew lobe bodies");
    };
    assert_eq!(created.bodies().len(), 2);
    created.bodies().to_vec()
}

fn certified(
    fixture: &Fixture,
    body: BodyId,
    settings: OperationSettings,
) -> (BodyProperties, u64) {
    let outcome = fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .body_properties(BodyPropertiesRequest::new(body).with_settings(settings))
        .unwrap();
    let consumed = outcome
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BODY_PROPERTIES_ANALYTIC_WORK && usage.resource == ResourceKind::Work
        })
        .expect("bounded-skew properties did not charge analytic work")
        .consumed;
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = outcome.into_result().unwrap()
    else {
        panic!("Full-valid bounded-skew lobe properties were refused");
    };
    assert_eq!(full_check.outcome(), kernel::CheckOutcome::Valid);
    (properties, consumed)
}

#[derive(Debug, Clone, Copy)]
struct LobeOracle {
    volume: f64,
    centroid_y: f64,
    centroid_z: f64,
    positive_inertia: [[f64; 3]; 3],
}

fn oracle() -> LobeOracle {
    const STEPS: usize = 32_768;
    let integrate = |term: fn(f64) -> f64| {
        let width = (BOUNDED_UPPER - BOUNDED_LOWER) / STEPS as f64;
        let mut sum = term(BOUNDED_LOWER) + term(BOUNDED_UPPER);
        for index in 1..STEPS {
            let z = BOUNDED_LOWER + index as f64 * width;
            sum += if index % 2 == 0 { 2.0 } else { 4.0 } * term(z);
        }
        sum * width / 3.0
    };
    fn section_area(z: f64) -> f64 {
        let threshold = (TRANSVERSE_RADIUS * TRANSVERSE_RADIUS - z * z).sqrt();
        deterministic_acos(threshold) - threshold * (1.0 - threshold * threshold).sqrt()
    }
    fn section_y_moment(z: f64) -> f64 {
        let threshold2 = TRANSVERSE_RADIUS * TRANSVERSE_RADIUS - z * z;
        (2.0 / 3.0) * (1.0 - threshold2) * (1.0 - threshold2).sqrt()
    }
    fn section_z_moment(z: f64) -> f64 {
        z * section_area(z)
    }
    fn section_xx(z: f64) -> f64 {
        let threshold = (TRANSVERSE_RADIUS * TRANSVERSE_RADIUS - z * z).sqrt();
        let theta = deterministic_asin(threshold);
        core::f64::consts::PI / 8.0
            - theta / 4.0
            - kcore::math::sin(2.0 * theta) / 6.0
            - kcore::math::sin(4.0 * theta) / 48.0
    }
    fn section_yy(z: f64) -> f64 {
        let threshold = (TRANSVERSE_RADIUS * TRANSVERSE_RADIUS - z * z).sqrt();
        let theta = deterministic_asin(threshold);
        core::f64::consts::PI / 8.0 - theta / 4.0 + kcore::math::sin(4.0 * theta) / 16.0
    }
    fn section_zz(z: f64) -> f64 {
        z * z * section_area(z)
    }
    fn section_yz(z: f64) -> f64 {
        z * section_y_moment(z)
    }
    let volume = integrate(section_area);
    let centroid_y = integrate(section_y_moment) / volume;
    let centroid_z = integrate(section_z_moment) / volume;
    let covariance_xx = integrate(section_xx);
    let covariance_yy = integrate(section_yy) - volume * centroid_y * centroid_y;
    let covariance_zz = integrate(section_zz) - volume * centroid_z * centroid_z;
    let covariance_yz = integrate(section_yz) - volume * centroid_y * centroid_z;
    LobeOracle {
        volume,
        centroid_y,
        centroid_z,
        positive_inertia: [
            [covariance_yy + covariance_zz, 0.0, 0.0],
            [0.0, covariance_xx + covariance_zz, -covariance_yz],
            [0.0, -covariance_yz, covariance_xx + covariance_yy],
        ],
    }
}

fn expected_inertia(frame: Frame, oracle: LobeOracle, sign: f64) -> [[f64; 3]; 3] {
    let mut local = oracle.positive_inertia;
    local[1][2] *= sign;
    local[2][1] *= sign;
    let axes = [frame.x(), frame.y(), frame.z()];
    let component = |axis: Vec3, index: usize| match index {
        0 => axis.x,
        1 => axis.y,
        _ => axis.z,
    };
    core::array::from_fn(|row| {
        core::array::from_fn(|column| {
            let mut value = 0.0;
            for left in 0..3 {
                for right in 0..3 {
                    value += component(axes[left], row)
                        * local[left][right]
                        * component(axes[right], column);
                }
            }
            value
        })
    })
}

fn exact_work_settings(allowed: u64) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(
            BODY_PROPERTIES_ANALYTIC_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap(),
    )
}

#[test]
fn public_bounded_skew_lobes_have_certified_rigid_invariant_properties() {
    let frames = [
        Frame::world(),
        Frame::new(
            Point3::new(2.5, -1.75, 0.625),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    ];
    let oracle = oracle();
    assert!(oracle.volume.is_finite() && oracle.volume > 0.0);
    let mut orientation_order = None;

    for (frame_index, frame) in frames.into_iter().enumerate() {
        let mut fixture = fixture(frame);
        let bodies = subtract(&mut fixture);
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        assert!(part.body(fixture.bounded.clone()).is_ok());
        assert!(part.body(fixture.transverse.clone()).is_ok());
        let body_count = part.bodies().len();
        drop(part);

        let mut properties = Vec::new();
        for body in &bodies {
            let request = BodyPropertiesRequest::new(body.clone());
            let first = fixture
                .session
                .part(fixture.part.clone())
                .unwrap()
                .body_properties(request.clone())
                .unwrap();
            let replay = fixture
                .session
                .part(fixture.part.clone())
                .unwrap()
                .body_properties(request)
                .unwrap();
            assert_eq!(replay, first, "property replay was not deterministic");
            let consumed = first
                .report()
                .usage()
                .iter()
                .find(|usage| {
                    usage.stage == BODY_PROPERTIES_ANALYTIC_WORK
                        && usage.resource == ResourceKind::Work
                })
                .unwrap()
                .consumed;
            let (value, full_check) = match first.into_result().unwrap() {
                BodyPropertiesOutcome::Certified {
                    properties,
                    full_check,
                } => (properties, full_check),
                BodyPropertiesOutcome::Refused { reason, .. } => {
                    panic!("bounded-skew property theorem refused: {reason:?}")
                }
            };
            assert_eq!(full_check.outcome(), kernel::CheckOutcome::Valid);
            assert!(
                value.volume().lower().is_finite()
                    && value.volume().upper().is_finite()
                    && value.volume().lower() > 0.0
                    && value.volume().contains(oracle.volume)
            );
            assert!(value.surface_area().lower() > 0.0);
            assert!(consumed > 0, "stage must meter work");
            properties.push((body.clone(), value, consumed));
        }

        let summed_lower = properties
            .iter()
            .map(|(_, value, _)| value.volume().lower())
            .sum::<f64>();
        let summed_upper = properties
            .iter()
            .map(|(_, value, _)| value.volume().upper())
            .sum::<f64>();
        assert!(summed_lower <= 2.0 * oracle.volume);
        assert!(summed_upper >= 2.0 * oracle.volume);

        let signs = if frame_index == 0 {
            let signs = properties
                .iter()
                .map(|(_, value, _)| {
                    let y = value.centroid().coordinates()[1];
                    if y.lower() > 0.0 {
                        1.0
                    } else if y.upper() < 0.0 {
                        -1.0
                    } else {
                        panic!("world-frame lobe centroid did not certify its orientation");
                    }
                })
                .collect::<Vec<_>>();
            let mut sorted = signs.clone();
            sorted.sort_by(f64::total_cmp);
            assert_eq!(sorted, vec![-1.0, 1.0]);
            orientation_order = Some(signs.clone());
            signs
        } else {
            orientation_order
                .clone()
                .expect("world-frame orientation order was not recorded")
        };
        for ((_, value, _), sign) in properties.iter().zip(&signs) {
            let expected = frame.point_at(0.0, *sign * oracle.centroid_y, oracle.centroid_z);
            assert!(
                value.centroid().contains(expected),
                "rigidly placed centroid oracle escaped its enclosure"
            );
            let inertia = expected_inertia(frame, oracle, *sign);
            assert!(
                value.centroidal_inertia().contains(inertia),
                "independent centroidal-inertia oracle escaped its enclosure"
            );
        }

        for (body, baseline, consumed) in properties {
            let (exact, exact_consumed) =
                certified(&fixture, body.clone(), exact_work_settings(consumed));
            assert_eq!(exact_consumed, consumed);
            assert_eq!(exact, baseline);
            let denied = fixture
                .session
                .part(fixture.part.clone())
                .unwrap()
                .body_properties(
                    BodyPropertiesRequest::new(body.clone())
                        .with_settings(exact_work_settings(consumed - 1)),
                )
                .unwrap();
            let error = denied.into_result().unwrap_err();
            assert_eq!(error.class(), ErrorClass::ResourceLimit);
            let limit = error.limit().unwrap();
            assert_eq!(limit.stage, BODY_PROPERTIES_ANALYTIC_WORK);
            assert_eq!(limit.consumed, consumed);
            assert_eq!(limit.allowed, consumed - 1);

            let part = fixture.session.part(fixture.part.clone()).unwrap();
            assert_eq!(part.bodies().len(), body_count);
            assert_eq!(part.body(body).unwrap().faces().unwrap().len(), 4);
            assert!(part.body(fixture.bounded.clone()).is_ok());
            assert!(part.body(fixture.transverse.clone()).is_ok());
        }
    }
}
