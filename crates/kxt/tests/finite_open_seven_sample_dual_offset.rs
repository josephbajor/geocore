//! Canonical finite-open seven-sample dual-offset polyline chart contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use kgraph::{IntersectionCertificateError, PairedTrace};
use ktopo::entity::Body;
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use kxt::parse::{Value, read_xt};
use kxt::schema::code;
use kxt::{
    INTERSECTION_CHART_CERTIFICATE_WORK, INTERSECTION_CHART_DEPTH, INTERSECTION_CHART_ITEMS,
    IntersectionImportBudgetProfile, XtCapability, XtError, reconstruct, reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const V11_WORK: u64 = 388_125_799;
const RECORD_3615_WORK: u64 = 26_443_776;
const RECORD_4230_WORK: u64 = 17_285_120;
const V12_WORK: u64 = 414_569_575;
const V13_WORK: u64 = 431_854_695;

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn logical(value: &Value) -> bool {
    match value {
        Value::Logical(value) => *value,
        _ => panic!("expected logical value"),
    }
}

fn doubles(value: &Value) -> Vec<f64> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_f64().unwrap()).collect(),
        _ => panic!("expected numeric array"),
    }
}

fn integers(value: &Value) -> Vec<i64> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_int().unwrap()).collect(),
        _ => panic!("expected integer array"),
    }
}

fn transplant_3615(file: &mut kxt::XtFile, destination: u32) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        set_field(file, destination, name, field(file, 3615, name).clone());
    }
}

fn context_with_plan<'a>(session: &'a SessionPolicy, plan: BudgetPlan) -> OperationContext<'a> {
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(plan)
}

fn limit(plan: &BudgetPlan, resource: ResourceKind) -> LimitSpec {
    plan.limits()
        .iter()
        .copied()
        .find(|limit| {
            limit.stage
                == match resource {
                    ResourceKind::Work => INTERSECTION_CHART_CERTIFICATE_WORK,
                    ResourceKind::Items => INTERSECTION_CHART_ITEMS,
                    ResourceKind::Depth => INTERSECTION_CHART_DEPTH,
                    ResourceKind::Bytes => unreachable!(),
                    _ => unreachable!(),
                }
                && limit.resource == resource
        })
        .unwrap()
}

fn usage(
    report: &kcore::operation::OperationReport,
    stage: kcore::operation::StageId,
    resource: ResourceKind,
) -> u64 {
    report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == resource)
        .unwrap()
        .consumed
}

fn assert_rollback(store: &Store) {
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<CurveGeom>(), 0);
    assert_eq!(store.count::<Curve2dGeom>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

#[test]
fn record_3615_pins_the_canonical_seven_sample_dual_offset_payload() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&3615].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 3615, "surface"),
        &Value::Arr(vec![Value::Ptr(3374), Value::Ptr(773)])
    );
    assert_eq!(field(&file, 3615, "chart").as_ptr(), Some(6047));
    assert_eq!(field(&file, 3615, "start").as_ptr(), Some(6053));
    assert_eq!(field(&file, 3615, "end").as_ptr(), Some(6054));
    assert_eq!(field(&file, 3615, "intersection_data").as_ptr(), Some(6052));
    for limit in [6053, 6054] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }

    for (root, basis, distance) in [(3374, 3730, -0.0015), (773, 1186, 0.00017)] {
        assert_eq!(file.nodes[&root].code, code::OFFSET_SURF);
        assert_eq!(field(&file, root, "surface").as_ptr(), Some(basis));
        assert_eq!(field(&file, root, "offset").as_f64(), Some(distance));
        assert_eq!(field(&file, root, "sense").as_char(), Some('+'));
    }

    assert_eq!(file.nodes[&3730].code, code::B_SURFACE);
    assert_eq!(field(&file, 3730, "nurbs").as_ptr(), Some(3739));
    for flag in ["u_periodic", "v_periodic", "u_closed", "v_closed"] {
        assert!(!logical(field(&file, 3739, flag)));
    }
    assert_eq!(field(&file, 3739, "u_degree").as_int(), Some(2));
    assert_eq!(field(&file, 3739, "v_degree").as_int(), Some(3));
    assert_eq!(field(&file, 3739, "n_u_vertices").as_int(), Some(5));
    assert_eq!(field(&file, 3739, "n_v_vertices").as_int(), Some(10));
    assert!(logical(field(&file, 3739, "rational")));
    let u_knots = field(&file, 3739, "u_knots").as_ptr().unwrap();
    let u_mult = field(&file, 3739, "u_knot_mult").as_ptr().unwrap();
    let v_knots = field(&file, 3739, "v_knots").as_ptr().unwrap();
    let v_mult = field(&file, 3739, "v_knot_mult").as_ptr().unwrap();
    assert_eq!(
        doubles(field(&file, u_knots, "knots")),
        vec![0.653420101246629, 1.0, 1.003465798987534]
    );
    assert_eq!(integers(field(&file, u_mult, "mult")), vec![3, 2, 3]);
    assert_eq!(
        doubles(field(&file, v_knots, "knots")),
        vec![-0.000404, 0.0, 0.04, 0.0404]
    );
    assert_eq!(integers(field(&file, v_mult, "mult")), vec![4, 3, 3, 4]);

    assert_eq!(field(&file, 1186, "nurbs").as_ptr(), Some(1208));
    assert!(logical(field(&file, 1208, "u_periodic")));
    assert!(logical(field(&file, 1208, "u_closed")));
    assert!(!logical(field(&file, 1208, "v_periodic")));
    assert!(!logical(field(&file, 1208, "v_closed")));
    assert_eq!(field(&file, 1208, "u_degree").as_int(), Some(3));
    assert_eq!(field(&file, 1208, "v_degree").as_int(), Some(3));
    assert_eq!(field(&file, 1208, "n_u_vertices").as_int(), Some(90));
    assert_eq!(field(&file, 1208, "n_v_vertices").as_int(), Some(11));
    assert!(!logical(field(&file, 1208, "rational")));

    assert_eq!(field(&file, 6047, "chart_count").as_int(), Some(7));
    assert_eq!(field(&file, 6047, "base_parameter").as_f64(), Some(0.0));
    assert_eq!(field(&file, 6047, "base_scale").as_f64(), Some(1.0));
    assert_eq!(
        field(&file, 6047, "chordal_error").as_f64(),
        Some(3.13071789187402e-5)
    );
    assert_eq!(
        field(&file, 6047, "angular_error").as_f64(),
        Some(0.1106090362239264)
    );
    assert_eq!(
        field(&file, 6047, "parameter_error"),
        &Value::Arr(vec![Value::Null, Value::Null])
    );
    assert_eq!(
        field(&file, 6047, "hvec"),
        &Value::Arr(
            [
                [
                    -0.01864864757179245,
                    0.0751796598838287,
                    0.00506250000000052
                ],
                [
                    -0.01864993684225575,
                    0.0751810903622932,
                    0.00506210836445814
                ],
                [
                    -0.01865509603938945,
                    0.0751868162880609,
                    0.00506056350310559
                ],
                [-0.01867576254349, 0.0752097801949779, 0.00505471020688174],
                [-0.0187587154744839, 0.0753024197893754, 0.00503557516153385],
                [-0.0190900580450541, 0.0756807270106007, 0.00500141360102368],
                [-0.01918300029771025, 0.0757893909053871, 0.005],
            ]
            .into_iter()
            .map(|point| Value::Vector(Some(point)))
            .collect()
        )
    );
    assert_eq!(field(&file, 6052, "uv_type").as_int(), Some(4));
    assert_eq!(
        doubles(field(&file, 6052, "values")),
        vec![
            0.790367757566836,
            0.00135135242820755,
            0.343522253164026,
            0.661753669881605,
            0.791260975481798,
            0.00135006315774426,
            0.3435071403509485,
            0.661747587312705,
            0.794774107975496,
            0.001344903960610572,
            0.343446654995263,
            0.661723439205038,
            0.807989535887904,
            0.001324237456509994,
            0.3432042126647255,
            0.66162958734064,
            0.851989336572122,
            0.001241284525516094,
            0.3422283353084525,
            0.661290117167757,
            0.972843788987022,
            0.000909941954945909,
            0.3382766828284965,
            0.660287649947587,
            1.000000000271838,
            0.000816999702289749,
            0.337150899960885,
            0.66007291894901,
        ]
    );
}

#[test]
fn record_3615_transplant_has_exact_isolated_work_items_depth_and_n_minus_one_boundaries() {
    let session = SessionPolicy::v1();
    for (resource, mode, exact) in [
        (
            ResourceKind::Work,
            AccountingMode::Cumulative,
            RECORD_3615_WORK,
        ),
        (ResourceKind::Items, AccountingMode::HighWater, 7),
        (ResourceKind::Depth, AccountingMode::HighWater, 10),
    ] {
        for allowed in [exact - 1, exact] {
            let mut file = read_xt(EXEMPLAR).unwrap();
            transplant_3615(&mut file, 1828);
            let stage = match resource {
                ResourceKind::Work => INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Items => INTERSECTION_CHART_ITEMS,
                ResourceKind::Depth => INTERSECTION_CHART_DEPTH,
                ResourceKind::Bytes => unreachable!(),
                _ => unreachable!(),
            };
            let mut limits = vec![LimitSpec::new(stage, resource, mode, allowed)];
            if resource != ResourceKind::Work {
                limits.push(LimitSpec::new(
                    INTERSECTION_CHART_CERTIFICATE_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    RECORD_3615_WORK,
                ));
            }
            let plan = BudgetPlan::new(limits).unwrap();
            let mut store = Store::new();
            let outcome =
                reconstruct_with_context(&file, &mut store, &context_with_plan(&session, plan))
                    .unwrap();
            let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
            if allowed + 1 == exact {
                assert_eq!(crossing.consumed, exact);
                assert_eq!(usage(outcome.report(), stage, resource), 0);
            } else {
                assert_eq!(usage(outcome.report(), stage, resource), exact,);
            }
            assert_rollback(&store);
        }
    }
    assert_eq!(V11_WORK + RECORD_3615_WORK, V12_WORK);
}

#[test]
fn malformed_record_3615_witnesses_and_limits_fail_typed_and_atomically() {
    let mut duplicate_position = read_xt(EXEMPLAR).unwrap();
    transplant_3615(&mut duplicate_position, 1828);
    let mut positions = match field(&duplicate_position, 6047, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    positions[3] = positions[2].clone();
    set_field(&mut duplicate_position, 6047, "hvec", Value::Arr(positions));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&duplicate_position, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset seven-sample carrier controls must be pairwise distinct",
            },
        })
    ));
    assert_rollback(&store);

    let mut duplicate_uv = read_xt(EXEMPLAR).unwrap();
    transplant_3615(&mut duplicate_uv, 1828);
    let mut values = match field(&duplicate_uv, 6052, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[12] = values[8].clone();
    values[13] = values[9].clone();
    set_field(&mut duplicate_uv, 6052, "values", Value::Arr(values));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&duplicate_uv, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::First,
                reason: "dual-offset seven-sample pcurve controls must be pairwise distinct",
            },
        })
    ));
    assert_rollback(&store);

    let mut displaced = read_xt(EXEMPLAR).unwrap();
    transplant_3615(&mut displaced, 1828);
    let mut positions = match field(&displaced, 6047, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let mut middle = positions[3].as_vector().unwrap();
    middle[2] += 1.0e-3;
    positions[3] = Value::Vector(Some(middle));
    set_field(&mut displaced, 6047, "hvec", Value::Arr(positions));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&displaced, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::ResidualExceedsTolerance { .. },
        })
    ));
    assert_rollback(&store);

    let mut equal_limit = read_xt(EXEMPLAR).unwrap();
    transplant_3615(&mut equal_limit, 1828);
    set_field(&mut equal_limit, 1828, "end", Value::Ptr(6053));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&equal_limit, &mut store),
        Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "only one shared closed LIMIT type H with term_use ? is supported for an equal-limit chart",
        })
    ));
    assert_rollback(&store);
}

#[test]
fn v1_through_v12_are_stable_and_v13_has_the_exact_aggregate_profile() {
    let expected = [
        131_072,
        81_267_732,
        115_485_725,
        116_396_069,
        117_478_445,
        208_228_426,
        272_430_166,
        315_245_660,
        323_814_492,
        336_759_900,
        V11_WORK,
        V12_WORK,
        V13_WORK,
    ];
    let plans = [
        IntersectionImportBudgetProfile::v1_defaults(),
        IntersectionImportBudgetProfile::v2_defaults(),
        IntersectionImportBudgetProfile::v3_defaults(),
        IntersectionImportBudgetProfile::v4_defaults(),
        IntersectionImportBudgetProfile::v5_defaults(),
        IntersectionImportBudgetProfile::v6_defaults(),
        IntersectionImportBudgetProfile::v7_defaults(),
        IntersectionImportBudgetProfile::v8_defaults(),
        IntersectionImportBudgetProfile::v9_defaults(),
        IntersectionImportBudgetProfile::v10_defaults(),
        IntersectionImportBudgetProfile::v11_defaults(),
        IntersectionImportBudgetProfile::v12_defaults(),
        IntersectionImportBudgetProfile::v13_defaults(),
    ];
    for (plan, expected_work) in plans.iter().zip(expected) {
        assert_eq!(limit(plan, ResourceKind::Work).allowed, expected_work);
        assert_eq!(limit(plan, ResourceKind::Items).allowed, 65_536);
        assert_eq!(limit(plan, ResourceKind::Depth).allowed, 10);
    }
}

#[test]
fn v12_certifies_3615_and_stops_at_the_five_sample_proof_boundary() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&4230].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 4230, "surface"),
        &Value::Arr(vec![Value::Ptr(3320), Value::Ptr(773)])
    );
    assert_eq!(file.nodes[&3320].code, code::OFFSET_SURF);
    assert_eq!(file.nodes[&773].code, code::OFFSET_SURF);
    assert_eq!(field(&file, 4230, "chart").as_ptr(), Some(4231));
    assert_eq!(field(&file, 4231, "chart_count").as_int(), Some(5));
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v12_defaults()),
    )
    .unwrap();
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, INTERSECTION_CHART_CERTIFICATE_WORK);
    assert_eq!(crossing.resource, ResourceKind::Work);
    assert_eq!(crossing.allowed, V12_WORK);
    assert_eq!(crossing.consumed, V12_WORK + RECORD_4230_WORK);
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V12_WORK
    );
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
        ),
        22
    );
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_DEPTH,
            ResourceKind::Depth,
        ),
        10
    );
    assert_rollback(&store);
}
