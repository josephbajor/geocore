//! Canonical finite-open five-sample dual-offset polyline chart contract.

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
const RECORD_4230_WORK: u64 = 17_285_120;
const RECORD_3609_WORK: u64 = 4_277_250;
const RECORD_6044_WORK: u64 = 4_352_000;
const RECORD_5921_WORK: u64 = 13_774_848;
const V13_WORK: u64 = 431_854_695;
const V14_WORK: u64 = 436_131_945;
const V15_WORK: u64 = 440_483_945;

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn transplant_4230(file: &mut kxt::XtFile) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        set_field(file, 1828, name, field(file, 4230, name).clone());
    }
}

fn transplant_6044(file: &mut kxt::XtFile) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        set_field(file, 1828, name, field(file, 6044, name).clone());
    }
}

fn context_with_plan<'a>(session: &'a SessionPolicy, plan: BudgetPlan) -> OperationContext<'a> {
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(plan)
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
        .map_or(0, |usage| usage.consumed)
}

fn assert_rollback(store: &Store) {
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<CurveGeom>(), 0);
    assert_eq!(store.count::<Curve2dGeom>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

#[test]
fn record_4230_pins_the_canonical_five_sample_dual_offset_payload() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&4230].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 4230, "surface"),
        &Value::Arr(vec![Value::Ptr(3320), Value::Ptr(773)])
    );
    assert_eq!(field(&file, 4230, "chart").as_ptr(), Some(4231));
    assert_eq!(field(&file, 4230, "start").as_ptr(), Some(4240));
    assert_eq!(field(&file, 4230, "end").as_ptr(), Some(4236));
    assert_eq!(field(&file, 4230, "intersection_data").as_ptr(), Some(4238));
    for limit in [4240, 4236] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }
    for (root, basis, nurbs, distance) in [(3320, 3612, 5961, -0.0015), (773, 1186, 1208, 0.00017)]
    {
        assert_eq!(file.nodes[&root].code, code::OFFSET_SURF);
        assert_eq!(field(&file, root, "surface").as_ptr(), Some(basis));
        assert_eq!(field(&file, root, "offset").as_f64(), Some(distance));
        assert_eq!(field(&file, root, "sense").as_char(), Some('+'));
        assert_eq!(field(&file, basis, "nurbs").as_ptr(), Some(nurbs));
    }

    assert_eq!(field(&file, 4231, "chart_count").as_int(), Some(5));
    assert_eq!(field(&file, 4231, "base_parameter").as_f64(), Some(0.0));
    assert_eq!(field(&file, 4231, "base_scale").as_f64(), Some(1.0));
    assert_eq!(
        field(&file, 4231, "chordal_error").as_f64(),
        Some(7.92075255925224e-5)
    );
    assert_eq!(
        field(&file, 4231, "angular_error").as_f64(),
        Some(0.291994958180986)
    );
    assert_eq!(
        field(&file, 4231, "parameter_error"),
        &Value::Arr(vec![Value::Null, Value::Null])
    );
    assert_eq!(
        field(&file, 4231, "hvec"),
        &Value::Arr(
            [
                [-0.0187839579817981, 0.0753157267297373, 0.00566013197726139],
                [-0.01878139424975365, 0.0753130485733002, 0.0056521849632974],
                [-0.0187712593641828, 0.0753024799638379, 0.00562030991159029],
                [
                    -0.01873285735607575,
                    0.0752627407145455,
                    0.00549130871552844
                ],
                [
                    -0.01864849581213355,
                    0.0751795417003913,
                    0.00506042753035674
                ],
            ]
            .into_iter()
            .map(|point| Value::Vector(Some(point)))
            .collect()
        )
    );
    assert_eq!(field(&file, 4238, "uv_type").as_int(), Some(4));
    assert_eq!(
        field(&file, 4238, "values"),
        &Value::Arr(
            [
                0.0,
                0.00121604201820188,
                0.342106087201043,
                0.666765091638957,
                0.002024448811111865,
                0.001218605750246344,
                0.342134016270183,
                0.666699999187497,
                0.01024441167532546,
                0.001228740635817184,
                0.342244236983841,
                0.666438616619765,
                0.0453862230888163,
                0.001267142643924256,
                0.342658719494154,
                0.66537548547436,
                0.210892715598746,
                0.00135150418786642,
                0.343523441415311,
                0.661735647034375,
            ]
            .into_iter()
            .map(Value::Double)
            .collect()
        )
    );
}

#[test]
fn record_4230_transplant_has_exact_isolated_work_items_depth_and_n_minus_one_boundaries() {
    let session = SessionPolicy::v1();
    for (resource, mode, exact) in [
        (
            ResourceKind::Work,
            AccountingMode::Cumulative,
            RECORD_4230_WORK,
        ),
        (ResourceKind::Items, AccountingMode::HighWater, 5),
        (ResourceKind::Depth, AccountingMode::HighWater, 10),
    ] {
        for allowed in [exact - 1, exact] {
            let mut file = read_xt(EXEMPLAR).unwrap();
            transplant_4230(&mut file);
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
                    RECORD_4230_WORK,
                ));
            }
            let mut store = Store::new();
            let outcome = reconstruct_with_context(
                &file,
                &mut store,
                &context_with_plan(&session, BudgetPlan::new(limits).unwrap()),
            )
            .unwrap();
            let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
            if allowed + 1 == exact {
                assert_eq!(crossing.stage, stage);
                assert_eq!(crossing.resource, resource);
                assert_eq!(crossing.allowed, allowed);
                assert_eq!(crossing.consumed, exact);
                assert_eq!(usage(outcome.report(), stage, resource), 0);
            } else {
                assert_eq!(usage(outcome.report(), stage, resource), exact);
            }
            assert_rollback(&store);
        }
    }
}

#[test]
fn malformed_record_4230_controls_limits_and_residuals_fail_typed_and_atomically() {
    let mut duplicate_position = read_xt(EXEMPLAR).unwrap();
    transplant_4230(&mut duplicate_position);
    let mut positions = match field(&duplicate_position, 4231, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    positions[2] = positions[1].clone();
    set_field(&mut duplicate_position, 4231, "hvec", Value::Arr(positions));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&duplicate_position, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset five-sample carrier controls must be pairwise distinct",
            },
        })
    ));
    assert_rollback(&store);

    let mut duplicate_uv = read_xt(EXEMPLAR).unwrap();
    transplant_4230(&mut duplicate_uv);
    let mut values = match field(&duplicate_uv, 4238, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[8] = values[4].clone();
    values[9] = values[5].clone();
    set_field(&mut duplicate_uv, 4238, "values", Value::Arr(values));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&duplicate_uv, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::First,
                reason: "dual-offset five-sample pcurve controls must be pairwise distinct",
            },
        })
    ));
    assert_rollback(&store);

    let mut displaced = read_xt(EXEMPLAR).unwrap();
    transplant_4230(&mut displaced);
    let mut positions = match field(&displaced, 4231, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let mut middle = positions[2].as_vector().unwrap();
    middle[2] += 1.0e-3;
    positions[2] = Value::Vector(Some(middle));
    set_field(&mut displaced, 4231, "hvec", Value::Arr(positions));
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
    transplant_4230(&mut equal_limit);
    set_field(&mut equal_limit, 1828, "end", Value::Ptr(4240));
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
fn record_6044_transplant_has_exact_isolated_work_items_depth_and_n_minus_one_boundaries() {
    let session = SessionPolicy::v1();
    for (resource, mode, exact) in [
        (
            ResourceKind::Work,
            AccountingMode::Cumulative,
            RECORD_6044_WORK,
        ),
        (ResourceKind::Items, AccountingMode::HighWater, 2),
        (ResourceKind::Depth, AccountingMode::HighWater, 10),
    ] {
        for allowed in [exact - 1, exact] {
            let mut file = read_xt(EXEMPLAR).unwrap();
            transplant_6044(&mut file);
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
                    RECORD_6044_WORK,
                ));
            }
            let mut store = Store::new();
            let outcome = reconstruct_with_context(
                &file,
                &mut store,
                &context_with_plan(&session, BudgetPlan::new(limits).unwrap()),
            )
            .unwrap();
            if allowed + 1 == exact {
                let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
                assert_eq!(crossing.stage, stage);
                assert_eq!(crossing.resource, resource);
                assert_eq!(crossing.allowed, allowed);
                assert_eq!(crossing.consumed, exact);
                assert_eq!(usage(outcome.report(), stage, resource), 0);
            } else {
                assert_eq!(usage(outcome.report(), stage, resource), exact);
            }
            assert_rollback(&store);
        }
    }
}

#[test]
fn v13_certifies_4230_and_pins_the_next_plane_offset_frontier() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&3609].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 3609, "surface"),
        &Value::Arr(vec![Value::Ptr(3321), Value::Ptr(773)])
    );
    assert_eq!(file.nodes[&3321].code, code::PLANE);
    assert_eq!(file.nodes[&773].code, code::OFFSET_SURF);
    assert_eq!(field(&file, 3609, "chart").as_ptr(), Some(3607));
    assert_eq!(field(&file, 3607, "chart_count").as_int(), Some(2));
    assert_eq!(field(&file, 3609, "start").as_ptr(), Some(3608));
    assert_eq!(field(&file, 3609, "end").as_ptr(), Some(3606));
    assert_eq!(field(&file, 3609, "intersection_data").as_ptr(), Some(3613));
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let context = context_with_plan(&session, IntersectionImportBudgetProfile::v13_defaults());
    let outcome = reconstruct_with_context(&file, &mut store, &context).unwrap();
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, INTERSECTION_CHART_CERTIFICATE_WORK);
    assert_eq!(crossing.resource, ResourceKind::Work);
    assert_eq!(crossing.allowed, V13_WORK);
    assert_eq!(crossing.consumed, V13_WORK + RECORD_3609_WORK);
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V13_WORK
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

#[test]
fn v14_certifies_3609_and_pins_the_next_two_sample_dual_offset_frontier() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&6044].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 6044, "surface"),
        &Value::Arr(vec![Value::Ptr(3312), Value::Ptr(773)])
    );
    assert_eq!(file.nodes[&3312].code, code::OFFSET_SURF);
    assert_eq!(file.nodes[&773].code, code::OFFSET_SURF);
    assert_eq!(field(&file, 6044, "chart").as_ptr(), Some(6043));
    assert_eq!(field(&file, 6043, "chart_count").as_int(), Some(2));
    assert_eq!(field(&file, 6044, "start").as_ptr(), Some(6049));
    assert_eq!(field(&file, 6044, "end").as_ptr(), Some(6046));
    assert_eq!(field(&file, 6044, "intersection_data").as_ptr(), Some(6050));
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let context = context_with_plan(&session, IntersectionImportBudgetProfile::v14_defaults());
    let outcome = reconstruct_with_context(&file, &mut store, &context).unwrap();
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, INTERSECTION_CHART_CERTIFICATE_WORK);
    assert_eq!(crossing.resource, ResourceKind::Work);
    assert_eq!(crossing.allowed, V14_WORK);
    assert_eq!(crossing.consumed, V14_WORK + RECORD_6044_WORK);
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V14_WORK
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

#[test]
fn v15_certifies_6044_and_pins_the_next_four_sample_dual_offset_frontier() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&5921].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 5921, "surface"),
        &Value::Arr(vec![Value::Ptr(3300), Value::Ptr(773)])
    );
    for (root, basis, nurbs, distance) in [(3300, 2850, 4137, -0.0015), (773, 1186, 1208, 0.00017)]
    {
        assert_eq!(file.nodes[&root].code, code::OFFSET_SURF);
        assert_eq!(field(&file, root, "surface").as_ptr(), Some(basis));
        assert_eq!(field(&file, root, "offset").as_f64(), Some(distance));
        assert_eq!(field(&file, root, "sense").as_char(), Some('+'));
        assert_eq!(file.nodes[&basis].code, code::B_SURFACE);
        assert_eq!(field(&file, basis, "nurbs").as_ptr(), Some(nurbs));
    }
    assert_eq!(field(&file, 5921, "chart").as_ptr(), Some(6027));
    assert_eq!(field(&file, 6027, "base_parameter").as_f64(), Some(0.0));
    assert_eq!(field(&file, 6027, "base_scale").as_f64(), Some(1.0));
    assert_eq!(field(&file, 6027, "chart_count").as_int(), Some(4));
    assert_eq!(field(&file, 5921, "start").as_ptr(), Some(6029));
    assert_eq!(field(&file, 5921, "end").as_ptr(), Some(6031));
    for limit in [6029, 6031] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }
    assert_eq!(field(&file, 5921, "intersection_data").as_ptr(), Some(6035));
    assert_eq!(field(&file, 6035, "uv_type").as_int(), Some(4));
    assert_eq!(
        match field(&file, 6035, "values") {
            Value::Arr(values) => values.len(),
            _ => 0,
        },
        16
    );

    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let outcome = reconstruct_with_context(&file, &mut store, &context).unwrap();
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(crossing.stage, INTERSECTION_CHART_CERTIFICATE_WORK);
    assert_eq!(crossing.resource, ResourceKind::Work);
    assert_eq!(crossing.allowed, V15_WORK);
    assert_eq!(crossing.consumed, V15_WORK + RECORD_5921_WORK);
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V15_WORK
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
