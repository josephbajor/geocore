//! Production finite-open NURBS endpoint-roundoff normalization contract.

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
    IntersectionImportBudgetProfile, XtError, reconstruct, reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const V7_WORK: u64 = 272_430_166;
const V7_ATTEMPTED_WORK: u64 = 285_283_414;
const V8_WORK: u64 = 315_245_660;
const V8_ATTEMPTED_WORK: u64 = 323_814_492;

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn context_with_plan<'a>(session: &'a SessionPolicy, plan: BudgetPlan) -> OperationContext<'a> {
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(plan)
}

fn context_with_limit<'a>(
    session: &'a SessionPolicy,
    stage: kcore::operation::StageId,
    resource: ResourceKind,
    mode: AccountingMode,
    allowed: u64,
) -> OperationContext<'a> {
    context_with_plan(
        session,
        BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap(),
    )
}

fn limit(plan: &BudgetPlan, stage: kcore::operation::StageId, resource: ResourceKind) -> LimitSpec {
    plan.limits()
        .iter()
        .copied()
        .find(|limit| limit.stage == stage && limit.resource == resource)
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

fn transplant_1984(file: &mut kxt::XtFile, destination: u32) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, 1984, name).clone();
        set_field(file, destination, name, value);
    }
}

#[test]
fn record_1984_pins_one_nonperiodic_endpoint_roundoff_and_source_domains() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&1984].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 1984, "surface"),
        &Value::Arr(vec![Value::Ptr(1939), Value::Ptr(773)])
    );
    assert_eq!(file.nodes[&1939].code, code::B_SURFACE);
    assert_eq!(field(&file, 1939, "nurbs").as_ptr(), Some(1953));
    for flag in ["u_periodic", "v_periodic", "u_closed", "v_closed"] {
        assert_eq!(field(&file, 1953, flag), &Value::Logical(false));
    }
    assert_eq!(field(&file, 1953, "u_degree").as_int(), Some(2));
    assert_eq!(field(&file, 1953, "v_degree").as_int(), Some(3));
    assert_eq!(field(&file, 1953, "n_u_vertices").as_int(), Some(3));
    assert_eq!(field(&file, 1953, "n_v_vertices").as_int(), Some(4));
    let u_knots = field(&file, 1953, "u_knots").as_ptr().unwrap();
    let v_knots = field(&file, 1953, "v_knots").as_ptr().unwrap();
    assert_eq!(
        field(&file, u_knots, "knots"),
        &Value::Arr(vec![Value::Double(0.0), Value::Double(1.0)])
    );
    assert_eq!(
        field(&file, v_knots, "knots"),
        &Value::Arr(vec![
            Value::Double(-0.0740285242331948),
            Value::Double(-0.025971475766802),
        ])
    );

    assert_eq!(file.nodes[&773].code, code::OFFSET_SURF);
    assert_eq!(field(&file, 773, "surface").as_ptr(), Some(1186));
    assert_eq!(field(&file, 1984, "chart").as_ptr(), Some(5059));
    assert_eq!(field(&file, 1984, "intersection_data").as_ptr(), Some(5064));
    assert_eq!(field(&file, 1984, "start").as_ptr(), Some(5062));
    assert_eq!(field(&file, 1984, "end").as_ptr(), Some(5065));
    assert_eq!(field(&file, 5059, "chart_count").as_int(), Some(4));
    for limit in [5062, 5065] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }

    let values = match field(&file, 5064, "values") {
        Value::Arr(values) if values.len() == 16 => values,
        value => panic!("unexpected record-1984 intersection data: {value:?}"),
    };
    let first_trace_u: Vec<_> = values
        .chunks_exact(4)
        .map(|tuple| tuple[0].as_f64().unwrap())
        .collect();
    assert_eq!(
        first_trace_u,
        vec![
            1.0,
            0.748798345691536,
            0.3776079474225015,
            -2.02217766823431e-15,
        ]
    );
    assert!(first_trace_u[..3].iter().all(|u| (0.0..=1.0).contains(u)));
    let source_scaled_slack = 16_384.0 * f64::EPSILON;
    assert!(first_trace_u[3] < 0.0);
    assert!(-first_trace_u[3] <= source_scaled_slack);
    for tuple in values.chunks_exact(4) {
        assert!((-0.0740285242331948..=-0.025971475766802).contains(&tuple[1].as_f64().unwrap()));
        assert!(tuple[2].as_f64().unwrap().is_finite());
        assert!(tuple[3].as_f64().unwrap().is_finite());
    }

    assert_eq!(file.nodes[&5945].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 5945, "surface"),
        &Value::Arr(vec![Value::Ptr(3338), Value::Ptr(773)])
    );
    assert_eq!(file.nodes[&3338].code, code::OFFSET_SURF);
    assert_eq!(file.nodes[&773].code, code::OFFSET_SURF);
    assert_eq!(field(&file, 5945, "chart").as_ptr(), Some(5944));
    assert_eq!(field(&file, 5944, "chart_count").as_int(), Some(3));
}

#[test]
fn v8_certifies_1984_and_stops_atomically_at_the_next_chart_proof() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v8_defaults()),
    )
    .unwrap();
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(
        (
            crossing.stage,
            crossing.resource,
            crossing.consumed,
            crossing.allowed,
        ),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            V8_ATTEMPTED_WORK,
            V8_WORK,
        )
    );
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V8_WORK
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
fn v7_is_stable_and_v8_has_exact_work_items_and_depth_n_minus_one_crossings() {
    assert_eq!(
        limit(
            &IntersectionImportBudgetProfile::v7_defaults(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        )
        .allowed,
        V7_WORK
    );
    assert_eq!(
        limit(
            &IntersectionImportBudgetProfile::v8_defaults(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        )
        .allowed,
        V8_WORK
    );

    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v7_defaults()),
    )
    .unwrap();
    let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(
        (
            crossing.stage,
            crossing.resource,
            crossing.consumed,
            crossing.allowed,
        ),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            V7_ATTEMPTED_WORK,
            V7_WORK,
        )
    );
    assert_rollback(&store);

    for (stage, resource, mode, exact) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            V8_WORK,
        ),
        (
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            22,
        ),
        (
            INTERSECTION_CHART_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            10,
        ),
    ] {
        let mut store = Store::new();
        let outcome = reconstruct_with_context(
            &file,
            &mut store,
            &context_with_limit(&session, stage, resource, mode, exact - 1),
        )
        .unwrap();
        let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
        assert_eq!(
            (
                crossing.stage,
                crossing.resource,
                crossing.consumed,
                crossing.allowed,
            ),
            (stage, resource, exact, exact - 1)
        );
        assert_rollback(&store);
    }
}

#[test]
fn material_or_interior_nonperiodic_overhangs_remain_typed_and_atomic() {
    let mut cases = Vec::new();

    let mut material = read_xt(EXEMPLAR).unwrap();
    transplant_1984(&mut material, 1828);
    let mut values = match field(&material, 5064, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[12] = Value::Double(-1.0e-5);
    set_field(&mut material, 5064, "values", Value::Arr(values));
    cases.push(material);

    let mut interior = read_xt(EXEMPLAR).unwrap();
    transplant_1984(&mut interior, 1828);
    let mut values = match field(&interior, 5064, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[8] = Value::Double(-2.02217766823431e-15);
    set_field(&mut interior, 5064, "values", Value::Arr(values));
    cases.push(interior);

    for file in cases {
        let mut store = Store::new();
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert!(matches!(
            error,
            XtError::IntersectionCertificate {
                index: 1828,
                source: IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace: PairedTrace::First,
                    reason: "transmitted NURBS pcurve leaves the source surface domain",
                },
            }
        ));
        assert_rollback(&store);
    }
}

#[test]
fn normalized_endpoint_still_requires_the_whole_carrier_certificate() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    transplant_1984(&mut file, 1828);
    let mut positions = match field(&file, 5059, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let mut displaced = positions[3].as_vector().unwrap();
    displaced[0] += 1.0e-3;
    positions[3] = Value::Vector(Some(displaced));
    set_field(&mut file, 5059, "hvec", Value::Arr(positions));
    set_field(
        &mut file,
        5065,
        "hvec",
        Value::Arr(vec![Value::Vector(Some(displaced))]),
    );

    let mut store = Store::new();
    let error = reconstruct(&file, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::ResidualExceedsTolerance {
                trace: PairedTrace::First,
                ..
            },
        }
    ));
    assert_rollback(&store);
}
