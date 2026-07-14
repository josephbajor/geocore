//! Production equal-limit transmitted-intersection contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use ktopo::entity::Body;
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use kxt::parse::{Value, read_xt};
use kxt::schema::code;
use kxt::{
    INTERSECTION_CHART_CERTIFICATE_WORK, INTERSECTION_CHART_DEPTH, INTERSECTION_CHART_ITEMS,
    XtCapability, XtError, reconstruct, reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const EQUAL_LIMIT_WORK: u64 = 115_485_725;
const RECORD_2008_TRANSPLANT_WORK: u64 = 124_040_223;

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn context_with_work<'a>(session: &'a SessionPolicy, work: u64) -> OperationContext<'a> {
    context_with_limit(
        session,
        INTERSECTION_CHART_CERTIFICATE_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        work,
    )
}

fn context_with_limit<'a>(
    session: &'a SessionPolicy,
    stage: kcore::operation::StageId,
    resource: ResourceKind,
    mode: AccountingMode,
    allowed: u64,
) -> OperationContext<'a> {
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap(),
        )
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

fn assert_terminated_work_boundary(error: &XtError, consumed: u64, allowed: u64) {
    let limit = error.limit().expect("terminated proof must reach Work cap");
    assert_eq!(
        (limit.stage, limit.resource, limit.consumed, limit.allowed),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            consumed,
            allowed,
        ),
        "unexpected exemplar boundary: {error:?}"
    );
}

fn assert_rollback(store: &Store) {
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<CurveGeom>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

#[test]
fn exemplar_pins_both_equal_limit_records_and_exact_period_unwraps() {
    let file = read_xt(EXEMPLAR).unwrap();
    for (intersection, sources, chart, data, limit, count) in [
        (1828, [1190, 1951], 2202, 2204, 2205, 20),
        (2008, [1951, 773], 2206, 2214, 2208, 22),
    ] {
        assert_eq!(file.nodes[&intersection].code, code::INTERSECTION);
        assert_eq!(
            field(&file, intersection, "surface"),
            &Value::Arr(sources.into_iter().map(Value::Ptr).collect())
        );
        assert_eq!(field(&file, intersection, "chart").as_ptr(), Some(chart));
        assert_eq!(
            field(&file, intersection, "intersection_data").as_ptr(),
            Some(data)
        );
        assert_eq!(field(&file, intersection, "start").as_ptr(), Some(limit));
        assert_eq!(field(&file, intersection, "end").as_ptr(), Some(limit));
        assert_eq!(field(&file, limit, "type").as_char(), Some('H'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
        assert_eq!(field(&file, chart, "chart_count").as_int(), Some(count));
        assert_eq!(field(&file, chart, "base_parameter").as_f64(), Some(0.0));
        assert_eq!(field(&file, chart, "base_scale").as_f64(), Some(1.0));
        assert_eq!(field(&file, data, "uv_type").as_int(), Some(4));
    }

    let values_1828 = match field(&file, 2204, "values") {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let original_last_1828 = values_1828[76].as_f64().unwrap();
    assert_eq!(values_1828[0].as_f64(), Some(0.0));
    assert_eq!(values_1828[4].as_f64(), Some(0.0777171431611849));
    assert_eq!(values_1828[72].as_f64(), Some(0.944104317629517));
    assert_eq!(original_last_1828, 2.466693516112175e-12);
    assert_eq!((original_last_1828 + 1.0) - original_last_1828, 1.0);

    let values_2008 = match field(&file, 2214, "values") {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    assert_eq!(values_2008[2].as_f64(), Some(0.0));
    assert_eq!(values_2008[6].as_f64(), Some(0.960520220522793));
    assert_eq!(values_2008[82].as_f64(), Some(0.0222478493831891));
    assert_eq!(values_2008[86].as_f64(), Some(1.0));
    assert_eq!(0.0 + 1.0, 1.0);
    assert_eq!(1.0 - 1.0, 0.0);
}

#[test]
fn record_1828_certifies_and_advances_to_the_terminated_limit_boundary() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_work(&session, EQUAL_LIMIT_WORK),
    )
    .unwrap();
    assert_terminated_work_boundary(
        outcome.result().as_ref().unwrap_err(),
        116_396_069,
        EQUAL_LIMIT_WORK,
    );
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        EQUAL_LIMIT_WORK
    );
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
        ),
        20
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
fn equal_limit_profile_has_exact_work_items_and_depth_n_minus_one_crossings() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    for (stage, resource, mode, exact) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            EQUAL_LIMIT_WORK,
        ),
        (
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            20,
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
        let limit = outcome.result().as_ref().unwrap_err().limit().unwrap();
        assert_eq!(
            (limit.stage, limit.resource, limit.consumed, limit.allowed),
            (stage, resource, exact, exact - 1)
        );
        assert_rollback(&store);
    }
}

#[test]
fn record_2008_payload_certifies_independently_of_the_earlier_terminated_boundary() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(&file, 2008, name).clone();
        set_field(&mut file, 1828, name, value);
    }

    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_work(&session, RECORD_2008_TRANSPLANT_WORK),
    )
    .unwrap();
    assert_terminated_work_boundary(
        outcome.result().as_ref().unwrap_err(),
        124_950_567,
        RECORD_2008_TRANSPLANT_WORK,
    );
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        RECORD_2008_TRANSPLANT_WORK
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
fn malformed_equal_limits_and_noncanonical_periodic_endpoints_remain_typed_and_atomic() {
    let mut cases = Vec::new();

    let mut null = read_xt(EXEMPLAR).unwrap();
    set_field(&mut null, 1828, "end", Value::Ptr(0));
    cases.push((null, "transmitted intersection has a null limit"));

    let mut distinct_closed = read_xt(EXEMPLAR).unwrap();
    set_field(&mut distinct_closed, 1828, "end", Value::Ptr(2208));
    cases.push((
        distinct_closed,
        "only finite open LIMIT type L with term_use ? is supported",
    ));

    let mut off_seam = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&off_seam, 2204, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Double(0.25);
    set_field(&mut off_seam, 2204, "values", Value::Arr(values));
    cases.push((
        off_seam,
        "equal-limit chart endpoints are not on one certified periodic seam",
    ));

    let mut store = Store::new();
    for (file, expected) in cases {
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert!(matches!(
            error,
            XtError::Unsupported {
                capability: XtCapability::IntersectionLimits,
                what,
            } if what == expected
        ));
        assert_rollback(&store);
    }
}
