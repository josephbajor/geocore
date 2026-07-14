//! Production end-terminated transmitted-intersection contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use ktopo::entity::Body;
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use kxt::parse::{Value, read_xt};
use kxt::{
    INTERSECTION_CHART_CERTIFICATE_WORK, INTERSECTION_CHART_DEPTH, INTERSECTION_CHART_ITEMS,
    IntersectionImportBudgetProfile, XtCapability, XtError, reconstruct, reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const EQUAL_LIMIT_WORK: u64 = 115_485_725;
const TERMINATED_WORK: u64 = 116_396_069;
const RECORD_1678_TRANSPLANT_WORK: u64 = 116_413_476;

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

fn assert_next_work_boundary(error: &XtError, consumed: u64, allowed: u64) {
    let limit = error
        .limit()
        .expect("next chart must reach the v4 Work cap");
    assert_eq!(
        (limit.stage, limit.resource, limit.consumed, limit.allowed),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            consumed,
            allowed,
        ),
        "unexpected post-terminator boundary: {error:?}"
    );
}

fn assert_rollback(store: &Store) {
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<CurveGeom>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

fn transplant_intersection(file: &mut kxt::XtFile, destination: u32, source: u32) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, source, name).clone();
        set_field(file, destination, name, value);
    }
}

#[test]
fn exemplar_pins_both_end_terminated_records_and_extra_singularity_tuples() {
    let file = read_xt(EXEMPLAR).unwrap();
    for (intersection, sources, chart, start, end, data, count) in [
        (1671, [1428, 1951], 2244, 2245, 2242, 2243, 7_i64),
        (1678, [1951, 2250], 2246, 2249, 2247, 2251, 6_i64),
    ] {
        assert_eq!(
            field(&file, intersection, "surface"),
            &Value::Arr(sources.into_iter().map(Value::Ptr).collect())
        );
        assert_eq!(field(&file, intersection, "chart").as_ptr(), Some(chart));
        assert_eq!(field(&file, intersection, "start").as_ptr(), Some(start));
        assert_eq!(field(&file, intersection, "end").as_ptr(), Some(end));
        assert_eq!(
            field(&file, intersection, "intersection_data").as_ptr(),
            Some(data)
        );
        assert_eq!(field(&file, chart, "chart_count").as_int(), Some(count));
        assert_eq!(field(&file, start, "type").as_char(), Some('L'));
        assert_eq!(field(&file, start, "term_use").as_char(), Some('?'));
        assert_eq!(field(&file, end, "type").as_char(), Some('T'));
        assert_eq!(field(&file, end, "term_use").as_char(), Some('F'));
        let limit_positions = match field(&file, end, "hvec") {
            Value::Arr(values) if values.len() == 2 => values,
            value => panic!("unexpected terminator positions: {value:?}"),
        };
        let chart_positions = match field(&file, chart, "hvec") {
            Value::Arr(values) if values.len() == count as usize => values,
            value => panic!("unexpected chart positions: {value:?}"),
        };
        let branch = limit_positions[1].as_vector().unwrap();
        let chart_end = chart_positions.last().unwrap().as_vector().unwrap();
        assert!(
            branch
                .into_iter()
                .zip(chart_end)
                .all(|(branch, chart)| (branch - chart).abs() <= 1.0e-16)
        );
        assert_ne!(limit_positions[0], limit_positions[1]);
        assert!(matches!(
            field(&file, data, "values"),
            Value::Arr(values) if values.len() == (count as usize + 1) * 4
        ));
    }
}

#[test]
fn first_end_terminator_certifies_and_v4_stops_at_the_next_proof() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v4_defaults()),
    )
    .unwrap();
    assert_next_work_boundary(
        outcome.result().as_ref().unwrap_err(),
        117_478_445,
        TERMINATED_WORK,
    );
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        TERMINATED_WORK
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
fn profiles_retain_v1_v2_v3_caps_and_cross_exact_v4_limits() {
    let v1 = IntersectionImportBudgetProfile::v1_defaults();
    let v2 = IntersectionImportBudgetProfile::v2_defaults();
    let v3 = IntersectionImportBudgetProfile::v3_defaults();
    let v4 = IntersectionImportBudgetProfile::v4_defaults();
    assert_eq!(
        limit(&v1, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        131_072
    );
    assert_eq!(
        limit(&v2, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        81_267_732
    );
    assert_eq!(
        limit(&v3, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        EQUAL_LIMIT_WORK
    );
    assert_eq!(
        limit(&v4, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        TERMINATED_WORK
    );

    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    for (stage, resource, mode, exact) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            TERMINATED_WORK,
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
fn second_terminated_payload_certifies_independently_in_the_first_slot() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    transplant_intersection(&mut file, 1671, 1678);

    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_limit(
            &session,
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            RECORD_1678_TRANSPLANT_WORK,
        ),
    )
    .unwrap();
    assert_next_work_boundary(
        outcome.result().as_ref().unwrap_err(),
        117_495_852,
        RECORD_1678_TRANSPLANT_WORK,
    );
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        RECORD_1678_TRANSPLANT_WORK
    );
    assert_rollback(&store);
}

#[test]
fn malformed_terminator_branch_and_unpaired_plane_uv_remain_typed_and_atomic() {
    let mut branch_mismatch = read_xt(EXEMPLAR).unwrap();
    let mut limit_positions = match field(&branch_mismatch, 2242, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let mut branch = limit_positions[1].as_vector().unwrap();
    branch[2] += 0.01;
    limit_positions[1] = Value::Vector(Some(branch));
    set_field(
        &mut branch_mismatch,
        2242,
        "hvec",
        Value::Arr(limit_positions),
    );

    let mut unpaired_plane_uv = read_xt(EXEMPLAR).unwrap();
    let mut data_values = match field(&unpaired_plane_uv, 2243, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    data_values[6] = Value::Double(0.0);
    set_field(
        &mut unpaired_plane_uv,
        2243,
        "values",
        Value::Arr(data_values),
    );

    let mut store = Store::new();
    let branch_error = reconstruct(&branch_mismatch, &mut store).unwrap_err();
    assert!(matches!(
        branch_error,
        XtError::BadField {
            index: 1671,
            what: "INTERSECTION terminator branch point does not match the CHART endpoint",
        }
    ));
    assert_rollback(&store);

    let data_error = reconstruct(&unpaired_plane_uv, &mut store).unwrap_err();
    assert!(matches!(
        data_error,
        XtError::Unsupported {
            capability: XtCapability::IntersectionChartData,
            what: "INTERSECTION_DATA contains null or non-finite UV values",
        }
    ));
    assert_rollback(&store);
}
