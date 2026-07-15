//! Production finite-open Plane/B-surface omitted-data contract.

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
    IntersectionImportBudgetProfile, XtCapability, XtError, reconstruct, reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const V4_WORK: u64 = 116_396_069;
const V5_WORK: u64 = 117_478_445;
const NONCANONICAL_FIVE_SAMPLE_WORK: u64 = 127_115_320;
const NONCANONICAL_FIVE_SAMPLE_PRIOR_WORK: u64 = 37_293_090;
const NONCANONICAL_BASE_PARAMETER: f64 = 0.003_586_209_316_397_325;
const NONCANONICAL_BASE_SCALE: f64 = 0.999_999_996_408_403;

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
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

fn assert_v5_work_boundary(error: &XtError) {
    let limit = error.limit().expect("v5 must stop at the next chart proof");
    assert_eq!(
        (limit.stage, limit.resource, limit.consumed, limit.allowed),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            118_406_196,
            V5_WORK,
        ),
        "unexpected post-v5 boundary: {error:?}"
    );
}

fn make_record_1252_five_sample_affine_noncanonical(file: &mut kxt::XtFile) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, 1252, name).clone();
        set_field(file, 1828, name, value);
    }
    let mut positions = match field(file, 2234, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    positions.truncate(5);
    let endpoint = positions[4].as_vector().unwrap();
    set_field(file, 2234, "chart_count", Value::Int(5));
    set_field(file, 2234, "hvec", Value::Arr(positions));
    set_field(
        file,
        2240,
        "hvec",
        Value::Arr(vec![Value::Vector(Some(endpoint))]),
    );

    let mut values = match field(file, 2237, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values.truncate(20);
    let origin = field(file, 1951, "pvec").as_vector().unwrap();
    values[18] = Value::Double(-(endpoint[0] - origin[0]));
    values[19] = Value::Double(endpoint[1] - origin[1]);
    set_field(file, 2237, "values", Value::Arr(values));
    set_field(
        file,
        2234,
        "base_parameter",
        Value::Double(NONCANONICAL_BASE_PARAMETER),
    );
    set_field(
        file,
        2234,
        "base_scale",
        Value::Double(NONCANONICAL_BASE_SCALE),
    );
}

#[test]
fn record_1252_pins_interior_plane_uv_omissions_and_exact_inversion() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&1206].code, code::B_SURFACE);
    assert_eq!(file.nodes[&1951].code, code::PLANE);
    assert_eq!(
        field(&file, 1252, "surface"),
        &Value::Arr(vec![Value::Ptr(1206), Value::Ptr(1951)])
    );
    assert_eq!(field(&file, 1252, "chart").as_ptr(), Some(2234));
    assert_eq!(field(&file, 1252, "start").as_ptr(), Some(2236));
    assert_eq!(field(&file, 1252, "end").as_ptr(), Some(2240));
    assert_eq!(field(&file, 1252, "intersection_data").as_ptr(), Some(2237));
    assert_eq!(field(&file, 2234, "chart_count").as_int(), Some(8));
    for limit in [2236, 2240] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }
    let positions = match field(&file, 2234, "hvec") {
        Value::Arr(values) if values.len() == 8 => values,
        value => panic!("unexpected chart: {value:?}"),
    };
    let values = match field(&file, 2237, "values") {
        Value::Arr(values) if values.len() == 32 => values,
        value => panic!("unexpected data: {value:?}"),
    };
    let plane_origin = field(&file, 1951, "pvec").as_vector().unwrap();
    for (sample, (position, tuple)) in positions.iter().zip(values.chunks_exact(4)).enumerate() {
        assert!(tuple[0].as_f64().unwrap().is_finite());
        assert!(tuple[1].as_f64().unwrap().is_finite());
        let position = position.as_vector().unwrap();
        let exact_plane_uv = [
            -(position[0] - plane_origin[0]),
            position[1] - plane_origin[1],
        ];
        if sample == 0 || sample == 7 {
            assert!((tuple[2].as_f64().unwrap() - exact_plane_uv[0]).abs() <= 1.0e-16);
            assert!((tuple[3].as_f64().unwrap() - exact_plane_uv[1]).abs() <= 1.0e-16);
        } else {
            assert_eq!(&tuple[2..], &[Value::Null, Value::Null]);
        }
    }
}

#[test]
fn v5_certifies_record_1252_and_stops_at_the_newly_exposed_chart_proof() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&30].code, code::SP_CURVE);
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v5_defaults()),
    )
    .unwrap();
    assert_v5_work_boundary(outcome.result().as_ref().unwrap_err());
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V5_WORK
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
fn v4_cap_is_stable_and_v5_has_exact_n_minus_one_crossings() {
    let v4 = IntersectionImportBudgetProfile::v4_defaults();
    let v5 = IntersectionImportBudgetProfile::v5_defaults();
    assert_eq!(
        limit(&v4, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        V4_WORK
    );
    assert_eq!(
        limit(&v5, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        V5_WORK
    );

    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    for (stage, resource, mode, exact) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            V5_WORK,
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
fn direct_plane_or_nurbs_endpoint_uv_omissions_remain_typed_and_atomic() {
    let mut cases = Vec::new();

    let mut plane_endpoint = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&plane_endpoint, 2237, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[2] = Value::Null;
    values[3] = Value::Null;
    set_field(&mut plane_endpoint, 2237, "values", Value::Arr(values));
    cases.push(plane_endpoint);

    let mut nurbs_endpoint = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&nurbs_endpoint, 2237, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Null;
    values[1] = Value::Null;
    set_field(&mut nurbs_endpoint, 2237, "values", Value::Arr(values));
    cases.push(nurbs_endpoint);

    for file in cases {
        let mut store = Store::new();
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert!(matches!(
            error,
            XtError::Unsupported {
                capability: XtCapability::IntersectionChartData,
                what: "INTERSECTION_DATA contains null or non-finite UV values",
            }
        ));
        assert_rollback(&store);
    }
}

#[test]
fn affine_noncanonical_five_sample_record_1252_has_exact_work_and_rollback() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    make_record_1252_five_sample_affine_noncanonical(&mut file);
    assert_eq!(
        field(&file, 1828, "surface"),
        &Value::Arr(vec![Value::Ptr(1206), Value::Ptr(1951)])
    );
    assert_eq!(field(&file, 2234, "chart_count").as_int(), Some(5));
    assert_eq!(
        field(&file, 2234, "base_parameter").as_f64(),
        Some(NONCANONICAL_BASE_PARAMETER)
    );
    assert_eq!(
        field(&file, 2234, "base_scale").as_f64(),
        Some(NONCANONICAL_BASE_SCALE)
    );
    let endpoint = match field(&file, 2234, "hvec") {
        Value::Arr(positions) => positions[4].as_vector().unwrap(),
        value => panic!("unexpected chart: {value:?}"),
    };
    let values = match field(&file, 2237, "values") {
        Value::Arr(values) if values.len() == 20 => values,
        value => panic!("unexpected data: {value:?}"),
    };
    let plane_origin = field(&file, 1951, "pvec").as_vector().unwrap();
    let exact_plane_uv = [
        -(endpoint[0] - plane_origin[0]),
        endpoint[1] - plane_origin[1],
    ];
    assert!((values[18].as_f64().unwrap() - exact_plane_uv[0]).abs() <= 1.0e-16);
    assert!((values[19].as_f64().unwrap() - exact_plane_uv[1]).abs() <= 1.0e-16);

    let session = SessionPolicy::v1();
    for allowed in [
        NONCANONICAL_FIVE_SAMPLE_WORK - 1,
        NONCANONICAL_FIVE_SAMPLE_WORK,
    ] {
        let mut store = Store::new();
        let outcome = reconstruct_with_context(
            &file,
            &mut store,
            &context_with_limit(
                &session,
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            ),
        )
        .unwrap();
        if allowed + 1 == NONCANONICAL_FIVE_SAMPLE_WORK {
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
                    NONCANONICAL_FIVE_SAMPLE_WORK,
                    allowed,
                )
            );
            assert_eq!(
                usage(
                    outcome.report(),
                    INTERSECTION_CHART_CERTIFICATE_WORK,
                    ResourceKind::Work,
                ),
                NONCANONICAL_FIVE_SAMPLE_PRIOR_WORK
            );
        } else {
            assert_eq!(
                usage(
                    outcome.report(),
                    INTERSECTION_CHART_CERTIFICATE_WORK,
                    ResourceKind::Work,
                ),
                NONCANONICAL_FIVE_SAMPLE_WORK
            );
        }
        assert_rollback(&store);
    }
}
