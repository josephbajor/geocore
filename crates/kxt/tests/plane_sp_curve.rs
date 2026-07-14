//! Production native Plane SP-curve reconstruction contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
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
const V5_WORK: u64 = 117_478_445;
const V6_WORK: u64 = 208_228_426;

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

fn assert_post_ring_chart_data_boundary(error: &XtError) {
    assert!(
        matches!(
            error,
            XtError::Unsupported {
                capability: XtCapability::IntersectionChartData,
                what: "INTERSECTION_DATA contains null or non-finite UV values",
            }
        ),
        "unexpected post-SP-curve boundary: {error:?}"
    );
}

#[test]
fn node_30_and_face_1195_pin_the_exact_plane_lift_and_ring_topology() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&30].code, code::SP_CURVE);
    assert_eq!(field(&file, 30, "sense").as_char(), Some('+'));
    assert_eq!(field(&file, 30, "surface").as_ptr(), Some(1951));
    assert_eq!(field(&file, 30, "b_curve").as_ptr(), Some(2254));
    assert_eq!(field(&file, 30, "original").as_ptr(), Some(0));
    assert_eq!(field(&file, 30, "tolerance_to_original"), &Value::Null);

    assert_eq!(file.nodes[&1951].code, code::PLANE);
    assert_eq!(
        field(&file, 1951, "pvec").as_vector(),
        Some([-0.04, 0.0921335816383362, 0.0035])
    );
    assert_eq!(
        field(&file, 1951, "normal").as_vector(),
        Some([0.0, 0.0, -1.0])
    );
    assert_eq!(
        field(&file, 1951, "x_axis").as_vector(),
        Some([-1.0, 0.0, 0.0])
    );

    assert_eq!(file.nodes[&2254].code, code::B_CURVE);
    assert_eq!(field(&file, 2254, "sense").as_char(), Some('+'));
    assert_eq!(field(&file, 2254, "nurbs").as_ptr(), Some(6893));
    assert_eq!(field(&file, 6893, "degree").as_int(), Some(2));
    assert_eq!(field(&file, 6893, "n_vertices").as_int(), Some(13));
    assert_eq!(field(&file, 6893, "vertex_dim").as_int(), Some(2));
    assert_eq!(field(&file, 6893, "periodic"), &Value::Logical(false));
    assert_eq!(field(&file, 6893, "closed"), &Value::Logical(false));
    assert_eq!(field(&file, 6893, "rational"), &Value::Logical(false));

    let poles = match field(&file, 6898, "vertices") {
        Value::Arr(values) if values.len() == 26 => values,
        value => panic!("unexpected 2D SP-curve poles: {value:?}"),
    };
    let lift = |u: f64, v: f64| [-0.04 - u, 0.0921335816383362 + v, 0.0035];
    let first = lift(poles[0].as_f64().unwrap(), poles[1].as_f64().unwrap());
    let last = lift(poles[24].as_f64().unwrap(), poles[25].as_f64().unwrap());
    for (actual, lifted) in [
        (field(&file, 4619, "point_1").as_vector().unwrap(), first),
        (field(&file, 4619, "point_2").as_vector().unwrap(), last),
    ] {
        assert!(
            actual
                .into_iter()
                .zip(lifted)
                .all(|(actual, lifted)| (actual - lifted).abs() <= 1.0e-16)
        );
    }
    assert_eq!(field(&file, 4619, "parm_1").as_f64(), Some(0.0));
    assert_eq!(field(&file, 4619, "parm_2").as_f64(), Some(1.0));

    assert_eq!(field(&file, 1195, "surface").as_ptr(), Some(1951));
    assert_eq!(field(&file, 1195, "loop").as_ptr(), Some(2196));
    assert_eq!(field(&file, 2196, "next").as_ptr(), Some(4723));
    assert_eq!(field(&file, 4723, "fin").as_ptr(), Some(4724));
    assert_eq!(field(&file, 4724, "vertex").as_ptr(), Some(0));
    assert_eq!(field(&file, 4724, "edge").as_ptr(), Some(2210));
    assert_eq!(field(&file, 2210, "curve").as_ptr(), Some(2008));
    assert_eq!(file.nodes[&2008].code, code::INTERSECTION);

    assert_eq!(file.nodes[&5089].code, code::INTERSECTION);
    assert_eq!(field(&file, 5089, "intersection_data").as_ptr(), Some(5092));
    let next_values = match field(&file, 5092, "values") {
        Value::Arr(values) => values,
        value => panic!("unexpected next INTERSECTION_DATA values: {value:?}"),
    };
    assert_eq!(&next_values[8..10], &[Value::Null, Value::Null]);
}

#[test]
fn v6_lifts_node_30_and_advances_atomically_past_the_ring_domain_boundary() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v6_defaults()),
    )
    .unwrap();
    assert_post_ring_chart_data_boundary(outcome.result().as_ref().unwrap_err());
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V6_WORK
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
fn historical_profiles_are_stable_and_v5_stops_at_the_newly_exposed_proof() {
    for (profile, expected) in [
        (IntersectionImportBudgetProfile::v1_defaults(), 131_072),
        (IntersectionImportBudgetProfile::v2_defaults(), 81_267_732),
        (IntersectionImportBudgetProfile::v3_defaults(), 115_485_725),
        (IntersectionImportBudgetProfile::v4_defaults(), 116_396_069),
        (IntersectionImportBudgetProfile::v5_defaults(), V5_WORK),
        (IntersectionImportBudgetProfile::v6_defaults(), V6_WORK),
    ] {
        assert_eq!(
            limit(
                &profile,
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
            )
            .allowed,
            expected
        );
    }

    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v5_defaults()),
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
            118_406_196,
            V5_WORK,
        )
    );
    assert!(outcome.report().limit_events().is_empty());
    assert_rollback(&store);
}

#[test]
fn v6_has_exact_work_items_and_depth_n_minus_one_crossings() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    for (stage, resource, mode, exact) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            V6_WORK,
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
fn approximated_plane_sp_curve_remains_typed_and_atomic() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    set_field(&mut file, 30, "original", Value::Ptr(2253));

    let mut store = Store::new();
    let error = reconstruct(&file, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::Unsupported {
            capability: XtCapability::ProceduralCurves,
            what: "only native Plane SP_CURVEs without an original or approximation tolerance are supported",
        }
    ));
    assert_rollback(&store);
}
