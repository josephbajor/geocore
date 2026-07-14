//! Zero-multiplicity null-knot padding and the next quadratic chart rung.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use ktopo::entity::Body;
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use kxt::parse::{Value, read_xt};
use kxt::{
    INTERSECTION_CHART_CERTIFICATE_WORK, INTERSECTION_CHART_DEPTH, INTERSECTION_CHART_ITEMS,
    IntersectionImportBudgetProfile, XtCapability, XtError, reconstruct, reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const V10_WORK: u64 = 336_759_900;
const RECORD_3790_WORK: u64 = 8_593_408;
const RECORD_3745_WORK: u64 = 42_772_491;
const V11_WORK: u64 = 388_125_799;

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn transplant_3790(file: &mut kxt::XtFile, destination: u32) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, 3790, name).clone();
        set_field(file, destination, name, value);
    }
}

fn context_with_plan<'a>(
    session: &'a SessionPolicy,
    plan: kcore::operation::BudgetPlan,
) -> OperationContext<'a> {
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
fn record_3790_source_pins_zero_multiplicity_null_knot_padding() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(
        field(&file, 3790, "surface"),
        &Value::Arr(vec![Value::Ptr(3351), Value::Ptr(773)])
    );
    assert_eq!(field(&file, 3790, "chart").as_ptr(), Some(6033));
    assert_eq!(field(&file, 6033, "chart_count").as_int(), Some(3));
    assert_eq!(field(&file, 3351, "surface").as_ptr(), Some(3771));
    assert_eq!(field(&file, 3771, "nurbs").as_ptr(), Some(3783));
    for flag in ["u_periodic", "v_periodic", "u_closed", "v_closed"] {
        assert_eq!(field(&file, 3783, flag), &Value::Logical(false));
    }
    assert_eq!(field(&file, 3783, "n_u_knots").as_int(), Some(2));
    assert_eq!(field(&file, 3783, "u_knot_mult").as_ptr(), Some(6756));
    assert_eq!(field(&file, 3783, "u_knots").as_ptr(), Some(6764));
    assert_eq!(
        field(&file, 6756, "mult"),
        &Value::Arr(vec![Value::Int(3), Value::Int(3), Value::Int(0)])
    );
    assert_eq!(
        field(&file, 6764, "knots"),
        &Value::Arr(vec![
            Value::Double(0.6171875),
            Value::Double(1.0),
            Value::Null,
        ])
    );
    assert_eq!(V10_WORK + RECORD_3790_WORK + RECORD_3745_WORK, V11_WORK);

    assert_eq!(
        field(&file, 3745, "surface"),
        &Value::Arr(vec![Value::Ptr(3359), Value::Ptr(773)])
    );
    assert_eq!(field(&file, 3745, "chart").as_ptr(), Some(6036));
    assert_eq!(field(&file, 6036, "chart_count").as_int(), Some(11));
    assert_eq!(file.nodes[&3359].code, kxt::schema::code::PLANE);
    assert_eq!(file.nodes[&773].code, kxt::schema::code::OFFSET_SURF);
}

#[test]
fn malformed_zero_multiplicity_padding_fails_typed_and_atomically() {
    for malformed in [Value::Char('x'), Value::Double(f64::NAN)] {
        let mut file = read_xt(EXEMPLAR).unwrap();
        transplant_3790(&mut file, 1828);
        let mut knots = match field(&file, 6764, "knots").clone() {
            Value::Arr(values) => values,
            _ => unreachable!(),
        };
        knots[2] = malformed;
        set_field(&mut file, 6764, "knots", Value::Arr(knots));
        let mut store = Store::new();
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert!(matches!(
            error,
            XtError::BadField {
                index: 6764,
                what: "zero-multiplicity knot padding is neither null nor finite numeric",
            }
        ));
        assert_rollback(&store);
    }

    let mut positive_null = read_xt(EXEMPLAR).unwrap();
    transplant_3790(&mut positive_null, 1828);
    set_field(
        &mut positive_null,
        6756,
        "mult",
        Value::Arr(vec![Value::Int(3), Value::Int(3), Value::Int(1)]),
    );
    let mut store = Store::new();
    let error = reconstruct(&positive_null, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::BadField {
            index: 6764,
            what: "positive-multiplicity knot is null, non-numeric, or non-finite",
        }
    ));
    assert_rollback(&store);
}

#[test]
fn record_3790_transplant_has_exact_isolated_work_and_n_minus_one_boundaries() {
    let session = SessionPolicy::v1();
    for (allowed, expected_usage) in [
        (RECORD_3790_WORK - 1, 0),
        (RECORD_3790_WORK, RECORD_3790_WORK),
    ] {
        let mut file = read_xt(EXEMPLAR).unwrap();
        transplant_3790(&mut file, 1828);
        let plan = BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                10,
            ),
        ])
        .unwrap();
        let mut store = Store::new();
        let outcome =
            reconstruct_with_context(&file, &mut store, &context_with_plan(&session, plan))
                .unwrap();
        let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
        if allowed + 1 == RECORD_3790_WORK {
            assert_eq!(crossing.consumed, RECORD_3790_WORK);
        }
        assert_eq!(
            usage(
                outcome.report(),
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
            ),
            expected_usage
        );
        assert_rollback(&store);
    }
}

#[test]
fn v11_certifies_3790_and_stops_at_the_count_seven_dual_offset_family() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v11_defaults()),
    )
    .unwrap();
    assert!(
        matches!(
            outcome.result(),
            Err(XtError::Unsupported {
                capability: XtCapability::IntersectionSurfaceFamily,
                what: "dual Offset(B-surface) charts require a canonical finite-open three-sample quadratic or four-sample cubic family",
            })
        ),
        "{:?}",
        outcome.result()
    );
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V11_WORK
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
