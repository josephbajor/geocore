//! Canonical finite-open four-sample dual-offset cubic chart contract.

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
const V9_WORK: u64 = 323_814_492;
const V10_WORK: u64 = 336_759_900;
const RECORD_3819_WORK: u64 = 12_945_408;

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn transplant_3819(file: &mut kxt::XtFile, destination: u32) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, 3819, name).clone();
        set_field(file, destination, name, value);
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
fn record_3819_pins_the_canonical_four_sample_dual_offset_payload() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&3819].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 3819, "surface"),
        &Value::Arr(vec![Value::Ptr(3370), Value::Ptr(773)])
    );
    for (root, basis, distance) in [(3370, 3808, -0.0015), (773, 1186, 0.00017)] {
        assert_eq!(file.nodes[&root].code, code::OFFSET_SURF);
        assert_eq!(field(&file, root, "surface").as_ptr(), Some(basis));
        assert_eq!(field(&file, root, "offset").as_f64(), Some(distance));
    }
    assert_eq!(field(&file, 3819, "chart").as_ptr(), Some(5934));
    assert_eq!(field(&file, 3819, "start").as_ptr(), Some(5931));
    assert_eq!(field(&file, 3819, "end").as_ptr(), Some(5933));
    assert_eq!(field(&file, 3819, "intersection_data").as_ptr(), Some(5936));
    assert_eq!(field(&file, 5934, "chart_count").as_int(), Some(4));
    assert_eq!(field(&file, 5934, "base_parameter").as_f64(), Some(0.0));
    assert_eq!(field(&file, 5934, "base_scale").as_f64(), Some(1.0));
    assert_eq!(
        field(&file, 5934, "chordal_error").as_f64(),
        Some(2.200712453324035e-5)
    );
    assert_eq!(
        field(&file, 5934, "hvec"),
        &Value::Arr(vec![
            Value::Vector(Some([
                -0.02349538773162355,
                0.0843271424716185,
                0.00506101406069072,
            ])),
            Value::Vector(Some([
                -0.0234971598764388,
                0.0843420458309177,
                0.00510731623850741,
            ])),
            Value::Vector(Some([
                -0.0235002814564443,
                0.0843706170706803,
                0.00530064924038444,
            ])),
            Value::Vector(Some([
                -0.0235000815370109,
                0.0843725724545112,
                0.00551988917744654,
            ])),
        ])
    );
    for limit in [5931, 5933] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }
    assert_eq!(field(&file, 5936, "uv_type").as_int(), Some(4));
    assert_eq!(
        field(&file, 5936, "values"),
        &Value::Arr(
            [
                0.3044814577934875,
                0.00789527182544393,
                0.257605011608447,
                0.644058110589539,
                0.232117063594252,
                0.00789280100929501,
                0.25745530154119,
                0.644459004930235,
                0.0951106058836419,
                0.00788770457186276,
                0.2571117117543325,
                0.646198217131237,
                3.124887041054185e-14,
                0.00788678300172838,
                0.256977331324844,
                0.648228581722427,
            ]
            .into_iter()
            .map(Value::Double)
            .collect()
        )
    );
}

#[test]
fn v10_certifies_3819_and_stops_before_the_next_quadratic_proof() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v10_defaults()),
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
            345_353_308,
            V10_WORK,
        )
    );
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V10_WORK
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
fn record_3819_transplant_has_exact_isolated_work_and_n_minus_one_boundaries() {
    let session = SessionPolicy::v1();
    for (allowed, expected_usage) in [
        (RECORD_3819_WORK - 1, 0),
        (RECORD_3819_WORK, RECORD_3819_WORK),
    ] {
        let mut file = read_xt(EXEMPLAR).unwrap();
        transplant_3819(&mut file, 1828);
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
        if allowed + 1 == RECORD_3819_WORK {
            assert_eq!(crossing.consumed, RECORD_3819_WORK);
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
    assert_eq!(V9_WORK + RECORD_3819_WORK, V10_WORK);
}

#[test]
fn malformed_record_3819_witnesses_and_limits_fail_typed_and_atomically() {
    let mut duplicate_position = read_xt(EXEMPLAR).unwrap();
    transplant_3819(&mut duplicate_position, 1828);
    let mut positions = match field(&duplicate_position, 5934, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    positions[1] = positions[0].clone();
    set_field(&mut duplicate_position, 5934, "hvec", Value::Arr(positions));
    let mut store = Store::new();
    let error = reconstruct(&duplicate_position, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset cubic carrier samples must be pairwise distinct",
            },
        }
    ));
    assert_rollback(&store);

    let mut duplicate_uv = read_xt(EXEMPLAR).unwrap();
    transplant_3819(&mut duplicate_uv, 1828);
    let mut values = match field(&duplicate_uv, 5936, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[4] = values[0].clone();
    values[5] = values[1].clone();
    set_field(&mut duplicate_uv, 5936, "values", Value::Arr(values));
    let mut store = Store::new();
    let error = reconstruct(&duplicate_uv, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::First,
                reason: "dual-offset cubic pcurve samples must be pairwise distinct",
            },
        }
    ));
    assert_rollback(&store);

    let mut displaced = read_xt(EXEMPLAR).unwrap();
    transplant_3819(&mut displaced, 1828);
    let mut positions = match field(&displaced, 5934, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let mut middle = positions[1].as_vector().unwrap();
    middle[0] += 1.0e-3;
    positions[1] = Value::Vector(Some(middle));
    set_field(&mut displaced, 5934, "hvec", Value::Arr(positions));
    let mut store = Store::new();
    let error = reconstruct(&displaced, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::ResidualExceedsTolerance { .. },
        }
    ));
    assert_rollback(&store);

    let mut equal_limit = read_xt(EXEMPLAR).unwrap();
    transplant_3819(&mut equal_limit, 1828);
    set_field(&mut equal_limit, 1828, "end", Value::Ptr(5931));
    let mut store = Store::new();
    let error = reconstruct(&equal_limit, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "only one shared closed LIMIT type H with term_use ? is supported for an equal-limit chart",
        }
    ));
    assert_rollback(&store);
}
