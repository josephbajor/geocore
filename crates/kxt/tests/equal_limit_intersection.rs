//! Production equal-limit transmitted-intersection contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use kgraph::{IntersectionCertificateError, PairedTrace};
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

fn alias_record_1828_interior_period(file: &mut kxt::XtFile, shift: f64) {
    let mut values = match field(file, 2204, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    for sample in 10..19 {
        let index = sample * 4;
        values[index] = Value::Double(values[index].as_f64().unwrap() + shift);
    }
    set_field(file, 2204, "values", Value::Arr(values));
}

fn make_record_1828_limits_distinct(file: &mut kxt::XtFile) -> u32 {
    let duplicate = file.nodes.keys().copied().max().unwrap() + 1;
    let limit = file.nodes[&2205].clone();
    file.nodes.insert(duplicate, limit);
    set_field(file, 1828, "end", Value::Ptr(duplicate));
    duplicate
}

fn assert_equal_limit_resource_boundaries(file: &kxt::XtFile) {
    let session = SessionPolicy::v1();
    for (stage, resource, mode, exact, prior) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            EQUAL_LIMIT_WORK,
            81_267_732,
        ),
        (
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            20,
            0,
        ),
        (
            INTERSECTION_CHART_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            10,
            0,
        ),
    ] {
        for allowed in [exact - 1, exact] {
            let mut store = Store::new();
            let outcome = reconstruct_with_context(
                file,
                &mut store,
                &context_with_limit(&session, stage, resource, mode, allowed),
            )
            .unwrap();
            if allowed + 1 == exact {
                let limit = outcome.result().as_ref().unwrap_err().limit().unwrap();
                assert_eq!(
                    (limit.stage, limit.resource, limit.consumed, limit.allowed),
                    (stage, resource, exact, allowed)
                );
                assert_eq!(usage(outcome.report(), stage, resource), prior);
            } else {
                assert_eq!(usage(outcome.report(), stage, resource), exact);
            }
            assert_rollback(&store);
        }
    }
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
fn interior_period_alias_has_exact_work_items_depth_boundaries_and_rollback() {
    let original = read_xt(EXEMPLAR).unwrap();
    assert_eq!(original.nodes[&1190].code, code::OFFSET_SURF);
    let basis = field(&original, 1190, "surface").as_ptr().unwrap();
    assert_eq!(original.nodes[&basis].code, code::B_SURFACE);
    let nurbs = field(&original, basis, "nurbs").as_ptr().unwrap();
    assert_eq!(field(&original, nurbs, "u_periodic"), &Value::Logical(true));
    assert_eq!(
        field(&original, nurbs, "v_periodic"),
        &Value::Logical(false)
    );
    let u_knots = field(&original, nurbs, "u_knots").as_ptr().unwrap();
    let u_knots = match field(&original, u_knots, "knots") {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    assert_eq!(u_knots.first().and_then(Value::as_f64), Some(0.0));
    assert_eq!(u_knots.last().and_then(Value::as_f64), Some(1.0));
    let mut aliased = read_xt(EXEMPLAR).unwrap();
    alias_record_1828_interior_period(&mut aliased, 1.0);
    let original_values = match field(&original, 2204, "values") {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let aliased_values = match field(&aliased, 2204, "values") {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    for sample in 10..19 {
        let index = sample * 4;
        assert_eq!(
            aliased_values[index].as_f64().unwrap(),
            original_values[index].as_f64().unwrap() + 1.0
        );
    }
    assert_eq!(aliased_values[0], original_values[0]);
    assert_eq!(aliased_values[76], original_values[76]);

    assert_equal_limit_resource_boundaries(&aliased);
}

#[test]
fn distinct_closed_limits_have_exact_periodic_proof_resources_and_rollback() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    let end = make_record_1828_limits_distinct(&mut file);
    assert_ne!(end, 2205);
    assert_eq!(field(&file, 1828, "start").as_ptr(), Some(2205));
    assert_eq!(field(&file, 1828, "end").as_ptr(), Some(end));
    for limit in [2205, end] {
        assert_eq!(file.nodes[&limit].code, code::LIMIT);
        assert_eq!(field(&file, limit, "type").as_char(), Some('H'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
        assert_eq!(field(&file, limit, "hvec"), field(&file, 2205, "hvec"));
    }
    assert_equal_limit_resource_boundaries(&file);
}

#[test]
fn material_ambiguous_multi_period_nonperiodic_and_null_interior_aliases_fail_atomically() {
    let mut material = read_xt(EXEMPLAR).unwrap();
    alias_record_1828_interior_period(&mut material, 0.75);
    let mut store = Store::new();
    let error = reconstruct(&material, &mut store).unwrap_err();
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

    let mut ambiguous = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&ambiguous, 2204, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[40] = Value::Double(values[36].as_f64().unwrap() + 0.5);
    set_field(&mut ambiguous, 2204, "values", Value::Arr(values));
    let error = reconstruct(&ambiguous, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "equal-limit periodic trace has an ambiguous period alias",
        }
    ));
    assert_rollback(&store);

    let mut multi_period = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&multi_period, 2204, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[40] = Value::Double(values[40].as_f64().unwrap() + 2.0);
    set_field(&mut multi_period, 2204, "values", Value::Arr(values));
    let error = reconstruct(&multi_period, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "equal-limit periodic trace uses more than one alias period",
        }
    ));
    assert_rollback(&store);

    let mut nonperiodic = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&nonperiodic, 2204, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[41] = Value::Double(values[41].as_f64().unwrap() + 1.0);
    set_field(&mut nonperiodic, 2204, "values", Value::Arr(values));
    let error = reconstruct(&nonperiodic, &mut store).unwrap_err();
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

    let mut null = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&null, 2204, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[40] = Value::Null;
    set_field(&mut null, 2204, "values", Value::Arr(values));
    let error = reconstruct(&null, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::Unsupported {
            capability: XtCapability::IntersectionChartData,
            what: "INTERSECTION_DATA contains null or non-finite UV values",
        }
    ));
    assert_rollback(&store);
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
fn malformed_closed_limits_and_noncanonical_periodic_endpoints_remain_typed_and_atomic() {
    let mut cases = Vec::new();

    let mut null = read_xt(EXEMPLAR).unwrap();
    set_field(&mut null, 1828, "end", Value::Ptr(0));
    cases.push((null, "transmitted intersection has a null limit"));

    let mut mixed_closed = read_xt(EXEMPLAR).unwrap();
    let mixed_end = make_record_1828_limits_distinct(&mut mixed_closed);
    set_field(&mut mixed_closed, mixed_end, "type", Value::Char('L'));
    cases.push((
        mixed_closed,
        "only finite open LIMIT type L with term_use ? is supported",
    ));

    let mut ambiguous_closed = read_xt(EXEMPLAR).unwrap();
    let ambiguous_end = make_record_1828_limits_distinct(&mut ambiguous_closed);
    let position = match field(&ambiguous_closed, ambiguous_end, "hvec") {
        Value::Arr(values) => values[0].clone(),
        _ => unreachable!(),
    };
    set_field(
        &mut ambiguous_closed,
        ambiguous_end,
        "hvec",
        Value::Arr(vec![position.clone(), position]),
    );
    cases.push((
        ambiguous_closed,
        "distinct closed LIMIT must contain exactly one position",
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

    let mut material_closed = read_xt(EXEMPLAR).unwrap();
    set_field(&mut material_closed, 1828, "end", Value::Ptr(2208));
    let error = reconstruct(&material_closed, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::BadField {
            index: 1828,
            what: "closed INTERSECTION LIMIT positions do not identify one point",
        }
    ));
    assert_rollback(&store);
}
