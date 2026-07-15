//! Production finite-open Plane/Offset(B-surface) omitted-data contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::vec::Vec3;
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
const V6_WORK: u64 = 208_228_426;
const V6_ATTEMPTED_WORK: u64 = 221_060_174;
const V7_WORK: u64 = 272_430_166;
const V7_ATTEMPTED_WORK: u64 = 285_283_414;
const NONCANONICAL_5089_WORK: u64 = 139_792_442;
const NONCANONICAL_5089_PRIOR_WORK: u64 = 49_970_212;
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
    assert_eq!(store.count::<Curve2dGeom>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

fn assert_v7_frontier(file: &kxt::XtFile) {
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        file,
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
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        usage(
            outcome.report(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        ),
        V7_WORK
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

fn transplant_5089(file: &mut kxt::XtFile, destination: u32) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, 5089, name).clone();
        set_field(file, destination, name, value);
    }
}

fn make_5089_affine_noncanonical(file: &mut kxt::XtFile, destination: u32) {
    transplant_5089(file, destination);
    set_field(
        file,
        5088,
        "base_parameter",
        Value::Double(NONCANONICAL_BASE_PARAMETER),
    );
    set_field(
        file,
        5088,
        "base_scale",
        Value::Double(NONCANONICAL_BASE_SCALE),
    );
}

#[test]
fn record_5089_pins_one_interior_plane_omission_and_exact_inversion() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&5089].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 5089, "surface"),
        &Value::Arr(vec![Value::Ptr(1985), Value::Ptr(773)])
    );
    assert_eq!(file.nodes[&1985].code, code::PLANE);
    assert_eq!(file.nodes[&773].code, code::OFFSET_SURF);
    assert_eq!(field(&file, 773, "surface").as_ptr(), Some(1186));
    assert_eq!(field(&file, 773, "offset").as_f64(), Some(0.00017));
    assert_eq!(file.nodes[&1186].code, code::B_SURFACE);
    assert_eq!(field(&file, 1186, "nurbs").as_ptr(), Some(1208));

    assert_eq!(field(&file, 5089, "chart").as_ptr(), Some(5088));
    assert_eq!(field(&file, 5089, "start").as_ptr(), Some(5091));
    assert_eq!(field(&file, 5089, "end").as_ptr(), Some(5095));
    assert_eq!(field(&file, 5089, "intersection_data").as_ptr(), Some(5092));
    assert_eq!(field(&file, 5088, "chart_count").as_int(), Some(4));
    for limit in [5091, 5095] {
        assert_eq!(field(&file, limit, "type").as_char(), Some('L'));
        assert_eq!(field(&file, limit, "term_use").as_char(), Some('?'));
    }
    assert_eq!(field(&file, 5092, "uv_type").as_int(), Some(4));

    let origin = field(&file, 1985, "pvec").as_vector().unwrap();
    let normal = field(&file, 1985, "normal").as_vector().unwrap();
    let x_axis = field(&file, 1985, "x_axis").as_vector().unwrap();
    assert_eq!(origin, [-0.05, 0.0774678433099791, 0.03193119147810785]);
    assert_eq!(normal, [0.0, 0.601815023152048, -0.798635510047293]);
    assert_eq!(x_axis, [0.0, 0.798635510047293, 0.601815023152048]);
    let frame = Frame::new(
        Vec3::new(origin[0], origin[1], origin[2]),
        Vec3::new(normal[0], normal[1], normal[2]),
        Vec3::new(x_axis[0], x_axis[1], x_axis[2]),
    )
    .unwrap();

    let positions = match field(&file, 5088, "hvec") {
        Value::Arr(values) if values.len() == 4 => values,
        value => panic!("unexpected record-5089 chart positions: {value:?}"),
    };
    let values = match field(&file, 5092, "values") {
        Value::Arr(values) if values.len() == 16 => values,
        value => panic!("unexpected record-5089 intersection data: {value:?}"),
    };
    for (sample, (position, tuple)) in positions.iter().zip(values.chunks_exact(4)).enumerate() {
        let position = position.as_vector().unwrap();
        let displacement = Vec3::new(position[0], position[1], position[2]) - frame.origin();
        let exact_plane_uv = [displacement.dot(frame.x()), displacement.dot(frame.y())];
        if sample == 2 {
            assert_eq!(&tuple[..2], &[Value::Null, Value::Null]);
            assert!((exact_plane_uv[0] - 0.010237981537791391).abs() <= 2.0e-17);
            assert!((exact_plane_uv[1] - 0.027_211_312_268_295_4).abs() <= 2.0e-17);
        } else {
            assert!((tuple[0].as_f64().unwrap() - exact_plane_uv[0]).abs() <= 2.0e-16);
            assert!((tuple[1].as_f64().unwrap() - exact_plane_uv[1]).abs() <= 2.0e-16);
        }
        assert!(tuple[2].as_f64().unwrap().is_finite());
        assert!(tuple[3].as_f64().unwrap().is_finite());
    }
}

#[test]
fn v7_certifies_5089_and_stops_atomically_at_the_next_chart_proof() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_v7_frontier(&file);
}

#[test]
fn synthetic_endpoint_plane_omission_preserves_v7_accounting_and_whole_carrier_proof() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    let mut values = match field(&file, 5092, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Null;
    values[1] = Value::Null;
    set_field(&mut file, 5092, "values", Value::Arr(values));

    assert_v7_frontier(&file);
}

#[test]
fn v6_is_stable_and_v7_has_exact_work_items_and_depth_n_minus_one_crossings() {
    assert_eq!(
        limit(
            &IntersectionImportBudgetProfile::v6_defaults(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        )
        .allowed,
        V6_WORK
    );
    assert_eq!(
        limit(
            &IntersectionImportBudgetProfile::v7_defaults(),
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
        )
        .allowed,
        V7_WORK
    );

    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let mut store = Store::new();
    let outcome = reconstruct_with_context(
        &file,
        &mut store,
        &context_with_plan(&session, IntersectionImportBudgetProfile::v6_defaults()),
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
            V6_ATTEMPTED_WORK,
            V6_WORK,
        )
    );
    assert_rollback(&store);

    for (stage, resource, mode, exact) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            V7_WORK,
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
fn nurbs_trace_or_half_null_omissions_remain_typed_and_atomic() {
    let mut cases = Vec::new();

    let mut endpoint_trace = read_xt(EXEMPLAR).unwrap();
    transplant_5089(&mut endpoint_trace, 1828);
    let mut values = match field(&endpoint_trace, 5092, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[2] = Value::Null;
    values[3] = Value::Null;
    set_field(&mut endpoint_trace, 5092, "values", Value::Arr(values));
    cases.push(endpoint_trace);

    let mut offset_trace = read_xt(EXEMPLAR).unwrap();
    transplant_5089(&mut offset_trace, 1828);
    let mut values = match field(&offset_trace, 5092, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[10] = Value::Null;
    values[11] = Value::Null;
    set_field(&mut offset_trace, 5092, "values", Value::Arr(values));
    cases.push(offset_trace);

    let mut half_null = read_xt(EXEMPLAR).unwrap();
    transplant_5089(&mut half_null, 1828);
    let mut values = match field(&half_null, 5092, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Null;
    set_field(&mut half_null, 5092, "values", Value::Arr(values));
    cases.push(half_null);

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
fn affine_noncanonical_record_5089_variant_has_exact_resources_and_rollback() {
    let mut file = read_xt(EXEMPLAR).unwrap();
    make_5089_affine_noncanonical(&mut file, 1828);
    assert_eq!(field(&file, 1828, "chart").as_ptr(), Some(5088));
    assert_eq!(field(&file, 5088, "chart_count").as_int(), Some(4));
    assert_eq!(
        field(&file, 5088, "base_parameter").as_f64(),
        Some(NONCANONICAL_BASE_PARAMETER)
    );
    assert_eq!(
        field(&file, 5088, "base_scale").as_f64(),
        Some(NONCANONICAL_BASE_SCALE)
    );
    let session = SessionPolicy::v1();
    for (stage, resource, mode, exact, prior) in [
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            NONCANONICAL_5089_WORK,
            NONCANONICAL_5089_PRIOR_WORK,
        ),
        (
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
            AccountingMode::HighWater,
            4,
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
                &file,
                &mut store,
                &context_with_limit(&session, stage, resource, mode, allowed),
            )
            .unwrap();
            if allowed + 1 == exact {
                let crossing = outcome.result().as_ref().unwrap_err().limit().unwrap();
                assert_eq!(
                    (
                        crossing.stage,
                        crossing.resource,
                        crossing.consumed,
                        crossing.allowed,
                    ),
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
fn corpus_noncanonical_candidates_remain_original_domain_typed_and_atomic() {
    for (intersection, trace) in [(778, PairedTrace::Second), (3620, PairedTrace::First)] {
        let mut file = read_xt(EXEMPLAR).unwrap();
        for name in ["surface", "chart", "start", "end", "intersection_data"] {
            let value = field(&file, intersection, name).clone();
            set_field(&mut file, 1828, name, value);
        }
        let mut store = Store::new();
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert!(matches!(
            error,
            XtError::IntersectionCertificate {
                index: 1828,
                source: IntersectionCertificateError::UnsupportedTraceParameterization {
                    trace: actual,
                    reason: "transmitted NURBS pcurve leaves the source surface domain",
                },
            } if actual == trace
        ));
        assert_rollback(&store);
    }
}

#[test]
fn invalid_or_out_of_family_affine_conventions_remain_typed_and_atomic() {
    let mut zero_scale = read_xt(EXEMPLAR).unwrap();
    make_5089_affine_noncanonical(&mut zero_scale, 1828);
    set_field(&mut zero_scale, 5088, "base_scale", Value::Double(0.0));

    let mut collapsed_samples = read_xt(EXEMPLAR).unwrap();
    make_5089_affine_noncanonical(&mut collapsed_samples, 1828);
    set_field(
        &mut collapsed_samples,
        5088,
        "base_parameter",
        Value::Double(f64::MAX),
    );
    set_field(
        &mut collapsed_samples,
        5088,
        "base_scale",
        Value::Double(1.0),
    );

    let mut out_of_family = read_xt(EXEMPLAR).unwrap();
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(&out_of_family, 1252, name).clone();
        set_field(&mut out_of_family, 1828, name, value);
    }
    set_field(
        &mut out_of_family,
        2234,
        "base_parameter",
        Value::Double(NONCANONICAL_BASE_PARAMETER),
    );
    set_field(
        &mut out_of_family,
        2234,
        "base_scale",
        Value::Double(NONCANONICAL_BASE_SCALE),
    );

    for (file, what) in [
        (
            zero_scale,
            "INTERSECTION CHART base_parameter/base_scale must be finite with positive scale",
        ),
        (
            collapsed_samples,
            "INTERSECTION CHART affine sample parameters must be finite and strictly increasing",
        ),
        (
            out_of_family,
            "noncanonical affine charts require the bounded finite-open two- through five-sample direct-Plane/B-surface, safe-Offset(Plane)/B-surface, direct-Plane/Offset(B-surface), direct-Offset(B-surface)/B-surface, independent direct-Offset(B-surface)/Offset(B-surface), or direct-B-surface/B-surface family",
        ),
    ] {
        let mut store = Store::new();
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert!(matches!(
            error,
            XtError::Unsupported {
                capability: XtCapability::IntersectionChartConvention,
                what: actual,
            } if actual == what
        ));
        assert_rollback(&store);
    }
}

#[test]
fn recovered_interior_and_endpoint_plane_uv_still_require_the_whole_carrier_certificate() {
    for sample in [2, 0] {
        let mut file = read_xt(EXEMPLAR).unwrap();
        transplant_5089(&mut file, 1828);
        let mut positions = match field(&file, 5088, "hvec").clone() {
            Value::Arr(values) => values,
            _ => unreachable!(),
        };
        let mut displaced = positions[sample].as_vector().unwrap();
        displaced[1] += 0.601815023152048 * 1.0e-3;
        displaced[2] -= 0.798635510047293 * 1.0e-3;
        positions[sample] = Value::Vector(Some(displaced));
        set_field(&mut file, 5088, "hvec", Value::Arr(positions));
        if sample == 0 {
            set_field(
                &mut file,
                5091,
                "hvec",
                Value::Arr(vec![Value::Vector(Some(displaced))]),
            );
            let mut values = match field(&file, 5092, "values").clone() {
                Value::Arr(values) => values,
                _ => unreachable!(),
            };
            values[0] = Value::Null;
            values[1] = Value::Null;
            set_field(&mut file, 5092, "values", Value::Arr(values));
        }

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
}
