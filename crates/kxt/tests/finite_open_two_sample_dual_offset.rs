//! Canonical finite-open two-sample dual-offset line chart contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy, StageId,
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
const RECORD_3595_WORK: u64 = 4_352_000;
const V12_WORK: u64 = 414_569_575;
const RESIDUALS: [f64; 2] = [3.468_467_250_779_673e-5, 3.384_554_176_162_513e-5];

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn doubles(value: &Value) -> Vec<f64> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_f64().unwrap()).collect(),
        _ => panic!("expected numeric array"),
    }
}

fn transplant_3595(file: &mut kxt::XtFile) {
    for name in ["surface", "chart", "start", "end", "intersection_data"] {
        let value = field(file, 3595, name).clone();
        set_field(file, 1828, name, value);
    }
}

fn context_with_plan<'a>(session: &'a SessionPolicy, plan: BudgetPlan) -> OperationContext<'a> {
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(plan)
}

fn usage(
    report: &kcore::operation::OperationReport,
    stage: StageId,
    resource: ResourceKind,
) -> u64 {
    report
        .usage()
        .iter()
        .find(|entry| entry.stage == stage && entry.resource == resource)
        .map_or(0, |entry| entry.consumed)
}

fn assert_rollback(store: &Store) {
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<CurveGeom>(), 0);
    assert_eq!(store.count::<Curve2dGeom>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);
}

#[test]
fn record_3595_pins_the_canonical_two_sample_dual_offset_payload() {
    let file = read_xt(EXEMPLAR).unwrap();
    assert_eq!(file.nodes[&3595].code, code::INTERSECTION);
    assert_eq!(
        field(&file, 3595, "surface"),
        &Value::Arr(vec![Value::Ptr(783), Value::Ptr(773)])
    );
    assert_eq!(field(&file, 3595, "chart").as_ptr(), Some(3593));
    assert_eq!(field(&file, 3595, "start").as_ptr(), Some(3596));
    assert_eq!(field(&file, 3595, "end").as_ptr(), Some(3600));
    assert_eq!(field(&file, 3595, "intersection_data").as_ptr(), Some(3597));
    for (root, basis, nurbs, distance, sense) in [
        (783, 762, 812, -0.0015, '-'),
        (773, 1186, 1208, 0.00017, '+'),
    ] {
        assert_eq!(file.nodes[&root].code, code::OFFSET_SURF);
        assert_eq!(field(&file, root, "surface").as_ptr(), Some(basis));
        assert_eq!(field(&file, root, "offset").as_f64(), Some(distance));
        assert_eq!(field(&file, root, "sense").as_char(), Some(sense));
        assert_eq!(file.nodes[&basis].code, code::B_SURFACE);
        assert_eq!(field(&file, basis, "nurbs").as_ptr(), Some(nurbs));
    }
    assert_eq!(field(&file, 3593, "chart_count").as_int(), Some(2));
    assert_eq!(field(&file, 3593, "base_parameter").as_f64(), Some(0.0));
    assert_eq!(field(&file, 3593, "base_scale").as_f64(), Some(1.0));
    assert_eq!(
        field(&file, 3593, "chordal_error").as_f64(),
        Some(0.0001817100064117055)
    );
    assert_eq!(field(&file, 3597, "uv_type").as_int(), Some(4));
    assert_eq!(
        doubles(field(&file, 3597, "values")),
        vec![
            0.484670404551706,
            -0.00962819796617773,
            0.670381586686039,
            0.695846998191936,
            0.957765958451465,
            0.03588571470810685,
            0.688494770828634,
            0.745506266602436,
        ]
    );
    assert!(RESIDUALS.into_iter().all(|bound| {
        bound.is_finite() && bound <= field(&file, 3593, "chordal_error").as_f64().unwrap()
    }));
}

#[test]
fn record_3595_transplant_has_exact_isolated_work_items_depth_and_n_minus_one_boundaries() {
    let session = SessionPolicy::v1();
    for (resource, mode, exact) in [
        (
            ResourceKind::Work,
            AccountingMode::Cumulative,
            RECORD_3595_WORK,
        ),
        (ResourceKind::Items, AccountingMode::HighWater, 2),
        (ResourceKind::Depth, AccountingMode::HighWater, 10),
    ] {
        for allowed in [exact - 1, exact] {
            let mut file = read_xt(EXEMPLAR).unwrap();
            transplant_3595(&mut file);
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
                    RECORD_3595_WORK,
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
    let v12 = IntersectionImportBudgetProfile::v12_defaults();
    assert_eq!(
        v12.limits()
            .iter()
            .find(|limit| {
                limit.stage == INTERSECTION_CHART_CERTIFICATE_WORK
                    && limit.resource == ResourceKind::Work
            })
            .unwrap()
            .allowed,
        V12_WORK
    );
}

#[test]
fn malformed_two_sample_witnesses_fail_typed_and_atomically() {
    let mut duplicate_position = read_xt(EXEMPLAR).unwrap();
    transplant_3595(&mut duplicate_position);
    let mut positions = match field(&duplicate_position, 3593, "hvec").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    positions[1] = positions[0].clone();
    let duplicate_endpoint = positions[0].clone();
    set_field(&mut duplicate_position, 3593, "hvec", Value::Arr(positions));
    set_field(
        &mut duplicate_position,
        3600,
        "hvec",
        Value::Arr(vec![duplicate_endpoint]),
    );
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&duplicate_position, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: "dual-offset two-sample carrier endpoints must be distinct",
            },
        })
    ));
    assert_rollback(&store);

    let mut duplicate_uv = read_xt(EXEMPLAR).unwrap();
    transplant_3595(&mut duplicate_uv);
    let mut values = match field(&duplicate_uv, 3597, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[4] = values[0].clone();
    values[5] = values[1].clone();
    set_field(&mut duplicate_uv, 3597, "values", Value::Arr(values));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&duplicate_uv, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::First,
                reason: "dual-offset two-sample pcurve endpoints must be distinct",
            },
        })
    ));
    assert_rollback(&store);

    let mut displaced_uv = read_xt(EXEMPLAR).unwrap();
    transplant_3595(&mut displaced_uv);
    let mut values = match field(&displaced_uv, 3597, "values").clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Double(values[0].as_f64().unwrap() + 0.1);
    set_field(&mut displaced_uv, 3597, "values", Value::Arr(values));
    let mut store = Store::new();
    assert!(matches!(
        reconstruct(&displaced_uv, &mut store),
        Err(XtError::IntersectionCertificate {
            index: 1828,
            source: IntersectionCertificateError::ResidualExceedsTolerance {
                trace: PairedTrace::First,
                ..
            },
        })
    ));
    assert_rollback(&store);
}
