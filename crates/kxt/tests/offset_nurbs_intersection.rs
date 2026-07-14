//! Production Offset/B-surface transmitted-intersection contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use ktopo::entity::Body;
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;
use kxt::parse::{Value, read_xt};
use kxt::schema::code;
use kxt::{
    INTERSECTION_CHART_CERTIFICATE_WORK, IntersectionImportBudgetProfile, XtError, reconstruct,
    reconstruct_with_context,
};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const BLOCK: &[u8] = include_bytes!("fixtures/block.x_t");
const EXEMPLAR_OFFSET_PROOF_WORK: u64 = 81_267_732;
const EXEMPLAR_EQUAL_LIMIT_WORK: u64 = 115_485_725;

#[derive(Debug, Clone, Copy, PartialEq)]
struct OffsetIntersectionIdentity {
    intersection: u32,
    sources: [u32; 2],
    chart: u32,
    data: u32,
    offset: u32,
    basis: u32,
    nurbs: u32,
    distance: f64,
}

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn pointers(value: &Value) -> Vec<u32> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_ptr().unwrap()).collect(),
        _ => panic!("expected pointer array"),
    }
}

fn doubles(value: &Value) -> Vec<f64> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_f64().unwrap()).collect(),
        _ => panic!("expected numeric array"),
    }
}

fn vectors(value: &Value) -> Vec<[f64; 3]> {
    match value {
        Value::Arr(values) => values
            .iter()
            .map(|value| value.as_vector().unwrap())
            .collect(),
        _ => panic!("expected vector array"),
    }
}

fn limit(plan: &BudgetPlan, stage: kcore::operation::StageId, resource: ResourceKind) -> LimitSpec {
    *plan
        .limits()
        .iter()
        .find(|limit| limit.stage == stage && limit.resource == resource)
        .unwrap()
}

fn context_with_plan<'a>(session: &'a SessionPolicy, plan: BudgetPlan) -> OperationContext<'a> {
    OperationContext::new(session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(plan)
}

fn assert_later_intersection_limit(error: &XtError) {
    let limit = error.limit().expect("v3 must stop at terminated proof");
    assert_eq!(
        (limit.stage, limit.resource, limit.consumed, limit.allowed),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            116_396_069,
            EXEMPLAR_EQUAL_LIMIT_WORK,
        ),
        "unexpected exemplar boundary: {error:?}"
    );
}

#[test]
fn exemplar_pins_offset_root_basis_chart_and_seam_safe_proof_rectangles() {
    let file = read_xt(EXEMPLAR).unwrap();
    let expected = [
        OffsetIntersectionIdentity {
            intersection: 1655,
            sources: [1531, 1510],
            chart: 1659,
            data: 1658,
            offset: 1531,
            basis: 1936,
            nurbs: 4918,
            distance: 0.0015,
        },
        OffsetIntersectionIdentity {
            intersection: 1656,
            sources: [1531, 1510],
            chart: 1662,
            data: 1668,
            offset: 1531,
            basis: 1936,
            nurbs: 4918,
            distance: 0.0015,
        },
        OffsetIntersectionIdentity {
            intersection: 1970,
            sources: [1939, 773],
            chart: 5131,
            data: 5136,
            offset: 773,
            basis: 1186,
            nurbs: 1208,
            distance: 0.00017,
        },
        OffsetIntersectionIdentity {
            intersection: 1984,
            sources: [1939, 773],
            chart: 5059,
            data: 5064,
            offset: 773,
            basis: 1186,
            nurbs: 1208,
            distance: 0.00017,
        },
        OffsetIntersectionIdentity {
            intersection: 2077,
            sources: [1939, 1531],
            chart: 2085,
            data: 2088,
            offset: 1531,
            basis: 1936,
            nurbs: 4918,
            distance: 0.0015,
        },
        OffsetIntersectionIdentity {
            intersection: 2078,
            sources: [1939, 1531],
            chart: 2090,
            data: 2091,
            offset: 1531,
            basis: 1936,
            nurbs: 4918,
            distance: 0.0015,
        },
        OffsetIntersectionIdentity {
            intersection: 2627,
            sources: [1531, 1480],
            chart: 2629,
            data: 2632,
            offset: 1531,
            basis: 1936,
            nurbs: 4918,
            distance: 0.0015,
        },
        OffsetIntersectionIdentity {
            intersection: 4752,
            sources: [1938, 773],
            chart: 4760,
            data: 4757,
            offset: 773,
            basis: 1186,
            nurbs: 1208,
            distance: 0.00017,
        },
        OffsetIntersectionIdentity {
            intersection: 4822,
            sources: [1531, 1480],
            chart: 4830,
            data: 4828,
            offset: 1531,
            basis: 1936,
            nurbs: 4918,
            distance: 0.0015,
        },
        OffsetIntersectionIdentity {
            intersection: 5055,
            sources: [1938, 773],
            chart: 5054,
            data: 5057,
            offset: 773,
            basis: 1186,
            nurbs: 1208,
            distance: 0.00017,
        },
    ];

    for identity in expected {
        assert_eq!(file.nodes[&identity.intersection].code, code::INTERSECTION);
        assert_eq!(
            pointers(field(&file, identity.intersection, "surface")),
            identity.sources
        );
        assert_eq!(
            field(&file, identity.intersection, "chart").as_ptr(),
            Some(identity.chart)
        );
        assert_eq!(
            field(&file, identity.intersection, "intersection_data").as_ptr(),
            Some(identity.data)
        );
        assert_eq!(file.nodes[&identity.offset].code, code::OFFSET_SURF);
        assert_eq!(
            field(&file, identity.offset, "surface").as_ptr(),
            Some(identity.basis)
        );
        assert_eq!(
            field(&file, identity.offset, "offset").as_f64(),
            Some(identity.distance)
        );
        assert_eq!(file.nodes[&identity.basis].code, code::B_SURFACE);
        assert_eq!(
            field(&file, identity.basis, "nurbs").as_ptr(),
            Some(identity.nurbs)
        );
        assert_eq!(
            field(&file, identity.chart, "chart_count").as_int(),
            Some(4)
        );
        assert_eq!(
            (
                field(&file, identity.chart, "base_parameter").as_f64(),
                field(&file, identity.chart, "base_scale").as_f64(),
            ),
            (Some(0.0), Some(1.0))
        );
        assert_eq!(field(&file, identity.data, "uv_type").as_int(), Some(4));

        let nurbs = identity.nurbs;
        assert_eq!(
            (
                field(&file, nurbs, "u_periodic"),
                field(&file, nurbs, "v_periodic"),
                field(&file, nurbs, "u_closed"),
                field(&file, nurbs, "v_closed"),
            ),
            (
                &Value::Logical(true),
                &Value::Logical(false),
                &Value::Logical(true),
                &Value::Logical(false),
            )
        );
        assert_eq!(
            (
                field(&file, nurbs, "u_degree").as_int(),
                field(&file, nurbs, "v_degree").as_int(),
                field(&file, nurbs, "n_u_vertices").as_int(),
                field(&file, nurbs, "n_v_vertices").as_int(),
            ),
            (Some(3), Some(3), Some(90), Some(11))
        );

        // Offset proof boxes are strictly inside the certified periodic base
        // domain. No proof rectangle crosses or wraps the u seam.
        let offset_operand = usize::from(identity.sources[1] == identity.offset);
        let uv = doubles(field(&file, identity.data, "values"));
        assert_eq!(uv.len(), 16);
        for sample in uv.chunks_exact(4) {
            let pair = &sample[offset_operand * 2..offset_operand * 2 + 2];
            assert!((0.0..=1.0).contains(&pair[0]));
            assert!((-0.01..=1.0).contains(&pair[1]));
        }
    }

    let chart_positions = vectors(field(&file, 1659, "hvec"));
    assert_eq!(chart_positions.len(), 4);
    let affine_midpoint = [
        chart_positions[0][0] + (chart_positions[3][0] - chart_positions[0][0]) / 3.0,
        chart_positions[0][1] + (chart_positions[3][1] - chart_positions[0][1]) / 3.0,
        chart_positions[0][2] + (chart_positions[3][2] - chart_positions[0][2]) / 3.0,
    ];
    assert_ne!(chart_positions[1], affine_midpoint);
    assert_eq!(
        doubles(field(&file, 1658, "values")),
        vec![
            0.757919065959854,
            0.883612153144728,
            0.0,
            0.0744382876272054,
            0.739215942796182,
            0.869339721876521,
            0.3581508863340505,
            0.0746014986883595,
            0.726418702313645,
            0.849110635703743,
            0.760638253261242,
            0.0744402900485097,
            0.722096023929452,
            0.835014993018288,
            1.0,
            0.0743174432009644,
        ]
    );
}

#[test]
fn historical_profiles_retain_their_caps_and_v3_advances_with_exact_rollback() {
    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let v1 = IntersectionImportBudgetProfile::v1_defaults();
    let v2 = IntersectionImportBudgetProfile::v2_defaults();
    let v3 = IntersectionImportBudgetProfile::v3_defaults();
    assert_eq!(
        limit(&v1, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work),
        LimitSpec::new(
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            131_072,
        )
    );
    assert_eq!(
        limit(&v2, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        EXEMPLAR_OFFSET_PROOF_WORK
    );
    assert_eq!(
        limit(&v3, INTERSECTION_CHART_CERTIFICATE_WORK, ResourceKind::Work).allowed,
        EXEMPLAR_EQUAL_LIMIT_WORK
    );

    let mut store = Store::new();
    let outcome =
        reconstruct_with_context(&file, &mut store, &context_with_plan(&session, v1)).unwrap();
    let limit = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(
        (limit.stage, limit.consumed, limit.allowed),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            EXEMPLAR_OFFSET_PROOF_WORK,
            131_072,
        )
    );
    assert_eq!(
        (store.count::<Body>(), store.count::<SurfaceGeom>()),
        (0, 0)
    );

    let outcome =
        reconstruct_with_context(&file, &mut store, &context_with_plan(&session, v2)).unwrap();
    let limit = outcome.result().as_ref().unwrap_err().limit().unwrap();
    assert_eq!(
        (limit.stage, limit.consumed, limit.allowed),
        (
            INTERSECTION_CHART_CERTIFICATE_WORK,
            EXEMPLAR_EQUAL_LIMIT_WORK,
            EXEMPLAR_OFFSET_PROOF_WORK,
        )
    );
    assert_eq!(
        (store.count::<Body>(), store.count::<SurfaceGeom>()),
        (0, 0)
    );

    let outcome =
        reconstruct_with_context(&file, &mut store, &context_with_plan(&session, v3)).unwrap();
    assert_later_intersection_limit(outcome.result().as_ref().unwrap_err());
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        (store.count::<Body>(), store.count::<SurfaceGeom>()),
        (0, 0)
    );

    let block = read_xt(BLOCK).unwrap();
    assert_eq!(reconstruct(&block, &mut store).unwrap().bodies.len(), 1);
}
