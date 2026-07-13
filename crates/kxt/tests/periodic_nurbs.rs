//! Production periodic/closed B-geometry reconstruction contract.

use kcore::operation::{OperationContext, SessionPolicy};
use kcore::tolerance::Tolerances;
use ktopo::entity::Body;
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;
use kxt::parse::{Value, read_xt};
use kxt::schema::code;
use kxt::{XtCapability, XtError, reconstruct, reconstruct_with_context};

const EXEMPLAR: &[u8] = include_bytes!("fixtures/exemplar.x_t");
const BLOCK: &[u8] = include_bytes!("fixtures/block.x_t");

fn field<'a>(file: &'a kxt::XtFile, index: u32, name: &str) -> &'a Value {
    file.field(&file.nodes[&index], name).unwrap()
}

fn logical(value: &Value) -> bool {
    match value {
        Value::Logical(value) => *value,
        _ => panic!("expected logical value"),
    }
}

fn array_len(value: &Value) -> usize {
    match value {
        Value::Arr(values) => values.len(),
        _ => panic!("expected array value"),
    }
}

fn doubles(value: &Value) -> Vec<f64> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_f64().unwrap()).collect(),
        _ => panic!("expected numeric array value"),
    }
}

fn integers(value: &Value) -> Vec<i64> {
    match value {
        Value::Arr(values) => values.iter().map(|value| value.as_int().unwrap()).collect(),
        _ => panic!("expected integer array value"),
    }
}

fn set_field(file: &mut kxt::XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let position = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[position] = value;
}

fn periodic_surfaces(file: &kxt::XtFile) -> Vec<(u32, u32)> {
    file.nodes
        .iter()
        .filter_map(|(&surface, node)| {
            if node.code != code::B_SURFACE {
                return None;
            }
            let nurbs = field(file, surface, "nurbs").as_ptr().unwrap();
            logical(field(file, nurbs, "u_periodic")).then_some((surface, nurbs))
        })
        .collect()
}

#[test]
fn exemplar_periodic_surfaces_pin_the_certified_clamped_seam_contract() {
    let file = read_xt(EXEMPLAR).unwrap();
    let mut periodic_curves = 0;
    let mut closed_curves = 0;
    for (&curve, node) in &file.nodes {
        if node.code != code::B_CURVE {
            continue;
        }
        let nurbs = field(&file, curve, "nurbs").as_ptr().unwrap();
        periodic_curves += usize::from(logical(field(&file, nurbs, "periodic")));
        closed_curves += usize::from(logical(field(&file, nurbs, "closed")));
    }
    assert_eq!((periodic_curves, closed_curves), (0, 0));
    let mut periodic_surfaces = Vec::new();
    for (&surface, node) in &file.nodes {
        if node.code != code::B_SURFACE {
            continue;
        }
        let nurbs = field(&file, surface, "nurbs").as_ptr().unwrap();
        let periodic = [
            logical(field(&file, nurbs, "u_periodic")),
            logical(field(&file, nurbs, "v_periodic")),
        ];
        let closed = [
            logical(field(&file, nurbs, "u_closed")),
            logical(field(&file, nurbs, "v_closed")),
        ];
        if periodic != [false, false] || closed != [false, false] {
            assert_eq!(periodic, [true, false]);
            assert_eq!(closed, periodic);
            assert_eq!(field(&file, nurbs, "u_degree").as_int(), Some(3));
            assert_eq!(field(&file, nurbs, "v_degree").as_int(), Some(3));
            assert_eq!(field(&file, nurbs, "n_u_vertices").as_int(), Some(90));
            assert_eq!(field(&file, nurbs, "n_v_vertices").as_int(), Some(11));
            assert!(!logical(field(&file, nurbs, "rational")));
            assert_eq!(field(&file, nurbs, "vertex_dim").as_int(), Some(3));
            let u_knots = field(&file, nurbs, "u_knots").as_ptr().unwrap();
            let v_knots = field(&file, nurbs, "v_knots").as_ptr().unwrap();
            let u_mult = field(&file, nurbs, "u_knot_mult").as_ptr().unwrap();
            let v_mult = field(&file, nurbs, "v_knot_mult").as_ptr().unwrap();
            let poles = field(&file, nurbs, "bspline_vertices").as_ptr().unwrap();
            assert_eq!(array_len(field(&file, poles, "vertices")), 2_970);
            let u_distinct = doubles(field(&file, u_knots, "knots"));
            let u_multiplicities = integers(field(&file, u_mult, "mult"));
            assert_eq!(u_distinct.len(), 45);
            assert_eq!((u_distinct[0], *u_distinct.last().unwrap()), (0.0, 1.0));
            assert!(u_distinct.windows(2).all(|pair| pair[0] < pair[1]));
            assert_eq!(
                (u_multiplicities[0], *u_multiplicities.last().unwrap()),
                (4, 4)
            );
            assert!(u_multiplicities[1..44].iter().all(|&value| value == 2));
            assert_eq!(
                doubles(field(&file, v_knots, "knots")),
                vec![-0.01, 0.0, 0.5625, 0.59375, 1.0]
            );
            assert_eq!(integers(field(&file, v_mult, "mult")), vec![4, 3, 2, 2, 4]);

            let values = doubles(field(&file, poles, "vertices"));
            let nv = field(&file, nurbs, "n_v_vertices").as_int().unwrap() as usize;
            let row_width = nv * 3;
            let seam_position_delta = values[..row_width]
                .iter()
                .zip(&values[89 * row_width..90 * row_width])
                .map(|(first, last)| (first - last).abs())
                .fold(0.0_f64, f64::max);
            assert!(seam_position_delta <= 2.0e-18);
            let expanded_u = {
                u_distinct
                    .into_iter()
                    .zip(u_multiplicities)
                    .flat_map(|(knot, count)| core::iter::repeat_n(knot, count as usize))
                    .collect::<Vec<_>>()
            };
            assert_eq!(expanded_u.len(), 94);
            let mut max_du_delta = 0.0_f64;
            for offset in 0..row_width {
                let start = 3.0 * (values[row_width + offset] - values[offset])
                    / (expanded_u[4] - expanded_u[3]);
                let end = 3.0 * (values[89 * row_width + offset] - values[88 * row_width + offset])
                    / (expanded_u[90] - expanded_u[89]);
                max_du_delta = max_du_delta.max((start - end).abs());
            }
            assert!(max_du_delta <= 2.0e-15);
            periodic_surfaces.push((surface, nurbs));
        }
    }
    assert_eq!(
        periodic_surfaces,
        vec![(1186, 1208), (1204, 1207), (1936, 4918)]
    );
}

#[test]
fn exemplar_offset_nurbs_proof_advances_to_intersection_limits_and_rolls_back() {
    let file = read_xt(EXEMPLAR).unwrap();
    let mut store = Store::new();
    let error = reconstruct(&file, &mut store).unwrap_err();
    assert!(
        matches!(
            error,
            XtError::Unsupported {
                capability: XtCapability::IntersectionLimits,
                what: "transmitted intersection is not finite and open with distinct limits",
            }
        ),
        "advanced exemplar error: {error:?}"
    );
    assert_eq!(store.count::<Body>(), 0);
    assert_eq!(store.count::<SurfaceGeom>(), 0);

    let block = read_xt(BLOCK).unwrap();
    assert_eq!(reconstruct(&block, &mut store).unwrap().bodies.len(), 1);
}

#[test]
fn periodic_flag_mismatches_and_unclamped_basis_remain_typed() {
    let original = read_xt(EXEMPLAR).unwrap();
    let periodic = periodic_surfaces(&original);
    for (field_name, value) in [
        ("u_closed", Value::Logical(false)),
        ("u_periodic", Value::Logical(false)),
    ] {
        let mut file = read_xt(EXEMPLAR).unwrap();
        for &(_, nurbs) in &periodic {
            set_field(&mut file, nurbs, field_name, value.clone());
        }
        let mut store = Store::new();
        let error = reconstruct(&file, &mut store).unwrap_err();
        assert_eq!(
            error.capability(),
            Some(XtCapability::PeriodicNurbsSurfaces)
        );
        assert_eq!(store.count::<Body>(), 0);
    }

    let mut unclamped = read_xt(EXEMPLAR).unwrap();
    for &(_, nurbs) in &periodic {
        let mult = field(&unclamped, nurbs, "u_knot_mult").as_ptr().unwrap();
        let mut values = integers(field(&unclamped, mult, "mult"));
        values[0] -= 1;
        values[1] += 1;
        set_field(
            &mut unclamped,
            mult,
            "mult",
            Value::Arr(values.into_iter().map(Value::Int).collect()),
        );
    }
    let mut store = Store::new();
    let error = reconstruct(&unclamped, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::PeriodicNurbsSurfaces)
    );
    assert_eq!(store.count::<Body>(), 0);
}

#[test]
fn malformed_periodic_seam_is_bad_data_and_accounting_is_deterministic() {
    let mut malformed = read_xt(EXEMPLAR).unwrap();
    let periodic = periodic_surfaces(&malformed);
    for &(_, nurbs) in &periodic {
        let poles = field(&malformed, nurbs, "bspline_vertices")
            .as_ptr()
            .unwrap();
        let mut values = doubles(field(&malformed, poles, "vertices"));
        let last_row = 89 * 11 * 3;
        values[last_row] += 1.0e-4;
        set_field(
            &mut malformed,
            poles,
            "vertices",
            Value::Arr(values.into_iter().map(Value::Double).collect()),
        );
    }
    let mut store = Store::new();
    let error = reconstruct(&malformed, &mut store).unwrap_err();
    assert!(
        matches!(error, XtError::BadField { index, .. } if periodic.iter().any(|&(surface, _)| surface == index))
    );
    assert_eq!(store.count::<Body>(), 0);

    let file = read_xt(EXEMPLAR).unwrap();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let mut reports = Vec::new();
    for _ in 0..2 {
        let mut store = Store::new();
        let outcome = reconstruct_with_context(&file, &mut store, &context).unwrap();
        assert_eq!(
            outcome.result().as_ref().unwrap_err().capability(),
            Some(XtCapability::IntersectionLimits)
        );
        assert!(outcome.report().limit_events().is_empty());
        assert_eq!(store.count::<Body>(), 0);
        reports.push(outcome.report().clone());
    }
    assert_eq!(reports[0].usage(), reports[1].usage());
}
