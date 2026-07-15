//! Verified finite open plane/plane transmitted-intersection import contract.

use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use kgraph::{CurveClass, GeometryRef, SurfaceClass};
use ktopo::entity::{Body, Edge};
use ktopo::make;
use ktopo::store::Store;
use kxt::parse::{Node, Value, XtFile};
use kxt::schema::{FieldSpec, FieldType, NodeDef, code};
use kxt::{
    INTERSECTION_CHART_CERTIFICATE_WORK, INTERSECTION_CHART_DEPTH, INTERSECTION_CHART_ITEMS,
    XtCapability, XtError, export_text, read_xt, reconstruct, reconstruct_with_context,
};

fn ptr(file: &XtFile, index: u32, name: &str) -> u32 {
    file.field(&file.nodes[&index], name)
        .and_then(Value::as_ptr)
        .unwrap()
}

fn vector(file: &XtFile, index: u32, name: &str) -> Vec3 {
    let value = file
        .field(&file.nodes[&index], name)
        .and_then(Value::as_vector)
        .unwrap();
    Vec3::new(value[0], value[1], value[2])
}

fn set_field(file: &mut XtFile, index: u32, name: &str, value: Value) {
    let code = file.nodes[&index].code;
    let field = file.defs[&code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[field] = value;
}

fn add_field(def: &mut NodeDef, name: &str, ty: FieldType, n_elts: u32) {
    def.fields.push(FieldSpec {
        name: name.to_owned(),
        ty,
        n_elts,
    });
}

fn point_of_vertex(file: &XtFile, vertex: u32) -> Point3 {
    let point = ptr(file, vertex, "point");
    vector(file, point, "pvec")
}

fn plane_frame(file: &XtFile, surface: u32) -> Frame {
    Frame::new(
        vector(file, surface, "pvec"),
        vector(file, surface, "normal"),
        vector(file, surface, "x_axis"),
    )
    .unwrap()
}

/// Writer-produced topology with one exact line replaced structurally by the
/// modern trimmed INTERSECTION -> CHART/LIMIT/INTERSECTION_DATA transport arm.
fn affine_plane_intersection_file() -> XtFile {
    let mut source = Store::new();
    let body = make::block(&mut source, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let text = export_text(&source, body).unwrap();
    let mut file = read_xt(text.as_bytes()).unwrap();

    let edge = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::EDGE).then_some(index))
        .unwrap();
    let trim = ptr(&file, edge, "curve");
    assert_eq!(file.nodes[&trim].code, code::LINE);
    assert_eq!(
        file.field(&file.nodes[&trim], "sense"),
        Some(&Value::Char('+'))
    );

    let real_fins = file
        .nodes
        .iter()
        .filter_map(|(&index, node)| {
            (node.code == code::FIN
                && file.field(node, "edge").and_then(Value::as_ptr) == Some(edge)
                && file.field(node, "loop").and_then(Value::as_ptr) != Some(0))
            .then_some(index)
        })
        .collect::<Vec<_>>();
    assert_eq!(real_fins.len(), 2);

    let mut start = None;
    let mut end = None;
    let mut surfaces = Vec::new();
    for fin in real_fins {
        let vertex = ptr(&file, fin, "vertex");
        match file.field(&file.nodes[&fin], "sense") {
            Some(Value::Char('+')) => end = Some(point_of_vertex(&file, vertex)),
            Some(Value::Char('-')) => start = Some(point_of_vertex(&file, vertex)),
            _ => unreachable!(),
        }
        let lp = ptr(&file, fin, "loop");
        let face = ptr(&file, lp, "face");
        surfaces.push(ptr(&file, face, "surface"));
    }
    let start = start.unwrap();
    let end = end.unwrap();
    let step = end - start;
    assert_eq!(step.norm(), 1.0);

    let oriented_normal = |surface| {
        let frame = plane_frame(&file, surface);
        match file.field(&file.nodes[&surface], "sense") {
            Some(Value::Char('+')) => frame.z(),
            Some(Value::Char('-')) => -frame.z(),
            _ => unreachable!(),
        }
    };
    if oriented_normal(surfaces[0])
        .cross(oriented_normal(surfaces[1]))
        .normalized()
        .unwrap()
        != step
    {
        surfaces.swap(0, 1);
    }
    assert_eq!(
        oriented_normal(surfaces[0])
            .cross(oriented_normal(surfaces[1]))
            .normalized()
            .unwrap(),
        step
    );

    let uv = |surface, point: Point3| {
        let local = plane_frame(&file, surface).to_local(point);
        [local.x, local.y]
    };
    let uv00 = uv(surfaces[0], start);
    let uv10 = uv(surfaces[1], start);
    let uv01 = uv(surfaces[0], end);
    let uv11 = uv(surfaces[1], end);

    let first_new = file.nodes.keys().next_back().copied().unwrap() + 1;
    let intersection = first_new;
    let chart = first_new + 1;
    let start_limit = first_new + 2;
    let end_limit = first_new + 3;
    let data = first_new + 4;

    let base_defs = kxt::schema::base_schema();
    let base_def = |node_code| {
        base_defs
            .iter()
            .find(|definition| definition.code == node_code)
            .unwrap()
            .clone()
    };
    let mut intersection_def = base_def(code::INTERSECTION);
    add_field(
        &mut intersection_def,
        "intersection_data",
        FieldType::Ptr,
        0,
    );
    file.defs.insert(code::INTERSECTION, intersection_def);
    file.defs.insert(code::CHART, base_def(code::CHART));
    file.defs
        .insert(code::TRIMMED_CURVE, base_def(code::TRIMMED_CURVE));
    let mut limit_def = base_def(code::LIMIT);
    limit_def.fields.insert(
        1,
        FieldSpec {
            name: "term_use".to_owned(),
            ty: FieldType::Char,
            n_elts: 0,
        },
    );
    file.defs.insert(code::LIMIT, limit_def);
    file.defs.insert(
        code::INTERSECTION_DATA,
        NodeDef {
            code: code::INTERSECTION_DATA,
            name: "INTERSECTION_DATA".to_owned(),
            fields: vec![
                FieldSpec {
                    name: "uv_type".to_owned(),
                    ty: FieldType::Byte,
                    n_elts: 0,
                },
                FieldSpec {
                    name: "values".to_owned(),
                    ty: FieldType::Double,
                    n_elts: 1,
                },
            ],
        },
    );
    file.foreign_codes.push(code::INTERSECTION_DATA);

    let mut common = file.nodes[&trim].values[..7].to_vec();
    common[0] = Value::Int(i64::from(intersection));
    let mut intersection_values = common;
    intersection_values.extend([
        Value::Arr(vec![Value::Ptr(surfaces[0]), Value::Ptr(surfaces[1])]),
        Value::Ptr(chart),
        Value::Ptr(start_limit),
        Value::Ptr(end_limit),
        Value::Ptr(data),
    ]);
    file.nodes.insert(
        intersection,
        Node {
            code: code::INTERSECTION,
            values: intersection_values,
        },
    );
    file.nodes.insert(
        chart,
        Node {
            code: code::CHART,
            values: vec![
                Value::Double(0.0),
                Value::Double(1.0),
                Value::Int(2),
                Value::Double(0.0),
                Value::Double(0.0),
                Value::Arr(vec![Value::Null, Value::Null]),
                Value::Arr(vec![
                    Value::Vector(Some(start.to_array())),
                    Value::Vector(Some(end.to_array())),
                ]),
            ],
        },
    );
    for (index, point) in [(start_limit, start), (end_limit, end)] {
        file.nodes.insert(
            index,
            Node {
                code: code::LIMIT,
                values: vec![
                    Value::Char('L'),
                    Value::Char('?'),
                    Value::Arr(vec![Value::Vector(Some(point.to_array()))]),
                ],
            },
        );
    }
    file.nodes.insert(
        data,
        Node {
            code: code::INTERSECTION_DATA,
            values: vec![
                Value::Int(4),
                Value::Arr(
                    [uv00, uv10, uv01, uv11]
                        .into_iter()
                        .flatten()
                        .map(Value::Double)
                        .collect(),
                ),
            ],
        },
    );

    let mut trim_values = file.nodes[&trim].values[..7].to_vec();
    trim_values.extend([
        Value::Ptr(intersection),
        Value::Vector(Some(start.to_array())),
        Value::Vector(Some(end.to_array())),
        Value::Double(0.0),
        Value::Double(1.0),
    ]);
    file.nodes.insert(
        trim,
        Node {
            code: code::TRIMMED_CURVE,
            values: trim_values,
        },
    );
    file
}

fn plane_offset_intersection_file(nested: bool, swapped: bool) -> XtFile {
    let mut file = affine_plane_intersection_file();
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let replaced_plane = sources[1].as_ptr().unwrap();
    let plane = file.nodes[&replaced_plane].clone();
    let frame = plane_frame(&file, replaced_plane);
    let sense = file
        .field(&plane, "sense")
        .and_then(Value::as_char)
        .unwrap();
    let signed_total = if sense == '+' { 0.25 } else { -0.25 };
    let basis_origin = frame.origin() - frame.z() * signed_total;

    let mut next = file.nodes.keys().next_back().copied().unwrap() + 1;
    let basis = next;
    next += 1;
    file.nodes.insert(
        basis,
        Node {
            code: code::PLANE,
            values: plane.values,
        },
    );
    set_field(&mut file, basis, "node_id", Value::Int(i64::from(basis)));
    for name in [
        "attributes_groups",
        "owner",
        "next",
        "previous",
        "geometric_owner",
    ] {
        set_field(&mut file, basis, name, Value::Ptr(0));
    }
    set_field(
        &mut file,
        basis,
        "pvec",
        Value::Vector(Some(basis_origin.to_array())),
    );

    let offset_def = kxt::schema::base_schema()
        .into_iter()
        .find(|definition| definition.code == code::OFFSET_SURF)
        .unwrap();
    file.defs.insert(code::OFFSET_SURF, offset_def);
    let offset_values = |index: u32, basis: u32, amount: f64| {
        vec![
            Value::Int(i64::from(index)),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Char(sense),
            Value::Char('U'),
            Value::Logical(false),
            Value::Ptr(basis),
            Value::Double(amount),
            Value::Null,
        ]
    };
    let source = if nested {
        let inner = next;
        next += 1;
        file.nodes.insert(
            inner,
            Node {
                code: code::OFFSET_SURF,
                values: offset_values(inner, basis, 0.125),
            },
        );
        let outer = next;
        file.nodes.insert(
            outer,
            Node {
                code: code::OFFSET_SURF,
                values: offset_values(outer, inner, 0.125),
            },
        );
        outer
    } else {
        let offset = next;
        file.nodes.insert(
            offset,
            Node {
                code: code::OFFSET_SURF,
                values: offset_values(offset, basis, 0.25),
            },
        );
        offset
    };
    sources[1] = Value::Ptr(source);
    set_field(&mut file, intersection, "surface", Value::Arr(sources));
    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

fn plane_bsurface_intersection_file(rational: bool, swapped: bool) -> XtFile {
    let mut file = affine_plane_intersection_file();
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let replaced_plane = sources[1].as_ptr().unwrap();
    let remaining_plane = sources[0].as_ptr().unwrap();
    let plane = file.nodes[&replaced_plane].clone();
    let chart = ptr(&file, intersection, "chart");
    let positions = match file.field(&file.nodes[&chart], "hvec").unwrap() {
        Value::Arr(values) => values
            .iter()
            .map(|value| {
                let value = value.as_vector().unwrap();
                Point3::new(value[0], value[1], value[2])
            })
            .collect::<Vec<_>>(),
        _ => unreachable!(),
    };
    let normal = plane_frame(&file, remaining_plane).z();
    let points = [
        positions[0],
        positions[0] + normal * 0.25,
        positions[0] + normal * 0.7,
        positions[1],
        positions[1] + normal * 0.45,
        positions[1] + normal * 0.9,
    ];
    let weights = [1.0, 0.75, 1.4, 1.0, 1.25, 0.8];
    let mut raw = Vec::new();
    for (index, point) in points.iter().copied().enumerate() {
        if rational {
            let weight = weights[index];
            raw.extend([point.x * weight, point.y * weight, point.z * weight, weight]);
        } else {
            raw.extend(point.to_array());
        }
    }

    let base_defs = kxt::schema::base_schema();
    for node_code in [
        code::B_SURFACE,
        code::NURBS_SURF,
        code::BSPLINE_VERTICES,
        code::KNOT_MULT,
        code::KNOT_SET,
    ] {
        file.defs.insert(
            node_code,
            base_defs
                .iter()
                .find(|definition| definition.code == node_code)
                .unwrap()
                .clone(),
        );
    }
    let first = file.nodes.keys().next_back().copied().unwrap() + 1;
    let bsurface = first;
    let nurbs = first + 1;
    let poles = first + 2;
    let u_mult = first + 3;
    let v_mult = first + 4;
    let u_knots = first + 5;
    let v_knots = first + 6;

    let mut common = plane.values[..7].to_vec();
    common[0] = Value::Int(i64::from(bsurface));
    for value in &mut common[1..6] {
        *value = Value::Ptr(0);
    }
    common.extend([Value::Ptr(nurbs), Value::Ptr(0)]);
    file.nodes.insert(
        bsurface,
        Node {
            code: code::B_SURFACE,
            values: common,
        },
    );
    file.nodes.insert(
        nurbs,
        Node {
            code: code::NURBS_SURF,
            values: vec![
                Value::Logical(false),
                Value::Logical(false),
                Value::Int(1),
                Value::Int(2),
                Value::Int(2),
                Value::Int(3),
                Value::Int(0),
                Value::Int(0),
                Value::Int(2),
                Value::Int(2),
                Value::Logical(rational),
                Value::Logical(false),
                Value::Logical(false),
                Value::Int(0),
                Value::Int(if rational { 4 } else { 3 }),
                Value::Ptr(poles),
                Value::Ptr(u_mult),
                Value::Ptr(v_mult),
                Value::Ptr(u_knots),
                Value::Ptr(v_knots),
            ],
        },
    );
    file.nodes.insert(
        poles,
        Node {
            code: code::BSPLINE_VERTICES,
            values: vec![Value::Arr(raw.into_iter().map(Value::Double).collect())],
        },
    );
    for (index, multiplicities) in [(u_mult, [2, 2]), (v_mult, [3, 3])] {
        file.nodes.insert(
            index,
            Node {
                code: code::KNOT_MULT,
                values: vec![Value::Arr(
                    multiplicities.into_iter().map(Value::Int).collect(),
                )],
            },
        );
    }
    for index in [u_knots, v_knots] {
        file.nodes.insert(
            index,
            Node {
                code: code::KNOT_SET,
                values: vec![Value::Arr(vec![Value::Double(0.0), Value::Double(1.0)])],
            },
        );
    }
    sources[1] = Value::Ptr(bsurface);
    set_field(&mut file, intersection, "surface", Value::Arr(sources));

    let data = ptr(&file, intersection, "intersection_data");
    let mut values = match file.field(&file.nodes[&data], "values").unwrap().clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[2] = Value::Double(0.0);
    values[3] = Value::Double(0.0);
    values[6] = Value::Double(1.0);
    values[7] = Value::Double(0.0);
    set_field(&mut file, data, "values", Value::Arr(values));
    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

/// Reparameterize the verified Plane/B-surface chart onto the affine
/// convention carried by corpus record 778, then wrap the B-surface in an
/// exactly compensating constant-normal offset. The resulting intersection
/// is unchanged; its carrier/pcurves keep the canonical sample-index basis
/// while the certificate retains the published noncanonical affine mapping
/// and the live offset root.
fn noncanonical_plane_offset_nurbs_intersection_file(rational: bool, swapped: bool) -> XtFile {
    let mut file = plane_bsurface_intersection_file(rational, false);
    let body = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::BODY).then_some(index))
        .unwrap();
    set_field(&mut file, body, "res_linear", Value::Double(1.0e-3));
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let plane = sources[0].as_ptr().unwrap();
    let basis = sources[1].as_ptr().unwrap();
    assert_eq!(file.nodes[&plane].code, code::PLANE);
    assert_eq!(file.nodes[&basis].code, code::B_SURFACE);

    let chart = ptr(&file, intersection, "chart");
    let positions = match file.field(&file.nodes[&chart], "hvec").unwrap() {
        Value::Arr(values) => values
            .iter()
            .map(|value| {
                let point = value.as_vector().unwrap();
                Point3::new(point[0], point[1], point[2])
            })
            .collect::<Vec<_>>(),
        _ => unreachable!(),
    };
    let tangent = (positions[1] - positions[0]).normalized().unwrap();
    let transverse = plane_frame(&file, plane).z();
    let natural_normal = tangent.cross(transverse).normalized().unwrap();
    let sense = file
        .field(&file.nodes[&basis], "sense")
        .and_then(Value::as_char)
        .unwrap();
    let amount = 0.125;
    let signed_distance = if sense == '+' { amount } else { -amount };
    let basis_shift = natural_normal * -signed_distance;

    let nurbs = ptr(&file, basis, "nurbs");
    let vertex_dim = file
        .field(&file.nodes[&nurbs], "vertex_dim")
        .and_then(Value::as_int)
        .unwrap() as usize;
    let poles = ptr(&file, nurbs, "bspline_vertices");
    let mut raw = match file.field(&file.nodes[&poles], "vertices").unwrap().clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    for pole in raw.chunks_exact_mut(vertex_dim) {
        let weight = if vertex_dim == 4 {
            pole[3].as_f64().unwrap()
        } else {
            1.0
        };
        for (coordinate, shift) in pole[..3].iter_mut().zip(basis_shift.to_array()) {
            *coordinate = Value::Double(coordinate.as_f64().unwrap() + shift * weight);
        }
    }
    set_field(&mut file, poles, "vertices", Value::Arr(raw));

    file.defs.insert(
        code::OFFSET_SURF,
        kxt::schema::base_schema()
            .into_iter()
            .find(|definition| definition.code == code::OFFSET_SURF)
            .unwrap(),
    );
    let offset = file.nodes.keys().next_back().copied().unwrap() + 1;
    file.nodes.insert(
        offset,
        Node {
            code: code::OFFSET_SURF,
            values: vec![
                Value::Int(i64::from(offset)),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Char(sense),
                Value::Char('U'),
                Value::Logical(false),
                Value::Ptr(basis),
                Value::Double(amount),
                Value::Null,
            ],
        },
    );
    sources[1] = Value::Ptr(offset);
    set_field(&mut file, intersection, "surface", Value::Arr(sources));

    let base_parameter = 0.003_586_209_316_397_325;
    let base_scale = 0.999_999_996_408_403;
    set_field(
        &mut file,
        chart,
        "base_parameter",
        Value::Double(base_parameter),
    );
    set_field(&mut file, chart, "base_scale", Value::Double(base_scale));

    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

fn bsurface_bsurface_intersection_file(
    rational_a: bool,
    rational_b: bool,
    swapped: bool,
) -> XtFile {
    let mut file = plane_bsurface_intersection_file(rational_b, false);
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let replaced_plane = sources[0].as_ptr().unwrap();
    assert_eq!(file.nodes[&replaced_plane].code, code::PLANE);
    let plane = file.nodes[&replaced_plane].clone();
    let chart = ptr(&file, intersection, "chart");
    let positions = match file.field(&file.nodes[&chart], "hvec").unwrap() {
        Value::Arr(values) => values
            .iter()
            .map(|value| {
                let value = value.as_vector().unwrap();
                Point3::new(value[0], value[1], value[2])
            })
            .collect::<Vec<_>>(),
        _ => unreachable!(),
    };
    let normal = plane_frame(&file, replaced_plane).z();
    let points = [
        positions[0],
        positions[0] + normal * 0.35,
        positions[0] - normal * 0.55,
        positions[1],
        positions[1] - normal * 0.4,
        positions[1] + normal * 0.8,
    ];
    let weights = [1.0, 1.3, 0.7, 1.0, 0.85, 1.4];
    let mut raw = Vec::new();
    for (index, point) in points.iter().copied().enumerate() {
        if rational_a {
            let weight = weights[index];
            raw.extend([point.x * weight, point.y * weight, point.z * weight, weight]);
        } else {
            raw.extend(point.to_array());
        }
    }

    let first = file.nodes.keys().next_back().copied().unwrap() + 1;
    let bsurface = first;
    let nurbs = first + 1;
    let poles = first + 2;
    let u_mult = first + 3;
    let v_mult = first + 4;
    let u_knots = first + 5;
    let v_knots = first + 6;
    let mut common = plane.values[..7].to_vec();
    common[0] = Value::Int(i64::from(bsurface));
    for value in &mut common[1..6] {
        *value = Value::Ptr(0);
    }
    common.extend([Value::Ptr(nurbs), Value::Ptr(0)]);
    file.nodes.insert(
        bsurface,
        Node {
            code: code::B_SURFACE,
            values: common,
        },
    );
    file.nodes.insert(
        nurbs,
        Node {
            code: code::NURBS_SURF,
            values: vec![
                Value::Logical(false),
                Value::Logical(false),
                Value::Int(1),
                Value::Int(2),
                Value::Int(2),
                Value::Int(3),
                Value::Int(0),
                Value::Int(0),
                Value::Int(2),
                Value::Int(2),
                Value::Logical(rational_a),
                Value::Logical(false),
                Value::Logical(false),
                Value::Int(0),
                Value::Int(if rational_a { 4 } else { 3 }),
                Value::Ptr(poles),
                Value::Ptr(u_mult),
                Value::Ptr(v_mult),
                Value::Ptr(u_knots),
                Value::Ptr(v_knots),
            ],
        },
    );
    file.nodes.insert(
        poles,
        Node {
            code: code::BSPLINE_VERTICES,
            values: vec![Value::Arr(raw.into_iter().map(Value::Double).collect())],
        },
    );
    for (index, multiplicities) in [(u_mult, [2, 2]), (v_mult, [3, 3])] {
        file.nodes.insert(
            index,
            Node {
                code: code::KNOT_MULT,
                values: vec![Value::Arr(
                    multiplicities.into_iter().map(Value::Int).collect(),
                )],
            },
        );
    }
    for index in [u_knots, v_knots] {
        file.nodes.insert(
            index,
            Node {
                code: code::KNOT_SET,
                values: vec![Value::Arr(vec![Value::Double(0.0), Value::Double(1.0)])],
            },
        );
    }
    sources[0] = Value::Ptr(bsurface);
    set_field(&mut file, intersection, "surface", Value::Arr(sources));

    let data = ptr(&file, intersection, "intersection_data");
    let mut values = match file.field(&file.nodes[&data], "values").unwrap().clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Double(0.0);
    values[1] = Value::Double(0.0);
    values[4] = Value::Double(1.0);
    values[5] = Value::Double(0.0);
    set_field(&mut file, data, "values", Value::Arr(values));
    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

/// Wrap one source of the verified B/B fixture in a direct nonzero offset.
/// The basis is translated by the inverse displacement so its lifted point
/// set is unchanged; certification must prove the complete basis normal is
/// regular and bind both the signed distance and offset root to that basis.
fn offset_nurbs_bsurface_intersection_file(rational: bool, swapped: bool) -> XtFile {
    let mut file = bsurface_bsurface_intersection_file(rational, rational, false);
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let basis = sources[0].as_ptr().unwrap();
    assert_eq!(file.nodes[&basis].code, code::B_SURFACE);
    let sense = file
        .field(&file.nodes[&basis], "sense")
        .and_then(Value::as_char)
        .unwrap();
    let amount = 0.125;
    let signed_distance = if sense == '+' { amount } else { -amount };
    let chart = ptr(&file, intersection, "chart");
    let chart_positions = match file.field(&file.nodes[&chart], "hvec").unwrap() {
        Value::Arr(values) => values
            .iter()
            .map(|value| {
                let point = value.as_vector().unwrap();
                Point3::new(point[0], point[1], point[2])
            })
            .collect::<Vec<_>>(),
        _ => unreachable!(),
    };
    let tangent = (chart_positions[1] - chart_positions[0])
        .normalized()
        .unwrap();
    let transverse_a = Frame::from_z(Point3::default(), tangent).unwrap().x();
    let natural_normal = tangent.cross(transverse_a).normalized().unwrap();
    let transverse_b = natural_normal;
    let direct = sources[1].as_ptr().unwrap();
    for (surface, transverse, shift) in [
        (basis, transverse_a, natural_normal * -signed_distance),
        (direct, transverse_b, Vec3::default()),
    ] {
        let nurbs = ptr(&file, surface, "nurbs");
        let rational = matches!(
            file.field(&file.nodes[&nurbs], "rational"),
            Some(Value::Logical(true))
        );
        let poles = ptr(&file, nurbs, "bspline_vertices");
        let mut raw = Vec::new();
        for origin in &chart_positions {
            for parameter in [0.0, 0.5, 1.0] {
                let point = *origin + transverse * parameter + shift;
                raw.extend(point.to_array().into_iter().map(Value::Double));
                if rational {
                    raw.push(Value::Double(1.0));
                }
            }
        }
        set_field(&mut file, poles, "vertices", Value::Arr(raw));
    }
    file.defs.insert(
        code::OFFSET_SURF,
        kxt::schema::base_schema()
            .into_iter()
            .find(|definition| definition.code == code::OFFSET_SURF)
            .unwrap(),
    );
    let offset = file.nodes.keys().next_back().copied().unwrap() + 1;
    file.nodes.insert(
        offset,
        Node {
            code: code::OFFSET_SURF,
            values: vec![
                Value::Int(i64::from(offset)),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Char(sense),
                Value::Char('U'),
                Value::Logical(false),
                Value::Ptr(basis),
                Value::Double(amount),
                Value::Null,
            ],
        },
    );
    sources[0] = Value::Ptr(offset);
    set_field(&mut file, intersection, "surface", Value::Arr(sources));
    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

/// Replace the plane operand of the verified Plane/B-surface fixture with a
/// direct or nested offset chain whose effective field is exactly that plane.
fn offset_bsurface_intersection_file(rational: bool, nested: bool, swapped: bool) -> XtFile {
    let mut file = plane_bsurface_intersection_file(rational, false);
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let replaced_plane = sources[0].as_ptr().unwrap();
    assert_eq!(file.nodes[&replaced_plane].code, code::PLANE);
    let plane = file.nodes[&replaced_plane].clone();
    let frame = plane_frame(&file, replaced_plane);
    let sense = file
        .field(&plane, "sense")
        .and_then(Value::as_char)
        .unwrap();
    let signed_total = if sense == '+' { 0.25 } else { -0.25 };
    let basis_origin = frame.origin() - frame.z() * signed_total;

    let mut next = file.nodes.keys().next_back().copied().unwrap() + 1;
    let basis = next;
    next += 1;
    file.nodes.insert(
        basis,
        Node {
            code: code::PLANE,
            values: plane.values,
        },
    );
    set_field(&mut file, basis, "node_id", Value::Int(i64::from(basis)));
    for name in [
        "attributes_groups",
        "owner",
        "next",
        "previous",
        "geometric_owner",
    ] {
        set_field(&mut file, basis, name, Value::Ptr(0));
    }
    set_field(
        &mut file,
        basis,
        "pvec",
        Value::Vector(Some(basis_origin.to_array())),
    );

    file.defs.insert(
        code::OFFSET_SURF,
        kxt::schema::base_schema()
            .into_iter()
            .find(|definition| definition.code == code::OFFSET_SURF)
            .unwrap(),
    );
    let offset_values = |index: u32, basis: u32, amount: f64| {
        vec![
            Value::Int(i64::from(index)),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Ptr(0),
            Value::Char(sense),
            Value::Char('U'),
            Value::Logical(false),
            Value::Ptr(basis),
            Value::Double(amount),
            Value::Null,
        ]
    };
    let source = if nested {
        let inner = next;
        next += 1;
        file.nodes.insert(
            inner,
            Node {
                code: code::OFFSET_SURF,
                values: offset_values(inner, basis, 0.125),
            },
        );
        let outer = next;
        file.nodes.insert(
            outer,
            Node {
                code: code::OFFSET_SURF,
                values: offset_values(outer, inner, 0.125),
            },
        );
        outer
    } else {
        let offset = next;
        file.nodes.insert(
            offset,
            Node {
                code: code::OFFSET_SURF,
                values: offset_values(offset, basis, 0.25),
            },
        );
        offset
    };
    sources[0] = Value::Ptr(source);
    set_field(&mut file, intersection, "surface", Value::Arr(sources));
    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

/// Replace both ordered plane operands with independent offset chains whose
/// effective fields remain the original distinct nonparallel planes.
fn offset_offset_intersection_file(
    nested: [bool; 2],
    amounts: [f64; 2],
    flip_senses: bool,
    swapped: bool,
) -> XtFile {
    let mut file = affine_plane_intersection_file();
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    file.defs.insert(
        code::OFFSET_SURF,
        kxt::schema::base_schema()
            .into_iter()
            .find(|definition| definition.code == code::OFFSET_SURF)
            .unwrap(),
    );
    let mut next = file.nodes.keys().next_back().copied().unwrap() + 1;
    for operand in 0..2 {
        let replaced_plane = sources[operand].as_ptr().unwrap();
        assert_eq!(file.nodes[&replaced_plane].code, code::PLANE);
        let plane = file.nodes[&replaced_plane].clone();
        let frame = plane_frame(&file, replaced_plane);
        let original_sense = file
            .field(&plane, "sense")
            .and_then(Value::as_char)
            .unwrap();
        let sense = if flip_senses {
            if original_sense == '+' { '-' } else { '+' }
        } else {
            original_sense
        };
        let signed_total = if sense == '+' {
            amounts[operand]
        } else {
            -amounts[operand]
        };
        let basis_origin = frame.origin() - frame.z() * signed_total;

        let basis = next;
        next += 1;
        file.nodes.insert(
            basis,
            Node {
                code: code::PLANE,
                values: plane.values,
            },
        );
        set_field(&mut file, basis, "node_id", Value::Int(i64::from(basis)));
        for name in [
            "attributes_groups",
            "owner",
            "next",
            "previous",
            "geometric_owner",
        ] {
            set_field(&mut file, basis, name, Value::Ptr(0));
        }
        set_field(&mut file, basis, "sense", Value::Char(sense));
        set_field(
            &mut file,
            basis,
            "pvec",
            Value::Vector(Some(basis_origin.to_array())),
        );

        let offset_values = |index: u32, basis: u32, amount: f64| {
            vec![
                Value::Int(i64::from(index)),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Char(sense),
                Value::Char('U'),
                Value::Logical(false),
                Value::Ptr(basis),
                Value::Double(amount),
                Value::Null,
            ]
        };
        let root = if nested[operand] {
            let inner = next;
            next += 1;
            file.nodes.insert(
                inner,
                Node {
                    code: code::OFFSET_SURF,
                    values: offset_values(inner, basis, amounts[operand] * 0.5),
                },
            );
            let outer = next;
            next += 1;
            file.nodes.insert(
                outer,
                Node {
                    code: code::OFFSET_SURF,
                    values: offset_values(outer, inner, amounts[operand] * 0.5),
                },
            );
            outer
        } else {
            let root = next;
            next += 1;
            file.nodes.insert(
                root,
                Node {
                    code: code::OFFSET_SURF,
                    values: offset_values(root, basis, amounts[operand]),
                },
            );
            root
        };
        sources[operand] = Value::Ptr(root);
    }
    set_field(&mut file, intersection, "surface", Value::Arr(sources));
    if swapped {
        swap_intersection_operands_and_orientation(&mut file, intersection);
    }
    file
}

fn swap_intersection_operands_and_orientation(file: &mut XtFile, intersection: u32) {
    let mut sources = match file
        .field(&file.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    sources.swap(0, 1);
    set_field(file, intersection, "surface", Value::Arr(sources));
    set_field(file, intersection, "sense", Value::Char('-'));

    let chart = ptr(file, intersection, "chart");
    let mut positions = match file.field(&file.nodes[&chart], "hvec").unwrap().clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    positions.reverse();
    set_field(file, chart, "hvec", Value::Arr(positions));

    let start = ptr(file, intersection, "start");
    let end = ptr(file, intersection, "end");
    set_field(file, intersection, "start", Value::Ptr(end));
    set_field(file, intersection, "end", Value::Ptr(start));

    let data = ptr(file, intersection, "intersection_data");
    let values = match file.field(&file.nodes[&data], "values").unwrap() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    let swapped = vec![
        values[6].clone(),
        values[7].clone(),
        values[4].clone(),
        values[5].clone(),
        values[2].clone(),
        values[3].clone(),
        values[0].clone(),
        values[1].clone(),
    ];
    set_field(file, data, "values", Value::Arr(swapped));

    let trim = file
        .nodes
        .iter()
        .find_map(|(&index, node)| {
            (node.code == code::TRIMMED_CURVE
                && file.field(node, "basis_curve").and_then(Value::as_ptr) == Some(intersection))
            .then_some(index)
        })
        .unwrap();
    set_field(file, trim, "parm_1", Value::Double(1.0));
    set_field(file, trim, "parm_2", Value::Double(0.0));
}

fn store_counts(store: &Store) -> (usize, usize, usize) {
    (
        store.count::<Body>(),
        store.count::<ktopo::geom::CurveGeom>(),
        store.geometry().len(),
    )
}

#[test]
fn modern_embedded_schema_wire_layout_pins_intersection_chart_limits_and_data() {
    let text = concat!(
        "**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n",
        "**PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~0123456789**************************\n",
        "**PART1;MC=none;APPL=kxt-tests;FORMAT=text;GUISE=transmit;\n",
        "**PART2;SCH=SCH_3701212_37102_13006;USFLD_SIZE=0;\n",
        "**PART3;\n",
        "**END_OF_HEADER*****************************************************************\n",
        "T1 X23 SCH_3701212_37102_13006 239 0 ",
        "38 12 CCCCCCCCCCCA17 intersection_data204 0 Z100 ",
        "1 0 0 0 0 0 +101 102 103 104 105 106 ",
        "40 255 2 103 0 1 2 0 0 ?? 0 0 0 1 0 0 ",
        "41 3 CI8 term_use0 0 1 cCZ1 104 L? 0 0 0 ",
        "41 1 105 L? 1 0 0 ",
        "204 2 17 INTERSECTION_DATA17 Intersection data",
        "7 uv_type0 0 1 u6 values0 1 1 fT",
        "8 106 4 0 0 0 0 1 0 1 0 ",
        "1 0 ",
    );
    let file = read_xt(text.as_bytes()).unwrap();
    let intersection = file.node(100).unwrap();
    assert_eq!(
        file.field(intersection, "intersection_data"),
        Some(&Value::Ptr(106))
    );
    let chart = file.node(103).unwrap();
    assert_eq!(file.field(chart, "chart_count"), Some(&Value::Int(2)));
    assert!(matches!(file.field(chart, "hvec"), Some(Value::Arr(values)) if values.len() == 2));
    let limit = file.node(104).unwrap();
    assert_eq!(file.field(limit, "type"), Some(&Value::Char('L')));
    assert_eq!(file.field(limit, "term_use"), Some(&Value::Char('?')));
    assert!(matches!(file.field(limit, "hvec"), Some(Value::Arr(values)) if values.len() == 1));
    let data = file.node(106).unwrap();
    assert_eq!(data.code, code::INTERSECTION_DATA);
    assert_eq!(file.field(data, "uv_type"), Some(&Value::Int(4)));
    assert!(matches!(file.field(data, "values"), Some(Value::Arr(values)) if values.len() == 8));
}

#[test]
fn finite_open_affine_plane_chart_retains_carrier_metadata_dependencies_and_bounds() {
    let file = affine_plane_intersection_file();
    let mut store = Store::new();
    let reconstruction = reconstruct(&file, &mut store).unwrap();
    assert_eq!(reconstruction.bodies.len(), 1);

    let (handle, descriptor) = store
        .geometry()
        .curves()
        .find(|(_, curve)| curve.class() == CurveClass::Intersection)
        .unwrap();
    let intersection = descriptor.as_transmitted_intersection().unwrap();
    let certificate = intersection.certificate();
    assert_eq!(certificate.metadata().base_parameter(), 0.0);
    assert_eq!(certificate.metadata().base_scale(), 1.0);
    assert_eq!(certificate.metadata().parameter_error(), [None, None]);
    assert_eq!(certificate.carrier().points().len(), 2);
    assert_eq!(intersection.source_surfaces().len(), 2);
    assert_eq!(intersection.pcurves().len(), 2);
    for dependency in intersection.source_surfaces().map(GeometryRef::Surface) {
        assert!(
            store
                .geometry()
                .dependents(dependency)
                .unwrap()
                .contains(&GeometryRef::Curve(handle))
        );
    }
    for dependency in intersection.pcurves().map(GeometryRef::Curve2d) {
        assert!(
            store
                .geometry()
                .dependents(dependency)
                .unwrap()
                .contains(&GeometryRef::Curve(handle))
        );
    }
    let edge = store
        .iter::<Edge>()
        .find_map(|(_, edge)| (edge.curve == Some(handle)).then_some(edge))
        .unwrap();
    assert_eq!(edge.bounds, Some((0.0, 1.0)));
    assert_eq!(
        descriptor.as_curve().eval(0.5),
        certificate.carrier().eval(0.5)
    );
}

#[test]
fn plane_offset_charts_retain_actual_sources_in_both_orders_and_nested_safe_chains() {
    for (nested, swapped, classes) in [
        (false, false, [SurfaceClass::Plane, SurfaceClass::Offset]),
        (false, true, [SurfaceClass::Offset, SurfaceClass::Plane]),
        (true, false, [SurfaceClass::Plane, SurfaceClass::Offset]),
    ] {
        let file = plane_offset_intersection_file(nested, swapped);
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.class() == CurveClass::Intersection)
            .unwrap();
        let intersection = descriptor.as_transmitted_intersection().unwrap();
        let sources = intersection.source_surfaces();
        assert_eq!(store.get(sources[0]).unwrap().class(), classes[0]);
        assert_eq!(store.get(sources[1]).unwrap().class(), classes[1]);
        for (index, source) in sources.iter().copied().enumerate() {
            let mut eval = store.eval_context(kgraph::EvalLimits::default(), Tolerances::default());
            assert_eq!(
                eval.surface_exact_plane(source).unwrap(),
                Some(intersection.certificate().surfaces()[index])
            );
        }
        assert_eq!(
            store
                .geometry()
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(intersection.pcurves()[0]),
                GeometryRef::Curve2d(intersection.pcurves()[1]),
            ]
        );
        let edge = store
            .iter::<Edge>()
            .find_map(|(_, edge)| (edge.curve == Some(curve)).then_some(edge))
            .unwrap();
        assert_eq!(edge.bounds, Some((0.0, 1.0)));
        if nested {
            let offset = sources[1];
            let inner = store.get(offset).unwrap().as_offset().unwrap().basis();
            assert_eq!(store.get(inner).unwrap().class(), SurfaceClass::Offset);
            let basis = store.get(inner).unwrap().as_offset().unwrap().basis();
            assert_eq!(store.get(basis).unwrap().class(), SurfaceClass::Plane);
        }
    }
}

#[test]
fn offset_offset_charts_retain_both_roots_senses_signed_distances_and_nested_chains() {
    for (nested, amounts, flip_senses, swapped) in [
        ([false, false], [0.25, 0.375], false, false),
        ([true, false], [-0.25, 0.375], false, true),
        ([false, true], [0.25, -0.375], true, false),
        ([true, true], [-0.25, -0.375], true, true),
    ] {
        let file = offset_offset_intersection_file(nested, amounts, flip_senses, swapped);
        let intersection_node = file
            .nodes
            .iter()
            .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
            .unwrap();
        let source_indices: [u32; 2] = match file
            .field(&file.nodes[&intersection_node], "surface")
            .unwrap()
        {
            Value::Arr(values) => [values[0].as_ptr().unwrap(), values[1].as_ptr().unwrap()],
            _ => unreachable!(),
        };
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.as_transmitted_intersection().is_some())
            .unwrap();
        let intersection = descriptor.as_transmitted_intersection().unwrap();
        let sources = intersection.source_surfaces();
        assert_eq!(
            sources.map(|source| store.get(source).unwrap().class()),
            [SurfaceClass::Offset, SurfaceClass::Offset]
        );
        assert_eq!(intersection.certificate().metadata().base_parameter(), 0.0);
        assert_eq!(intersection.certificate().metadata().base_scale(), 1.0);
        assert_eq!(intersection.certificate().carrier().points().len(), 2);
        assert!(
            intersection.certificate().surfaces()[0]
                .frame()
                .z()
                .cross(intersection.certificate().surfaces()[1].frame().z())
                .normalized()
                .is_some()
        );

        for index in 0..2 {
            let mut graph_current = sources[index];
            let mut graph_distance = 0.0;
            let mut graph_depth = 0;
            while let Some(offset) = store.get(graph_current).unwrap().as_offset() {
                graph_distance += offset.signed_distance();
                graph_depth += 1;
                graph_current = offset.basis();
            }
            assert_eq!(
                store.get(graph_current).unwrap().class(),
                SurfaceClass::Plane
            );

            let sense = file
                .field(&file.nodes[&source_indices[index]], "sense")
                .and_then(Value::as_char)
                .unwrap();
            let mut xt_current = source_indices[index];
            let mut transmitted_distance = 0.0;
            let mut xt_depth = 0;
            while file.nodes[&xt_current].code == code::OFFSET_SURF {
                let amount = file
                    .field(&file.nodes[&xt_current], "offset")
                    .and_then(Value::as_f64)
                    .unwrap();
                transmitted_distance += if sense == '+' { amount } else { -amount };
                xt_depth += 1;
                xt_current = ptr(&file, xt_current, "surface");
            }
            assert_eq!(graph_depth, xt_depth);
            assert_eq!(graph_distance, transmitted_distance);

            let mut evaluator =
                store.eval_context(kgraph::EvalLimits::default(), Tolerances::default());
            assert_eq!(
                evaluator.surface_exact_plane(sources[index]).unwrap(),
                Some(intersection.certificate().surfaces()[index])
            );
            assert_eq!(evaluator.last_query_usage().node_visits(), graph_depth + 1);
            assert_eq!(
                evaluator.last_query_usage().dependency_depth(),
                graph_depth + 1
            );

            assert!(
                store
                    .geometry()
                    .dependents(GeometryRef::Surface(sources[index]))
                    .unwrap()
                    .contains(&GeometryRef::Curve(curve))
            );
            let basis_dependents = store
                .geometry()
                .dependents(GeometryRef::Surface(graph_current))
                .unwrap();
            assert_eq!(basis_dependents.len(), 1);
            assert!(matches!(basis_dependents[0], GeometryRef::Surface(_)));
        }
        assert_eq!(
            store
                .geometry()
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(intersection.pcurves()[0]),
                GeometryRef::Curve2d(intersection.pcurves()[1]),
            ]
        );
        let edge = store
            .iter::<Edge>()
            .find_map(|(_, edge)| (edge.curve == Some(curve)).then_some(edge))
            .unwrap();
        assert_eq!(edge.bounds, Some((0.0, 1.0)));
        assert_eq!(
            descriptor.as_curve().eval(0.5),
            intersection.certificate().carrier().eval(0.5)
        );
        store.geometry().validate().unwrap();
    }
}

#[test]
fn plane_bsurface_charts_certify_nonplanar_polynomial_and_rational_sources_in_both_orders() {
    for (rational, swapped, classes) in [
        (false, false, [SurfaceClass::Plane, SurfaceClass::Nurbs]),
        (true, false, [SurfaceClass::Plane, SurfaceClass::Nurbs]),
        (true, true, [SurfaceClass::Nurbs, SurfaceClass::Plane]),
    ] {
        let file = plane_bsurface_intersection_file(rational, swapped);
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.as_transmitted_nurbs_intersection().is_some())
            .unwrap();
        let intersection = descriptor.as_transmitted_nurbs_intersection().unwrap();
        let sources = intersection.source_surfaces();
        assert_eq!(store.get(sources[0]).unwrap().class(), classes[0]);
        assert_eq!(store.get(sources[1]).unwrap().class(), classes[1]);
        let certificate = intersection.certificate();
        assert_eq!(certificate.metadata().base_parameter(), 0.0);
        assert_eq!(certificate.metadata().base_scale(), 1.0);
        assert_eq!(certificate.proof_depth(), 10);
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );
        let nurbs = sources
            .into_iter()
            .find_map(|source| store.get(source).unwrap().as_nurbs())
            .unwrap();
        assert_eq!(nurbs.is_rational(), rational);
        assert_ne!(nurbs.points()[1].z, nurbs.points()[4].z);
        assert_eq!(
            store
                .geometry()
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(intersection.pcurves()[0]),
                GeometryRef::Curve2d(intersection.pcurves()[1]),
            ]
        );
        let edge = store
            .iter::<Edge>()
            .find_map(|(_, edge)| (edge.curve == Some(curve)).then_some(edge))
            .unwrap();
        assert_eq!(edge.bounds, Some((0.0, 1.0)));
        assert_eq!(
            descriptor.as_curve().eval(0.5),
            certificate.carrier().eval(0.5)
        );
        store.geometry().validate().unwrap();
    }
}

#[test]
fn offset_bsurface_charts_retain_roots_and_protect_complete_plane_chains() {
    for (rational, nested, swapped) in [
        (false, false, false),
        (true, false, true),
        (false, true, true),
        (true, true, false),
    ] {
        let file = offset_bsurface_intersection_file(rational, nested, swapped);
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.as_transmitted_nurbs_intersection().is_some())
            .unwrap();
        let intersection = descriptor.as_transmitted_nurbs_intersection().unwrap();
        let sources = intersection.source_surfaces();
        let expected_classes = if swapped {
            [SurfaceClass::Nurbs, SurfaceClass::Offset]
        } else {
            [SurfaceClass::Offset, SurfaceClass::Nurbs]
        };
        assert_eq!(store.get(sources[0]).unwrap().class(), expected_classes[0]);
        assert_eq!(store.get(sources[1]).unwrap().class(), expected_classes[1]);
        assert_eq!(intersection.certificate().proof_depth(), 10);

        let plane_trace_index = usize::from(swapped);
        let certified_plane = intersection.certificate().traces()[plane_trace_index]
            .as_plane()
            .unwrap();
        let root = sources[plane_trace_index];
        let mut evaluator =
            store.eval_context(kgraph::EvalLimits::default(), Tolerances::default());
        assert_eq!(
            evaluator.surface_exact_plane(root).unwrap(),
            Some(certified_plane)
        );
        assert_eq!(
            evaluator.last_query_usage().node_visits(),
            if nested { 3 } else { 2 }
        );
        assert_eq!(
            evaluator.last_query_usage().dependency_depth(),
            if nested { 3 } else { 2 }
        );

        let inner = store.get(root).unwrap().as_offset().unwrap().basis();
        let basis = if nested {
            assert_eq!(store.get(inner).unwrap().class(), SurfaceClass::Offset);
            store.get(inner).unwrap().as_offset().unwrap().basis()
        } else {
            inner
        };
        assert_eq!(store.get(basis).unwrap().class(), SurfaceClass::Plane);
        assert_eq!(
            store
                .geometry()
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(intersection.pcurves()[0]),
                GeometryRef::Curve2d(intersection.pcurves()[1]),
            ]
        );
        let basis_dependents = store
            .geometry()
            .dependents(GeometryRef::Surface(basis))
            .unwrap();
        assert_eq!(basis_dependents.len(), 1);
        assert!(matches!(basis_dependents[0], GeometryRef::Surface(_)));
        assert!(
            store
                .geometry()
                .dependents(GeometryRef::Surface(root))
                .unwrap()
                .contains(&GeometryRef::Curve(curve)),
            "a live transmitted chart must protect its actual offset root"
        );
        let edge = store
            .iter::<Edge>()
            .find_map(|(_, edge)| (edge.curve == Some(curve)).then_some(edge))
            .unwrap();
        assert_eq!(edge.bounds, Some((0.0, 1.0)));
        store.geometry().validate().unwrap();
    }
}

#[test]
fn bsurface_bsurface_charts_certify_two_original_nonplanar_sources_in_both_orders() {
    for (rational_a, rational_b, swapped) in [
        (false, false, false),
        (true, false, false),
        (false, true, true),
        (true, true, true),
    ] {
        let file = bsurface_bsurface_intersection_file(rational_a, rational_b, swapped);
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.as_transmitted_nurbs_intersection().is_some())
            .unwrap();
        let intersection = descriptor.as_transmitted_nurbs_intersection().unwrap();
        let sources = intersection.source_surfaces();
        assert_eq!(
            sources.map(|source| store.get(source).unwrap().class()),
            [SurfaceClass::Nurbs, SurfaceClass::Nurbs]
        );
        let expected_rational = if swapped {
            [rational_b, rational_a]
        } else {
            [rational_a, rational_b]
        };
        for index in 0..2 {
            let live = store.get(sources[index]).unwrap().as_nurbs().unwrap();
            assert_eq!(live.is_rational(), expected_rational[index]);
            assert_eq!(
                intersection.certificate().traces()[index].as_nurbs(),
                Some(live)
            );
            assert_ne!(live.points()[1], live.points()[2]);
        }
        assert_ne!(
            store.get(sources[0]).unwrap().as_nurbs(),
            store.get(sources[1]).unwrap().as_nurbs()
        );
        let certificate = intersection.certificate();
        assert_eq!(certificate.metadata().base_parameter(), 0.0);
        assert_eq!(certificate.metadata().base_scale(), 1.0);
        assert_eq!(certificate.proof_depth(), 10);
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );
        assert_eq!(
            store
                .geometry()
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(intersection.pcurves()[0]),
                GeometryRef::Curve2d(intersection.pcurves()[1]),
            ]
        );
        let edge = store
            .iter::<Edge>()
            .find_map(|(_, edge)| (edge.curve == Some(curve)).then_some(edge))
            .unwrap();
        assert_eq!(edge.bounds, Some((0.0, 1.0)));
        assert_eq!(
            descriptor.as_curve().eval(0.5),
            certificate.carrier().eval(0.5)
        );
        store.geometry().validate().unwrap();
    }
}

#[test]
fn offset_nurbs_charts_bind_live_root_basis_and_signed_distance_in_both_orders() {
    for (rational, swapped) in [(false, false), (true, false), (true, true)] {
        let file = offset_nurbs_bsurface_intersection_file(rational, swapped);
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.as_transmitted_nurbs_intersection().is_some())
            .unwrap();
        let intersection = descriptor.as_transmitted_nurbs_intersection().unwrap();
        let offset_index = usize::from(swapped);
        let root = intersection.source_surfaces()[offset_index];
        let live_offset = store.get(root).unwrap().as_offset().copied().unwrap();
        let trace = intersection.certificate().traces()[offset_index]
            .as_offset_nurbs()
            .unwrap();
        assert_eq!(trace.signed_distance(), live_offset.signed_distance());
        assert_eq!(trace.signed_distance().abs(), 0.125);
        assert_eq!(
            store.get(live_offset.basis()).unwrap().as_nurbs(),
            Some(trace.basis())
        );
        assert!(
            store
                .geometry()
                .dependents(GeometryRef::Surface(root))
                .unwrap()
                .contains(&GeometryRef::Curve(curve))
        );
        assert!(
            store
                .geometry()
                .dependents(GeometryRef::Surface(live_offset.basis()))
                .unwrap()
                .contains(&GeometryRef::Surface(root))
        );
        store.geometry().validate().unwrap();
    }
}

#[test]
fn noncanonical_plane_offset_nurbs_retains_affine_metadata_sources_and_dependencies() {
    const BASE_PARAMETER: f64 = 0.003_586_209_316_397_325;
    const BASE_SCALE: f64 = 0.999_999_996_408_403;
    let expected_knots = [0.0, 0.0, 1.0, 1.0];

    for (rational, swapped) in [(false, false), (true, false), (true, true)] {
        let file = noncanonical_plane_offset_nurbs_intersection_file(rational, swapped);
        let mut store = Store::new();
        let reconstruction = reconstruct(&file, &mut store).unwrap();
        assert_eq!(reconstruction.bodies.len(), 1);
        let (curve, descriptor) = store
            .geometry()
            .curves()
            .find(|(_, curve)| curve.as_transmitted_nurbs_intersection().is_some())
            .unwrap();
        let intersection = descriptor.as_transmitted_nurbs_intersection().unwrap();
        let certificate = intersection.certificate();
        assert_eq!(certificate.metadata().base_parameter(), BASE_PARAMETER);
        assert_eq!(certificate.metadata().base_scale(), BASE_SCALE);
        assert_eq!(
            (
                certificate.carrier_range().lo,
                certificate.carrier_range().hi
            ),
            (0.0, 1.0)
        );
        assert_eq!(certificate.carrier().knots().as_slice(), expected_knots);
        for pcurve in certificate.pcurves() {
            assert_eq!(pcurve.knots().as_slice(), expected_knots);
            assert_eq!(pcurve.param_range(), certificate.carrier_range());
        }
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );

        let sources = intersection.source_surfaces();
        let expected_classes = if swapped {
            [SurfaceClass::Offset, SurfaceClass::Plane]
        } else {
            [SurfaceClass::Plane, SurfaceClass::Offset]
        };
        assert_eq!(
            sources.map(|source| store.get(source).unwrap().class()),
            expected_classes
        );
        let offset_index = usize::from(!swapped);
        let root = sources[offset_index];
        let live_offset = store.get(root).unwrap().as_offset().copied().unwrap();
        let trace = certificate.traces()[offset_index]
            .as_offset_nurbs()
            .unwrap();
        assert_eq!(trace.signed_distance(), live_offset.signed_distance());
        assert_eq!(
            store.get(live_offset.basis()).unwrap().as_nurbs(),
            Some(trace.basis())
        );
        assert_eq!(
            store
                .geometry()
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(intersection.pcurves()[0]),
                GeometryRef::Curve2d(intersection.pcurves()[1]),
            ]
        );
        let edge = store
            .iter::<Edge>()
            .find_map(|(_, edge)| (edge.curve == Some(curve)).then_some(edge))
            .unwrap();
        assert_eq!(edge.bounds, Some((0.0, 1.0)));
        assert_eq!(
            descriptor.as_curve().eval(0.5),
            certificate.carrier().eval(0.5)
        );
        store.geometry().validate().unwrap();
    }
}

fn contextual_with_limit(
    file: &XtFile,
    spec: LimitSpec,
) -> (
    Store,
    kcore::operation::OperationOutcome<kxt::Reconstruction, XtError>,
) {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_budget_overrides(BudgetPlan::new([spec]).unwrap());
    let mut store = Store::new();
    let outcome = reconstruct_with_context(file, &mut store, &context).unwrap();
    (store, outcome)
}

#[test]
fn intersection_chart_work_items_and_depth_have_exact_n_and_n_minus_one_limits() {
    for file in [
        affine_plane_intersection_file(),
        plane_offset_intersection_file(false, false),
        offset_offset_intersection_file([false, false], [0.25, 0.375], false, false),
        offset_offset_intersection_file([true, true], [-0.25, -0.375], true, true),
    ] {
        for (stage, resource, mode, exact) in [
            (
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                4,
            ),
            (
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                2,
            ),
            (
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                1,
            ),
        ] {
            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact));
            assert!(outcome.result().is_ok(), "exact {stage:?} limit must pass");
            assert_eq!(store.count::<Body>(), 1);
            let usage = outcome
                .report()
                .usage()
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap();
            assert_eq!((usage.consumed, usage.allowed), (exact, exact));

            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact - 1));
            let result = outcome.result();
            let error = result.as_ref().unwrap_err();
            let limit = error.limit().unwrap();
            assert_eq!(limit.stage, stage);
            assert_eq!((limit.consumed, limit.allowed), (exact, exact - 1));
            assert_eq!(store_counts(&store), (0, 0, 0));
        }
    }
}

#[test]
fn offset_offset_graph_query_has_exact_node_and_depth_boundaries() {
    for (file, visits, depth) in [
        (
            offset_offset_intersection_file([false, false], [0.25, 0.375], false, false),
            34,
            2,
        ),
        (
            offset_offset_intersection_file([true, true], [-0.25, -0.375], true, true),
            36,
            3,
        ),
    ] {
        for (stage, resource, mode, exact) in [
            (
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                visits,
            ),
            (
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                depth,
            ),
        ] {
            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact));
            assert!(outcome.result().is_ok(), "exact {stage:?} limit must pass");
            assert_eq!(store.count::<Body>(), 1);
            let usage = outcome
                .report()
                .usage()
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap();
            assert_eq!((usage.consumed, usage.allowed), (exact, exact));

            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact - 1));
            let result = outcome.result();
            let error = result.as_ref().unwrap_err();
            let limit = error.limit().unwrap();
            assert_eq!(limit.stage, stage);
            assert_eq!((limit.consumed, limit.allowed), (exact, exact - 1));
            assert_eq!(store_counts(&store), (0, 0, 0));
        }
    }
}

#[test]
fn plane_bsurface_proof_has_exact_work_items_and_depth_boundaries() {
    for file in [
        plane_bsurface_intersection_file(true, false),
        offset_bsurface_intersection_file(true, false, false),
        offset_bsurface_intersection_file(true, true, true),
        noncanonical_plane_offset_nurbs_intersection_file(true, false),
    ] {
        for (stage, resource, mode, exact) in [
            (
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                7_170,
            ),
            (
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                2,
            ),
            (
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                10,
            ),
        ] {
            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact));
            assert!(outcome.result().is_ok(), "exact {stage:?} limit must pass");
            assert_eq!(store.count::<Body>(), 1);
            let usage = outcome
                .report()
                .usage()
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap();
            assert_eq!((usage.consumed, usage.allowed), (exact, exact));

            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact - 1));
            let result = outcome.result();
            let error = result.as_ref().unwrap_err();
            let limit = error.limit().unwrap();
            assert_eq!(limit.stage, stage);
            assert_eq!((limit.consumed, limit.allowed), (exact, exact - 1));
            assert_eq!(store_counts(&store), (0, 0, 0));
        }
    }
}

#[test]
fn offset_bsurface_graph_query_has_exact_node_and_depth_boundaries() {
    for (file, visits, depth) in [
        (offset_bsurface_intersection_file(true, false, false), 32, 2),
        (offset_bsurface_intersection_file(true, true, true), 33, 3),
    ] {
        for (stage, resource, mode, exact) in [
            (
                kgraph::eval_stage::NODE_VISITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                visits,
            ),
            (
                kgraph::eval_stage::DEPENDENCY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                depth,
            ),
        ] {
            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact));
            assert!(outcome.result().is_ok(), "exact {stage:?} limit must pass");
            assert_eq!(store.count::<Body>(), 1);
            let usage = outcome
                .report()
                .usage()
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap();
            assert_eq!((usage.consumed, usage.allowed), (exact, exact));

            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact - 1));
            let result = outcome.result();
            let error = result.as_ref().unwrap_err();
            let limit = error.limit().unwrap();
            assert_eq!(limit.stage, stage);
            assert_eq!((limit.consumed, limit.allowed), (exact, exact - 1));
            assert_eq!(store_counts(&store), (0, 0, 0));
        }
    }
}

#[test]
fn two_nurbs_trace_proofs_have_exact_summed_work_items_and_depth_boundaries() {
    for file in [
        bsurface_bsurface_intersection_file(true, true, false),
        offset_nurbs_bsurface_intersection_file(true, false),
        offset_nurbs_bsurface_intersection_file(true, true),
    ] {
        for (stage, resource, mode, exact) in [
            (
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                14_336,
            ),
            (
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                2,
            ),
            (
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                10,
            ),
        ] {
            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact));
            assert!(outcome.result().is_ok(), "exact {stage:?} limit must pass");
            assert_eq!(store.count::<Body>(), 1);
            let usage = outcome
                .report()
                .usage()
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap();
            assert_eq!((usage.consumed, usage.allowed), (exact, exact));

            let (store, outcome) =
                contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact - 1));
            let result = outcome.result();
            let error = result.as_ref().unwrap_err();
            let limit = error.limit().unwrap();
            assert_eq!(limit.stage, stage);
            assert_eq!((limit.consumed, limit.allowed), (exact, exact - 1));
            assert_eq!(store_counts(&store), (0, 0, 0));
        }
    }
}

#[test]
fn bsurface_bsurface_graph_accounting_has_exact_node_and_depth_boundaries() {
    let file = bsurface_bsurface_intersection_file(true, true, true);
    for (stage, resource, mode, exact) in [
        (
            kgraph::eval_stage::NODE_VISITS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            30,
        ),
        (
            kgraph::eval_stage::DEPENDENCY_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            1,
        ),
    ] {
        let (store, outcome) =
            contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact));
        assert!(outcome.result().is_ok(), "exact {stage:?} limit must pass");
        assert_eq!(store.count::<Body>(), 1);
        let usage = outcome
            .report()
            .usage()
            .iter()
            .find(|entry| entry.stage == stage)
            .unwrap();
        assert_eq!((usage.consumed, usage.allowed), (exact, exact));

        let (store, outcome) =
            contextual_with_limit(&file, LimitSpec::new(stage, resource, mode, exact - 1));
        let result = outcome.result();
        let error = result.as_ref().unwrap_err();
        let limit = error.limit().unwrap();
        assert_eq!(limit.stage, stage);
        assert_eq!((limit.consumed, limit.allowed), (exact, exact - 1));
        assert_eq!(store_counts(&store), (0, 0, 0));
    }
}

#[test]
fn null_and_malformed_uvs_reject_with_typed_evidence_and_atomic_rollback() {
    let valid = affine_plane_intersection_file();
    let data = valid
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION_DATA).then_some(index))
        .unwrap();
    let mut null = affine_plane_intersection_file();
    let mut values = match null.field(&null.nodes[&data], "values").unwrap().clone() {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Null;
    set_field(&mut null, data, "values", Value::Arr(values));
    let mut store = Store::new();
    let error = reconstruct(&null, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionChartData)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut malformed = affine_plane_intersection_file();
    let mut values = match malformed
        .field(&malformed.nodes[&data], "values")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values.pop();
    set_field(&mut malformed, data, "values", Value::Arr(values));
    let error = reconstruct(&malformed, &mut store).unwrap_err();
    assert!(matches!(error, XtError::BadField { index, .. } if index == data));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let reconstruction = reconstruct(&valid, &mut store).unwrap();
    let mut fresh = Store::new();
    let fresh_reconstruction = reconstruct(&valid, &mut fresh).unwrap();
    assert_eq!(reconstruction.bodies, fresh_reconstruction.bodies);
    assert_eq!(store_counts(&store), store_counts(&fresh));
}

#[test]
fn non_affine_chart_and_surface_residual_fail_closed() {
    let valid = affine_plane_intersection_file();
    let chart = valid
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::CHART).then_some(index))
        .unwrap();
    let mut convention = affine_plane_intersection_file();
    let positions = match convention
        .field(&convention.nodes[&chart], "hvec")
        .unwrap()
        .clone()
    {
        Value::Arr(mut positions) => {
            if let Value::Vector(Some(point)) = &mut positions[1] {
                point[0] *= 0.5;
                point[1] *= 0.5;
                point[2] *= 0.5;
            }
            positions
        }
        _ => unreachable!(),
    };
    set_field(&mut convention, chart, "hvec", Value::Arr(positions));
    let mut store = Store::new();
    let error = reconstruct(&convention, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionChartConvention)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let data = valid
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION_DATA).then_some(index))
        .unwrap();
    let mut residual = valid;
    let mut values = match residual
        .field(&residual.nodes[&data], "values")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[0] = Value::Double(values[0].as_f64().unwrap() + 0.25);
    set_field(&mut residual, data, "values", Value::Arr(values));
    let error = reconstruct(&residual, &mut store).unwrap_err();
    assert!(matches!(error, XtError::IntersectionCertificate { .. }));
    assert_eq!(store_counts(&store), (0, 0, 0));
}

fn intersection_offset_source(file: &XtFile) -> u32 {
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    match file.field(&file.nodes[&intersection], "surface").unwrap() {
        Value::Arr(values) => values
            .iter()
            .filter_map(Value::as_ptr)
            .find(|index| file.nodes[index].code == code::OFFSET_SURF)
            .unwrap(),
        _ => unreachable!(),
    }
}

fn intersection_offset_sources(file: &XtFile) -> [u32; 2] {
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    match file.field(&file.nodes[&intersection], "surface").unwrap() {
        Value::Arr(values) => {
            let sources = [values[0].as_ptr().unwrap(), values[1].as_ptr().unwrap()];
            assert!(
                sources
                    .iter()
                    .all(|source| file.nodes[source].code == code::OFFSET_SURF)
            );
            sources
        }
        _ => unreachable!(),
    }
}

fn offset_chain_terminal(file: &XtFile, root: u32) -> u32 {
    let mut current = root;
    while file.nodes[&current].code == code::OFFSET_SURF {
        current = ptr(file, current, "surface");
    }
    current
}

fn intersection_bsurface_source(file: &XtFile) -> u32 {
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    match file.field(&file.nodes[&intersection], "surface").unwrap() {
        Value::Arr(values) => values
            .iter()
            .filter_map(Value::as_ptr)
            .find(|index| file.nodes[index].code == code::B_SURFACE)
            .unwrap(),
        _ => unreachable!(),
    }
}

fn intersection_bsurface_sources(file: &XtFile) -> [u32; 2] {
    let intersection = file
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    match file.field(&file.nodes[&intersection], "surface").unwrap() {
        Value::Arr(values) => {
            let sources = [values[0].as_ptr().unwrap(), values[1].as_ptr().unwrap()];
            assert!(
                sources
                    .iter()
                    .all(|source| file.nodes[source].code == code::B_SURFACE)
            );
            sources
        }
        _ => unreachable!(),
    }
}

#[test]
fn plane_bsurface_perturbations_and_periodicity_fail_closed_with_reuse() {
    let valid = plane_bsurface_intersection_file(true, false);
    let intersection = valid
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let bsurface = intersection_bsurface_source(&valid);
    let nurbs = ptr(&valid, bsurface, "nurbs");
    let poles = ptr(&valid, nurbs, "bspline_vertices");
    let data = ptr(&valid, intersection, "intersection_data");
    let mut store = Store::new();

    let mut wrong_uv = plane_bsurface_intersection_file(true, false);
    let mut values = match wrong_uv
        .field(&wrong_uv.nodes[&data], "values")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[2] = Value::Double(0.05);
    set_field(&mut wrong_uv, data, "values", Value::Arr(values));
    assert!(matches!(
        reconstruct(&wrong_uv, &mut store),
        Err(XtError::IntersectionCertificate { .. })
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut wrong_source = plane_bsurface_intersection_file(true, false);
    let mut values = match wrong_source
        .field(&wrong_source.nodes[&poles], "vertices")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    values[2] = Value::Double(values[2].as_f64().unwrap() + 0.05);
    set_field(&mut wrong_source, poles, "vertices", Value::Arr(values));
    assert!(matches!(
        reconstruct(&wrong_source, &mut store),
        Err(XtError::IntersectionCertificate { .. })
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let chart = ptr(&valid, intersection, "chart");
    let mut wrong_carrier = plane_bsurface_intersection_file(true, false);
    let plane = match wrong_carrier
        .field(&wrong_carrier.nodes[&intersection], "surface")
        .unwrap()
    {
        Value::Arr(values) => values[0].as_ptr().unwrap(),
        _ => unreachable!(),
    };
    let shift = plane_frame(&wrong_carrier, plane).z() * 0.05;
    let mut positions = match wrong_carrier
        .field(&wrong_carrier.nodes[&chart], "hvec")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    for value in &mut positions {
        let point = value.as_vector().unwrap();
        *value = Value::Vector(Some(
            (Point3::new(point[0], point[1], point[2]) + shift).to_array(),
        ));
    }
    set_field(
        &mut wrong_carrier,
        chart,
        "hvec",
        Value::Arr(positions.clone()),
    );
    for (name, position) in [("start", 0), ("end", 1)] {
        let limit = ptr(&wrong_carrier, intersection, name);
        set_field(
            &mut wrong_carrier,
            limit,
            "hvec",
            Value::Arr(vec![positions[position].clone()]),
        );
    }
    assert!(matches!(
        reconstruct(&wrong_carrier, &mut store),
        Err(XtError::IntersectionCertificate { .. })
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    // The declared 3x2 net has the same six-pole product as the knot-implied
    // 2x3 net, but would undercharge source tensor-span work (zero versus one
    // span slot in v) if accounting trusted only the declared factors.
    let mut mismatched_factors = plane_bsurface_intersection_file(true, false);
    set_field(
        &mut mismatched_factors,
        nurbs,
        "n_u_vertices",
        Value::Int(3),
    );
    set_field(
        &mut mismatched_factors,
        nurbs,
        "n_v_vertices",
        Value::Int(2),
    );
    let error = reconstruct(&mismatched_factors, &mut store).unwrap_err();
    assert!(matches!(error, XtError::BadField { index, .. } if index == bsurface));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut periodic = plane_bsurface_intersection_file(true, false);
    set_field(&mut periodic, nurbs, "u_periodic", Value::Logical(true));
    let error = reconstruct(&periodic, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::PeriodicNurbsSurfaces)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let reconstruction = reconstruct(&valid, &mut store).unwrap();
    let mut fresh = Store::new();
    let fresh_reconstruction = reconstruct(&valid, &mut fresh).unwrap();
    assert_eq!(reconstruction.bodies, fresh_reconstruction.bodies);
    assert_eq!(store_counts(&store), store_counts(&fresh));
}

#[test]
fn bsurface_bsurface_periodic_closed_altered_and_noncanonical_cases_fail_with_reuse() {
    let fixture = || bsurface_bsurface_intersection_file(true, true, false);
    let valid = fixture();
    let intersection = valid
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let sources = intersection_bsurface_sources(&valid);
    let mut store = Store::new();

    for source in sources {
        let mut periodic = fixture();
        let nurbs = ptr(&periodic, source, "nurbs");
        set_field(&mut periodic, nurbs, "u_periodic", Value::Logical(true));
        let error = reconstruct(&periodic, &mut store).unwrap_err();
        assert_eq!(
            error.capability(),
            Some(XtCapability::PeriodicNurbsSurfaces)
        );
        assert_eq!(store_counts(&store), (0, 0, 0));

        let mut closed = fixture();
        let nurbs = ptr(&closed, source, "nurbs");
        set_field(&mut closed, nurbs, "v_closed", Value::Logical(true));
        let error = reconstruct(&closed, &mut store).unwrap_err();
        assert_eq!(
            error.capability(),
            Some(XtCapability::PeriodicNurbsSurfaces)
        );
        assert_eq!(store_counts(&store), (0, 0, 0));

        let mut altered = fixture();
        let nurbs = ptr(&altered, source, "nurbs");
        let poles = ptr(&altered, nurbs, "bspline_vertices");
        let mut values = match altered
            .field(&altered.nodes[&poles], "vertices")
            .unwrap()
            .clone()
        {
            Value::Arr(values) => values,
            _ => unreachable!(),
        };
        values[2] = Value::Double(values[2].as_f64().unwrap() + 0.05);
        set_field(&mut altered, poles, "vertices", Value::Arr(values));
        assert!(matches!(
            reconstruct(&altered, &mut store),
            Err(XtError::IntersectionCertificate { .. })
        ));
        assert_eq!(store_counts(&store), (0, 0, 0));
    }

    let data = ptr(&valid, intersection, "intersection_data");
    for uv_index in [0, 2] {
        let mut wrong_uv = fixture();
        let mut values = match wrong_uv
            .field(&wrong_uv.nodes[&data], "values")
            .unwrap()
            .clone()
        {
            Value::Arr(values) => values,
            _ => unreachable!(),
        };
        values[uv_index] = Value::Double(values[uv_index].as_f64().unwrap() + 0.05);
        set_field(&mut wrong_uv, data, "values", Value::Arr(values));
        assert!(matches!(
            reconstruct(&wrong_uv, &mut store),
            Err(XtError::IntersectionCertificate { .. })
        ));
        assert_eq!(store_counts(&store), (0, 0, 0));
    }

    let chart = ptr(&valid, intersection, "chart");
    let mut wrong_carrier = fixture();
    let mut positions = match wrong_carrier
        .field(&wrong_carrier.nodes[&chart], "hvec")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    for value in &mut positions {
        let point = value.as_vector().unwrap();
        *value = Value::Vector(Some([point[0], point[1], point[2] + 0.05]));
    }
    set_field(
        &mut wrong_carrier,
        chart,
        "hvec",
        Value::Arr(positions.clone()),
    );
    for (name, position) in [("start", 0), ("end", 1)] {
        let limit = ptr(&wrong_carrier, intersection, name);
        set_field(
            &mut wrong_carrier,
            limit,
            "hvec",
            Value::Arr(vec![positions[position].clone()]),
        );
    }
    assert!(matches!(
        reconstruct(&wrong_carrier, &mut store),
        Err(XtError::IntersectionCertificate { .. })
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut mismatched = fixture();
    let nurbs = ptr(&mismatched, sources[1], "nurbs");
    set_field(&mut mismatched, nurbs, "n_u_vertices", Value::Int(3));
    assert!(matches!(
        reconstruct(&mismatched, &mut store),
        Err(XtError::BadField { index, .. }) if index == sources[1]
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut overflowing_vertex_dimension = fixture();
    let nurbs = ptr(&overflowing_vertex_dimension, sources[1], "nurbs");
    set_field(
        &mut overflowing_vertex_dimension,
        nurbs,
        "vertex_dim",
        Value::Double(f64::MAX),
    );
    assert!(matches!(
        reconstruct(&overflowing_vertex_dimension, &mut store),
        Err(XtError::BadField { index, .. }) if index == sources[1]
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut noncanonical = fixture();
    set_field(
        &mut noncanonical,
        chart,
        "base_parameter",
        Value::Double(0.25),
    );
    let error = reconstruct(&noncanonical, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionChartConvention)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut null_limit = fixture();
    set_field(&mut null_limit, intersection, "end", Value::Ptr(0));
    let error = reconstruct(&null_limit, &mut store).unwrap_err();
    assert_eq!(error.capability(), Some(XtCapability::IntersectionLimits));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let reconstruction = reconstruct(&valid, &mut store).unwrap();
    let mut fresh = Store::new();
    let fresh_reconstruction = reconstruct(&fixture(), &mut fresh).unwrap();
    assert_eq!(reconstruction.bodies, fresh_reconstruction.bodies);
    assert_eq!(reconstruction.journal, fresh_reconstruction.journal);
    assert_eq!(store_counts(&store), store_counts(&fresh));
}

#[test]
fn offset_bsurface_altered_unsafe_cyclic_nested_and_shared_basis_fail_atomically_with_reuse() {
    let valid = offset_bsurface_intersection_file(true, false, false);
    let root = intersection_offset_source(&valid);
    let bsurface = intersection_bsurface_source(&valid);
    let mut store = Store::new();

    let mut altered = offset_bsurface_intersection_file(true, false, false);
    set_field(&mut altered, root, "offset", Value::Double(0.3));
    assert!(matches!(
        reconstruct(&altered, &mut store),
        Err(XtError::IntersectionCertificate { .. })
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut unsafe_offset = offset_bsurface_intersection_file(true, false, false);
    set_field(&mut unsafe_offset, root, "check", Value::Char('I'));
    assert!(matches!(
        reconstruct(&unsafe_offset, &mut store),
        Err(XtError::BadField { index, .. }) if index == root
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut cyclic = offset_bsurface_intersection_file(true, false, false);
    set_field(&mut cyclic, root, "surface", Value::Ptr(root));
    assert!(matches!(
        reconstruct(&cyclic, &mut store),
        Err(XtError::SurfaceDependencyCycle { path }) if path == vec![root, root]
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut nested_offset_nurbs = offset_bsurface_intersection_file(true, false, false);
    let inner = nested_offset_nurbs
        .nodes
        .keys()
        .next_back()
        .copied()
        .unwrap()
        + 1;
    nested_offset_nurbs
        .nodes
        .insert(inner, nested_offset_nurbs.nodes[&root].clone());
    set_field(
        &mut nested_offset_nurbs,
        inner,
        "surface",
        Value::Ptr(bsurface),
    );
    set_field(&mut nested_offset_nurbs, root, "surface", Value::Ptr(inner));
    let error = reconstruct(&nested_offset_nurbs, &mut store).unwrap_err();
    assert!(matches!(
        error,
        XtError::Unsupported {
            capability: XtCapability::IntersectionSurfaceFamily,
            what: "INTERSECTION offset source does not resolve to an exact plane field",
        }
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut shared_basis = offset_bsurface_intersection_file(true, false, false);
    let nurbs_sense = shared_basis
        .field(&shared_basis.nodes[&bsurface], "sense")
        .and_then(Value::as_char)
        .unwrap();
    set_field(&mut shared_basis, root, "sense", Value::Char(nurbs_sense));
    set_field(&mut shared_basis, root, "surface", Value::Ptr(bsurface));
    assert!(matches!(
        reconstruct(&shared_basis, &mut store),
        Err(XtError::IntersectionCertificate { .. })
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let reconstruction = reconstruct(&valid, &mut store).unwrap();
    let mut fresh = Store::new();
    let fresh_reconstruction = reconstruct(&valid, &mut fresh).unwrap();
    assert_eq!(reconstruction.bodies, fresh_reconstruction.bodies);
    assert_eq!(reconstruction.journal, fresh_reconstruction.journal);
    assert_eq!(store_counts(&store), store_counts(&fresh));
}

#[test]
fn offset_offset_altered_crosslinked_cyclic_nonplane_and_noncanonical_inputs_fail_atomically() {
    let fixture = || offset_offset_intersection_file([true, true], [0.25, -0.375], false, false);
    let valid = fixture();
    let roots = intersection_offset_sources(&valid);
    let intersection = valid
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut store = Store::new();

    for root in roots {
        let mut altered = fixture();
        let amount = altered
            .field(&altered.nodes[&root], "offset")
            .and_then(Value::as_f64)
            .unwrap();
        set_field(&mut altered, root, "offset", Value::Double(amount + 0.05));
        assert!(matches!(
            reconstruct(&altered, &mut store),
            Err(XtError::IntersectionCertificate { .. })
        ));
        assert_eq!(store_counts(&store), (0, 0, 0));
    }

    let mut unsafe_offset = fixture();
    set_field(&mut unsafe_offset, roots[1], "check", Value::Char('I'));
    assert!(matches!(
        reconstruct(&unsafe_offset, &mut store),
        Err(XtError::BadField { index, .. }) if index == roots[1]
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut crosslinked = fixture();
    set_field(&mut crosslinked, roots[0], "surface", Value::Ptr(roots[1]));
    let error = reconstruct(&crosslinked, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionSurfaceFamily)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut cyclic = fixture();
    set_field(&mut cyclic, roots[0], "surface", Value::Ptr(roots[1]));
    set_field(&mut cyclic, roots[1], "surface", Value::Ptr(roots[0]));
    assert!(matches!(
        reconstruct(&cyclic, &mut store),
        Err(XtError::SurfaceDependencyCycle { path })
            if path == vec![roots[0], roots[1], roots[0]]
    ));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut nonplane = fixture();
    let cylinder = nonplane.nodes.keys().next_back().copied().unwrap() + 1;
    let sense = nonplane
        .field(&nonplane.nodes[&roots[0]], "sense")
        .and_then(Value::as_char)
        .unwrap();
    nonplane.defs.insert(
        code::CYLINDER,
        kxt::schema::base_schema()
            .into_iter()
            .find(|definition| definition.code == code::CYLINDER)
            .unwrap(),
    );
    nonplane.nodes.insert(
        cylinder,
        Node {
            code: code::CYLINDER,
            values: vec![
                Value::Int(i64::from(cylinder)),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Char(sense),
                Value::Vector(Some([0.0, 0.0, 0.0])),
                Value::Vector(Some([0.0, 0.0, 1.0])),
                Value::Double(1.0),
                Value::Vector(Some([1.0, 0.0, 0.0])),
            ],
        },
    );
    set_field(&mut nonplane, roots[0], "surface", Value::Ptr(cylinder));
    let error = reconstruct(&nonplane, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionSurfaceFamily)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut parallel = fixture();
    let first_basis = offset_chain_terminal(&parallel, roots[0]);
    let second_basis = offset_chain_terminal(&parallel, roots[1]);
    for name in ["pvec", "normal", "x_axis"] {
        let value = parallel
            .field(&parallel.nodes[&first_basis], name)
            .unwrap()
            .clone();
        set_field(&mut parallel, second_basis, name, value);
    }
    let error = reconstruct(&parallel, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionSurfaceFamily)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut coincident = fixture();
    let first_basis = offset_chain_terminal(&coincident, roots[0]);
    let second_basis = offset_chain_terminal(&coincident, roots[1]);
    for name in ["pvec", "normal", "x_axis"] {
        let value = coincident
            .field(&coincident.nodes[&first_basis], name)
            .unwrap()
            .clone();
        set_field(&mut coincident, second_basis, name, value);
    }
    let signed_chain_distance = |file: &XtFile, root: u32| {
        let sense = file
            .field(&file.nodes[&root], "sense")
            .and_then(Value::as_char)
            .unwrap();
        let mut current = root;
        let mut distance = 0.0;
        while file.nodes[&current].code == code::OFFSET_SURF {
            distance += file
                .field(&file.nodes[&current], "offset")
                .and_then(Value::as_f64)
                .unwrap();
            current = ptr(file, current, "surface");
        }
        if sense == '+' { distance } else { -distance }
    };
    let target_distance = signed_chain_distance(&coincident, roots[0]);
    let second_sense = coincident
        .field(&coincident.nodes[&roots[1]], "sense")
        .and_then(Value::as_char)
        .unwrap();
    let transmitted_total = if second_sense == '+' {
        target_distance
    } else {
        -target_distance
    };
    let mut second_chain = Vec::new();
    let mut current = roots[1];
    while coincident.nodes[&current].code == code::OFFSET_SURF {
        second_chain.push(current);
        current = ptr(&coincident, current, "surface");
    }
    let per_offset = transmitted_total / second_chain.len() as f64;
    for offset in second_chain {
        set_field(&mut coincident, offset, "offset", Value::Double(per_offset));
    }
    let error = reconstruct(&coincident, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionSurfaceFamily)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let chart = ptr(&valid, intersection, "chart");
    let mut noncanonical = fixture();
    set_field(&mut noncanonical, chart, "base_scale", Value::Double(0.5));
    let error = reconstruct(&noncanonical, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionChartConvention)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut null_limit = fixture();
    set_field(&mut null_limit, intersection, "start", Value::Ptr(0));
    let error = reconstruct(&null_limit, &mut store).unwrap_err();
    assert_eq!(error.capability(), Some(XtCapability::IntersectionLimits));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let reconstruction = reconstruct(&valid, &mut store).unwrap();
    let mut fresh = Store::new();
    let fresh_reconstruction = reconstruct(&valid, &mut fresh).unwrap();
    assert_eq!(reconstruction.bodies, fresh_reconstruction.bodies);
    assert_eq!(reconstruction.journal, fresh_reconstruction.journal);
    assert_eq!(store_counts(&store), store_counts(&fresh));
}

#[test]
fn offset_wrong_unsafe_cyclic_and_nonplane_sources_fail_typed_and_atomically() {
    let valid = plane_offset_intersection_file(false, false);
    let offset = intersection_offset_source(&valid);

    let mut wrong = plane_offset_intersection_file(false, false);
    set_field(&mut wrong, offset, "offset", Value::Double(0.3));
    let mut store = Store::new();
    let error = reconstruct(&wrong, &mut store).unwrap_err();
    assert!(matches!(error, XtError::IntersectionCertificate { .. }));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut unsafe_offset = plane_offset_intersection_file(false, false);
    set_field(&mut unsafe_offset, offset, "check", Value::Char('I'));
    let error = reconstruct(&unsafe_offset, &mut store).unwrap_err();
    assert!(matches!(error, XtError::BadField { index, .. } if index == offset));
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut cyclic = plane_offset_intersection_file(false, false);
    set_field(&mut cyclic, offset, "surface", Value::Ptr(offset));
    let error = reconstruct(&cyclic, &mut store).unwrap_err();
    assert!(
        matches!(error, XtError::SurfaceDependencyCycle { path } if path == vec![offset, offset])
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut nonplane = plane_offset_intersection_file(false, false);
    let cylinder = nonplane.nodes.keys().next_back().copied().unwrap() + 1;
    let sense = nonplane
        .field(&nonplane.nodes[&offset], "sense")
        .and_then(Value::as_char)
        .unwrap();
    nonplane.defs.insert(
        code::CYLINDER,
        kxt::schema::base_schema()
            .into_iter()
            .find(|definition| definition.code == code::CYLINDER)
            .unwrap(),
    );
    nonplane.nodes.insert(
        cylinder,
        Node {
            code: code::CYLINDER,
            values: vec![
                Value::Int(i64::from(cylinder)),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Ptr(0),
                Value::Char(sense),
                Value::Vector(Some([0.0, 0.0, 0.0])),
                Value::Vector(Some([0.0, 0.0, 1.0])),
                Value::Double(1.0),
                Value::Vector(Some([1.0, 0.0, 0.0])),
            ],
        },
    );
    set_field(&mut nonplane, offset, "surface", Value::Ptr(cylinder));
    let error = reconstruct(&nonplane, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionSurfaceFamily),
        "unexpected non-plane-chain error: {error:?}"
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let mut two_offsets = plane_offset_intersection_file(false, false);
    let second_offset = intersection_offset_source(&two_offsets);
    let first_offset = two_offsets.nodes.keys().next_back().copied().unwrap() + 1;
    let first_offset_values = two_offsets.nodes[&second_offset].values.clone();
    two_offsets.nodes.insert(
        first_offset,
        Node {
            code: code::OFFSET_SURF,
            values: first_offset_values,
        },
    );
    set_field(
        &mut two_offsets,
        first_offset,
        "node_id",
        Value::Int(i64::from(first_offset)),
    );
    let intersection = two_offsets
        .nodes
        .iter()
        .find_map(|(&index, node)| (node.code == code::INTERSECTION).then_some(index))
        .unwrap();
    let mut sources = match two_offsets
        .field(&two_offsets.nodes[&intersection], "surface")
        .unwrap()
        .clone()
    {
        Value::Arr(values) => values,
        _ => unreachable!(),
    };
    sources[0] = Value::Ptr(first_offset);
    set_field(
        &mut two_offsets,
        intersection,
        "surface",
        Value::Arr(sources),
    );
    let error = reconstruct(&two_offsets, &mut store).unwrap_err();
    assert_eq!(
        error.capability(),
        Some(XtCapability::IntersectionSurfaceFamily)
    );
    assert_eq!(store_counts(&store), (0, 0, 0));

    let reconstruction = reconstruct(&valid, &mut store).unwrap();
    let mut fresh = Store::new();
    let fresh_reconstruction = reconstruct(&valid, &mut fresh).unwrap();
    assert_eq!(reconstruction.bodies, fresh_reconstruction.bodies);
    assert_eq!(store_counts(&store), store_counts(&fresh));
}
