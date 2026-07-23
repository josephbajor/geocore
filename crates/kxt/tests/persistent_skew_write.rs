//! Parsed writer invariants for proof-bearing bounded skew-cylinder edges.

use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point2, Vec3};
use kgraph::{
    PersistentSkewCylinderOpenSpanCertificate, PersistentSkewCylinderOpenSpanOrientation,
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderBranchPcurveEnclosure, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_open_span,
};
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellSkewCylinderOpenSpan, AnalyticShellSurface,
};
use ktopo::entity::{BodyId, FaceDomain, PcurveChart, Sense};
use ktopo::geom::Curve2dGeom;
use ktopo::store::Store;
use kxt::schema::code;
use kxt::{Node, Value, XtFile};

#[derive(Debug)]
struct Segment {
    lo: f64,
    hi: f64,
    enclosure: SkewCylinderBranchPcurveEnclosure,
}

#[derive(Debug)]
struct ExpectedTransport {
    knots: Vec<f64>,
    poles: Vec<f64>,
    lift_error: f64,
}

fn certificate() -> PersistentSkewCylinderOpenSpanCertificate {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Vec3::default(),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let cylinders = [first, second];
    let ranges = [
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(1.8, 2.1),
        ],
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.25, 0.0),
        ],
    ];
    let roots = [2.082_769_014_844_373_6, 4.200_416_292_335_213];
    let mut guarded = ParamRange::new(roots[0], roots[1]);
    for _ in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        guarded.lo = guarded.lo.next_up();
        guarded.hi = guarded.hi.next_down();
    }
    let residual = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        guarded,
        SkewCylinderSheet::Upper,
        LINEAR_RESOLUTION,
    )
    .unwrap();
    let root_intervals = roots.map(|root| Interval::new(root.next_down(), root.next_up()));
    let corridors = [
        residual
            .certify_lower_pcurve_root_corridor(root_intervals[0])
            .unwrap(),
        residual
            .certify_upper_pcurve_root_corridor(root_intervals[1])
            .unwrap(),
    ];
    let endpoints = roots.map(|parameter| {
        let (sine, cosine) = math::sincos(parameter);
        Vec3::new(cosine, sine, (4.0 - sine * sine).sqrt())
    });
    certify_persistent_skew_cylinder_open_span(
        residual,
        corridors,
        endpoints,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    )
    .unwrap()
}

fn pcurve_domain(
    certificate: PersistentSkewCylinderOpenSpanCertificate,
    operand: usize,
    chart: PcurveChart,
) -> FaceDomain {
    let pcurve = certificate.pcurves()[operand];
    let cylinder = certificate.carrier().cylinders()[operand];
    let bounds = pcurve.bounding_box(ParamRange::new(0.0, 1.0));
    let periods = cylinder.periodicity();
    let min = chart.apply(bounds.min, periods).unwrap();
    let max = chart.apply(bounds.max, periods).unwrap();
    FaceDomain::from_bounds(min.x, max.x, min.y, max.y).unwrap()
}

fn persistent_body(certificate: PersistentSkewCylinderOpenSpanCertificate) -> (Store, BodyId) {
    let edge_keys = [AnalyticEdgeKey::new(0), AnalyticEdgeKey::new(1)];
    let vertex_keys = [
        ktopo::analytic_shell::AnalyticVertexKey::new(0),
        ktopo::analytic_shell::AnalyticVertexKey::new(1),
    ];
    let spans = edge_keys
        .map(|edge| AnalyticShellSkewCylinderOpenSpan::new(edge, vertex_keys, certificate));
    let cylinders = certificate.carrier().cylinders();
    let charts = [PcurveChart::identity(), PcurveChart::shifted([1, 0])];
    let uses = [
        spans[0].pcurves()[0].with_chart(charts[0]),
        spans[0].pcurves()[1].with_chart(charts[1]),
    ];
    let faces = vec![
        AnalyticShellFace::new(
            AnalyticFaceKey::new(0),
            AnalyticShellSurface::Cylinder(cylinders[0]),
            Sense::Forward,
            pcurve_domain(certificate, 0, charts[0]),
            vec![AnalyticShellLoop::new(vec![
                AnalyticShellFin::new(edge_keys[0], Sense::Forward, uses[0]),
                AnalyticShellFin::new(edge_keys[1], Sense::Reversed, uses[0]),
            ])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(1),
            AnalyticShellSurface::Cylinder(cylinders[1]),
            Sense::Forward,
            pcurve_domain(certificate, 1, charts[1]),
            vec![AnalyticShellLoop::new(vec![
                AnalyticShellFin::new(edge_keys[0], Sense::Reversed, uses[1]),
                AnalyticShellFin::new(edge_keys[1], Sense::Forward, uses[1]),
            ])],
        ),
    ];
    let input = AnalyticShellInput::new(
        spans[0].vertices().to_vec(),
        spans.into_iter().map(|span| span.edge()).collect(),
        faces,
    );
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let body = transaction
        .assemble_analytic_shell(&input, LINEAR_RESOLUTION)
        .unwrap()
        .body();
    transaction.commit_checked_body(body).unwrap();
    (store, body)
}

fn proof_segments(
    certificate: PersistentSkewCylinderOpenSpanCertificate,
    operand: usize,
) -> Vec<Segment> {
    let residual = certificate.residual_certificate();
    let guarded = residual.carrier_range();
    let corridors = certificate.root_corridors();
    let roots = corridors.map(|corridor| {
        let root = corridor.root_parameter();
        0.5 * root.lo() + 0.5 * root.hi()
    });
    let mut segments = Vec::with_capacity(SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS + 2);
    segments.push(Segment {
        lo: roots[0],
        hi: guarded.lo,
        enclosure: corridors[0].corridor().pcurves()[operand],
    });
    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let cell = residual.certify_pcurve_cell(index).unwrap();
        segments.push(Segment {
            lo: cell.parameter().lo(),
            hi: cell.parameter().hi(),
            enclosure: cell.pcurves()[operand],
        });
    }
    segments.push(Segment {
        lo: guarded.hi,
        hi: roots[1],
        enclosure: corridors[1].corridor().pcurves()[operand],
    });
    segments.retain(|segment| segment.lo < segment.hi);
    segments
}

fn expected_transport(
    certificate: PersistentSkewCylinderOpenSpanCertificate,
    operand: usize,
    chart: PcurveChart,
) -> ExpectedTransport {
    let segments = proof_segments(certificate, operand);
    let root_parameters = certificate.root_corridors().map(|corridor| {
        let root = corridor.root_parameter();
        0.5 * root.lo() + 0.5 * root.hi()
    });
    let logical =
        |canonical| (canonical - root_parameters[0]) / (root_parameters[1] - root_parameters[0]);
    let mut knots = Vec::with_capacity(segments.len() + 1);
    knots.push(0.0);
    for (index, segment) in segments.iter().enumerate() {
        knots.push(if index + 1 == segments.len() {
            1.0
        } else {
            logical(segment.hi)
        });
    }

    let cylinder = certificate.carrier().cylinders()[operand];
    let pcurve = certificate.pcurves()[operand];
    let points = knots
        .iter()
        .map(|&parameter| {
            chart
                .apply(pcurve.eval(parameter), cylinder.periodicity())
                .unwrap()
        })
        .collect::<Vec<Point2>>();
    let mut lift_error = 0.0_f64;
    for (index, segment) in segments.iter().enumerate() {
        let span = Interval::point(segment.hi) - Interval::point(segment.lo);
        let derivative = segment.enclosure.stored_derivative();
        let error = derivative
            .map(|range| span * (Interval::point(range.hi()) - Interval::point(range.lo())));
        let local = (Interval::point(cylinder.radius()) * error[0] + error[1]).hi();
        lift_error = lift_error.max(local);

        for fraction in [0.25, 0.5, 0.75] {
            let parameter = knots[index] + fraction * (knots[index + 1] - knots[index]);
            let exact_uv = chart
                .apply(pcurve.eval(parameter), cylinder.periodicity())
                .unwrap();
            let chord_uv = points[index] + (points[index + 1] - points[index]) * fraction;
            let exact = cylinder.eval([exact_uv.x, exact_uv.y]);
            let chord = cylinder.eval([chord_uv.x, chord_uv.y]);
            assert!((exact - chord).norm() <= local + LINEAR_RESOLUTION);
        }
    }
    ExpectedTransport {
        knots,
        poles: points
            .into_iter()
            .flat_map(|point| [point.x, point.y])
            .collect(),
        lift_error,
    }
}

fn array<'a>(file: &'a XtFile, node: &'a Node, field: &str) -> &'a [Value] {
    match file.field(node, field) {
        Some(Value::Arr(values)) => values,
        value => panic!("expected array field {field}, got {value:?}"),
    }
}

fn pointed<'a>(file: &'a XtFile, node: &Node, field: &str) -> &'a Node {
    let index = file
        .field(node, field)
        .and_then(Value::as_ptr)
        .unwrap_or_else(|| panic!("missing pointer field {field}"));
    file.node(index)
        .unwrap_or_else(|| panic!("dangling pointer field {field}"))
}

#[test]
fn persistent_skew_edges_emit_certificate_partitioned_degree_one_sp_curves() {
    let certificate = certificate();
    let expected = [
        expected_transport(certificate, 0, PcurveChart::identity()),
        expected_transport(certificate, 1, PcurveChart::shifted([1, 0])),
    ];
    assert_eq!(
        expected[0].knots.len(),
        SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS + 3
    );
    assert_eq!(expected[0].knots, expected[1].knots);
    let residuals = certificate.residual_bounds();
    let paired = Interval::point(expected[0].lift_error)
        + Interval::point(expected[1].lift_error)
        + Interval::point(residuals[0])
        + Interval::point(residuals[1])
        + Interval::point(LINEAR_RESOLUTION);
    let expected_tolerance = certificate
        .required_edge_tolerance()
        .max(LINEAR_RESOLUTION)
        .max(paired.hi());

    let (store, body) = persistent_body(certificate);
    let text = kxt::export_text(&store, body).unwrap();
    assert_eq!(text, kxt::export_text(&store, body).unwrap());
    let file = kxt::read_xt(text.as_bytes()).unwrap();
    let edges = file
        .nodes
        .values()
        .filter(|node| node.code == code::EDGE)
        .collect::<Vec<_>>();
    assert_eq!(edges.len(), 2);
    for edge in edges {
        assert_eq!(file.field(edge, "curve").and_then(Value::as_ptr), Some(0));
        assert_eq!(
            file.field(edge, "tolerance")
                .and_then(Value::as_f64)
                .map(f64::to_bits),
            Some(expected_tolerance.to_bits())
        );
    }

    let sp_curves = file
        .nodes
        .values()
        .filter(|node| node.code == code::SP_CURVE)
        .collect::<Vec<_>>();
    assert_eq!(sp_curves.len(), 4);
    let mut operands = [0_usize; 2];
    let expected_multiplicities = (0..expected[0].knots.len())
        .map(|index| {
            if index == 0 || index + 1 == expected[0].knots.len() {
                2_i64
            } else {
                1_i64
            }
        })
        .collect::<Vec<_>>();
    for sp_curve in sp_curves {
        let b_curve = pointed(&file, sp_curve, "b_curve");
        assert_eq!(b_curve.code, code::B_CURVE);
        let nurbs = pointed(&file, b_curve, "nurbs");
        assert_eq!(nurbs.code, code::NURBS_CURVE);
        assert_eq!(file.field(nurbs, "degree").and_then(Value::as_int), Some(1));
        assert_eq!(
            file.field(nurbs, "vertex_dim").and_then(Value::as_int),
            Some(2)
        );
        assert_eq!(
            file.field(nurbs, "n_vertices").and_then(Value::as_int),
            Some(i64::try_from(expected[0].knots.len()).unwrap())
        );
        assert_eq!(
            file.field(nurbs, "n_knots").and_then(Value::as_int),
            Some(i64::try_from(expected[0].knots.len()).unwrap())
        );
        assert!(matches!(
            file.field(nurbs, "rational"),
            Some(Value::Logical(false))
        ));

        let poles = array(&file, pointed(&file, nurbs, "bspline_vertices"), "vertices")
            .iter()
            .map(|value| value.as_f64().unwrap())
            .collect::<Vec<_>>();
        let knots = array(&file, pointed(&file, nurbs, "knots"), "knots")
            .iter()
            .map(|value| value.as_f64().unwrap())
            .collect::<Vec<_>>();
        let multiplicities = array(&file, pointed(&file, nurbs, "knot_mult"), "mult")
            .iter()
            .map(|value| value.as_int().unwrap())
            .collect::<Vec<_>>();
        assert_eq!(knots, expected[0].knots);
        assert_eq!(multiplicities, expected_multiplicities);
        let operand = expected
            .iter()
            .position(|transport| transport.poles == poles)
            .expect("emitted poles equal one certificate pcurve with its chart shift baked in");
        operands[operand] += 1;
    }
    assert_eq!(operands, [2, 2]);

    let mut imported = Store::new();
    let reconstruction = kxt::import(text.as_bytes(), &mut imported).unwrap();
    assert_eq!(reconstruction.bodies.len(), 1);
    let imported_edges = imported.edges_of_body(reconstruction.bodies[0]).unwrap();
    assert_eq!(imported_edges.len(), 2);
    for edge_id in imported_edges {
        let edge = imported.get(edge_id).unwrap();
        assert!(edge.curve().is_none());
        assert_eq!(edge.fins().len(), 2);
        assert_eq!(
            edge.tolerance()
                .map(|tolerance| tolerance.value().to_bits()),
            Some(expected_tolerance.to_bits())
        );
        for &fin_id in edge.fins() {
            let pcurve = imported.get(fin_id).unwrap().pcurve().unwrap();
            let Curve2dGeom::Nurbs(nurbs) = imported.get(pcurve.curve()).unwrap() else {
                panic!("imported SP_CURVE must reconstruct as a NURBS pcurve")
            };
            assert_eq!(nurbs.degree(), 1);
            assert_eq!(nurbs.points().len(), expected[0].knots.len());
        }
    }
}
