//! Graph leaf classes delegate exactly to their `kgeom` evaluator contracts.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::{Circle2d, Curve2d, Line2d, NurbsCurve2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    Curve2dClass, Curve2dHandle, CurveClass, CurveHandle, EvalContext, EvalLimits, GeometryGraph,
    SurfaceClass, SurfaceDerivativeOrder, SurfaceHandle,
};

fn context(graph: &GeometryGraph) -> EvalContext<'_> {
    EvalContext::new(graph, EvalLimits::default(), Tolerances::default())
}

fn assert_curve_parity(
    graph: &GeometryGraph,
    handle: CurveHandle,
    leaf: &dyn Curve,
    t: f64,
    range: ParamRange,
) {
    let mut eval = context(graph);
    assert_eq!(
        eval.eval_curve(handle, t, 3).unwrap(),
        leaf.eval_derivs(t, 3)
    );
    assert_eq!(eval.curve_param_range(handle).unwrap(), leaf.param_range());
    assert_eq!(eval.curve_periodicity(handle).unwrap(), leaf.periodicity());
    assert_eq!(
        eval.curve_bounds(handle, range).unwrap(),
        leaf.bounding_box(range)
    );
}

fn assert_curve2d_parity(
    graph: &GeometryGraph,
    handle: Curve2dHandle,
    leaf: &dyn Curve2d,
    t: f64,
    range: ParamRange,
) {
    let mut eval = context(graph);
    assert_eq!(
        eval.eval_curve2d(handle, t, 3).unwrap(),
        leaf.eval_derivs(t, 3)
    );
    assert_eq!(
        eval.curve2d_param_range(handle).unwrap(),
        leaf.param_range()
    );
    assert_eq!(
        eval.curve2d_periodicity(handle).unwrap(),
        leaf.periodicity()
    );
    assert_eq!(
        eval.curve2d_bounds(handle, range).unwrap(),
        leaf.bounding_box(range)
    );
}

fn assert_surface_parity(
    graph: &GeometryGraph,
    handle: SurfaceHandle,
    leaf: &dyn Surface,
    uv: [f64; 2],
    range: [ParamRange; 2],
) {
    let mut eval = context(graph);
    assert_eq!(
        eval.eval_surface(handle, uv, SurfaceDerivativeOrder::Second)
            .unwrap(),
        leaf.eval_derivs(uv, 2)
    );
    assert_eq!(
        eval.surface_param_range(handle).unwrap(),
        leaf.param_range()
    );
    assert_eq!(
        eval.surface_periodicity(handle).unwrap(),
        leaf.periodicity()
    );
    assert_eq!(
        eval.surface_degeneracies(handle).unwrap(),
        leaf.degeneracies()
    );
    assert_eq!(
        eval.surface_bounds(handle, range).unwrap(),
        leaf.bounding_box(range)
    );
}

#[test]
fn every_curve_leaf_has_evaluation_and_metadata_parity() {
    let frame = Frame::world();
    let line = Line::new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(1.0, -2.0, 0.5)).unwrap();
    let circle = Circle::new(frame, 2.0).unwrap();
    let ellipse = Ellipse::new(frame, 3.0, 1.5).unwrap();
    let nurbs = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 2.0, 0.0),
            Vec3::new(3.0, 1.0, 0.0),
        ],
        Some(vec![1.0, 0.75, 1.0]),
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let h_line = graph.insert_curve(line).unwrap();
    let h_circle = graph.insert_curve(circle).unwrap();
    let h_ellipse = graph.insert_curve(ellipse).unwrap();
    let h_nurbs = graph.insert_curve(nurbs.clone()).unwrap();

    assert_curve_parity(&graph, h_line, &line, 0.3, ParamRange::new(-2.0, 4.0));
    assert_curve_parity(&graph, h_circle, &circle, 0.3, ParamRange::new(0.1, 4.0));
    assert_curve_parity(&graph, h_ellipse, &ellipse, 0.3, ParamRange::new(0.1, 4.0));
    assert_curve_parity(&graph, h_nurbs, &nurbs, 0.3, ParamRange::new(0.1, 0.9));

    let classes: Vec<_> = graph
        .curves()
        .map(|(_, n)| (n.descriptor().class(), n.descriptor().class_key().as_str()))
        .collect();
    assert_eq!(
        classes,
        vec![
            (CurveClass::Line, "kernel.curve.line.v1"),
            (CurveClass::Circle, "kernel.curve.circle.v1"),
            (CurveClass::Ellipse, "kernel.curve.ellipse.v1"),
            (CurveClass::Nurbs, "kernel.curve.nurbs.v1"),
        ]
    );
    assert!(
        graph
            .curve(h_line)
            .unwrap()
            .descriptor()
            .as_line()
            .is_some()
    );
    assert!(
        graph
            .curve(h_nurbs)
            .unwrap()
            .descriptor()
            .as_nurbs()
            .is_some()
    );
}

#[test]
fn every_surface_leaf_has_evaluation_and_metadata_parity() {
    let frame = Frame::world();
    let plane = Plane::new(frame);
    let cylinder = Cylinder::new(frame, 2.0).unwrap();
    let cone = Cone::new(frame, 2.0, 0.4).unwrap();
    let sphere = Sphere::new(frame, 2.0).unwrap();
    let torus = Torus::new(frame, 3.0, 1.0).unwrap();
    let nurbs = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.5),
            Vec3::new(1.0, 0.0, 0.25),
            Vec3::new(1.0, 1.0, 1.0),
        ],
        Some(vec![1.0, 0.8, 1.2, 1.0]),
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let h_plane = graph.insert_surface(plane).unwrap();
    let h_cylinder = graph.insert_surface(cylinder).unwrap();
    let h_cone = graph.insert_surface(cone).unwrap();
    let h_sphere = graph.insert_surface(sphere).unwrap();
    let h_torus = graph.insert_surface(torus).unwrap();
    let h_nurbs = graph.insert_surface(nurbs.clone()).unwrap();

    assert_surface_parity(
        &graph,
        h_plane,
        &plane,
        [0.3, 0.2],
        [ParamRange::new(-1.0, 2.0), ParamRange::new(-2.0, 1.0)],
    );
    assert_surface_parity(
        &graph,
        h_cylinder,
        &cylinder,
        [0.3, 0.2],
        [ParamRange::new(0.1, 4.0), ParamRange::new(-2.0, 1.0)],
    );
    assert_surface_parity(
        &graph,
        h_cone,
        &cone,
        [0.3, 0.2],
        [ParamRange::new(0.1, 4.0), ParamRange::new(-0.5, 1.0)],
    );
    assert_surface_parity(
        &graph,
        h_sphere,
        &sphere,
        [0.3, 0.2],
        [ParamRange::new(0.1, 4.0), ParamRange::new(-1.0, 1.0)],
    );
    assert_surface_parity(
        &graph,
        h_torus,
        &torus,
        [0.3, 0.2],
        [ParamRange::new(0.1, 4.0), ParamRange::new(0.2, 5.0)],
    );
    assert_surface_parity(
        &graph,
        h_nurbs,
        &nurbs,
        [0.3, 0.2],
        [ParamRange::new(0.1, 0.9), ParamRange::new(0.1, 0.8)],
    );

    let classes: Vec<_> = graph
        .surfaces()
        .map(|(_, n)| (n.descriptor().class(), n.descriptor().class_key().as_str()))
        .collect();
    assert_eq!(
        classes,
        vec![
            (SurfaceClass::Plane, "kernel.surface.plane.v1"),
            (SurfaceClass::Cylinder, "kernel.surface.cylinder.v1"),
            (SurfaceClass::Cone, "kernel.surface.cone.v1"),
            (SurfaceClass::Sphere, "kernel.surface.sphere.v1"),
            (SurfaceClass::Torus, "kernel.surface.torus.v1"),
            (SurfaceClass::Nurbs, "kernel.surface.nurbs.v1")
        ]
    );
    assert!(
        graph
            .surface(h_plane)
            .unwrap()
            .descriptor()
            .as_plane()
            .is_some()
    );
    assert!(
        graph
            .surface(h_nurbs)
            .unwrap()
            .descriptor()
            .as_nurbs()
            .is_some()
    );
}

#[test]
fn every_curve2d_leaf_has_evaluation_and_metadata_parity() {
    let line = Line2d::new(Vec2::new(1.0, 2.0), Vec2::new(1.0, -2.0)).unwrap();
    let circle = Circle2d::new(Vec2::new(0.5, -0.5), 2.0, Vec2::new(1.0, 1.0)).unwrap();
    let nurbs = NurbsCurve2d::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 2.0),
            Vec2::new(3.0, 1.0),
        ],
        Some(vec![1.0, 0.75, 1.0]),
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let h_line = graph.insert_curve2d(line).unwrap();
    let h_circle = graph.insert_curve2d(circle).unwrap();
    let h_nurbs = graph.insert_curve2d(nurbs.clone()).unwrap();

    assert_curve2d_parity(&graph, h_line, &line, 0.3, ParamRange::new(-2.0, 4.0));
    assert_curve2d_parity(&graph, h_circle, &circle, 0.3, ParamRange::new(0.1, 4.0));
    assert_curve2d_parity(&graph, h_nurbs, &nurbs, 0.3, ParamRange::new(0.1, 0.9));

    let classes: Vec<_> = graph
        .curves_2d()
        .map(|(_, n)| (n.descriptor().class(), n.descriptor().class_key().as_str()))
        .collect();
    assert_eq!(
        classes,
        vec![
            (Curve2dClass::Line, "kernel.curve2d.line.v1"),
            (Curve2dClass::Circle, "kernel.curve2d.circle.v1"),
            (Curve2dClass::Nurbs, "kernel.curve2d.nurbs.v1")
        ]
    );
    assert!(
        graph
            .curve2d(h_line)
            .unwrap()
            .descriptor()
            .as_line()
            .is_some()
    );
    assert!(
        graph
            .curve2d(h_nurbs)
            .unwrap()
            .descriptor()
            .as_nurbs()
            .is_some()
    );
}
