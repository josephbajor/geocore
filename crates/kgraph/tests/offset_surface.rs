//! Exact offset evaluation, dependency, validity, and failure-atomicity tests.

use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane, Sphere};
use kgeom::vec::Vec3;
use kgraph::{
    EvalContext, EvalError, EvalLimits, GeometryGraph, GeometryGraphError, GeometryRef,
    OffsetSurfaceDescriptor, SurfaceClass, SurfaceDerivativeOrder, SurfaceValidity,
};

fn evaluator(graph: &GeometryGraph) -> EvalContext<'_> {
    EvalContext::new(graph, EvalLimits::default(), Tolerances::default())
}

#[test]
fn plane_offset_has_exact_first_derivatives_metadata_and_conservative_bounds() {
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 2.5))
        .unwrap();
    let mut eval = evaluator(&graph);
    let value = eval
        .eval_surface(offset, [1.0, -3.0], SurfaceDerivativeOrder::First)
        .unwrap();
    assert_eq!(value.p, Vec3::new(1.0, -3.0, 2.5));
    assert_eq!(value.du, Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(value.dv, Vec3::new(0.0, 1.0, 0.0));
    assert_eq!(eval.surface_periodicity(offset).unwrap(), [None, None]);
    let range = [ParamRange::new(-1.0, 1.0), ParamRange::new(-2.0, 2.0)];
    let basis_box = eval.surface_bounds(basis, range).unwrap();
    assert_eq!(
        eval.surface_bounds(offset, range).unwrap(),
        basis_box.inflated(2.5)
    );
    assert!(matches!(
        eval.surface_validity(offset, [0.0, 0.0]).unwrap(),
        SurfaceValidity::Regular { .. }
    ));
    assert_eq!(
        eval.eval_surface(offset, [0.0, 0.0], SurfaceDerivativeOrder::Second),
        Err(EvalError::DerivativeUnavailable {
            class: graph.surface(offset).unwrap().class_key(),
            requested: 2
        })
    );
}

#[test]
fn cylinder_offsets_follow_both_signed_radial_directions() {
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(Cylinder::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let outward = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.5))
        .unwrap();
    let inward = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, -0.5))
        .unwrap();
    let mut eval = evaluator(&graph);

    let outward_value = eval
        .eval_surface(outward, [0.0, 3.0], SurfaceDerivativeOrder::First)
        .unwrap();
    assert_eq!(outward_value.p, Vec3::new(2.5, 0.0, 3.0));
    assert_eq!(outward_value.du, Vec3::new(0.0, 2.5, 0.0));
    assert_eq!(outward_value.dv, Vec3::new(0.0, 0.0, 1.0));

    let inward_value = eval
        .eval_surface(inward, [0.0, -1.0], SurfaceDerivativeOrder::First)
        .unwrap();
    assert_eq!(inward_value.p, Vec3::new(1.5, 0.0, -1.0));
    assert_eq!(inward_value.du, Vec3::new(0.0, 1.5, 0.0));
    assert_eq!(inward_value.dv, Vec3::new(0.0, 0.0, 1.0));
}

#[test]
fn inward_sphere_focal_offset_is_singular() {
    let mut graph = GeometryGraph::new();
    let sphere = graph
        .insert_surface(Sphere::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let collapsed = graph
        .insert_surface(OffsetSurfaceDescriptor::new(sphere, -2.0))
        .unwrap();
    let mut eval = evaluator(&graph);
    assert_eq!(
        eval.surface_validity(collapsed, [0.0, 0.0]).unwrap(),
        SurfaceValidity::Singular
    );
    assert_eq!(
        eval.eval_surface(collapsed, [0.0, 0.0], SurfaceDerivativeOrder::First),
        Err(EvalError::SingularSurface {
            surface: collapsed,
            uv: [0.0, 0.0]
        })
    );
}

#[test]
fn nested_offset_cannot_skip_a_singular_intermediate_surface() {
    let mut graph = GeometryGraph::new();
    let basis = graph
        .insert_surface(Cylinder::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let singular = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, -2.0))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(singular, 1.0))
        .unwrap();
    let mut eval = evaluator(&graph);
    assert_eq!(
        eval.eval_surface(nested, [0.0, 0.0], SurfaceDerivativeOrder::Position),
        Err(EvalError::SingularSurface {
            surface: singular,
            uv: [0.0, 0.0],
        })
    );
}

#[test]
fn offset_rejects_an_ill_conditioned_basis_before_normal_division() {
    let epsilon = 1.0e-12;
    let nurbs = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, epsilon, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, epsilon, 0.0),
        ],
        None,
    )
    .unwrap();
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(nurbs).unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 1.0))
        .unwrap();
    let mut eval = evaluator(&graph);
    assert_eq!(
        eval.eval_surface(offset, [0.5, 0.5], SurfaceDerivativeOrder::Position),
        Err(EvalError::IllConditionedSurface {
            surface: basis,
            uv: [0.5, 0.5],
        })
    );
}

#[test]
fn nurbs_basis_and_nested_shared_offsets_are_exact_and_bounded() {
    let nurbs = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap();
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(nurbs).unwrap();
    let first = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.25))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(first, 0.5))
        .unwrap();
    let sibling = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, -0.25))
        .unwrap();
    let mut eval = evaluator(&graph);
    let value = eval
        .eval_surface(nested, [0.4, 0.6], SurfaceDerivativeOrder::First)
        .unwrap();
    assert_eq!(value.p, Vec3::new(0.4, 0.6, 0.75));
    assert_eq!(
        graph
            .dependency_closure(GeometryRef::Surface(nested))
            .unwrap(),
        vec![
            GeometryRef::Surface(basis),
            GeometryRef::Surface(first),
            GeometryRef::Surface(nested)
        ]
    );
    assert_eq!(
        graph.dependents(GeometryRef::Surface(basis)).unwrap(),
        vec![GeometryRef::Surface(first), GeometryRef::Surface(sibling)]
    );
    let range = [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)];
    let nested_bounds = eval.surface_bounds(nested, range).unwrap();
    let expected_bounds = eval.surface_bounds(basis, range).unwrap().inflated(0.75);
    let rounding_slack = 4.0 * f64::EPSILON;
    assert!(nested_bounds.min.dist(expected_bounds.min) <= rounding_slack);
    assert!(nested_bounds.max.dist(expected_bounds.max) <= rounding_slack);
}

#[test]
fn offset_insertion_and_cycle_replacement_are_failure_atomic() {
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    let first = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 1.0))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(first, 1.0))
        .unwrap();
    let before = graph.surface(basis).unwrap().clone();
    assert!(matches!(
        graph.replace_surface(basis, OffsetSurfaceDescriptor::new(nested, 1.0)),
        Err(GeometryGraphError::DependencyCycle { path })
            if path == vec![
                GeometryRef::Surface(basis),
                GeometryRef::Surface(nested),
                GeometryRef::Surface(first),
                GeometryRef::Surface(basis)
            ]
    ));
    assert_eq!(graph.surface(basis), Some(&before));
    assert!(
        graph
            .insert_surface(OffsetSurfaceDescriptor::new(basis, f64::NAN))
            .is_err()
    );
    assert_eq!(graph.surface_count(), 3);
    graph.validate().unwrap();
}

#[test]
fn nested_offsets_charge_every_dependency_visit() {
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    let first = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 1.0))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(first, 1.0))
        .unwrap();
    let mut eval = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 8,
            max_node_visits_per_query: 2,
        },
        Tolerances::default(),
    );
    assert_eq!(
        eval.eval_surface(nested, [0.0, 0.0], SurfaceDerivativeOrder::First),
        Err(EvalError::NodeVisitLimitExceeded {
            consumed: 3,
            limit: 2
        })
    );
    assert_eq!(eval.last_query_usage().node_visits(), 2);
    assert_eq!(eval.last_query_usage().dependency_depth(), 2);
    let mut eval = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 2,
            max_node_visits_per_query: 8,
        },
        Tolerances::default(),
    );
    assert_eq!(
        eval.eval_surface(nested, [0.0, 0.0], SurfaceDerivativeOrder::First),
        Err(EvalError::DependencyDepthExceeded {
            consumed: 3,
            limit: 2
        })
    );
    assert_eq!(eval.last_query_usage().node_visits(), 3);
    assert_eq!(eval.last_query_usage().dependency_depth(), 2);
}

#[test]
fn surface_leaf_class_is_accounted_and_preserves_exact_limit_evidence() {
    let mut graph = GeometryGraph::new();
    let plane = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    let first = graph
        .insert_surface(OffsetSurfaceDescriptor::new(plane, 1.0))
        .unwrap();
    let nested = graph
        .insert_surface(OffsetSurfaceDescriptor::new(first, 2.0))
        .unwrap();

    let mut eval = evaluator(&graph);
    assert_eq!(eval.surface_leaf_class(plane), Ok(SurfaceClass::Plane));
    assert_eq!(eval.last_query_usage().node_visits(), 1);
    assert_eq!(eval.last_query_usage().dependency_depth(), 1);
    assert_eq!(eval.surface_leaf_class(nested), Ok(SurfaceClass::Plane));
    assert_eq!(eval.last_query_usage().node_visits(), 3);
    assert_eq!(eval.last_query_usage().dependency_depth(), 3);

    let mut visit_limited = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 3,
            max_node_visits_per_query: 2,
        },
        Tolerances::default(),
    );
    assert_eq!(
        visit_limited.surface_leaf_class(nested),
        Err(EvalError::NodeVisitLimitExceeded {
            consumed: 3,
            limit: 2,
        })
    );
    assert_eq!(visit_limited.last_query_usage().node_visits(), 2);
    assert_eq!(visit_limited.last_query_usage().dependency_depth(), 2);

    let mut depth_limited = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 2,
            max_node_visits_per_query: 3,
        },
        Tolerances::default(),
    );
    assert_eq!(
        depth_limited.surface_leaf_class(nested),
        Err(EvalError::DependencyDepthExceeded {
            consumed: 3,
            limit: 2,
        })
    );
    assert_eq!(depth_limited.last_query_usage().node_visits(), 3);
    assert_eq!(depth_limited.last_query_usage().dependency_depth(), 2);

    let stale = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    graph.remove_surface(stale).unwrap();
    let mut stale_eval = evaluator(&graph);
    assert_eq!(
        stale_eval.surface_leaf_class(stale),
        Err(EvalError::StaleGeometryHandle {
            geometry: GeometryRef::Surface(stale),
        })
    );
    assert_eq!(stale_eval.last_query_usage().node_visits(), 1);
    assert_eq!(stale_eval.last_query_usage().dependency_depth(), 1);
}
