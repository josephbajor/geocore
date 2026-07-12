//! Ownership, validation, limits, determinism, and clone contract tests.

use kcore::operation::{AccountingMode, ResourceKind};
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    EvalBudgetProfile, EvalContext, EvalError, EvalLimits, GeometryGraph, GeometryGraphError,
    GeometryRef, SurfaceDerivativeOrder, eval_stage,
};

fn line(origin: [f64; 3]) -> Line {
    Line::new(Vec3::from_array(origin), Vec3::new(1.0, 0.0, 0.0)).unwrap()
}

#[test]
fn stale_handles_fail_without_aliasing_reused_slots() {
    let mut graph = GeometryGraph::new();
    let stale = graph.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    graph.remove_curve(stale).unwrap();
    let replacement = graph.insert_curve(line([2.0, 0.0, 0.0])).unwrap();
    assert_ne!(stale, replacement);
    assert!(graph.curve(stale).is_none());

    let mut eval = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(
        eval.eval_curve(stale, 0.0, 0),
        Err(EvalError::StaleGeometryHandle {
            geometry: GeometryRef::Curve(stale)
        })
    );
    assert_eq!(
        graph.direct_dependencies(GeometryRef::Curve(stale)),
        Err(GeometryGraphError::StaleGeometryHandle {
            geometry: GeometryRef::Curve(stale)
        })
    );
}

#[test]
fn insertion_rejects_non_finite_leaf_state() {
    // `kgeom::Line` predates graph-boundary validation and accepts this
    // non-finite origin because its direction is valid.
    let non_finite = Line::new(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let mut graph = GeometryGraph::new();
    assert!(matches!(
        graph.insert_curve(non_finite),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert!(graph.is_empty());
}

#[test]
fn evaluation_validates_parameters_ranges_orders_and_resets_query_budget() {
    let mut graph = GeometryGraph::new();
    let curve = graph.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    let surface = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    let bounded = graph
        .insert_curve(
            NurbsCurve::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
                None,
            )
            .unwrap(),
        )
        .unwrap();

    let mut eval = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 1,
            max_node_visits_per_query: 1,
        },
        Tolerances::default(),
    );
    // Both succeed: node visits are per query, not cumulative across reuse.
    assert!(eval.eval_curve(curve, 0.0, 1).is_ok());
    assert!(eval.eval_curve(curve, 1.0, 1).is_ok());
    assert_eq!(
        eval.eval_curve(curve, f64::INFINITY, 0),
        Err(EvalError::InvalidParameter)
    );
    assert!(matches!(
        eval.eval_curve(curve, 0.0, 4),
        Err(EvalError::DerivativeUnavailable { requested: 4, .. })
    ));
    assert_eq!(
        eval.eval_surface(surface, [f64::NAN, 0.0], SurfaceDerivativeOrder::Position),
        Err(EvalError::InvalidParameter)
    );
    assert_eq!(
        eval.eval_curve(bounded, 2.0, 0),
        Err(EvalError::ParameterOutsideDomain)
    );
    assert_eq!(
        eval.curve_bounds(bounded, ParamRange::unbounded()),
        Err(EvalError::InvalidRange)
    );

    let mut no_visits = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 1,
            max_node_visits_per_query: 0,
        },
        Tolerances::default(),
    );
    assert_eq!(
        no_visits.eval_curve(curve, 0.0, 0),
        Err(EvalError::NodeVisitLimitExceeded {
            consumed: 1,
            limit: 0
        })
    );
    assert_eq!(no_visits.last_query_usage().node_visits(), 0);
    assert_eq!(no_visits.last_query_usage().dependency_depth(), 0);

    let mut no_depth = EvalContext::new(
        &graph,
        EvalLimits {
            max_dependency_depth: 0,
            max_node_visits_per_query: 1,
        },
        Tolerances::default(),
    );
    assert_eq!(
        no_depth.eval_curve(curve, 0.0, 0),
        Err(EvalError::DependencyDepthExceeded {
            consumed: 1,
            limit: 0
        })
    );
    assert_eq!(no_depth.last_query_usage().node_visits(), 1);
    assert_eq!(no_depth.last_query_usage().dependency_depth(), 0);
}

#[test]
fn evaluation_limits_round_trip_through_the_owned_f2_profile() {
    let defaults = EvalLimits::default();
    let plan = EvalBudgetProfile::v1_defaults();
    assert_eq!(EvalLimits::from_budget_plan(&plan).unwrap(), defaults);
    plan.require_limit(
        eval_stage::NODE_VISITS,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )
    .unwrap();
    plan.require_limit(
        eval_stage::DEPENDENCY_DEPTH,
        ResourceKind::Depth,
        AccountingMode::HighWater,
    )
    .unwrap();
}

#[test]
fn iteration_and_leaf_traversal_are_deterministic() {
    let mut graph = GeometryGraph::new();
    let c0 = graph.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    let c1 = graph.insert_curve(line([1.0, 0.0, 0.0])).unwrap();
    let s0 = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    let p0 = graph
        .insert_curve2d(Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap())
        .unwrap();

    let order: Vec<_> = graph.geometry().collect();
    assert_eq!(
        order,
        vec![
            GeometryRef::Curve(c0),
            GeometryRef::Curve(c1),
            GeometryRef::Surface(s0),
            GeometryRef::Curve2d(p0)
        ]
    );
    for geometry in order {
        assert!(graph.direct_dependencies(geometry).unwrap().is_empty());
        assert_eq!(graph.dependency_closure(geometry).unwrap(), vec![geometry]);
        assert!(graph.dependents(geometry).unwrap().is_empty());
        assert!(graph.reaches(geometry, geometry).unwrap());
        assert_eq!(
            graph.dependency_path(geometry, geometry).unwrap(),
            Some(vec![geometry])
        );
    }
    assert_eq!(
        graph
            .dependency_path(GeometryRef::Curve(c0), GeometryRef::Curve(c1))
            .unwrap(),
        None
    );
    graph.validate().unwrap();
}

#[test]
fn cloning_produces_an_independent_current_state_snapshot() {
    let mut source = GeometryGraph::new();
    let original = source.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    let mut cloned = source.clone();

    let extra = cloned.insert_curve(line([3.0, 0.0, 0.0])).unwrap();
    assert_eq!(source.curve_count(), 1);
    assert_eq!(cloned.curve_count(), 2);
    assert!(source.curve(extra).is_none());
    assert!(cloned.curve(extra).is_some());

    cloned.remove_curve(original).unwrap();
    assert!(cloned.curve(original).is_none());
    assert!(source.curve(original).is_some());
    source.validate().unwrap();
    cloned.validate().unwrap();
}

#[test]
fn replacement_is_validated_and_failed_replacement_is_atomic() {
    let mut graph = GeometryGraph::new();
    let curve = graph.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    let original = graph.curve(curve).unwrap().clone();
    let invalid = Line::new(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();

    assert!(matches!(
        graph.replace_curve(curve, invalid),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert_eq!(graph.curve(curve), Some(&original));
    graph.validate().unwrap();
}

#[test]
fn graph_undo_restores_payloads_handles_and_future_allocation_order() {
    let mut graph = GeometryGraph::new();
    let original = graph.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    let original_value = graph.curve(original).unwrap().clone();

    graph.begin_undo_frame();
    graph
        .replace_curve(original, line([5.0, 0.0, 0.0]))
        .unwrap();
    let transient = graph.insert_curve(line([9.0, 0.0, 0.0])).unwrap();
    assert_eq!(graph.pending_undo_frame_changes().unwrap().curves.len(), 2);
    graph.rollback_undo_frame().unwrap();

    assert_eq!(graph.curve(original), Some(&original_value));
    assert!(graph.curve(transient).is_none());
    let reused = graph.insert_curve(line([11.0, 0.0, 0.0])).unwrap();
    assert_eq!(reused, transient);
    graph.validate().unwrap();
}

#[test]
fn nested_graph_frames_restore_their_own_dependency_and_arena_state() {
    let mut graph = GeometryGraph::new();
    let curve = graph.insert_curve(line([0.0, 0.0, 0.0])).unwrap();
    let original = graph.curve(curve).unwrap().clone();

    graph.begin_undo_frame();
    graph.replace_curve(curve, line([1.0, 0.0, 0.0])).unwrap();
    let outer_value = graph.curve(curve).unwrap().clone();

    graph.begin_undo_frame();
    graph.replace_curve(curve, line([2.0, 0.0, 0.0])).unwrap();
    let transient = graph.insert_curve(line([3.0, 0.0, 0.0])).unwrap();
    graph.rollback_undo_frame().unwrap();
    assert_eq!(graph.curve(curve), Some(&outer_value));
    assert!(graph.curve(transient).is_none());

    graph.begin_undo_frame();
    graph.replace_curve(curve, line([4.0, 0.0, 0.0])).unwrap();
    graph.commit_undo_frame().unwrap();
    assert_ne!(graph.curve(curve), Some(&outer_value));

    graph.rollback_undo_frame().unwrap();
    assert_eq!(graph.curve(curve), Some(&original));
    let reused = graph.insert_curve(line([5.0, 0.0, 0.0])).unwrap();
    assert_eq!(reused, transient);
    graph.validate().unwrap();
}
