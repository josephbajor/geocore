//! Focused coverage for shared point-to-surface services.

use core::f64::consts::{FRAC_PI_2, TAU};

use kcore::error::ErrorClass;
use kcore::operation::{
    AccountingMode, BudgetPlan, ExecutionPolicy, LimitSpec, NumericalPolicy, OperationContext,
    OperationPolicyError, OperationScope, PolicyVersion, ResourceKind, SessionPolicy,
    SessionPrecision, TOTAL_WORK_STAGE,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::project::{
    ProjectionBudgetProfile, ProjectionError, SURFACE_PROJECTION_QUERIES,
    SURFACE_PROJECTION_SAMPLES,
};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use kgeom::surface_point::{
    SurfacePointContextError, SurfacePointMethod, capability, distance_to_surface,
    distance_to_surface_in_scope, distance_to_surface_with_context, invert_surface_point,
    invert_surface_point_in_scope, invert_surface_point_with_context, normalize_surface_uv,
};
use kgeom::vec::{Point3, Vec3};

fn tilted_frame() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:?} differs from {expected:?} by more than {tolerance:?}"
    );
}

fn session(budget: BudgetPlan) -> SessionPolicy {
    SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        budget,
        PolicyVersion::V1,
    )
}

fn override_limit(
    stage: kcore::operation::StageId,
    resource: ResourceKind,
    mode: AccountingMode,
    allowed: u64,
) -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap()
}

fn nurbs_plane() -> NurbsSurface {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    NurbsSurface::new(
        1,
        1,
        knots.clone(),
        knots,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
        ],
        None,
    )
    .unwrap()
}

fn assert_analytic_surface(
    surface: &dyn Surface,
    uv: [f64; 2],
    expected_raw: [f64; 2],
    expected_base: [f64; 2],
) {
    let point = surface.eval(uv);
    let mapped = invert_surface_point(surface, point).unwrap();
    assert_eq!(mapped.method, SurfacePointMethod::Analytic);
    assert_close(mapped.uv[0], expected_raw[0], 2.0e-12);
    assert_close(mapped.uv[1], expected_raw[1], 2.0e-12);

    let base = normalize_surface_uv(surface, mapped.uv);
    assert_close(base[0], expected_base[0], 2.0e-12);
    assert_close(base[1], expected_base[1], 2.0e-12);

    let on_surface = distance_to_surface(surface, point).unwrap();
    assert_eq!(on_surface.method, SurfacePointMethod::Analytic);
    assert_close(on_surface.distance, 0.0, 2.0e-12);

    let normal = surface.normal(uv).unwrap();
    let offset = distance_to_surface(surface, point + normal * 0.25).unwrap();
    assert_eq!(offset.method, SurfacePointMethod::Analytic);
    assert_close(offset.distance, 0.25, 2.0e-12);
}

#[test]
fn analytic_classes_share_raw_inversion_normalization_and_distance_semantics() {
    let frame = tilted_frame();
    let plane = Plane::new(frame);
    assert_analytic_surface(&plane, [-1.2, 2.3], [-1.2, 2.3], [-1.2, 2.3]);

    let cylinder = Cylinder::new(frame, 1.7).unwrap();
    assert_analytic_surface(&cylinder, [-0.25, 1.1], [-0.25, 1.1], [TAU - 0.25, 1.1]);

    let cone = Cone::new(frame, 1.4, 0.35).unwrap();
    assert_analytic_surface(&cone, [-0.4, 0.7], [-0.4, 0.7], [TAU - 0.4, 0.7]);

    let sphere = Sphere::new(frame, 2.2).unwrap();
    assert_analytic_surface(&sphere, [-0.3, 0.4], [-0.3, 0.4], [TAU - 0.3, 0.4]);

    let torus = Torus::new(frame, 3.0, 0.8).unwrap();
    assert_analytic_surface(&torus, [-0.2, -0.6], [-0.2, -0.6], [TAU - 0.2, TAU - 0.6]);
}

#[test]
fn periodic_seams_and_analytic_singularities_are_deterministic() {
    let frame = Frame::world();
    let cylinder = Cylinder::new(frame, 2.0).unwrap();
    let near_seam = invert_surface_point(&cylinder, cylinder.eval([-1.0e-8, 0.75])).unwrap();
    assert!(near_seam.uv[0] < 0.0);
    let normalized = normalize_surface_uv(&cylinder, near_seam.uv);
    assert_close(normalized[0], TAU - 1.0e-8, 2.0e-12);

    let sphere = Sphere::new(frame, 1.5).unwrap();
    let north = frame.origin() + frame.z() * sphere.radius();
    let north_uv = invert_surface_point(&sphere, north).unwrap();
    assert_eq!(north_uv.method, SurfacePointMethod::Analytic);
    assert_close(north_uv.uv[1], FRAC_PI_2, 1.0e-15);
    assert_eq!(normalize_surface_uv(&sphere, north_uv.uv), north_uv.uv);

    let cone = Cone::new(frame, 1.25, 0.4).unwrap();
    let apex_uv = invert_surface_point(&cone, cone.apex()).unwrap();
    assert_eq!(apex_uv.method, SurfacePointMethod::Analytic);
    assert_close(apex_uv.uv[1], cone.apex_v(), 2.0e-12);
    let reconstructed = cone.eval(normalize_surface_uv(&cone, apex_uv.uv));
    assert_close(reconstructed.dist(cone.apex()), 0.0, 2.0e-12);
}

#[test]
fn nurbs_uses_finite_domain_projection_for_uv_and_distance() {
    let surface = nurbs_plane();

    let point = surface.eval([0.3, 0.7]);
    let mapped = invert_surface_point(&surface, point).unwrap();
    assert_eq!(mapped.method, SurfacePointMethod::Projected);
    assert_close(mapped.uv[0], 0.3, 2.0e-10);
    assert_close(mapped.uv[1], 0.7, 2.0e-10);
    assert_eq!(normalize_surface_uv(&surface, mapped.uv), mapped.uv);

    let distance = distance_to_surface(&surface, point + Vec3::new(0.0, 0.0, 0.5)).unwrap();
    assert_eq!(distance.method, SurfacePointMethod::Projected);
    assert_close(distance.distance, 0.5, 2.0e-10);
}

#[test]
fn contextual_analytic_queries_are_bit_equivalent_and_consume_zero_projection_work() {
    let plane = Plane::new(tilted_frame());
    let point = Point3::new(0.25, -0.5, 3.0);
    let budget = override_limit(
        SURFACE_PROJECTION_QUERIES,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        3,
    );
    let policy = session(budget);
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();

    let legacy_uv = invert_surface_point(&plane, point).unwrap();
    let contextual_uv = invert_surface_point_with_context(&plane, point, &context).unwrap();
    assert_eq!(contextual_uv.result(), Ok(&legacy_uv));
    assert_eq!(contextual_uv.report().usage().len(), 1);
    assert_eq!(contextual_uv.report().usage()[0].consumed, 0);

    let legacy_distance = distance_to_surface(&plane, point).unwrap();
    let contextual_distance = distance_to_surface_with_context(&plane, point, &context).unwrap();
    assert_eq!(contextual_distance.result(), Ok(&legacy_distance));
    assert_eq!(contextual_distance.report().usage().len(), 1);
    assert_eq!(contextual_distance.report().usage()[0].consumed, 0);
}

#[test]
fn contextual_nurbs_fallback_is_bit_equivalent_and_charges_one_query() {
    let surface = nurbs_plane();
    let point = surface.eval([0.3, 0.7]) + Vec3::new(0.0, 0.0, 0.5);
    let policy = session(BudgetPlan::empty());
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();

    let legacy_uv = invert_surface_point(&surface, point).unwrap();
    let contextual_uv = invert_surface_point_with_context(&surface, point, &context).unwrap();
    assert_eq!(contextual_uv.result(), Ok(&legacy_uv));
    let uv_queries = contextual_uv
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == SURFACE_PROJECTION_QUERIES)
        .unwrap();
    assert_eq!(uv_queries.consumed, 1);

    let legacy_distance = distance_to_surface(&surface, point).unwrap();
    let contextual_distance = distance_to_surface_with_context(&surface, point, &context).unwrap();
    assert_eq!(contextual_distance.result(), Ok(&legacy_distance));
    let distance_queries = contextual_distance
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == SURFACE_PROJECTION_QUERIES)
        .unwrap();
    assert_eq!(distance_queries.consumed, 1);
}

#[test]
fn contextual_errors_preserve_projection_input_limits_classification_and_sources() {
    let surface = nurbs_plane();
    let policy = session(BudgetPlan::empty());
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();

    let invalid =
        invert_surface_point_with_context(&surface, Point3::new(f64::NAN, 0.0, 0.0), &context)
            .unwrap();
    let invalid = invalid.into_result().unwrap_err();
    assert!(matches!(
        invalid,
        SurfacePointContextError::Projection(ProjectionError::InvalidQueryPoint)
    ));
    assert_eq!(invalid.class(), ErrorClass::InvalidInput);
    assert!(std::error::Error::source(&invalid).is_some());

    let analytic_invalid = distance_to_surface_with_context(
        &Plane::new(Frame::world()),
        Point3::new(0.0, f64::INFINITY, 0.0),
        &context,
    )
    .unwrap()
    .into_result()
    .unwrap_err();
    assert!(matches!(
        analytic_invalid,
        SurfacePointContextError::Projection(ProjectionError::InvalidQueryPoint)
    ));
    assert_eq!(analytic_invalid.class(), ErrorClass::InvalidInput);

    let denied_context = OperationContext::new(&policy, Tolerances::default())
        .unwrap()
        .with_budget_overrides(override_limit(
            SURFACE_PROJECTION_QUERIES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            0,
        ));
    let denied = distance_to_surface_with_context(&surface, Point3::default(), &denied_context)
        .unwrap()
        .into_result()
        .unwrap_err();
    let snapshot = denied.limit().expect("projection limit remains structured");
    assert_eq!(snapshot.stage, SURFACE_PROJECTION_QUERIES);
    assert_eq!(snapshot.consumed, 1);
    assert_eq!(snapshot.allowed, 0);
    assert_eq!(denied.class(), ErrorClass::ResourceLimit);
    assert_eq!(denied.code(), kcore::error::code::RESOURCE_LIMIT);
    let projection_source = std::error::Error::source(&denied).unwrap();
    assert!(projection_source.source().is_some());
    assert!(matches!(
        denied,
        SurfacePointContextError::Projection(ProjectionError::Policy(
            OperationPolicyError::LimitReached(_)
        ))
    ));

    let unsupported = SurfacePointContextError::UnboundedProjectionWindow;
    assert_eq!(unsupported.class(), ErrorClass::Unsupported);
    assert_eq!(
        unsupported.capability(),
        Some(capability::FINITE_PROJECTION_WINDOW)
    );
}

#[test]
fn analytic_queries_ignore_invalid_fallback_contracts_but_fallbacks_validate_them() {
    let mismatched = BudgetPlan::new([LimitSpec::new(
        SURFACE_PROJECTION_SAMPLES,
        ResourceKind::Items,
        AccountingMode::Cumulative,
        625,
    )])
    .unwrap();
    let policy = session(mismatched);
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();

    assert!(
        invert_surface_point_with_context(&Plane::new(Frame::world()), Point3::default(), &context)
            .unwrap()
            .result()
            .is_ok()
    );
    assert!(matches!(
        invert_surface_point_with_context(&nurbs_plane(), Point3::default(), &context),
        Err(OperationPolicyError::AccountingModeMismatch {
            stage: SURFACE_PROJECTION_SAMPLES,
            resource: ResourceKind::Items,
        })
    ));
}

#[test]
fn shared_scope_queries_accumulate_and_root_total_work_keeps_precedence() {
    let surface = nurbs_plane();
    let point = surface.eval([0.25, 0.75]);
    let repeated_policy = session(BudgetPlan::empty());
    let repeated_context = OperationContext::new(&repeated_policy, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(ProjectionBudgetProfile::surface_defaults())
        .with_budget_overrides(override_limit(
            SURFACE_PROJECTION_QUERIES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            2,
        ));
    let mut repeated_scope = OperationScope::new(&repeated_context);
    invert_surface_point_in_scope(&surface, point, &mut repeated_scope).unwrap();
    distance_to_surface_in_scope(&surface, point, &mut repeated_scope).unwrap();
    let repeated_queries = repeated_scope
        .ledger()
        .snapshots()
        .into_iter()
        .find(|snapshot| snapshot.stage == SURFACE_PROJECTION_QUERIES)
        .unwrap();
    assert_eq!(repeated_queries.consumed, 2);

    let policy = session(BudgetPlan::empty().with_total_work_limit(1));
    let context = OperationContext::new(&policy, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(ProjectionBudgetProfile::surface_defaults())
        .with_budget_overrides(override_limit(
            SURFACE_PROJECTION_QUERIES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            2,
        ));
    let mut scope = OperationScope::new(&context);

    let first = invert_surface_point_in_scope(&surface, point, &mut scope).unwrap();
    assert_eq!(first, invert_surface_point(&surface, point).unwrap());
    let second = distance_to_surface_in_scope(&surface, point, &mut scope).unwrap_err();
    assert!(matches!(
        second,
        SurfacePointContextError::Projection(ProjectionError::Policy(
            OperationPolicyError::LimitReached(snapshot)
        )) if snapshot.stage == TOTAL_WORK_STAGE
            && snapshot.consumed == 2
            && snapshot.allowed == 1
    ));
    assert_eq!(second.limit().unwrap().stage, TOTAL_WORK_STAGE);
}
