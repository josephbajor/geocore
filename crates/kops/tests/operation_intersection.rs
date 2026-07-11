//! Context-aware deterministic accounting for the NURBS-surface marcher.

use std::num::NonZeroUsize;

use kcore::error::{Error, ErrorClass};
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticKind, DiagnosticLevel, ExecutionPolicy, LimitSnapshot,
    LimitSpec, NumericalPolicy, OperationContext, OperationPolicyError, PolicyVersion,
    ResourceKind, SessionPolicy, SessionPrecision, TOTAL_WORK_STAGE,
};
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::nurbs::NurbsSurface;
use kgeom::nurbs::{
    NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT, NURBS_IMPLICIT_ISOLATION_CANDIDATES,
    NURBS_IMPLICIT_ISOLATION_DEPTH, NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT,
    NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION, NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    NURBS_SURFACE_MARCH_SAMPLES, NurbsSurfaceMarchBudgetProfile,
    intersect_bounded_plane_nurbs_surface, intersect_bounded_plane_nurbs_surface_with_context,
};

const ACTUAL_GRID_SAMPLES: u64 = 25 * 25;

fn plane() -> Plane {
    Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    )
}

fn surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, -0.4),
            Point3::new(0.0, 1.0, -0.4),
            Point3::new(1.0, 0.0, 0.6),
            Point3::new(1.0, 1.0, 0.6),
        ],
        None,
    )
    .unwrap()
}

fn separated_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.0, 1.0, 2.0),
            Point3::new(1.0, 0.0, 2.0),
            Point3::new(1.0, 1.0, 2.0),
        ],
        None,
    )
    .unwrap()
}

fn coincident_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap()
}

fn resolution_limited_surface() -> NurbsSurface {
    let lo = 1.0_f64;
    let hi = lo.next_up();
    NurbsSurface::new(
        1,
        1,
        vec![lo, lo, hi, hi],
        vec![lo, lo, hi, hi],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        None,
    )
    .unwrap()
}

fn plane_range() -> [ParamRange; 2] {
    [ParamRange::new(0.0, 1.0), ParamRange::new(-0.1, 1.1)]
}

fn session(execution: ExecutionPolicy) -> SessionPolicy {
    SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        execution,
        NurbsSurfaceMarchBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    )
}

fn override_budget(allowed: u64) -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        NURBS_SURFACE_MARCH_SAMPLES,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap()
}

fn override_limit(
    stage: kcore::operation::StageId,
    resource: ResourceKind,
    mode: AccountingMode,
    allowed: u64,
) -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(stage, resource, mode, allowed)]).unwrap()
}

fn usage_for(
    report: &kcore::operation::OperationReport,
    stage: kcore::operation::StageId,
) -> LimitSnapshot {
    *report
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == stage)
        .unwrap()
}

#[test]
fn v1_context_is_bit_exact_with_compatibility_and_retains_usage() {
    let plane = plane();
    let surface = surface();
    let tolerances = Tolerances::default();
    let compatibility = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        tolerances,
    )
    .unwrap();

    let session = session(ExecutionPolicy::Serial);
    let context = OperationContext::new(&session, tolerances).unwrap();
    let outcome = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &context,
    )
    .unwrap();

    assert_eq!(outcome.result(), Ok(&compatibility));
    assert!(!compatibility.is_complete());
    let usage = outcome
        .report()
        .usage()
        .iter()
        .find(|snapshot| snapshot.stage == NURBS_SURFACE_MARCH_SAMPLES)
        .unwrap();
    assert_eq!(usage.resource, ResourceKind::Work);
    assert_eq!(usage.consumed, ACTUAL_GRID_SAMPLES);
    assert_eq!(usage.allowed, 9_409);
}

#[test]
fn exact_boundary_override_succeeds_and_exhaustion_retains_one_snapshot_everywhere() {
    let plane = plane();
    let surface = surface();
    let tolerances = Tolerances::default();
    let session = session(ExecutionPolicy::Serial);

    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_budget(ACTUAL_GRID_SAMPLES));
    let exact = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &exact_context,
    )
    .unwrap();
    assert!(exact.result().is_ok());
    let exact_usage = usage_for(exact.report(), NURBS_SURFACE_MARCH_SAMPLES);
    assert_eq!(exact_usage.consumed, ACTUAL_GRID_SAMPLES);
    assert_eq!(exact_usage.allowed, ACTUAL_GRID_SAMPLES);

    let plus_one_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_budget(ACTUAL_GRID_SAMPLES + 1));
    let plus_one = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &plus_one_context,
    )
    .unwrap();
    let plus_one_usage = usage_for(plus_one.report(), NURBS_SURFACE_MARCH_SAMPLES);
    assert!(plus_one.result().is_ok());
    assert_eq!(plus_one_usage.consumed, ACTUAL_GRID_SAMPLES);
    assert_eq!(plus_one_usage.allowed, ACTUAL_GRID_SAMPLES + 1);

    let allowed = ACTUAL_GRID_SAMPLES - 1;
    let limited_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_budget(allowed))
        .with_diagnostics(DiagnosticLevel::Summary, 1);
    let limited = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &limited_context,
    )
    .unwrap();

    let expected = LimitSnapshot {
        stage: NURBS_SURFACE_MARCH_SAMPLES,
        resource: ResourceKind::Work,
        consumed: ACTUAL_GRID_SAMPLES,
        allowed,
    };
    assert_eq!(limited.report().limit_events(), &[expected]);
    let error = limited.result().unwrap_err();
    assert_eq!(*error, Error::ResourceLimit { snapshot: expected });
    assert_eq!(error.class(), ErrorClass::ResourceLimit);
    assert_eq!(error.limit(), Some(expected));
    let diagnostic_snapshot = match limited.report().diagnostics()[0].kind {
        DiagnosticKind::LimitReached(snapshot) => snapshot,
        other => panic!("unexpected limit diagnostic: {other:?}"),
    };
    assert_eq!(diagnostic_snapshot, expected);

    let accepted_usage = usage_for(limited.report(), NURBS_SURFACE_MARCH_SAMPLES);
    assert_eq!(accepted_usage.stage, expected.stage);
    assert_eq!(accepted_usage.resource, expected.resource);
    assert_eq!(accepted_usage.consumed, allowed);
    assert_eq!(accepted_usage.allowed, expected.allowed);

    let (result, report) = limited.into_parts();
    assert_eq!(result, Err(Error::ResourceLimit { snapshot: expected }));
    assert_eq!(
        report.diagnostics()[0].kind,
        DiagnosticKind::LimitReached(expected)
    );
    assert_eq!(report.limit_events(), &[expected]);
}

#[test]
fn execution_policy_does_not_change_serial_march_results_or_reports() {
    let plane = plane();
    let surface = surface();
    let tolerances = Tolerances::default();
    let serial_session = session(ExecutionPolicy::Serial);
    let parallel_session = session(ExecutionPolicy::AtMost(NonZeroUsize::new(2).unwrap()));
    let serial_context = OperationContext::new(&serial_session, tolerances).unwrap();
    let parallel_context = OperationContext::new(&parallel_session, tolerances).unwrap();

    let serial = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &serial_context,
    )
    .unwrap();
    let parallel = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &parallel_context,
    )
    .unwrap();

    assert_eq!(serial, parallel);
}

#[test]
fn missing_stage_is_rejected_before_an_early_complete_proof_exit() {
    let plane = plane();
    let surface = separated_surface();
    let compatibility = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        Tolerances::default(),
    )
    .unwrap();
    assert!(compatibility.is_proven_empty());

    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let error = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &context,
    )
    .unwrap_err();
    assert_eq!(
        error,
        OperationPolicyError::UnknownLimit {
            stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            resource: ResourceKind::Work,
        }
    );
}

#[test]
fn complete_empty_and_kernel_errors_match_the_legacy_adapter() {
    let plane = plane();
    let tolerances = Tolerances::default();
    let session = session(ExecutionPolicy::Serial);
    let context = OperationContext::new(&session, tolerances).unwrap();

    let separated = separated_surface();
    let legacy_empty = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_range(),
        &separated,
        separated.param_range(),
        tolerances,
    )
    .unwrap();
    let contextual_empty = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &separated,
        separated.param_range(),
        &context,
    )
    .unwrap();
    assert_eq!(contextual_empty.result(), Ok(&legacy_empty));
    assert!(legacy_empty.is_proven_empty());
    assert_eq!(
        usage_for(
            contextual_empty.report(),
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
        )
        .consumed,
        1
    );
    assert_eq!(
        usage_for(contextual_empty.report(), NURBS_SURFACE_MARCH_SAMPLES).consumed,
        0
    );

    let coincident = coincident_surface();
    let legacy_overlap = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_range(),
        &coincident,
        coincident.param_range(),
        tolerances,
    )
    .unwrap_err();
    let contextual_overlap = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &coincident,
        coincident.param_range(),
        &context,
    )
    .unwrap();
    assert_eq!(contextual_overlap.result(), Err(&legacy_overlap));

    let invalid_plane_range = [ParamRange::unbounded(), ParamRange::new(0.0, 1.0)];
    let legacy_invalid = intersect_bounded_plane_nurbs_surface(
        &plane,
        invalid_plane_range,
        &separated,
        separated.param_range(),
        tolerances,
    )
    .unwrap_err();
    let contextual_invalid = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        invalid_plane_range,
        &separated,
        separated.param_range(),
        &context,
    )
    .unwrap();
    assert_eq!(contextual_invalid.result(), Err(&legacy_invalid));
}

#[test]
fn proof_candidate_and_depth_limits_are_structured_and_never_complete() {
    let plane = plane();
    let surface = surface();
    let tolerances = Tolerances::default();
    let session = session(ExecutionPolicy::Serial);

    let candidate_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_limit(
            NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            0,
        ))
        .with_diagnostics(DiagnosticLevel::Summary, 8);
    let candidate = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &candidate_context,
    )
    .unwrap();
    assert!(candidate.result().is_ok_and(|result| !result.is_complete()));
    let candidate_limit = candidate
        .report()
        .diagnostics()
        .iter()
        .find(|diagnostic| diagnostic.code == NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT)
        .unwrap();
    assert_eq!(candidate_limit.stage, NURBS_IMPLICIT_ISOLATION_CANDIDATES);
    let candidate_snapshot = LimitSnapshot {
        stage: NURBS_IMPLICIT_ISOLATION_CANDIDATES,
        resource: ResourceKind::Items,
        consumed: 1,
        allowed: 0,
    };
    assert_eq!(
        candidate_limit.kind,
        DiagnosticKind::LimitReached(candidate_snapshot)
    );
    assert_eq!(candidate.report().limit_events(), &[candidate_snapshot]);

    let depth_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_limit(
            NURBS_IMPLICIT_ISOLATION_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            0,
        ))
        .with_diagnostics(DiagnosticLevel::Summary, 8);
    let depth = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &depth_context,
    )
    .unwrap();
    assert!(depth.result().is_ok_and(|result| !result.is_complete()));
    let depth_limit = depth
        .report()
        .diagnostics()
        .iter()
        .find(|diagnostic| diagnostic.code == NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT)
        .unwrap();
    let depth_snapshot = LimitSnapshot {
        stage: NURBS_IMPLICIT_ISOLATION_DEPTH,
        resource: ResourceKind::Depth,
        consumed: 1,
        allowed: 0,
    };
    assert_eq!(
        depth_limit.kind,
        DiagnosticKind::LimitReached(depth_snapshot)
    );
    assert_eq!(depth.report().limit_events(), &[depth_snapshot]);
}

#[test]
fn proof_work_and_root_zero_limits_stop_before_complete_proof() {
    let plane = plane();
    let surface = separated_surface();
    let tolerances = Tolerances::default();
    let session = session(ExecutionPolicy::Serial);

    let proof_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_limit(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            0,
        ))
        .with_diagnostics(DiagnosticLevel::Summary, 4);
    let proof_limited = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &proof_context,
    )
    .unwrap();
    let proof_snapshot = LimitSnapshot {
        stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
        resource: ResourceKind::Work,
        consumed: 1,
        allowed: 0,
    };
    assert_eq!(proof_limited.report().limit_events(), &[proof_snapshot]);
    assert!(
        proof_limited
            .result()
            .is_ok_and(|result| !result.is_complete())
    );
    assert!(
        proof_limited
            .report()
            .diagnostics()
            .iter()
            .any(|diagnostic| {
                diagnostic.code == NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT
                    && matches!(
                        diagnostic.kind,
                        DiagnosticKind::LimitReached(snapshot) if snapshot == proof_snapshot
                    )
            })
    );

    let proof_off_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_limit(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            0,
        ));
    let proof_off = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &proof_off_context,
    )
    .unwrap();
    assert!(proof_off.result().is_ok_and(|result| !result.is_complete()));
    assert!(proof_off.report().diagnostics().is_empty());
    assert_eq!(proof_off.report().limit_events(), &[proof_snapshot]);

    let root_session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        NurbsSurfaceMarchBudgetProfile::v1_defaults().with_total_work_limit(0),
        PolicyVersion::V1,
    );
    let root_context = OperationContext::new(&root_session, tolerances)
        .unwrap()
        .with_diagnostics(DiagnosticLevel::Summary, 4);
    let root_limited = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &root_context,
    )
    .unwrap();
    let error = root_limited.result().unwrap_err();
    let snapshot = error.limit().unwrap();
    assert_eq!(snapshot.stage, TOTAL_WORK_STAGE);
    assert_eq!(snapshot.resource, ResourceKind::Work);
    assert_eq!(snapshot.consumed, 1);
    assert_eq!(snapshot.allowed, 0);
    assert_eq!(root_limited.report().limit_events(), &[snapshot]);
    assert!(
        root_limited
            .report()
            .diagnostics()
            .iter()
            .any(|diagnostic| {
                diagnostic.code == NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT
                    && diagnostic.kind == DiagnosticKind::LimitReached(snapshot)
            })
    );
    assert_eq!(
        usage_for(root_limited.report(), NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,).consumed,
        0
    );
}

#[test]
fn numeric_resolution_remains_structured_when_diagnostics_are_off() {
    let plane = plane();
    let surface = resolution_limited_surface();
    let session = session(ExecutionPolicy::Serial);
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let outcome = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &context,
    )
    .unwrap();
    assert!(outcome.result().is_ok_and(|result| !result.is_complete()));
    assert!(outcome.report().diagnostics().is_empty());
    assert!(outcome.report().limit_events().is_empty());
    assert_eq!(
        outcome.report().numeric_resolution_stages(),
        &[NURBS_IMPLICIT_ISOLATION_DEPTH]
    );

    let diagnostic_context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_diagnostics(DiagnosticLevel::Summary, 2);
    let diagnosed = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &diagnostic_context,
    )
    .unwrap();
    assert_eq!(
        diagnosed.report().numeric_resolution_stages(),
        &[NURBS_IMPLICIT_ISOLATION_DEPTH]
    );
    assert!(diagnosed.report().diagnostics().iter().any(|diagnostic| {
        diagnostic.stage == NURBS_IMPLICIT_ISOLATION_DEPTH
            && diagnostic.code == NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION
            && diagnostic.kind == DiagnosticKind::NumericResolution
    }));
}
