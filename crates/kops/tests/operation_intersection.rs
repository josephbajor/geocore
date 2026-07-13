//! Context-aware deterministic accounting for the NURBS-surface marcher.

use std::num::NonZeroUsize;

use kcore::error::{Error, ErrorClass};
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticKind, DiagnosticLevel, ExecutionPolicy, LimitSnapshot,
    LimitSpec, NumericalPolicy, OperationContext, OperationPolicyError, PolicyVersion,
    ResourceKind, SessionPolicy, SessionPrecision, TOTAL_WORK_STAGE,
};
use kcore::proof::{IncompleteCause, IncompleteEvidence};
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
    NURBS_SURFACE_MARCH_CAPABILITIES, NURBS_SURFACE_MARCH_COMPLETE_COVERAGE,
    NURBS_SURFACE_MARCH_DIAGNOSTICS, NURBS_SURFACE_MARCH_INCOMPLETE,
    NURBS_SURFACE_MARCH_SAMPLE_LIMIT, NURBS_SURFACE_MARCH_SAMPLES, NurbsSurfaceMarchBudgetProfile,
    SurfaceSurfaceIntersections, intersect_bounded_plane_nurbs_surface,
    intersect_bounded_plane_nurbs_surface_with_context, intersect_bounded_surfaces,
};

const ACTUAL_GRID_SAMPLES: u64 = 25 * 25;
const MARCH_INCOMPLETE_REASON: &str =
    "fixed-grid NURBS surface marching does not prove complete coverage";

fn fixed_grid_evidence() -> IncompleteEvidence {
    IncompleteEvidence {
        code: NURBS_SURFACE_MARCH_INCOMPLETE,
        stage: NURBS_SURFACE_MARCH_SAMPLES,
        cause: IncompleteCause::ProofMethodUnavailable {
            capability: NURBS_SURFACE_MARCH_COMPLETE_COVERAGE,
        },
        message: MARCH_INCOMPLETE_REASON,
    }
}

#[test]
fn marcher_identifier_inventories_are_finite_unique_and_stable() {
    use std::collections::BTreeSet;

    const FROZEN_DIAGNOSTICS: &[&str] = &[
        "kops.intersect.ssi-grid-sample-limit",
        "kops.intersect.ssi-fixed-grid-incomplete",
    ];
    const FROZEN_CAPABILITIES: &[&str] = &["kops.intersect.ssi-fixed-grid-complete-coverage"];

    assert_eq!(
        NURBS_SURFACE_MARCH_DIAGNOSTICS
            .iter()
            .map(|code| code.as_str())
            .collect::<Vec<_>>(),
        FROZEN_DIAGNOSTICS
    );
    assert_eq!(
        NURBS_SURFACE_MARCH_CAPABILITIES
            .iter()
            .map(|capability| capability.as_str())
            .collect::<Vec<_>>(),
        FROZEN_CAPABILITIES
    );

    let diagnostics: BTreeSet<_> = NURBS_SURFACE_MARCH_DIAGNOSTICS
        .iter()
        .map(|code| code.as_str())
        .collect();
    let capabilities: BTreeSet<_> = NURBS_SURFACE_MARCH_CAPABILITIES
        .iter()
        .map(|capability| capability.as_str())
        .collect();
    assert_eq!(diagnostics.len(), NURBS_SURFACE_MARCH_DIAGNOSTICS.len());
    assert_eq!(capabilities.len(), NURBS_SURFACE_MARCH_CAPABILITIES.len());
    assert!(
        diagnostics
            .iter()
            .chain(capabilities.iter())
            .all(|identifier| identifier.starts_with("kops.intersect."))
    );
    assert!(
        FROZEN_DIAGNOSTICS
            .iter()
            .all(|&identifier| kcore::operation::DiagnosticCode::new(identifier).is_ok())
    );
    assert!(
        FROZEN_CAPABILITIES
            .iter()
            .all(|&identifier| kcore::error::CapabilityId::new(identifier).is_ok())
    );
    assert!(diagnostics.is_disjoint(&capabilities));
    assert_eq!(
        NURBS_SURFACE_MARCH_DIAGNOSTICS[0],
        NURBS_SURFACE_MARCH_SAMPLE_LIMIT
    );
    assert_eq!(
        NURBS_SURFACE_MARCH_DIAGNOSTICS[1],
        NURBS_SURFACE_MARCH_INCOMPLETE
    );
    assert_eq!(
        NURBS_SURFACE_MARCH_CAPABILITIES[0],
        NURBS_SURFACE_MARCH_COMPLETE_COVERAGE
    );
    assert!(
        !NURBS_SURFACE_MARCH_DIAGNOSTICS.contains(&NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT),
        "kgeom-owned proof diagnostics must not be duplicated in kops inventory"
    );
}

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
fn structured_incompleteness_survives_canonicalization_swapping_and_dispatch() {
    let plane = plane();
    let surface = surface();
    let tolerances = Tolerances::default();
    let result = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    let expected = vec![fixed_grid_evidence()];
    assert_eq!(result.incomplete_evidence(), expected);
    assert_eq!(
        NURBS_SURFACE_MARCH_INCOMPLETE.as_str(),
        "kops.intersect.ssi-fixed-grid-incomplete"
    );
    assert_eq!(
        NURBS_SURFACE_MARCH_COMPLETE_COVERAGE.as_str(),
        "kops.intersect.ssi-fixed-grid-complete-coverage"
    );
    assert_eq!(
        result.completion().indeterminate_reason(),
        Some(MARCH_INCOMPLETE_REASON)
    );

    let rebuilt = SurfaceSurfaceIntersections::canonicalized_with_incomplete_evidence(
        result.points.clone(),
        result.curves.clone(),
        MARCH_INCOMPLETE_REASON,
        expected.clone(),
    )
    .unwrap();
    assert_eq!(rebuilt.incomplete_evidence(), expected);
    assert_eq!(result.clone().swapped().incomplete_evidence(), expected);
    let repeated = vec![fixed_grid_evidence(), fixed_grid_evidence()];
    let repeated_result = SurfaceSurfaceIntersections::canonicalized_with_incomplete_evidence(
        result.points.clone(),
        result.curves.clone(),
        MARCH_INCOMPLETE_REASON,
        repeated.clone(),
    )
    .unwrap();
    assert_eq!(repeated_result.incomplete_evidence(), repeated);
    assert_eq!(
        repeated_result.swapped().incomplete_evidence(),
        repeated,
        "normalization must not sort or deduplicate proof observations"
    );

    let direct = intersect_bounded_surfaces(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        tolerances,
    )
    .unwrap();
    let reversed = intersect_bounded_surfaces(
        &surface,
        surface.param_range(),
        &plane,
        plane_range(),
        tolerances,
    )
    .unwrap();
    assert_eq!(direct.incomplete_evidence(), expected);
    assert_eq!(reversed.incomplete_evidence(), expected);

    let complete = intersect_bounded_plane_nurbs_surface(
        &plane,
        plane_range(),
        &separated_surface(),
        [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)],
        tolerances,
    )
    .unwrap();
    assert!(complete.is_complete());
    assert!(complete.incomplete_evidence().is_empty());
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
        8
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
    let candidate_result = candidate.result().unwrap();
    assert_eq!(candidate_result.incomplete_evidence().len(), 2);
    assert_eq!(
        candidate_result.incomplete_evidence()[0],
        IncompleteEvidence {
            code: NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT,
            stage: NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            cause: IncompleteCause::Limit {
                snapshot: candidate_snapshot,
            },
            message: "NURBS implicit-isolation candidate-cover limit reached",
        }
    );
    assert_eq!(
        candidate_result.incomplete_evidence()[1],
        fixed_grid_evidence()
    );

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
    let depth_result = depth.result().unwrap();
    assert_eq!(depth_result.incomplete_evidence().len(), 2);
    assert_eq!(
        depth_result.incomplete_evidence()[0],
        IncompleteEvidence {
            code: NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT,
            stage: NURBS_IMPLICIT_ISOLATION_DEPTH,
            cause: IncompleteCause::Limit {
                snapshot: depth_snapshot,
            },
            message: "NURBS implicit-isolation depth limit reached",
        }
    );
    assert_eq!(depth_result.incomplete_evidence()[1], fixed_grid_evidence());
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
    assert_eq!(
        proof_limited.result().unwrap().incomplete_evidence(),
        [
            IncompleteEvidence {
                code: NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT,
                stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
                cause: IncompleteCause::Limit {
                    snapshot: proof_snapshot,
                },
                message: "NURBS implicit-isolation proof setup limit reached",
            },
            fixed_grid_evidence(),
        ]
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
    assert_eq!(
        proof_off.result().unwrap().incomplete_evidence(),
        proof_limited.result().unwrap().incomplete_evidence()
    );

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
fn source_rectangle_bvh_work_has_exact_composed_n_and_n_minus_one_boundaries() {
    let plane = plane();
    let surface = separated_surface();
    let tolerances = Tolerances::default();
    let session = session(ExecutionPolicy::Serial);
    let exact_work = 8; // setup + one single-span source-range BVH enclosure

    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_limit(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            exact_work,
        ));
    let exact = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &exact_context,
    )
    .unwrap();
    assert!(exact.result().is_ok_and(|result| result.is_proven_empty()));
    assert_eq!(
        usage_for(exact.report(), NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
        exact_work
    );

    let low_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(override_limit(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            exact_work - 1,
        ));
    let low = intersect_bounded_plane_nurbs_surface_with_context(
        &plane,
        plane_range(),
        &surface,
        surface.param_range(),
        &low_context,
    )
    .unwrap();
    let snapshot = LimitSnapshot {
        stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
        resource: ResourceKind::Work,
        consumed: exact_work,
        allowed: exact_work - 1,
    };
    assert!(low.result().is_ok_and(|result| !result.is_complete()));
    assert_eq!(low.report().limit_events(), &[snapshot]);
    assert_eq!(
        usage_for(low.report(), NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
        1,
        "denied BVH source-range preflight leaves only the accepted setup unit"
    );
    assert_eq!(
        low.result().unwrap().incomplete_evidence()[0],
        IncompleteEvidence {
            code: NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT,
            stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            cause: IncompleteCause::Limit { snapshot },
            message: "NURBS source-rectangle BVH work limit reached",
        }
    );

    let default_work = NurbsSurfaceMarchBudgetProfile::v1_defaults()
        .limits()
        .iter()
        .find(|limit| {
            limit.stage == NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS
                && limit.resource == ResourceKind::Work
        })
        .unwrap()
        .allowed;
    assert_eq!(
        default_work,
        1 + 4_096 * (6 * 4_096 + 1) + 12 * 4_096 * (1 + 4 * (6 * 4_096 + 1)),
        "the profile composes setup, maximum source-BVH scans, and every candidate child scan"
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
    let evidence = outcome.result().unwrap().incomplete_evidence();
    assert_eq!(evidence.len(), 2);
    assert_eq!(
        evidence[0],
        IncompleteEvidence {
            code: NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION,
            stage: NURBS_IMPLICIT_ISOLATION_DEPTH,
            cause: IncompleteCause::NumericResolution,
            message: "NURBS implicit isolation stopped at floating-point parameter resolution",
        }
    );
    assert_eq!(evidence[1], fixed_grid_evidence());

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
    assert_eq!(diagnosed.result().unwrap().incomplete_evidence(), evidence);
    assert!(diagnosed.report().diagnostics().iter().any(|diagnostic| {
        diagnostic.stage == NURBS_IMPLICIT_ISOLATION_DEPTH
            && diagnostic.code == NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION
            && diagnostic.kind == DiagnosticKind::NumericResolution
    }));
}
