use super::*;
use crate::analytic_shell::AnalyticShellOutput;
use crate::analytic_shell::cylinder_cylinder_tests::cap_reaching_cylinder_notch_input;
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::shell_proof::CAP_REACHING_CYLINDER_SHELL_WORK;
use crate::transaction::FullCommitRequirement;

const TOLERANCE: f64 = 1.0e-12;

fn oblique_frame() -> Frame {
    Frame::new(
        Point3::new(2.5, -1.75, 0.625),
        Vec3::new(0.48, 0.64, 0.6),
        Vec3::new(0.8, -0.6, 0.0),
    )
    .unwrap()
}

fn face_for_key(output: &AnalyticShellOutput, key: u64) -> FaceId {
    output
        .faces()
        .iter()
        .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
        .unwrap()
}

fn edge_for_key(output: &AnalyticShellOutput, key: u64) -> EdgeId {
    output
        .edges()
        .iter()
        .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
        .unwrap()
}

fn expected_positive() -> Option<ShellCertification> {
    Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: ShellOrientation::Positive,
    })
}

#[test]
fn cap_reaching_notch_is_full_valid_across_frames_and_permutations() {
    for frame in [Frame::world(), oblique_frame()] {
        for permuted in [false, true] {
            let mut store = Store::new();
            let mut transaction = store.transaction().unwrap();
            let output = transaction
                .assemble_analytic_shell(
                    &cap_reaching_cylinder_notch_input(frame, permuted),
                    TOLERANCE,
                )
                .unwrap();
            assert_eq!(output.faces().len(), 5);
            assert_eq!(output.edges().len(), 7);
            assert_eq!(output.vertices().len(), 4);
            assert_eq!(
                certify_cap_reaching_cylinder_shell(transaction.store(), output.shell(), None,)
                    .unwrap(),
                expected_positive(),
            );
            let report =
                check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
            assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
            transaction
                .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                .unwrap();
        }
    }
}

#[test]
fn cap_reaching_orientation_and_endpoint_tampers_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &cap_reaching_cylinder_notch_input(Frame::world(), false),
            TOLERANCE,
        )
        .unwrap();

    let feature_face = face_for_key(&output, 1);
    let mut wrong_sense = transaction.store().clone();
    wrong_sense.get_mut(feature_face).unwrap().sense = Sense::Forward;
    assert_eq!(
        certify_cap_reaching_cylinder_shell(&wrong_sense, output.shell(), None).unwrap(),
        Some(ShellCertification {
            embedding: ShellEmbedding::Certified,
            orientation: ShellOrientation::Invalid,
        }),
    );

    let reached_host_arc = edge_for_key(&output, 1);
    let mut wrong_endpoints = transaction.store().clone();
    wrong_endpoints
        .get_mut(reached_host_arc)
        .unwrap()
        .vertices
        .swap(0, 1);
    assert_eq!(
        certify_cap_reaching_cylinder_shell(&wrong_endpoints, output.shell(), None).unwrap(),
        None,
    );

    let feature_arc = edge_for_key(&output, 2);
    let mut wrong_incidence = transaction.store().clone();
    let feature_fin = wrong_incidence
        .get(feature_arc)
        .unwrap()
        .fins
        .iter()
        .copied()
        .find(|fin| {
            let loop_id = wrong_incidence.get(*fin).unwrap().parent;
            wrong_incidence.get(loop_id).unwrap().face == feature_face
        })
        .unwrap();
    wrong_incidence.get_mut(feature_fin).unwrap().sense = Sense::Forward;
    assert_eq!(
        certify_cap_reaching_cylinder_shell(&wrong_incidence, output.shell(), None).unwrap(),
        None,
    );
}

fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
    let budget = BudgetPlan::new([LimitSpec::new(
        CAP_REACHING_CYLINDER_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    kcore::operation::SessionPolicy::new(
        kcore::operation::SessionPrecision::parasolid(),
        kcore::operation::NumericalPolicy::v1(),
        kcore::operation::ExecutionPolicy::Serial,
        budget,
        kcore::operation::PolicyVersion::V1,
    )
}

#[test]
fn cap_reaching_work_accepts_exact_n_rejects_n_minus_one_and_skips_inapplicable() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &cap_reaching_cylinder_notch_input(Frame::world(), false),
            TOLERANCE,
        )
        .unwrap();
    let required = proof_work(transaction.store(), output.shell(), 2)
        .unwrap()
        .unwrap();
    assert_eq!(required, 13_600);

    let exact_policy = session_with_work(required);
    let exact_context = kcore::operation::OperationContext::new(
        &exact_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut exact_scope = OperationScope::new(&exact_context);
    assert_eq!(
        certify_cap_reaching_cylinder_shell(
            transaction.store(),
            output.shell(),
            Some(&mut exact_scope),
        )
        .unwrap(),
        expected_positive(),
    );

    let denied_policy = session_with_work(required - 1);
    let denied_context = kcore::operation::OperationContext::new(
        &denied_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut denied_scope = OperationScope::new(&denied_context);
    let error = certify_cap_reaching_cylinder_shell(
        transaction.store(),
        output.shell(),
        Some(&mut denied_scope),
    )
    .unwrap_err();
    assert_eq!(
        error.limit().map(|limit| limit.stage),
        Some(CAP_REACHING_CYLINDER_SHELL_WORK),
    );

    let mut inapplicable = transaction.store().clone();
    inapplicable
        .get_mut(output.shell())
        .unwrap()
        .edges
        .push(edge_for_key(&output, 0));
    let zero_policy = session_with_work(0);
    let zero_context = kcore::operation::OperationContext::new(
        &zero_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut zero_scope = OperationScope::new(&zero_context);
    assert_eq!(
        certify_cap_reaching_cylinder_shell(&inapplicable, output.shell(), Some(&mut zero_scope),)
            .unwrap(),
        None,
    );
}
