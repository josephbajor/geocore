//! Deterministic Q2 topology fixtures and checked-commit operations.

use kgeom::frame::Frame;
use kgeom::vec::Point3;
use ktopo::benchmark::{
    CommitObservation, IndexAudit, IndexSnapshot, StoreSnapshot, audit_full_rebuild,
    index_snapshot, last_commit, store_snapshot,
};
use ktopo::entity::{BodyId, PointId};
use ktopo::make;
use ktopo::store::Store;

/// Fixture identity shared by all Q2 cases.
pub const FIXTURE_VERSION: &str = "topology-commit.v1";
/// Fixture identity for the mixed-store affected-root scaling matrix.
pub const COHORT_FIXTURE_VERSION: &str = "topology-commit-cohort.v2";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_4f50_4f00_0002;

/// One of the seven Q2 benchmark ladders.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ladder {
    /// Begin and commit a transaction with no edits.
    Clean,
    /// Edit exactly one independent body's point.
    Local,
    /// Edit one point shared by every dependent body.
    Fanout,
    /// Edit one point shared by a bounded cohort amid unrelated solid bodies.
    Cohort,
    /// Edit every independent body's point in one transaction.
    Batched,
    /// Attempt one invalid body mutation and verify rollback.
    Rejected,
    /// Commit cleanly, then compare an independent full index rebuild.
    FullRebuild,
}

/// Stable Q2 case definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TopologyCase {
    /// Five-segment benchmark path.
    pub path: &'static str,
    /// Ladder operation.
    pub ladder: Ladder,
    /// Number of bodies in the fixture.
    pub bodies: usize,
    /// Exact body roots expected to be affected, refreshed, and checked.
    pub affected_bodies: usize,
    /// Reviewed digest of affected bodies encoded as store ordinals.
    pub expected_affected_digest: u64,
    /// Reviewed digest of complete semantic result evidence.
    pub expected_output_digest: u64,
}

/// Exactly the 28 Q2 cases specified by the quality contract.
pub const CASES: [TopologyCase; 28] = [
    case(
        "topology/checked-commit/isolated-acorns-v1/1/clean-v1",
        Ladder::Clean,
        1,
    ),
    case(
        "topology/checked-commit/isolated-acorns-v1/10/clean-v1",
        Ladder::Clean,
        10,
    ),
    case(
        "topology/checked-commit/isolated-acorns-v1/100/clean-v1",
        Ladder::Clean,
        100,
    ),
    case(
        "topology/checked-commit/isolated-acorns-v1/1000/clean-v1",
        Ladder::Clean,
        1_000,
    ),
    case(
        "topology/index-refresh/isolated-acorns-v1/1/local-v1",
        Ladder::Local,
        1,
    ),
    case(
        "topology/index-refresh/isolated-acorns-v1/10/local-v1",
        Ladder::Local,
        10,
    ),
    case(
        "topology/index-refresh/isolated-acorns-v1/100/local-v1",
        Ladder::Local,
        100,
    ),
    case(
        "topology/index-refresh/isolated-acorns-v1/1000/local-v1",
        Ladder::Local,
        1_000,
    ),
    case(
        "topology/index-refresh/shared-geometry-v1/1/fanout-v1",
        Ladder::Fanout,
        1,
    ),
    case(
        "topology/index-refresh/shared-geometry-v1/10/fanout-v1",
        Ladder::Fanout,
        10,
    ),
    case(
        "topology/index-refresh/shared-geometry-v1/100/fanout-v1",
        Ladder::Fanout,
        100,
    ),
    case(
        "topology/checked-commit/isolated-acorns-v1/1/batched-v1",
        Ladder::Batched,
        1,
    ),
    case(
        "topology/checked-commit/isolated-acorns-v1/10/batched-v1",
        Ladder::Batched,
        10,
    ),
    case(
        "topology/checked-commit/isolated-acorns-v1/100/batched-v1",
        Ladder::Batched,
        100,
    ),
    case(
        "topology/checked-commit/rejected-edit-v1/1/rollback-v1",
        Ladder::Rejected,
        1,
    ),
    case(
        "topology/checked-commit/rejected-edit-v1/10/rollback-v1",
        Ladder::Rejected,
        10,
    ),
    case(
        "topology/checked-commit/rejected-edit-v1/100/rollback-v1",
        Ladder::Rejected,
        100,
    ),
    case(
        "topology/index-audit/primitive-mix-v1/1/full-rebuild-v1",
        Ladder::FullRebuild,
        1,
    ),
    case(
        "topology/index-audit/primitive-mix-v1/10/full-rebuild-v1",
        Ladder::FullRebuild,
        10,
    ),
    case(
        "topology/index-audit/primitive-mix-v1/100/full-rebuild-v1",
        Ladder::FullRebuild,
        100,
    ),
    case(
        "topology/index-audit/primitive-mix-v1/1000/full-rebuild-v1",
        Ladder::FullRebuild,
        1_000,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-4/affected-4-v2",
        4,
        4,
        0xd8de_f909_5c13_1d26,
        0x2beb_eafe_609b_daf7,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-16/affected-4-v2",
        16,
        4,
        0xd8de_f909_5c13_1d26,
        0x3598_0935_f734_98d9,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-64/affected-4-v2",
        64,
        4,
        0xd8de_f909_5c13_1d26,
        0x428c_9c20_a0f4_1b65,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-256/affected-4-v2",
        256,
        4,
        0xd8de_f909_5c13_1d26,
        0x40fc_21ff_7e68_c557,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-64/affected-1-v2",
        64,
        1,
        0x0300_00b9_a2c2_19c6,
        0x08a2_efcc_a4b1_bbe1,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-64/affected-16-v2",
        64,
        16,
        0x121b_7067_4cb3_6c32,
        0x7966_581c_5db4_4246,
    ),
    cohort_case(
        "topology/affected-root-scope/mixed-store-cohort-v2/total-64/affected-64-v2",
        64,
        64,
        0xac06_c657_a252_9702,
        0xecc4_d042_f79f_8a7a,
    ),
];

const fn case(path: &'static str, ladder: Ladder, bodies: usize) -> TopologyCase {
    let expected_affected_digest = match ladder {
        Ladder::Clean | Ladder::FullRebuild => 0x8aa9_84d6_2998_05c2,
        Ladder::Local | Ladder::Rejected => 0x0300_00b9_a2c2_19c6,
        Ladder::Fanout | Ladder::Batched if bodies == 1 => 0x0300_00b9_a2c2_19c6,
        Ladder::Fanout | Ladder::Batched if bodies == 10 => 0x64f9_99dc_db3f_1b07,
        Ladder::Fanout | Ladder::Batched if bodies == 100 => 0x3334_ddf2_07ed_af46,
        _ => 0,
    };
    let expected_output_digest = match (ladder, bodies) {
        (Ladder::Clean, 1) => 0x5760_1e34_8d11_8f48,
        (Ladder::Clean, 10) => 0x7526_7408_688c_9bbd,
        (Ladder::Clean, 100) => 0xef39_58b6_6430_39b3,
        (Ladder::Clean, 1_000) => 0x07d2_045f_0918_791c,
        (Ladder::Local, 1) => 0x3c02_beb5_c4ce_906a,
        (Ladder::Local, 10) => 0xf176_3b67_ec71_baca,
        (Ladder::Local, 100) => 0x28a2_4a29_d46d_4c7c,
        (Ladder::Local, 1_000) => 0xd82f_13d6_5f5c_ef19,
        (Ladder::Fanout, 1) => 0xed4d_2c28_ab30_2bea,
        (Ladder::Fanout, 10) => 0xb5a5_1314_90fe_0ae1,
        (Ladder::Fanout, 100) => 0x8c6e_d51a_f43f_5fba,
        (Ladder::Batched, 1) => 0x3c02_beb5_c4ce_906a,
        (Ladder::Batched, 10) => 0x1a53_52bc_5dc4_cc2d,
        (Ladder::Batched, 100) => 0x5034_a7d2_2cc3_f692,
        (Ladder::Rejected, 1) => 0x54ca_000d_26fb_d60a,
        (Ladder::Rejected, 10) => 0xe4c0_f293_7e5f_5f9f,
        (Ladder::Rejected, 100) => 0x9f0f_069a_b339_5355,
        (Ladder::FullRebuild, 1) => 0x6114_4f04_492f_23c8,
        (Ladder::FullRebuild, 10) => 0xe118_43f5_7e94_1eaf,
        (Ladder::FullRebuild, 100) => 0x4963_c619_e747_6031,
        (Ladder::FullRebuild, 1_000) => 0x11a6_523b_b4cc_68bc,
        _ => 0,
    };
    TopologyCase {
        path,
        ladder,
        bodies,
        affected_bodies: match ladder {
            Ladder::Clean | Ladder::FullRebuild => 0,
            Ladder::Local | Ladder::Rejected => 1,
            Ladder::Fanout | Ladder::Batched => bodies,
            Ladder::Cohort => 0,
        },
        expected_affected_digest,
        expected_output_digest,
    }
}

const fn cohort_case(
    path: &'static str,
    bodies: usize,
    affected_bodies: usize,
    expected_affected_digest: u64,
    expected_output_digest: u64,
) -> TopologyCase {
    TopologyCase {
        path,
        ladder: Ladder::Cohort,
        bodies,
        affected_bodies,
        expected_affected_digest,
        expected_output_digest,
    }
}

/// Fully constructed Q2 fixture. Clone it outside timed work for each sample.
#[derive(Clone)]
pub struct TopologyFixture {
    store: Store,
    bodies: Box<[BodyId]>,
    points: Box<[PointId]>,
}

impl TopologyFixture {
    /// Build independent minimal bodies in deterministic insertion order.
    pub fn isolated_acorns(body_count: usize) -> Self {
        assert!(body_count > 0);
        let mut store = Store::new();
        let mut bodies = Vec::with_capacity(body_count);
        let mut points = Vec::with_capacity(body_count);
        for ordinal in 0..body_count {
            let body = make::acorn(&mut store, Point3::new(ordinal as f64 * 0.01, 0.0, 0.0))
                .expect("Q2 acorn fixture must be valid");
            let vertex = store.vertices_of_body(body).expect("valid body")[0];
            bodies.push(body);
            points.push(store.get(vertex).expect("valid vertex").point);
        }
        Self {
            store,
            bodies: bodies.into_boxed_slice(),
            points: points.into_boxed_slice(),
        }
    }

    /// Build bodies sharing one legal point-geometry dependency.
    pub fn shared_geometry_fanout(body_count: usize) -> Self {
        let mut fixture = Self::isolated_acorns(body_count);
        let shared = fixture.points[0];
        let mut transaction = fixture.store.transaction().expect("fixture transaction");
        for ordinal in 1..body_count {
            let vertex = transaction
                .store()
                .vertices_of_body(fixture.bodies[ordinal])
                .expect("valid body")[0];
            let old_point = transaction.store().get(vertex).expect("valid vertex").point;
            transaction
                .assembly()
                .get_mut(vertex)
                .expect("valid vertex")
                .point = shared;
            transaction
                .assembly()
                .remove(old_point)
                .expect("unreferenced point");
        }
        transaction
            .commit_checked(&fixture.bodies)
            .expect("shared dependency fixture must remain valid");
        fixture.points = vec![shared; body_count].into_boxed_slice();
        fixture
    }

    /// Build a shared-point cohort followed by unrelated production solids.
    pub fn mixed_store_shared_point_cohort(body_count: usize, cohort_count: usize) -> Self {
        assert!(cohort_count > 0 && cohort_count <= body_count);
        let mut store = Store::new();
        let mut bodies = Vec::with_capacity(body_count);
        let mut points = Vec::with_capacity(cohort_count);
        for _ in 0..cohort_count {
            let body = make::acorn(&mut store, Point3::new(0.0, 0.0, 0.0))
                .expect("Q2 cohort acorn must be valid");
            let vertex = store.vertices_of_body(body).expect("valid body")[0];
            bodies.push(body);
            points.push(store.get(vertex).expect("valid vertex").point);
        }
        for ordinal in 0..body_count - cohort_count {
            bodies.push(insert_primitive(&mut store, ordinal));
        }

        let shared = points[0];
        let mut transaction = store.transaction().expect("cohort fixture transaction");
        for &body in bodies.iter().take(cohort_count).skip(1) {
            let vertex = transaction
                .store()
                .vertices_of_body(body)
                .expect("valid body")[0];
            let old_point = transaction.store().get(vertex).expect("valid vertex").point;
            transaction
                .assembly()
                .get_mut(vertex)
                .expect("valid vertex")
                .point = shared;
            transaction
                .assembly()
                .remove(old_point)
                .expect("unreferenced cohort point");
        }
        transaction
            .commit_checked(&[])
            .expect("shared cohort fixture must remain valid");
        Self {
            store,
            bodies: bodies.into_boxed_slice(),
            points: vec![shared].into_boxed_slice(),
        }
    }

    /// Build a deterministic bounded cycle of all implemented solid primitives.
    pub fn primitive_mix(body_count: usize) -> Self {
        let mut store = Store::new();
        let mut bodies = Vec::with_capacity(body_count);
        for ordinal in 0..body_count {
            bodies.push(insert_primitive(&mut store, ordinal));
        }
        Self {
            store,
            bodies: bodies.into_boxed_slice(),
            points: Box::new([]),
        }
    }

    /// Execute one case using the ordinary checked-commit entry point.
    pub fn execute(self, ladder: Ladder) -> TopologyResult {
        self.measure_once(ladder).1
    }

    /// Execute one iteration, returning only transaction/edit/commit duration.
    /// Fixture cloning and every correctness/audit check remain outside it.
    pub fn measure_once(&self, ladder: Ladder) -> (core::time::Duration, TopologyResult) {
        if ladder == Ladder::FullRebuild {
            let mut template = self.prepare_full_rebuild();
            let (elapsed, audit) = self.measure_prepared_full_rebuild();
            template.audit = Some(audit);
            return (elapsed, template);
        }
        let mut fixture = self.clone();
        let rollback_control = (ladder == Ladder::Rejected).then(|| fixture.store.clone());
        let before_store = store_snapshot(&fixture.store);
        let before_index = index_snapshot(&fixture.store);
        let started = std::time::Instant::now();
        let mut rejected = false;
        match ladder {
            Ladder::Clean => {
                fixture
                    .store
                    .transaction()
                    .expect("clean transaction")
                    .commit_checked(&[])
                    .expect("clean checked commit");
            }
            Ladder::FullRebuild => unreachable!("handled by prepared read-only path"),
            Ladder::Local => fixture.edit_points(1),
            Ladder::Fanout => fixture.edit_shared_point(),
            Ladder::Cohort => fixture.edit_shared_point(),
            Ladder::Batched => fixture.edit_points(fixture.bodies.len()),
            Ladder::Rejected => {
                let mut transaction = fixture.store.transaction().expect("rejected transaction");
                transaction
                    .assembly()
                    .get_mut(fixture.bodies[0])
                    .expect("valid body")
                    .regions
                    .clear();
                rejected = transaction.commit_checked(&[]).is_err();
                assert!(rejected, "rejected-edit fixture unexpectedly committed");
            }
        }
        let elapsed = started.elapsed();
        let observation =
            last_commit(&fixture.store).expect("checked commit records an observation");
        let after_store = store_snapshot(&fixture.store);
        let after_index = index_snapshot(&fixture.store);
        if ladder == Ladder::Rejected {
            assert_eq!(after_store, before_store, "rejection changed model state");
            assert_eq!(
                after_index, before_index,
                "rejection changed installed index"
            );
        }
        let future_behavior_equal = rollback_control.is_none_or(|mut control| {
            let mut candidate = fixture.store.clone();
            let point = Point3::new(400.0, 0.0, 0.0);
            let control_body = make::acorn(&mut control, point).expect("future control insert");
            let candidate_body =
                make::acorn(&mut candidate, point).expect("future candidate insert");
            control_body == candidate_body
                && store_snapshot(&control) == store_snapshot(&candidate)
                && index_snapshot(&control) == index_snapshot(&candidate)
        });
        (
            elapsed,
            TopologyResult {
                observation,
                before_store,
                after_store,
                before_index,
                after_index,
                audit: None,
                rejected,
                future_behavior_equal,
            },
        )
    }

    fn prepare_full_rebuild(&self) -> TopologyResult {
        let mut observation_store = self.store.clone();
        observation_store
            .transaction()
            .expect("full-rebuild setup transaction")
            .commit_checked(&[])
            .expect("full-rebuild setup commit");
        let observation =
            last_commit(&observation_store).expect("full-rebuild setup records observation");
        let store = store_snapshot(&self.store);
        let index = index_snapshot(&self.store);
        TopologyResult {
            observation,
            before_store: store,
            after_store: store,
            before_index: index,
            after_index: index,
            audit: Some(audit_full_rebuild(&self.store)),
            rejected: false,
            // The timed operation accepts only `&Store`; mutation and allocator
            // drift are excluded by the type boundary.
            future_behavior_equal: true,
        }
    }

    /// Time only an independent index rebuild over an immutable prepared
    /// fixture, returning the equality evidence for out-of-band verification.
    pub fn measure_prepared_full_rebuild(&self) -> (core::time::Duration, IndexAudit) {
        let started = std::time::Instant::now();
        let audit = audit_full_rebuild(&self.store);
        let elapsed = started.elapsed();
        (elapsed, audit)
    }

    fn edit_points(&mut self, count: usize) {
        let mut transaction = self.store.transaction().expect("edit transaction");
        for ordinal in 0..count {
            let mut assembly = transaction.assembly();
            let point = assembly.get_mut(self.points[ordinal]).expect("valid point");
            point.y = (ordinal + 1) as f64 * 0.25;
        }
        transaction
            .commit_checked(&[])
            .expect("point edits remain valid");
    }

    fn edit_shared_point(&mut self) {
        let mut transaction = self.store.transaction().expect("fanout transaction");
        transaction
            .assembly()
            .get_mut(self.points[0])
            .expect("shared point")
            .z = 0.5;
        transaction
            .commit_checked(&[])
            .expect("shared point edit remains valid");
    }
}

/// Semantic counters and rollback/audit evidence from one iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TopologyResult {
    /// Internal commit counters.
    pub observation: CommitObservation,
    /// Model before the attempt.
    pub before_store: StoreSnapshot,
    /// Model after the attempt.
    pub after_store: StoreSnapshot,
    /// Installed index before the attempt.
    pub before_index: IndexSnapshot,
    /// Installed index after the attempt.
    pub after_index: IndexSnapshot,
    /// Full-rebuild evidence for the reference ladder.
    pub audit: Option<IndexAudit>,
    /// Whether the invalid edit was rejected.
    pub rejected: bool,
    /// Whether rollback restored the allocator and next checked insertion.
    pub future_behavior_equal: bool,
}

impl TopologyResult {
    /// Stable digest over semantic counters and correctness evidence.
    pub fn output_digest(&self) -> u64 {
        fn store(digest: &mut ResultHasher, snapshot: StoreSnapshot) {
            digest.count(snapshot.bodies);
            digest.count(snapshot.regions);
            digest.count(snapshot.shells);
            digest.count(snapshot.faces);
            digest.count(snapshot.loops);
            digest.count(snapshot.fins);
            digest.count(snapshot.edges);
            digest.count(snapshot.vertices);
            digest.count(snapshot.points);
            digest.count(snapshot.curves);
            digest.count(snapshot.surfaces);
            digest.count(snapshot.pcurves);
            digest.u64(snapshot.digest);
        }
        fn index(digest: &mut ResultHasher, snapshot: IndexSnapshot) {
            digest.count(snapshot.bodies);
            digest.count(snapshot.ownership_entries);
            digest.count(snapshot.dependency_entries);
            digest.count(snapshot.ownership_faults);
            digest.u64(snapshot.digest);
        }

        let mut digest = ResultHasher::new();
        digest.tag(0x71);
        digest.boolean(self.observation.committed);
        digest.count(self.observation.body_count);
        digest.count(self.observation.affected_bodies);
        digest.count(self.observation.refreshed_bodies);
        digest.count(self.observation.checked_bodies);
        digest.count(self.observation.mutations);
        digest.u64(self.observation.affected_order_digest);
        store(&mut digest, self.before_store);
        store(&mut digest, self.after_store);
        index(&mut digest, self.before_index);
        index(&mut digest, self.after_index);
        if let Some(audit) = self.audit {
            digest.tag(1);
            index(&mut digest, audit.committed);
            index(&mut digest, audit.rebuilt);
            digest.boolean(audit.structurally_equal);
        } else {
            digest.tag(0);
        }
        digest.boolean(self.rejected);
        digest.boolean(self.future_behavior_equal);
        digest.finish()
    }
}

struct ResultHasher(u64);

impl ResultHasher {
    const fn new() -> Self {
        Self(14_695_981_039_346_656_037)
    }

    fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(1_099_511_628_211);
        }
    }

    fn tag(&mut self, value: u8) {
        self.bytes(&[value]);
    }

    fn boolean(&mut self, value: bool) {
        self.tag(u8::from(value));
    }

    fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    fn count(&mut self, value: usize) {
        self.u64(value as u64);
    }

    const fn finish(self) -> u64 {
        self.0
    }
}

/// Construct the fixture required by a case.
pub fn fixture(case: TopologyCase) -> TopologyFixture {
    match case.ladder {
        Ladder::Fanout => TopologyFixture::shared_geometry_fanout(case.bodies),
        Ladder::Cohort => {
            TopologyFixture::mixed_store_shared_point_cohort(case.bodies, case.affected_bodies)
        }
        Ladder::FullRebuild => TopologyFixture::primitive_mix(case.bodies),
        _ => TopologyFixture::isolated_acorns(case.bodies),
    }
}

/// Check exact scale-dependent counters before accepting a timed sample.
pub fn verify(case: TopologyCase, result: &TopologyResult) {
    let expected_scope = expected_scope(case);
    assert_eq!(result.observation.body_count, case.bodies);
    assert_eq!(result.observation.affected_bodies, expected_scope);
    assert_eq!(result.observation.refreshed_bodies, expected_scope);
    assert_eq!(result.observation.checked_bodies, expected_scope);
    let expected_mutations = expected_mutations(case);
    assert_eq!(result.observation.mutations, expected_mutations);
    assert_eq!(
        result.observation.committed,
        case.ladder != Ladder::Rejected
    );
    assert_eq!(
        result.observation.affected_order_digest,
        case.expected_affected_digest
    );
    assert_ne!(case.expected_output_digest, 0);
    assert_eq!(result.output_digest(), case.expected_output_digest);
    assert_eq!(result.rejected, case.ladder == Ladder::Rejected);
    assert!(result.future_behavior_equal);
    assert_eq!(
        result.before_index, result.after_index,
        "Q2 edits must not perturb index entries outside refreshed scope"
    );
    if let Some(audit) = result.audit {
        assert!(audit.structurally_equal);
        assert_eq!(audit.committed, audit.rebuilt);
    }
    if case.ladder == Ladder::Rejected {
        assert_eq!(result.before_store, result.after_store);
        assert_eq!(result.before_index, result.after_index);
    }
}

/// Verify the exact read-only evidence produced by a full-rebuild sample.
pub fn verify_full_rebuild(case: TopologyCase, audit: IndexAudit) {
    assert_eq!(case.ladder, Ladder::FullRebuild);
    assert!(audit.structurally_equal);
    assert_eq!(audit.committed, audit.rebuilt);
    assert_eq!(audit.committed.bodies, case.bodies);
    assert_eq!(audit.committed.ownership_faults, 0);
}

const fn expected_scope(case: TopologyCase) -> usize {
    case.affected_bodies
}

const fn expected_mutations(case: TopologyCase) -> usize {
    match case.ladder {
        Ladder::Clean | Ladder::FullRebuild => 0,
        Ladder::Local | Ladder::Fanout | Ladder::Cohort | Ladder::Rejected => 1,
        Ladder::Batched => case.bodies,
    }
}

fn insert_primitive(store: &mut Store, ordinal: usize) -> BodyId {
    let origin = Point3::new(
        (ordinal % 10) as f64 * 8.0 - 36.0,
        ((ordinal / 10) % 10) as f64 * 8.0 - 36.0,
        (ordinal / 100) as f64 * 8.0 - 36.0,
    );
    let frame = Frame::from_z(origin, Frame::world().z()).expect("valid fixture frame");
    match ordinal % 5 {
        0 => make::block(store, &frame, [1.0, 2.0, 3.0]),
        1 => make::cylinder(store, &frame, 1.0, 2.0),
        2 => make::cone(store, &frame, 1.0, 0.5, 2.0),
        3 => make::sphere(store, &frame, 1.0),
        _ => make::torus(store, &frame, 2.0, 0.5),
    }
    .expect("Q2 primitive fixture must be valid")
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::surface::Sphere;
    use ktopo::geom::SurfaceGeom;
    use std::collections::BTreeSet;

    #[test]
    fn registry_contains_exactly_28_unique_canonical_cases() {
        assert_eq!(CASES.len(), 28);
        let unique: BTreeSet<_> = CASES.iter().map(|case| case.path).collect();
        assert_eq!(unique.len(), CASES.len());
        for case in CASES {
            crate::validate_case_path(case.path).unwrap();
            assert_eq!(include_str!("../cases.json").matches(case.path).count(), 1);
        }
        assert_eq!(
            include_str!("../cases.json")
                .matches("\"benchmark_target\": \"topology_commit\"")
                .count(),
            28
        );
    }

    #[test]
    fn json_registry_matches_rust_cases_and_reviewed_counters() {
        let manifest: serde_json::Value =
            serde_json::from_str(include_str!("../cases.json")).unwrap();
        let entries = manifest["cases"].as_array().unwrap();
        for case in CASES {
            let matches: Vec<_> = entries
                .iter()
                .filter(|entry| entry["path"] == case.path)
                .collect();
            assert_eq!(matches.len(), 1, "registry mismatch for {}", case.path);
            let entry = matches[0];
            assert_eq!(entry["benchmark_target"], "topology_commit");
            let fixture_version = if case.ladder == Ladder::Cohort {
                COHORT_FIXTURE_VERSION
            } else {
                FIXTURE_VERSION
            };
            assert_eq!(entry["fixture_version"], fixture_version);
            assert_eq!(entry["deterministic_seed"].as_u64(), Some(FIXTURE_SEED));
            assert_eq!(
                entry["size_parameters"]["elements"].as_u64(),
                Some(case.bodies as u64)
            );
            assert_eq!(
                entry["size_parameters"]["bodies"].as_u64(),
                Some(case.bodies as u64)
            );
            if case.ladder == Ladder::Cohort {
                assert_eq!(
                    entry["size_parameters"]["affected_bodies"].as_u64(),
                    Some(case.affected_bodies as u64)
                );
            }
            let counters = &entry["expected_result_counters"];
            assert_eq!(counters["body_count"].as_u64(), Some(case.bodies as u64));
            assert_eq!(
                counters["affected_bodies"].as_u64(),
                Some(expected_scope(case) as u64)
            );
            assert_eq!(
                counters["refreshed_bodies"].as_u64(),
                Some(expected_scope(case) as u64)
            );
            assert_eq!(
                counters["checked_bodies"].as_u64(),
                Some(expected_scope(case) as u64)
            );
            assert_eq!(
                counters["mutations"].as_u64(),
                Some(expected_mutations(case) as u64)
            );
            assert_eq!(
                counters["committed"].as_bool(),
                Some(case.ladder != Ladder::Rejected)
            );
            assert_eq!(
                counters["affected_order_digest"].as_str(),
                Some(format!("{:016x}", case.expected_affected_digest).as_str())
            );
            assert_eq!(
                counters["output_digest"].as_str(),
                Some(format!("{:016x}", case.expected_output_digest).as_str())
            );
        }
    }

    #[test]
    fn all_seven_smallest_ladders_match_reviewed_result_evidence() {
        for case in [
            CASES[0], CASES[4], CASES[8], CASES[11], CASES[14], CASES[17], CASES[21],
        ] {
            let result = fixture(case).execute(case.ladder);
            verify(case, &result);
        }
    }

    #[test]
    fn cohort_matrix_separates_total_bodies_from_affected_roots() {
        let cohort = CASES
            .iter()
            .filter(|case| case.ladder == Ladder::Cohort)
            .collect::<Vec<_>>();
        assert_eq!(cohort.len(), 7);
        let fixed_affected = cohort
            .iter()
            .filter(|case| case.affected_bodies == 4)
            .map(|case| case.bodies)
            .collect::<BTreeSet<_>>();
        assert_eq!(fixed_affected, BTreeSet::from([4, 16, 64, 256]));
        let fixed_total = cohort
            .iter()
            .filter(|case| case.bodies == 64)
            .map(|case| case.affected_bodies)
            .collect::<BTreeSet<_>>();
        assert_eq!(fixed_total, BTreeSet::from([1, 4, 16, 64]));
    }

    #[test]
    fn every_cohort_case_matches_reviewed_scope_and_digest_evidence() {
        for case in CASES
            .iter()
            .copied()
            .filter(|case| case.ladder == Ladder::Cohort)
        {
            let result = fixture(case).execute(case.ladder);
            verify(case, &result);
        }
    }

    #[cfg(not(debug_assertions))]
    #[test]
    fn every_registered_case_matches_reviewed_result_evidence_in_release() {
        for case in CASES {
            let result = fixture(case).execute(case.ladder);
            verify(case, &result);
        }
    }

    #[test]
    fn model_digest_observes_topology_relationship_changes() {
        let mut fixture = TopologyFixture::isolated_acorns(1);
        let original = store_snapshot(&fixture.store);
        let body = fixture.bodies[0];
        let mut transaction = fixture.store.transaction().unwrap();
        transaction
            .assembly()
            .get_mut(body)
            .unwrap()
            .regions
            .clear();
        let mut mutated = transaction.store().clone();
        drop(transaction);
        // Re-establish a candidate index in the clone without committing the
        // deliberately invalid topology. This leaves the payload available
        // for a read-only digest and never changes the original fixture.
        drop(mutated.transaction().unwrap());
        let changed = store_snapshot(&mutated);
        assert_ne!(original.digest, changed.digest);
        assert_eq!(original, store_snapshot(&fixture.store));
    }

    #[test]
    fn model_digest_observes_geometry_descriptor_payload_changes() {
        let mut fixture = TopologyFixture::primitive_mix(4);
        let original = store_snapshot(&fixture.store);
        let body = fixture.bodies[3];
        let face = fixture.store.faces_of_body(body).unwrap()[0];
        let surface = fixture.store.get(face).unwrap().surface;
        let mut transaction = fixture.store.transaction().unwrap();
        transaction
            .assembly()
            .replace_surface(
                surface,
                SurfaceGeom::Sphere(Sphere::new(Frame::world(), 2.0).unwrap()),
            )
            .unwrap();
        transaction.commit_checked_body(body).unwrap();
        assert_ne!(original.digest, store_snapshot(&fixture.store).digest);
    }

    #[test]
    fn rejected_attempt_restores_future_allocation_behavior() {
        let fixture = TopologyFixture::isolated_acorns(10);
        let mut control = fixture.clone().store;
        let mut rejected = fixture.store;
        let body = rejected.iter::<ktopo::entity::Body>().next().unwrap().0;
        let mut transaction = rejected.transaction().unwrap();
        transaction
            .assembly()
            .get_mut(body)
            .unwrap()
            .regions
            .clear();
        assert!(transaction.commit_checked(&[]).is_err());
        let control_body = make::acorn(&mut control, Point3::new(99.0, 0.0, 0.0)).unwrap();
        let future_body = make::acorn(&mut rejected, Point3::new(99.0, 0.0, 0.0)).unwrap();
        assert_eq!(format!("{control_body:?}"), format!("{future_body:?}"));
        assert_eq!(store_snapshot(&control), store_snapshot(&rejected));
        assert_eq!(index_snapshot(&control), index_snapshot(&rejected));
    }

    #[test]
    fn primitive_mix_and_full_audit_are_deterministic() {
        let a = TopologyFixture::primitive_mix(10);
        let b = TopologyFixture::primitive_mix(10);
        assert_eq!(a.store.count::<ktopo::entity::Body>(), 10);
        assert_eq!(store_snapshot(&a.store), store_snapshot(&b.store));
        assert!(audit_full_rebuild(&a.store).structurally_equal);
    }

    #[test]
    fn prepared_full_rebuild_path_matches_complete_result_evidence() {
        let case = CASES
            .iter()
            .copied()
            .find(|case| case.ladder == Ladder::FullRebuild && case.bodies == 10)
            .unwrap();
        let fixture = fixture(case);
        let template = fixture.measure_once(case.ladder).1;
        let (_, audit) = fixture.measure_prepared_full_rebuild();
        assert_eq!(Some(audit), template.audit);
        verify_full_rebuild(case, audit);
    }
}
