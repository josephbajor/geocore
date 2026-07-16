//! Deterministic Q2 topology fixtures and checked-commit operations.

use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::frame::Frame;
use kgeom::vec::Point3;
use ktopo::benchmark::{
    CommitObservation, IndexAudit, IndexSnapshot, StoreSnapshot, audit_full_rebuild,
    index_snapshot, last_commit, store_snapshot,
};
use ktopo::entity::{BodyId, EdgeId, FaceId, PointId, VertexId};
use ktopo::make;
use ktopo::store::Store;

/// Fixture identity shared by all Q2 cases.
pub const FIXTURE_VERSION: &str = "topology-commit.v1";
/// Fixture identity for the mixed-store affected-root scaling matrix.
pub const COHORT_FIXTURE_VERSION: &str = "topology-commit-cohort.v2";
/// Fixture identity for the production-solid affected-footprint matrix.
pub const AFFECTED_SOLID_FIXTURE_VERSION: &str = "topology-commit-affected-solid-footprint.v1";
/// Fixture identity for the block-cohort production-edit footprint matrix.
pub const AFFECTED_BLOCK_COHORT_FIXTURE_VERSION: &str =
    "topology-commit-affected-block-cohort-footprint.v1";
/// Fixture identity for the production-solid no-op ordinary-commit ladder.
pub const PRODUCTION_CLEAN_FIXTURE_VERSION: &str = "topology-commit-production-clean.v1";
/// Deterministic fixture seed (construction itself is not randomized).
pub const FIXTURE_SEED: u64 = 0x5154_4f50_4f00_0002;

const AFFECTED_SOLID_OPERATION: &str = "q2-affected-solid-footprint";
const AFFECTED_SOLID_FACE_TOLERANCE: f64 = 2.0 * LINEAR_RESOLUTION;
const AFFECTED_SOLID_GROWTH_PER_FACE: f64 = AFFECTED_SOLID_FACE_TOLERANCE - LINEAR_RESOLUTION;
const AFFECTED_BLOCK_COHORT_OPERATION: &str = "q2-affected-block-cohort-footprint";
const AFFECTED_BLOCK_COHORT_TOLERANCE: f64 = 2.0 * LINEAR_RESOLUTION;
const AFFECTED_BLOCK_COHORT_GROWTH_PER_ENTITY: f64 =
    AFFECTED_BLOCK_COHORT_TOLERANCE - LINEAR_RESOLUTION;

fn affected_block_cohort_growth_budget(affected_bodies: usize) -> f64 {
    (0..3 * affected_bodies).fold(0.0, |sum, _| sum + AFFECTED_BLOCK_COHORT_GROWTH_PER_ENTITY)
}

/// One of the ten Q2 benchmark ladders.
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
    /// Grow one face tolerance on each selected production solid.
    AffectedSolidFootprint,
    /// Grow one face, edge, and vertex tolerance per selected block root.
    AffectedBlockCohortFootprint,
    /// Commit an unchanged production-solid mix through the ordinary path.
    ProductionClean,
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
    /// Reviewed semantic store digest before a ratcheted production commit, or zero.
    pub expected_before_store_digest: u64,
    /// Reviewed semantic store digest after a ratcheted production commit, or zero.
    pub expected_after_store_digest: u64,
    /// Reviewed installed-index digest before a production-clean commit, or zero.
    pub expected_before_index_digest: u64,
    /// Reviewed installed-index digest after a production-clean commit, or zero.
    pub expected_after_index_digest: u64,
    /// Reviewed digest of complete semantic result evidence.
    pub expected_output_digest: u64,
}

/// Exactly the 43 Q2 cases specified by the quality contract.
pub const CASES: [TopologyCase; 43] = [
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
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-4/affected-1-v1",
        4,
        1,
        0x0300_00b9_a2c2_19c6,
        0x94a7_8b9a_2c4e_e0b3,
        0x4c0e_9e1e_bb7f_c5b0,
        0x500a_aa06_4584_bd77,
    ),
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-16/affected-1-v1",
        16,
        1,
        0x0300_00b9_a2c2_19c6,
        0x5fc1_edc7_4231_81f4,
        0x749d_d9b2_6445_a1d1,
        0x8bb3_c70c_4457_d9d8,
    ),
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-64/affected-1-v1",
        64,
        1,
        0x0300_00b9_a2c2_19c6,
        0x4bbf_07cb_a16f_69f6,
        0x4a1b_2ee5_e725_2b29,
        0x01e4_b75d_9cef_4bd4,
    ),
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-256/affected-1-v1",
        256,
        1,
        0x0300_00b9_a2c2_19c6,
        0x55b8_ac42_b1da_1110,
        0xcc8f_a6e2_35df_51b9,
        0x8273_88cd_a355_f89a,
    ),
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-64/affected-4-v1",
        64,
        4,
        0xd8de_f909_5c13_1d26,
        0x4bbf_07cb_a16f_69f6,
        0x0564_f99e_7777_2452,
        0x4a2a_2291_e64f_7ff5,
    ),
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-64/affected-16-v1",
        64,
        16,
        0x121b_7067_4cb3_6c32,
        0x4bbf_07cb_a16f_69f6,
        0x4e0f_0626_fb12_df52,
        0xc115_dc5c_8f14_48b7,
    ),
    affected_solid_case(
        "topology/affected-solid-footprint/primitive-mix-v1/total-64/affected-64-v1",
        64,
        64,
        0xac06_c657_a252_9702,
        0x4bbf_07cb_a16f_69f6,
        0x0076_7ca8_95fc_280a,
        0x18e3_f6db_49e4_dce0,
    ),
    affected_block_cohort_case(
        "topology/affected-block-cohort/primitive-mix-blocks-v1/total-64/affected-1-v1",
        1,
        0x0300_00b9_a2c2_19c6,
        0x4bbf_07cb_a16f_69f6,
        0xf321_73bd_c29e_d2a5,
        0xdcc3_867d_ead5_72d5,
    ),
    affected_block_cohort_case(
        "topology/affected-block-cohort/primitive-mix-blocks-v1/total-64/affected-4-v1",
        4,
        0x629c_ebb9_7362_0ee6,
        0x4bbf_07cb_a16f_69f6,
        0x906e_8305_50aa_9016,
        0x4cea_0f83_69a3_8358,
    ),
    affected_block_cohort_case(
        "topology/affected-block-cohort/primitive-mix-blocks-v1/total-64/affected-8-v1",
        8,
        0x35ee_f1d6_1786_234a,
        0x4bbf_07cb_a16f_69f6,
        0x1a00_c485_06bf_a116,
        0x4ba5_d8a7_55d7_7654,
    ),
    affected_block_cohort_case(
        "topology/affected-block-cohort/primitive-mix-blocks-v1/total-64/affected-13-v1",
        13,
        0x7600_3dd5_2e0a_7ea2,
        0x4bbf_07cb_a16f_69f6,
        0xf236_4103_47e5_5af1,
        0x74af_6f11_72cb_c20a,
    ),
    production_clean_case(
        "topology/production-clean/primitive-mix-v1/total-4/ordinary-noop-v1",
        4,
        0x94a7_8b9a_2c4e_e0b3,
        0x940a_2fb1_7674_ba69,
        0xb597_82ce_63d0_ce2f,
    ),
    production_clean_case(
        "topology/production-clean/primitive-mix-v1/total-16/ordinary-noop-v1",
        16,
        0x5fc1_edc7_4231_81f4,
        0x76c1_174d_73fb_5880,
        0x1f88_5469_ef31_5b53,
    ),
    production_clean_case(
        "topology/production-clean/primitive-mix-v1/total-64/ordinary-noop-v1",
        64,
        0x4bbf_07cb_a16f_69f6,
        0xdc20_0696_664d_93ad,
        0xd153_e833_70f7_4247,
    ),
    production_clean_case(
        "topology/production-clean/primitive-mix-v1/total-256/ordinary-noop-v1",
        256,
        0x55b8_ac42_b1da_1110,
        0x3755_ea7b_5497_afed,
        0x160c_2bf6_7635_e5ee,
    ),
];

const fn case(path: &'static str, ladder: Ladder, bodies: usize) -> TopologyCase {
    let expected_affected_digest = match ladder {
        Ladder::Clean | Ladder::ProductionClean | Ladder::FullRebuild => 0x8aa9_84d6_2998_05c2,
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
            Ladder::Clean | Ladder::ProductionClean | Ladder::FullRebuild => 0,
            Ladder::Local | Ladder::Rejected => 1,
            Ladder::Fanout | Ladder::Batched => bodies,
            Ladder::Cohort
            | Ladder::AffectedSolidFootprint
            | Ladder::AffectedBlockCohortFootprint => 0,
        },
        expected_affected_digest,
        expected_before_store_digest: 0,
        expected_after_store_digest: 0,
        expected_before_index_digest: 0,
        expected_after_index_digest: 0,
        expected_output_digest,
    }
}

const fn production_clean_case(
    path: &'static str,
    bodies: usize,
    expected_store_digest: u64,
    expected_index_digest: u64,
    expected_output_digest: u64,
) -> TopologyCase {
    TopologyCase {
        path,
        ladder: Ladder::ProductionClean,
        bodies,
        affected_bodies: 0,
        expected_affected_digest: 0x8aa9_84d6_2998_05c2,
        expected_before_store_digest: expected_store_digest,
        expected_after_store_digest: expected_store_digest,
        expected_before_index_digest: expected_index_digest,
        expected_after_index_digest: expected_index_digest,
        expected_output_digest,
    }
}

const fn affected_block_cohort_case(
    path: &'static str,
    affected_bodies: usize,
    expected_affected_digest: u64,
    expected_before_store_digest: u64,
    expected_after_store_digest: u64,
    expected_output_digest: u64,
) -> TopologyCase {
    TopologyCase {
        path,
        ladder: Ladder::AffectedBlockCohortFootprint,
        bodies: 64,
        affected_bodies,
        expected_affected_digest,
        expected_before_store_digest,
        expected_after_store_digest,
        expected_before_index_digest: 0,
        expected_after_index_digest: 0,
        expected_output_digest,
    }
}

const fn affected_solid_case(
    path: &'static str,
    bodies: usize,
    affected_bodies: usize,
    expected_affected_digest: u64,
    expected_before_store_digest: u64,
    expected_after_store_digest: u64,
    expected_output_digest: u64,
) -> TopologyCase {
    TopologyCase {
        path,
        ladder: Ladder::AffectedSolidFootprint,
        bodies,
        affected_bodies,
        expected_affected_digest,
        expected_before_store_digest,
        expected_after_store_digest,
        expected_before_index_digest: 0,
        expected_after_index_digest: 0,
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
        expected_before_store_digest: 0,
        expected_after_store_digest: 0,
        expected_before_index_digest: 0,
        expected_after_index_digest: 0,
        expected_output_digest,
    }
}

/// Fully constructed Q2 fixture. Clone it outside timed work for each sample.
#[derive(Clone)]
pub struct TopologyFixture {
    store: Store,
    bodies: Box<[BodyId]>,
    points: Box<[PointId]>,
    affected_faces: Box<[FaceId]>,
    affected_edges: Box<[EdgeId]>,
    affected_vertices: Box<[VertexId]>,
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
            affected_faces: Box::new([]),
            affected_edges: Box::new([]),
            affected_vertices: Box::new([]),
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
            affected_faces: Box::new([]),
            affected_edges: Box::new([]),
            affected_vertices: Box::new([]),
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
            affected_faces: Box::new([]),
            affected_edges: Box::new([]),
            affected_vertices: Box::new([]),
        }
    }

    /// Build a reviewed production-solid total and select one deterministic
    /// face per affected root without changing ordinary primitive construction.
    pub fn primitive_mix_affected_solid_footprint(
        body_count: usize,
        affected_bodies: usize,
    ) -> Self {
        assert!(matches!(body_count, 4 | 16 | 64 | 256));
        assert!(matches!(affected_bodies, 1 | 4 | 16 | 64));
        assert!(affected_bodies <= body_count);
        let mut fixture = Self::primitive_mix(body_count);
        fixture.affected_faces = fixture
            .bodies
            .iter()
            .take(affected_bodies)
            .map(|&body| {
                fixture
                    .store
                    .faces_of_body(body)
                    .expect("valid Q2 production solid")[0]
            })
            .collect::<Vec<_>>()
            .into_boxed_slice();
        fixture
    }

    /// Build the unchanged 64-body production mix and select only block roots
    /// at primitive ordinals divisible by five.
    pub fn primitive_mix_affected_block_cohort(affected_bodies: usize) -> Self {
        assert!(matches!(affected_bodies, 1 | 4 | 8 | 13));
        let mut fixture = Self::primitive_mix(64);
        let block_roots = fixture
            .bodies
            .iter()
            .copied()
            .enumerate()
            .filter_map(|(ordinal, body)| (ordinal % 5 == 0).then_some(body))
            .collect::<Vec<_>>();
        assert_eq!(block_roots.len(), 13);

        let mut faces = Vec::with_capacity(affected_bodies);
        let mut edges = Vec::with_capacity(affected_bodies);
        let mut vertices = Vec::with_capacity(affected_bodies);
        for body in block_roots.into_iter().take(affected_bodies) {
            let body_faces = fixture
                .store
                .faces_of_body(body)
                .expect("valid Q2 block cohort body");
            let body_edges = fixture
                .store
                .edges_of_body(body)
                .expect("valid Q2 block cohort body");
            let body_vertices = fixture
                .store
                .vertices_of_body(body)
                .expect("valid Q2 block cohort body");
            assert_eq!(body_faces.len(), 6);
            assert_eq!(body_edges.len(), 12);
            assert_eq!(body_vertices.len(), 8);
            faces.push(body_faces[0]);
            edges.push(body_edges[0]);
            vertices.push(body_vertices[0]);
        }
        fixture.affected_faces = faces.into_boxed_slice();
        fixture.affected_edges = edges.into_boxed_slice();
        fixture.affected_vertices = vertices.into_boxed_slice();
        fixture
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
        let mut affected_solid_journal = None;
        match ladder {
            Ladder::Clean | Ladder::ProductionClean => {
                fixture
                    .store
                    .transaction()
                    .expect("clean ordinary transaction")
                    .commit_checked(&[])
                    .expect("clean ordinary checked commit");
            }
            Ladder::FullRebuild => unreachable!("handled by prepared read-only path"),
            Ladder::Local => fixture.edit_points(1),
            Ladder::Fanout => fixture.edit_shared_point(),
            Ladder::Cohort => fixture.edit_shared_point(),
            Ladder::AffectedSolidFootprint => {
                affected_solid_journal = Some(fixture.grow_affected_face_tolerances());
            }
            Ladder::AffectedBlockCohortFootprint => {
                affected_solid_journal = Some(fixture.grow_affected_block_cohort_tolerances());
            }
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
        if let Some(journal) = affected_solid_journal.as_ref() {
            match ladder {
                Ladder::AffectedSolidFootprint => fixture.verify_affected_solid_journal(journal),
                Ladder::AffectedBlockCohortFootprint => {
                    fixture.verify_affected_block_cohort_journal(journal);
                }
                _ => unreachable!("only tolerance ladders retain journals"),
            }
        }
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

    fn grow_affected_face_tolerances(&mut self) -> ktopo::transaction::Journal {
        let mut transaction = self
            .store
            .transaction()
            .expect("affected-solid transaction");
        let budget = transaction
            .declare_tolerance_budget(
                AFFECTED_SOLID_OPERATION,
                self.affected_faces.len() as f64 * AFFECTED_SOLID_GROWTH_PER_FACE,
            )
            .expect("fixed Q2 tolerance-growth budget");
        for &face in self.affected_faces.iter() {
            transaction
                .grow_face_tolerance(budget, face, AFFECTED_SOLID_FACE_TOLERANCE)
                .expect("Q2 production face tolerance growth");
        }
        transaction
            .commit_checked(&[])
            .expect("affected production solids remain valid")
    }

    fn verify_affected_solid_journal(&self, journal: &ktopo::transaction::Journal) {
        use ktopo::entity::EntityRef;
        use ktopo::tolerance::ToleranceOrigin;
        use ktopo::transaction::MutationKind;

        assert_eq!(journal.mutations().len(), self.affected_faces.len());
        assert!(journal.mutations().iter().all(|mutation| {
            mutation.kind == MutationKind::Modified && matches!(mutation.entity, EntityRef::Face(_))
        }));
        let budgets = journal.tolerance_budgets();
        assert_eq!(budgets.len(), 1);
        let expected_growth = self.affected_faces.len() as f64 * AFFECTED_SOLID_GROWTH_PER_FACE;
        assert_eq!(budgets[0].operation(), AFFECTED_SOLID_OPERATION);
        assert_eq!(budgets[0].limit(), expected_growth);
        assert_eq!(budgets[0].consumed(), expected_growth);
        assert_eq!(budgets[0].remaining(), 0.0);
        assert_eq!(journal.tolerance_events().len(), self.affected_faces.len());
        for (event, &face) in journal
            .tolerance_events()
            .iter()
            .zip(self.affected_faces.iter())
        {
            assert_eq!(event.entity(), EntityRef::Face(face));
            assert_eq!(event.previous(), None);
            let tolerance = event.current();
            assert_eq!(tolerance.value(), AFFECTED_SOLID_FACE_TOLERANCE);
            assert_eq!(
                tolerance.origin(),
                ToleranceOrigin::Operation(AFFECTED_SOLID_OPERATION)
            );
            assert_eq!(tolerance.origin_value(), AFFECTED_SOLID_FACE_TOLERANCE);
            assert_eq!(tolerance.accumulated_growth(), 0.0);
            assert_eq!(tolerance.last_operation(), Some(AFFECTED_SOLID_OPERATION));
            assert_eq!(self.store.get(face).unwrap().tolerance, Some(tolerance));
        }
    }

    fn grow_affected_block_cohort_tolerances(&mut self) -> ktopo::transaction::Journal {
        let mut transaction = self
            .store
            .transaction()
            .expect("affected block-cohort transaction");
        let expected_growth = affected_block_cohort_growth_budget(self.affected_faces.len());
        let budget = transaction
            .declare_tolerance_budget(AFFECTED_BLOCK_COHORT_OPERATION, expected_growth)
            .expect("fixed Q2 block-cohort tolerance-growth budget");
        for ((&face, &edge), &vertex) in self
            .affected_faces
            .iter()
            .zip(self.affected_edges.iter())
            .zip(self.affected_vertices.iter())
        {
            transaction
                .grow_face_tolerance(budget, face, AFFECTED_BLOCK_COHORT_TOLERANCE)
                .expect("Q2 block-cohort face tolerance growth");
            transaction
                .grow_edge_tolerance(budget, edge, AFFECTED_BLOCK_COHORT_TOLERANCE)
                .expect("Q2 block-cohort edge tolerance growth");
            transaction
                .grow_vertex_tolerance(budget, vertex, AFFECTED_BLOCK_COHORT_TOLERANCE)
                .expect("Q2 block-cohort vertex tolerance growth");
        }
        transaction
            .commit_checked(&[])
            .expect("affected block-cohort solids remain valid")
    }

    fn verify_affected_block_cohort_journal(&self, journal: &ktopo::transaction::Journal) {
        use ktopo::entity::EntityRef;
        use ktopo::tolerance::ToleranceOrigin;
        use ktopo::transaction::MutationKind;

        let expected_entities = self
            .affected_faces
            .iter()
            .zip(self.affected_edges.iter())
            .zip(self.affected_vertices.iter())
            .flat_map(|((&face, &edge), &vertex)| {
                [
                    EntityRef::Face(face),
                    EntityRef::Edge(edge),
                    EntityRef::Vertex(vertex),
                ]
            })
            .collect::<Vec<_>>();
        assert_eq!(expected_entities.len(), 3 * self.affected_faces.len());
        assert_eq!(journal.mutations().len(), expected_entities.len());
        assert!(journal.mutations().iter().all(|mutation| {
            mutation.kind == MutationKind::Modified && expected_entities.contains(&mutation.entity)
        }));
        assert_eq!(
            journal
                .mutations()
                .iter()
                .filter(|mutation| matches!(mutation.entity, EntityRef::Face(_)))
                .count(),
            self.affected_faces.len()
        );
        assert_eq!(
            journal
                .mutations()
                .iter()
                .filter(|mutation| matches!(mutation.entity, EntityRef::Edge(_)))
                .count(),
            self.affected_faces.len()
        );
        assert_eq!(
            journal
                .mutations()
                .iter()
                .filter(|mutation| matches!(mutation.entity, EntityRef::Vertex(_)))
                .count(),
            self.affected_faces.len()
        );

        let budgets = journal.tolerance_budgets();
        assert_eq!(budgets.len(), 1);
        let expected_growth = affected_block_cohort_growth_budget(self.affected_faces.len());
        assert_eq!(budgets[0].operation(), AFFECTED_BLOCK_COHORT_OPERATION);
        assert_eq!(budgets[0].limit(), expected_growth);
        assert_eq!(budgets[0].consumed(), expected_growth);
        assert_eq!(budgets[0].remaining(), 0.0);

        assert_eq!(journal.tolerance_events().len(), expected_entities.len());
        for (event, expected_entity) in journal
            .tolerance_events()
            .iter()
            .zip(expected_entities.into_iter())
        {
            assert_eq!(event.entity(), expected_entity);
            assert_eq!(event.previous(), None);
            let tolerance = event.current();
            assert_eq!(tolerance.value(), AFFECTED_BLOCK_COHORT_TOLERANCE);
            assert_eq!(
                tolerance.origin(),
                ToleranceOrigin::Operation(AFFECTED_BLOCK_COHORT_OPERATION)
            );
            assert_eq!(tolerance.origin_value(), AFFECTED_BLOCK_COHORT_TOLERANCE);
            assert_eq!(tolerance.accumulated_growth(), 0.0);
            assert_eq!(
                tolerance.last_operation(),
                Some(AFFECTED_BLOCK_COHORT_OPERATION)
            );
            let installed = match expected_entity {
                EntityRef::Face(face) => self.store.get(face).unwrap().tolerance,
                EntityRef::Edge(edge) => self.store.get(edge).unwrap().tolerance,
                EntityRef::Vertex(vertex) => self.store.get(vertex).unwrap().tolerance,
                _ => unreachable!("block-cohort targets are Face, Edge, or Vertex"),
            };
            assert_eq!(installed, Some(tolerance));
        }
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
        Ladder::AffectedSolidFootprint => TopologyFixture::primitive_mix_affected_solid_footprint(
            case.bodies,
            case.affected_bodies,
        ),
        Ladder::AffectedBlockCohortFootprint => {
            TopologyFixture::primitive_mix_affected_block_cohort(case.affected_bodies)
        }
        Ladder::ProductionClean | Ladder::FullRebuild => {
            TopologyFixture::primitive_mix(case.bodies)
        }
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
    if matches!(
        case.ladder,
        Ladder::AffectedSolidFootprint
            | Ladder::AffectedBlockCohortFootprint
            | Ladder::ProductionClean
    ) {
        assert_ne!(case.expected_before_store_digest, 0);
        assert_ne!(case.expected_after_store_digest, 0);
        assert_eq!(
            result.before_store.digest,
            case.expected_before_store_digest
        );
        assert_eq!(result.after_store.digest, case.expected_after_store_digest);
    }
    if case.ladder == Ladder::ProductionClean {
        assert_ne!(case.expected_before_index_digest, 0);
        assert_ne!(case.expected_after_index_digest, 0);
        assert_eq!(
            result.before_index.digest,
            case.expected_before_index_digest
        );
        assert_eq!(result.after_index.digest, case.expected_after_index_digest);
        assert_eq!(result.before_store, result.after_store);
        assert_eq!(result.before_index, result.after_index);
    }
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
        Ladder::Clean | Ladder::ProductionClean | Ladder::FullRebuild => 0,
        Ladder::Local | Ladder::Fanout | Ladder::Cohort | Ladder::Rejected => 1,
        Ladder::AffectedSolidFootprint => case.affected_bodies,
        Ladder::AffectedBlockCohortFootprint => 3 * case.affected_bodies,
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
    fn registry_contains_exactly_43_unique_canonical_cases() {
        assert_eq!(CASES.len(), 43);
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
            43
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
            let fixture_version = match case.ladder {
                Ladder::Cohort => COHORT_FIXTURE_VERSION,
                Ladder::AffectedSolidFootprint => AFFECTED_SOLID_FIXTURE_VERSION,
                Ladder::AffectedBlockCohortFootprint => AFFECTED_BLOCK_COHORT_FIXTURE_VERSION,
                Ladder::ProductionClean => PRODUCTION_CLEAN_FIXTURE_VERSION,
                _ => FIXTURE_VERSION,
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
            if matches!(
                case.ladder,
                Ladder::Cohort
                    | Ladder::AffectedSolidFootprint
                    | Ladder::AffectedBlockCohortFootprint
                    | Ladder::ProductionClean
            ) {
                assert_eq!(
                    entry["size_parameters"]["affected_bodies"].as_u64(),
                    Some(case.affected_bodies as u64)
                );
            }
            if case.ladder == Ladder::AffectedSolidFootprint {
                assert_eq!(
                    entry["tolerances"]["requested_face_tolerance"].as_f64(),
                    Some(AFFECTED_SOLID_FACE_TOLERANCE)
                );
                assert_eq!(
                    entry["tolerances"]["aggregate_growth_budget"].as_f64(),
                    Some(case.affected_bodies as f64 * AFFECTED_SOLID_GROWTH_PER_FACE)
                );
                assert_eq!(entry["policy_values"]["checked_commit"], "ordinary");
                assert_eq!(
                    entry["policy_values"]["operation_scope"],
                    "single-ordinary-checked-commit"
                );
                assert_eq!(
                    entry["policy_values"]["mutation"],
                    "operation-owned-face-tolerance-growth"
                );
                assert_eq!(
                    entry["policy_values"]["target_order"],
                    "first-body-first-face"
                );
                assert_eq!(
                    entry["expected_result_counters"]["before_store_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_before_store_digest).as_str())
                );
                assert_eq!(
                    entry["expected_result_counters"]["after_store_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_after_store_digest).as_str())
                );
            }
            if case.ladder == Ladder::AffectedBlockCohortFootprint {
                for key in [
                    "requested_face_tolerance",
                    "requested_edge_tolerance",
                    "requested_vertex_tolerance",
                ] {
                    assert_eq!(
                        entry["tolerances"][key].as_f64(),
                        Some(AFFECTED_BLOCK_COHORT_TOLERANCE)
                    );
                }
                assert_eq!(
                    entry["tolerances"]["aggregate_growth_budget"].as_f64(),
                    Some(affected_block_cohort_growth_budget(case.affected_bodies))
                );
                assert_eq!(entry["policy_values"]["checked_commit"], "ordinary");
                assert_eq!(
                    entry["policy_values"]["operation_scope"],
                    "single-ordinary-checked-commit"
                );
                assert_eq!(
                    entry["policy_values"]["mutation"],
                    "operation-owned-face-edge-vertex-tolerance-growth"
                );
                assert_eq!(
                    entry["policy_values"]["target_order"],
                    "body-ordinal-then-face-edge-vertex"
                );
                assert_eq!(
                    entry["policy_values"]["root_selection"],
                    "primitive-ordinal-mod-5-equals-0"
                );
                assert_eq!(
                    entry["policy_values"]["distractor_store"],
                    "unchanged-64-body-primitive-mix"
                );
                assert_eq!(
                    entry["expected_result_counters"]["before_store_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_before_store_digest).as_str())
                );
                assert_eq!(
                    entry["expected_result_counters"]["after_store_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_after_store_digest).as_str())
                );
            }
            if case.ladder == Ladder::ProductionClean {
                assert_eq!(entry["tolerances"], serde_json::json!({}));
                assert_eq!(entry["policy_values"]["ladder"], "production-clean");
                assert_eq!(entry["policy_values"]["checked_commit"], "ordinary");
                assert_eq!(
                    entry["policy_values"]["operation_scope"],
                    "single-ordinary-checked-commit"
                );
                assert_eq!(entry["policy_values"]["mutation"], "none");
                assert_eq!(
                    entry["policy_values"]["production_fixture"],
                    "unchanged-primitive-mix"
                );
                assert_eq!(
                    entry["policy_values"]["covered_path"],
                    "global-validation-index-clone-body-order"
                );
                assert_eq!(
                    entry["expected_result_counters"]["before_store_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_before_store_digest).as_str())
                );
                assert_eq!(
                    entry["expected_result_counters"]["after_store_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_after_store_digest).as_str())
                );
                assert_eq!(
                    entry["expected_result_counters"]["before_index_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_before_index_digest).as_str())
                );
                assert_eq!(
                    entry["expected_result_counters"]["after_index_digest"].as_str(),
                    Some(format!("{:016x}", case.expected_after_index_digest).as_str())
                );
                assert_eq!(
                    entry["expected_result_counters"]["store_unchanged"].as_bool(),
                    Some(true)
                );
                assert_eq!(
                    entry["expected_result_counters"]["index_unchanged"].as_bool(),
                    Some(true)
                );
                assert_eq!(
                    entry["expected_result_counters"]["repeat_deterministic"].as_bool(),
                    Some(true)
                );
            }
            let counters = &entry["expected_result_counters"];
            if matches!(
                case.ladder,
                Ladder::AffectedSolidFootprint
                    | Ladder::AffectedBlockCohortFootprint
                    | Ladder::ProductionClean
            ) {
                assert_eq!(counters["operation_scopes"].as_u64(), Some(1));
            }
            if case.ladder == Ladder::AffectedBlockCohortFootprint {
                assert_eq!(
                    counters["modified_faces"].as_u64(),
                    Some(case.affected_bodies as u64)
                );
                assert_eq!(
                    counters["modified_edges"].as_u64(),
                    Some(case.affected_bodies as u64)
                );
                assert_eq!(
                    counters["modified_vertices"].as_u64(),
                    Some(case.affected_bodies as u64)
                );
                assert_eq!(
                    counters["tolerance_events"].as_u64(),
                    Some((3 * case.affected_bodies) as u64)
                );
            }
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
    fn all_ten_smallest_ladders_match_reviewed_result_evidence() {
        for case in [
            CASES[0], CASES[4], CASES[8], CASES[11], CASES[14], CASES[17], CASES[21], CASES[28],
            CASES[35], CASES[39],
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

    #[test]
    fn affected_solid_footprint_crossed_ladders_pin_scope_mutations_and_digests() {
        let cases = CASES
            .iter()
            .copied()
            .filter(|case| case.ladder == Ladder::AffectedSolidFootprint)
            .collect::<Vec<_>>();
        assert_eq!(cases.len(), 7);
        assert_eq!(
            cases
                .iter()
                .filter(|case| case.bodies == 64)
                .map(|case| case.affected_bodies)
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([1, 4, 16, 64])
        );
        let fixed_affected = cases
            .iter()
            .filter(|case| case.affected_bodies == 1)
            .collect::<Vec<_>>();
        assert_eq!(
            fixed_affected
                .iter()
                .map(|case| case.bodies)
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([4, 16, 64, 256])
        );
        assert_eq!(
            fixed_affected
                .iter()
                .map(|case| case.expected_affected_digest)
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([0x0300_00b9_a2c2_19c6])
        );
        for case in cases {
            let result = fixture(case).execute(case.ladder);
            verify(case, &result);
            let repeat = fixture(case).execute(case.ladder);
            verify(case, &repeat);
            assert_eq!(repeat.before_store.digest, result.before_store.digest);
            assert_eq!(repeat.after_store.digest, result.after_store.digest);
            assert_eq!(repeat.output_digest(), result.output_digest());
            if case.affected_bodies == 1 {
                assert_eq!(result.observation.affected_bodies, 1);
                assert_eq!(result.observation.refreshed_bodies, 1);
                assert_eq!(result.observation.checked_bodies, 1);
                assert_eq!(result.observation.mutations, 1);
            }
        }
    }

    #[test]
    fn affected_block_cohort_footprint_pins_mixed_entity_scope_and_digests() {
        let cases = CASES
            .iter()
            .copied()
            .filter(|case| case.ladder == Ladder::AffectedBlockCohortFootprint)
            .collect::<Vec<_>>();
        assert_eq!(cases.len(), 4);
        assert!(cases.iter().all(|case| case.bodies == 64));
        assert_eq!(
            cases
                .iter()
                .map(|case| case.affected_bodies)
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([1, 4, 8, 13])
        );
        for case in cases {
            let result = fixture(case).execute(case.ladder);
            verify(case, &result);
            assert_eq!(result.observation.affected_bodies, case.affected_bodies);
            assert_eq!(result.observation.refreshed_bodies, case.affected_bodies);
            assert_eq!(result.observation.checked_bodies, case.affected_bodies);
            assert_eq!(result.observation.mutations, 3 * case.affected_bodies);

            let repeat = fixture(case).execute(case.ladder);
            verify(case, &repeat);
            assert_eq!(repeat.before_store.digest, result.before_store.digest);
            assert_eq!(repeat.after_store.digest, result.after_store.digest);
            assert_eq!(repeat.observation, result.observation);
            assert_eq!(repeat.output_digest(), result.output_digest());
        }
    }

    #[test]
    fn production_clean_ladder_pins_global_ordinary_commit_evidence() {
        let cases = CASES
            .iter()
            .copied()
            .filter(|case| case.ladder == Ladder::ProductionClean)
            .collect::<Vec<_>>();
        assert_eq!(cases.len(), 4);
        assert_eq!(
            cases
                .iter()
                .map(|case| case.bodies)
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([4, 16, 64, 256])
        );
        for case in cases {
            let result = fixture(case).execute(case.ladder);
            verify(case, &result);
            assert_eq!(result.before_store, result.after_store);
            assert_eq!(result.before_index, result.after_index);
            assert_eq!(result.observation.body_count, case.bodies);
            assert_eq!(result.observation.affected_bodies, 0);
            assert_eq!(result.observation.refreshed_bodies, 0);
            assert_eq!(result.observation.checked_bodies, 0);
            assert_eq!(result.observation.mutations, 0);
            assert!(result.observation.committed);

            let repeat = fixture(case).execute(case.ladder);
            verify(case, &repeat);
            assert_eq!(repeat, result);
            assert_eq!(repeat.output_digest(), result.output_digest());
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
