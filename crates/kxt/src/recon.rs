//! Reconstruction: from a parsed [`XtFile`] node graph to `ktopo` bodies.
//!
//! Mapping notes (XT → kernel), all Tier 0:
//!
//! - XT stores the edge/curve orientation on the **curve** (`sense`).
//!   Our model has no edge sense — an edge always runs along its curve's
//!   natural direction — so a `-` curve sense is absorbed by swapping the
//!   edge's vertices and flipping every fin sense on that edge. This is
//!   exact: no geometry is modified.
//! - The face normal in XT is the natural surface normal iff
//!   `face.sense == surface.sense`; that combination becomes our
//!   `Face::sense`.
//! - XT shells list *back-faces* (normal out of the owning region) and
//!   *front-faces* separately. Our model attaches each face to the shell
//!   it is a back-face of, which matches our outward-normal convention.
//!   Shells left with no content (e.g. the void exterior shell of a
//!   solid, which only lists front-faces) are dropped.
//! - Exact-edge parameter bounds are not stored in XT; they are recovered by
//!   closed-form inversion of the vertex positions on analytic curves
//!   (arc length on lines, `atan2` angles on circles/ellipses) and by
//!   projection on B-curves. A tolerant edge with `EDGE.curve = null` gets
//!   the canonical logical domain `[0, 1]`; each trimmed SP-curve on its
//!   fins supplies the correspondence to a 2D B-curve. Ring edges (no
//!   vertices) get `bounds: None` and must have exact curve geometry.
//! - Geometry conventions transfer exactly: XT and kernel
//!   parameterizations coincide for plane/cylinder/sphere/torus/circle/
//!   ellipse/line. Cones differ (XT measures `v` against the axis with a
//!   `tan α` taper; ours is slant-parameterized): the cone is rebuilt
//!   geometrically — same point set, different `(u, v)`.
//! - Attributes, groups, transforms, and construction geometry are parsed
//!   but not reconstructed (recorded in [`Reconstruction::skipped`]).

use crate::error::{Result, XtCapability, XtError};
use crate::parse::{Node, Value, XtFile};
use crate::schema::code;
use kcore::math;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationOutcome,
    OperationPolicyError, OperationScope, ResourceKind, StageId, WorkLedger,
};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::{Curve2d, NurbsCurve2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::{ParamRange, wrap_periodic};
use kgeom::project::{ProjectionBudgetProfile, ProjectionError};
use kgeom::surface::{Cone, Cylinder, Dir, Plane, Sphere, Surface, Torus};
use kgeom::vec::{Point2, Point3, Vec3};
use kgraph::{
    EvalBudgetProfile, EvalLimits, OffsetSurfaceDescriptor, SurfaceDerivativeOrder,
    TRANSMITTED_NURBS_TRACE_PROOF_DEPTH, TransmittedIntersectionChartMetadata,
    TransmittedNurbsIntersectionTrace, TransmittedOffsetNurbsTrace,
    certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
    certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals,
    certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals,
};
use ktopo::entity::{
    Body, BodyId, BodyKind, Curve2dId, CurveId, Edge, EdgeId, Face, FaceDomain, FaceId, Fin,
    FinPcurve, Loop, ParamMap1d, PcurveEndpointKind, Region, RegionId, RegionKind, Sense, Shell,
    ShellId, SurfaceId, Vertex, VertexId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::graph_work::GraphQueryWork;
use ktopo::store::Store;
use ktopo::tolerance::EntityTolerance;
use ktopo::transaction::{AssemblyStore, Journal, MutationKind};
use std::collections::BTreeMap;

const fn stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid X_T reconstruction stage"),
    }
}

/// Cumulative whole-range certificate work for transmitted intersection charts.
pub const INTERSECTION_CHART_CERTIFICATE_WORK: StageId =
    stage("kxt.intersection-chart-certificate-work");
/// High-water transmitted positions retained by one intersection chart.
pub const INTERSECTION_CHART_ITEMS: StageId = stage("kxt.intersection-chart-items");
/// High-water dependency/proof nesting used by transmitted intersection import.
pub const INTERSECTION_CHART_DEPTH: StageId = stage("kxt.intersection-chart-depth");

/// Versioned resource profile for transmitted intersection chart import.
#[derive(Debug, Clone, Copy, Default)]
pub struct IntersectionImportBudgetProfile;

impl IntersectionImportBudgetProfile {
    /// Bounded defaults for canonical plane charts and fixed-depth
    /// original-source NURBS trace subdivision.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                131_072,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T intersection import profile is valid")
    }

    /// Corpus-backed defaults for production B-surface transmitted charts.
    ///
    /// The Work cap admits the exemplar's exact fixed-depth original-source
    /// scan preflight while retaining the v1 Items/Depth ceilings. Callers may
    /// still install a stricter operation override; v1 remains available as
    /// the historical policy contract.
    pub fn v2_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                81_267_732,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T production intersection import profile is valid")
    }

    /// Corpus-backed defaults through the canonical equal-limit chart rung.
    ///
    /// The Work cap admits record 1828 and every later chart reached before
    /// the exemplar's still-unsupported terminated `T/F` limit. The v2
    /// profile remains the historical pre-equal-limit policy contract.
    pub fn v3_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                115_485_725,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T equal-limit intersection import profile is valid")
    }

    /// Corpus-backed defaults through the canonical end-terminator chart rung.
    ///
    /// The Work cap admits the exemplar's first finite-open chart ending at a
    /// documented `T/F` singular terminator. Historical v1-v3 profiles retain
    /// their exact policy contracts.
    pub fn v4_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                116_396_069,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T terminated intersection import profile is valid")
    }

    /// Corpus-backed defaults through the first finite-open Plane/B-surface
    /// chart whose redundant interior Plane UVs are omitted.
    ///
    /// Historical v1-v4 profiles retain their exact policy contracts.
    pub fn v5_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                117_478_445,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T finite-open Plane/B-surface intersection profile is valid")
    }

    /// Corpus-backed defaults through the first exact native Plane SP-curve
    /// lift and the next transmitted intersection chart it exposes.
    ///
    /// Historical v1-v5 profiles retain their exact policy contracts.
    pub fn v6_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                208_228_426,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T native Plane SP-curve profile is valid")
    }

    /// Corpus-backed defaults through the first finite-open
    /// Plane/Offset(B-surface) chart with an omitted interior Plane UV pair.
    ///
    /// Historical v1-v6 profiles retain their exact policy contracts.
    pub fn v7_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                272_430_166,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T finite-open Plane/Offset(B-surface) profile is valid")
    }

    /// Corpus-backed defaults through the first finite-open nonperiodic NURBS
    /// chart with a source-domain endpoint affected only by decimal roundoff.
    ///
    /// Historical v1-v7 profiles retain their exact policy contracts.
    pub fn v8_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                315_245_660,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T finite-open NURBS endpoint-roundoff profile is valid")
    }

    /// Corpus-backed defaults through the canonical finite-open three-sample
    /// Offset(B-surface)/Offset(B-surface) quadratic chart.
    ///
    /// Historical v1-v8 profiles retain their exact policy contracts.
    pub fn v9_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                323_814_492,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T quadratic dual-offset intersection profile is valid")
    }

    /// Corpus-backed defaults through the canonical finite-open four-sample
    /// Offset(B-surface)/Offset(B-surface) cubic chart.
    ///
    /// Historical v1-v9 profiles retain their exact policy contracts.
    pub fn v10_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                336_759_900,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T cubic dual-offset intersection profile is valid")
    }

    /// Corpus-backed defaults through the next canonical quadratic
    /// dual-offset chart whose source uses zero-multiplicity null-knot
    /// padding, plus the already-supported 11-sample Plane/Offset(B-surface)
    /// chart 3745 it exposes before the next unsupported dual-offset family.
    ///
    /// Historical v1-v10 profiles retain their exact policy contracts.
    pub fn v11_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                388_125_799,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T zero-multiplicity knot-padding profile is valid")
    }

    /// Corpus-backed defaults through the canonical finite-open seven-sample
    /// Offset(B-surface)/Offset(B-surface) polyline chart.
    ///
    /// Historical v1-v11 profiles retain their exact policy contracts.
    pub fn v12_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                414_569_575,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T seven-sample dual-offset intersection profile is valid")
    }

    /// Corpus-backed defaults through the canonical finite-open five-sample
    /// Offset(B-surface)/Offset(B-surface) polyline chart.
    ///
    /// Historical v1-v12 profiles retain their exact policy contracts.
    pub fn v13_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                431_854_695,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T five-sample dual-offset intersection profile is valid")
    }

    /// Corpus-backed defaults through the next canonical finite-open
    /// Plane/Offset(B-surface) line chart.
    ///
    /// Historical v1-v13 profiles retain their exact policy contracts.
    pub fn v14_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                INTERSECTION_CHART_CERTIFICATE_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                436_131_945,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_ITEMS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                65_536,
            ),
            LimitSpec::new(
                INTERSECTION_CHART_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64,
            ),
        ])
        .expect("built-in X_T Plane/Offset(B-surface) line profile is valid")
    }

    fn validate(ledger: &WorkLedger) -> core::result::Result<(), OperationPolicyError> {
        ledger.require_limit(
            INTERSECTION_CHART_CERTIFICATE_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        ledger.require_limit(
            INTERSECTION_CHART_ITEMS,
            ResourceKind::Items,
            AccountingMode::HighWater,
        )?;
        ledger.require_limit(
            INTERSECTION_CHART_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
        )?;
        Ok(())
    }
}

/// Everything produced by reconstructing one transmit file.
#[derive(Debug)]
pub struct Reconstruction {
    /// The bodies created in the store, in file order.
    pub bodies: Vec<BodyId>,
    /// Node types that were present but intentionally not reconstructed
    /// (attributes, groups, …), with occurrence counts.
    pub skipped: Vec<(u16, usize)>,
    /// Transaction journal for every entity created or changed by this
    /// reconstruction. Import currently emits raw mutations; file-node
    /// provenance becomes semantic lineage when interchange provenance IDs
    /// are introduced.
    pub journal: Journal,
}

/// Owner-level defaults for contextual X_T reconstruction.
///
/// Graph evaluation retains its finite aggregate/depth limits. Curve
/// projection retains finite per-query high-water limits and an accounting-
/// only aggregate query allowance until import-corpus evidence supports a
/// finite model-level cap.
pub fn reconstruction_budget_profile() -> BudgetPlan {
    let graph = EvalBudgetProfile::v1_defaults();
    let projection = ProjectionBudgetProfile::curve_aggregate_compatibility();
    let intersection = IntersectionImportBudgetProfile::v14_defaults();
    BudgetPlan::new(
        graph
            .limits()
            .iter()
            .chain(projection.limits())
            .chain(intersection.limits())
            .copied(),
    )
    .expect("built-in X_T reconstruction budget is valid")
}

fn reconstruction_compatibility_budget() -> BudgetPlan {
    let graph =
        EvalBudgetProfile::for_limits(EvalLimits::default().max_dependency_depth, usize::MAX);
    let projection = ProjectionBudgetProfile::curve_aggregate_compatibility();
    let intersection = IntersectionImportBudgetProfile::v14_defaults();
    BudgetPlan::new(
        graph
            .limits()
            .iter()
            .chain(projection.limits())
            .chain(intersection.limits())
            .copied(),
    )
    .expect("built-in X_T compatibility budget is valid")
}

/// Reconstruct every body in the file into `store`.
///
/// This compatibility wrapper discards operation accounting and uses a
/// non-binding aggregate visit allowance while retaining the historical
/// dependency-depth bound. The contextual entry points below apply the v1
/// aggregate graph-work profile and expose its report. For the currently
/// supported X_T graph (leaf surfaces and single-dependency offset chains),
/// depth 64 also bounds every individual query below the former 4,096-visit
/// ceiling, so the wrapper preserves existing result/error behavior.
pub fn reconstruct(file: &XtFile, store: &mut Store) -> Result<Reconstruction> {
    let session = kcore::operation::SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("built-in X_T reconstruction context is valid")
        .with_budget_overrides(reconstruction_compatibility_budget());
    reconstruct_with_context(file, store, &context)
        .expect("built-in X_T reconstruction budget is valid")
        .into_result()
}

/// Reconstruct with graph and curve-projection work charged to a fresh
/// operation scope.
///
/// X_T reconstruction defaults fill only entries omitted by the caller;
/// session policy and explicit operation overrides retain normal F2
/// precedence across both families.
pub fn reconstruct_with_context(
    file: &XtFile,
    store: &mut Store,
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<Reconstruction, XtError>, OperationPolicyError> {
    let context = context
        .clone()
        .with_family_budget_defaults(reconstruction_budget_profile());
    EvalLimits::from_budget_plan(&context.effective_budget())?;
    let mut scope = OperationScope::new(&context);
    let result = reconstruct_in_scope(file, store, &mut scope, 0);
    Ok(scope.finish_typed(result))
}

/// Reconstruct inside an existing caller-owned operation scope.
///
/// `child_ordinal` is the stable work-item ordinal assigned by the caller.
/// The scope must already contain the X_T reconstruction budget profile; this
/// nested entry point never installs defaults or resets accounting.
pub fn reconstruct_in_scope(
    file: &XtFile,
    store: &mut Store,
    scope: &mut OperationScope<'_, '_>,
    child_ordinal: u64,
) -> Result<Reconstruction> {
    IntersectionImportBudgetProfile::validate(scope.ledger()).map_err(policy_error)?;
    let graph = GraphQueryWork::reserve(scope, child_ordinal).map_err(policy_error)?;
    let mut transaction = match store.transaction() {
        Ok(transaction) => transaction,
        Err(error) => {
            graph.merge(scope).map_err(policy_error)?;
            return Err(error.into());
        }
    };
    let (lower, mut graph) = {
        let mut assembly = transaction.assembly();
        let mut graph = graph;
        let result = reconstruct_into(file, &mut assembly, &mut graph, scope);
        (result, graph)
    };
    let mut reconstruction = match lower {
        Ok(reconstruction) => reconstruction,
        Err(error) => {
            graph.merge(scope).map_err(policy_error)?;
            return Err(error);
        }
    };
    let commit = transaction.commit_checked_with_graph(&reconstruction.bodies, &mut graph);
    let accounting = graph.merge(scope).map_err(policy_error);
    reconstruction.journal = match (commit, accounting) {
        (Err(error), _) => return Err(error.into()),
        (Ok(_), Err(error)) => return Err(error),
        (Ok(journal), Ok(())) => journal,
    };
    debug_assert!(
        reconstruction
            .journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind != MutationKind::Deleted)
    );
    Ok(reconstruction)
}

/// Reconstruct inside the caller's active copy-on-write transaction.
fn reconstruct_into(
    file: &XtFile,
    store: &mut AssemblyStore<'_>,
    graph: &mut GraphQueryWork,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Reconstruction> {
    let root = xnode(file, 1)?;
    let mut body_indices = Vec::new();
    match root.code {
        code::BODY => body_indices.push(1u32),
        code::POINTER_LIS_BLOCK => {
            // An array-of-parts file: entries point to the parts.
            let mut block_idx = 1u32;
            while block_idx != 0 {
                let block = xnode(file, block_idx)?;
                for v in entries(file, block)? {
                    if xnode(file, v)?.code == code::BODY {
                        body_indices.push(v);
                    } else {
                        return Err(XtError::Unsupported {
                            capability: XtCapability::Assemblies,
                            what: "non-body parts (assemblies) in array-of-parts files",
                        });
                    }
                }
                block_idx = ptr(file, block, "next_block")?;
            }
        }
        code::ASSEMBLY => {
            return Err(XtError::Unsupported {
                capability: XtCapability::Assemblies,
                what: "assembly transmit files (Tier-0 reads body files)",
            });
        }
        code::WORLD => {
            return Err(XtError::Unsupported {
                capability: XtCapability::Partitions,
                what: "partition transmit files (Tier-0 reads body files)",
            });
        }
        _ => {
            return Err(XtError::BadField {
                index: 1,
                what: "root node is not a body, part list, assembly, or world",
            });
        }
    }

    let mut recon = Recon {
        file,
        store,
        graph,
        scope,
        curves: BTreeMap::new(),
        pcurves: BTreeMap::new(),
        surfaces: BTreeMap::new(),
        surface_stack: Vec::new(),
        points: BTreeMap::new(),
        vertices: BTreeMap::new(),
        edges: BTreeMap::new(),
        body_linear_tolerance: None,
    };
    let bodies = body_indices
        .iter()
        .map(|&b| recon.body(b))
        .collect::<Result<Vec<_>>>()?;

    // Count node types we deliberately did not reconstruct.
    let mut skipped: BTreeMap<u16, usize> = BTreeMap::new();
    for node in file.nodes.values() {
        let deliberately_skipped = matches!(
            node.code,
            code::ATTRIBUTE
                | code::ATTRIB_DEF
                | code::ATT_DEF_ID
                | code::INT_VALUES
                | code::REAL_VALUES
                | code::CHAR_VALUES
                | code::POINT_VALUES
                | code::VECTOR_VALUES
                | code::AXIS_VALUES
                | code::TAG_VALUES
                | code::DIRECTION_VALUES
                | code::UNICODE_VALUES
                | code::FIELD_NAMES
                | code::GROUP
                | code::MEMBER_OF_GROUP
                | code::LIST
                | code::POINTER_LIS_BLOCK
                | code::TRANSFORM
                | code::GEOMETRIC_OWNER
                | code::KEY
        ) || (node.code != code::INTERSECTION_DATA
            && file.foreign_codes.contains(&node.code));
        if deliberately_skipped {
            *skipped.entry(node.code).or_insert(0) += 1;
        }
    }
    Ok(Reconstruction {
        bodies,
        skipped: skipped.into_iter().collect(),
        journal: Journal::default(),
    })
}

fn policy_error(error: OperationPolicyError) -> XtError {
    XtError::Kernel(error.into())
}

fn preflight_intersection_chart(
    scope: &mut OperationScope<'_, '_>,
    count: u64,
    proof_depth: u64,
    proof_work: u64,
) -> Result<()> {
    scope
        .ledger_mut()
        .observe(INTERSECTION_CHART_ITEMS, ResourceKind::Items, count)
        .map_err(policy_error)?;
    scope
        .ledger_mut()
        .observe(INTERSECTION_CHART_DEPTH, ResourceKind::Depth, proof_depth)
        .map_err(policy_error)?;
    scope
        .ledger()
        .check_charge(INTERSECTION_CHART_CERTIFICATE_WORK, proof_work)
        .map_err(policy_error)
}

// ------------------------------------------------------- field helpers --

fn xnode(file: &XtFile, index: u32) -> Result<&Node> {
    file.node(index).ok_or(XtError::MissingNode { index })
}

fn field<'a>(file: &'a XtFile, node: &'a Node, name: &'static str) -> Result<&'a Value> {
    file.field(node, name).ok_or(XtError::BadField {
        index: 0,
        what: name,
    })
}

fn ptr(file: &XtFile, node: &Node, name: &'static str) -> Result<u32> {
    field(file, node, name)?.as_ptr().ok_or(XtError::BadField {
        index: 0,
        what: "expected a pointer field",
    })
}

fn ch(file: &XtFile, node: &Node, name: &'static str) -> Result<char> {
    field(file, node, name)?.as_char().ok_or(XtError::BadField {
        index: 0,
        what: "expected a char field",
    })
}

fn f64_of(file: &XtFile, node: &Node, name: &'static str) -> Result<f64> {
    field(file, node, name)?.as_f64().ok_or(XtError::BadField {
        index: 0,
        what: "expected a numeric field",
    })
}

fn logical_of(file: &XtFile, node: &Node, name: &'static str) -> Result<bool> {
    match field(file, node, name)? {
        Value::Logical(b) => Ok(*b),
        _ => Err(XtError::BadField {
            index: 0,
            what: "expected a logical field",
        }),
    }
}

fn vector(file: &XtFile, node: &Node, name: &'static str) -> Result<Vec3> {
    let v = field(file, node, name)?
        .as_vector()
        .ok_or(XtError::BadField {
            index: 0,
            what: "expected a non-null vector field",
        })?;
    Ok(Vec3::new(v[0], v[1], v[2]))
}

/// Optional tolerance: null double → `None`.
fn tolerance(file: &XtFile, node: &Node) -> Result<Option<EntityTolerance>> {
    Ok(match field(file, node, "tolerance")? {
        Value::Null => None,
        v => Some(EntityTolerance::imported_xt(v.as_f64().ok_or(
            XtError::BadField {
                index: 0,
                what: "tolerance is not numeric",
            },
        )?)?),
    })
}

fn entries(file: &XtFile, block: &Node) -> Result<Vec<u32>> {
    match field(file, block, "entries")? {
        Value::Arr(vs) => Ok(vs
            .iter()
            .filter_map(Value::as_ptr)
            .filter(|&p| p != 0)
            .collect()),
        _ => Err(XtError::BadField {
            index: 0,
            what: "entries is not an array",
        }),
    }
}

fn in_size_box(p: Vec3) -> Result<Vec3> {
    for c in [p.x, p.y, p.z] {
        if !c.is_finite() || c.abs() > 500.0 {
            return Err(XtError::OutsideSizeBox { value: c });
        }
    }
    Ok(p)
}

fn intersection_limit(file: &XtFile, index: u32) -> Result<Point3> {
    let node = xnode(file, index)?;
    if node.code != code::LIMIT {
        return Err(XtError::BadField {
            index,
            what: "INTERSECTION limit pointer is not a LIMIT",
        });
    }
    if ch(file, node, "type")? != 'L'
        || !matches!(file.field(node, "term_use"), Some(Value::Char('?')))
    {
        return Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "only finite open LIMIT type L with term_use ? is supported",
        });
    }
    match field(file, node, "hvec")? {
        Value::Arr(values) if values.len() == 1 => {
            let value = values[0].as_vector().ok_or(XtError::BadField {
                index,
                what: "LIMIT contains a null or malformed position",
            })?;
            in_size_box(Point3::new(value[0], value[1], value[2]))
        }
        _ => Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "finite open LIMIT must contain exactly one position",
        }),
    }
}

fn equal_intersection_limit(file: &XtFile, index: u32) -> Result<Point3> {
    let node = xnode(file, index)?;
    if node.code != code::LIMIT {
        return Err(XtError::BadField {
            index,
            what: "INTERSECTION equal-limit pointer is not a LIMIT",
        });
    }
    if ch(file, node, "type")? != 'H'
        || !matches!(file.field(node, "term_use"), Some(Value::Char('?')))
    {
        return Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "only one shared closed LIMIT type H with term_use ? is supported for an equal-limit chart",
        });
    }
    match field(file, node, "hvec")? {
        Value::Arr(values) if values.len() == 1 => {
            let value = values[0].as_vector().ok_or(XtError::BadField {
                index,
                what: "equal LIMIT contains a null or malformed position",
            })?;
            in_size_box(Point3::new(value[0], value[1], value[2]))
        }
        _ => Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "shared closed LIMIT must contain exactly one position",
        }),
    }
}

fn terminated_intersection_limit(file: &XtFile, index: u32) -> Result<[Point3; 2]> {
    let node = xnode(file, index)?;
    if node.code != code::LIMIT {
        return Err(XtError::BadField {
            index,
            what: "INTERSECTION terminator pointer is not a LIMIT",
        });
    }
    if ch(file, node, "type")? != 'T'
        || !matches!(file.field(node, "term_use"), Some(Value::Char('F')))
    {
        return Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "only end LIMIT type T with term_use F is supported as a singular terminator",
        });
    }
    match field(file, node, "hvec")? {
        Value::Arr(values) if values.len() == 2 => {
            let mut positions = Vec::with_capacity(2);
            for value in values {
                let value = value.as_vector().ok_or(XtError::BadField {
                    index,
                    what: "terminator LIMIT contains a null or malformed position",
                })?;
                positions.push(in_size_box(Point3::new(value[0], value[1], value[2]))?);
            }
            Ok([positions[0], positions[1]])
        }
        _ => Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "singular terminator LIMIT must contain exactly two positions",
        }),
    }
}

fn canonicalize_equal_limit_periodic_trace_endpoints(
    curve_idx: u32,
    traces: &[TransmittedNurbsIntersectionTrace; 2],
    uv: &mut [Vec<Point2>; 2],
) -> Result<()> {
    let mut periodic_axes = 0_usize;
    for (trace_index, trace) in traces.iter().enumerate() {
        let surface = match trace {
            TransmittedNurbsIntersectionTrace::Plane(_)
            | TransmittedNurbsIntersectionTrace::Sphere(_) => continue,
            TransmittedNurbsIntersectionTrace::Nurbs(surface) => surface,
            TransmittedNurbsIntersectionTrace::OffsetNurbs(offset) => offset.basis(),
        };
        for (axis, periodicity) in surface.periodicity().into_iter().enumerate() {
            let Some(period) = periodicity else {
                continue;
            };
            periodic_axes += 1;
            let domain = surface
                .knots(if axis == 0 { Dir::U } else { Dir::V })
                .domain();
            if period != domain.width() || uv[trace_index].len() < 2 {
                return Err(XtError::Unsupported {
                    capability: XtCapability::IntersectionLimits,
                    what: "equal-limit chart does not use one canonical certified periodic seam",
                });
            }
            let coordinate = |point: Point2| if axis == 0 { point.x } else { point.y };
            let first_raw = coordinate(uv[trace_index][0]);
            let last_index = uv[trace_index].len() - 1;
            let last_raw = coordinate(uv[trace_index][last_index]);
            let scale = domain
                .lo
                .abs()
                .max(domain.hi.abs())
                .max(period.abs())
                .max(1.0);
            let seam_slack = 16_384.0 * f64::EPSILON * scale;
            let near_seam =
                |value: f64| (value - domain.lo).abs().min((value - domain.hi).abs()) <= seam_slack;
            if !near_seam(first_raw) || !near_seam(last_raw) {
                return Err(XtError::Unsupported {
                    capability: XtCapability::IntersectionLimits,
                    what: "equal-limit chart endpoints are not on one certified periodic seam",
                });
            }
            let second = coordinate(uv[trace_index][1]);
            let penultimate = coordinate(uv[trace_index][last_index - 1]);
            let nearest_boundary = |neighbor: f64| {
                if (neighbor - domain.lo).abs() <= (neighbor - domain.hi).abs() {
                    domain.lo
                } else {
                    domain.hi
                }
            };
            let unwrap_endpoint = |raw: f64, neighbor: f64| {
                let raw_boundary = nearest_boundary(raw);
                let desired_boundary = nearest_boundary(neighbor);
                // Preserve the transmitted value modulo one exact certified
                // period. No intermediate UV or model-space chart position
                // is rewritten.
                raw + (desired_boundary - raw_boundary)
            };
            if axis == 0 {
                uv[trace_index][0].x = unwrap_endpoint(first_raw, second);
                uv[trace_index][last_index].x = unwrap_endpoint(last_raw, penultimate);
            } else {
                uv[trace_index][0].y = unwrap_endpoint(first_raw, second);
                uv[trace_index][last_index].y = unwrap_endpoint(last_raw, penultimate);
            }
        }
    }
    if periodic_axes != 1 {
        return Err(XtError::Unsupported {
            capability: XtCapability::IntersectionLimits,
            what: "equal-limit chart must close on exactly one certified periodic NURBS axis",
        });
    }
    if uv.iter().any(|trace| trace.len() < 2) {
        return Err(XtError::BadField {
            index: curve_idx,
            what: "equal-limit chart has no endpoint pair",
        });
    }
    Ok(())
}

/// Snap only endpoint coordinates whose decimal overhang is within a bounded
/// source-domain floating slack. Interior values are never rewritten, and the
/// complete resulting pcurve must still pass its original-source certificate.
fn canonicalize_trace_endpoint_roundoff(
    traces: &[TransmittedNurbsIntersectionTrace; 2],
    uv: &mut [Vec<Point2>; 2],
) {
    for (trace_index, trace) in traces.iter().enumerate() {
        let surface = match trace {
            TransmittedNurbsIntersectionTrace::Plane(_)
            | TransmittedNurbsIntersectionTrace::Sphere(_) => continue,
            TransmittedNurbsIntersectionTrace::Nurbs(surface) => surface,
            TransmittedNurbsIntersectionTrace::OffsetNurbs(offset) => offset.basis(),
        };
        let last = uv[trace_index].len() - 1;
        for endpoint in [0, last] {
            for axis in 0..2 {
                let domain = surface
                    .knots(if axis == 0 { Dir::U } else { Dir::V })
                    .domain();
                let coordinate = if axis == 0 {
                    &mut uv[trace_index][endpoint].x
                } else {
                    &mut uv[trace_index][endpoint].y
                };
                let scale = domain.lo.abs().max(domain.hi.abs()).max(1.0);
                let endpoint_slack = 16_384.0 * f64::EPSILON * scale;
                if *coordinate < domain.lo && domain.lo - *coordinate <= endpoint_slack {
                    *coordinate = domain.lo;
                } else if *coordinate > domain.hi && *coordinate - domain.hi <= endpoint_slack {
                    *coordinate = domain.hi;
                }
            }
        }
    }
}

fn transmitted_nurbs_trace_proof_work(
    file: &XtFile,
    surface_index: u32,
    chart_count: u64,
) -> Result<u64> {
    let surface = xnode(file, surface_index)?;
    let nurbs_index = ptr(file, surface, "nurbs")?;
    let nurbs = xnode(file, nurbs_index)?;
    let degree_u = f64_of(file, nurbs, "u_degree")? as u64;
    let degree_v = f64_of(file, nurbs, "v_degree")? as u64;
    let declared_control_u = f64_of(file, nurbs, "n_u_vertices")? as u64;
    let declared_control_v = f64_of(file, nurbs, "n_v_vertices")? as u64;
    let control_u =
        transmitted_knot_control_count(file, nurbs, degree_u, "u_knots", "u_knot_mult")?;
    let control_v =
        transmitted_knot_control_count(file, nurbs, degree_v, "v_knots", "v_knot_mult")?;
    if [declared_control_u, declared_control_v] != [control_u, control_v] {
        return Err(XtError::BadField {
            index: surface_index,
            what: "B-surface declared control dimensions do not match its knot-implied control net",
        });
    }
    let span_slots = control_u
        .checked_sub(degree_u)
        .and_then(|u| {
            control_v
                .checked_sub(degree_v)
                .and_then(|v| u.checked_mul(v))
        })
        .ok_or(XtError::BadField {
            index: surface_index,
            what: "B-surface source span count underflows or overflows",
        })?;
    let enclosure_work = span_slots
        .checked_mul(6)
        .and_then(|work| work.checked_add(1))
        .ok_or(XtError::BadField {
            index: surface_index,
            what: "B-surface source enclosure work overflows",
        })?;
    let subdivisions = 1_u64
        .checked_shl(TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u32)
        .ok_or(XtError::BadField {
            index: surface_index,
            what: "B-surface proof subdivision count overflows",
        })?;
    let span_count = chart_count.checked_sub(1).ok_or(XtError::BadField {
        index: surface_index,
        what: "B-surface proof chart has no spans",
    })?;
    span_count
        .checked_mul(subdivisions)
        .and_then(|segments| segments.checked_mul(enclosure_work))
        .ok_or(XtError::BadField {
            index: surface_index,
            what: "INTERSECTION CHART B-surface proof work overflows",
        })
}

fn transmitted_offset_chains_are_independent(file: &XtFile, roots: [u32; 2]) -> Result<bool> {
    fn chain(file: &XtFile, root: u32) -> Result<Vec<u32>> {
        let mut path = Vec::new();
        let mut current = root;
        loop {
            if let Some(start) = path.iter().position(|&index| index == current) {
                let mut cycle = path[start..].to_vec();
                cycle.push(current);
                return Err(XtError::SurfaceDependencyCycle { path: cycle });
            }
            path.push(current);
            let node = xnode(file, current)?;
            if node.code != code::OFFSET_SURF {
                return Ok(path);
            }
            current = ptr(file, node, "surface")?;
        }
    }

    let first = chain(file, roots[0])?;
    let second = chain(file, roots[1])?;
    Ok(!first.iter().any(|node| second.contains(node)))
}

fn transmitted_knot_control_count(
    file: &XtFile,
    nurbs: &Node,
    degree: u64,
    knots_field: &'static str,
    multiplicities_field: &'static str,
) -> Result<u64> {
    let knots_index = ptr(file, nurbs, knots_field)?;
    let multiplicities_index = ptr(file, nurbs, multiplicities_field)?;
    let knots = match field(file, xnode(file, knots_index)?, "knots")? {
        Value::Arr(values) => values,
        _ => {
            return Err(XtError::BadField {
                index: knots_index,
                what: "B-surface knot set is not an array",
            });
        }
    };
    let multiplicities = match field(file, xnode(file, multiplicities_index)?, "mult")? {
        Value::Arr(values) if values.len() == knots.len() => values,
        Value::Arr(_) => {
            return Err(XtError::BadField {
                index: multiplicities_index,
                what: "knot and multiplicity arrays differ in length",
            });
        }
        _ => {
            return Err(XtError::BadField {
                index: multiplicities_index,
                what: "B-surface knot multiplicities are not an array",
            });
        }
    };
    let expanded_count =
        knots
            .iter()
            .zip(multiplicities)
            .try_fold(0_u64, |count, (knot, multiplicity)| {
                let multiplicity = multiplicity
                    .as_int()
                    .and_then(|value| u64::try_from(value).ok())
                    .ok_or(XtError::BadField {
                        index: multiplicities_index,
                        what: "B-surface knot multiplicity is negative, nonintegral, or too large",
                    })?;
                if multiplicity == 0 {
                    if matches!(knot, Value::Null)
                        || knot.as_f64().is_some_and(|value| value.is_finite())
                    {
                        return Ok(count);
                    }
                    return Err(XtError::BadField {
                        index: knots_index,
                        what: "zero-multiplicity knot padding is neither null nor finite numeric",
                    });
                }
                if !knot.as_f64().is_some_and(|value| value.is_finite()) {
                    return Err(XtError::BadField {
                        index: knots_index,
                        what: "positive-multiplicity knot is null, non-numeric, or non-finite",
                    });
                }
                count.checked_add(multiplicity).ok_or(XtError::BadField {
                    index: multiplicities_index,
                    what: "B-surface expanded knot count overflows",
                })
            })?;
    expanded_count
        .checked_sub(degree)
        .and_then(|count| count.checked_sub(1))
        .ok_or(XtError::BadField {
            index: multiplicities_index,
            what: "B-surface knot-implied control count underflows",
        })
}

// ---------------------------------------------------------------- body --

struct Recon<'file, 'assembly, 'store, 'graph, 'scope, 'context, 'session> {
    file: &'file XtFile,
    store: &'assembly mut AssemblyStore<'store>,
    graph: &'graph mut GraphQueryWork,
    scope: &'scope mut OperationScope<'context, 'session>,
    /// XT curve index → (kernel curve, XT sense was `-`).
    curves: BTreeMap<u32, (CurveId, bool)>,
    /// XT 2D B-curve index → kernel pcurve geometry.
    pcurves: BTreeMap<u32, Curve2dId>,
    /// Black tri-color entries: XT surface index → completed kernel node.
    surfaces: BTreeMap<u32, (SurfaceId, char)>,
    /// Gray tri-color entries in deterministic transport-ID stack order.
    /// Indices absent from both containers are white.
    surface_stack: Vec<u32>,
    /// XT point index → kernel point.
    points: BTreeMap<u32, ktopo::entity::PointId>,
    vertices: BTreeMap<u32, VertexId>,
    /// XT edge index → (kernel edge, fins must flip: curve sense was `-`).
    edges: BTreeMap<u32, (EdgeId, bool)>,
    /// Active BODY declaration used only by transmitted intersection proofs.
    body_linear_tolerance: Option<f64>,
}

impl Recon<'_, '_, '_, '_, '_, '_, '_> {
    fn body(&mut self, body_idx: u32) -> Result<BodyId> {
        let file = self.file;
        let body_node = xnode(file, body_idx)?;
        if body_node.code != code::BODY {
            return Err(XtError::BadField {
                index: body_idx,
                what: "referenced part is not a BODY node",
            });
        }
        self.body_linear_tolerance = field(file, body_node, "res_linear")?.as_f64();
        let kind = match field(file, body_node, "body_type")?.as_int() {
            Some(1) => BodyKind::Solid,
            Some(2) => BodyKind::Wire, // acorn detection below
            Some(3) => BodyKind::Sheet,
            _ => {
                return Err(XtError::Unsupported {
                    capability: XtCapability::GeneralBodies,
                    what: "general bodies (body_type 6)",
                });
            }
        };
        let body = self.store.add(Body {
            kind,
            regions: Vec::new(),
        });

        // Region chain; the head is the infinite (exterior) region.
        let mut region_idx = ptr(file, body_node, "region")?;
        let mut is_acorn = false;
        while region_idx != 0 {
            let region_node = xnode(file, region_idx)?;
            let region_kind = match ch(file, region_node, "type")? {
                'S' => RegionKind::Solid,
                'V' => RegionKind::Void,
                _ => {
                    return Err(XtError::BadField {
                        index: region_idx,
                        what: "region type is not S/V",
                    });
                }
            };
            let next_region = ptr(file, region_node, "next")?;
            let first_shell = ptr(file, region_node, "shell")?;
            let region = self.store.add(Region {
                body,
                kind: region_kind,
                shells: Vec::new(),
            });
            self.store.get_mut(body)?.regions.push(region);

            let mut shell_idx = first_shell;
            while shell_idx != 0 {
                let next_shell = ptr(file, xnode(file, shell_idx)?, "next")?;
                if let Some(acorn) = self.shell(region, shell_idx)? {
                    is_acorn |= acorn;
                }
                shell_idx = next_shell;
            }
            region_idx = next_region;
        }
        if self.store.get(body)?.regions.is_empty() {
            return Err(XtError::BadField {
                index: body_idx,
                what: "body has no regions",
            });
        }
        if is_acorn {
            self.store.get_mut(body)?.kind = BodyKind::Acorn;
        }
        Ok(body)
    }

    /// Reconstruct one shell; returns `Some(is_acorn)`, or `None` if the
    /// shell was dropped as empty (a solid's void-exterior shell).
    fn shell(&mut self, region: RegionId, shell_idx: u32) -> Result<Option<bool>> {
        let file = self.file;
        let shell_node = xnode(file, shell_idx)?;
        let first_face = ptr(file, shell_node, "face")?;
        let first_edge = ptr(file, shell_node, "edge")?;
        let vertex_idx = ptr(file, shell_node, "vertex")?;

        let shell = self.store.add(Shell {
            region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });

        // Back-faces: the faces whose normal points out of this shell's
        // region — exactly our convention.
        let mut face_idx = first_face;
        while face_idx != 0 {
            let next = ptr(file, xnode(file, face_idx)?, "next")?;
            self.face(shell, face_idx)?;
            face_idx = next;
        }
        // Wireframe edges.
        let mut edge_idx = first_edge;
        while edge_idx != 0 {
            let next = ptr(file, xnode(file, edge_idx)?, "next")?;
            let (edge, _) = self.edge(edge_idx)?;
            self.store.get_mut(shell)?.edges.push(edge);
            edge_idx = next;
        }
        // Acorn vertex.
        let mut acorn = false;
        if vertex_idx != 0 {
            let v = self.vertex(vertex_idx)?;
            self.store.get_mut(shell)?.vertex = Some(v);
            acorn = true;
        }

        let s = self.store.get(shell)?;
        if s.faces.is_empty() && s.edges.is_empty() && s.vertex.is_none() {
            // The void-exterior shell of a solid lists only front-faces;
            // it carries no information our model keeps. Nothing points
            // to it yet, so it can simply be removed.
            self.store.remove(shell)?;
            return Ok(None);
        }
        self.store.get_mut(region)?.shells.push(shell);
        Ok(Some(acorn))
    }

    fn face(&mut self, shell: ShellId, face_idx: u32) -> Result<FaceId> {
        let file = self.file;
        let face_node = xnode(file, face_idx)?;
        let surface_idx = ptr(file, face_node, "surface")?;
        if surface_idx == 0 {
            return Err(XtError::Unsupported {
                capability: XtCapability::SurfaceLessFaces,
                what: "faces without surface geometry",
            });
        }
        let first_loop = ptr(file, face_node, "loop")?;
        let xt_face_sense = ch(file, face_node, "sense")?;
        let tolerance = tolerance(file, face_node)?;

        let (surface, surf_sense) = self.surface(surface_idx)?;
        let domain = FaceDomain::natural(self.store.get(surface)?);
        // Face normal == natural surface normal iff the two senses agree.
        let sense = if xt_face_sense == surf_sense {
            Sense::Forward
        } else {
            Sense::Reversed
        };
        let face = self.store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense,
            domain,
            tolerance,
        });
        self.store.get_mut(shell)?.faces.push(face);

        let mut loop_idx = first_loop;
        while loop_idx != 0 {
            let next = ptr(file, xnode(file, loop_idx)?, "next")?;
            self.lp(face, loop_idx)?;
            loop_idx = next;
        }
        let natural = self
            .graph
            .query(self.store, |evaluator| {
                evaluator.surface_param_range(surface)
            })
            .map_err(policy_error)?
            .map_err(XtError::Evaluation)?;
        let periods = self
            .graph
            .query(self.store, |evaluator| {
                evaluator.surface_periodicity(surface)
            })
            .map_err(policy_error)?
            .map_err(XtError::Evaluation)?;
        let domain =
            ktopo::domain::derive_face_domain_from_metadata(self.store, face, natural, periods)?;
        self.store.get_mut(face)?.domain = domain;
        Ok(face)
    }

    fn lp(&mut self, face: FaceId, loop_idx: u32) -> Result<()> {
        let file = self.file;
        let loop_node = xnode(file, loop_idx)?;
        let first_fin = ptr(file, loop_node, "fin")?;
        if first_fin == 0 {
            return Err(XtError::Unsupported {
                capability: XtCapability::IsolatedLoops,
                what: "isolated loops (single-vertex loops)",
            });
        }
        let lp = self.store.add(Loop {
            face,
            fins: Vec::new(),
        });
        self.store.get_mut(face)?.loops.push(lp);

        // Walk the fin ring via forward pointers.
        let mut fin_idx = first_fin;
        let mut fins = Vec::new();
        loop {
            let fin_node = xnode(file, fin_idx)?;
            let edge_idx = ptr(file, fin_node, "edge")?;
            if edge_idx == 0 {
                return Err(XtError::BadField {
                    index: fin_idx,
                    what: "loop fin has no edge",
                });
            }
            let xt_sense = ch(file, fin_node, "sense")?;
            let forward = ptr(file, fin_node, "forward")?;

            let (edge, flip) = self.edge(edge_idx)?;
            let mut sense = match xt_sense {
                '+' => Sense::Forward,
                '-' => Sense::Reversed,
                _ => {
                    return Err(XtError::BadField {
                        index: fin_idx,
                        what: "fin sense is not +/-",
                    });
                }
            };
            if flip {
                sense = sense.flipped();
            }
            let pcurve = self.fin_pcurve(fin_idx, face, edge)?;
            let fin = self.store.add(Fin {
                parent: lp,
                edge,
                sense,
                pcurve,
            });
            self.store.get_mut(edge)?.fins.push(fin);
            fins.push(fin);

            if forward == first_fin || forward == 0 {
                break;
            }
            fin_idx = forward;
            if fins.len() > 1_000_000 {
                return Err(XtError::BadField {
                    index: loop_idx,
                    what: "fin ring does not close",
                });
            }
        }
        self.store.get_mut(lp)?.fins = fins;
        Ok(())
    }

    fn vertex(&mut self, vertex_idx: u32) -> Result<VertexId> {
        if let Some(&v) = self.vertices.get(&vertex_idx) {
            return Ok(v);
        }
        let file = self.file;
        let vertex_node = xnode(file, vertex_idx)?;
        let point_idx = ptr(file, vertex_node, "point")?;
        let point = if let Some(&point) = self.points.get(&point_idx) {
            point
        } else {
            let point_node = xnode(file, point_idx)?;
            let p = in_size_box(vector(file, point_node, "pvec")?)?;
            let point = self.store.add(p);
            self.points.insert(point_idx, point);
            point
        };
        let tol = tolerance(file, vertex_node)?;
        let v = self.store.add(Vertex {
            point,
            tolerance: tol,
        });
        self.vertices.insert(vertex_idx, v);
        Ok(v)
    }

    fn edge(&mut self, edge_idx: u32) -> Result<(EdgeId, bool)> {
        if let Some(&e) = self.edges.get(&edge_idx) {
            return Ok(e);
        }
        let file = self.file;
        let edge_node = xnode(file, edge_idx)?;
        let curve_idx = ptr(file, edge_node, "curve")?;
        let (curve, curve_reversed, trim) = if curve_idx == 0 {
            (None, false, None)
        } else {
            // Trimmed curves carry their bounds; plain curves get inverted.
            let curve_node = xnode(file, curve_idx)?;
            let (geom_curve_idx, trim) = if curve_node.code == code::TRIMMED_CURVE {
                let basis = ptr(file, curve_node, "basis_curve")?;
                let p1 = f64_of(file, curve_node, "parm_1")?;
                let p2 = f64_of(file, curve_node, "parm_2")?;
                (basis, Some((p1, p2)))
            } else {
                (curve_idx, None)
            };
            let (curve, reversed) = self.curve(geom_curve_idx)?;
            (Some(curve), reversed, trim)
        };

        // Vertices via the edge's fin ring: a `+` fin's forward vertex is
        // the edge end, a `-` fin's is the edge start (dummy fins exist
        // precisely to make both reachable).
        let mut start_idx = 0u32;
        let mut end_idx = 0u32;
        let head_fin_idx = ptr(file, edge_node, "fin")?;
        let mut f_idx = head_fin_idx;
        let mut hops = 0;
        while f_idx != 0 {
            let fin_node = xnode(file, f_idx)?;
            let v = ptr(file, fin_node, "vertex")?;
            match ch(file, fin_node, "sense")? {
                '+' if end_idx == 0 => end_idx = v,
                '-' if start_idx == 0 => start_idx = v,
                _ => {}
            }
            f_idx = ptr(file, fin_node, "other")?;
            if f_idx == head_fin_idx {
                break;
            }
            hops += 1;
            if hops > 10_000 {
                return Err(XtError::BadField {
                    index: edge_idx,
                    what: "fin ring around edge does not close",
                });
            }
        }
        let mut start = if start_idx != 0 {
            Some(self.vertex(start_idx)?)
        } else {
            None
        };
        let mut end = if end_idx != 0 {
            Some(self.vertex(end_idx)?)
        } else {
            None
        };
        // A reversed XT curve sense flips the edge into curve direction.
        if curve_reversed {
            core::mem::swap(&mut start, &mut end);
        }
        if start.is_some() != end.is_some() {
            return Err(XtError::BadField {
                index: edge_idx,
                what: "edge has exactly one vertex",
            });
        }

        let bounds = match (start, end) {
            (None, None) => None,
            (Some(s), Some(e)) => {
                let sp = self.store.vertex_position(s)?;
                let ep = self.store.vertex_position(e)?;
                match curve {
                    Some(curve) => {
                        let curve_geom = self.store.get(curve)?;
                        let recovered = edge_bounds(
                            curve_geom,
                            sp,
                            ep,
                            trim,
                            curve_reversed,
                            self.scope,
                        )
                        .map_err(|error| match error {
                            ProjectionError::Policy(error) => policy_error(error),
                            _ => XtError::BadField {
                                index: edge_idx,
                                what: "could not recover edge parameter bounds on its curve",
                            },
                        })?;
                        Some(recovered.ok_or(XtError::BadField {
                            index: edge_idx,
                            what: "could not recover edge parameter bounds on its curve",
                        })?)
                    }
                    None => Some((0.0, 1.0)),
                }
            }
            _ => unreachable!("checked above"),
        };

        let tol = tolerance(file, edge_node)?;
        if curve.is_none() && tol.is_none() {
            return Err(XtError::BadField {
                index: edge_idx,
                what: "curve-less edge has no tolerance",
            });
        }
        if curve.is_none() && bounds.is_none() {
            return Err(XtError::Unsupported {
                capability: XtCapability::TolerantRingEdges,
                what: "curve-less tolerant ring edges",
            });
        }
        let e = self.store.add(Edge {
            curve,
            vertices: [start, end],
            bounds,
            fins: Vec::new(),
            tolerance: tol,
        });
        self.edges.insert(edge_idx, (e, curve_reversed));
        Ok((e, curve_reversed))
    }

    /// Reconstruct the trimmed SP-curve attached to one real FIN.
    fn fin_pcurve(
        &mut self,
        fin_idx: u32,
        face: FaceId,
        edge: EdgeId,
    ) -> Result<Option<FinPcurve>> {
        let file = self.file;
        let fin_node = xnode(file, fin_idx)?;
        let trim_idx = ptr(file, fin_node, "curve")?;
        if trim_idx == 0 {
            if self.store.get(edge)?.curve.is_none() {
                return Err(XtError::BadField {
                    index: fin_idx,
                    what: "curve-less tolerant edge fin has no SP-curve",
                });
            }
            return Ok(None);
        }
        let trim = xnode(file, trim_idx)?;
        if trim.code != code::TRIMMED_CURVE {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "FIN curve is not a TRIMMED_CURVE",
            });
        }
        if ch(file, trim, "sense")? != '+' {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "FIN TRIMMED_CURVE sense must be positive",
            });
        }
        let sp_idx = ptr(file, trim, "basis_curve")?;
        let sp = xnode(file, sp_idx)?;
        if sp.code != code::SP_CURVE {
            return Err(XtError::BadField {
                index: sp_idx,
                what: "FIN TRIMMED_CURVE basis is not an SP_CURVE",
            });
        }
        let sp_surface = ptr(file, sp, "surface")?;
        let (surface, _) = self.surface(sp_surface)?;
        if surface != self.store.get(face)?.surface {
            return Err(XtError::BadField {
                index: sp_idx,
                what: "SP_CURVE surface is not the FIN's face surface",
            });
        }
        let bcurve_idx = ptr(file, sp, "b_curve")?;
        let pcurve = self.pcurve_b_curve(bcurve_idx)?;
        let p1 = f64_of(file, trim, "parm_1")?;
        let p2 = f64_of(file, trim, "parm_2")?;
        let sp_forward = match ch(file, sp, "sense")? {
            '+' => true,
            '-' => false,
            _ => {
                return Err(XtError::BadField {
                    index: sp_idx,
                    what: "SP_CURVE sense is not +/-",
                });
            }
        };
        if !(p1.is_finite() && p2.is_finite() && p1 != p2 && ((p2 > p1) == sp_forward)) {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "SP-curve trim parameters disagree with basis sense",
            });
        }
        let (t0, t1) = self.store.get(edge)?.bounds.ok_or(XtError::BadField {
            index: fin_idx,
            what: "FIN SP-curve is attached to an unbounded edge",
        })?;
        let scale = (p2 - p1) / (t1 - t0);
        let map = ParamMap1d::affine(scale, p1 - scale * t0).map_err(XtError::Kernel)?;
        let curve = self.store.get(pcurve)?.as_curve();
        let natural = curve.param_range();
        if !natural.contains(p1) || !natural.contains(p2) {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "SP-curve trim parameters lie outside the 2D B-curve domain",
            });
        }
        let trim_point_1 = vector(file, trim, "point_1")?;
        let trim_point_2 = vector(file, trim, "point_2")?;
        let uv1 = curve.eval(p1);
        let uv2 = curve.eval(p2);
        let tolerance = self
            .store
            .get(edge)?
            .tolerance
            .map(EntityTolerance::value)
            .unwrap_or(LINEAR_RESOLUTION)
            .max(LINEAR_RESOLUTION);
        let point1 = self
            .graph
            .query(self.store, |evaluator| {
                evaluator.eval_surface(surface, [uv1.x, uv1.y], SurfaceDerivativeOrder::Position)
            })
            .map_err(policy_error)?
            .map_err(XtError::Evaluation)?
            .p;
        let point2 = self
            .graph
            .query(self.store, |evaluator| {
                evaluator.eval_surface(surface, [uv2.x, uv2.y], SurfaceDerivativeOrder::Position)
            })
            .map_err(policy_error)?
            .map_err(XtError::Evaluation)?
            .p;
        if point1.dist(trim_point_1) > tolerance || point2.dist(trim_point_2) > tolerance {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "TRIMMED_CURVE points do not match its SP-curve parameters",
            });
        }
        let degeneracies = self
            .graph
            .query(self.store, |evaluator| {
                evaluator.surface_degeneracies(surface)
            })
            .map_err(policy_error)?
            .map_err(XtError::Evaluation)?;
        let endpoint_kinds = infer_pcurve_endpoint_kinds(&degeneracies, curve, map, [t0, t1]);
        let use_ = FinPcurve::new(pcurve, ParamRange::new(p1.min(p2), p1.max(p2)), map)
            .map_err(XtError::Kernel)?
            .with_endpoint_kinds(endpoint_kinds);
        Ok(Some(use_))
    }

    fn pcurve_b_curve(&mut self, curve_idx: u32) -> Result<Curve2dId> {
        if let Some(&curve) = self.pcurves.get(&curve_idx) {
            return Ok(curve);
        }
        let node = xnode(self.file, curve_idx)?;
        if node.code != code::B_CURVE {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "SP_CURVE parameter geometry is not a B_CURVE",
            });
        }
        let curve = self.b_curve_2d(curve_idx, node)?;
        let id = self
            .store
            .insert_pcurve(Curve2dGeom::Nurbs(curve))
            .map_err(XtError::Kernel)?;
        self.pcurves.insert(curve_idx, id);
        Ok(id)
    }

    fn b_curve_2d(&mut self, curve_idx: u32, node: &Node) -> Result<NurbsCurve2d> {
        let file = self.file;
        let nurbs_idx = ptr(file, node, "nurbs")?;
        let n = xnode(file, nurbs_idx)?;
        let degree = f64_of(file, n, "degree")? as usize;
        let n_vertices = f64_of(file, n, "n_vertices")? as usize;
        let vertex_dim = f64_of(file, n, "vertex_dim")? as usize;
        if logical_of(file, n, "periodic")? {
            return Err(XtError::Unsupported {
                capability: XtCapability::PeriodicPcurves,
                what: "periodic 2D B-curves",
            });
        }
        let rational = logical_of(file, n, "rational")?;
        let knots = self.knot_vector(ptr(file, n, "knots")?, ptr(file, n, "knot_mult")?)?;
        let raw = self.doubles(ptr(file, n, "bspline_vertices")?, "vertices")?;
        if raw.len() != n_vertices * vertex_dim {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "2D bspline vertex array length mismatch",
            });
        }
        let (points, weights) = split_poles_2d(&raw, vertex_dim, rational)?;
        NurbsCurve2d::new(degree, knots, points, weights).map_err(XtError::Kernel)
    }

    /// Convert an XT curve node to kernel geometry. Returns the curve and
    /// whether the XT sense was `-` (reversed against its
    /// parameterization).
    fn curve(&mut self, curve_idx: u32) -> Result<(CurveId, bool)> {
        if let Some(&(c, r)) = self.curves.get(&curve_idx) {
            return Ok((c, r));
        }
        let file = self.file;
        let node = xnode(file, curve_idx)?;
        let reversed = match ch(file, node, "sense")? {
            '+' => false,
            '-' => true,
            _ => {
                return Err(XtError::BadField {
                    index: curve_idx,
                    what: "curve sense is not +/-",
                });
            }
        };
        if node.code == code::INTERSECTION {
            let c = self.intersection_curve(curve_idx, node)?;
            self.curves.insert(curve_idx, (c, reversed));
            return Ok((c, reversed));
        }
        let geom: CurveGeom = match node.code {
            code::LINE => {
                let origin = in_size_box(vector(file, node, "pvec")?)?;
                let dir = vector(file, node, "direction")?;
                Line::new(origin, dir).map_err(XtError::Kernel)?.into()
            }
            code::CIRCLE => {
                let frame = frame_from(file, node, "centre", "normal", "x_axis")?;
                let radius = f64_of(file, node, "radius")?;
                Circle::new(frame, radius).map_err(XtError::Kernel)?.into()
            }
            code::ELLIPSE => {
                let frame = frame_from(file, node, "centre", "normal", "x_axis")?;
                let major = f64_of(file, node, "major_radius")?;
                let minor = f64_of(file, node, "minor_radius")?;
                Ellipse::new(frame, major, minor)
                    .map_err(XtError::Kernel)?
                    .into()
            }
            code::B_CURVE => self.b_curve(curve_idx, node)?.into(),
            code::SP_CURVE => self.plane_sp_curve(curve_idx, node)?.into(),
            code::PE_CURVE => {
                return Err(XtError::Unsupported {
                    capability: XtCapability::ProceduralCurves,
                    what: "procedural curves (intersection/SP/foreign) — Tier 2",
                });
            }
            _ => {
                return Err(XtError::BadField {
                    index: curve_idx,
                    what: "node referenced as a curve is not a curve",
                });
            }
        };
        let c = self.store.insert_curve(geom).map_err(XtError::Kernel)?;
        self.curves.insert(curve_idx, (c, reversed));
        Ok((c, reversed))
    }

    /// Lift a native 2D B-curve through a direct Plane parameterization.
    /// An affine map commutes exactly with the NURBS basis, so lifting the
    /// control points preserves the complete curve and its parameterization.
    fn plane_sp_curve(&mut self, curve_idx: u32, node: &Node) -> Result<NurbsCurve> {
        let file = self.file;
        if ptr(file, node, "original")? != 0
            || !matches!(field(file, node, "tolerance_to_original")?, Value::Null)
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::ProceduralCurves,
                what: "only native Plane SP_CURVEs without an original or approximation tolerance are supported",
            });
        }

        let surface_idx = ptr(file, node, "surface")?;
        let surface = xnode(file, surface_idx)?;
        if surface.code != code::PLANE {
            return Err(XtError::Unsupported {
                capability: XtCapability::ProceduralCurves,
                what: "only native SP_CURVEs on direct Plane surfaces are supported",
            });
        }
        let plane = Plane::new(frame_from(file, surface, "pvec", "normal", "x_axis")?);

        let bcurve_idx = ptr(file, node, "b_curve")?;
        let bcurve = xnode(file, bcurve_idx)?;
        if bcurve.code != code::B_CURVE {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "SP_CURVE parameter geometry is not a B_CURVE",
            });
        }
        if ch(file, bcurve, "sense")? != '+' {
            return Err(XtError::Unsupported {
                capability: XtCapability::ProceduralCurves,
                what: "native Plane SP_CURVE parameter B-curve sense must be positive",
            });
        }
        let nurbs_idx = ptr(file, bcurve, "nurbs")?;
        let nurbs = xnode(file, nurbs_idx)?;
        if logical_of(file, nurbs, "periodic")?
            || logical_of(file, nurbs, "closed")?
            || logical_of(file, nurbs, "rational")?
            || f64_of(file, nurbs, "vertex_dim")? != 2.0
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::ProceduralCurves,
                what: "only open nonperiodic nonrational two-dimensional Plane SP_CURVE B-geometry is supported",
            });
        }

        let pcurve = self.pcurve_b_curve(bcurve_idx)?;
        let pcurve = match self.store.get(pcurve)? {
            Curve2dGeom::Nurbs(curve) => curve.clone(),
            _ => unreachable!("B_CURVE parameter geometry reconstructs as NURBS"),
        };
        let frame = plane.frame();
        let points = pcurve
            .points()
            .iter()
            .map(|point| in_size_box(frame.origin() + frame.x() * point.x + frame.y() * point.y))
            .collect::<Result<Vec<_>>>()?;
        NurbsCurve::new(
            pcurve.degree(),
            pcurve.knots().as_slice().to_vec(),
            points,
            None,
        )
        .map_err(XtError::Kernel)
    }

    /// Import canonical finite-open charts and the one-shared-`H` periodic
    /// equal-limit form without recomputing the spatial intersection. The
    /// transmitted positions and interleaved UVs become one shared degree-1
    /// basis and are certified over every span.
    fn intersection_curve(&mut self, curve_idx: u32, node: &Node) -> Result<CurveId> {
        let file = self.file;
        let source_indices = match field(file, node, "surface")? {
            Value::Arr(values) if values.len() == 2 => {
                let first = values[0].as_ptr().unwrap_or(0);
                let second = values[1].as_ptr().unwrap_or(0);
                if first == 0 || second == 0 || first == second {
                    return Err(XtError::BadField {
                        index: curve_idx,
                        what: "INTERSECTION must reference two distinct source surfaces",
                    });
                }
                [first, second]
            }
            _ => {
                return Err(XtError::BadField {
                    index: curve_idx,
                    what: "INTERSECTION surface field is not two pointers",
                });
            }
        };
        let mut direct_planes = Vec::with_capacity(2);
        let mut source_senses = Vec::with_capacity(2);
        let mut has_offset_source = false;
        let mut offset_source_count = 0_u8;
        let mut nurbs_source_indices = [None, None];
        let mut nurbs_source_count = 0_u8;
        let mut offset_nurbs_sources = [false, false];
        for (operand, &index) in source_indices.iter().enumerate() {
            let source = xnode(file, index)?;
            let sense = match ch(file, source, "sense")? {
                '+' => '+',
                '-' => '-',
                _ => {
                    return Err(XtError::BadField {
                        index,
                        what: "INTERSECTION source surface sense is not +/-",
                    });
                }
            };
            source_senses.push(sense);
            match source.code {
                code::PLANE => direct_planes.push(Some(Plane::new(frame_from(
                    file, source, "pvec", "normal", "x_axis",
                )?))),
                code::OFFSET_SURF => {
                    direct_planes.push(None);
                    has_offset_source = true;
                    offset_source_count += 1;
                    let basis = ptr(file, source, "surface")?;
                    if xnode(file, basis)?.code == code::B_SURFACE {
                        nurbs_source_indices[operand] = Some(basis);
                        offset_nurbs_sources[operand] = true;
                    }
                }
                code::B_SURFACE => {
                    direct_planes.push(None);
                    nurbs_source_indices[operand] = Some(index);
                    nurbs_source_count += 1;
                }
                _ => {
                    return Err(XtError::Unsupported {
                        capability: XtCapability::IntersectionSurfaceFamily,
                        what: "transmitted intersection source is not a supported Plane/Plane, Plane/Offset, Offset/Offset, Plane/B-surface, Offset/B-surface, or B-surface/B-surface family",
                    });
                }
            }
        }
        let nurbs_trace_count = nurbs_source_indices.iter().flatten().count() as u64;
        let has_nurbs_source = nurbs_trace_count != 0;

        let chart_idx = ptr(file, node, "chart")?;
        let chart = xnode(file, chart_idx)?;
        if chart.code != code::CHART {
            return Err(XtError::BadField {
                index: chart_idx,
                what: "INTERSECTION chart pointer is not a CHART",
            });
        }
        let base_parameter = f64_of(file, chart, "base_parameter")?;
        let base_scale = f64_of(file, chart, "base_scale")?;
        if base_parameter != 0.0 || base_scale != 1.0 {
            return Err(XtError::Unsupported {
                capability: XtCapability::IntersectionChartConvention,
                what: "only canonical base_parameter=0/base_scale=1 charts are supported",
            });
        }
        let count_i64 = field(file, chart, "chart_count")?
            .as_int()
            .ok_or(XtError::BadField {
                index: chart_idx,
                what: "CHART chart_count is not integral",
            })?;
        let count = usize::try_from(count_i64).map_err(|_| XtError::BadField {
            index: chart_idx,
            what: "CHART chart_count is negative or too large",
        })?;
        if count < 2 {
            return Err(XtError::BadField {
                index: chart_idx,
                what: "INTERSECTION CHART must contain at least two positions",
            });
        }
        let early_source_surfaces = if has_offset_source || has_nurbs_source {
            Some([
                self.surface(source_indices[0])?.0,
                self.surface(source_indices[1])?.0,
            ])
        } else {
            None
        };
        if offset_source_count == 2
            && !transmitted_offset_chains_are_independent(file, source_indices)?
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::IntersectionSurfaceFamily,
                what: "Offset/Offset transmitted sources must have independent basis chains",
            });
        }
        let mut nurbs_effective_offset_planes = [None, None];
        let planes = if has_nurbs_source {
            for (index, direct_plane) in direct_planes.iter().enumerate() {
                if direct_plane.is_some() {
                    continue;
                }
                let source_node = xnode(file, source_indices[index])?;
                if source_node.code != code::OFFSET_SURF {
                    continue;
                }
                if offset_nurbs_sources[index] {
                    continue;
                }
                let source =
                    early_source_surfaces.expect("offset sources resolve before proof")[index];
                let plane = self
                    .graph
                    .query(self.store, |evaluator| {
                        evaluator.surface_exact_plane(source)
                    })
                    .map_err(policy_error)?
                    .map_err(XtError::Evaluation)?
                    .ok_or(XtError::Unsupported {
                        capability: XtCapability::IntersectionSurfaceFamily,
                        what: "INTERSECTION offset source does not resolve to an exact plane field",
                    })?;
                nurbs_effective_offset_planes[index] = Some(plane);
            }
            None
        } else {
            let mut effective_planes = Vec::with_capacity(2);
            for (index, direct_plane) in direct_planes.iter().copied().enumerate() {
                if let Some(plane) = direct_plane {
                    effective_planes.push(plane);
                    continue;
                }
                let source =
                    early_source_surfaces.expect("offset sources resolve before proof")[index];
                let plane = self
                    .graph
                    .query(self.store, |evaluator| {
                        evaluator.surface_exact_plane(source)
                    })
                    .map_err(policy_error)?
                    .map_err(XtError::Evaluation)?
                    .ok_or(XtError::Unsupported {
                        capability: XtCapability::IntersectionSurfaceFamily,
                        what: "INTERSECTION offset source does not resolve to an exact plane field",
                    })?;
                effective_planes.push(plane);
            }
            Some([effective_planes[0], effective_planes[1]])
        };

        let mut positions = match field(file, chart, "hvec")? {
            Value::Arr(values) if values.len() == count => values
                .iter()
                .map(|value| {
                    value.as_vector().ok_or(XtError::BadField {
                        index: chart_idx,
                        what: "INTERSECTION CHART contains a null or malformed position",
                    })
                })
                .map(|value| value.and_then(|v| in_size_box(Point3::new(v[0], v[1], v[2]))))
                .collect::<Result<Vec<_>>>()?,
            Value::Arr(_) => {
                return Err(XtError::BadField {
                    index: chart_idx,
                    what: "INTERSECTION CHART position count does not match chart_count",
                });
            }
            _ => {
                return Err(XtError::BadField {
                    index: chart_idx,
                    what: "INTERSECTION CHART hvec is not an array",
                });
            }
        };
        if let Some(planes) = planes {
            let oriented_normal = |index: usize| {
                let normal = planes[index].frame().z();
                if source_senses[index] == '+' {
                    normal
                } else {
                    -normal
                }
            };
            let expected_step = oriented_normal(0)
                .cross(oriented_normal(1))
                .normalized()
                .ok_or(XtError::Unsupported {
                    capability: XtCapability::IntersectionSurfaceFamily,
                    what: "parallel or coincident exact plane fields are not supported",
                })?
                * base_scale;
            if positions
                .windows(2)
                .any(|pair| pair[1] - pair[0] != expected_step)
            {
                return Err(XtError::Unsupported {
                    capability: XtCapability::IntersectionChartConvention,
                    what: "CHART positions do not prove the canonical affine recurrence",
                });
            }
        }

        let chordal_error = f64_of(file, chart, "chordal_error")?;
        let angular_error = f64_of(file, chart, "angular_error")?;
        let parameter_error = match field(file, chart, "parameter_error")? {
            Value::Arr(values) if values.len() == 2 => values
                .iter()
                .map(|value| match value {
                    Value::Null => Ok(None),
                    value => value
                        .as_f64()
                        .filter(|value| value.is_finite() && *value >= 0.0)
                        .map(Some)
                        .ok_or(XtError::BadField {
                            index: chart_idx,
                            what: "CHART parameter_error is neither null nor finite nonnegative",
                        }),
                })
                .collect::<Result<Vec<_>>>()?
                .try_into()
                .expect("a two-element parameter-error array remains two elements"),
            _ => {
                return Err(XtError::BadField {
                    index: chart_idx,
                    what: "CHART parameter_error is not a pair",
                });
            }
        };
        let metadata = TransmittedIntersectionChartMetadata::new(
            base_parameter,
            base_scale,
            chordal_error,
            angular_error,
            parameter_error,
        )
        .map_err(|source| XtError::IntersectionCertificate {
            index: curve_idx,
            source,
        })?;
        let declaration = self.body_linear_tolerance.ok_or(XtError::BadField {
            index: curve_idx,
            what: "owning BODY has no numeric res_linear declaration",
        })?;
        if !declaration.is_finite() || declaration < 0.0 {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "owning BODY res_linear is not finite and nonnegative",
            });
        }
        let proof_tolerance = declaration.max(chordal_error);

        let start_idx = ptr(file, node, "start")?;
        let end_idx = ptr(file, node, "end")?;
        if start_idx == 0 || end_idx == 0 {
            return Err(XtError::Unsupported {
                capability: XtCapability::IntersectionLimits,
                what: "transmitted intersection has a null limit",
            });
        }
        let equal_limits = start_idx == end_idx;
        let end_limit = xnode(file, end_idx)?;
        let terminated = !equal_limits
            && end_limit.code == code::LIMIT
            && matches!(file.field(end_limit, "type"), Some(Value::Char('T')));
        let (start, end) = if equal_limits {
            let limit = equal_intersection_limit(file, start_idx)?;
            (limit, limit)
        } else if terminated {
            let start = intersection_limit(file, start_idx)?;
            let [singularity, branch] = terminated_intersection_limit(file, end_idx)?;
            if branch.dist(positions[count - 1]) > proof_tolerance {
                return Err(XtError::BadField {
                    index: curve_idx,
                    what: "INTERSECTION terminator branch point does not match the CHART endpoint",
                });
            }
            let separation = singularity.dist(branch);
            if separation == 0.0 || separation > proof_tolerance {
                return Err(XtError::BadField {
                    index: curve_idx,
                    what: "INTERSECTION terminator singularity is not distinct and within chart tolerance of its branch point",
                });
            }
            positions.push(singularity);
            (start, singularity)
        } else {
            (
                intersection_limit(file, start_idx)?,
                intersection_limit(file, end_idx)?,
            )
        };
        if start.dist(positions[0]) > proof_tolerance
            || end.dist(positions[positions.len() - 1]) > proof_tolerance
        {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "INTERSECTION LIMIT endpoints do not match the CHART endpoints",
            });
        }
        if equal_limits && positions[0].dist(positions[count - 1]) > proof_tolerance {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "equal-limit INTERSECTION CHART is not spatially closed",
            });
        }

        let retained_count = positions.len();
        let retained_count_u64 = u64::try_from(retained_count).map_err(|_| XtError::BadField {
            index: chart_idx,
            what: "INTERSECTION retained sample count is too large",
        })?;
        let (proof_work, proof_depth) = if has_nurbs_source {
            let plane_trace_count = 2_u64 - nurbs_trace_count;
            let mut proof_work =
                retained_count_u64
                    .checked_mul(plane_trace_count)
                    .ok_or(XtError::BadField {
                        index: chart_idx,
                        what: "INTERSECTION CHART plane-trace proof work overflows",
                    })?;
            for surface_index in nurbs_source_indices.into_iter().flatten() {
                proof_work = proof_work
                    .checked_add(transmitted_nurbs_trace_proof_work(
                        file,
                        surface_index,
                        retained_count_u64,
                    )?)
                    .ok_or(XtError::BadField {
                        index: chart_idx,
                        what: "INTERSECTION CHART B-surface proof work overflows",
                    })?;
            }
            (proof_work, TRANSMITTED_NURBS_TRACE_PROOF_DEPTH as u64)
        } else {
            (
                retained_count_u64.checked_mul(2).ok_or(XtError::BadField {
                    index: chart_idx,
                    what: "INTERSECTION CHART proof work overflows",
                })?,
                1,
            )
        };

        let data_idx = file
            .field(node, "intersection_data")
            .and_then(Value::as_ptr)
            .filter(|&index| index != 0)
            .ok_or(XtError::Unsupported {
                capability: XtCapability::IntersectionChartData,
                what: "INTERSECTION has no modern INTERSECTION_DATA pointer",
            })?;
        let data = xnode(file, data_idx)?;
        if data.code != code::INTERSECTION_DATA {
            return Err(XtError::BadField {
                index: data_idx,
                what: "intersection_data pointer is not INTERSECTION_DATA(204)",
            });
        }
        if field(file, data, "uv_type")?.as_int() != Some(4) {
            return Err(XtError::Unsupported {
                capability: XtCapability::IntersectionChartData,
                what: "only INTERSECTION_DATA uv_type=4 is supported",
            });
        }
        let expected_values = retained_count.checked_mul(4).ok_or(XtError::BadField {
            index: data_idx,
            what: "INTERSECTION_DATA expected value count overflows",
        })?;
        let values = match field(file, data, "values")? {
            Value::Arr(values) if values.len() == expected_values => values,
            Value::Arr(_) => {
                return Err(XtError::BadField {
                    index: data_idx,
                    what: "INTERSECTION_DATA values length is not four times the retained sample count",
                });
            }
            _ => {
                return Err(XtError::BadField {
                    index: data_idx,
                    what: "INTERSECTION_DATA values is not an array",
                });
            }
        };
        let exact_trace_planes = [
            direct_planes[0]
                .or(nurbs_effective_offset_planes[0])
                .or_else(|| planes.map(|planes| planes[0])),
            direct_planes[1]
                .or(nurbs_effective_offset_planes[1])
                .or_else(|| planes.map(|planes| planes[1])),
        ];
        let finite_open_plane_nurbs_interior_omissions = !equal_limits
            && !terminated
            && nurbs_trace_count == 1
            && exact_trace_planes.iter().flatten().count() == 1;
        let mut uv = [
            Vec::with_capacity(retained_count),
            Vec::with_capacity(retained_count),
        ];
        for (sample, tuple) in values.chunks_exact(4).enumerate() {
            for operand in 0..2 {
                let offset = operand * 2;
                match (
                    tuple[offset].as_f64().filter(|value| value.is_finite()),
                    tuple[offset + 1].as_f64().filter(|value| value.is_finite()),
                ) {
                    (Some(u), Some(v)) => uv[operand].push(Point2::new(u, v)),
                    (None, None)
                        if (terminated
                            || (finite_open_plane_nurbs_interior_omissions
                                && sample != 0
                                && sample + 1 != retained_count))
                            && matches!(tuple[offset], Value::Null)
                            && matches!(tuple[offset + 1], Value::Null)
                            && exact_trace_planes[operand].is_some() =>
                    {
                        let plane = exact_trace_planes[operand].expect("guarded exact trace plane");
                        let frame = plane.frame();
                        let displacement = positions[sample] - frame.origin();
                        uv[operand].push(Point2::new(
                            displacement.dot(frame.x()),
                            displacement.dot(frame.y()),
                        ));
                    }
                    _ => {
                        return Err(XtError::Unsupported {
                            capability: XtCapability::IntersectionChartData,
                            what: "INTERSECTION_DATA contains null or non-finite UV values",
                        });
                    }
                }
            }
        }

        let dual_offset_nurbs = offset_nurbs_sources == [true, true];
        let two_sample_dual_offset =
            dual_offset_nurbs && retained_count == 2 && !equal_limits && !terminated;
        let quadratic_dual_offset =
            dual_offset_nurbs && retained_count == 3 && !equal_limits && !terminated;
        let cubic_dual_offset =
            dual_offset_nurbs && retained_count == 4 && !equal_limits && !terminated;
        let five_sample_dual_offset =
            dual_offset_nurbs && retained_count == 5 && !equal_limits && !terminated;
        let seven_sample_dual_offset =
            dual_offset_nurbs && retained_count == 7 && !equal_limits && !terminated;
        if dual_offset_nurbs
            && !two_sample_dual_offset
            && !quadratic_dual_offset
            && !cubic_dual_offset
            && !five_sample_dual_offset
            && !seven_sample_dual_offset
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::IntersectionSurfaceFamily,
                what: "dual Offset(B-surface) charts require a canonical finite-open two-sample line, three-sample quadratic, four-sample cubic, five-sample polyline, or seven-sample polyline family",
            });
        }
        let quadratic_position_samples =
            quadratic_dual_offset.then(|| [positions[0], positions[1], positions[2]]);
        let cubic_position_samples =
            cubic_dual_offset.then(|| [positions[0], positions[1], positions[2], positions[3]]);
        let (carrier_degree, knots, carrier_points) = if quadratic_dual_offset {
            let midpoint_control = positions[1] * 2.0 - (positions[0] + positions[2]) * 0.5;
            (
                2,
                vec![0.0, 0.0, 0.0, 2.0, 2.0, 2.0],
                vec![positions[0], midpoint_control, positions[2]],
            )
        } else if cubic_dual_offset {
            let first = positions[1] * 27.0 - positions[0] * 8.0 - positions[3];
            let second = positions[2] * 27.0 - positions[0] - positions[3] * 8.0;
            (
                3,
                vec![0.0, 0.0, 0.0, 0.0, 3.0, 3.0, 3.0, 3.0],
                vec![
                    positions[0],
                    (first * 2.0 - second) / 18.0,
                    (second * 2.0 - first) / 18.0,
                    positions[3],
                ],
            )
        } else {
            let mut knots = vec![0.0, 0.0];
            knots.extend((1..retained_count - 1).map(|index| index as f64));
            knots.extend([retained_count_u64.saturating_sub(1) as f64; 2]);
            (1, knots, positions)
        };
        let carrier = NurbsCurve::new(carrier_degree, knots.clone(), carrier_points, None)
            .map_err(XtError::Kernel)?;

        let source_surfaces = if let Some(sources) = early_source_surfaces {
            sources
        } else {
            [
                self.surface(source_indices[0])?.0,
                self.surface(source_indices[1])?.0,
            ]
        };
        if let Some(planes) = planes {
            if equal_limits {
                return Err(XtError::Unsupported {
                    capability: XtCapability::IntersectionLimits,
                    what: "equal-limit chart must close on exactly one certified periodic NURBS axis",
                });
            }
            if !has_offset_source {
                for (index, plane) in planes.iter().copied().enumerate() {
                    if self.store.get(source_surfaces[index])?.as_plane().copied() != Some(plane) {
                        return Err(XtError::BadField {
                            index: curve_idx,
                            what: "live source plane does not match its transmitted declaration",
                        });
                    }
                }
            }
            let pcurves = [
                NurbsCurve2d::new(1, knots.clone(), uv[0].clone(), None)
                    .map_err(XtError::Kernel)?,
                NurbsCurve2d::new(1, knots, uv[1].clone(), None).map_err(XtError::Kernel)?,
            ];
            preflight_intersection_chart(self.scope, retained_count_u64, proof_depth, proof_work)?;
            let certificate = certify_transmitted_plane_intersection_residuals(
                carrier,
                planes,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
            .map_err(|source| XtError::IntersectionCertificate {
                index: curve_idx,
                source,
            })?;
            self.scope
                .ledger_mut()
                .charge(INTERSECTION_CHART_CERTIFICATE_WORK, proof_work)
                .map_err(policy_error)?;
            let pcurve_handles = [
                self.store
                    .insert_pcurve(Curve2dGeom::Nurbs(pcurves[0].clone()))
                    .map_err(XtError::Kernel)?,
                self.store
                    .insert_pcurve(Curve2dGeom::Nurbs(pcurves[1].clone()))
                    .map_err(XtError::Kernel)?,
            ];
            return self
                .store
                .insert_verified_transmitted_plane_intersection_curve(
                    source_surfaces,
                    pcurve_handles,
                    certificate,
                )
                .map_err(XtError::Kernel);
        }

        let mut traces = Vec::with_capacity(2);
        for (index, direct_plane) in direct_planes.iter().copied().enumerate() {
            if let Some(plane) = direct_plane {
                if self.store.get(source_surfaces[index])?.as_plane().copied() != Some(plane) {
                    return Err(XtError::BadField {
                        index: curve_idx,
                        what: "live source plane does not match its transmitted declaration",
                    });
                }
                traces.push(TransmittedNurbsIntersectionTrace::Plane(plane));
            } else if let Some(plane) = nurbs_effective_offset_planes[index] {
                traces.push(TransmittedNurbsIntersectionTrace::Plane(plane));
            } else if offset_nurbs_sources[index] {
                let source = self.store.get(source_surfaces[index])?;
                let offset = source.as_offset().copied().ok_or(XtError::BadField {
                    index: curve_idx,
                    what: "live offset B-surface source is not an offset descriptor",
                })?;
                let basis = self
                    .store
                    .get(offset.basis())?
                    .as_nurbs()
                    .cloned()
                    .ok_or(XtError::BadField {
                        index: curve_idx,
                        what: "live offset B-surface basis is not its transmitted NURBS declaration",
                    })?;
                traces.push(TransmittedNurbsIntersectionTrace::OffsetNurbs(
                    TransmittedOffsetNurbsTrace::new(basis, offset.signed_distance()),
                ));
            } else {
                let source = self.store.get(source_surfaces[index])?;
                let surface = source.as_nurbs().ok_or(XtError::BadField {
                    index: curve_idx,
                    what: "live B-surface does not match its transmitted declaration",
                })?;
                traces.push(TransmittedNurbsIntersectionTrace::Nurbs(surface.clone()));
            }
        }
        let traces: [TransmittedNurbsIntersectionTrace; 2] = traces
            .try_into()
            .expect("two transmitted sources remain two ordered traces");
        if equal_limits {
            canonicalize_equal_limit_periodic_trace_endpoints(curve_idx, &traces, &mut uv)?;
        } else {
            canonicalize_trace_endpoint_roundoff(&traces, &mut uv);
        }
        let quadratic_canonicalized_pcurve_samples = quadratic_dual_offset.then(|| {
            [
                [uv[0][0], uv[0][1], uv[0][2]],
                [uv[1][0], uv[1][1], uv[1][2]],
            ]
        });
        let cubic_canonicalized_pcurve_samples = cubic_dual_offset.then(|| {
            [
                [uv[0][0], uv[0][1], uv[0][2], uv[0][3]],
                [uv[1][0], uv[1][1], uv[1][2], uv[1][3]],
            ]
        });
        let pcurve_points = if quadratic_dual_offset {
            uv.each_ref().map(|points| {
                vec![
                    points[0],
                    points[1] * 2.0 - (points[0] + points[2]) * 0.5,
                    points[2],
                ]
            })
        } else if cubic_dual_offset {
            uv.each_ref().map(|points| {
                let first = points[1] * 27.0 - points[0] * 8.0 - points[3];
                let second = points[2] * 27.0 - points[0] - points[3] * 8.0;
                vec![
                    points[0],
                    (first * 2.0 - second) / 18.0,
                    (second * 2.0 - first) / 18.0,
                    points[3],
                ]
            })
        } else {
            uv
        };
        let pcurves = [
            NurbsCurve2d::new(
                carrier_degree,
                knots.clone(),
                pcurve_points[0].clone(),
                None,
            )
            .map_err(XtError::Kernel)?,
            NurbsCurve2d::new(carrier_degree, knots, pcurve_points[1].clone(), None)
                .map_err(XtError::Kernel)?,
        ];
        preflight_intersection_chart(self.scope, retained_count_u64, proof_depth, proof_work)?;
        let certificate = if two_sample_dual_offset {
            certify_transmitted_two_sample_dual_offset_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
        } else if quadratic_dual_offset {
            certify_transmitted_quadratic_dual_offset_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                quadratic_position_samples.expect("quadratic carrier retains three positions"),
                quadratic_canonicalized_pcurve_samples
                    .expect("quadratic pcurves retain three canonicalized paired UV tuples"),
                metadata,
                proof_tolerance,
            )
        } else if cubic_dual_offset {
            certify_transmitted_cubic_dual_offset_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                cubic_position_samples.expect("cubic carrier retains four positions"),
                cubic_canonicalized_pcurve_samples
                    .expect("cubic pcurves retain four canonicalized paired UV tuples"),
                metadata,
                proof_tolerance,
            )
        } else if five_sample_dual_offset {
            certify_transmitted_five_sample_dual_offset_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
        } else if seven_sample_dual_offset {
            certify_transmitted_seven_sample_dual_offset_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
        } else if offset_nurbs_sources.into_iter().any(|offset| offset) {
            certify_transmitted_offset_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
        } else if nurbs_source_count == 2 {
            certify_transmitted_nurbs_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
        } else {
            certify_transmitted_plane_nurbs_intersection_residuals(
                carrier,
                traces,
                pcurves.clone(),
                metadata,
                proof_tolerance,
            )
        }
        .map_err(|source| XtError::IntersectionCertificate {
            index: curve_idx,
            source,
        })?;
        let certificate = if equal_limits {
            certificate
                .with_certified_carrier_periodicity()
                .map_err(|source| XtError::IntersectionCertificate {
                    index: curve_idx,
                    source,
                })?
        } else {
            certificate
        };
        self.scope
            .ledger_mut()
            .charge(INTERSECTION_CHART_CERTIFICATE_WORK, proof_work)
            .map_err(policy_error)?;
        let pcurve_handles = [
            self.store
                .insert_pcurve(Curve2dGeom::Nurbs(pcurves[0].clone()))
                .map_err(XtError::Kernel)?,
            self.store
                .insert_pcurve(Curve2dGeom::Nurbs(pcurves[1].clone()))
                .map_err(XtError::Kernel)?,
        ];
        self.store
            .insert_verified_transmitted_nurbs_intersection_curve(
                source_surfaces,
                pcurve_handles,
                certificate,
            )
            .map_err(XtError::Kernel)
    }

    fn b_curve(&mut self, curve_idx: u32, node: &Node) -> Result<NurbsCurve> {
        let file = self.file;
        let nurbs_idx = ptr(file, node, "nurbs")?;
        let n = xnode(file, nurbs_idx)?;
        let degree = f64_of(file, n, "degree")? as usize;
        let n_vertices = f64_of(file, n, "n_vertices")? as usize;
        let vertex_dim = f64_of(file, n, "vertex_dim")? as usize;
        if logical_of(file, n, "periodic")? {
            return Err(XtError::Unsupported {
                capability: XtCapability::PeriodicNurbsCurves,
                what: "periodic B-curves (kernel periodic NURBS lands at M3)",
            });
        }
        let rational = logical_of(file, n, "rational")?;
        let knots = self.knot_vector(ptr(file, n, "knots")?, ptr(file, n, "knot_mult")?)?;
        let raw = self.doubles(ptr(file, n, "bspline_vertices")?, "vertices")?;
        if raw.len() != n_vertices * vertex_dim {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "bspline vertex array length mismatch",
            });
        }
        let (points, weights) = split_poles(&raw, vertex_dim, rational)?;
        for p in &points {
            in_size_box(*p)?;
        }
        NurbsCurve::new(degree, knots, points, weights).map_err(XtError::Kernel)
    }

    fn surface(&mut self, surface_idx: u32) -> Result<(SurfaceId, char)> {
        if let Some(&(s, sense)) = self.surfaces.get(&surface_idx) {
            return Ok((s, sense));
        }
        if let Some(start) = self
            .surface_stack
            .iter()
            .position(|&active| active == surface_idx)
        {
            let mut path = self.surface_stack[start..].to_vec();
            path.push(surface_idx);
            return Err(XtError::SurfaceDependencyCycle { path });
        }
        self.surface_stack.push(surface_idx);
        let result = self.surface_unmemoized(surface_idx);
        let popped = self.surface_stack.pop();
        debug_assert_eq!(popped, Some(surface_idx));
        if let Ok(value) = result {
            self.surfaces.insert(surface_idx, value);
        }
        result
    }

    fn surface_unmemoized(&mut self, surface_idx: u32) -> Result<(SurfaceId, char)> {
        let file = self.file;
        let node = xnode(file, surface_idx)?.clone();
        let sense = match ch(file, &node, "sense")? {
            '+' => '+',
            '-' => '-',
            _ => {
                return Err(XtError::BadField {
                    index: surface_idx,
                    what: "surface sense is not +/-",
                });
            }
        };
        let geom: SurfaceGeom = match node.code {
            code::PLANE => {
                let frame = frame_from(file, &node, "pvec", "normal", "x_axis")?;
                Plane::new(frame).into()
            }
            code::CYLINDER => {
                let frame = frame_from(file, &node, "pvec", "axis", "x_axis")?;
                let radius = f64_of(file, &node, "radius")?;
                Cylinder::new(frame, radius)
                    .map_err(XtError::Kernel)?
                    .into()
            }
            code::CONE => cone_from(file, &node)?.into(),
            code::SPHERE => {
                let frame = frame_from(file, &node, "centre", "axis", "x_axis")?;
                let radius = f64_of(file, &node, "radius")?;
                Sphere::new(frame, radius).map_err(XtError::Kernel)?.into()
            }
            code::TORUS => {
                let frame = frame_from(file, &node, "centre", "axis", "x_axis")?;
                let major = f64_of(file, &node, "major_radius")?;
                let minor = f64_of(file, &node, "minor_radius")?;
                if major <= minor {
                    return Err(XtError::Unsupported {
                        capability: XtCapability::SelfIntersectingTori,
                        what: "self-intersecting (apple/lemon) tori",
                    });
                }
                Torus::new(frame, major, minor)
                    .map_err(XtError::Kernel)?
                    .into()
            }
            code::B_SURFACE => self.b_surface(surface_idx, &node)?.into(),
            code::OFFSET_SURF => {
                match ch(file, &node, "check")? {
                    'U' | 'V' => {}
                    'I' => {
                        return Err(XtError::BadField {
                            index: surface_idx,
                            what: "OFFSET_SURF check I declares invalid geometry",
                        });
                    }
                    _ => {
                        return Err(XtError::BadField {
                            index: surface_idx,
                            what: "OFFSET_SURF check is not U/V/I",
                        });
                    }
                }
                let _true_offset = logical_of(file, &node, "true_offset")?;
                match field(file, &node, "scale")? {
                    Value::Null | Value::Double(_) | Value::Int(_) => {}
                    _ => {
                        return Err(XtError::BadField {
                            index: surface_idx,
                            what: "OFFSET_SURF scale is not null or numeric",
                        });
                    }
                }
                let offset = f64_of(file, &node, "offset")?;
                if !offset.is_finite() || offset.abs() <= LINEAR_RESOLUTION {
                    return Err(XtError::BadField {
                        index: surface_idx,
                        what: "OFFSET_SURF offset must be finite and exceed linear resolution",
                    });
                }
                let basis_idx = ptr(file, &node, "surface")?;
                if basis_idx == 0 {
                    return Err(XtError::BadField {
                        index: surface_idx,
                        what: "OFFSET_SURF basis is null",
                    });
                }
                let (basis, basis_sense) = self.surface(basis_idx)?;
                if basis_sense != sense {
                    return Err(XtError::BadField {
                        index: surface_idx,
                        what: "OFFSET_SURF and basis senses differ",
                    });
                }
                let signed_distance = if basis_sense == '+' { offset } else { -offset };
                OffsetSurfaceDescriptor::new(basis, signed_distance).into()
            }
            code::SWEPT_SURF
            | code::SPUN_SURF
            | code::BLENDED_EDGE
            | code::BLEND_BOUND
            | code::PE_SURF => {
                return Err(XtError::Unsupported {
                    capability: XtCapability::ProceduralSurfaces,
                    what: "procedural surfaces (swept/spun/blend/foreign) — Tier 2",
                });
            }
            _ => {
                return Err(XtError::BadField {
                    index: surface_idx,
                    what: "node referenced as a surface is not a surface",
                });
            }
        };
        let s = self.store.insert_surface(geom).map_err(XtError::Kernel)?;
        Ok((s, sense))
    }

    fn b_surface(&mut self, surface_idx: u32, node: &Node) -> Result<NurbsSurface> {
        let file = self.file;
        let nurbs_idx = ptr(file, node, "nurbs")?;
        let n = xnode(file, nurbs_idx)?;
        let periodic = [
            logical_of(file, n, "u_periodic")?,
            logical_of(file, n, "v_periodic")?,
        ];
        let closed = [
            logical_of(file, n, "u_closed")?,
            logical_of(file, n, "v_closed")?,
        ];
        if periodic != closed {
            return Err(XtError::Unsupported {
                capability: XtCapability::PeriodicNurbsSurfaces,
                what: "B-surface periodic/closed flags do not match the certified clamped-seam representation",
            });
        }
        let rational = logical_of(file, n, "rational")?;
        let u_degree = f64_of(file, n, "u_degree")? as usize;
        let v_degree = f64_of(file, n, "v_degree")? as usize;
        let n_u = f64_of(file, n, "n_u_vertices")? as usize;
        let n_v = f64_of(file, n, "n_v_vertices")? as usize;
        let vertex_dim = f64_of(file, n, "vertex_dim")? as usize;
        let u_knots = self.knot_vector(ptr(file, n, "u_knots")?, ptr(file, n, "u_knot_mult")?)?;
        let v_knots = self.knot_vector(ptr(file, n, "v_knots")?, ptr(file, n, "v_knot_mult")?)?;
        let u_order = u_degree.checked_add(1).ok_or(XtError::BadField {
            index: surface_idx,
            what: "B-surface u degree overflows its order",
        })?;
        let v_order = v_degree.checked_add(1).ok_or(XtError::BadField {
            index: surface_idx,
            what: "B-surface v degree overflows its order",
        })?;
        let implied_n_u = u_knots
            .len()
            .checked_sub(u_order)
            .ok_or(XtError::BadField {
                index: surface_idx,
                what: "B-surface u knot-implied control count underflows",
            })?;
        let implied_n_v = v_knots
            .len()
            .checked_sub(v_order)
            .ok_or(XtError::BadField {
                index: surface_idx,
                what: "B-surface v knot-implied control count underflows",
            })?;
        if [n_u, n_v] != [implied_n_u, implied_n_v] {
            return Err(XtError::BadField {
                index: surface_idx,
                what: "B-surface declared control dimensions do not match its knot-implied control net",
            });
        }
        let raw = self.doubles(ptr(file, n, "bspline_vertices")?, "vertices")?;
        let expected_vertex_values = n_u
            .checked_mul(n_v)
            .and_then(|control_count| control_count.checked_mul(vertex_dim))
            .ok_or(XtError::BadField {
                index: surface_idx,
                what: "B-surface control dimensions overflow the vertex array length",
            })?;
        if raw.len() != expected_vertex_values {
            return Err(XtError::BadField {
                index: surface_idx,
                what: "bspline vertex array length mismatch",
            });
        }
        // Pole ordering: assumed v-fastest (matching the kernel's
        // `i*nv + j` layout). Provisional — to be re-verified against a
        // real-world B-surface part during M3b round-trip testing.
        let (points, weights) = split_poles(&raw, vertex_dim, rational)?;
        for p in &points {
            in_size_box(*p)?;
        }
        let surface = NurbsSurface::new(u_degree, v_degree, u_knots, v_knots, points, weights)
            .map_err(XtError::Kernel)?;
        if periodic
            .into_iter()
            .zip([Dir::U, Dir::V])
            .any(|(is_periodic, dir)| is_periodic && !surface.knots(dir).is_clamped())
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::PeriodicNurbsSurfaces,
                what: "unclamped cyclic periodic B-surface basis",
            });
        }
        if periodic == [false, false] {
            return Ok(surface);
        }
        let seam_tolerance = self.body_linear_tolerance.ok_or(XtError::BadField {
            index: surface_idx,
            what: "periodic B-surface has no owning BODY res_linear seam tolerance",
        })?;
        if !seam_tolerance.is_finite() || seam_tolerance < 0.0 {
            return Err(XtError::BadField {
                index: surface_idx,
                what: "periodic B-surface owning BODY res_linear is not finite and nonnegative",
            });
        }
        surface
            .with_certified_periodicity(periodic, seam_tolerance)
            .map_err(|_| XtError::BadField {
                index: surface_idx,
                what: "periodic B-surface fails its clamped position/C1 seam contract",
            })
    }

    /// Expand an XT (distinct knots, multiplicities) pair into the full
    /// knot vector.
    fn knot_vector(&mut self, knots_idx: u32, mult_idx: u32) -> Result<Vec<f64>> {
        let knot_node = xnode(self.file, knots_idx)?;
        let knots = match field(self.file, knot_node, "knots")? {
            Value::Arr(values) => values,
            _ => {
                return Err(XtError::BadField {
                    index: knots_idx,
                    what: "knot values are not an array",
                });
            }
        };
        let mult_node = xnode(self.file, mult_idx)?;
        let mults = match field(self.file, mult_node, "mult")? {
            Value::Arr(values) => values,
            _ => {
                return Err(XtError::BadField {
                    index: mult_idx,
                    what: "knot multiplicities are not an array",
                });
            }
        };
        if mults.len() != knots.len() {
            return Err(XtError::BadField {
                index: mult_idx,
                what: "knot and multiplicity arrays differ in length",
            });
        }
        let mut out = Vec::new();
        for (knot, multiplicity) in knots.iter().zip(mults) {
            let multiplicity = multiplicity.as_int().ok_or(XtError::BadField {
                index: mult_idx,
                what: "knot multiplicity is not integral",
            })?;
            let multiplicity = usize::try_from(multiplicity).map_err(|_| XtError::BadField {
                index: mult_idx,
                what: "knot multiplicity is negative or too large",
            })?;
            if multiplicity == 0 {
                if matches!(knot, Value::Null)
                    || knot.as_f64().is_some_and(|value| value.is_finite())
                {
                    continue;
                }
                return Err(XtError::BadField {
                    index: knots_idx,
                    what: "zero-multiplicity knot padding is neither null nor finite numeric",
                });
            }
            let knot =
                knot.as_f64()
                    .filter(|value| value.is_finite())
                    .ok_or(XtError::BadField {
                        index: knots_idx,
                        what: "positive-multiplicity knot is null, non-numeric, or non-finite",
                    })?;
            out.try_reserve(multiplicity)
                .map_err(|_| XtError::BadField {
                    index: mult_idx,
                    what: "expanded knot vector is too large",
                })?;
            for _ in 0..multiplicity {
                out.push(knot);
            }
        }
        Ok(out)
    }

    fn doubles(&self, idx: u32, name: &'static str) -> Result<Vec<f64>> {
        let node = xnode(self.file, idx)?;
        match field(self.file, node, name)? {
            Value::Arr(vs) => vs
                .iter()
                .map(|v| {
                    v.as_f64().ok_or(XtError::BadField {
                        index: idx,
                        what: "non-numeric value in double array",
                    })
                })
                .collect(),
            _ => Err(XtError::BadField {
                index: idx,
                what: "expected a double array",
            }),
        }
    }
}

/// Build a kernel frame from XT origin/axis/x-axis fields.
fn frame_from(
    file: &XtFile,
    node: &Node,
    origin: &'static str,
    z: &'static str,
    x: &'static str,
) -> Result<Frame> {
    let o = in_size_box(vector(file, node, origin)?)?;
    let zv = vector(file, node, z)?;
    let xv = vector(file, node, x)?;
    Frame::new(o, zv, xv).map_err(XtError::Kernel)
}

/// XT cone → kernel cone. XT: `R(u,v) = P − vA + (X cos u + Y sin u)
/// (r + v tan α)` — the axis points away from the half in use — so the
/// kernel frame takes `z = −A`, giving the same point set under our
/// slant parameterization.
fn cone_from(file: &XtFile, node: &Node) -> Result<Cone> {
    let pvec = in_size_box(vector(file, node, "pvec")?)?;
    let axis = vector(file, node, "axis")?;
    let x_axis = vector(file, node, "x_axis")?;
    let radius = f64_of(file, node, "radius")?;
    let sin_a = f64_of(file, node, "sin_half_angle")?;
    let cos_a = f64_of(file, node, "cos_half_angle")?;
    let half_angle = math::atan2(sin_a, cos_a);
    let frame = Frame::new(pvec, -axis, x_axis).map_err(XtError::Kernel)?;
    Cone::new(frame, radius, half_angle).map_err(XtError::Kernel)
}

/// Split a flat XT pole array into points and optional weights.
/// Rational poles are stored premultiplied (`x·w, y·w, z·w, w`).
fn split_poles(raw: &[f64], dim: usize, rational: bool) -> Result<(Vec<Point3>, Option<Vec<f64>>)> {
    let expected_dim = if rational { 4 } else { 3 };
    if dim != expected_dim {
        return Err(XtError::Unsupported {
            capability: XtCapability::NonstandardNurbsPoles,
            what: "B-geometry with vertex dimension other than 3 (or 4 rational)",
        });
    }
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for pole in raw.chunks_exact(dim) {
        if rational {
            let w = pole[3];
            if w <= 0.0 {
                return Err(XtError::BadField {
                    index: 0,
                    what: "non-positive rational weight",
                });
            }
            points.push(Point3::new(pole[0] / w, pole[1] / w, pole[2] / w));
            weights.push(w);
        } else {
            points.push(Point3::new(pole[0], pole[1], pole[2]));
        }
    }
    Ok((points, if rational { Some(weights) } else { None }))
}

/// Split a flat XT 2D pole array. Rational poles are premultiplied
/// (`u·w, v·w, w`).
fn split_poles_2d(
    raw: &[f64],
    dim: usize,
    rational: bool,
) -> Result<(Vec<Point2>, Option<Vec<f64>>)> {
    let expected_dim = if rational { 3 } else { 2 };
    if dim != expected_dim {
        return Err(XtError::Unsupported {
            capability: XtCapability::NonstandardNurbsPoles,
            what: "2D B-geometry with vertex dimension other than 2 (or 3 rational)",
        });
    }
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for pole in raw.chunks_exact(dim) {
        if rational {
            let w = pole[2];
            if w <= 0.0 {
                return Err(XtError::BadField {
                    index: 0,
                    what: "non-positive 2D rational weight",
                });
            }
            points.push(Point2::new(pole[0] / w, pole[1] / w));
            weights.push(w);
        } else {
            points.push(Point2::new(pole[0], pole[1]));
        }
    }
    Ok((points, if rational { Some(weights) } else { None }))
}

/// Recover the parameter interval of an edge from its endpoint positions
/// on the (natural-direction) curve. `trim` short-circuits with the
/// parameters stored in a trimmed curve, oriented to increase along the
/// natural direction.
fn edge_bounds(
    curve: &CurveGeom,
    start: Point3,
    end: Point3,
    trim: Option<(f64, f64)>,
    curve_reversed: bool,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<Option<(f64, f64)>, ProjectionError> {
    if let Some((p1, p2)) = trim {
        // With a '+' basis sense parm_2 > parm_1; with '-' they come
        // reversed (the edge flip already swapped the vertices).
        let (lo, hi) = if curve_reversed { (p2, p1) } else { (p1, p2) };
        return Ok((hi > lo).then_some((lo, hi)));
    }
    let tau = core::f64::consts::TAU;
    match curve {
        CurveGeom::Line(line) => {
            let t0 = (start - line.origin()).dot(line.dir());
            let t1 = (end - line.origin()).dot(line.dir());
            Ok((t1 > t0).then_some((t0, t1)))
        }
        CurveGeom::Circle(c) => {
            let f = c.frame();
            let angle = |p: Point3| {
                let l = f.to_local(p);
                wrap_periodic(math::atan2(l.y, l.x), 0.0, tau)
            };
            let (t0, t1) = unwrap_interval(angle(start), angle(end), tau);
            Ok(Some((t0, clamp_period_width(t0, t1, tau))))
        }
        CurveGeom::Ellipse(e) => {
            let f = e.frame();
            let angle = |p: Point3| {
                let l = f.to_local(p);
                wrap_periodic(
                    math::atan2(l.y / e.minor_radius(), l.x / e.major_radius()),
                    0.0,
                    tau,
                )
            };
            let (t0, t1) = unwrap_interval(angle(start), angle(end), tau);
            Ok(Some((t0, clamp_period_width(t0, t1, tau))))
        }
        CurveGeom::Nurbs(n) => {
            let range = n.param_range();
            let t0 = kgeom::project::project_to_curve_in_scope(n, start, range, scope)?.t;
            let t1 = kgeom::project::project_to_curve_in_scope(n, end, range, scope)?.t;
            Ok((t1 > t0).then_some((t0, t1)))
        }
        CurveGeom::TransmittedIntersection(intersection) => {
            let range = intersection.certificate().carrier_range();
            Ok(Some((range.lo, range.hi)))
        }
        CurveGeom::TransmittedNurbsIntersection(intersection) => {
            let range = intersection.certificate().carrier_range();
            Ok(Some((range.lo, range.hi)))
        }
        _ => Ok(None),
    }
}

fn infer_pcurve_endpoint_kinds(
    degeneracies: &[kgeom::surface::Degeneracy],
    curve: &dyn Curve2d,
    map: ParamMap1d,
    edge_parameters: [f64; 2],
) -> [PcurveEndpointKind; 2] {
    edge_parameters.map(|t| {
        let uv = curve.eval(map.map(t));
        if degeneracies.iter().any(|degeneracy| {
            let value = match degeneracy.dir {
                kgeom::surface::Dir::U => uv.x,
                kgeom::surface::Dir::V => uv.y,
            };
            let slack = 256.0 * f64::EPSILON * (1.0 + value.abs().max(degeneracy.at.abs()));
            (value - degeneracy.at).abs() <= slack
        }) {
            PcurveEndpointKind::SurfaceSingularity
        } else {
            PcurveEndpointKind::Regular
        }
    })
}

/// Make `(t0, t1)` increasing on a periodic curve, unwrapping past the
/// seam; coincident endpoints mean a full-period closed edge.
fn unwrap_interval(t0: f64, t1: f64, period: f64) -> (f64, f64) {
    if t1 > t0 { (t0, t1) } else { (t0, t1 + period) }
}

/// The unwrap addition `t1 + period` can overshoot a full-period width by
/// an ulp for some seam angles, and edge bounds wider than the curve's
/// period are structurally invalid. Walk `t1` down to restore
/// `t1 - t0 <= period`; a full-turn edge loses at most a few ulps of
/// parameter, far below evaluation tolerance.
fn clamp_period_width(t0: f64, mut t1: f64, period: f64) -> f64 {
    // next_down is bit-level (no platform libm involved).
    while t1 - t0 > period {
        t1 = t1.next_down();
    }
    t1
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::curve2d::Line2d;
    use kgeom::surface::Surface;
    use kgeom::vec::Vec2;

    #[test]
    fn unwrap_interval_handles_seam_and_closure() {
        let tau = core::f64::consts::TAU;
        assert_eq!(unwrap_interval(1.0, 2.0, tau), (1.0, 2.0));
        let (a, b) = unwrap_interval(5.0, 1.0, tau);
        assert_eq!(a, 5.0);
        assert!((b - (1.0 + tau)).abs() < 1e-15);
        // Coincident endpoints: full period.
        let (a, b) = unwrap_interval(0.5, 0.5, tau);
        assert_eq!(a, 0.5);
        assert!((b - (0.5 + tau)).abs() < 1e-15);
    }

    #[test]
    fn imported_pcurve_endpoints_infer_surface_singularities() {
        let surface = Sphere::new(Frame::world(), 1.0).unwrap();
        let curve = Line2d::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap();
        let kinds = infer_pcurve_endpoint_kinds(
            &surface.degeneracies(),
            &curve,
            ParamMap1d::identity(),
            [0.0, core::f64::consts::FRAC_PI_2],
        );
        assert_eq!(
            kinds,
            [
                PcurveEndpointKind::Regular,
                PcurveEndpointKind::SurfaceSingularity
            ]
        );
    }

    #[test]
    fn split_poles_unweights_rational_data() {
        let raw = [2.0, 4.0, 6.0, 2.0, 1.0, 0.0, 0.0, 1.0];
        let (pts, w) = split_poles(&raw, 4, true).unwrap();
        assert_eq!(pts[0], Point3::new(1.0, 2.0, 3.0));
        assert_eq!(w.unwrap(), vec![2.0, 1.0]);
        assert!(split_poles(&raw, 2, false).is_err());
    }

    #[test]
    fn split_poles_2d_unweights_rational_data() {
        let raw = [2.0, 4.0, 2.0, 1.0, 0.0, 1.0];
        let (points, weights) = split_poles_2d(&raw, 3, true).unwrap();
        assert_eq!(points, vec![Point2::new(1.0, 2.0), Point2::new(1.0, 0.0)]);
        assert_eq!(weights.unwrap(), vec![2.0, 1.0]);
        assert!(split_poles_2d(&raw, 2, true).is_err());
    }
}
