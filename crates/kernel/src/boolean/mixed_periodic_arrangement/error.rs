use super::{
    FaceId, PeriodicBoundaryTraceKey, PeriodicCutFragmentKey, PeriodicSurfaceError, RawLoopId,
    SectionPeriodicEmbeddingGap,
};

/// Postcondition whose violation would contradict the admitted theorem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MixedPeriodicArrangementContractGap {
    CellCount,
    RemainderTopology,
    DiskTopology(usize),
    CutAdjacency(PeriodicCutFragmentKey),
    TraceCellTopology(PeriodicBoundaryTraceKey),
    TraceCutAdjacency(PeriodicCutFragmentKey),
    Conservation,
}

/// Typed refusal at the periodic-evidence/arrangement boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum MixedPeriodicArrangementError {
    InvalidOperand(usize),
    IncompleteSectionGraph,
    MissingEmbeddingEvidence {
        operand: usize,
        face: FaceId,
    },
    DuplicateEmbeddingEvidence {
        operand: usize,
        face: FaceId,
    },
    EmbeddingIndeterminate(SectionPeriodicEmbeddingGap),
    SourceLoopPartMismatch(RawLoopId),
    DuplicateSourceLoop(RawLoopId),
    SourceLoopDirectionMismatch,
    MixedClosedAndBoundaryEvidence,
    BoundaryTraceEvidenceRequired(usize),
    BoundaryRootCountMismatch {
        source_loop: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryRootOrderMismatch {
        source_loop: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryRootLoopMismatch {
        endpoint: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryRootCoverageMismatch(usize),
    DuplicateBoundaryTrace(PeriodicBoundaryTraceKey),
    BoundaryTraceEmpty(PeriodicBoundaryTraceKey),
    BoundaryTraceOrdinalMismatch {
        trace: PeriodicBoundaryTraceKey,
        trace_ordinal: usize,
        component_ordinal: usize,
    },
    BoundaryTraceFragmentMismatch {
        trace: PeriodicBoundaryTraceKey,
        component_ordinal: usize,
        expected: usize,
        actual: usize,
    },
    BoundaryTraceEndpointMismatch {
        trace: PeriodicBoundaryTraceKey,
        expected: usize,
        actual: usize,
    },
    MixedBoundaryTraceFamiliesUnsupported {
        returning: PeriodicBoundaryTraceKey,
        transverse: PeriodicBoundaryTraceKey,
    },
    BoundaryTraceMatchingMismatch(PeriodicBoundaryTraceKey),
    UnknownBranch {
        fragment: usize,
        branch: usize,
    },
    UnknownFragment {
        component: usize,
        fragment: usize,
    },
    FaceLocalPathUnavailable(usize),
    ComponentLeavesFace(usize),
    MissingComponentEvidence(usize),
    UnexpectedComponentEvidence(usize),
    DuplicateComponentEvidence(usize),
    OpenComponent(usize),
    EmptyComponent(usize),
    NonContractibleComponent {
        component: usize,
        winding: i64,
    },
    NestedComponent {
        component: usize,
        parent: usize,
    },
    FragmentCountMismatch {
        component: usize,
        expected: usize,
        actual: usize,
    },
    FragmentOrderMismatch {
        component: usize,
        ordinal: usize,
        expected: usize,
        actual: usize,
    },
    FragmentEmbeddingEndpointMismatch {
        fragment: usize,
        end: usize,
        expected: usize,
        actual: usize,
    },
    DuplicateFragment(usize),
    WholeFragment(usize),
    UnknownEndpoint {
        fragment: usize,
        endpoint: usize,
    },
    ComponentEndpointMismatch {
        component: usize,
        ordinal: usize,
        expected: usize,
        actual: usize,
    },
    TopologyArithmeticOverflow,
    Arrangement(PeriodicSurfaceError),
    Contract(MixedPeriodicArrangementContractGap),
}
