//! Exact source-face arrangements for a complete transverse cylinder pair.
//!
//! Section owns every curve, endpoint, source-ring root, and periodic lift.
//! This bridge checks that those fragments cover both cylinder-side annuli and
//! every cut cap exactly, classifies the resulting open cells against the
//! opposite solid, and exposes ordinary mixed-shell planning bindings.  It
//! neither selects Boolean truth nor allocates topology, and does not claim
//! that the skew bindings are materializable without physical-root carrier
//! trims and persistent skew pcurves.

// The shared boundary error retains exact arrangement diagnostics inline.
#![allow(clippy::result_large_err)]

use std::collections::BTreeSet;

use kcore::operation::OperationScope;

use super::boundary_select::ClassifiedBoundaryFragment;
use super::curved_source::CertifiedCylinderSource;
use super::disk_face_arrangement::{
    ArrangedDiskFace, DiskCellClassification, arrange_section_disk_face,
};
use super::mixed_boundary::{
    MixedBoundaryError, as_boundary_classification, classify_anchor, classify_disk_face,
    classify_periodic_face, operand_side,
};
use super::mixed_cap_boundary::{MixedCylinderCapRing, bind_cylinder_cap_ring};
use super::mixed_periodic_arrangement::{
    MixedPeriodicArrangementError, MixedPeriodicFaceArrangement,
    arrange_mixed_periodic_face_from_embedding,
};
use super::mixed_shell_plan::{MixedArrangementBinding, MixedShellCellKey, source_face_key};
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use crate::error::Error;
use crate::{
    BodyId, BodySectionGraph, CertifiedSectionPeriodicFaceEmbedding, FaceId, Part,
    SectionCompletion, SectionPeriodicFaceEmbeddingEvidence,
};

struct PreparedPeriodicFace {
    face: FaceId,
    operand: usize,
    arrangement: MixedPeriodicFaceArrangement,
    embedding: CertifiedSectionPeriodicFaceEmbedding,
}

struct PreparedDiskFace {
    face: FaceId,
    operand: usize,
    arrangement: ArrangedDiskFace,
}

/// Owned arrangements plus a classification for every open source-face cell.
pub(super) struct PreparedCylinderPairBoundary {
    periodic: Vec<PreparedPeriodicFace>,
    disks: Vec<PreparedDiskFace>,
    caps: Vec<MixedCylinderCapRing>,
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

impl PreparedCylinderPairBoundary {
    pub(super) fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.periodic
            .iter()
            .map(|face| MixedArrangementBinding::Periodic {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: &face.arrangement,
                embedding: Some(&face.embedding),
            })
            .chain(self.disks.iter().map(|face| MixedArrangementBinding::Disk {
                face: face.face.clone(),
                operand: face.operand,
                arranged: &face.arrangement,
            }))
            .chain(
                self.caps
                    .iter()
                    .map(|ring| MixedArrangementBinding::CylinderCap { ring }),
            )
            .collect()
    }

    pub(super) fn classified(&self) -> Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>> {
        self.classified.clone()
    }
}

/// Arrange both side annuli and all four finite-cylinder caps.
///
/// Complete Section evidence is the only geometry case distinction.  A cap
/// carrying Section fragments becomes an exact disk arrangement; an uncut cap
/// keeps its topology-owned whole ring and receives one point/body occupancy
/// certificate.  No fragment count or authored placement is special-cased.
pub(super) fn prepare_cylinder_pair_boundary(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedCylinderPairBoundary, MixedBoundaryError> {
    let part_id = part.id();
    for body in bodies {
        if body.part() != &part_id {
            return Err(MixedBoundaryError::Execution(Error::WrongPart {
                expected: part_id,
                actual: body.part().clone(),
            }));
        }
    }
    if graph.completion() != SectionCompletion::Complete
        || !graph.gaps().is_empty()
        || graph.bodies() != bodies
    {
        return Err(MixedBoundaryError::IncompleteSection);
    }
    let source_faces = cylinders.iter().try_fold(0_usize, |count, cylinder| {
        count.checked_add(1 + cylinder.boundaries().len())
    });
    let work = source_faces
        .and_then(|source_faces| {
            cylinder_pair_boundary_work(
                source_faces,
                graph.curve_fragments().len(),
                graph.curve_endpoints().len(),
                graph.curve_components().len(),
            )
        })
        .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;
    validate_cylinder_fragment_domain(graph, bodies, cylinders)?;

    let mut periodic = Vec::with_capacity(cylinders.len());
    let mut classified = Vec::new();
    for operand in 0..cylinders.len() {
        let face = FaceId::new(
            bodies[operand].part().clone(),
            cylinders[operand].side_face(),
        );
        let embedding = unique_periodic_embedding(graph, &face, operand)?.clone();
        let arrangement = arrange_mixed_periodic_face_from_embedding(graph, &embedding)
            .map_err(MixedBoundaryError::PeriodicArrangement)?;
        validate_periodic_coverage(graph, &face, operand, &arrangement)?;
        let source = source_face_key(&part.state.store, graph, &face, operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let classes = classify_periodic_face(
            part,
            graph,
            &bodies[1 - operand],
            &face,
            operand,
            &arrangement,
            linear,
            scope,
        )?;
        for cell in arrangement.cells() {
            let inside = classes
                .get(cell.key())
                .copied()
                .ok_or(MixedBoundaryError::SourceTopology)?;
            classified.push(ClassifiedBoundaryFragment::new(
                MixedShellCellKey::periodic(source, *cell.key()),
                operand_side(operand),
                (),
                as_boundary_classification(inside),
            ));
        }
        periodic.push(PreparedPeriodicFace {
            face,
            operand,
            arrangement,
            embedding,
        });
    }

    let mut disks = Vec::new();
    let mut caps = Vec::new();
    for operand in 0..cylinders.len() {
        let periodic_face = &periodic[operand];
        for (boundary, source_boundary) in cylinders[operand].boundaries().iter().enumerate() {
            let cap_face = FaceId::new(bodies[operand].part().clone(), source_boundary.cap_face());
            let expected = fragments_on_face(graph, &cap_face, operand)?;
            if expected.is_empty() {
                let inside = classify_anchor(
                    part,
                    &bodies[1 - operand],
                    source_boundary.center(),
                    linear,
                    scope,
                )?;
                let ring = bind_cylinder_cap_ring(
                    &part.state.store,
                    graph,
                    cylinders[operand],
                    operand,
                    boundary,
                    &periodic_face.face,
                    &periodic_face.arrangement,
                )
                .map_err(|_| MixedBoundaryError::SourceTopology)?;
                classified.push(ClassifiedBoundaryFragment::new(
                    MixedShellCellKey::cylinder_cap(ring.cap_source(), ring.boundary()),
                    operand_side(operand),
                    (),
                    as_boundary_classification(inside),
                ));
                caps.push(ring);
                continue;
            }

            let arrangement =
                arrange_section_disk_face(&part.state.store, graph, &cap_face, operand)
                    .map_err(|_| MixedBoundaryError::SourceTopology)?;
            validate_disk_coverage(&expected, &arrangement)?;
            let source = source_face_key(&part.state.store, graph, &cap_face, operand)
                .map_err(|_| MixedBoundaryError::SourceTopology)?;
            let classes =
                classify_disk_face(part, &bodies[1 - operand], &arrangement, linear, scope)?;
            for cell in arrangement.arrangement().cells() {
                let class = classes
                    .get(&cell.key())
                    .copied()
                    .ok_or(MixedBoundaryError::SourceTopology)?;
                let inside = matches!(class, DiskCellClassification::Interior);
                classified.push(ClassifiedBoundaryFragment::new(
                    MixedShellCellKey::disk(source, cell.key()),
                    operand_side(operand),
                    (),
                    as_boundary_classification(inside),
                ));
            }
            disks.push(PreparedDiskFace {
                face: cap_face,
                operand,
                arrangement,
            });
        }
    }
    let expected_caps = cylinders
        .iter()
        .try_fold(0_usize, |count, cylinder| {
            count.checked_add(cylinder.boundaries().len())
        })
        .ok_or(MixedBoundaryError::SourceTopology)?;
    if periodic.len() != cylinders.len() || disks.len() + caps.len() != expected_caps {
        return Err(MixedBoundaryError::SourceTopology);
    }
    validate_classified_binding_coverage(part, graph, &periodic, &disks, &caps, &classified)?;
    Ok(PreparedCylinderPairBoundary {
        periodic,
        disks,
        caps,
        classified,
    })
}

fn unique_periodic_embedding<'a>(
    graph: &'a BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<&'a CertifiedSectionPeriodicFaceEmbedding, MixedBoundaryError> {
    let mut matches = graph
        .periodic_face_embeddings()
        .iter()
        .filter(|evidence| evidence.operand() == operand && evidence.face() == *face);
    let evidence = matches.next().ok_or_else(|| {
        MixedBoundaryError::PeriodicArrangement(
            MixedPeriodicArrangementError::MissingEmbeddingEvidence {
                operand,
                face: face.clone(),
            },
        )
    })?;
    if matches.next().is_some() {
        return Err(MixedBoundaryError::PeriodicArrangement(
            MixedPeriodicArrangementError::DuplicateEmbeddingEvidence {
                operand,
                face: face.clone(),
            },
        ));
    }
    match evidence {
        SectionPeriodicFaceEmbeddingEvidence::Certified(embedding) => Ok(embedding),
        SectionPeriodicFaceEmbeddingEvidence::Indeterminate { gap, .. } => {
            Err(MixedBoundaryError::PeriodicArrangement(
                MixedPeriodicArrangementError::EmbeddingIndeterminate(gap.clone()),
            ))
        }
    }
}

fn validate_cylinder_fragment_domain(
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
) -> Result<(), MixedBoundaryError> {
    for operand in 0..2 {
        let source_faces = [
            FaceId::new(
                bodies[operand].part().clone(),
                cylinders[operand].side_face(),
            ),
            FaceId::new(
                bodies[operand].part().clone(),
                cylinders[operand].boundaries()[0].cap_face(),
            ),
            FaceId::new(
                bodies[operand].part().clone(),
                cylinders[operand].boundaries()[1].cap_face(),
            ),
        ];
        if source_faces[0] == source_faces[1]
            || source_faces[0] == source_faces[2]
            || source_faces[1] == source_faces[2]
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        for fragment in graph.curve_fragments() {
            let branch = graph
                .branches()
                .get(fragment.branch())
                .ok_or(MixedBoundaryError::SourceTopology)?;
            let face = branch
                .faces()
                .get(operand)
                .ok_or(MixedBoundaryError::SourceTopology)?;
            if !source_faces.contains(face) {
                return Err(MixedBoundaryError::SourceTopology);
            }
        }
        let mut coverage = vec![0_usize; graph.curve_fragments().len()];
        for face in &source_faces {
            for fragment in fragments_on_face(graph, face, operand)? {
                let count = coverage
                    .get_mut(fragment)
                    .ok_or(MixedBoundaryError::SourceTopology)?;
                *count = count
                    .checked_add(1)
                    .ok_or(MixedBoundaryError::SourceTopology)?;
            }
        }
        if coverage.iter().any(|&count| count != 1) {
            return Err(MixedBoundaryError::SourceTopology);
        }
    }
    Ok(())
}

fn validate_classified_binding_coverage(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    periodic: &[PreparedPeriodicFace],
    disks: &[PreparedDiskFace],
    caps: &[MixedCylinderCapRing],
    classified: &[ClassifiedBoundaryFragment<MixedShellCellKey, ()>],
) -> Result<(), MixedBoundaryError> {
    let mut expected = BTreeSet::new();
    for face in periodic {
        let source = source_face_key(&part.state.store, graph, &face.face, face.operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        for cell in face.arrangement.cells() {
            if !expected.insert(MixedShellCellKey::periodic(source, *cell.key())) {
                return Err(MixedBoundaryError::SourceTopology);
            }
        }
    }
    for face in disks {
        let source = source_face_key(&part.state.store, graph, &face.face, face.operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        for cell in face.arrangement.arrangement().cells() {
            if !expected.insert(MixedShellCellKey::disk(source, cell.key())) {
                return Err(MixedBoundaryError::SourceTopology);
            }
        }
    }
    for ring in caps {
        if !expected.insert(MixedShellCellKey::cylinder_cap(
            ring.cap_source(),
            ring.boundary(),
        )) {
            return Err(MixedBoundaryError::SourceTopology);
        }
    }

    let mut actual = BTreeSet::new();
    for fragment in classified {
        if fragment.operand() != operand_side(fragment.key().source().operand())
            || !actual.insert(*fragment.key())
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
    }
    (actual == expected)
        .then_some(())
        .ok_or(MixedBoundaryError::SourceTopology)
}

fn fragments_on_face(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
) -> Result<Vec<usize>, MixedBoundaryError> {
    let mut fragments = Vec::new();
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch = graph
            .branches()
            .get(fragment.branch())
            .ok_or(MixedBoundaryError::SourceTopology)?;
        if branch.faces().get(operand) == Some(face) {
            fragments.push(fragment_index);
        }
    }
    Ok(fragments)
}

fn validate_periodic_coverage(
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPeriodicFaceArrangement,
) -> Result<(), MixedBoundaryError> {
    let mut expected = fragments_on_face(graph, face, operand)?;
    let mut actual = arrangement
        .cut_fragments()
        .iter()
        .map(|fragment| fragment.key().fragment())
        .collect::<Vec<_>>();
    expected.sort_unstable();
    actual.sort_unstable();
    if actual != expected || actual.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(MixedBoundaryError::SourceTopology);
    }
    Ok(())
}

fn validate_disk_coverage(
    expected: &[usize],
    arrangement: &ArrangedDiskFace,
) -> Result<(), MixedBoundaryError> {
    let mut expected = expected.to_vec();
    let mut actual = arrangement
        .arrangement()
        .cut_fragments()
        .iter()
        .map(|fragment| fragment.key().fragment())
        .collect::<Vec<_>>();
    expected.sort_unstable();
    actual.sort_unstable();
    if actual != expected || actual.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(MixedBoundaryError::SourceTopology);
    }
    Ok(())
}

/// Geometry-independent ceiling charged before the first arrangement exit.
fn cylinder_pair_boundary_work(
    source_faces: usize,
    fragments: usize,
    endpoints: usize,
    components: usize,
) -> Option<u64> {
    u64::try_from(source_faces)
        .ok()?
        .checked_add(u64::try_from(fragments).ok()?.checked_mul(4)?)?
        .checked_add(u64::try_from(endpoints).ok()?.checked_mul(2)?)?
        .checked_add(u64::try_from(components).ok()?)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use kcore::{
        operation::{
            AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationReport, ResourceKind,
        },
        tolerance::Tolerances,
    };
    use kgeom::{
        frame::Frame,
        vec::{Point3, Vec3},
    };

    use super::*;
    use crate::{
        CylinderRequest, Kernel, PartId, SectionBodiesRequest, SectionPeriodicEmbeddingGap,
        Session,
        boolean::{
            BooleanBudgetProfile,
            curved_source::{CylinderSourceOutcome, extract_cylinder_source},
        },
    };

    const BOUNDED_SKEW_LOWER: f64 = 1.8;
    const BOUNDED_SKEW_UPPER: f64 = 1.9;
    const BOUNDED_SKEW_TRANSVERSE_HALF_HEIGHT: f64 = 1.25;
    const BOUNDED_SKEW_TRANSVERSE_RADIUS: f64 = 2.0;
    const BOUNDED_SKEW_BOUNDARY_WORK: u64 = 56;

    #[derive(Debug, Clone, Copy)]
    enum Placement {
        World,
        Oblique,
    }

    struct Fixture {
        session: Session,
        part: PartId,
        bodies: [BodyId; 2],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    struct PreparationShape {
        periodic: Vec<(usize, usize, usize, usize)>,
        disks: Vec<(usize, usize, usize, usize, usize)>,
        cap_boundaries: Vec<usize>,
        classifications: usize,
    }

    fn placement_frame(placement: Placement) -> Frame {
        match placement {
            Placement::World => Frame::world(),
            Placement::Oblique => Frame::new(
                Point3::new(2.5, -1.75, 0.625),
                Vec3::new(0.48, 0.64, 0.6),
                Vec3::new(0.8, -0.6, 0.0),
            )
            .unwrap(),
        }
    }

    fn fixture(placement: Placement) -> Fixture {
        let frame = placement_frame(placement);
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let bodies = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let bounded = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(frame.point_at(0.0, 0.0, BOUNDED_SKEW_LOWER)),
                    1.0,
                    BOUNDED_SKEW_UPPER - BOUNDED_SKEW_LOWER,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let transverse_frame = Frame::new(
                frame.point_at(-BOUNDED_SKEW_TRANSVERSE_HALF_HEIGHT, 0.0, 0.0),
                frame.x(),
                frame.y(),
            )
            .unwrap();
            let transverse = edit
                .create_cylinder(CylinderRequest::new(
                    transverse_frame,
                    BOUNDED_SKEW_TRANSVERSE_RADIUS,
                    2.0 * BOUNDED_SKEW_TRANSVERSE_HALF_HEIGHT,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            [bounded, transverse]
        };
        Fixture {
            session,
            part,
            bodies,
        }
    }

    fn ordered_bodies(fixture: &Fixture, swapped: bool) -> [BodyId; 2] {
        if swapped {
            [fixture.bodies[1].clone(), fixture.bodies[0].clone()]
        } else {
            fixture.bodies.clone()
        }
    }

    fn source_signature(fixture: &Fixture) -> ([[usize; 3]; 2], usize) {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let topology = fixture.bodies.each_ref().map(|body| {
            let body = part.body(body.clone()).unwrap();
            [
                body.faces().unwrap().len(),
                body.edges().unwrap().len(),
                body.vertices().unwrap().len(),
            ]
        });
        (topology, part.bodies().len())
    }

    fn section(fixture: &Fixture, bodies: &[BodyId; 2]) -> BodySectionGraph {
        fixture
            .session
            .part(fixture.part.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(
                bodies[0].clone(),
                bodies[1].clone(),
            ))
            .unwrap()
            .into_result()
            .unwrap()
    }

    fn cylinder_sources(part: &Part<'_>, bodies: &[BodyId; 2]) -> [CertifiedCylinderSource; 2] {
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BooleanBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        bodies.each_ref().map(|body| {
            match extract_cylinder_source(&part.state.store, body.raw(), &mut scope).unwrap() {
                CylinderSourceOutcome::Ready(source) => source,
                other => panic!("bounded-skew fixture lost cylinder source: {other:?}"),
            }
        })
    }

    fn prepare(
        part: &Part<'_>,
        graph: &BodySectionGraph,
        bodies: &[BodyId; 2],
        sources: &[CertifiedCylinderSource; 2],
        allowed: Option<u64>,
    ) -> (
        Result<PreparedCylinderPairBoundary, MixedBoundaryError>,
        OperationReport,
    ) {
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BooleanBudgetProfile::v1_defaults());
        let context = match allowed {
            Some(allowed) => context.with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    PLANAR_BOOLEAN_BSP_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            ),
            None => context,
        };
        let mut scope = OperationScope::new(&context);
        let result = prepare_cylinder_pair_boundary(
            part,
            graph,
            bodies,
            [&sources[0], &sources[1]],
            context.tolerances().linear(),
            &mut scope,
        );
        scope.finish_typed(result).into_parts()
    }

    fn bsp_snapshot(report: &OperationReport) -> kcore::operation::LimitSnapshot {
        report
            .usage()
            .iter()
            .copied()
            .find(|snapshot| {
                snapshot.stage == PLANAR_BOOLEAN_BSP_WORK && snapshot.resource == ResourceKind::Work
            })
            .expect("Boolean defaults must retain boundary work accounting")
    }

    fn preparation_shape(prepared: &PreparedCylinderPairBoundary) -> PreparationShape {
        let mut periodic = prepared
            .periodic
            .iter()
            .map(|face| {
                (
                    face.arrangement.source_spans().len(),
                    face.arrangement.cut_fragments().len(),
                    face.arrangement.cells().len(),
                    face.arrangement.adjacency().len(),
                )
            })
            .collect::<Vec<_>>();
        periodic.sort_unstable();
        let mut disks = prepared
            .disks
            .iter()
            .map(|face| {
                let proof = face.arrangement.proof();
                (
                    proof.roots_conserved(),
                    proof.source_arcs_conserved(),
                    proof.opposed_chords(),
                    proof.cells(),
                    proof.dual_edges(),
                )
            })
            .collect::<Vec<_>>();
        disks.sort_unstable();
        let mut cap_boundaries = prepared
            .caps
            .iter()
            .map(MixedCylinderCapRing::boundary)
            .collect::<Vec<_>>();
        cap_boundaries.sort_unstable();
        PreparationShape {
            periodic,
            disks,
            cap_boundaries,
            classifications: prepared.classified.len(),
        }
    }

    fn assert_replay(
        prepared: &PreparedCylinderPairBoundary,
        replay: &PreparedCylinderPairBoundary,
    ) {
        assert_eq!(prepared.periodic.len(), replay.periodic.len());
        for (face, replay_face) in prepared.periodic.iter().zip(&replay.periodic) {
            assert_eq!(face.face, replay_face.face);
            assert_eq!(face.operand, replay_face.operand);
            assert_eq!(face.arrangement, replay_face.arrangement);
            assert_eq!(face.embedding, replay_face.embedding);
        }
        assert_eq!(prepared.disks.len(), replay.disks.len());
        for (face, replay_face) in prepared.disks.iter().zip(&replay.disks) {
            assert_eq!(face.face, replay_face.face);
            assert_eq!(face.operand, replay_face.operand);
            assert_eq!(face.arrangement, replay_face.arrangement);
        }
        assert_eq!(prepared.caps, replay.caps);
        assert_eq!(prepared.classified, replay.classified);
    }

    fn assert_complete_classification(
        part: &Part<'_>,
        graph: &BodySectionGraph,
        prepared: &PreparedCylinderPairBoundary,
    ) {
        let mut expected = BTreeSet::new();
        for face in &prepared.periodic {
            let source =
                source_face_key(&part.state.store, graph, &face.face, face.operand).unwrap();
            for cell in face.arrangement.cells() {
                assert!(expected.insert(MixedShellCellKey::periodic(source, *cell.key())));
            }
        }
        for face in &prepared.disks {
            let source =
                source_face_key(&part.state.store, graph, &face.face, face.operand).unwrap();
            for cell in face.arrangement.arrangement().cells() {
                assert!(expected.insert(MixedShellCellKey::disk(source, cell.key())));
            }
        }
        for ring in &prepared.caps {
            assert!(expected.insert(MixedShellCellKey::cylinder_cap(
                ring.cap_source(),
                ring.boundary(),
            )));
        }

        let mut actual = BTreeSet::new();
        for fragment in &prepared.classified {
            assert!(
                actual.insert(*fragment.key()),
                "classification keys must be unique"
            );
            assert_eq!(
                fragment.operand(),
                operand_side(fragment.key().source().operand())
            );
        }
        let expected_count = prepared
            .periodic
            .iter()
            .map(|face| face.arrangement.cells().len())
            .sum::<usize>()
            + prepared
                .disks
                .iter()
                .map(|face| face.arrangement.arrangement().cells().len())
                .sum::<usize>()
            + prepared.caps.len();
        assert_eq!(prepared.classified.len(), expected_count);
        assert_eq!(actual, expected);
    }

    fn assert_bounded_skew_preparation(
        part: &Part<'_>,
        graph: &BodySectionGraph,
        prepared: &PreparedCylinderPairBoundary,
        bounded_operand: usize,
    ) {
        assert_eq!(prepared.periodic.len(), 2);
        let mut periodic_operands = prepared
            .periodic
            .iter()
            .map(|face| face.operand)
            .collect::<Vec<_>>();
        periodic_operands.sort_unstable();
        assert_eq!(periodic_operands, vec![0, 1]);
        for face in &prepared.periodic {
            assert!(!face.arrangement.cut_fragments().is_empty());
            assert!(!face.arrangement.cells().is_empty());
            assert!(face.arrangement.proof().dual_connected());
        }

        assert_eq!(prepared.disks.len(), 2);
        assert!(
            prepared
                .disks
                .iter()
                .all(|face| face.operand == bounded_operand)
        );
        assert_ne!(prepared.disks[0].face, prepared.disks[1].face);
        for face in &prepared.disks {
            let disk = &face.arrangement;
            assert_eq!(disk.proof().roots_conserved(), 4);
            assert_eq!(disk.proof().source_arcs_conserved(), 4);
            assert_eq!(disk.proof().opposed_chords(), 2);
            assert_eq!(disk.proof().cells(), 3);
            assert_eq!(disk.proof().dual_edges(), 2);
            assert!(disk.proof().dual_connected());
            assert_eq!(disk.arrangement().cut_fragments().len(), 2);
            assert_eq!(disk.arrangement().cells().len(), 3);
        }

        assert_eq!(prepared.caps.len(), 2);
        assert!(
            prepared
                .caps
                .iter()
                .all(|ring| ring.operand() == 1 - bounded_operand)
        );
        assert_ne!(prepared.caps[0].cap_face(), prepared.caps[1].cap_face());
        let mut boundaries = prepared
            .caps
            .iter()
            .map(MixedCylinderCapRing::boundary)
            .collect::<Vec<_>>();
        boundaries.sort_unstable();
        assert_eq!(boundaries, vec![0, 1]);
        for ring in &prepared.caps {
            assert!(ring.side_loop_key().is_whole_loop());
            let side = prepared
                .periodic
                .iter()
                .find(|face| face.operand == ring.operand())
                .unwrap();
            assert!(
                side.arrangement
                    .source_spans()
                    .iter()
                    .any(|span| { span.key() == &ring.side_loop_key() && span.is_whole_loop() })
            );
        }

        let bindings = prepared.bindings();
        assert_eq!(bindings.len(), 6);
        let mut binding_counts = [0_usize; 3];
        for binding in bindings {
            match binding {
                MixedArrangementBinding::Periodic { .. } => binding_counts[0] += 1,
                MixedArrangementBinding::Disk { .. } => binding_counts[1] += 1,
                MixedArrangementBinding::CylinderCap { .. } => binding_counts[2] += 1,
                MixedArrangementBinding::Planar { .. } => {
                    panic!("cylinder pair must not publish a planar binding")
                }
            }
        }
        assert_eq!(binding_counts, [2, 2, 2]);

        let mut coverage = vec![vec![0_usize; graph.curve_fragments().len()]; 2];
        for face in &prepared.periodic {
            for cut in face.arrangement.cut_fragments() {
                coverage[face.operand][cut.key().fragment()] += 1;
            }
        }
        for face in &prepared.disks {
            for cut in face.arrangement.arrangement().cut_fragments() {
                coverage[face.operand][cut.key().fragment()] += 1;
            }
        }
        for fragment in 0..graph.curve_fragments().len() {
            assert_eq!(
                [coverage[0][fragment], coverage[1][fragment]],
                [1, 1],
                "Section fragment {fragment} must cover one source face per operand"
            );
        }
        assert_eq!(
            coverage.iter().flatten().copied().sum::<usize>(),
            2 * graph.curve_fragments().len()
        );
        assert_complete_classification(part, graph, prepared);
    }

    #[test]
    fn cylinder_pair_boundary_work_is_an_exact_collection_ceiling() {
        assert_eq!(
            cylinder_pair_boundary_work(6, 8, 8, 2),
            Some(BOUNDED_SKEW_BOUNDARY_WORK)
        );
        assert_eq!(cylinder_pair_boundary_work(0, 0, 0, 0), Some(0));
        assert_eq!(
            cylinder_pair_boundary_work(usize::MAX, usize::MAX, 0, 0),
            None
        );
    }

    #[test]
    fn bounded_skew_preparation_is_complete_and_rigid_replay_stable() {
        let mut reference_shape = None;
        for placement in [Placement::World, Placement::Oblique] {
            let fixture = fixture(placement);
            let before = source_signature(&fixture);
            for swapped in [false, true] {
                let bodies = ordered_bodies(&fixture, swapped);
                let graph = section(&fixture, &bodies);
                let replay_graph = section(&fixture, &bodies);
                assert_eq!(
                    graph, replay_graph,
                    "{placement:?} swapped={swapped}: Section replay changed"
                );
                assert_eq!(graph.completion(), SectionCompletion::Complete);
                assert!(graph.gaps().is_empty());
                assert_eq!(graph.curve_fragments().len(), 8);
                assert_eq!(graph.curve_endpoints().len(), 8);
                assert_eq!(graph.curve_components().len(), 2);
                assert_eq!(graph.periodic_face_embeddings().len(), 2);

                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let sources = cylinder_sources(&part, &bodies);
                let (prepared, report) = prepare(&part, &graph, &bodies, &sources, None);
                let prepared = prepared.unwrap_or_else(|error| {
                    panic!("{placement:?} swapped={swapped}: preparation failed: {error:?}")
                });
                let (replay, replay_report) =
                    prepare(&part, &replay_graph, &bodies, &sources, None);
                let replay = replay.unwrap_or_else(|error| {
                    panic!("{placement:?} swapped={swapped}: replay preparation failed: {error:?}")
                });
                assert_replay(&prepared, &replay);
                assert_eq!(report, replay_report);

                let bounded_operand = usize::from(swapped);
                assert_bounded_skew_preparation(&part, &graph, &prepared, bounded_operand);
                let snapshot = bsp_snapshot(&report);
                assert_eq!(snapshot.consumed, BOUNDED_SKEW_BOUNDARY_WORK);

                let shape = preparation_shape(&prepared);
                if let Some(reference) = &reference_shape {
                    assert_eq!(
                        &shape, reference,
                        "{placement:?} swapped={swapped}: rigid/swap topology changed"
                    );
                } else {
                    reference_shape = Some(shape);
                }
            }
            assert_eq!(
                source_signature(&fixture),
                before,
                "{placement:?}: preparation mutated either source body"
            );
        }
    }

    #[test]
    fn bounded_skew_boundary_work_accepts_exact_n_and_refuses_n_minus_one_atomically() {
        for placement in [Placement::World, Placement::Oblique] {
            let fixture = fixture(placement);
            let before = source_signature(&fixture);
            for swapped in [false, true] {
                let bodies = ordered_bodies(&fixture, swapped);
                let graph = section(&fixture, &bodies);
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let sources = cylinder_sources(&part, &bodies);

                let (prepared, report) = prepare(
                    &part,
                    &graph,
                    &bodies,
                    &sources,
                    Some(BOUNDED_SKEW_BOUNDARY_WORK),
                );
                let prepared = prepared.unwrap_or_else(|error| {
                    panic!("{placement:?} swapped={swapped}: exact-N preparation failed: {error:?}")
                });
                assert_bounded_skew_preparation(&part, &graph, &prepared, usize::from(swapped));
                let snapshot = bsp_snapshot(&report);
                assert_eq!(snapshot.consumed, BOUNDED_SKEW_BOUNDARY_WORK);
                assert_eq!(snapshot.allowed, BOUNDED_SKEW_BOUNDARY_WORK);
                assert!(report.limit_events().is_empty());

                let (denied, report) = prepare(
                    &part,
                    &graph,
                    &bodies,
                    &sources,
                    Some(BOUNDED_SKEW_BOUNDARY_WORK - 1),
                );
                let error = match denied {
                    Ok(_) => panic!("N-1 must refuse before arrangement"),
                    Err(error) => error,
                };
                let limit = match error {
                    MixedBoundaryError::Execution(error) => error
                        .limit()
                        .expect("boundary work refusal must retain limit evidence"),
                    other => panic!("unexpected boundary work refusal: {other:?}"),
                };
                assert_eq!(limit.stage, PLANAR_BOOLEAN_BSP_WORK);
                assert_eq!(limit.resource, ResourceKind::Work);
                assert_eq!(limit.consumed, BOUNDED_SKEW_BOUNDARY_WORK);
                assert_eq!(limit.allowed, BOUNDED_SKEW_BOUNDARY_WORK - 1);
                let snapshot = bsp_snapshot(&report);
                assert_eq!(snapshot.consumed, 0, "failed precharge must be atomic");
                assert_eq!(snapshot.allowed, BOUNDED_SKEW_BOUNDARY_WORK - 1);
                assert_eq!(report.limit_events(), &[limit]);
            }
            assert_eq!(
                source_signature(&fixture),
                before,
                "{placement:?}: exact-N/N-1 preparation mutated either source body"
            );
        }
    }

    #[test]
    fn periodic_evidence_selection_refuses_indeterminate_and_duplicate_matches() {
        let fixture = fixture(Placement::World);
        let bodies = ordered_bodies(&fixture, false);
        let graph = section(&fixture, &bodies);
        let evidence = graph.periodic_face_embeddings()[0].clone();
        let operand = evidence.operand();
        let face = evidence.face();

        let mut indeterminate = graph.clone();
        let slot = indeterminate
            .periodic_face_embeddings
            .iter_mut()
            .find(|candidate| candidate.operand() == operand && candidate.face() == face)
            .unwrap();
        *slot = SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
            operand,
            face: face.clone(),
            gap: SectionPeriodicEmbeddingGap::SourceFaceTopology,
        };
        assert!(matches!(
            unique_periodic_embedding(&indeterminate, &face, operand),
            Err(MixedBoundaryError::PeriodicArrangement(
                MixedPeriodicArrangementError::EmbeddingIndeterminate(
                    SectionPeriodicEmbeddingGap::SourceFaceTopology
                )
            ))
        ));

        let mut duplicate = graph;
        duplicate.periodic_face_embeddings.push(evidence);
        assert!(matches!(
            unique_periodic_embedding(&duplicate, &face, operand),
            Err(MixedBoundaryError::PeriodicArrangement(
                MixedPeriodicArrangementError::DuplicateEmbeddingEvidence {
                    operand: duplicate_operand,
                    face: duplicate_face,
                }
            )) if duplicate_operand == operand && duplicate_face == face
        ));
    }

    #[test]
    fn wrong_part_refuses_before_boundary_work() {
        let mut fixture = fixture(Placement::World);
        let bodies = ordered_bodies(&fixture, false);
        let graph = section(&fixture, &bodies);
        let foreign_part = fixture.session.create_part();
        let foreign = {
            let mut edit = fixture.session.edit_part(foreign_part.clone()).unwrap();
            edit.create_cylinder(CylinderRequest::new(Frame::world(), 1.0, 1.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body()
        };
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let sources = cylinder_sources(&part, &bodies);
        let wrong_bodies = [bodies[0].clone(), foreign];
        let (result, report) = prepare(&part, &graph, &wrong_bodies, &sources, None);
        match result {
            Err(MixedBoundaryError::Execution(Error::WrongPart { expected, actual })) => {
                assert_eq!(expected, fixture.part);
                assert_eq!(actual, foreign_part);
            }
            Err(other) => panic!("unexpected wrong-part refusal: {other:?}"),
            Ok(_) => panic!("wrong-part preparation must refuse"),
        }
        assert_eq!(bsp_snapshot(&report).consumed, 0);
    }
}
