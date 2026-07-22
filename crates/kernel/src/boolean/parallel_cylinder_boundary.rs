//! Exact face arrangements for a strictly nested parallel-cylinder section.
//!
//! Both side faces are ordinary periodic arrangements and every cut inner cap
//! is an ordinary disk arrangement. This bridge only gathers those general
//! arrangements, classifies their connected duals against the opposite solid,
//! and exposes them to the shared truth selector and mixed-shell planner.

// The shared boundary error keeps exact arrangement diagnostics inline. This
// operation-local bridge preserves that typed contract rather than boxing it.
#![allow(clippy::result_large_err)]

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
use super::mixed_cap_boundary::{
    MixedCylinderCapRing, bind_cylinder_cap_rings, classified_exterior_cap,
};
use super::mixed_periodic_arrangement::{
    MixedPeriodicFaceArrangement, arrange_mixed_periodic_face,
};
use super::mixed_shell_plan::{MixedArrangementBinding, MixedShellCellKey, source_face_key};
use super::parallel_cylinder_relation::CertifiedParallelCylinderLensRelation;
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use crate::error::Error;
use crate::{
    BodyId, BodySectionGraph, FaceId, Part, SectionCompletion, SectionPeriodicFaceEmbeddingEvidence,
};

struct PreparedPeriodicFace {
    face: FaceId,
    operand: usize,
    arrangement: MixedPeriodicFaceArrangement,
}

struct PreparedDiskFace {
    face: FaceId,
    operand: usize,
    arrangement: ArrangedDiskFace,
}

struct PreparedPeriodicFaces {
    faces: Vec<PreparedPeriodicFace>,
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

struct PreparedCapFaces {
    disks: Vec<PreparedDiskFace>,
    rings: Vec<MixedCylinderCapRing>,
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

/// Owned two-cylinder arrangements plus every classified open source cell.
pub(super) struct PreparedParallelCylinderBoundary {
    periodic: Vec<PreparedPeriodicFace>,
    disks: Vec<PreparedDiskFace>,
    caps: Vec<MixedCylinderCapRing>,
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

impl PreparedParallelCylinderBoundary {
    pub(super) fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.periodic
            .iter()
            .map(|face| MixedArrangementBinding::Periodic {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: &face.arrangement,
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

/// Arrange both side annuli, both cut inner caps, and both exterior outer caps.
///
/// The caller owns the strict axial-containment and complete-section theorem.
/// This function verifies its topological consequences before exposing any
/// arrangement to truth selection.
pub(super) fn prepare_parallel_cylinder_boundary(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderLensRelation,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedParallelCylinderBoundary, MixedBoundaryError> {
    if graph.completion() != SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Err(MixedBoundaryError::IncompleteSection);
    }
    let source_faces = cylinders
        .iter()
        .try_fold(0_usize, |count, cylinder| {
            count.checked_add(1 + cylinder.boundaries().len())
        })
        .ok_or(MixedBoundaryError::SourceTopology)?;
    let work = parallel_boundary_work(
        source_faces,
        graph.curve_fragments().len(),
        graph.curve_endpoints().len(),
        graph.curve_components().len(),
    )
    .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;

    let mut periodic =
        prepare_periodic_faces(part, graph, bodies, cylinders, relation, linear, scope)?;
    let caps = prepare_cap_faces(
        part,
        graph,
        bodies,
        cylinders,
        relation,
        &periodic.faces,
        linear,
        scope,
    )?;
    periodic.classified.extend(caps.classified);
    Ok(PreparedParallelCylinderBoundary {
        periodic: periodic.faces,
        disks: caps.disks,
        caps: caps.rings,
        classified: periodic.classified,
    })
}

#[allow(clippy::too_many_arguments)]
fn prepare_periodic_faces(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderLensRelation,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedPeriodicFaces, MixedBoundaryError> {
    if [relation.inner_operand(), relation.outer_operand()] != [0, 1]
        && [relation.inner_operand(), relation.outer_operand()] != [1, 0]
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let store = &part.state.store;
    let mut periodic = Vec::with_capacity(2);
    let mut classified = Vec::new();
    for operand in 0..2 {
        let face = unique_periodic_face(graph, cylinders[operand], operand)?;
        let arrangement = arrange_mixed_periodic_face(graph, face.clone(), operand)
            .map_err(MixedBoundaryError::PeriodicArrangement)?;
        validate_periodic_fragments(&arrangement, relation, operand)?;
        let source = source_face_key(store, graph, &face, operand)
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
        classified.extend(arrangement.cells().iter().map(|cell| {
            ClassifiedBoundaryFragment::new(
                MixedShellCellKey::periodic(source, *cell.key()),
                operand_side(operand),
                (),
                as_boundary_classification(classes[cell.key()]),
            )
        }));
        periodic.push(PreparedPeriodicFace {
            face,
            operand,
            arrangement,
        });
    }
    Ok(PreparedPeriodicFaces {
        faces: periodic,
        classified,
    })
}

#[allow(clippy::too_many_arguments)]
fn prepare_cap_faces(
    part: &Part<'_>,
    graph: &BodySectionGraph,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderLensRelation,
    periodic: &[PreparedPeriodicFace],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedCapFaces, MixedBoundaryError> {
    let store = &part.state.store;
    let inner = relation.inner_operand();
    let outer = relation.outer_operand();
    let mut disks = Vec::with_capacity(2);
    let mut classified = Vec::new();
    for witness in relation.cap_boundaries() {
        let boundary = cylinders[inner]
            .boundaries()
            .get(witness.boundary())
            .ok_or(MixedBoundaryError::SourceTopology)?;
        if boundary.cap_face() != witness.cap_face() || boundary.edge() != witness.edge() {
            return Err(MixedBoundaryError::SourceTopology);
        }
        let cap_face = FaceId::new(bodies[inner].part().clone(), witness.cap_face());
        let arrangement = arrange_section_disk_face(store, graph, &cap_face, inner)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let [cut] = arrangement.arrangement().cut_fragments() else {
            return Err(MixedBoundaryError::SourceTopology);
        };
        let mut actual_root_ordinals = arrangement
            .source_arcs()
            .iter()
            .flat_map(|arc| arc.roots())
            .map(|root| root.key().source_root_ordinal())
            .collect::<Vec<_>>();
        actual_root_ordinals.sort_unstable();
        actual_root_ordinals.dedup();
        if cut.key().fragment() != witness.fragment()
            || actual_root_ordinals.as_slice() != witness.root_ordinals()
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        let source = source_face_key(store, graph, &cap_face, inner)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let classes = classify_disk_face(part, &bodies[outer], &arrangement, linear, scope)?;
        classified.extend(arrangement.arrangement().cells().iter().map(|cell| {
            let class = match classes[&cell.key()] {
                DiskCellClassification::Interior => {
                    super::boundary_select::BoundaryFragmentClassification::Interior
                }
                DiskCellClassification::Exterior => {
                    super::boundary_select::BoundaryFragmentClassification::Exterior
                }
            };
            ClassifiedBoundaryFragment::new(
                MixedShellCellKey::disk(source, cell.key()),
                operand_side(inner),
                (),
                class,
            )
        }));
        disks.push(PreparedDiskFace {
            face: cap_face,
            operand: inner,
            arrangement,
        });
    }

    let periodic_face = periodic
        .iter()
        .find(|face| face.operand == outer)
        .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
    let mut caps = Vec::with_capacity(2);
    for boundary in cylinders[outer].boundaries() {
        if classify_anchor(part, &bodies[inner], boundary.center(), linear, scope)? {
            return Err(MixedBoundaryError::CylinderCapNotExterior);
        }
    }
    let bound = bind_cylinder_cap_rings(
        store,
        graph,
        cylinders[outer],
        outer,
        &periodic_face.face,
        &periodic_face.arrangement,
    )
    .map_err(|_| MixedBoundaryError::SourceTopology)?;
    for ring in bound {
        classified.push(classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(ring.cap_source(), ring.boundary()),
            outer,
        ));
        caps.push(ring);
    }
    if periodic.len() != 2 || disks.len() != 2 || caps.len() != 2 {
        return Err(MixedBoundaryError::SourceTopology);
    }
    Ok(PreparedCapFaces {
        disks,
        rings: caps,
        classified,
    })
}

fn validate_periodic_fragments(
    arrangement: &MixedPeriodicFaceArrangement,
    relation: &CertifiedParallelCylinderLensRelation,
    operand: usize,
) -> Result<(), MixedBoundaryError> {
    let mut actual = arrangement
        .cut_fragments()
        .iter()
        .map(|fragment| {
            if fragment.key().component() != relation.component() {
                return Err(MixedBoundaryError::SourceTopology);
            }
            Ok(fragment.key().fragment())
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut expected = relation
        .rulings()
        .iter()
        .map(|witness| witness.fragment())
        .collect::<Vec<_>>();
    if operand == relation.outer_operand() {
        expected.extend(
            relation
                .cap_boundaries()
                .iter()
                .map(|witness| witness.fragment()),
        );
    }
    actual.sort_unstable();
    expected.sort_unstable();
    if actual != expected {
        return Err(MixedBoundaryError::SourceTopology);
    }
    if operand == relation.inner_operand() {
        validate_inner_periodic_roots(arrangement, relation)?;
    }
    Ok(())
}

fn validate_inner_periodic_roots(
    arrangement: &MixedPeriodicFaceArrangement,
    relation: &CertifiedParallelCylinderLensRelation,
) -> Result<(), MixedBoundaryError> {
    let mut actual = arrangement
        .source_spans()
        .iter()
        .filter_map(|span| span.key().terminal_roots())
        .flatten()
        .map(|root| (root.endpoint(), root.source_root_ordinal()))
        .collect::<Vec<_>>();
    let mut expected = relation
        .rulings()
        .iter()
        .flat_map(|witness| witness.endpoints().into_iter().zip(witness.root_ordinals()))
        .collect::<Vec<_>>();
    actual.sort_unstable();
    actual.dedup();
    expected.sort_unstable();
    expected.dedup();
    if actual != expected {
        return Err(MixedBoundaryError::SourceTopology);
    }
    Ok(())
}

fn unique_periodic_face(
    graph: &BodySectionGraph,
    cylinder: &CertifiedCylinderSource,
    operand: usize,
) -> Result<FaceId, MixedBoundaryError> {
    let mut matches =
        graph
            .periodic_face_embeddings()
            .iter()
            .filter_map(|evidence| match evidence {
                SectionPeriodicFaceEmbeddingEvidence::Certified(certified)
                    if certified.operand() == operand
                        && certified.face().raw() == cylinder.side_face() =>
                {
                    Some(certified.face())
                }
                _ => None,
            });
    let face = matches
        .next()
        .ok_or(MixedBoundaryError::MissingPeriodicFaceEvidence)?;
    if matches.next().is_some() {
        return Err(MixedBoundaryError::SourceTopology);
    }
    Ok(face)
}

/// Geometry-independent ceiling charged before the first arrangement exit.
fn parallel_boundary_work(
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
