//! Operation-local theorem for the first parallel-cylinder Boolean slice.
//!
//! The section graph remains the general intersection authority.  This module
//! recognizes the proof-complete relations needed by the first Cylinder/Cylinder
//! Boolean slices: strict axial separation or exact axial contact of exactly
//! parallel or antiparallel finite sources, exact external radial tangency over
//! a strictly positive axial overlap, and the strict finite lens-prism relation.
//! Both authored boundary orders are normalized onto one certified common axial
//! coordinate before gap, contact, or overlap ownership is decided. Every
//! retained boundary is topology-owned; rounded points are never used as
//! topology keys.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3, orient3d};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ParallelCylinderRadialRelation, classify_parallel_cylinder_radial_relation};
use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use super::curved_source::CertifiedCylinderSource;
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use crate::error::{Error, Result};
use crate::{
    BodySectionGraph, SectionBranch, SectionBranchTopology, SectionCarrier,
    SectionCurveEndpointTopology, SectionCurveFragmentSpan, SectionPeriodicFaceEmbeddingEvidence,
    SectionSite, SectionSourceParameterKey, SectionUvCurve,
};

#[path = "parallel_cylinder_relation/coincident_caps.rs"]
mod coincident_caps;
pub(super) use coincident_caps::{
    CertifiedParallelCylinderCoincidentCapRelation, ParallelCylinderCoincidentCapEndWitness,
    ParallelCylinderSourceRootWitness,
};

/// Fixed proof work charged before the first semantic exit.
///
/// Constant source normalization and gap comparisons cover the strict axial
/// exit. Overlap paths additionally admit exactly four branches, fragments,
/// and endpoints and one component; oversized graph shapes are rejected from
/// their lengths before collection scans. This remains a geometry-independent
/// ceiling for every exit.
pub(super) const PARALLEL_CYLINDER_RELATION_WORK: u64 = 64;

/// First unmet obligation in the strict parallel-cylinder lens relation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub(super) enum ParallelCylinderRelationGap {
    /// A source value contains non-finite or otherwise unusable arithmetic.
    ArithmeticGuard,
    /// The two authored cylinder axes are not exactly parallel.
    AxesNotExactlyParallel,
    /// The radial circles are not certified as a strict two-root secant.
    RadialSecancyNotStrict,
    /// A source's authored cap interval is not strictly positive on its own axis.
    SourceAxialOrder,
    /// A cap plane/ring cannot be proof-bound to its finite-cylinder support.
    SourceBoundaryBinding,
    /// The source cap intervals are disjoint or touch without positive overlap.
    AxialOverlapNotStrictlyPositive,
    /// The supplied section is not globally complete and gap-free.
    SectionIncomplete,
    /// Fixed collection counts, component coverage, or alternation did not match.
    SectionLayout,
    /// Periodic or branch face evidence does not bind to the supplied sources.
    SectionOperandBinding,
    /// A branch carrier, pcurve family, range, or residual proof did not match.
    SectionBranchEvidence,
    /// A trim endpoint did not retain the required overlap-end cap-ring provenance.
    SectionEndpointProvenance,
}

/// One physical axial-overlap end and the unique section arc cut into its cap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ParallelCylinderOverlapEndWitness {
    operand: usize,
    boundary: usize,
    cap_face: RawFaceId,
    edge: RawEdgeId,
    branch: usize,
    fragment: usize,
    root_ordinals: [usize; 2],
}

impl ParallelCylinderOverlapEndWitness {
    /// Operand owning this physical overlap end.
    pub(super) const fn operand(&self) -> usize {
        self.operand
    }

    /// Owning source boundary ordinal in authored-axis order.
    pub(super) const fn boundary(&self) -> usize {
        self.boundary
    }

    /// Topology-owned cap face.
    pub(super) const fn cap_face(&self) -> RawFaceId {
        self.cap_face
    }

    /// Vertexless cap-ring edge owning both arc endpoints.
    pub(super) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    /// Section branch carrying the cap circle.
    pub(super) const fn branch(&self) -> usize {
        self.branch
    }

    /// Section fragment carrying the retained cap arc.
    pub(super) const fn fragment(&self) -> usize {
        self.fragment
    }

    /// The two exact roots in intrinsic cap-ring parameter order.
    pub(super) const fn root_ordinals(&self) -> [usize; 2] {
        self.root_ordinals
    }
}

/// One of the two bounded common-axis rulings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ParallelCylinderRulingWitness {
    branch: usize,
    fragment: usize,
    /// Endpoint indices in low/high physical-overlap-end order.
    endpoints: [usize; 2],
    /// Source-root ordinals in low/high physical-overlap-end order.
    root_ordinals: [usize; 2],
}

impl ParallelCylinderRulingWitness {
    /// Section branch carrying this ruling.
    pub(super) const fn branch(&self) -> usize {
        self.branch
    }

    /// Section fragment carrying this bounded ruling.
    pub(super) const fn fragment(&self) -> usize {
        self.fragment
    }

    /// Endpoint indices in low/high physical-overlap-end order.
    pub(super) const fn endpoints(&self) -> [usize; 2] {
        self.endpoints
    }

    /// Source-root ordinals in low/high physical-overlap-end order.
    pub(super) const fn root_ordinals(&self) -> [usize; 2] {
        self.root_ordinals
    }
}

/// Complete operation-local proof of the strict finite lens-prism relation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CertifiedParallelCylinderLensRelation {
    component: usize,
    overlap_ends: [ParallelCylinderOverlapEndWitness; 2],
    rulings: [ParallelCylinderRulingWitness; 2],
}

impl CertifiedParallelCylinderLensRelation {
    /// Return `[inner, outer]` only when the overlap is strict axial nesting.
    pub(super) const fn strict_nesting_operands(&self) -> Option<[usize; 2]> {
        if self.overlap_ends[0].operand == self.overlap_ends[1].operand {
            let inner = self.overlap_ends[0].operand;
            Some([inner, 1 - inner])
        } else {
            None
        }
    }

    /// Unique closed section-component index.
    pub(super) const fn component(&self) -> usize {
        self.component
    }

    /// Low/high physical-overlap-end cap-arc witnesses.
    pub(super) const fn overlap_ends(&self) -> &[ParallelCylinderOverlapEndWitness; 2] {
        &self.overlap_ends
    }

    /// Two rulings sorted by their low/high source-root ordinal pair.
    pub(super) const fn rulings(&self) -> &[ParallelCylinderRulingWitness; 2] {
        &self.rulings
    }
}

/// One topology-owned cap boundary participating in a certified axial relation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ParallelCylinderAxialBoundaryWitness {
    operand: usize,
    boundary: usize,
    cap_face: RawFaceId,
    edge: RawEdgeId,
}

impl ParallelCylinderAxialBoundaryWitness {
    /// Operand owning this axial boundary.
    pub(super) const fn operand(&self) -> usize {
        self.operand
    }

    /// Owning source boundary ordinal in authored-axis order.
    pub(super) const fn boundary(&self) -> usize {
        self.boundary
    }

    /// Topology-owned cap face.
    pub(super) const fn cap_face(&self) -> RawFaceId {
        self.cap_face
    }

    /// Vertexless cap-ring edge bounding the cap.
    pub(super) const fn edge(&self) -> RawEdgeId {
        self.edge
    }
}

/// Certified proof that two finite parallel-cylinder sources have a strict axial gap.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CertifiedParallelCylinderAxialSeparation {
    gap_boundaries: [ParallelCylinderAxialBoundaryWitness; 2],
}

impl CertifiedParallelCylinderAxialSeparation {
    /// Gap boundaries in low/high order on the canonical common axis.
    pub(super) const fn gap_boundaries(&self) -> &[ParallelCylinderAxialBoundaryWitness; 2] {
        &self.gap_boundaries
    }
}

/// Certified proof that two finite parallel-cylinder sources have exact axial contact.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CertifiedParallelCylinderAxialContact {
    contact_boundaries: [ParallelCylinderAxialBoundaryWitness; 2],
}

/// Certified proof of exact external radial tangency over positive axial overlap.
///
/// The infinite side supports have exactly zero external clearance, while the
/// normalized finite source intervals overlap with strict positive length. The
/// retained cap witnesses bind that finite-overlap proof back to source topology.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CertifiedParallelCylinderExternalRadialTangency {
    overlap_boundaries: [ParallelCylinderAxialBoundaryWitness; 2],
}

impl CertifiedParallelCylinderExternalRadialTangency {
    /// Low/high boundaries of the strictly positive physical axial overlap.
    pub(super) const fn overlap_boundaries(&self) -> &[ParallelCylinderAxialBoundaryWitness; 2] {
        &self.overlap_boundaries
    }
}

impl CertifiedParallelCylinderAxialContact {
    /// Touching cap/ring boundaries in low/high order on the canonical common axis.
    pub(super) const fn contact_boundaries(&self) -> &[ParallelCylinderAxialBoundaryWitness; 2] {
        &self.contact_boundaries
    }
}

/// Certified relation or a typed fail-closed missing obligation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ParallelCylinderRelationOutcome {
    /// Exact graph proof bound to both certified source side faces shows that
    /// the infinite radial supports are strictly exterior-disjoint.
    CertifiedExteriorRadialSeparation,
    /// Exact infinite-support tangency and a strict finite axial overlap were
    /// both proved and bound to the certified source topology.
    CertifiedExternalRadialTangency(Box<CertifiedParallelCylinderExternalRadialTangency>),
    /// Exact or Full-envelope-bounded source supports prove a strict cap gap.
    CertifiedAxialSeparation(Box<CertifiedParallelCylinderAxialSeparation>),
    /// Exact live cap/ring supports prove one zero axial gap.
    CertifiedAxialContact(Box<CertifiedParallelCylinderAxialContact>),
    /// Every analytic, topology, and provenance obligation was discharged.
    Certified(Box<CertifiedParallelCylinderLensRelation>),
    /// The operation-local incomplete-Section theorem for one or two shared
    /// physical cap planes was discharged.
    CertifiedCoincidentCaps(Box<CertifiedParallelCylinderCoincidentCapRelation>),
    /// The first stable relation obligation that could not be discharged.
    Indeterminate(ParallelCylinderRelationGap),
}

/// Certify the exact strict-overlap relation consumed by the first
/// parallel-cylinder Boolean realization slices.
pub(super) fn certify_parallel_cylinder_relation(
    store: &Store,
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<ParallelCylinderRelationOutcome> {
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, PARALLEL_CYLINDER_RELATION_WORK)
        .map_err(Error::from)?;

    let supports = match certify_source_cap_supports(store, cylinders)? {
        Some(supports) => supports,
        None => {
            return Ok(ParallelCylinderRelationOutcome::Indeterminate(
                ParallelCylinderRelationGap::SourceBoundaryBinding,
            ));
        }
    };
    let normalized = match normalize_source_axial_intervals(cylinders, supports) {
        Ok(normalized) => normalized,
        Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    };
    let axial_contact = match certify_axial_nonoverlap(cylinders, &normalized) {
        Ok(Some(CertifiedAxialNonOverlap::Separation(certificate))) => {
            return Ok(ParallelCylinderRelationOutcome::CertifiedAxialSeparation(
                Box::new(certificate),
            ));
        }
        Ok(Some(CertifiedAxialNonOverlap::Contact(certificate))) => Some(certificate),
        Ok(None) => None,
        Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    };

    if certifies_exterior_radial_separation(graph, cylinders, &normalized.supports) {
        return Ok(ParallelCylinderRelationOutcome::CertifiedExteriorRadialSeparation);
    }
    if let Some(certificate) = axial_contact {
        return Ok(ParallelCylinderRelationOutcome::CertifiedAxialContact(
            Box::new(certificate),
        ));
    }
    match certify_external_radial_tangency(cylinders, &normalized) {
        Ok(Some(certificate)) => {
            return Ok(
                ParallelCylinderRelationOutcome::CertifiedExternalRadialTangency(Box::new(
                    certificate,
                )),
            );
        }
        Ok(None) => {}
        Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    }

    let overlap_ends = match certify_source_relation_from_normalized(cylinders, &normalized) {
        Ok(relation) => relation,
        Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
    };
    let overlap_ends =
        match coincident_caps::reconcile_shared_overlap_ends(graph, cylinders, overlap_ends) {
            Ok(relation) => relation,
            Err(gap) => return Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
        };
    if overlap_ends.iter().any(|end| end.contributor_count() == 2) {
        match coincident_caps::certify_coincident_cap_relation(graph, cylinders, overlap_ends) {
            Ok(certificate) => Ok(ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(
                Box::new(certificate),
            )),
            Err(gap) => Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
        }
    } else {
        match certify_section_relation(graph, cylinders, overlap_ends) {
            Ok(certificate) => Ok(ParallelCylinderRelationOutcome::Certified(Box::new(
                certificate,
            ))),
            Err(gap) => Ok(ParallelCylinderRelationOutcome::Indeterminate(gap)),
        }
    }
}

/// Bind kops' exact infinite-support tangency proof to a strict finite overlap.
///
/// The radial classifier is intentionally independent of finite windows. A
/// tangent support therefore becomes a finite-cylinder set relation only after
/// the already-normalized live cap supports prove a positive overlap interval.
fn certify_external_radial_tangency(
    cylinders: [&CertifiedCylinderSource; 2],
    normalized: &NormalizedAxialIntervals,
) -> core::result::Result<
    Option<CertifiedParallelCylinderExternalRadialTangency>,
    ParallelCylinderRelationGap,
> {
    if classify_parallel_cylinder_radial_relation([
        cylinders[0].cylinder(),
        cylinders[1].cylinder(),
    ]) != ParallelCylinderRadialRelation::ExactExternalTangent
    {
        return Ok(None);
    }

    let overlap_boundaries = source_overlap_ends(normalized)?.map(|end| {
        let boundary = cylinders[end.operand].boundaries()[end.boundary];
        ParallelCylinderAxialBoundaryWitness {
            operand: end.operand,
            boundary: end.boundary,
            cap_face: boundary.cap_face(),
            edge: boundary.edge(),
        }
    });
    Ok(Some(CertifiedParallelCylinderExternalRadialTangency {
        overlap_boundaries,
    }))
}

/// Bind Section's non-forgeable analytic miss witness to the two Full-checked
/// finite-cylinder sources consumed by this relation.
///
/// Global Section completion is intentionally irrelevant here: coincident cap
/// support planes can retain unrelated trim gaps even though exact arithmetic
/// has already proved the complete infinite radial supports disjoint. Requiring
/// the unique witnessed pair to be the two extracted side faces prevents a
/// proof from another face pair from escaping its source topology.
fn certifies_exterior_radial_separation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    supports: &[[AxialBoundarySupport; 2]; 2],
) -> bool {
    let [separation] = graph.cylinder_cylinder_exterior_radial_separations() else {
        return false;
    };
    separation
        .faces()
        .iter()
        .zip(cylinders)
        .all(|(face, cylinder)| face.raw() == cylinder.side_face())
        && exterior_radial_gap_clears_support_envelopes(cylinders, supports)
}

/// Extend the exact infinite-carrier witness over any tolerance-backed ring
/// centering admitted by Full validation. Exact sources keep zero inflation;
/// otherwise the radial carrier gap must clear both source envelopes.
fn exterior_radial_gap_clears_support_envelopes(
    cylinders: [&CertifiedCylinderSource; 2],
    supports: &[[AxialBoundarySupport; 2]; 2],
) -> bool {
    let envelopes = supports.map(|boundaries| boundaries[0].envelope.max(boundaries[1].envelope));
    if envelopes == [0.0; 2] {
        return true;
    }
    let first = cylinders[0].cylinder();
    let second = cylinders[1].cylinder();
    let distance_squared = match interval_axis_distance_squared(
        second.frame().origin(),
        first.frame().origin(),
        first.frame().z(),
    ) {
        Some(distance) => distance,
        None => return false,
    };
    let inflated_radius = Interval::point(first.radius())
        + Interval::point(second.radius())
        + Interval::point(envelopes[0])
        + Interval::point(envelopes[1]);
    let inflated_radius_squared = inflated_radius.square();
    finite_interval(distance_squared)
        && finite_interval(inflated_radius_squared)
        && distance_squared.lo() > inflated_radius_squared.hi()
}

/// Bind every finite-source cap plane and ring to its side cylinder.
///
/// Full topology validity proves incidence only to tolerance. The axial
/// theorem therefore requires exact live carrier, circle, normal, cap-plane,
/// and constant-v pcurve identities. A ring center that is an exact affine
/// cylinder evaluation has zero uncertainty. Rounded authored evaluations
/// retain the Full checker's conservative linear incidence envelope; later
/// separation comparisons must clear both boundary envelopes.
fn certify_source_cap_supports(
    store: &Store,
    cylinders: [&CertifiedCylinderSource; 2],
) -> Result<Option<[[AxialBoundarySupport; 2]; 2]>> {
    let mut supports = [[AxialBoundarySupport::zero(); 2]; 2];
    for (operand, source) in cylinders.into_iter().enumerate() {
        let side_face = store
            .get(source.side_face())
            .map_err(|source| Error::InconsistentTopology { source })?;
        let SurfaceGeom::Cylinder(live_cylinder) = store
            .surface(side_face.surface())
            .map_err(|source| Error::InconsistentTopology { source })?
        else {
            return Ok(None);
        };
        let cylinder = source.cylinder();
        if *live_cylinder != cylinder {
            return Ok(None);
        }
        let axis = cylinder.frame().z();
        for (boundary_ordinal, boundary) in source.boundaries().iter().enumerate() {
            let cap_face = store
                .get(boundary.cap_face())
                .map_err(|source| Error::InconsistentTopology { source })?;
            let SurfaceGeom::Plane(plane) = store
                .surface(cap_face.surface())
                .map_err(|source| Error::InconsistentTopology { source })?
            else {
                return Ok(None);
            };
            let edge = store
                .get(boundary.edge())
                .map_err(|source| Error::InconsistentTopology { source })?;
            let Some(curve) = edge.curve() else {
                return Ok(None);
            };
            let CurveGeom::Circle(circle) = store
                .curve(curve)
                .map_err(|source| Error::InconsistentTopology { source })?
            else {
                return Ok(None);
            };
            let side_fin = store
                .get(boundary.side_fin())
                .map_err(|source| Error::InconsistentTopology { source })?;
            let Some(side_pcurve) = side_fin.pcurve() else {
                return Ok(None);
            };
            let Curve2dGeom::Line(side_line) = store
                .pcurve(side_pcurve.curve())
                .map_err(|source| Error::InconsistentTopology { source })?
            else {
                return Ok(None);
            };
            let center = circle.frame().origin();
            let plane_normal = plane.frame().z();
            let circle_normal = circle.frame().z();
            let side_line_origin = side_line.origin();
            let side_line_direction = side_line.dir();
            if center != boundary.center()
                || circle.radius() != cylinder.radius()
                || side_fin.edge() != boundary.edge()
                || side_line_direction.y != 0.0
                || !side_line_origin.y.is_finite()
                || !finite_vec3(center)
                || !finite_vec3(plane.frame().origin())
                || !finite_vec3(plane_normal)
                || !finite_vec3(circle_normal)
                || !vectors_are_exactly_parallel(plane_normal, axis)
                || !vectors_are_exactly_parallel(circle_normal, axis)
                || !vectors_are_exactly_parallel(circle_normal, plane_normal)
                || affine_dot3(
                    plane_normal.to_array(),
                    center.to_array(),
                    plane.frame().origin().to_array(),
                    0.0,
                )
                .is_none_or(|value| value.sign() != Orientation::Zero)
            {
                return Ok(None);
            }
            supports[operand][boundary_ordinal] = AxialBoundarySupport {
                point: center,
                envelope: if axis_parameter_identity_is_exact(
                    center,
                    cylinder.frame().origin(),
                    axis,
                    side_line_origin.y,
                ) {
                    0.0
                } else {
                    // `CertifiedCylinderSource` is Full-valid, and its edge
                    // and faces carry no larger entity tolerance. The exact
                    // cap-plane/circle proof above leaves only the side
                    // pcurve lift's fixed checker envelope.
                    LINEAR_RESOLUTION
                },
                exact_domain_boundary: side_face.domain().is_some_and(|domain| {
                    side_line_origin.y
                        == if boundary_ordinal == 0 {
                            domain.v.lo
                        } else {
                            domain.v.hi
                        }
                }),
            };
        }
    }
    Ok(Some(supports))
}

/// Prove `point = origin + axis * parameter` component-by-component without
/// allowing a rounded multiply-add to erase a nonzero dyadic residual.
fn axis_parameter_identity_is_exact(
    point: Point3,
    origin: Point3,
    axis: Vec3,
    parameter: f64,
) -> bool {
    let point = point.to_array();
    let origin = origin.to_array();
    let axis = axis.to_array();
    (0..3).all(|component| {
        affine_dot3(
            [1.0, axis[component], -1.0],
            [origin[component], parameter, point[component]],
            [0.0; 3],
            0.0,
        )
        .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}

#[cfg(test)]
fn certify_source_relation(
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<[SourceOverlapEnd; 2], ParallelCylinderRelationGap> {
    let supports = cylinders.map(|source| {
        source
            .boundaries()
            .map(|boundary| AxialBoundarySupport::exact(boundary.center()))
    });
    let normalized = normalize_source_axial_intervals(cylinders, supports)?;
    certify_source_relation_from_normalized(cylinders, &normalized)
}

/// Normalize both authored cap orders onto one deterministic unoriented axis.
///
/// Every comparison is an exact affine-dot orientation. The resulting low and
/// high ordinals therefore remain source topology identities rather than
/// rounded coordinates, including when an authored axis is antiparallel.
fn normalize_source_axial_intervals(
    cylinders: [&CertifiedCylinderSource; 2],
    supports: [[AxialBoundarySupport; 2]; 2],
) -> core::result::Result<NormalizedAxialIntervals, ParallelCylinderRelationGap> {
    let cylinder_a = cylinders[0].cylinder();
    let cylinder_b = cylinders[1].cylinder();
    let axis_a = cylinder_a.frame().z();
    let axis_b = cylinder_b.frame().z();
    if !finite_vec3(axis_a)
        || !finite_vec3(axis_b)
        || !finite_vec3(cylinder_a.frame().origin())
        || !finite_vec3(cylinder_b.frame().origin())
        || !cylinder_a.radius().is_finite()
        || !cylinder_b.radius().is_finite()
        || supports
            .iter()
            .flatten()
            .any(|support| !finite_vec3(support.point) || !support.envelope.is_finite())
    {
        return Err(ParallelCylinderRelationGap::ArithmeticGuard);
    }
    if !vectors_are_exactly_parallel(axis_a, axis_b) {
        return Err(ParallelCylinderRelationGap::AxesNotExactlyParallel);
    }
    let axis_alignment = affine_dot3(axis_a.to_array(), axis_b.to_array(), [0.0; 3], 0.0)
        .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?
        .sign();
    if axis_alignment == Orientation::Zero {
        return Err(ParallelCylinderRelationGap::ArithmeticGuard);
    }
    let common_axis = canonical_unoriented_axis(axis_a)?;

    let mut intervals = [NormalizedSourceInterval::default(); 2];
    for (operand, source) in cylinders.into_iter().enumerate() {
        if axial_compare(
            source.cylinder().frame().z(),
            supports[operand][1].point,
            supports[operand][0].point,
        )? != Orientation::Positive
        {
            return Err(ParallelCylinderRelationGap::SourceAxialOrder);
        }
        intervals[operand] = match axial_compare(
            common_axis,
            supports[operand][1].point,
            supports[operand][0].point,
        )? {
            Orientation::Positive => NormalizedSourceInterval { low: 0, high: 1 },
            Orientation::Negative => NormalizedSourceInterval { low: 1, high: 0 },
            Orientation::Zero => return Err(ParallelCylinderRelationGap::SourceAxialOrder),
        };
    }

    Ok(NormalizedAxialIntervals {
        common_axis,
        sources: intervals,
        supports,
    })
}

/// Certified non-overlap of the two normalized finite axial intervals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AxialNonOverlapBoundaries {
    StrictGap([SourceBoundary; 2]),
    ExactContact([SourceBoundary; 2]),
}

/// Return the two source caps delimiting either a strict gap or exact contact.
///
/// Both source intervals must first be certified strictly positive. Contact
/// requires an exact affine-zero comparison between live ring centers and an
/// exact identity between each side pcurve height and its authored face-domain
/// end. A tolerance-backed pcurve drift therefore cannot masquerade as contact,
/// even when both cap planes happen to compare at zero.
fn axial_nonoverlap_boundaries(
    normalized: &NormalizedAxialIntervals,
) -> core::result::Result<Option<AxialNonOverlapBoundaries>, ParallelCylinderRelationGap> {
    let intervals = normalized.sources;
    for (operand, interval) in intervals.into_iter().enumerate() {
        if certified_axial_compare(
            normalized.common_axis,
            normalized.supports[operand][interval.high],
            normalized.supports[operand][interval.low],
        )? != Some(Orientation::Positive)
        {
            return Ok(None);
        }
    }

    let first_before_second = [
        SourceBoundary {
            operand: 0,
            boundary: intervals[0].high,
        },
        SourceBoundary {
            operand: 1,
            boundary: intervals[1].low,
        },
    ];
    match certified_axial_compare(
        normalized.common_axis,
        normalized.supports[1][intervals[1].low],
        normalized.supports[0][intervals[0].high],
    )? {
        Some(Orientation::Positive) => {
            return Ok(Some(AxialNonOverlapBoundaries::StrictGap(
                first_before_second,
            )));
        }
        Some(Orientation::Negative | Orientation::Zero) | None => {}
    }
    if exact_axial_contact(
        normalized.common_axis,
        normalized.supports[1][intervals[1].low],
        normalized.supports[0][intervals[0].high],
    )? {
        return Ok(Some(AxialNonOverlapBoundaries::ExactContact(
            first_before_second,
        )));
    }

    let second_before_first = [
        SourceBoundary {
            operand: 1,
            boundary: intervals[1].high,
        },
        SourceBoundary {
            operand: 0,
            boundary: intervals[0].low,
        },
    ];
    let comparison = certified_axial_compare(
        normalized.common_axis,
        normalized.supports[0][intervals[0].low],
        normalized.supports[1][intervals[1].high],
    )?;
    match comparison {
        Some(Orientation::Positive) => Ok(Some(AxialNonOverlapBoundaries::StrictGap(
            second_before_first,
        ))),
        Some(Orientation::Negative | Orientation::Zero) | None => {
            if exact_axial_contact(
                normalized.common_axis,
                normalized.supports[0][intervals[0].low],
                normalized.supports[1][intervals[1].high],
            )? {
                Ok(Some(AxialNonOverlapBoundaries::ExactContact(
                    second_before_first,
                )))
            } else {
                Ok(None)
            }
        }
    }
}

/// Test-facing strict-gap projection retained independently of contact.
#[cfg(test)]
fn strict_axial_gap_boundaries(
    normalized: &NormalizedAxialIntervals,
) -> core::result::Result<Option<[SourceBoundary; 2]>, ParallelCylinderRelationGap> {
    match axial_nonoverlap_boundaries(normalized)? {
        Some(AxialNonOverlapBoundaries::StrictGap(boundaries)) => Ok(Some(boundaries)),
        Some(AxialNonOverlapBoundaries::ExactContact(_)) | None => Ok(None),
    }
}

enum CertifiedAxialNonOverlap {
    Separation(CertifiedParallelCylinderAxialSeparation),
    Contact(CertifiedParallelCylinderAxialContact),
}

fn certify_axial_nonoverlap(
    cylinders: [&CertifiedCylinderSource; 2],
    normalized: &NormalizedAxialIntervals,
) -> core::result::Result<Option<CertifiedAxialNonOverlap>, ParallelCylinderRelationGap> {
    let Some(relation) = axial_nonoverlap_boundaries(normalized)? else {
        return Ok(None);
    };
    let boundaries = match relation {
        AxialNonOverlapBoundaries::StrictGap(boundaries)
        | AxialNonOverlapBoundaries::ExactContact(boundaries) => boundaries,
    };
    let witnesses = boundaries.map(|source| {
        let boundary = cylinders[source.operand].boundaries()[source.boundary];
        ParallelCylinderAxialBoundaryWitness {
            operand: source.operand,
            boundary: source.boundary,
            cap_face: boundary.cap_face(),
            edge: boundary.edge(),
        }
    });
    Ok(Some(match relation {
        AxialNonOverlapBoundaries::StrictGap(_) => {
            CertifiedAxialNonOverlap::Separation(CertifiedParallelCylinderAxialSeparation {
                gap_boundaries: witnesses,
            })
        }
        AxialNonOverlapBoundaries::ExactContact(_) => {
            CertifiedAxialNonOverlap::Contact(CertifiedParallelCylinderAxialContact {
                contact_boundaries: witnesses,
            })
        }
    }))
}

fn certify_source_relation_from_normalized(
    cylinders: [&CertifiedCylinderSource; 2],
    normalized: &NormalizedAxialIntervals,
) -> core::result::Result<[SourceOverlapEnd; 2], ParallelCylinderRelationGap> {
    certify_strict_radial_secancy(cylinders)?;
    source_overlap_ends(normalized)
}

fn source_overlap_ends(
    normalized: &NormalizedAxialIntervals,
) -> core::result::Result<[SourceOverlapEnd; 2], ParallelCylinderRelationGap> {
    let common_axis = normalized.common_axis;
    let intervals = normalized.sources;
    let low = axial_compare(
        common_axis,
        normalized.supports[1][intervals[1].low].point,
        normalized.supports[0][intervals[0].low].point,
    )?;
    let high = axial_compare(
        common_axis,
        normalized.supports[1][intervals[1].high].point,
        normalized.supports[0][intervals[0].high].point,
    )?;
    let low_end = match low {
        Orientation::Positive => SourceOverlapEnd {
            operand: 1,
            boundary: intervals[1].low,
            peer_boundary: None,
        },
        Orientation::Negative => SourceOverlapEnd {
            operand: 0,
            boundary: intervals[0].low,
            peer_boundary: None,
        },
        Orientation::Zero => SourceOverlapEnd {
            operand: 0,
            boundary: intervals[0].low,
            peer_boundary: Some(intervals[1].low),
        },
    };
    let high_end = match high {
        Orientation::Positive => SourceOverlapEnd {
            operand: 0,
            boundary: intervals[0].high,
            peer_boundary: None,
        },
        Orientation::Negative => SourceOverlapEnd {
            operand: 1,
            boundary: intervals[1].high,
            peer_boundary: None,
        },
        Orientation::Zero => SourceOverlapEnd {
            operand: 0,
            boundary: intervals[0].high,
            peer_boundary: Some(intervals[1].high),
        },
    };
    if axial_compare(
        common_axis,
        normalized.supports[high_end.operand][high_end.boundary].point,
        normalized.supports[low_end.operand][low_end.boundary].point,
    )? != Orientation::Positive
    {
        return Err(ParallelCylinderRelationGap::AxialOverlapNotStrictlyPositive);
    }
    Ok([low_end, high_end])
}

fn canonical_unoriented_axis(
    axis: Vec3,
) -> core::result::Result<Vec3, ParallelCylinderRelationGap> {
    for basis in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        let sign = affine_dot3(axis.to_array(), basis, [0.0; 3], 0.0)
            .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?
            .sign();
        match sign {
            Orientation::Positive => return Ok(axis),
            Orientation::Negative => return Ok(-axis),
            Orientation::Zero => {}
        }
    }
    Err(ParallelCylinderRelationGap::ArithmeticGuard)
}

fn certify_strict_radial_secancy(
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    let first = cylinders[0].cylinder();
    let second = cylinders[1].cylinder();
    let distance_squared = interval_axis_distance_squared(
        second.frame().origin(),
        first.frame().origin(),
        first.frame().z(),
    )
    .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?;
    let radius_a = Interval::point(first.radius());
    let radius_b = Interval::point(second.radius());
    let radius_difference_squared = (radius_a - radius_b).square();
    let radius_sum_squared = (radius_a + radius_b).square();
    if !finite_interval(distance_squared)
        || !finite_interval(radius_difference_squared)
        || !finite_interval(radius_sum_squared)
    {
        return Err(ParallelCylinderRelationGap::ArithmeticGuard);
    }
    if distance_squared.lo() > radius_difference_squared.hi()
        && distance_squared.hi() < radius_sum_squared.lo()
    {
        Ok(())
    } else {
        Err(ParallelCylinderRelationGap::RadialSecancyNotStrict)
    }
}

fn axial_compare(
    axis: Vec3,
    point: Point3,
    origin: Point3,
) -> core::result::Result<Orientation, ParallelCylinderRelationGap> {
    affine_dot3(axis.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
        .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)
}

/// Prove zero separation from exact live cap/ring and side-chart identities.
///
/// Rounded model-space construction can require a Full incidence envelope for
/// a ring center relative to its cylinder evaluation. That does not weaken an
/// exact cap-plane contact: both constant-height side pcurves must still equal
/// their live face-domain ends bit-for-bit, and the cap centers must have exact
/// zero affine projection on the common axis.
fn exact_axial_contact(
    axis: Vec3,
    point: AxialBoundarySupport,
    origin: AxialBoundarySupport,
) -> core::result::Result<bool, ParallelCylinderRelationGap> {
    if !point.exact_domain_boundary || !origin.exact_domain_boundary {
        return Ok(false);
    }
    axial_compare(axis, point.point, origin.point)
        .map(|orientation| orientation == Orientation::Zero)
}

/// Compare two cap supports only when their projected separation clears the
/// Full-certified Euclidean incidence envelopes. Zero-envelope comparisons
/// retain exact affine-dot behavior, including a one-ULP world-frame gap.
fn certified_axial_compare(
    axis: Vec3,
    point: AxialBoundarySupport,
    origin: AxialBoundarySupport,
) -> core::result::Result<Option<Orientation>, ParallelCylinderRelationGap> {
    if point.envelope == 0.0 && origin.envelope == 0.0 {
        return axial_compare(axis, point.point, origin.point).map(Some);
    }
    let axis_norm = interval_norm_squared(interval_vec3(axis))
        .sqrt()
        .ok_or(ParallelCylinderRelationGap::ArithmeticGuard)?;
    let envelope =
        ((Interval::point(point.envelope) + Interval::point(origin.envelope)) * axis_norm).hi();
    let projection = affine_interval(axis, point.point, origin.point);
    if !envelope.is_finite() || !finite_interval(projection) {
        return Err(ParallelCylinderRelationGap::ArithmeticGuard);
    }
    if projection.lo() > envelope {
        Ok(Some(Orientation::Positive))
    } else if projection.hi() < -envelope {
        Ok(Some(Orientation::Negative))
    } else {
        Ok(None)
    }
}

fn affine_interval(axis: Vec3, point: Point3, origin: Point3) -> Interval {
    let axis = axis.to_array();
    let point = point.to_array();
    let origin = origin.to_array();
    (0..3).fold(Interval::point(0.0), |sum, component| {
        sum + Interval::point(axis[component])
            * (Interval::point(point[component]) - Interval::point(origin[component]))
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SourceOverlapEnd {
    /// Canonical first contributor. Shared ends use operand zero here and
    /// retain operand one's authored boundary in `peer_boundary`.
    operand: usize,
    boundary: usize,
    peer_boundary: Option<usize>,
}

impl SourceOverlapEnd {
    const fn boundary_for(self, operand: usize) -> Option<usize> {
        if operand == self.operand {
            Some(self.boundary)
        } else if operand == 1 - self.operand {
            self.peer_boundary
        } else {
            None
        }
    }

    const fn contributor_count(self) -> usize {
        if self.peer_boundary.is_some() { 2 } else { 1 }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct NormalizedSourceInterval {
    low: usize,
    high: usize,
}

#[derive(Debug, Clone, Copy)]
struct AxialBoundarySupport {
    point: Point3,
    envelope: f64,
    exact_domain_boundary: bool,
}

impl AxialBoundarySupport {
    const fn zero() -> Self {
        Self {
            point: Point3::new(0.0, 0.0, 0.0),
            envelope: 0.0,
            exact_domain_boundary: false,
        }
    }

    #[cfg(test)]
    const fn exact(point: Point3) -> Self {
        Self {
            point,
            envelope: 0.0,
            exact_domain_boundary: true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct NormalizedAxialIntervals {
    common_axis: Vec3,
    sources: [NormalizedSourceInterval; 2],
    supports: [[AxialBoundarySupport; 2]; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SourceBoundary {
    operand: usize,
    boundary: usize,
}

#[derive(Debug, Clone, Copy)]
struct BoundEndpoint {
    endpoint: usize,
    overlap_end: usize,
    root_ordinal: usize,
}

#[derive(Debug, Clone, Copy)]
struct EndpointBindRequest<'a> {
    endpoint: usize,
    source_parameter: &'a SectionSourceParameterKey,
    trim: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
struct PendingRuling {
    branch: usize,
    fragment: usize,
    ends: [BoundEndpoint; 2],
}

#[derive(Debug, Clone, Copy)]
struct PendingCapArc {
    branch: usize,
    fragment: usize,
    overlap_end: usize,
    cap_operand: usize,
    ends: [BoundEndpoint; 2],
}

fn certify_section_relation(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    overlap_ends: [SourceOverlapEnd; 2],
) -> core::result::Result<CertifiedParallelCylinderLensRelation, ParallelCylinderRelationGap> {
    if graph.completion() != crate::SectionCompletion::Complete || !graph.gaps().is_empty() {
        return Err(ParallelCylinderRelationGap::SectionIncomplete);
    }
    if graph.branches().len() != 4
        || graph.curve_fragments().len() != 4
        || graph.curve_endpoints().len() != 4
        || graph.curve_components().len() != 1
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    certify_periodic_bindings(graph, cylinders)?;

    let component = &graph.curve_components()[0];
    if !component.closed() || component.fragments().len() != 4 {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    let mut covered_fragments = [false; 4];
    let mut covered_branches = [false; 4];
    let mut kinds = [0_u8; 4];
    let mut rulings = Vec::with_capacity(2);
    let mut cap_arcs: [Option<PendingCapArc>; 2] = [None, None];
    for (component_ordinal, &fragment_index) in component.fragments().iter().enumerate() {
        if fragment_index >= 4 || covered_fragments[fragment_index] {
            return Err(ParallelCylinderRelationGap::SectionLayout);
        }
        covered_fragments[fragment_index] = true;
        let fragment = &graph.curve_fragments()[fragment_index];
        let branch_index = fragment.branch();
        if branch_index >= 4 || covered_branches[branch_index] || fragment.source_ordinal() != 0 {
            return Err(ParallelCylinderRelationGap::SectionLayout);
        }
        covered_branches[branch_index] = true;
        let branch = &graph.branches()[branch_index];
        match fragment.span() {
            SectionCurveFragmentSpan::LineSegment { endpoints } => {
                kinds[component_ordinal] = 1;
                if rulings.len() == 2 {
                    return Err(ParallelCylinderRelationGap::SectionLayout);
                }
                rulings.push(certify_ruling(
                    graph,
                    cylinders,
                    overlap_ends,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?);
            }
            SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                kinds[component_ordinal] = 2;
                let arc = certify_cap_arc(
                    graph,
                    cylinders,
                    overlap_ends,
                    branch_index,
                    fragment_index,
                    branch,
                    endpoints,
                )?;
                if cap_arcs[arc.overlap_end].replace(arc).is_some() {
                    return Err(ParallelCylinderRelationGap::SectionLayout);
                }
            }
            SectionCurveFragmentSpan::Whole => {
                return Err(ParallelCylinderRelationGap::SectionLayout);
            }
        }
    }
    if !covered_fragments.into_iter().all(|covered| covered)
        || !covered_branches.into_iter().all(|covered| covered)
        || rulings.len() != 2
        || kinds
            .into_iter()
            .enumerate()
            .any(|(index, kind)| kind == kinds[(index + 1) % kinds.len()])
    {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    }
    let [Some(cap_low), Some(cap_high)] = cap_arcs else {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    };
    let [first_ruling, second_ruling] = rulings.as_slice() else {
        return Err(ParallelCylinderRelationGap::SectionLayout);
    };
    let mut pending_rulings = [*first_ruling, *second_ruling];

    let mut incidence = [0_u8; 4];
    let mut endpoint_kinds = [0_u8; 4];
    let mut endpoint_binding: [Option<(usize, usize)>; 4] = [None; 4];
    for ruling in pending_rulings {
        for end in ruling.ends {
            record_endpoint(
                end,
                1,
                &mut incidence,
                &mut endpoint_kinds,
                &mut endpoint_binding,
            )?;
        }
    }
    for arc in [cap_low, cap_high] {
        for end in arc.ends {
            record_endpoint(
                end,
                2,
                &mut incidence,
                &mut endpoint_kinds,
                &mut endpoint_binding,
            )?;
        }
    }
    if incidence.into_iter().any(|count| count != 2)
        || endpoint_kinds.into_iter().any(|kind| kind != 3)
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }

    let mut arc_endpoint_by_root = [[usize::MAX; 2]; 2];
    for arc in [cap_low, cap_high] {
        for end in arc.ends {
            if end.root_ordinal >= 2
                || arc_endpoint_by_root[arc.overlap_end][end.root_ordinal] != usize::MAX
            {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
            arc_endpoint_by_root[arc.overlap_end][end.root_ordinal] = end.endpoint;
        }
    }
    if arc_endpoint_by_root
        .into_iter()
        .flatten()
        .any(|endpoint| endpoint == usize::MAX)
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    for ruling in pending_rulings {
        for end in ruling.ends {
            if arc_endpoint_by_root[end.overlap_end][end.root_ordinal] != end.endpoint {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
        }
    }

    pending_rulings.sort_by_key(|ruling| {
        let by_overlap_end = ends_by_overlap_end(ruling.ends);
        [
            by_overlap_end[0].root_ordinal,
            by_overlap_end[1].root_ordinal,
        ]
    });
    let rulings = pending_rulings.map(|ruling| {
        let ends = ends_by_overlap_end(ruling.ends);
        ParallelCylinderRulingWitness {
            branch: ruling.branch,
            fragment: ruling.fragment,
            endpoints: [ends[0].endpoint, ends[1].endpoint],
            root_ordinals: [ends[0].root_ordinal, ends[1].root_ordinal],
        }
    });
    let overlap_ends = [cap_low, cap_high].map(|arc| {
        let mut roots = [arc.ends[0].root_ordinal, arc.ends[1].root_ordinal];
        roots.sort_unstable();
        let source_end = overlap_ends[arc.overlap_end];
        ParallelCylinderOverlapEndWitness {
            operand: source_end.operand,
            boundary: source_end.boundary,
            cap_face: cylinders[source_end.operand].boundaries()[source_end.boundary].cap_face(),
            edge: cylinders[source_end.operand].boundaries()[source_end.boundary].edge(),
            branch: arc.branch,
            fragment: arc.fragment,
            root_ordinals: roots,
        }
    });
    Ok(CertifiedParallelCylinderLensRelation {
        component: 0,
        overlap_ends,
        rulings,
    })
}

fn certify_periodic_bindings(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    if graph.periodic_face_embeddings().len() != 2 {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let mut seen = [false; 2];
    for evidence in graph.periodic_face_embeddings() {
        let SectionPeriodicFaceEmbeddingEvidence::Certified(evidence) = evidence else {
            return Err(ParallelCylinderRelationGap::SectionOperandBinding);
        };
        let operand = evidence.operand();
        if operand >= 2 || seen[operand] || evidence.face().raw() != cylinders[operand].side_face()
        {
            return Err(ParallelCylinderRelationGap::SectionOperandBinding);
        }
        seen[operand] = true;
    }
    if seen.into_iter().all(|value| value) {
        Ok(())
    } else {
        Err(ParallelCylinderRelationGap::SectionOperandBinding)
    }
}

#[allow(clippy::too_many_arguments)]
fn certify_ruling(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    overlap_ends: [SourceOverlapEnd; 2],
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionRulingFragmentEnd; 2],
) -> core::result::Result<PendingRuling, ParallelCylinderRelationGap> {
    if branch.faces()[0].raw() != cylinders[0].side_face()
        || branch.faces()[1].raw() != cylinders[1].side_face()
        || branch.topology() != SectionBranchTopology::Open
        || branch.endpoint_sites() != [0, 1]
        || branch.fragment_sites().len() != 2
        || !matches!(branch.pcurves()[0], SectionUvCurve::Line(_))
        || !matches!(branch.pcurves()[1], SectionUvCurve::Line(_))
    {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let SectionCarrier::Line { origin, direction } = branch.carrier() else {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    };
    if !finite_vec3(origin)
        || !finite_vec3(direction)
        || !vectors_are_exactly_parallel(direction, cylinders[0].cylinder().frame().z())
        || !valid_branch_evidence(branch)
    {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    }
    let mut ends: [Option<BoundEndpoint>; 2] = [None, None];
    for end in endpoints.iter() {
        if !finite_vec3(end.point()) || !end.carrier_parameter().is_finite() {
            return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
        }
        let trims = end.trims();
        let (trim_operand, trim) = match trims {
            [Some(trim), None] => (0, trim),
            [None, Some(trim)] => (1, trim),
            _ => return Err(ParallelCylinderRelationGap::SectionEndpointProvenance),
        };
        if trim.operand() != trim_operand
            || trim.face().raw() != cylinders[trim_operand].side_face()
            || !valid_interval(trim.carrier_parameter().lo(), trim.carrier_parameter().hi())
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let binding = bind_endpoint(
            graph,
            cylinders,
            overlap_ends,
            EndpointBindRequest {
                endpoint: end.endpoint(),
                source_parameter: trim.source_parameter(),
                trim: [trim.edge_parameter().lo(), trim.edge_parameter().hi()],
            },
        )?;
        if ends[binding.overlap_end].replace(binding).is_some() {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    }
    let [Some(low), Some(high)] = ends else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    Ok(PendingRuling {
        branch: branch_index,
        fragment: fragment_index,
        ends: [low, high],
    })
}

#[allow(clippy::too_many_arguments)]
fn certify_cap_arc(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    overlap_ends: [SourceOverlapEnd; 2],
    branch_index: usize,
    fragment_index: usize,
    branch: &SectionBranch,
    endpoints: &[crate::SectionCurveFragmentEnd; 2],
) -> core::result::Result<PendingCapArc, ParallelCylinderRelationGap> {
    let mut matching_contributors = Vec::new();
    for (overlap_end, source_end) in overlap_ends.into_iter().enumerate() {
        for cap_operand in 0..2 {
            let Some(boundary) = source_end.boundary_for(cap_operand) else {
                continue;
            };
            if branch.faces()[cap_operand].raw()
                == cylinders[cap_operand].boundaries()[boundary].cap_face()
            {
                matching_contributors.push((overlap_end, source_end, cap_operand, boundary));
            }
        }
    }
    let [(overlap_end, _source_end, cap_operand, boundary)] = matching_contributors.as_slice()
    else {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    };
    let overlap_end = *overlap_end;
    let cap_operand = *cap_operand;
    let boundary = *boundary;
    let side_operand = 1 - cap_operand;
    if branch.faces()[side_operand].raw() != cylinders[side_operand].side_face()
        || branch.topology() != SectionBranchTopology::Closed
        || !matches!(branch.pcurves()[cap_operand], SectionUvCurve::Circle(_))
        || !matches!(branch.pcurves()[side_operand], SectionUvCurve::Line(_))
    {
        return Err(ParallelCylinderRelationGap::SectionOperandBinding);
    }
    let SectionCarrier::Circle {
        center,
        normal,
        x_direction,
        radius,
    } = branch.carrier()
    else {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    };
    if !finite_vec3(center)
        || !finite_vec3(normal)
        || !finite_vec3(x_direction)
        || !radius.is_finite()
        || radius <= 0.0
        || (!vectors_are_exactly_parallel(normal, cylinders[0].cylinder().frame().z())
            && !has_certified_plane_cylinder_circle_traces(branch, cap_operand, side_operand))
        || !valid_branch_evidence(branch)
    {
        return Err(ParallelCylinderRelationGap::SectionBranchEvidence);
    }
    let mut ends: [Option<BoundEndpoint>; 2] = [None, None];
    for (end_index, end) in endpoints.iter().enumerate() {
        let trim = end.trim();
        if !finite_vec3(end.point())
            || !end.carrier_parameter().is_finite()
            || trim.operand() != cap_operand
            || trim.face().raw() != cylinders[cap_operand].boundaries()[boundary].cap_face()
            || !valid_interval(trim.edge_parameter().lo(), trim.edge_parameter().hi())
            || !valid_interval(trim.pcurve_half_angle().lo(), trim.pcurve_half_angle().hi())
        {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        let binding = bind_endpoint(
            graph,
            cylinders,
            overlap_ends,
            EndpointBindRequest {
                endpoint: end.endpoint(),
                source_parameter: trim.source_parameter(),
                trim: [trim.edge_parameter().lo(), trim.edge_parameter().hi()],
            },
        )?;
        if binding.overlap_end != overlap_end {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
        ends[end_index] = Some(binding);
    }
    let [Some(first), Some(second)] = ends else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    if first.endpoint == second.endpoint || first.root_ordinal == second.root_ordinal {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    Ok(PendingCapArc {
        branch: branch_index,
        fragment: fragment_index,
        overlap_end,
        cap_operand,
        ends: [first, second],
    })
}

fn bind_endpoint(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    overlap_ends: [SourceOverlapEnd; 2],
    request: EndpointBindRequest<'_>,
) -> core::result::Result<BoundEndpoint, ParallelCylinderRelationGap> {
    let [trim_lo, trim_hi] = request.trim;
    let endpoint_index = request.endpoint;
    let source_parameter = request.source_parameter;
    let endpoint = graph
        .curve_endpoints()
        .get(endpoint_index)
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let mut matching_contributors = Vec::new();
    for (overlap_end, source_end) in overlap_ends.into_iter().enumerate() {
        for cap_operand in 0..2 {
            let Some(boundary) = source_end.boundary_for(cap_operand) else {
                continue;
            };
            if source_parameter.edge().raw() == cylinders[cap_operand].boundaries()[boundary].edge()
            {
                matching_contributors.push((overlap_end, source_end, cap_operand, boundary));
            }
        }
    }
    let [(overlap_end, source_end, cap_operand, boundary)] = matching_contributors.as_slice()
    else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    let overlap_end = *overlap_end;
    let source_end = *source_end;
    let cap_operand = *cap_operand;
    let boundary = *boundary;
    let side_operand = 1 - cap_operand;
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = endpoint.topology()
    else {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    };
    if !matches!(
        &sites[cap_operand],
        SectionSite::EdgeInterior(edge) if edge.raw() == cylinders[cap_operand].boundaries()[boundary].edge()
    ) || source_parameters[cap_operand].as_ref() != Some(source_parameter)
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    match source_end.boundary_for(side_operand) {
        Some(peer_boundary) => {
            let expected_edge = cylinders[side_operand].boundaries()[peer_boundary].edge();
            let peer = source_parameters[side_operand]
                .as_ref()
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            let peer_interval = peer.root_parameter_enclosure();
            let common = endpoint.edge_parameters()[side_operand]
                .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
            if !matches!(
                &sites[side_operand],
                SectionSite::EdgeInterior(edge) if edge.raw() == expected_edge
            ) || peer.edge().raw() != expected_edge
                || peer.root_ordinal() >= 2
                || !peer.root_parameter().is_finite()
                || !valid_interval(peer_interval.lo(), peer_interval.hi())
                || !peer_interval.contains(peer.root_parameter())
                || !valid_interval(common.lo(), common.hi())
            {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
        }
        None => {
            if !matches!(
                &sites[side_operand],
                SectionSite::FaceInterior(face) if face.raw() == cylinders[side_operand].side_face()
            ) || source_parameters[side_operand].is_some()
                || endpoint.edge_parameters()[side_operand].is_some()
            {
                return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
            }
        }
    }
    let common = endpoint.edge_parameters()[cap_operand]
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    let root_enclosure = source_parameter.root_parameter_enclosure();
    if source_parameter.root_ordinal() >= 2
        || !source_parameter.root_parameter().is_finite()
        || !valid_interval(root_enclosure.lo(), root_enclosure.hi())
        || !valid_interval(trim_lo, trim_hi)
        || !valid_interval(common.lo(), common.hi())
        || common.lo() < trim_lo
        || common.hi() > trim_hi
        || !root_enclosure.contains(source_parameter.root_parameter())
    {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    Ok(BoundEndpoint {
        endpoint: endpoint_index,
        overlap_end,
        root_ordinal: source_parameter.root_ordinal(),
    })
}

fn valid_branch_evidence(branch: &SectionBranch) -> bool {
    let range = branch.range();
    let evidence = branch.evidence();
    range.is_finite()
        && range.lo < range.hi
        && evidence.tolerance().is_finite()
        && evidence.tolerance() > 0.0
        && evidence
            .residual_bounds()
            .into_iter()
            .all(|bound| bound.is_finite() && bound >= 0.0 && bound <= evidence.tolerance())
}

/// Recognize the semantic chart family carried by a graph-certified
/// Plane/Cylinder circle. This is the representation theorem used when a
/// translated oblique carrier's reconstructed model-space normal is not
/// bitwise parallel to the authored cylinder axis: the exact face binding,
/// closed carrier topology, and paired whole-range residual evidence remain
/// the incidence authority, while these coefficients prove a plane circle
/// paired with a constant-height, whole-period cylinder trace.
fn has_certified_plane_cylinder_circle_traces(
    branch: &SectionBranch,
    plane_operand: usize,
    cylinder_operand: usize,
) -> bool {
    let (SectionUvCurve::Circle(circle), SectionUvCurve::Line(line)) = (
        branch.pcurves()[plane_operand],
        branch.pcurves()[cylinder_operand],
    ) else {
        return false;
    };
    let center = circle.center();
    let x_direction = circle.x_direction();
    let origin = line.origin();
    let direction = line.direction();
    [
        center.x,
        center.y,
        circle.radius(),
        x_direction.x,
        x_direction.y,
        circle.parameter_scale(),
        circle.parameter_offset(),
        origin.x,
        origin.y,
        direction.x,
        direction.y,
    ]
    .into_iter()
    .all(f64::is_finite)
        && circle.radius() > 0.0
        && circle.parameter_scale() != 0.0
        && direction.x != 0.0
        && direction.y == 0.0
}

fn record_endpoint(
    end: BoundEndpoint,
    kind: u8,
    incidence: &mut [u8; 4],
    kinds: &mut [u8; 4],
    bindings: &mut [Option<(usize, usize)>; 4],
) -> core::result::Result<(), ParallelCylinderRelationGap> {
    if end.endpoint >= incidence.len() {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    incidence[end.endpoint] = incidence[end.endpoint]
        .checked_add(1)
        .ok_or(ParallelCylinderRelationGap::SectionEndpointProvenance)?;
    if kinds[end.endpoint] & kind != 0 {
        return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
    }
    kinds[end.endpoint] |= kind;
    let binding = (end.overlap_end, end.root_ordinal);
    if let Some(existing) = bindings[end.endpoint] {
        if existing != binding {
            return Err(ParallelCylinderRelationGap::SectionEndpointProvenance);
        }
    } else {
        bindings[end.endpoint] = Some(binding);
    }
    Ok(())
}

fn ends_by_overlap_end(ends: [BoundEndpoint; 2]) -> [BoundEndpoint; 2] {
    if ends[0].overlap_end == 0 {
        ends
    } else {
        [ends[1], ends[0]]
    }
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

#[derive(Debug, Clone, Copy)]
struct IntervalVec3 {
    x: Interval,
    y: Interval,
    z: Interval,
}

fn interval_vec3(value: Vec3) -> IntervalVec3 {
    IntervalVec3 {
        x: Interval::point(value.x),
        y: Interval::point(value.y),
        z: Interval::point(value.z),
    }
}

fn interval_sub(first: IntervalVec3, second: IntervalVec3) -> IntervalVec3 {
    IntervalVec3 {
        x: first.x - second.x,
        y: first.y - second.y,
        z: first.z - second.z,
    }
}

fn interval_cross(first: IntervalVec3, second: IntervalVec3) -> IntervalVec3 {
    IntervalVec3 {
        x: first.y * second.z - first.z * second.y,
        y: first.z * second.x - first.x * second.z,
        z: first.x * second.y - first.y * second.x,
    }
}

fn interval_norm_squared(value: IntervalVec3) -> Interval {
    value.x.square() + value.y.square() + value.z.square()
}

/// Outward enclosure of squared Euclidean distance from `point` to the
/// infinite line through `origin` in direction `axis`.
pub(super) fn interval_axis_distance_squared(
    point: Point3,
    origin: Point3,
    axis: Vec3,
) -> Option<Interval> {
    let axis = interval_vec3(axis);
    let displacement = interval_sub(interval_vec3(point), interval_vec3(origin));
    interval_norm_squared(interval_cross(displacement, axis))
        .checked_div(interval_norm_squared(axis))
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn finite_vec3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn valid_interval(lo: f64, hi: f64) -> bool {
    lo.is_finite() && hi.is_finite() && lo <= hi
}

#[cfg(test)]
#[path = "parallel_cylinder_relation/tests.rs"]
mod tests;
