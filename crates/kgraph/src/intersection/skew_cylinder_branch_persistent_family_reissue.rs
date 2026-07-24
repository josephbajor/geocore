//! Rigid-placement reissuance for complete finite-window skew families.

use kgeom::surface::Cylinder;
use kgeom::vec::Vec3;

use super::super::{
    IntersectionCertificateError, PersistentSkewCylinderOpenSpanCertificate,
    PersistentSkewCylinderOpenSpanOrientation, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    SkewCylinderAxialBoundProvenance, SkewCylinderAxialBoundary,
    SkewCylinderExactDiscriminantTopology, SkewCylinderOpenSpanTopologyInput, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_finite_window_family,
    certify_persistent_skew_cylinder_open_span_in_family, classify_skew_cylinder_axial_bound,
    classify_skew_cylinder_exact_discriminant, classify_skew_cylinder_open_spans,
};
use super::{
    PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS, PersistentSkewCylinderAxialBoundary,
    PersistentSkewCylinderFiniteWindowFamilyCertificate,
    PersistentSkewCylinderFiniteWindowMemberInput, derived_open_members,
    validate_finite_window_family_membership,
};

/// A complete transformed family plus the freshly certified member evidence.
///
/// The source family is retained so selecting a member from a different
/// family cannot silently reuse an ordinal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowFamilyReissue {
    source: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    certificate: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    members: [Option<PersistentSkewCylinderFiniteWindowMemberInput>;
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS],
}

impl PersistentSkewCylinderFiniteWindowFamilyReissue {
    /// Source family whose deterministic member order was transformed.
    pub const fn source_family(self) -> PersistentSkewCylinderFiniteWindowFamilyCertificate {
        self.source
    }

    /// Fresh complete family over the transformed cylinders.
    pub const fn certificate(self) -> PersistentSkewCylinderFiniteWindowFamilyCertificate {
        self.certificate
    }

    /// Reissue one source member using transformed logical endpoint points.
    pub fn reissue_member(
        self,
        source: PersistentSkewCylinderOpenSpanCertificate,
        transformed_logical_endpoint_points: [Vec3; 2],
    ) -> Result<PersistentSkewCylinderOpenSpanCertificate, IntersectionCertificateError> {
        let membership = source
            .finite_window_family_membership()
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        if membership.family() != self.source {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        validate_finite_window_family_membership(
            membership,
            source.residual_certificate(),
            source.root_corridors(),
        )?;
        let ordinal = membership.ordinal();
        let input = self
            .members
            .get(ordinal)
            .copied()
            .flatten()
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let transformed_membership = self
            .certificate
            .membership(ordinal)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let canonical_endpoint_points = match source.orientation() {
            PersistentSkewCylinderOpenSpanOrientation::Forward => {
                transformed_logical_endpoint_points
            }
            PersistentSkewCylinderOpenSpanOrientation::Reversed => [
                transformed_logical_endpoint_points[1],
                transformed_logical_endpoint_points[0],
            ],
        };
        certify_persistent_skew_cylinder_open_span_in_family(
            input.residual,
            input.root_corridors,
            canonical_endpoint_points,
            source.orientation(),
            transformed_membership,
        )
    }
}

/// Rebuild a complete finite-window family over rigidly transformed cylinders.
///
/// Parameter windows and source ordering stay unchanged under a rigid
/// placement. Exact discriminant, axial-bound, finite-occupancy, residual,
/// and root-corridor evidence are all recomputed from the transformed
/// cylinders; no evaluator or enclosure from the source family is cloned.
pub fn reissue_persistent_skew_cylinder_finite_window_family(
    source: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    transformed_formula_cylinders: [Cylinder; 2],
) -> Result<PersistentSkewCylinderFiniteWindowFamilyReissue, IntersectionCertificateError> {
    let admission = match classify_skew_cylinder_exact_discriminant(
        transformed_formula_cylinders,
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    )
    .map_err(|_| IntersectionCertificateError::HarmonicRootClassification)?
    {
        SkewCylinderExactDiscriminantTopology::StrictPositive(admission) => admission,
        SkewCylinderExactDiscriminantTopology::StrictNegative
        | SkewCylinderExactDiscriminantTopology::Contact => {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
    };
    let formula_to_source = source.formula_to_source();
    let formula_windows = source.formula_windows();
    let mut bound_topologies = Vec::with_capacity(4);
    for index in 0..4 {
        let outcome = source
            .axial_bound_outcome(index)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let boundary = match outcome.tag().boundary() {
            PersistentSkewCylinderAxialBoundary::Lower => SkewCylinderAxialBoundary::Lower,
            PersistentSkewCylinderAxialBoundary::Upper => SkewCylinderAxialBoundary::Upper,
        };
        bound_topologies.push(
            classify_skew_cylinder_axial_bound(
                transformed_formula_cylinders,
                formula_to_source,
                SkewCylinderAxialBoundProvenance {
                    source_operand: outcome.tag().source_slot(),
                    boundary,
                    value: outcome.bound(),
                },
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )
            .map_err(|_| IntersectionCertificateError::HarmonicRootClassification)?,
        );
    }
    let bound_topologies = bound_topologies
        .try_into()
        .map_err(|_| IntersectionCertificateError::InvalidTraceFamily)?;
    let finite_topology = classify_skew_cylinder_open_spans(SkewCylinderOpenSpanTopologyInput {
        topologies: &bound_topologies,
        ranges: formula_windows,
        canonical_to_source: formula_to_source,
    })
    .map_err(|_| IntersectionCertificateError::InvalidTraceFamily)?;
    let mut member_inputs = Vec::new();
    for span in derived_open_members(&finite_topology) {
        let residual = certify_paired_skew_cylinder_branch_subrange_residuals(
            transformed_formula_cylinders,
            formula_windows,
            span.range,
            span.sheet,
            source.tolerance(),
        )?;
        let roots = span
            .root_longitude_intervals(formula_windows[0][0])
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        member_inputs.push(PersistentSkewCylinderFiniteWindowMemberInput {
            residual,
            root_corridors: [
                residual.certify_lower_pcurve_root_corridor(roots[0])?,
                residual.certify_upper_pcurve_root_corridor(roots[1])?,
            ],
        });
    }
    let certificate = certify_persistent_skew_cylinder_finite_window_family(
        admission,
        &finite_topology,
        &member_inputs,
        source.tolerance(),
    )?;
    validate_reissued_shape(source, certificate)?;
    let mut members = [None; PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS];
    for (slot, member) in members.iter_mut().zip(member_inputs) {
        *slot = Some(member);
    }
    Ok(PersistentSkewCylinderFiniteWindowFamilyReissue {
        source,
        certificate,
        members,
    })
}

fn validate_reissued_shape(
    source: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    reissued: PersistentSkewCylinderFiniteWindowFamilyCertificate,
) -> Result<(), IntersectionCertificateError> {
    if source.formula_windows() != reissued.formula_windows()
        || source.formula_to_source() != reissued.formula_to_source()
        || source.root_cluster_query_plan() != reissued.root_cluster_query_plan()
        || source.member_count() != reissued.member_count()
        || source.sheet_occupancy(SkewCylinderSheet::Lower)
            != reissued.sheet_occupancy(SkewCylinderSheet::Lower)
        || source.sheet_occupancy(SkewCylinderSheet::Upper)
            != reissued.sheet_occupancy(SkewCylinderSheet::Upper)
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        if source.root_event_count(sheet) != reissued.root_event_count(sheet) {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        for ordinal in 0..source.root_event_count(sheet) {
            let original = source
                .root_event(sheet, ordinal)
                .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
            let transformed = reissued
                .root_event(sheet, ordinal)
                .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
            if original.sheet() != transformed.sheet()
                || original.kind() != transformed.kind()
                || original.root_count() != transformed.root_count()
                || (0..original.root_count()).any(|root_ordinal| {
                    source
                        .root_event_root(sheet, ordinal, root_ordinal)
                        .zip(reissued.root_event_root(sheet, ordinal, root_ordinal))
                        .is_none_or(|roots| !same_root_shape(roots))
                })
            {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
        }
    }
    for ordinal in 0..source.member_count() {
        let original = source
            .member(ordinal)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let transformed = reissued
            .member(ordinal)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let endpoint_identity_matches = original.endpoints().into_iter().enumerate().all(
            |(endpoint_ordinal, original_endpoint)| {
                let transformed_endpoint = transformed.endpoints()[endpoint_ordinal];
                original_endpoint.sheet() == transformed_endpoint.sheet()
                    && original_endpoint.inside_side() == transformed_endpoint.inside_side()
                    && original_endpoint.root_count() == transformed_endpoint.root_count()
                    && (0..original_endpoint.root_count()).all(|root_ordinal| {
                        source
                            .member_endpoint_root(ordinal, endpoint_ordinal, root_ordinal)
                            .zip(reissued.member_endpoint_root(
                                ordinal,
                                endpoint_ordinal,
                                root_ordinal,
                            ))
                            .is_some_and(same_root_shape)
                    })
            },
        );
        if original.sheet() != transformed.sheet() || !endpoint_identity_matches {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
    }
    Ok(())
}

fn same_root_shape(
    (original, transformed): (
        super::PersistentSkewCylinderAxialRootEventInput,
        super::PersistentSkewCylinderAxialRootEventInput,
    ),
) -> bool {
    original.tag == transformed.tag
        && original.bound.to_bits() == transformed.bound.to_bits()
        && original.sheet == transformed.sheet
        && original.cyclic_ordinal == transformed.cyclic_ordinal
        && original.half_angle_chart == transformed.half_angle_chart
        && original.before == transformed.before
        && original.after == transformed.after
        && original.repeated == transformed.repeated
}
