//! Exact endpoint evidence for bounded procedural skew-cylinder branches.

use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kgraph::SkewCylinderSheet;

/// Caller-authored axial side that clips a skew-cylinder branch endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderAxialBoundaryProof {
    /// Low end of the source cylinder's axial window.
    Lower,
    /// High end of the source cylinder's axial window.
    Upper,
}

/// Strict sheet relation to an authored axial bound beside one exact root.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderAxialRelationProof {
    /// The sheet height is strictly below the bound.
    Below,
    /// The sheet height is strictly above the bound.
    Above,
}

/// Projective chart that owns an exact skew-cylinder axial root enclosure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderHalfAngleChartProof {
    /// Tangent half-angle chart.
    Tangent,
    /// Cotangent half-angle chart.
    Cotangent,
}

/// Side of the exact root corridor retained by the bounded component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderRootInsideSideProof {
    /// Increasing-longitude side immediately before the root.
    Before,
    /// Increasing-longitude side immediately after the root.
    After,
}

/// Exact-source identity and certified inside-side representative for one
/// bounded skew-cylinder endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderAxialRootEndpointProof {
    /// Source cylinder index in the caller's operand order.
    pub source_operand: usize,
    /// Authored side of that cylinder's axial window.
    pub boundary: SkewCylinderAxialBoundaryProof,
    /// Exact caller-authored axial bound used by the root equation.
    pub bound: f64,
    /// Procedural sheet that owns the root.
    pub sheet: SkewCylinderSheet,
    /// Ordinal of the distinct cut in the source equation's canonical cycle.
    pub cyclic_ordinal: usize,
    /// Projective chart retaining the exact source-root identity.
    pub half_angle_chart: SkewCylinderHalfAngleChartProof,
    /// Isolating interval in the owning half-angle chart.
    pub half_angle_bracket: [f64; 2],
    /// Strict relation immediately before the root in increasing longitude.
    pub before: SkewCylinderAxialRelationProof,
    /// Strict relation immediately after the root in increasing longitude.
    pub after: SkewCylinderAxialRelationProof,
    /// Which side of the root corridor belongs to the retained component.
    pub inside_side: SkewCylinderRootInsideSideProof,
    /// Representable carrier parameter on the retained span's inside side.
    pub inside_parameter: f64,
}

/// Exact discriminant-root identity and metric representative for one shared
/// join of a two-sheet folded support component.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderFoldedSupportRootEndpointProof {
    /// Ordinal in the exact canonical two-root cycle.
    pub root_ordinal: usize,
    /// Projective chart retaining the exact source-root identity.
    pub half_angle_chart: SkewCylinderHalfAngleChartProof,
    /// Isolating interval in the owning half-angle chart.
    pub half_angle_bracket: [f64; 2],
    /// Hidden inside-cell carrier coordinate used by the guarded evaluator.
    pub inside_parameter: f64,
    /// Exact-root-owned deterministic model-space representative.
    pub point: Point3,
    /// Caller-order source parameters at the exact support join.
    pub surface_parameters: [[f64; 2]; 2],
}

/// Exact topological evidence attached to one branch endpoint slot.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum IntersectionBranchEndpointProof {
    /// Simple transverse root of one caller-authored cylinder axial bound.
    SkewCylinderAxialRoot(SkewCylinderAxialRootEndpointProof),
    /// Simple discriminant root shared by both members of a folded component.
    SkewCylinderFoldedSupportRoot(SkewCylinderFoldedSupportRootEndpointProof),
}

impl IntersectionBranchEndpointProof {
    pub(super) fn validated_boundary_surfaces(
        self,
        parameter: f64,
        sheet: SkewCylinderSheet,
        surface_ranges: [[ParamRange; 2]; 2],
    ) -> Option<[bool; 2]> {
        let Self::SkewCylinderAxialRoot(proof) = self else {
            return None;
        };
        if proof.source_operand > 1 {
            return None;
        }
        let expected_bound = match proof.boundary {
            SkewCylinderAxialBoundaryProof::Lower => surface_ranges[proof.source_operand][1].lo,
            SkewCylinderAxialBoundaryProof::Upper => surface_ranges[proof.source_operand][1].hi,
        };
        let inside_relation = match proof.inside_side {
            SkewCylinderRootInsideSideProof::Before => proof.before,
            SkewCylinderRootInsideSideProof::After => proof.after,
        };
        let required_relation = match proof.boundary {
            SkewCylinderAxialBoundaryProof::Lower => SkewCylinderAxialRelationProof::Above,
            SkewCylinderAxialBoundaryProof::Upper => SkewCylinderAxialRelationProof::Below,
        };
        if proof.inside_parameter != parameter
            || proof.sheet != sheet
            || expected_bound.to_bits() != proof.bound.to_bits()
            || inside_relation != required_relation
            || proof.before == proof.after
        {
            return None;
        }
        Some(core::array::from_fn(|operand| {
            operand == proof.source_operand
        }))
    }

    pub(super) fn validated_folded_support_root(
        self,
        parameter: f64,
        surface_ranges: [[ParamRange; 2]; 2],
    ) -> Option<SkewCylinderFoldedSupportRootEndpointProof> {
        let Self::SkewCylinderFoldedSupportRoot(proof) = self else {
            return None;
        };
        if proof.root_ordinal > 1
            || proof.inside_parameter != parameter
            || !proof.half_angle_bracket[0].is_finite()
            || !proof.half_angle_bracket[1].is_finite()
            || proof.half_angle_bracket[0] > proof.half_angle_bracket[1]
            || !proof.point.to_array().into_iter().all(f64::is_finite)
            || proof
                .surface_parameters
                .into_iter()
                .flatten()
                .any(|value| !value.is_finite())
            || proof
                .surface_parameters
                .into_iter()
                .zip(surface_ranges)
                .any(|(parameters, ranges)| {
                    !ranges[0].contains(parameters[0]) || !ranges[1].contains(parameters[1])
                })
        {
            None
        } else {
            Some(proof)
        }
    }
}
