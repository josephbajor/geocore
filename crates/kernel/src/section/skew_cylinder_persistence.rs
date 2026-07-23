//! Validated Section handoff for persistent bounded skew-cylinder branches.
//!
//! The operation-local graph proof and physical Section roots are intentionally
//! private in different owners. This adapter rejoins them only after checking
//! the complete branch facade and both directed ends. Boolean materialization
//! can therefore consume one sealed value instead of independently selecting a
//! residual proof, root corridors, traversal orientation, and topology points.

use kcore::interval::Interval;
use kgeom::vec::Point3;
use kgraph::{
    PairedSkewCylinderBranchResidualCertificate, SkewCylinderBranchGuardedEnd,
    SkewCylinderBranchPcurveRootCorridorCertificate,
};

use super::skew_cylinder_public::orient_parameter_interval;
use super::{
    SectionBoundedProceduralFragmentEnd, SectionBoundedProceduralPhysicalRoot, SectionBranch,
    SectionBranchTopology, SectionCarrier, SectionCurveFragment, SectionCurveFragmentSpan,
    SectionSkewCylinderInterval, SectionUvCurve,
};

/// Sealed persistence handoff for one bounded skew-cylinder Section fragment.
///
/// `root_corridors` and `physical_roots` are both in graph canonical
/// `[lower, upper]` order. A physical root's carrier enclosure remains in
/// Section orientation; `reversed` records the exact involution relating that
/// enclosure to the corresponding raw graph corridor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SectionSkewCylinderPersistenceInput {
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    reversed: bool,
    physical_roots: [SectionBoundedProceduralPhysicalRoot; 2],
}

impl SectionSkewCylinderPersistenceInput {
    /// Guarded paired residual certificate in graph source order.
    pub(crate) const fn residual_certificate(self) -> PairedSkewCylinderBranchResidualCertificate {
        self.residual
    }

    /// Raw root corridors in graph canonical `[lower, upper]` order.
    pub(crate) const fn root_corridors(
        self,
    ) -> [SkewCylinderBranchPcurveRootCorridorCertificate; 2] {
        self.root_corridors
    }

    /// Whether Section traversal reverses graph canonical longitude.
    pub(crate) const fn reversed(self) -> bool {
        self.reversed
    }

    /// Physical topology roots in graph canonical `[lower, upper]` order.
    ///
    /// Each root retains its topology endpoint, topology-owned point, and
    /// Section-oriented canonical-longitude enclosure.
    pub(crate) const fn physical_roots(self) -> [SectionBoundedProceduralPhysicalRoot; 2] {
        self.physical_roots
    }

    /// Topology-owned endpoint points in graph canonical `[lower, upper]` order.
    pub(crate) const fn physical_endpoint_points(self) -> [Point3; 2] {
        [
            self.physical_roots[0].point(),
            self.physical_roots[1].point(),
        ]
    }
}

/// Rejoin one bounded procedural fragment with its graph-owned skew proof.
///
/// The raw proof never crosses the Section boundary unless the branch facade,
/// directed root corridors, guarded representatives, physical root
/// enclosures, and trim provenance still agree. The returned value is ordered
/// for the graph certifier, while retaining the Section reversal explicitly.
///
/// The caller must resolve `branch` from the same graph using
/// `fragment.branch()`. A standalone [`SectionBranch`] intentionally carries no
/// graph index, so this value-level adapter cannot prove that index association;
/// mixed-plan adoption must establish it before calling this function.
pub(crate) fn bounded_skew_persistence_input(
    branch: &SectionBranch,
    fragment: &SectionCurveFragment,
) -> Option<SectionSkewCylinderPersistenceInput> {
    let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = fragment.span() else {
        return None;
    };
    let ends: &[SectionBoundedProceduralFragmentEnd; 2] = endpoints.as_ref();
    let embedding = branch.embedding_certificate()?;
    let source = embedding.source_certificate();
    let residual = source.residual_certificate();
    let root_corridors = source.root_corridors();
    let range = branch.range();
    let reversed = embedding.reversed();

    if branch.topology() != SectionBranchTopology::Open
        || !range.is_finite()
        || embedding.range() != range
        || residual.carrier_range() != range
        || branch.evidence().residual_bounds().map(f64::to_bits)
            != residual.residual_bounds().map(f64::to_bits)
        || branch.evidence().tolerance().to_bits() != residual.tolerance().to_bits()
    {
        return None;
    }

    let SectionCarrier::SkewCylinderBranch(carrier) = branch.carrier() else {
        return None;
    };
    if carrier.source() != residual.carrier()
        || carrier.range() != range
        || carrier.reversed() != reversed
    {
        return None;
    }

    let SectionUvCurve::SkewCylinderBranch(first_pcurve) = branch.pcurves()[0] else {
        return None;
    };
    let SectionUvCurve::SkewCylinderBranch(second_pcurve) = branch.pcurves()[1] else {
        return None;
    };
    let pcurves = [first_pcurve, second_pcurve];
    let traces = residual.traces();
    if pcurves.iter().zip(traces).any(|(pcurve, trace)| {
        pcurve.source() != trace.pcurve()
            || pcurve.range() != range
            || pcurve.reversed() != reversed
    }) {
        return None;
    }

    if !valid_raw_corridors(residual, root_corridors) {
        return None;
    }

    let section_roots = ends.each_ref().map(|end| end.physical_root());
    if section_roots[0].endpoint() == section_roots[1].endpoint() {
        return None;
    }
    for (section_end, end) in ends.iter().enumerate() {
        let graph_end = if reversed {
            1 - section_end
        } else {
            section_end
        };
        if !valid_section_end(
            branch,
            carrier,
            range,
            reversed,
            section_end,
            end,
            root_corridors[graph_end],
        ) {
            return None;
        }
    }

    let physical_roots = if reversed {
        [section_roots[1], section_roots[0]]
    } else {
        section_roots
    };
    Some(SectionSkewCylinderPersistenceInput {
        residual,
        root_corridors,
        reversed,
        physical_roots,
    })
}

fn valid_raw_corridors(
    residual: PairedSkewCylinderBranchResidualCertificate,
    corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
) -> bool {
    let range = residual.carrier_range();
    let expected_operands = residual.traces().map(|trace| trace.pcurve().operand());
    for (graph_end, corridor) in corridors.into_iter().enumerate() {
        let expected_end = if graph_end == 0 {
            SkewCylinderBranchGuardedEnd::Lower
        } else {
            SkewCylinderBranchGuardedEnd::Upper
        };
        let root = corridor.root_parameter();
        let cell = corridor.corridor();
        let expected_parameter = if graph_end == 0 {
            if root.hi() >= range.lo {
                return false;
            }
            Interval::new(root.lo(), range.lo)
        } else {
            if root.lo() <= range.hi {
                return false;
            }
            Interval::new(range.hi, root.hi())
        };
        if corridor.guarded_end() != expected_end
            || !finite_interval(root)
            || cell.parameter() != expected_parameter
            || corridor.root_pcurves().map(|pcurve| pcurve.operand()) != expected_operands
            || cell.pcurves().map(|pcurve| pcurve.operand()) != expected_operands
            || !cell.carrier_box().is_finite()
            || cell
                .residual_bounds()
                .into_iter()
                .any(|bound| !bound.is_finite() || bound < 0.0 || bound > residual.tolerance())
            || corridor
                .root_pcurves()
                .into_iter()
                .chain(cell.pcurves())
                .any(|pcurve| {
                    !pcurve.stored_is_strictly_regular() || !pcurve.source_is_strictly_regular()
                })
        {
            return false;
        }
    }
    true
}

#[allow(clippy::too_many_arguments)]
fn valid_section_end(
    branch: &SectionBranch,
    carrier: super::SectionSkewCylinderBranchCarrier,
    range: kgeom::param::ParamRange,
    reversed: bool,
    section_end: usize,
    end: &SectionBoundedProceduralFragmentEnd,
    graph_corridor: SkewCylinderBranchPcurveRootCorridorCertificate,
) -> bool {
    let physical = end.physical_root();
    let section_enclosure = physical.carrier_parameter();
    let expected_enclosure =
        orient_parameter_interval(range, graph_corridor.root_parameter(), reversed);
    let expected_guard = if section_end == 0 { range.lo } else { range.hi };
    let inside_point = end.inside_point();
    let trim = end.trim();
    let trim_operand = trim.operand();
    let source_parameter = trim.source_parameter();
    let source_enclosure = source_parameter.root_parameter_enclosure();
    let edge_enclosure = trim.edge_parameter();
    let projective_root = trim.carrier_root();

    section_enclosure == expected_enclosure
        && finite_section_interval(section_enclosure)
        && end.inside_carrier_parameter().to_bits() == expected_guard.to_bits()
        && finite_point(physical.point())
        && finite_point(inside_point)
        && same_point_bits(inside_point, carrier.eval(expected_guard))
        && trim_operand < 2
        && trim.face() == branch.faces()[trim_operand]
        && source_parameter.root_parameter().is_finite()
        && source_enclosure.lo().is_finite()
        && source_enclosure.hi().is_finite()
        && source_enclosure.lo() <= source_enclosure.hi()
        && source_enclosure.contains(source_parameter.root_parameter())
        && edge_enclosure.lo().is_finite()
        && edge_enclosure.hi().is_finite()
        && edge_enclosure.lo() <= source_enclosure.lo()
        && source_enclosure.hi() <= edge_enclosure.hi()
        && projective_root.lo().is_finite()
        && projective_root.hi().is_finite()
        && projective_root.lo() <= projective_root.hi()
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite() && value.lo() <= value.hi()
}

fn finite_section_interval(value: SectionSkewCylinderInterval) -> bool {
    value.lo().is_finite() && value.hi().is_finite() && value.lo() <= value.hi()
}

fn finite_point(point: Point3) -> bool {
    point.to_array().into_iter().all(f64::is_finite)
}

fn same_point_bits(first: Point3, second: Point3) -> bool {
    first.to_array().map(f64::to_bits) == second.to_array().map(f64::to_bits)
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;

    use super::*;
    use crate::{BodySectionGraph, CylinderRequest, Kernel, SectionBodiesRequest};

    fn bounded_skew_graph(swapped: bool) -> BodySectionGraph {
        let frame = Frame::world();
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (first, second) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let first = edit
                .create_cylinder(CylinderRequest::new(
                    frame.with_origin(Point3::new(0.0, 0.0, 1.8)),
                    1.0,
                    0.1,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let second = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::new(Point3::new(-1.25, 0.0, 0.0), frame.x(), frame.y()).unwrap(),
                    2.0,
                    2.5,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (first, second)
        };
        let bodies = if swapped {
            [second, first]
        } else {
            [first, second]
        };
        session
            .part(part)
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(
                bodies[0].clone(),
                bodies[1].clone(),
            ))
            .unwrap()
            .into_result()
            .unwrap()
    }

    #[test]
    fn persistence_input_preserves_graph_root_order_across_section_reversal() {
        let mut saw_orientation = [false; 2];
        for swapped in [false, true] {
            let graph = bounded_skew_graph(swapped);
            for fragment in graph.curve_fragments() {
                let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = fragment.span()
                else {
                    continue;
                };
                let branch = &graph.branches()[fragment.branch()];
                let input = bounded_skew_persistence_input(branch, fragment)
                    .expect("published bounded skew evidence must rejoin");
                saw_orientation[usize::from(input.reversed())] = true;

                let residual = input.residual_certificate();
                let corridors = input.root_corridors();
                assert_eq!(residual.carrier_range(), branch.range());
                assert_eq!(
                    corridors.map(|corridor| corridor.guarded_end()),
                    [
                        SkewCylinderBranchGuardedEnd::Lower,
                        SkewCylinderBranchGuardedEnd::Upper
                    ]
                );

                let section_roots = endpoints
                    .each_ref()
                    .map(|endpoint| endpoint.physical_root());
                let expected_graph_roots = if input.reversed() {
                    [section_roots[1], section_roots[0]]
                } else {
                    section_roots
                };
                assert_eq!(input.physical_roots(), expected_graph_roots);
                assert_eq!(
                    input.physical_endpoint_points(),
                    expected_graph_roots.map(|root| root.point())
                );
                for (graph_end, physical) in input.physical_roots().into_iter().enumerate() {
                    assert_eq!(
                        physical.carrier_parameter(),
                        orient_parameter_interval(
                            branch.range(),
                            corridors[graph_end].root_parameter(),
                            input.reversed()
                        )
                    );
                }
            }
        }
        assert_eq!(
            saw_orientation,
            [true, true],
            "fixture must exercise both Section traversal orientations"
        );
    }

    #[test]
    fn malformed_branch_and_endpoint_splices_are_rejected() {
        let graph = bounded_skew_graph(false);
        let bounded = graph
            .curve_fragments()
            .iter()
            .filter(|fragment| {
                matches!(
                    fragment.span(),
                    SectionCurveFragmentSpan::BoundedProcedural { .. }
                )
            })
            .collect::<Vec<_>>();
        let first_fragment = bounded[0];
        let first_branch = &graph.branches()[first_fragment.branch()];
        let other_fragment = bounded
            .iter()
            .copied()
            .find(|fragment| fragment.branch() != first_fragment.branch())
            .expect("fixture must retain distinct bounded branches");
        let other_branch = &graph.branches()[other_fragment.branch()];

        assert!(bounded_skew_persistence_input(first_branch, first_fragment).is_some());
        assert!(
            bounded_skew_persistence_input(first_branch, other_fragment).is_none(),
            "an end pair from another branch must not splice into this proof"
        );

        let mut swapped_ends = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut swapped_ends.span
        else {
            unreachable!()
        };
        endpoints.swap(0, 1);
        assert!(bounded_skew_persistence_input(first_branch, &swapped_ends).is_none());

        let mut mismatched_embedding = first_branch.clone();
        mismatched_embedding.skew_cylinder_embedding = other_branch.skew_cylinder_embedding.clone();
        assert!(bounded_skew_persistence_input(&mismatched_embedding, first_fragment).is_none());

        let mut mismatched_pcurves = first_branch.clone();
        mismatched_pcurves.pcurves.swap(0, 1);
        assert!(bounded_skew_persistence_input(&mismatched_pcurves, first_fragment).is_none());

        let mut nonfinite_point = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut nonfinite_point.span
        else {
            unreachable!()
        };
        let root = endpoints[0].physical_root();
        endpoints[0].physical_root = SectionBoundedProceduralPhysicalRoot::new(
            root.endpoint(),
            root.carrier_parameter(),
            Point3::new(f64::NAN, root.point().y, root.point().z),
        );
        assert!(bounded_skew_persistence_input(first_branch, &nonfinite_point).is_none());

        let mut mismatched_trim = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut mismatched_trim.span
        else {
            unreachable!()
        };
        let trim_operand = endpoints[0].trim.operand;
        endpoints[0].trim.face = first_branch.faces()[1 - trim_operand].clone();
        assert!(bounded_skew_persistence_input(first_branch, &mismatched_trim).is_none());
    }
}
