//! Validated Section handoff for persistent bounded skew-cylinder branches.
//!
//! The operation-local graph proof and physical Section roots are intentionally
//! private in different owners. This adapter rejoins them only after checking
//! the complete branch facade and both directed ends. Boolean materialization
//! can therefore consume one sealed value instead of independently selecting a
//! residual proof, root corridors, traversal orientation, and topology points.

use kcore::interval::Interval;
use kgeom::surface::Cylinder;
use kgeom::vec::Point3;
use kgraph::{
    PairedSkewCylinderBranchResidualCertificate, PersistentSkewCylinderAxialBoundary,
    PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate,
    PersistentSkewCylinderHalfAngleChart, PersistentSkewCylinderRootInsideSide,
    SkewCylinderBranchGuardedEnd, SkewCylinderBranchPcurveRootCorridorCertificate,
};
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;

use super::skew_cylinder_public::orient_parameter_interval;
use super::{
    SectionBoundedProceduralFragmentEnd, SectionBoundedProceduralPhysicalRoot, SectionBranch,
    SectionBranchTopology, SectionCarrier, SectionCurveFragment, SectionCurveFragmentSpan,
    SectionSkewCylinderAxialBoundary, SectionSkewCylinderInterval, SectionUvCurve,
};

/// Exact caller-authored slab identity retained in graph canonical end order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SectionSkewCylinderEndpointSlab {
    source_operand: usize,
    boundary: SectionSkewCylinderAxialBoundary,
    bound: f64,
}

impl SectionSkewCylinderEndpointSlab {
    /// Source cylinder slot in current Section operand order.
    pub(crate) const fn source_operand(self) -> usize {
        self.source_operand
    }

    /// Authored lower/upper axial side.
    pub(crate) const fn boundary(self) -> SectionSkewCylinderAxialBoundary {
        self.boundary
    }

    /// Bit-exact axial bound used by the source root equation.
    pub(crate) const fn bound(self) -> f64 {
        self.bound
    }
}

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
    family_membership: PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate,
    reversed: bool,
    physical_roots: [SectionBoundedProceduralPhysicalRoot; 2],
    endpoint_slabs: [SectionSkewCylinderEndpointSlab; 2],
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

    /// Complete finite-window family and immutable represented ordinal.
    pub(crate) const fn family_membership(
        self,
    ) -> PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate {
        self.family_membership
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

    /// Exact source-window tags in graph canonical `[lower, upper]` order.
    pub(crate) const fn endpoint_slabs(self) -> [SectionSkewCylinderEndpointSlab; 2] {
        self.endpoint_slabs
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
/// `store` re-resolves the retained source side face so the authored lower/
/// upper tag and bit-exact bound cannot be replaced after Section publication.
pub(crate) fn bounded_skew_persistence_input(
    store: &Store,
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
    let family_membership = source.finite_window_family_membership()?;
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
        let trim_operand = end.trim().operand();
        if trim_operand > 1 {
            return None;
        }
        let graph_end = if reversed {
            1 - section_end
        } else {
            section_end
        };
        if !valid_section_end(
            store,
            branch,
            carrier,
            traces[trim_operand].surface(),
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
    let section_slabs = ends.each_ref().map(|end| {
        let trim = end.trim();
        SectionSkewCylinderEndpointSlab {
            source_operand: trim.operand(),
            boundary: trim.axial_boundary(),
            bound: trim.authored_bound(),
        }
    });
    let endpoint_slabs = if reversed {
        [section_slabs[1], section_slabs[0]]
    } else {
        section_slabs
    };
    let graph_ends = if reversed {
        [&ends[1], &ends[0]]
    } else {
        [&ends[0], &ends[1]]
    };
    if !valid_family_member(
        family_membership,
        residual,
        root_corridors,
        range,
        graph_ends,
    ) {
        return None;
    }
    Some(SectionSkewCylinderPersistenceInput {
        residual,
        root_corridors,
        family_membership,
        reversed,
        physical_roots,
        endpoint_slabs,
    })
}

fn valid_family_member(
    membership: PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate,
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    range: kgeom::param::ParamRange,
    graph_ends: [&SectionBoundedProceduralFragmentEnd; 2],
) -> bool {
    let member = membership.member();
    if member.sheet() != residual.sheet()
        || member.guarded_range() != range
        || member.root_parameter_enclosures()
            != root_corridors.map(|corridor| corridor.root_parameter())
        || member.tolerance().to_bits() != residual.tolerance().to_bits()
        || membership.family().source_cylinders() != residual.traces().map(|trace| trace.surface())
    {
        return false;
    }

    for (graph_end, (proof, section_end)) in
        member.endpoints().into_iter().zip(graph_ends).enumerate()
    {
        let Some(root) = membership.endpoint_root(graph_end, 0) else {
            return false;
        };
        let trim = section_end.trim();
        let projective = trim.carrier_root();
        let expected_boundary = match trim.axial_boundary() {
            SectionSkewCylinderAxialBoundary::Lower => PersistentSkewCylinderAxialBoundary::Lower,
            SectionSkewCylinderAxialBoundary::Upper => PersistentSkewCylinderAxialBoundary::Upper,
        };
        let expected_chart = match projective.chart() {
            super::SectionSkewCylinderRootChart::TangentHalfAngle => {
                PersistentSkewCylinderHalfAngleChart::Tangent
            }
            super::SectionSkewCylinderRootChart::CotangentHalfAngle => {
                PersistentSkewCylinderHalfAngleChart::Cotangent
            }
        };
        let expected_inside_side = if graph_end == 0 {
            PersistentSkewCylinderRootInsideSide::After
        } else {
            PersistentSkewCylinderRootInsideSide::Before
        };
        let expected_inside_parameter = if graph_end == 0 { range.lo } else { range.hi };
        let bracket = root.half_angle_bracket;
        if proof.root_count() != 1
            || proof.sheet() != residual.sheet()
            || root.tag.source_slot() != trim.operand()
            || root.tag.boundary() != expected_boundary
            || root.bound.to_bits() != trim.authored_bound().to_bits()
            || proof.inside_side() != expected_inside_side
            || proof.inside_parameter().to_bits() != expected_inside_parameter.to_bits()
            || root.half_angle_chart != expected_chart
            || bracket.map(f64::to_bits) != [projective.lo(), projective.hi()].map(f64::to_bits)
        {
            return false;
        }
    }
    true
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
    store: &Store,
    branch: &SectionBranch,
    carrier: super::SectionSkewCylinderBranchCarrier,
    source_cylinder: Cylinder,
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
    if trim_operand > 1 {
        return false;
    }
    let source_parameter = trim.source_parameter();
    let source_enclosure = source_parameter.root_parameter_enclosure();
    let edge_enclosure = trim.edge_parameter();
    let projective_root = trim.carrier_root();
    let source_face = store.get(trim.face().raw()).ok();
    let source_domain = source_face.and_then(|face| face.domain());
    let source_surface = source_face
        .and_then(|face| store.surface(face.surface()).ok())
        .and_then(|surface| match surface {
            SurfaceGeom::Cylinder(cylinder) => Some(*cylinder),
            _ => None,
        });
    let authored_bound = source_domain.map(|domain| match trim.axial_boundary() {
        SectionSkewCylinderAxialBoundary::Lower => domain.v.lo,
        SectionSkewCylinderAxialBoundary::Upper => domain.v.hi,
    });
    let root_height = graph_corridor.root_pcurves()[trim_operand];

    section_enclosure == expected_enclosure
        && finite_section_interval(section_enclosure)
        && end.inside_carrier_parameter().to_bits() == expected_guard.to_bits()
        && finite_point(physical.point())
        && finite_point(inside_point)
        && same_point_bits(inside_point, carrier.eval(expected_guard))
        && trim.face() == branch.faces()[trim_operand]
        && source_surface == Some(source_cylinder)
        && authored_bound.is_some_and(|bound| bound.to_bits() == trim.authored_bound().to_bits())
        && root_height.stored_uv()[1].contains(trim.authored_bound())
        && root_height.source_uv()[1].contains(trim.authored_bound())
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
    use crate::{BodySectionGraph, CylinderRequest, Kernel, PartId, SectionBodiesRequest, Session};

    fn bounded_skew_fixture(swapped: bool) -> (Session, PartId, BodySectionGraph) {
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
        let graph = session
            .part(part.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(
                bodies[0].clone(),
                bodies[1].clone(),
            ))
            .unwrap()
            .into_result()
            .unwrap();
        (session, part, graph)
    }

    #[test]
    fn persistence_input_preserves_graph_root_order_across_section_reversal() {
        let mut saw_orientation = [false; 2];
        for swapped in [false, true] {
            let (session, part_id, graph) = bounded_skew_fixture(swapped);
            let part = session.part(part_id).unwrap();
            let mut observed_family = None;
            let mut observed_ordinals = Vec::new();
            for fragment in graph.curve_fragments() {
                let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = fragment.span()
                else {
                    continue;
                };
                let branch = &graph.branches()[fragment.branch()];
                let input = bounded_skew_persistence_input(&part.state.store, branch, fragment)
                    .expect("published bounded skew evidence must rejoin");
                saw_orientation[usize::from(input.reversed())] = true;

                let membership = input.family_membership();
                let family = membership.family();
                if let Some(expected) = observed_family {
                    assert_eq!(
                        family, expected,
                        "all fragments must share one complete family"
                    );
                } else {
                    observed_family = Some(family);
                }
                observed_ordinals.push(membership.ordinal());

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
                let section_slabs = endpoints.each_ref().map(|endpoint| {
                    let trim = endpoint.trim();
                    SectionSkewCylinderEndpointSlab {
                        source_operand: trim.operand(),
                        boundary: trim.axial_boundary(),
                        bound: trim.authored_bound(),
                    }
                });
                let expected_graph_slabs = if input.reversed() {
                    [section_slabs[1], section_slabs[0]]
                } else {
                    section_slabs
                };
                assert_eq!(input.physical_roots(), expected_graph_roots);
                assert_eq!(input.endpoint_slabs(), expected_graph_slabs);
                for (endpoint_ordinal, (endpoint, slab)) in membership
                    .member()
                    .endpoints()
                    .into_iter()
                    .zip(expected_graph_slabs)
                    .enumerate()
                {
                    let root = membership.endpoint_root(endpoint_ordinal, 0).unwrap();
                    let expected_boundary = match slab.boundary() {
                        SectionSkewCylinderAxialBoundary::Lower => {
                            PersistentSkewCylinderAxialBoundary::Lower
                        }
                        SectionSkewCylinderAxialBoundary::Upper => {
                            PersistentSkewCylinderAxialBoundary::Upper
                        }
                    };
                    assert_eq!(endpoint.root_count(), 1);
                    assert_eq!(root.tag.source_slot(), slab.source_operand());
                    assert_eq!(root.tag.boundary(), expected_boundary);
                    assert_eq!(root.bound.to_bits(), slab.bound().to_bits());
                }
                assert_eq!(
                    input.physical_endpoint_points(),
                    expected_graph_roots.map(|root| root.point())
                );
                for endpoint in endpoints.iter() {
                    let trim = endpoint.trim();
                    let face = part.state.store.get(trim.face().raw()).unwrap();
                    let domain = face.domain().expect("cylinder side face must be bounded");
                    let expected = match trim.axial_boundary() {
                        SectionSkewCylinderAxialBoundary::Lower => domain.v.lo,
                        SectionSkewCylinderAxialBoundary::Upper => domain.v.hi,
                    };
                    assert_eq!(
                        trim.authored_bound().to_bits(),
                        expected.to_bits(),
                        "Section must retain the authored slab side and exact local bound"
                    );
                }
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
            let family = observed_family.expect("fixture must publish bounded family members");
            observed_ordinals.sort_unstable();
            assert_eq!(
                observed_ordinals,
                (0..family.member_count()).collect::<Vec<_>>(),
                "Section must publish every admitted family ordinal exactly once"
            );
        }
        assert_eq!(
            saw_orientation,
            [true, true],
            "fixture must exercise both Section traversal orientations"
        );
    }

    #[test]
    fn malformed_branch_and_endpoint_splices_are_rejected() {
        let (session, part_id, graph) = bounded_skew_fixture(false);
        let part = session.part(part_id).unwrap();
        let store = &part.state.store;
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

        assert!(bounded_skew_persistence_input(store, first_branch, first_fragment).is_some());
        assert!(
            bounded_skew_persistence_input(store, first_branch, other_fragment).is_none(),
            "an end pair from another branch must not splice into this proof"
        );

        let mut swapped_ends = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut swapped_ends.span
        else {
            unreachable!()
        };
        endpoints.swap(0, 1);
        assert!(bounded_skew_persistence_input(store, first_branch, &swapped_ends).is_none());

        let mut mismatched_embedding = first_branch.clone();
        mismatched_embedding.skew_cylinder_embedding = other_branch.skew_cylinder_embedding.clone();
        assert!(
            bounded_skew_persistence_input(store, &mismatched_embedding, first_fragment).is_none()
        );

        let mut mismatched_pcurves = first_branch.clone();
        mismatched_pcurves.pcurves.swap(0, 1);
        assert!(
            bounded_skew_persistence_input(store, &mismatched_pcurves, first_fragment).is_none()
        );

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
        assert!(bounded_skew_persistence_input(store, first_branch, &nonfinite_point).is_none());

        let mut mismatched_trim = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut mismatched_trim.span
        else {
            unreachable!()
        };
        let trim_operand = endpoints[0].trim.operand;
        endpoints[0].trim.face = first_branch.faces()[1 - trim_operand].clone();
        assert!(bounded_skew_persistence_input(store, first_branch, &mismatched_trim).is_none());

        let mut mismatched_bound = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut mismatched_bound.span
        else {
            unreachable!()
        };
        endpoints[0].trim.authored_bound = endpoints[0].trim.authored_bound.next_up();
        assert!(bounded_skew_persistence_input(store, first_branch, &mismatched_bound).is_none());

        let mut mismatched_boundary = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } =
            &mut mismatched_boundary.span
        else {
            unreachable!()
        };
        endpoints[0].trim.axial_boundary = match endpoints[0].trim.axial_boundary {
            SectionSkewCylinderAxialBoundary::Lower => SectionSkewCylinderAxialBoundary::Upper,
            SectionSkewCylinderAxialBoundary::Upper => SectionSkewCylinderAxialBoundary::Lower,
        };
        assert!(
            bounded_skew_persistence_input(store, first_branch, &mismatched_boundary).is_none()
        );

        let mut mismatched_source = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = &mut mismatched_source.span
        else {
            unreachable!()
        };
        endpoints[0].trim.operand = 1 - endpoints[0].trim.operand;
        assert!(bounded_skew_persistence_input(store, first_branch, &mismatched_source).is_none());

        let mut mismatched_projective_root = first_fragment.clone();
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } =
            &mut mismatched_projective_root.span
        else {
            unreachable!()
        };
        endpoints[0].trim.carrier_root.lo = endpoints[0].trim.carrier_root.lo.next_down();
        assert!(
            bounded_skew_persistence_input(store, first_branch, &mismatched_projective_root)
                .is_none(),
            "Section projective-root identity must match the sealed family member"
        );
    }
}
