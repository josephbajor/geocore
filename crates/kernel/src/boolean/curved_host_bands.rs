//! Selected-boundary adapter for cylindrical bands attached to a planar host.
//!
//! The adapter consumes the selected boundary as an axial incidence graph. It
//! is deliberately independent of Boolean operation and primitive layout:
//! exact cut order canonicalizes the bands, every cut becomes one host port,
//! and every selected source cap closes one source endpoint. Port supports
//! derive the result orientation of each band before a semantic topology input
//! is prepared.

use std::collections::{BTreeMap, BTreeSet};

use kcore::predicates::{Orientation, orient3d};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use ktopo::cylindrical_host::{
    CylindricalHostBandInput, CylindricalHostEndpoint, CylindricalHostSolidInput,
    cylindrical_host_preflight_work,
};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_host::{prepare_curved_host, source_operand, source_plane_for_face};
use super::curved_pipeline::{CertifiedRingCut, CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::extract::CertifiedConvexPlanarSource;
use super::face_partition::{AxialBoundary, FaceRegionKey};
use super::planar_bsp::SourcePlane;

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Semantic connected-host proposal plus source-exact accounting inputs.
#[derive(Debug, Clone)]
pub(super) struct PreparedCylindricalHostBands {
    input: CylindricalHostSolidInput,
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
    semantic_preflight_work: u64,
}

impl PreparedCylindricalHostBands {
    pub(super) fn into_input(self) -> CylindricalHostSolidInput {
        self.input
    }

    pub(super) const fn host_vertices(&self) -> usize {
        self.host_vertices
    }

    pub(super) const fn host_faces(&self) -> usize {
        self.host_faces
    }

    pub(super) const fn host_face_uses(&self) -> usize {
        self.host_face_uses
    }

    pub(super) const fn semantic_preflight_work(&self) -> u64 {
        self.semantic_preflight_work
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct SelectedBand {
    lower: AxialBoundary<usize>,
    upper: AxialBoundary<usize>,
    orientation: SelectedOrientation,
}

#[derive(Debug, Clone, PartialEq)]
struct PreparedBand {
    endpoints: [AxialBoundary<usize>; 2],
    axial_range: ParamRange,
    orientation: SelectedOrientation,
}

#[derive(Debug, Clone, PartialEq)]
struct PreparedIncidenceGraph {
    cuts: Vec<usize>,
    bands: Vec<PreparedBand>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct AxialCutEvidence {
    key: usize,
    exact_order: usize,
    axial_parameter: f64,
}

/// Recognize all selected cylindrical bands connected to one convex host.
///
/// `Ok(None)` means the selected truth belongs to another topology class.
/// `Err` reports inconsistency in already-certified source evidence.
pub(super) fn prepare_cylindrical_host_bands(
    planar: &CertifiedConvexPlanarSource,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: &[SelectedCurvedFragment],
) -> Result<Option<PreparedCylindricalHostBands>, &'static str> {
    if cuts.is_empty() {
        return Ok(None);
    }
    let planar_operand = source_operand(planar)?;
    let planar_side = operand_side(planar_operand);
    let cylinder_side = operand_side(planar_operand ^ 1);
    let expected_faces = planar
        .faces()
        .iter()
        .map(|face| face.face().raw())
        .collect::<Vec<_>>();
    let mut planar_faces = Vec::with_capacity(expected_faces.len());
    let mut side_bands = Vec::new();
    let mut source_caps = [None; 2];
    for selected in selected {
        match selected.fragment() {
            CurvedFragment::Planar {
                face,
                region: FaceRegionKey::PlanarOuter,
            } if selected.operand() == planar_side
                && selected.orientation() == SelectedOrientation::Preserved
                && expected_faces.contains(face) =>
            {
                if planar_faces.contains(face) {
                    return Ok(None);
                }
                planar_faces.push(*face);
            }
            CurvedFragment::CylinderSide {
                region: FaceRegionKey::AxialBand { lower, upper },
            } if selected.operand() == cylinder_side => side_bands.push(SelectedBand {
                lower: lower.clone(),
                upper: upper.clone(),
                orientation: selected.orientation(),
            }),
            CurvedFragment::CylinderCap { face, boundary }
                if selected.operand() == cylinder_side
                    && *boundary < cylinder.boundaries().len()
                    && *face == cylinder.boundaries()[*boundary].cap_face() =>
            {
                if source_caps[*boundary]
                    .replace(selected.orientation())
                    .is_some()
                {
                    return Ok(None);
                }
            }
            _ => return Ok(None),
        }
    }
    if planar_faces.len() != expected_faces.len()
        || expected_faces
            .iter()
            .any(|face| !planar_faces.contains(face))
        || side_bands.is_empty()
    {
        return Ok(None);
    }

    let bounds = cylinder.boundaries();
    let source_parameters = [
        axial_parameter(cylinder, bounds[0].center())
            .ok_or("cylindrical host lower endpoint has no finite parameter")?,
        axial_parameter(cylinder, bounds[1].center())
            .ok_or("cylindrical host upper endpoint has no finite parameter")?,
    ];
    let cut_evidence = cuts
        .iter()
        .map(|cut| AxialCutEvidence {
            key: cut.key,
            exact_order: cut.exact_order,
            axial_parameter: cut.axial_parameter,
        })
        .collect::<Vec<_>>();
    let Some(graph) =
        prepare_incidence_graph(&cut_evidence, source_parameters, side_bands, source_caps)?
    else {
        return Ok(None);
    };
    for band in &graph.bands {
        if !validate_band_orientation(planar, cylinder, cuts, band)? {
            return Ok(None);
        }
    }

    let cut_by_key = cuts
        .iter()
        .map(|cut| (cut.key, cut))
        .collect::<BTreeMap<_, _>>();
    let port_sources = graph
        .cuts
        .iter()
        .map(|cut| {
            cut_by_key
                .get(cut)
                .map(|evidence| evidence.planar_face)
                .ok_or("cylindrical host graph references an unknown cut")
        })
        .collect::<Result<Vec<_>, _>>()?;
    let (host, port_faces) = prepare_curved_host(planar, &port_sources)?;
    if port_faces.len() != graph.cuts.len() {
        return Err("cylindrical host did not prepare every selected port");
    }
    let port_by_cut = graph
        .cuts
        .iter()
        .copied()
        .zip(port_faces.iter().copied())
        .collect::<BTreeMap<_, _>>();
    let cylinder_geom = cylinder.cylinder();
    let bands = graph
        .bands
        .into_iter()
        .map(|band| {
            let endpoints = band.endpoints.map(|endpoint| match endpoint {
                AxialBoundary::Cut(cut) => port_by_cut
                    .get(&cut)
                    .copied()
                    .map(CylindricalHostEndpoint::port)
                    .ok_or("cylindrical host band references an unknown port"),
                AxialBoundary::LowerSource => Ok(CylindricalHostEndpoint::cap_with_source(
                    bounds[0].cap_face(),
                )),
                AxialBoundary::UpperSource => Ok(CylindricalHostEndpoint::cap_with_source(
                    bounds[1].cap_face(),
                )),
            });
            let [low, high] = endpoints;
            Ok(CylindricalHostBandInput::new(
                *cylinder_geom.frame(),
                cylinder_geom.radius(),
                band.axial_range,
                [low?, high?],
            )
            .with_side_source(cylinder.side_face()))
        })
        .collect::<Result<Vec<_>, &'static str>>()?;

    let host_vertices = host.vertices().len();
    let host_faces = host.faces().len();
    let host_face_uses = host.faces().iter().map(|face| face.vertices().len()).sum();
    let input = CylindricalHostSolidInput::new(host, bands);
    let semantic_preflight_work = cylindrical_host_preflight_work(&input)
        .map_err(|_| "cylindrical host semantic work accounting failed")?
        .total();
    Ok(Some(PreparedCylindricalHostBands {
        input,
        host_vertices,
        host_faces,
        host_face_uses,
        semantic_preflight_work,
    }))
}

fn prepare_incidence_graph(
    cuts: &[AxialCutEvidence],
    source_parameters: [f64; 2],
    mut selected_bands: Vec<SelectedBand>,
    mut source_caps: [Option<SelectedOrientation>; 2],
) -> Result<Option<PreparedIncidenceGraph>, &'static str> {
    if !(source_parameters[0].is_finite()
        && source_parameters[1].is_finite()
        && source_parameters[0] < source_parameters[1])
    {
        return Err("cylindrical host source range must be finite and increasing");
    }
    let mut cuts_by_key = BTreeMap::new();
    let mut cuts_by_order = vec![None; cuts.len()];
    for cut in cuts {
        if !cut.axial_parameter.is_finite()
            || cut.exact_order >= cuts.len()
            || cuts_by_key
                .insert(cut.key, (cut.exact_order, cut.axial_parameter))
                .is_some()
            || cuts_by_order[cut.exact_order]
                .replace((cut.key, cut.axial_parameter))
                .is_some()
        {
            return Err("cylindrical host cuts have inconsistent exact identities");
        }
    }
    let cuts_by_order = cuts_by_order
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .ok_or("cylindrical host cuts have incomplete exact order")?;
    let mut previous = source_parameters[0];
    for (_, parameter) in &cuts_by_order {
        if previous >= *parameter || *parameter >= source_parameters[1] {
            return Err("cylindrical host cuts do not increase inside the source range");
        }
        previous = *parameter;
    }

    selected_bands.sort_by_key(|band| {
        (
            boundary_rank(&band.lower, &cuts_by_key),
            boundary_rank(&band.upper, &cuts_by_key),
        )
    });
    let mut endpoint_use = BTreeSet::new();
    let mut prepared = Vec::with_capacity(selected_bands.len());
    for band in selected_bands {
        let Some(low_rank) = boundary_rank(&band.lower, &cuts_by_key) else {
            return Ok(None);
        };
        let Some(high_rank) = boundary_rank(&band.upper, &cuts_by_key) else {
            return Ok(None);
        };
        if low_rank.checked_add(1) != Some(high_rank)
            || (!matches!(band.lower, AxialBoundary::Cut(_))
                && !matches!(band.upper, AxialBoundary::Cut(_)))
            || !endpoint_use.insert(low_rank)
            || !endpoint_use.insert(high_rank)
        {
            return Ok(None);
        }
        for endpoint in [&band.lower, &band.upper] {
            match endpoint {
                AxialBoundary::LowerSource => {
                    if source_caps[0].take() != Some(band.orientation) {
                        return Ok(None);
                    }
                }
                AxialBoundary::UpperSource => {
                    if source_caps[1].take() != Some(band.orientation) {
                        return Ok(None);
                    }
                }
                AxialBoundary::Cut(_) => {}
            }
        }
        let low = boundary_parameter(&band.lower, source_parameters, &cuts_by_key)
            .ok_or("cylindrical host band has no lower parameter")?;
        let high = boundary_parameter(&band.upper, source_parameters, &cuts_by_key)
            .ok_or("cylindrical host band has no upper parameter")?;
        if !(low.is_finite() && high.is_finite() && low < high) {
            return Err("cylindrical host band range must be finite and increasing");
        }
        prepared.push(PreparedBand {
            endpoints: [band.lower, band.upper],
            axial_range: ParamRange::new(low, high),
            orientation: band.orientation,
        });
    }
    if source_caps.into_iter().any(|cap| cap.is_some())
        || cuts_by_order
            .iter()
            .enumerate()
            .any(|(order, _)| !endpoint_use.contains(&(order + 1)))
    {
        return Ok(None);
    }
    Ok(Some(PreparedIncidenceGraph {
        cuts: cuts_by_order.into_iter().map(|(key, _)| key).collect(),
        bands: prepared,
    }))
}

fn validate_band_orientation(
    planar: &CertifiedConvexPlanarSource,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    band: &PreparedBand,
) -> Result<bool, &'static str> {
    let endpoint_points = band
        .endpoints
        .each_ref()
        .map(|endpoint| boundary_point(endpoint, cylinder, cuts));
    let [low_point, high_point] = endpoint_points;
    let low_point = low_point?;
    let high_point = high_point?;
    let mut derived = None;
    for (endpoint, (point, other)) in band
        .endpoints
        .iter()
        .zip([(low_point, high_point), (high_point, low_point)])
    {
        let AxialBoundary::Cut(key) = endpoint else {
            continue;
        };
        let cut = cuts
            .iter()
            .find(|cut| cut.key == *key)
            .ok_or("cylindrical host band references an unknown cut")?;
        let plane = source_plane_for_face(planar, cut.planar_face)?;
        if point_side(plane, point) != Orientation::Zero {
            return Err("cylindrical host port center is not incident with its support plane");
        }
        let orientation = orientation_from_port(plane, other)?;
        if derived
            .replace(orientation)
            .is_some_and(|prior| prior != orientation)
        {
            return Ok(false);
        }
    }
    Ok(derived == Some(band.orientation))
}

fn boundary_point(
    boundary: &AxialBoundary<usize>,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
) -> Result<Point3, &'static str> {
    match boundary {
        AxialBoundary::LowerSource => Ok(cylinder.boundaries()[0].center()),
        AxialBoundary::UpperSource => Ok(cylinder.boundaries()[1].center()),
        AxialBoundary::Cut(key) => cuts
            .iter()
            .find(|cut| cut.key == *key)
            .map(|cut| cut.center)
            .ok_or("cylindrical host endpoint references an unknown cut"),
    }
}

fn orientation_from_port(
    plane: SourcePlane,
    opposite_endpoint: Point3,
) -> Result<SelectedOrientation, &'static str> {
    match point_side(plane, opposite_endpoint) {
        Orientation::Zero => Err("cylindrical host band lies in its port support plane"),
        side if side == plane.interior_side() => Ok(SelectedOrientation::Reversed),
        _ => Ok(SelectedOrientation::Preserved),
    }
}

fn point_side(plane: SourcePlane, point: Point3) -> Orientation {
    let witness = plane.points();
    orient3d(witness[0], witness[1], witness[2], point.to_array())
}

fn boundary_rank(
    boundary: &AxialBoundary<usize>,
    cuts: &BTreeMap<usize, (usize, f64)>,
) -> Option<usize> {
    match boundary {
        AxialBoundary::LowerSource => Some(0),
        AxialBoundary::Cut(cut) => cuts.get(cut).and_then(|(order, _)| order.checked_add(1)),
        AxialBoundary::UpperSource => cuts.len().checked_add(1),
    }
}

fn boundary_parameter(
    boundary: &AxialBoundary<usize>,
    source: [f64; 2],
    cuts: &BTreeMap<usize, (usize, f64)>,
) -> Option<f64> {
    match boundary {
        AxialBoundary::LowerSource => Some(source[0]),
        AxialBoundary::Cut(cut) => cuts.get(cut).map(|(_, parameter)| *parameter),
        AxialBoundary::UpperSource => Some(source[1]),
    }
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

fn axial_parameter(source: &CertifiedCylinderSource, point: Point3) -> Option<f64> {
    let cylinder = source.cylinder();
    let frame = cylinder.frame();
    let parameter = (point - frame.origin()).dot(frame.z());
    parameter.is_finite().then_some(parameter)
}

#[cfg(test)]
mod tests {
    use super::super::planar_bsp::SourcePlaneRef;
    use super::*;
    use kgeom::frame::Frame;
    use ktopo::check::CheckOutcome;
    use ktopo::entity::RegionKind;

    use crate::{BlockRequest, BodyId, CylinderRequest, Kernel};

    fn cut(key: usize, exact_order: usize, axial_parameter: f64) -> AxialCutEvidence {
        AxialCutEvidence {
            key,
            axial_parameter,
            exact_order,
        }
    }

    fn band(
        lower: AxialBoundary<usize>,
        upper: AxialBoundary<usize>,
        orientation: SelectedOrientation,
    ) -> SelectedBand {
        SelectedBand {
            lower,
            upper,
            orientation,
        }
    }

    fn port_plane(z: f64, interior_z: f64) -> SourcePlane {
        SourcePlane::from_interior_sample(
            SourcePlaneRef::new(0, 0),
            [[-1.0, -1.0, z], [1.0, -1.0, z], [1.0, 1.0, z]],
            [0.0, 0.0, interior_z],
        )
        .unwrap()
    }

    #[test]
    fn port_support_derives_outward_and_inward_band_orientations() {
        let plane = port_plane(0.0, -1.0);
        assert_eq!(
            orientation_from_port(plane, Point3::new(0.0, 0.0, 1.0)),
            Ok(SelectedOrientation::Preserved)
        );
        assert_eq!(
            orientation_from_port(plane, Point3::new(0.0, 0.0, -0.5)),
            Ok(SelectedOrientation::Reversed)
        );
        assert!(orientation_from_port(plane, Point3::new(0.0, 0.0, 0.0)).is_err());
    }

    #[test]
    fn two_ended_graph_is_exact_and_permutation_independent() {
        let cuts = [cut(22, 1, 1.5), cut(11, 0, 0.5)];
        let bands = vec![
            band(
                AxialBoundary::Cut(22),
                AxialBoundary::UpperSource,
                SelectedOrientation::Preserved,
            ),
            band(
                AxialBoundary::LowerSource,
                AxialBoundary::Cut(11),
                SelectedOrientation::Preserved,
            ),
        ];
        let graph = prepare_incidence_graph(
            &cuts,
            [0.0, 2.0],
            bands,
            [
                Some(SelectedOrientation::Preserved),
                Some(SelectedOrientation::Preserved),
            ],
        )
        .unwrap()
        .unwrap();
        assert_eq!(graph.cuts, vec![11, 22]);
        assert_eq!(graph.bands.len(), 2);
        assert_eq!(
            graph.bands[0].endpoints,
            [AxialBoundary::LowerSource, AxialBoundary::Cut(11)]
        );
        assert_eq!(graph.bands[0].axial_range, ParamRange::new(0.0, 0.5));
        assert_eq!(
            graph.bands[1].endpoints,
            [AxialBoundary::Cut(22), AxialBoundary::UpperSource]
        );
        assert_eq!(graph.bands[1].axial_range, ParamRange::new(1.5, 2.0));

        let permuted = prepare_incidence_graph(
            &[cut(11, 0, 0.5), cut(22, 1, 1.5)],
            [0.0, 2.0],
            graph
                .bands
                .iter()
                .rev()
                .map(|prepared| SelectedBand {
                    lower: prepared.endpoints[0].clone(),
                    upper: prepared.endpoints[1].clone(),
                    orientation: prepared.orientation,
                })
                .collect(),
            [
                Some(SelectedOrientation::Preserved),
                Some(SelectedOrientation::Preserved),
            ],
        )
        .unwrap()
        .unwrap();
        assert_eq!(permuted, graph);
    }

    #[test]
    fn one_port_and_two_port_graphs_share_the_same_incidence_seam() {
        let one_port = prepare_incidence_graph(
            &[cut(7, 0, 1.0)],
            [0.0, 2.0],
            vec![band(
                AxialBoundary::LowerSource,
                AxialBoundary::Cut(7),
                SelectedOrientation::Preserved,
            )],
            [Some(SelectedOrientation::Preserved), None],
        )
        .unwrap()
        .unwrap();
        assert_eq!(one_port.cuts, vec![7]);
        assert_eq!(one_port.bands.len(), 1);

        let two_port = prepare_incidence_graph(
            &[cut(19, 1, 1.5), cut(13, 0, 0.5)],
            [0.0, 2.0],
            vec![band(
                AxialBoundary::Cut(13),
                AxialBoundary::Cut(19),
                SelectedOrientation::Reversed,
            )],
            [None, None],
        )
        .unwrap()
        .unwrap();
        assert_eq!(two_port.cuts, vec![13, 19]);
        assert_eq!(two_port.bands.len(), 1);
        assert_eq!(two_port.bands[0].orientation, SelectedOrientation::Reversed);
    }

    #[test]
    fn incidence_graph_refuses_reused_or_unconsumed_endpoints() {
        let cuts = [cut(5, 0, 0.5), cut(9, 1, 1.5)];
        let reused_cut = prepare_incidence_graph(
            &cuts,
            [0.0, 2.0],
            vec![
                band(
                    AxialBoundary::LowerSource,
                    AxialBoundary::Cut(5),
                    SelectedOrientation::Preserved,
                ),
                band(
                    AxialBoundary::Cut(5),
                    AxialBoundary::Cut(9),
                    SelectedOrientation::Reversed,
                ),
            ],
            [Some(SelectedOrientation::Preserved), None],
        )
        .unwrap();
        assert_eq!(reused_cut, None);

        let unconsumed_cut = prepare_incidence_graph(
            &cuts,
            [0.0, 2.0],
            vec![band(
                AxialBoundary::LowerSource,
                AxialBoundary::Cut(5),
                SelectedOrientation::Preserved,
            )],
            [Some(SelectedOrientation::Preserved), None],
        )
        .unwrap();
        assert_eq!(unconsumed_cut, None);

        let orphan_cap = prepare_incidence_graph(
            &[cut(5, 0, 0.5)],
            [0.0, 2.0],
            vec![band(
                AxialBoundary::Cut(5),
                AxialBoundary::UpperSource,
                SelectedOrientation::Preserved,
            )],
            [
                Some(SelectedOrientation::Preserved),
                Some(SelectedOrientation::Preserved),
            ],
        )
        .unwrap();
        assert_eq!(orphan_cap, None);
    }

    #[test]
    fn incidence_graph_refuses_nonconsecutive_or_misoriented_caps() {
        let cuts = [cut(5, 0, 0.5), cut(9, 1, 1.5)];
        let skipped_boundary = prepare_incidence_graph(
            &cuts,
            [0.0, 2.0],
            vec![band(
                AxialBoundary::LowerSource,
                AxialBoundary::Cut(9),
                SelectedOrientation::Preserved,
            )],
            [Some(SelectedOrientation::Preserved), None],
        )
        .unwrap();
        assert_eq!(skipped_boundary, None);

        let wrong_cap_orientation = prepare_incidence_graph(
            &[cut(5, 0, 0.5)],
            [0.0, 2.0],
            vec![band(
                AxialBoundary::LowerSource,
                AxialBoundary::Cut(5),
                SelectedOrientation::Preserved,
            )],
            [Some(SelectedOrientation::Reversed), None],
        )
        .unwrap();
        assert_eq!(wrong_cap_orientation, None);
    }

    #[test]
    fn incidence_graph_rejects_invalid_certified_cut_order() {
        let error = prepare_incidence_graph(
            &[cut(5, 0, 0.5), cut(9, 0, 1.5)],
            [0.0, 2.0],
            Vec::new(),
            [None, None],
        )
        .unwrap_err();
        assert_eq!(
            error,
            "cylindrical host cuts have inconsistent exact identities"
        );
    }

    fn reverse_body_face_storage(edit: &mut crate::session::PartEdit<'_>, body: &BodyId) {
        let store = edit.store_mut_for_test();
        let material = store
            .get(body.raw())
            .unwrap()
            .regions()
            .iter()
            .copied()
            .find(|region| store.get(*region).unwrap().kind() == RegionKind::Solid)
            .unwrap();
        let shell = store.get(material).unwrap().shells()[0];
        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .get_mut(shell)
            .unwrap()
            .faces
            .reverse();
        transaction.commit_checked_body(body.raw()).unwrap();
    }

    #[test]
    fn two_ended_result_ignores_operand_and_face_storage_order() {
        for reverse_storage in [false, true] {
            for cylinder_first in [false, true] {
                let mut session = Kernel::new().create_session();
                let part = session.create_part();
                let (block, cylinder) = {
                    let mut edit = session.edit_part(part.clone()).unwrap();
                    let block = edit
                        .create_block(BlockRequest::new(
                            Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
                            [4.0, 4.0, 1.0],
                        ))
                        .unwrap()
                        .into_result()
                        .unwrap()
                        .body();
                    let cylinder = edit
                        .create_cylinder(CylinderRequest::new(Frame::world(), 0.75, 2.0))
                        .unwrap()
                        .into_result()
                        .unwrap()
                        .body();
                    if reverse_storage {
                        reverse_body_face_storage(&mut edit, &block);
                        reverse_body_face_storage(&mut edit, &cylinder);
                    }
                    (block, cylinder)
                };
                let (left, right) = if cylinder_first {
                    (cylinder, block)
                } else {
                    (block, cylinder)
                };
                let outcome = super::super::dispatch::execute_boolean(
                    &mut session.edit_part(part.clone()).unwrap(),
                    super::super::select::PlanarBooleanOperation::Unite,
                    left,
                    right,
                    crate::OperationSettings::new(),
                )
                .unwrap()
                .into_result()
                .unwrap();
                let super::super::dispatch::BooleanPipelineOutcome::Curved(
                    super::super::curved_pipeline::CurvedBooleanPipelineOutcome::Committed(
                        committed,
                    ),
                ) = outcome
                else {
                    panic!("expected committed two-ended union, got {outcome:?}")
                };
                let (bodies, _, full_checks) = committed.into_parts();
                assert_eq!(bodies.len(), 1);
                assert!(
                    full_checks
                        .iter()
                        .all(|check| check.report().outcome() == CheckOutcome::Valid)
                );
                let view = session.part(part).unwrap();
                let body = view.body(bodies[0].clone()).unwrap();
                assert_eq!(body.faces().unwrap().len(), 10);
                assert_eq!(body.edges().unwrap().len(), 16);
                assert_eq!(body.vertices().unwrap().len(), 8);
            }
        }
    }
}
