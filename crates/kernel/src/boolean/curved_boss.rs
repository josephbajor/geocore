//! Selected-boundary adapter for one capped cylindrical port feature.
//!
//! This module recognizes a complete convex-planar source boundary with one
//! circular port joined to one cut-adjacent cylinder band and its source cap.
//! Exact source-plane orientation decides whether the selected boundary is an
//! outward boss or an inward blind pocket. The adapter translates that proof-
//! selected boundary into the topology layer's neutral semantic input; it does
//! not inspect the Boolean operation or primitive constructor layout.

use std::collections::BTreeMap;

use kcore::predicates::{Orientation, orient3d};
use kgeom::param::ParamRange;
use ktopo::cylindrical_boss::CappedCylinderSolidInput;
use ktopo::entity::EntityRef;
use ktopo::planar::{
    PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey,
};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_pipeline::{CertifiedRingCut, CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::face_partition::{AxialBoundary, FaceRegionKey};
use super::planar_bsp::{PlaneTripleVertexKey, SourcePlane, SourcePlaneRef};

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Semantic proposal plus exact source-size accounting inputs.
#[derive(Debug, Clone)]
pub(super) struct PreparedCappedCylinder {
    input: CappedCylinderSolidInput,
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
}

impl PreparedCappedCylinder {
    pub(super) fn into_input(self) -> CappedCylinderSolidInput {
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
}

/// Recognize and prepare one one-port connected curved boundary.
///
/// `Ok(None)` means the selected truth belongs to another topology class.
/// `Err` is reserved for an inconsistency in already-certified source data.
pub(super) fn prepare_capped_cylinder(
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: &[SelectedCurvedFragment],
) -> Result<Option<PreparedCappedCylinder>, &'static str> {
    let [cut] = cuts else {
        return Ok(None);
    };
    let planar_operand = common_planar_operand(planar)?;
    let planar_side = operand_side(planar_operand);
    let cylinder_side = operand_side(planar_operand ^ 1);
    if selected.len() != planar.faces().len().saturating_add(2) {
        return Ok(None);
    }

    let expected_faces = planar
        .faces()
        .iter()
        .map(|face| face.face().raw())
        .collect::<Vec<_>>();
    let mut planar_faces = Vec::new();
    let mut retained_boundary = None;
    let mut side_orientation = None;
    let mut cap_orientation = None;
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
            } if selected.operand() == cylinder_side && retained_boundary.is_none() => {
                let Some(boundary) = cut_adjacent_source_boundary(lower, upper, cut.key) else {
                    return Ok(None);
                };
                retained_boundary = Some(boundary);
                side_orientation = Some(selected.orientation());
            }
            CurvedFragment::CylinderCap { face, boundary }
                if selected.operand() == cylinder_side
                    && *boundary < cylinder.boundaries().len()
                    && *face == cylinder.boundaries()[*boundary].cap_face()
                    && cap_orientation.is_none() =>
            {
                cap_orientation = Some((*boundary, selected.orientation()));
            }
            _ => return Ok(None),
        }
    }
    if planar_faces.len() != expected_faces.len()
        || expected_faces
            .iter()
            .any(|face| !planar_faces.contains(face))
    {
        return Ok(None);
    }
    let (Some(retained_boundary), Some(side_orientation), Some((cap_boundary, cap_orientation))) =
        (retained_boundary, side_orientation, cap_orientation)
    else {
        return Ok(None);
    };
    if retained_boundary != cap_boundary || side_orientation != cap_orientation {
        return Ok(None);
    }

    let cap = cylinder.boundaries()[retained_boundary];
    let expected_orientation = capped_feature_orientation(planar, cut.planar_face, cap.center())?;
    if side_orientation != expected_orientation {
        return Ok(None);
    }

    let (host, port_face) = prepare_host(planar, cut.planar_face)?;
    let cap_parameter = axial_parameter(cylinder, cap.center())
        .ok_or("capped cylinder source cap has no finite axial parameter")?;
    let axial_range = ordered_range(cut.axial_parameter, cap_parameter)?;
    let host_vertices = host.vertices().len();
    let host_faces = host.faces().len();
    let host_face_uses = host.faces().iter().map(|face| face.vertices().len()).sum();
    Ok(Some(PreparedCappedCylinder {
        input: CappedCylinderSolidInput::new(
            host,
            port_face,
            *cylinder.cylinder().frame(),
            cylinder.cylinder().radius(),
            axial_range,
        )
        .with_side_source(cylinder.side_face())
        .with_cap_source(cap.cap_face()),
        host_vertices,
        host_faces,
        host_face_uses,
    }))
}

fn cut_adjacent_source_boundary(
    lower: &AxialBoundary<usize>,
    upper: &AxialBoundary<usize>,
    cut: usize,
) -> Option<usize> {
    match (lower, upper) {
        (AxialBoundary::LowerSource, AxialBoundary::Cut(key)) if *key == cut => Some(0),
        (AxialBoundary::Cut(key), AxialBoundary::UpperSource) if *key == cut => Some(1),
        _ => None,
    }
}

fn capped_feature_orientation(
    planar: &ExtractedPlanarSourceBody,
    port_face: ktopo::entity::FaceId,
    cap_center: kgeom::vec::Point3,
) -> Result<SelectedOrientation, &'static str> {
    let source_face = planar
        .faces()
        .iter()
        .find(|face| face.face().raw() == port_face)
        .ok_or("capped cylinder port has no planar source face")?;
    let source_plane = planar
        .planes()
        .iter()
        .find(|plane| plane.id() == source_face.plane())
        .ok_or("capped cylinder port has no source-plane proof")?;
    orientation_for_source_plane(*source_plane, cap_center)
}

fn orientation_for_source_plane(
    source_plane: SourcePlane,
    cap_center: kgeom::vec::Point3,
) -> Result<SelectedOrientation, &'static str> {
    let points = source_plane.points();
    let side = orient3d(points[0], points[1], points[2], cap_center.to_array());
    match side {
        Orientation::Zero => Err("capped cylinder cap lies on its port support plane"),
        side if side == source_plane.interior_side() => Ok(SelectedOrientation::Reversed),
        _ => Ok(SelectedOrientation::Preserved),
    }
}

fn ordered_range(first: f64, second: f64) -> Result<ParamRange, &'static str> {
    if !first.is_finite() || !second.is_finite() || first == second {
        return Err("capped cylinder endpoints must be finite and distinct");
    }
    Ok(if first < second {
        ParamRange::new(first, second)
    } else {
        ParamRange::new(second, first)
    })
}

fn common_planar_operand(planar: &ExtractedPlanarSourceBody) -> Result<u8, &'static str> {
    let Some(first) = planar.planes().first().map(|plane| plane.id().operand()) else {
        return Err("capped cylinder host has no source planes");
    };
    if first > 1
        || planar
            .planes()
            .iter()
            .any(|plane| plane.id().operand() != first)
    {
        return Err("capped cylinder host planes have inconsistent operand identities");
    }
    Ok(first)
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

fn prepare_host(
    source: &ExtractedPlanarSourceBody,
    port_source: ktopo::entity::FaceId,
) -> Result<(PlanarSolidInput, usize), &'static str> {
    let mut key_by_vertex = BTreeMap::<PlaneTripleVertexKey, PlanarVertexKey>::new();
    let mut vertices = Vec::with_capacity(source.vertices().len());
    for (index, vertex) in source.vertices().iter().enumerate() {
        let value =
            u64::try_from(index).map_err(|_| "capped cylinder host vertex identity exceeds u64")?;
        let key = PlanarVertexKey::new(value);
        if key_by_vertex.insert(vertex.key(), key).is_some() {
            return Err("capped cylinder host repeats a source vertex identity");
        }
        vertices.push(PlanarSolidVertex::new(key, vertex.position()));
    }

    let registry = source
        .faces()
        .iter()
        .map(|face| (face.plane(), (face.face().raw(), face.surface())))
        .collect::<BTreeMap<_, _>>();
    if registry.len() != source.faces().len() {
        return Err("capped cylinder host repeats a source face identity");
    }

    let mut faces = Vec::with_capacity(source.faces().len());
    let mut port_face = None;
    for source_face in source.faces() {
        let fragment = unique_fragment(source, source_face.plane())?;
        let ring = fragment
            .vertices()
            .iter()
            .map(|key| {
                key_by_vertex
                    .get(key)
                    .copied()
                    .ok_or("capped cylinder face references an unknown source vertex")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let carriers = fragment
            .edge_planes()
            .iter()
            .map(|plane| {
                registry
                    .get(plane)
                    .map(|(_, surface)| *surface)
                    .ok_or("capped cylinder edge references an unknown source plane")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let face_index = faces.len();
        faces.push(
            PlanarSolidFace::new(ring)
                .with_source(EntityRef::Face(source_face.face().raw()))
                .with_plane_binding(PlanarFacePlaneBinding::new(source_face.surface(), carriers)),
        );
        if source_face.face().raw() == port_source && port_face.replace(face_index).is_some() {
            return Err("capped cylinder cut matches more than one host face");
        }
    }
    let port_face = port_face.ok_or("capped cylinder cut has no host face")?;
    Ok((PlanarSolidInput::new(vertices, faces), port_face))
}

fn unique_fragment(
    source: &ExtractedPlanarSourceBody,
    face: SourcePlaneRef,
) -> Result<&super::planar_bsp::ConvexPlanarFragment, &'static str> {
    let mut matches = source
        .fragments()
        .iter()
        .filter(|fragment| fragment.source_face() == face);
    let fragment = matches
        .next()
        .ok_or("capped cylinder host face has no source fragment")?;
    if matches.next().is_some() {
        return Err("capped cylinder host face has multiple source fragments");
    }
    Ok(fragment)
}

fn axial_parameter(source: &CertifiedCylinderSource, point: kgeom::vec::Point3) -> Option<f64> {
    let cylinder = source.cylinder();
    let frame = cylinder.frame();
    let parameter = (point - frame.origin()).dot(frame.z());
    parameter.is_finite().then_some(parameter)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capped_feature_accepts_either_cut_adjacent_source_endpoint() {
        assert_eq!(
            cut_adjacent_source_boundary(&AxialBoundary::LowerSource, &AxialBoundary::Cut(7), 7,),
            Some(0)
        );
        assert_eq!(
            cut_adjacent_source_boundary(&AxialBoundary::Cut(7), &AxialBoundary::UpperSource, 7,),
            Some(1)
        );
        assert_eq!(
            cut_adjacent_source_boundary(&AxialBoundary::LowerSource, &AxialBoundary::Cut(8), 7,),
            None
        );
        assert_eq!(
            cut_adjacent_source_boundary(&AxialBoundary::Cut(7), &AxialBoundary::Cut(8), 7,),
            None
        );
    }

    #[test]
    fn capped_feature_range_is_endpoint_order_independent() {
        assert_eq!(
            ordered_range(3.0, -2.0).unwrap(),
            ParamRange::new(-2.0, 3.0)
        );
        assert_eq!(
            ordered_range(-2.0, 3.0).unwrap(),
            ParamRange::new(-2.0, 3.0)
        );
        assert!(ordered_range(1.0, 1.0).is_err());
        assert!(ordered_range(f64::NAN, 1.0).is_err());
    }

    #[test]
    fn exact_host_halfspace_decides_feature_orientation() {
        let plane = SourcePlane::from_interior_sample(
            SourcePlaneRef::new(0, 0),
            [[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [1.0, 1.0, 0.0]],
            [0.0, 0.0, -1.0],
        )
        .unwrap();
        assert_eq!(
            orientation_for_source_plane(plane, kgeom::vec::Point3::new(0.0, 0.0, 1.0)),
            Ok(SelectedOrientation::Preserved)
        );
        assert_eq!(
            orientation_for_source_plane(plane, kgeom::vec::Point3::new(0.0, 0.0, -0.5)),
            Ok(SelectedOrientation::Reversed)
        );
        assert!(
            orientation_for_source_plane(plane, kgeom::vec::Point3::new(0.0, 0.0, 0.0)).is_err()
        );
    }
}
