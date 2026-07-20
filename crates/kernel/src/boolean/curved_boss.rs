//! Selected-boundary adapter for one capped cylindrical boss.
//!
//! This module recognizes a complete convex-planar source boundary with one
//! circular port replaced by a preserved outward cylinder band and its source
//! cap. It translates that proof-selected boundary into the topology layer's
//! semantic boss input; it does not inspect the Boolean operation or primitive
//! constructor layout.

use std::collections::BTreeMap;

use kgeom::param::ParamRange;
use ktopo::cylindrical_boss::CylindricalBossSolidInput;
use ktopo::entity::EntityRef;
use ktopo::planar::{
    PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey,
};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_pipeline::{CertifiedRingCut, CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::face_partition::{AxialBoundary, FaceRegionKey};
use super::planar_bsp::{PlaneTripleVertexKey, SourcePlaneRef};

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Semantic proposal plus exact source-size accounting inputs.
#[derive(Debug, Clone)]
pub(super) struct PreparedCylindricalBoss {
    input: CylindricalBossSolidInput,
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
}

impl PreparedCylindricalBoss {
    pub(super) fn input(&self) -> &CylindricalBossSolidInput {
        &self.input
    }

    pub(super) fn into_input(self) -> CylindricalBossSolidInput {
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
pub(super) fn prepare_cylindrical_boss(
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: &[SelectedCurvedFragment],
) -> Result<Option<PreparedCylindricalBoss>, &'static str> {
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
    let mut side_seen = false;
    let mut cap_seen = false;
    for selected in selected {
        if selected.orientation() != SelectedOrientation::Preserved {
            return Ok(None);
        }
        match selected.fragment() {
            CurvedFragment::Planar {
                face,
                region: FaceRegionKey::PlanarOuter,
            } if selected.operand() == planar_side && expected_faces.contains(face) => {
                if planar_faces.contains(face) {
                    return Ok(None);
                }
                planar_faces.push(*face);
            }
            CurvedFragment::CylinderSide {
                region:
                    FaceRegionKey::AxialBand {
                        lower: AxialBoundary::Cut(key),
                        upper: AxialBoundary::UpperSource,
                    },
            } if selected.operand() == cylinder_side && *key == cut.key && !side_seen => {
                side_seen = true;
            }
            CurvedFragment::CylinderCap { face, boundary: 1 }
                if selected.operand() == cylinder_side
                    && *face == cylinder.boundaries()[1].cap_face()
                    && !cap_seen =>
            {
                cap_seen = true;
            }
            _ => return Ok(None),
        }
    }
    if planar_faces.len() != expected_faces.len()
        || expected_faces
            .iter()
            .any(|face| !planar_faces.contains(face))
        || !side_seen
        || !cap_seen
    {
        return Ok(None);
    }

    let (host, port_face) = prepare_host(planar, cut.planar_face)?;
    let upper = axial_parameter(cylinder, cylinder.boundaries()[1].center())
        .ok_or("cylindrical boss source cap has no finite axial parameter")?;
    if cut.axial_parameter >= upper {
        return Err("cylindrical boss cut must precede its retained source cap");
    }
    let host_vertices = host.vertices().len();
    let host_faces = host.faces().len();
    let host_face_uses = host.faces().iter().map(|face| face.vertices().len()).sum();
    Ok(Some(PreparedCylindricalBoss {
        input: CylindricalBossSolidInput::new(
            host,
            port_face,
            *cylinder.cylinder().frame(),
            cylinder.cylinder().radius(),
            ParamRange::new(cut.axial_parameter, upper),
        )
        .with_side_source(cylinder.side_face())
        .with_cap_source(cylinder.boundaries()[1].cap_face()),
        host_vertices,
        host_faces,
        host_face_uses,
    }))
}

fn common_planar_operand(planar: &ExtractedPlanarSourceBody) -> Result<u8, &'static str> {
    let Some(first) = planar.planes().first().map(|plane| plane.id().operand()) else {
        return Err("cylindrical boss host has no source planes");
    };
    if first > 1
        || planar
            .planes()
            .iter()
            .any(|plane| plane.id().operand() != first)
    {
        return Err("cylindrical boss host planes have inconsistent operand identities");
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
        let value = u64::try_from(index)
            .map_err(|_| "cylindrical boss host vertex identity exceeds u64")?;
        let key = PlanarVertexKey::new(value);
        if key_by_vertex.insert(vertex.key(), key).is_some() {
            return Err("cylindrical boss host repeats a source vertex identity");
        }
        vertices.push(PlanarSolidVertex::new(key, vertex.position()));
    }

    let registry = source
        .faces()
        .iter()
        .map(|face| (face.plane(), (face.face().raw(), face.surface())))
        .collect::<BTreeMap<_, _>>();
    if registry.len() != source.faces().len() {
        return Err("cylindrical boss host repeats a source face identity");
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
                    .ok_or("cylindrical boss face references an unknown source vertex")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let carriers = fragment
            .edge_planes()
            .iter()
            .map(|plane| {
                registry
                    .get(plane)
                    .map(|(_, surface)| *surface)
                    .ok_or("cylindrical boss edge references an unknown source plane")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let face_index = faces.len();
        faces.push(
            PlanarSolidFace::new(ring)
                .with_source(EntityRef::Face(source_face.face().raw()))
                .with_plane_binding(PlanarFacePlaneBinding::new(source_face.surface(), carriers)),
        );
        if source_face.face().raw() == port_source && port_face.replace(face_index).is_some() {
            return Err("cylindrical boss cut matches more than one host face");
        }
    }
    let port_face = port_face.ok_or("cylindrical boss cut has no host face")?;
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
        .ok_or("cylindrical boss host face has no source fragment")?;
    if matches.next().is_some() {
        return Err("cylindrical boss host face has multiple source fragments");
    }
    Ok(fragment)
}

fn axial_parameter(source: &CertifiedCylinderSource, point: kgeom::vec::Point3) -> Option<f64> {
    let cylinder = source.cylinder();
    let frame = cylinder.frame();
    let parameter = (point - frame.origin()).dot(frame.z());
    parameter.is_finite().then_some(parameter)
}
