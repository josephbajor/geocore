//! Shared convex-planar host preparation for connected curved results.
//!
//! Boolean result adapters name only proof-selected source port faces. This
//! module binds the already-certified planar source into one semantic solid
//! input and returns the matching face indices without exposing raw topology
//! to the lower assembler.

use std::collections::BTreeMap;

use ktopo::entity::{EntityRef, FaceId as RawFaceId};
use ktopo::planar::{
    PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey,
};

use super::extract::ExtractedPlanarSourceBody;
use super::planar_bsp::{PlaneTripleVertexKey, SourcePlane, SourcePlaneRef};

pub(super) fn source_operand(source: &ExtractedPlanarSourceBody) -> Result<u8, &'static str> {
    let Some(first) = source.planes().first().map(|plane| plane.id().operand()) else {
        return Err("curved result host has no source planes");
    };
    if first > 1
        || source
            .planes()
            .iter()
            .any(|plane| plane.id().operand() != first)
    {
        return Err("curved result host planes have inconsistent operand identities");
    }
    Ok(first)
}

pub(super) fn source_plane_for_face(
    source: &ExtractedPlanarSourceBody,
    face: RawFaceId,
) -> Result<SourcePlane, &'static str> {
    let source_face = source
        .faces()
        .iter()
        .find(|candidate| candidate.face().raw() == face)
        .ok_or("curved result port has no planar source face")?;
    source
        .planes()
        .iter()
        .copied()
        .find(|plane| plane.id() == source_face.plane())
        .ok_or("curved result port has no source-plane proof")
}

pub(super) fn prepare_curved_host(
    source: &ExtractedPlanarSourceBody,
    port_sources: &[RawFaceId],
) -> Result<(PlanarSolidInput, Vec<usize>), &'static str> {
    let mut expected_ports = Vec::with_capacity(port_sources.len());
    for &port in port_sources {
        if expected_ports.contains(&port) {
            return Err("curved result repeats a source port face");
        }
        expected_ports.push(port);
    }

    let mut key_by_vertex = BTreeMap::<PlaneTripleVertexKey, PlanarVertexKey>::new();
    let mut vertices = Vec::with_capacity(source.vertices().len());
    for (index, vertex) in source.vertices().iter().enumerate() {
        let value = u64::try_from(index).map_err(|_| "curved host vertex identity exceeds u64")?;
        let key = PlanarVertexKey::new(value);
        if key_by_vertex.insert(vertex.key(), key).is_some() {
            return Err("curved host repeats a source vertex identity");
        }
        vertices.push(PlanarSolidVertex::new(key, vertex.position()));
    }

    let registry = source
        .faces()
        .iter()
        .map(|face| (face.plane(), (face.face().raw(), face.surface())))
        .collect::<BTreeMap<_, _>>();
    if registry.len() != source.faces().len() {
        return Err("curved host repeats a source face identity");
    }

    let mut faces = Vec::with_capacity(source.faces().len());
    let mut port_by_source = Vec::with_capacity(expected_ports.len());
    for source_face in source.faces() {
        let fragment = unique_fragment(source, source_face.plane())?;
        let ring = fragment
            .vertices()
            .iter()
            .map(|key| {
                key_by_vertex
                    .get(key)
                    .copied()
                    .ok_or("curved host face references an unknown source vertex")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let carriers = fragment
            .edge_planes()
            .iter()
            .map(|plane| {
                registry
                    .get(plane)
                    .map(|(_, surface)| *surface)
                    .ok_or("curved host edge references an unknown source plane")
            })
            .collect::<Result<Vec<_>, _>>()?;
        let face_index = faces.len();
        faces.push(
            PlanarSolidFace::new(ring)
                .with_source(EntityRef::Face(source_face.face().raw()))
                .with_plane_binding(PlanarFacePlaneBinding::new(source_face.surface(), carriers)),
        );
        let raw = source_face.face().raw();
        if expected_ports.contains(&raw) {
            if port_by_source
                .iter()
                .any(|(candidate, _)| *candidate == raw)
            {
                return Err("curved result port matches more than one host face");
            }
            port_by_source.push((raw, face_index));
        }
    }

    let port_faces = port_sources
        .iter()
        .map(|source| {
            port_by_source
                .iter()
                .find_map(|(candidate, index)| (*candidate == *source).then_some(*index))
                .ok_or("curved result port has no host face")
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok((PlanarSolidInput::new(vertices, faces), port_faces))
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
        .ok_or("curved host face has no source fragment")?;
    if matches.next().is_some() {
        return Err("curved host face has multiple source fragments");
    }
    Ok(fragment)
}
