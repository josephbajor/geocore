//! Read-only authority for whole-fin edge/surface incidence.
//!
//! Higher kernel layers use this narrow seam instead of reconstructing
//! topology ownership or promoting sampled geometric agreement to proof.
//! Unsupported representations and every structural or geometric mismatch
//! fail closed without exposing checker-internal diagnostic categories.

use crate::entity::{Edge, FaceId, FinId, LoopId, Sense};
use crate::incidence::{
    IncidenceCertification, certify_edge_surface_incidence, certify_pcurve_incidence,
    check_pcurve_incidence, check_pcurve_metadata,
};
use crate::store::Store;

/// Result of topology-owned whole-fin incidence validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WholeFinIncidence {
    /// Ownership, manifold use, metadata, and both whole-interval incidence
    /// obligations are certified.
    Certified,
    /// The tuple is invalid or not covered by the current proof machinery.
    Indeterminate,
}

/// Certify one fin's entire 3D edge trace and pcurve lift on its owning face.
///
/// Certification requires exact face/loop/fin ownership backlinks, a
/// two-fin manifold edge with opposed traversal, valid pcurve metadata and
/// parameter correspondence, and both topology incidence certificates.
/// Deterministic incidence samples remain a rejection filter only; they are
/// never promoted to proof.
pub fn certify_whole_fin_incidence(
    store: &Store,
    face_id: FaceId,
    loop_id: LoopId,
    fin_id: FinId,
    tolerance: f64,
) -> WholeFinIncidence {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return WholeFinIncidence::Indeterminate;
    }

    let Ok(face) = store.get(face_id) else {
        return WholeFinIncidence::Indeterminate;
    };
    let Ok(loop_) = store.get(loop_id) else {
        return WholeFinIncidence::Indeterminate;
    };
    let Ok(fin) = store.get(fin_id) else {
        return WholeFinIncidence::Indeterminate;
    };
    if loop_.face != face_id
        || fin.parent != loop_id
        || !exactly_once(&face.loops, loop_id)
        || !exactly_once(&loop_.fins, fin_id)
        || !face_owned_once(store, face_id)
    {
        return WholeFinIncidence::Indeterminate;
    }

    let edge_id = fin.edge;
    let Ok(edge) = store.get(edge_id) else {
        return WholeFinIncidence::Indeterminate;
    };
    if !valid_manifold_use(store, face.shell, edge_id, edge) || !exactly_once(&edge.fins, fin_id) {
        return WholeFinIncidence::Indeterminate;
    }
    let Some(curve_id) = edge.curve else {
        return WholeFinIncidence::Indeterminate;
    };
    if !valid_edge_domain(store, edge) {
        return WholeFinIncidence::Indeterminate;
    }
    let Some(pcurve) = fin.pcurve else {
        return WholeFinIncidence::Indeterminate;
    };

    if check_pcurve_metadata(store, edge, face.surface, face.domain, pcurve).is_err()
        || check_pcurve_incidence(
            store,
            curve_id,
            edge.bounds,
            face.surface,
            pcurve,
            tolerance,
        )
        .is_err()
    {
        return WholeFinIncidence::Indeterminate;
    }

    if certify_edge_surface_incidence(store, edge_id, face.surface, tolerance).ok()
        != Some(IncidenceCertification::Certified)
        || certify_pcurve_incidence(store, edge_id, face.surface, pcurve, tolerance).ok()
            != Some(IncidenceCertification::Certified)
    {
        return WholeFinIncidence::Indeterminate;
    }

    WholeFinIncidence::Certified
}

fn exactly_once<T: Copy + PartialEq>(items: &[T], expected: T) -> bool {
    items.iter().filter(|&&item| item == expected).count() == 1
}

fn face_owned_once(store: &Store, face_id: FaceId) -> bool {
    store
        .get(face_id)
        .ok()
        .and_then(|face| store.get(face.shell).ok())
        .is_some_and(|shell| exactly_once(&shell.faces, face_id))
}

fn valid_manifold_use(
    store: &Store,
    shell_id: crate::entity::ShellId,
    edge_id: crate::entity::EdgeId,
    edge: &Edge,
) -> bool {
    if edge.fins.len() != 2 || edge.fins[0] == edge.fins[1] {
        return false;
    }
    let mut senses = [Sense::Forward; 2];
    for (index, &fin_id) in edge.fins.iter().enumerate() {
        let Ok(fin) = store.get(fin_id) else {
            return false;
        };
        let Ok(loop_) = store.get(fin.parent) else {
            return false;
        };
        let face_id = loop_.face;
        let Ok(face) = store.get(face_id) else {
            return false;
        };
        if fin.edge != edge_id
            || face.shell != shell_id
            || !exactly_once(&loop_.fins, fin_id)
            || !exactly_once(&face.loops, fin.parent)
            || !face_owned_once(store, face_id)
        {
            return false;
        }
        senses[index] = fin.sense;
    }
    senses[0] != senses[1]
}

fn valid_edge_domain(store: &Store, edge: &Edge) -> bool {
    match (edge.bounds, edge.vertices) {
        (None, [None, None]) => edge
            .curve
            .and_then(|curve| store.get(curve).ok())
            .is_some_and(|curve| curve.as_curve().periodicity().is_some()),
        (Some((lo, hi)), [Some(_), Some(_)]) => lo.is_finite() && hi.is_finite() && lo < hi,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{FinPcurve, ParamMap1d, PcurveChart};
    use crate::geom::Curve2dGeom;
    use crate::make;
    use kgeom::curve2d::Circle2d;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point2, Vec2};

    const TOLERANCE: f64 = 1.0e-9;

    fn ring_uses(store: &Store, body: crate::entity::BodyId) -> Vec<(FaceId, LoopId, FinId)> {
        store
            .edges_of_body(body)
            .unwrap()
            .into_iter()
            .flat_map(|edge| store.get(edge).unwrap().fins.clone())
            .map(|fin| {
                let loop_ = store.get(fin).unwrap().parent;
                (store.get(loop_).unwrap().face, loop_, fin)
            })
            .collect()
    }

    #[test]
    fn cylinder_ring_uses_are_whole_interval_certified() {
        let mut store = Store::new();
        let body = make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();

        for (face, loop_, fin) in ring_uses(&store, body) {
            assert_eq!(
                certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
                WholeFinIncidence::Certified
            );
        }
    }

    #[test]
    fn mismatched_circle_pcurve_is_indeterminate() {
        let mut store = Store::new();
        let body = make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let (face, loop_, fin) = ring_uses(&store, body)
            .into_iter()
            .find(|&(face, _, _)| {
                matches!(
                    store.get(store.get(face).unwrap().surface).unwrap(),
                    crate::geom::SurfaceGeom::Plane(_)
                )
            })
            .unwrap();
        let old = store.get(fin).unwrap().pcurve.unwrap();
        let wrong = store
            .insert_pcurve(Curve2dGeom::Circle(
                Circle2d::new(Point2::new(0.0, 0.0), 2.25, Vec2::new(1.0, 0.0)).unwrap(),
            ))
            .unwrap();
        store.get_mut(fin).unwrap().pcurve = Some(
            FinPcurve::new(wrong, old.range(), old.edge_to_pcurve())
                .unwrap()
                .with_closure_winding([0, 0]),
        );

        assert_eq!(
            certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
            WholeFinIncidence::Indeterminate
        );
    }

    #[test]
    fn nonperiodic_plane_chart_shift_is_indeterminate() {
        let mut store = Store::new();
        let body = make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let (face, loop_, fin) = ring_uses(&store, body)
            .into_iter()
            .find(|&(face, _, _)| {
                matches!(
                    store.get(store.get(face).unwrap().surface).unwrap(),
                    crate::geom::SurfaceGeom::Plane(_)
                )
            })
            .unwrap();
        let old = store.get(fin).unwrap().pcurve.unwrap();
        store.get_mut(fin).unwrap().pcurve = Some(old.with_chart(PcurveChart::shifted([1, 0])));

        assert_eq!(
            certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
            WholeFinIncidence::Indeterminate
        );
    }

    #[test]
    fn inconsistent_winding_and_parameter_map_are_indeterminate() {
        let mut store = Store::new();
        let body = make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let (face, loop_, fin) = ring_uses(&store, body)
            .into_iter()
            .find(|&(face, _, _)| {
                matches!(
                    store.get(store.get(face).unwrap().surface).unwrap(),
                    crate::geom::SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let old = store.get(fin).unwrap().pcurve.unwrap();
        store.get_mut(fin).unwrap().pcurve = Some(old.with_closure_winding([0, 0]));
        assert_eq!(
            certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
            WholeFinIncidence::Indeterminate
        );

        let shifted_map = ParamMap1d::affine(
            old.edge_to_pcurve().scale(),
            old.edge_to_pcurve().offset() + 0.25,
        )
        .unwrap();
        store.get_mut(fin).unwrap().pcurve = Some(
            FinPcurve::new(old.curve(), old.range(), shifted_map)
                .unwrap()
                .with_closure_winding([1, 0]),
        );
        assert_eq!(
            certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
            WholeFinIncidence::Indeterminate
        );
    }

    #[test]
    fn duplicate_manifold_fin_backlink_is_indeterminate() {
        let mut store = Store::new();
        let body = make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let (face, loop_, fin) = ring_uses(&store, body)[0];
        let edge = store.get(fin).unwrap().edge;
        store.get_mut(edge).unwrap().fins = vec![fin, fin];

        assert_eq!(
            certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
            WholeFinIncidence::Indeterminate
        );
    }

    #[test]
    fn manifold_peer_face_in_another_shell_is_indeterminate() {
        let mut store = Store::new();
        let body = make::cylinder(&mut store, &Frame::world(), 2.0, 3.0).unwrap();
        let foreign_body = make::cylinder(&mut store, &Frame::world(), 4.0, 5.0).unwrap();
        let (face, loop_, fin) = ring_uses(&store, body)[0];
        let edge = store.get(fin).unwrap().edge;
        let peer_fin = store
            .get(edge)
            .unwrap()
            .fins
            .iter()
            .copied()
            .find(|&candidate| candidate != fin)
            .unwrap();
        let peer_loop = store.get(peer_fin).unwrap().parent;
        let peer_face = store.get(peer_loop).unwrap().face;
        let original_shell = store.get(peer_face).unwrap().shell;
        let foreign_face = store.faces_of_body(foreign_body).unwrap()[0];
        let foreign_shell = store.get(foreign_face).unwrap().shell;
        store
            .get_mut(original_shell)
            .unwrap()
            .faces
            .retain(|&candidate| candidate != peer_face);
        store.get_mut(foreign_shell).unwrap().faces.push(peer_face);
        store.get_mut(peer_face).unwrap().shell = foreign_shell;

        assert_eq!(
            certify_whole_fin_incidence(&store, face, loop_, fin, TOLERANCE),
            WholeFinIncidence::Indeterminate
        );
    }
}
