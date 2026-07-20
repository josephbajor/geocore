use super::*;
use crate::entity::{Body, Fin, Region, Sense};
use crate::make::block;
use crate::planar::{
    PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey,
};
use crate::planar_multishell::{PlanarMultiShellSolidInput, PlanarMultiShellSolidOutput};
use crate::store::Store;
use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};

fn assert_has(faults: &[Fault], kind: FaultKind) {
    assert!(
        faults.iter().any(|fault| fault.kind == kind),
        "expected {kind:?} in {faults:?}"
    );
}

fn clean_block(store: &mut Store) -> BodyId {
    block(store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap()
}

fn semantic_box_input(
    store: &mut Store,
    first_key: u64,
    center: [f64; 3],
    half_extents: [f64; 3],
    negative: bool,
) -> PlanarSolidInput {
    let frame = Frame::new(
        Point3::new(center[0], center[1], center[2]),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let source = block(
        store,
        &frame,
        half_extents.map(|half_extent| 2.0 * half_extent),
    )
    .unwrap();
    let surfaces: Vec<_> = store
        .faces_of_body(source)
        .unwrap()
        .into_iter()
        .map(|face| store.get(face).unwrap().surface)
        .collect();
    let [cx, cy, cz] = center;
    let [hx, hy, hz] = half_extents;
    let points = [
        Point3::new(cx - hx, cy - hy, cz - hz),
        Point3::new(cx + hx, cy - hy, cz - hz),
        Point3::new(cx - hx, cy + hy, cz - hz),
        Point3::new(cx + hx, cy + hy, cz - hz),
        Point3::new(cx - hx, cy - hy, cz + hz),
        Point3::new(cx + hx, cy - hy, cz + hz),
        Point3::new(cx - hx, cy + hy, cz + hz),
        Point3::new(cx + hx, cy + hy, cz + hz),
    ];
    let keys: [PlanarVertexKey; 8] =
        core::array::from_fn(|index| PlanarVertexKey::new(first_key + index as u64));
    let vertices = points
        .into_iter()
        .enumerate()
        .map(|(index, point)| PlanarSolidVertex::new(keys[index], point))
        .collect();
    let rings = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let faces = rings
        .iter()
        .enumerate()
        .map(|(face_index, ring)| {
            let mut directed_ring = ring.to_vec();
            if negative {
                directed_ring.reverse();
            }
            let carriers = (0..directed_ring.len())
                .map(|edge_index| {
                    let a = directed_ring[edge_index];
                    let b = directed_ring[(edge_index + 1) % directed_ring.len()];
                    let other = rings
                        .iter()
                        .enumerate()
                        .find(|(candidate_index, candidate)| {
                            *candidate_index != face_index
                                && (0..candidate.len()).any(|index| {
                                    let c = candidate[index];
                                    let d = candidate[(index + 1) % candidate.len()];
                                    a == c && b == d || a == d && b == c
                                })
                        })
                        .unwrap()
                        .0;
                    surfaces[other]
                })
                .collect();
            PlanarSolidFace::new(directed_ring.into_iter().map(|index| keys[index]).collect())
                .with_plane_binding(PlanarFacePlaneBinding::new(surfaces[face_index], carriers))
        })
        .collect();
    PlanarSolidInput::new(vertices, faces)
}

fn semantic_multishell_store(
    cavities: &[([f64; 3], [f64; 3])],
) -> (Store, PlanarMultiShellSolidOutput) {
    let mut store = Store::new();
    let outer = semantic_box_input(&mut store, 10, [0.0; 3], [3.0, 2.5, 2.0], false);
    let cavities = cavities
        .iter()
        .enumerate()
        .map(|(index, &(center, half_extents))| {
            semantic_box_input(
                &mut store,
                100 + u64::try_from(index).unwrap() * 10,
                center,
                half_extents,
                true,
            )
        })
        .collect();
    let input = PlanarMultiShellSolidInput::new(outer, cavities);
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_planar_multishell_solid(&input)
        .unwrap();
    (transaction.store().clone(), output)
}

#[test]
fn flipped_fin_sense_opens_the_loop() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let face = store.faces_of_body(body).unwrap()[0];
    let lp = store.get(face).unwrap().loops[0];
    let fin = store.get(lp).unwrap().fins[0];
    let sense = store.get(fin).unwrap().sense;
    store.get_mut(fin).unwrap().sense = sense.flipped();
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::OpenLoop);
    assert_has(&faults, FaultKind::FinsNotOpposed);
}

#[test]
fn reversed_loop_has_wrong_orientation() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let face = store.faces_of_body(body).unwrap()[0];
    let lp = store.get(face).unwrap().loops[0];
    let mut fins = store.get(lp).unwrap().fins.clone();
    fins.reverse();
    for &fin in &fins {
        let sense = store.get(fin).unwrap().sense;
        store.get_mut(fin).unwrap().sense = sense.flipped();
    }
    store.get_mut(lp).unwrap().fins = fins;
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::WrongLoopOrientation);
    assert_has(&faults, FaultKind::FinsNotOpposed);
}

#[test]
fn moved_vertex_is_off_curve() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let vertex = store.vertices_of_body(body).unwrap()[0];
    let point = store.get(vertex).unwrap().point;
    *store.get_mut(point).unwrap() += Vec3::new(1e-4, 0.0, 0.0);
    assert_has(
        &check_body(&store, body).unwrap(),
        FaultKind::VertexOffCurve,
    );
}

#[test]
fn oversized_coordinate_leaves_size_box() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let vertex = store.vertices_of_body(body).unwrap()[0];
    let point = store.get(vertex).unwrap().point;
    *store.get_mut(point).unwrap() = Point3::new(600.0, 0.0, 0.0);
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::OutsideSizeBox);
    assert_has(&faults, FaultKind::VertexOffCurve);
}

#[test]
fn region_kind_faults() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let exterior = store.get(body).unwrap().regions[0];
    store.get_mut(exterior).unwrap().kind = RegionKind::Solid;
    assert_has(
        &check_body(&store, body).unwrap(),
        FaultKind::ExteriorNotVoid,
    );
    store.get_mut(exterior).unwrap().kind = RegionKind::Void;

    let solid = store.get(body).unwrap().regions[1];
    store.get_mut(solid).unwrap().kind = RegionKind::Void;
    assert_has(&check_body(&store, body).unwrap(), FaultKind::NoSolidRegion);
}

#[test]
fn fast_checker_enforces_reduced_solid_region_partition() {
    let (store, output) = semantic_multishell_store(&[([0.25, -0.2, 0.1], [0.75, 0.5, 0.4])]);
    let body = output.body();
    let canonical = check_body_report(&store, body, CheckLevel::Fast).unwrap();
    assert_eq!(canonical.outcome(), CheckOutcome::Valid, "{canonical:?}");

    let body_regions = store.get(body).unwrap().regions.clone();
    let finite_void = *body_regions.last().unwrap();
    let cavity_shell = output.cavity_shells()[0];
    let solid_region = store.get(cavity_shell).unwrap().region;

    let mut missing_void = store.clone();
    missing_void
        .get_mut(body)
        .unwrap()
        .regions
        .retain(|region| *region != finite_void);
    assert_has(
        &check_body(&missing_void, body).unwrap(),
        FaultKind::RegionShellLayout,
    );

    let mut extra_void = store.clone();
    let extra = extra_void.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    extra_void.get_mut(body).unwrap().regions.push(extra);
    assert_has(
        &check_body(&extra_void, body).unwrap(),
        FaultKind::RegionShellLayout,
    );

    let mut duplicate_void = store.clone();
    duplicate_void
        .get_mut(body)
        .unwrap()
        .regions
        .push(finite_void);
    let duplicate_faults = check_body(&duplicate_void, body).unwrap();
    assert_has(&duplicate_faults, FaultKind::BackPointerMismatch);
    assert_has(&duplicate_faults, FaultKind::RegionShellLayout);

    let mut wrong_kind = store.clone();
    wrong_kind.get_mut(finite_void).unwrap().kind = RegionKind::Solid;
    assert_has(
        &check_body(&wrong_kind, body).unwrap(),
        FaultKind::RegionShellLayout,
    );

    let mut void_owned_shell = store.clone();
    void_owned_shell
        .get_mut(solid_region)
        .unwrap()
        .shells
        .retain(|shell| *shell != cavity_shell);
    void_owned_shell.get_mut(cavity_shell).unwrap().region = finite_void;
    void_owned_shell
        .get_mut(finite_void)
        .unwrap()
        .shells
        .push(cavity_shell);
    assert_has(
        &check_body(&void_owned_shell, body).unwrap(),
        FaultKind::RegionShellLayout,
    );
}

#[test]
fn full_checker_keeps_contact_and_unsupported_multicavity_indeterminate() {
    let (contact_store, contact) =
        semantic_multishell_store(&[([2.25, 0.0, 0.0], [0.75, 0.5, 0.4])]);
    let contact_report =
        check_body_report(&contact_store, contact.body(), CheckLevel::Full).unwrap();
    assert_eq!(
        contact_report.outcome(),
        CheckOutcome::Indeterminate,
        "{contact_report:?}"
    );
    assert!(contact_report.faults.is_empty(), "{contact_report:?}");
    assert!(contact_report.gaps.iter().any(|gap| {
        gap.kind == VerificationGapKind::RegionContainment
            && matches!(gap.entity, EntityRef::Region(_))
    }));

    let (multicavity_store, multicavity) = semantic_multishell_store(&[
        ([-1.0, 0.0, 0.0], [0.4, 0.4, 0.4]),
        ([1.0, 0.0, 0.0], [0.4, 0.4, 0.4]),
    ]);
    let fast = check_body_report(&multicavity_store, multicavity.body(), CheckLevel::Fast).unwrap();
    assert_eq!(fast.outcome(), CheckOutcome::Valid, "{fast:?}");
    let full = check_body_report(&multicavity_store, multicavity.body(), CheckLevel::Full).unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Indeterminate, "{full:?}");
    assert!(full.faults.is_empty(), "{full:?}");
    assert!(full.gaps.iter().any(|gap| {
        gap.kind == VerificationGapKind::RegionContainment
            && matches!(gap.entity, EntityRef::Region(_))
    }));
}

#[test]
fn body_without_regions_faults() {
    let mut store = Store::new();
    let body = store.add(Body {
        kind: BodyKind::Solid,
        regions: Vec::new(),
    });
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::NoRegions);
    assert_has(&faults, FaultKind::NoSolidRegion);
}

#[test]
fn fin_missing_from_loop_list_opens_ring() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let face = store.faces_of_body(body).unwrap()[0];
    let lp = store.get(face).unwrap().loops[0];
    store.get_mut(lp).unwrap().fins.remove(1);
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::OpenLoop);
    assert_has(&faults, FaultKind::BackPointerMismatch);
}

#[test]
fn reversed_bounds_are_bad() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let edge = store.edges_of_body(body).unwrap()[0];
    let (t0, t1) = store.get(edge).unwrap().bounds.unwrap();
    store.get_mut(edge).unwrap().bounds = Some((t1, t0));
    assert_has(&check_body(&store, body).unwrap(), FaultKind::BadBounds);
}

#[test]
fn vertex_bearing_edge_without_bounds_faults() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let edge = store.edges_of_body(body).unwrap()[0];
    store.get_mut(edge).unwrap().bounds = None;
    assert_has(&check_body(&store, body).unwrap(), FaultKind::MissingBounds);
}

#[test]
fn sub_resolution_tolerance_faults() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let vertex = store.vertices_of_body(body).unwrap()[0];
    store.get_mut(vertex).unwrap().tolerance =
        Some(crate::tolerance::EntityTolerance::unchecked(1e-12));
    assert_has(&check_body(&store, body).unwrap(), FaultKind::BadTolerance);
}

#[test]
fn removed_face_breaks_euler_and_fin_counts() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let face = store.faces_of_body(body).unwrap()[0];
    let shell = store.get(face).unwrap().shell;
    let lp = store.get(face).unwrap().loops[0];
    for fin in store.get(lp).unwrap().fins.clone() {
        let edge = store.get(fin).unwrap().edge;
        store
            .get_mut(edge)
            .unwrap()
            .fins
            .retain(|&candidate| candidate != fin);
        store.remove(fin).unwrap();
    }
    store.remove(lp).unwrap();
    store
        .get_mut(shell)
        .unwrap()
        .faces
        .retain(|&candidate| candidate != face);
    store.remove(face).unwrap();
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::EulerViolation);
    assert_has(&faults, FaultKind::BadFinCount);
}

#[test]
fn loop_with_foreign_parent_mismatches() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let faces = store.faces_of_body(body).unwrap();
    let lp = store.get(faces[0]).unwrap().loops[0];
    store.get_mut(lp).unwrap().face = faces[1];
    assert_has(
        &check_body(&store, body).unwrap(),
        FaultKind::BackPointerMismatch,
    );
}

#[test]
fn extra_fin_breaks_manifold_count() {
    let mut store = Store::new();
    let body = clean_block(&mut store);
    let edge = store.edges_of_body(body).unwrap()[0];
    let some_loop = {
        let face = store.faces_of_body(body).unwrap()[0];
        store.get(face).unwrap().loops[0]
    };
    let fin = store.add(Fin {
        parent: some_loop,
        edge,
        sense: Sense::Forward,
        pcurve: None,
    });
    store.get_mut(edge).unwrap().fins.push(fin);
    let faults = check_body(&store, body).unwrap();
    assert_has(&faults, FaultKind::BadFinCount);
    assert_has(&faults, FaultKind::BackPointerMismatch);
}
