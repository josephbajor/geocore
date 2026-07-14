//! Euler operators: the only sanctioned way to *edit* topology.
//!
//! Each operator makes one local, invariant-preserving change (Mäntylä's
//! operator set adapted to the Parasolid entity model):
//!
//! - MVFS / KVFS — make/kill the minimal body (vertex, face, shell).
//! - MEV / KEV — make/kill edge + vertex.
//! - MEF / KEF — make/kill edge + face (splitting/merging a loop).
//! - KEMR / MEKR — kill edge, make ring loop / inverse.
//! - KFMRH / MFKRH — kill face, make ring hole (genus change) / inverse.
//!
//! Raw operators are topology-internal. External modeling code uses the
//! corresponding [`crate::transaction::Transaction`] methods so multi-step
//! edits are rollback-safe, pcurve-bearing creation is mandatory, checked
//! commit gates finished bodies, and semantic lineage is recorded.
//!
//! Every operator keeps the Euler–Poincaré identity
//! `V − E + F = 2(S − G) + R` (`R` = inner-loop count, i.e. loops beyond
//! the first on each loop-bearing face) true by construction; property
//! tests exercise this across randomized operator sequences. The identity
//! as stated applies to vertex-and-loop-rich bodies; bodies with ring
//! edges or zero-loop faces (revolved primitives) are validated by the
//! checker instead.
//!
//! **Geometry policy.** Euler operators never compute or validate
//! geometry: raw operators take already-stored geometry handles
//! ([`crate::entity::PointId`], [`crate::entity::CurveId`],
//! [`crate::entity::SurfaceId`]) each new entity should reference, and the
//! checker later verifies geometric consistency. The semantic position-owning
//! MEV transaction entry point validates its [`Point3`] and all raw MEV
//! preconditions before inserting the point. The `*_with_pcurves`
//! variants additionally preflight the complete edge/pcurve/surface tuple
//! and attach an explicit pcurve to every new fin. Operators that move
//! pcurve-bearing fins between faces preflight them on the destination
//! surface and reject the move if reparameterization is required. Kill
//! operators remove topology only — attached geometry stays in the store
//! (it may be shared).
//!
//! **Transient states.** Between operator applications a body may be
//! topologically consistent but not checker-clean (e.g. the zero-fin loop
//! and lone shell vertex made by MVFS, or "strut" edges whose two fins
//! belong to one loop). Only *finished* bodies are expected to pass
//! [`crate::check::check_body`].
//!
//! Precondition violations are reported as
//! [`kcore::error::Error::InvalidGeometry`] with a static reason string
//! (a dedicated topology variant in `kcore::error` is an integration
//! candidate).

use crate::entity::{
    BodyId, CurveId, Edge, EdgeId, Face, FaceDomain, FaceId, Fin, FinId, FinPcurve, Loop, LoopId,
    PointId, RegionId, Sense, ShellId, SurfaceId, Vertex, VertexId,
};
use crate::incidence::{PcurveIssue, check_pcurve_incidence};
use crate::store::Store;
use crate::tolerance::EntityTolerance;
use kcore::error::{Error, Result};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::vec::Point3;

fn topo_err<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

fn merged_face_metadata(
    store: &Store,
    a: &Face,
    b: &Face,
) -> Result<(Option<FaceDomain>, Option<EntityTolerance>)> {
    let mut domain = if a.surface == b.surface {
        match (a.domain, b.domain) {
            (Some(a), Some(b)) => Some(a.union(b)?),
            _ => None,
        }
    } else {
        None
    };
    if let Some(candidate) = domain {
        let periodicity = store
            .eval_context(
                kgraph::EvalLimits::default(),
                kcore::tolerance::Tolerances::default(),
            )
            .surface_periodicity(a.surface)
            .ok();
        match periodicity {
            Some(periodicity)
                if [candidate.u, candidate.v].into_iter().zip(periodicity).any(
                    |(range, period)| {
                        period.is_some_and(|period| {
                            let epsilon = 128.0 * f64::EPSILON * period.max(1.0);
                            range.width() > period + epsilon
                        })
                    },
                ) =>
            {
                // Equivalent seam branches need explicit branch metadata before
                // they can be unioned safely. Preserve correctness by marking the
                // merged work box unknown rather than guessing a period shift.
                domain = None;
            }
            None => {
                domain = None;
            }
            Some(_) => {}
        }
    }
    let tolerance = match (a.tolerance, b.tolerance) {
        (Some(a), Some(b)) => Some(a.inherited_max(b)),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    };
    Ok((domain, tolerance))
}

/// The two independent parameter-space uses of a new edge, keyed by the
/// edge-relative sense of the fins that will own them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FinPcurvePair {
    /// Pcurve for the fin whose sense is [`Sense::Forward`].
    pub forward: FinPcurve,
    /// Pcurve for the fin whose sense is [`Sense::Reversed`].
    pub reversed: FinPcurve,
}

impl FinPcurvePair {
    /// Construct an explicitly ordered pcurve pair.
    pub const fn new(forward: FinPcurve, reversed: FinPcurve) -> Self {
        Self { forward, reversed }
    }
}

fn pcurve_error(issue: PcurveIssue) -> Error {
    match issue {
        PcurveIssue::StaleReference => Error::StaleHandle,
        PcurveIssue::BadRange => Error::InvalidGeometry {
            reason: "Euler pcurve range does not cover the new edge parameter interval",
        },
        PcurveIssue::BadChart => Error::InvalidGeometry {
            reason: "Euler pcurve chart is invalid for the destination surface",
        },
        PcurveIssue::BadClosure => Error::InvalidGeometry {
            reason: "Euler pcurve closure winding is invalid",
        },
        PcurveIssue::BadSingularity => Error::InvalidGeometry {
            reason: "Euler pcurve singular endpoint metadata is invalid",
        },
        PcurveIssue::BadSeam => Error::InvalidGeometry {
            reason: "Euler pcurve seam metadata is invalid",
        },
        PcurveIssue::OffSurface => Error::InvalidGeometry {
            reason: "Euler pcurve does not lift to the new 3D edge on its face surface",
        },
    }
}

fn validate_pcurve_pair(
    store: &Store,
    curve: CurveId,
    bounds: (f64, f64),
    surfaces: [SurfaceId; 2],
    pcurves: FinPcurvePair,
) -> Result<()> {
    for (surface, pcurve) in [
        (surfaces[0], pcurves.forward),
        (surfaces[1], pcurves.reversed),
    ] {
        check_pcurve_incidence(
            store,
            curve,
            Some(bounds),
            surface,
            pcurve,
            LINEAR_RESOLUTION,
        )
        .map_err(pcurve_error)?;
    }
    Ok(())
}

fn validate_fins_on_surface(store: &Store, fins: &[FinId], surface: SurfaceId) -> Result<()> {
    for &fin_id in fins {
        let fin = store.get(fin_id)?;
        let Some(pcurve) = fin.pcurve else {
            continue;
        };
        let edge = store.get(fin.edge)?;
        let curve = edge.curve.ok_or(Error::InvalidGeometry {
            reason: "moving curve-less pcurve fins requires tolerant-edge support",
        })?;
        check_pcurve_incidence(
            store,
            curve,
            edge.bounds,
            surface,
            pcurve,
            edge.tolerance
                .map(EntityTolerance::value)
                .unwrap_or(0.0)
                .max(LINEAR_RESOLUTION),
        )
        .map_err(pcurve_error)?;
    }
    Ok(())
}

/// Validate an edge parameter interval: finite and increasing.
fn valid_bounds(bounds: (f64, f64)) -> Result<()> {
    if bounds.0.is_finite() && bounds.1.is_finite() && bounds.0 < bounds.1 {
        Ok(())
    } else {
        topo_err("edge bounds must be finite with t0 < t1")
    }
}

/// The shell owning a loop (via its face).
fn shell_of_loop(store: &Store, lp: LoopId) -> Result<ShellId> {
    Ok(store.get(store.get(lp)?.face)?.shell)
}

/// Entities created by MVFS.
#[derive(Debug, Clone, Copy)]
pub struct Mvfs {
    /// The new body.
    pub body: BodyId,
    /// Its infinite void exterior region.
    pub void_region: RegionId,
    /// Its solid region (owns the shell).
    pub solid_region: RegionId,
    /// The new shell; holds `vertex` in its acorn slot until the first
    /// MEV hangs an edge off it.
    pub shell: ShellId,
    /// The new face (one zero-fin loop).
    pub face: FaceId,
    /// The face's loop, initially with no fins.
    pub ring: LoopId,
    /// The seed vertex.
    pub vertex: VertexId,
}

#[derive(Debug, Clone, Copy)]
struct MvfsPreflight {
    domain: Option<FaceDomain>,
}

/// Topology and geometry identities detached by KVFS.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Kvfs {
    /// Point formerly referenced by the removed seed vertex.
    pub point: PointId,
}

#[derive(Debug, Clone, Copy)]
struct KvfsPreflight {
    void: RegionId,
    solid: RegionId,
    shell: ShellId,
    face: FaceId,
    ring: LoopId,
    vertex: VertexId,
    point: PointId,
}

/// Make vertex, face, shell (and body): the minimal solid body.
///
/// Creates a solid body scaffold (void + solid regions, one shell), one
/// face on `surface` with a single zero-fin loop, and a seed vertex stored
/// in the shell's acorn slot. `V − E + F = 1 − 0 + 1 = 2 = 2(S − G) + R`.
pub(crate) fn mvfs(
    store: &mut Store,
    surface: SurfaceId,
    sense: Sense,
    point: PointId,
) -> Result<Mvfs> {
    let preflight = preflight_mvfs(store, surface)?;
    store.get(point)?;
    Ok(apply_mvfs(store, surface, sense, point, preflight))
}

pub(crate) fn mvfs_at_position(
    store: &mut Store,
    surface: SurfaceId,
    sense: Sense,
    position: Point3,
) -> Result<(Mvfs, PointId)> {
    Store::validate_point(position)?;
    let preflight = preflight_mvfs(store, surface)?;
    let point = store.insert_point(position)?;
    Ok((apply_mvfs(store, surface, sense, point, preflight), point))
}

fn preflight_mvfs(store: &Store, surface: SurfaceId) -> Result<MvfsPreflight> {
    Ok(MvfsPreflight {
        domain: FaceDomain::natural(store.get(surface)?),
    })
}

fn apply_mvfs(
    store: &mut Store,
    surface: SurfaceId,
    sense: Sense,
    point: PointId,
    preflight: MvfsPreflight,
) -> Mvfs {
    let (body, shell) = crate::make::solid_body_scaffold(store);
    let regions = store
        .get(body)
        .expect("fresh MVFS body remains live")
        .regions
        .clone();
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense,
        domain: preflight.domain,
        tolerance: None,
    });
    let ring = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    store
        .get_mut(face)
        .expect("fresh MVFS face remains live")
        .loops
        .push(ring);
    let vertex = store.add(Vertex {
        point,
        tolerance: None,
    });
    let sh = store.get_mut(shell).expect("fresh MVFS shell remains live");
    sh.faces.push(face);
    sh.vertex = Some(vertex);
    Mvfs {
        body,
        void_region: regions[0],
        solid_region: regions[1],
        shell,
        face,
        ring,
        vertex,
    }
}

/// Kill vertex, face, shell, body: exact inverse of MVFS.
///
/// The body must still be in the minimal shape MVFS made (one face
/// with one empty loop, seed vertex in the shell slot, no edges).
pub(crate) fn kvfs(store: &mut Store, body: BodyId) -> Result<Kvfs> {
    let preflight = preflight_kvfs(store, body)?;
    apply_kvfs(store, body, preflight);
    Ok(Kvfs {
        point: preflight.point,
    })
}

fn preflight_kvfs(store: &Store, body: BodyId) -> Result<KvfsPreflight> {
    let regions = store.get(body)?.regions.clone();
    if regions.len() != 2 {
        return topo_err("kvfs: body must have exactly void + solid regions");
    }
    let (void, solid) = (regions[0], regions[1]);
    if !store.get(void)?.shells.is_empty() {
        return topo_err("kvfs: void region must own no shells");
    }
    let shells = store.get(solid)?.shells.clone();
    let [shell] = shells[..] else {
        return topo_err("kvfs: solid region must own exactly one shell");
    };
    let sh = store.get(shell)?;
    let (faces, edges, vertex) = (sh.faces.clone(), sh.edges.clone(), sh.vertex);
    let [face] = faces[..] else {
        return topo_err("kvfs: shell must own exactly one face");
    };
    if !edges.is_empty() {
        return topo_err("kvfs: shell must own no wire edges");
    }
    let Some(vertex) = vertex else {
        return topo_err("kvfs: shell must still hold the seed vertex");
    };
    let loops = store.get(face)?.loops.clone();
    let [ring] = loops[..] else {
        return topo_err("kvfs: face must have exactly one loop");
    };
    if !store.get(ring)?.fins.is_empty() {
        return topo_err("kvfs: loop must have no fins");
    }
    let point = store.get(vertex)?.point;
    store.get(point)?;
    Ok(KvfsPreflight {
        void,
        solid,
        shell,
        face,
        ring,
        vertex,
        point,
    })
}

fn apply_kvfs(store: &mut Store, body: BodyId, preflight: KvfsPreflight) {
    store
        .remove(preflight.ring)
        .expect("KVFS preflight keeps the seed loop live");
    store
        .remove(preflight.face)
        .expect("KVFS preflight keeps the seed face live");
    store
        .remove(preflight.vertex)
        .expect("KVFS preflight keeps the seed vertex live");
    store
        .remove(preflight.shell)
        .expect("KVFS preflight keeps the seed shell live");
    store
        .remove(preflight.void)
        .expect("KVFS preflight keeps the void region live");
    store
        .remove(preflight.solid)
        .expect("KVFS preflight keeps the solid region live");
    store
        .remove(body)
        .expect("KVFS preflight keeps the seed body live");
}

/// Entities created by MEV.
#[derive(Debug, Clone, Copy)]
pub struct Mev {
    /// The new edge, directed sprout vertex → new vertex.
    pub edge: EdgeId,
    /// The new vertex (at the edge's head).
    pub vertex: VertexId,
    /// The outbound fin (`Forward`), at loop index `at`.
    pub fin_out: FinId,
    /// The return fin (`Reversed`), at loop index `at + 1`.
    pub fin_back: FinId,
}

/// Topology and geometry identities detached by KEV.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Kev {
    /// Removed vertex.
    pub vertex: VertexId,
    /// Point formerly referenced by that vertex.
    pub point: PointId,
}

#[derive(Debug, Clone, Copy)]
struct MevPreflight {
    sprout: VertexId,
    empty_shell: Option<ShellId>,
}

/// Make edge and vertex: sprout a new edge into a loop.
///
/// The new edge runs from the *sprout vertex* to a new vertex at `point`,
/// and both of its fins are spliced consecutively into `lp` at index `at`
/// (out then back — a "strut" the next MEF can split away). The
/// sprout vertex is the tail of the fin currently at `at`; on a zero-fin
/// loop (fresh from MVFS) `at` must be 0 and the shell's seed vertex
/// is consumed as the sprout. The caller guarantees `curve`/`bounds` run
/// sprout → new vertex.
#[cfg(test)]
pub(crate) fn mev(
    store: &mut Store,
    lp: LoopId,
    at: usize,
    curve: CurveId,
    bounds: (f64, f64),
    point: PointId,
) -> Result<Mev> {
    let preflight = preflight_mev(store, lp, at, curve, bounds, None)?;
    store.get(point)?;
    Ok(apply_mev(
        store, lp, at, curve, bounds, point, None, preflight,
    ))
}

/// Pcurve-bearing MEV. Both pcurves are validated against the owning
/// face before topology is mutated, then attached to the new fins.
pub(crate) fn mev_with_pcurves(
    store: &mut Store,
    lp: LoopId,
    at: usize,
    curve: CurveId,
    bounds: (f64, f64),
    point: PointId,
    pcurves: FinPcurvePair,
) -> Result<Mev> {
    let preflight = preflight_mev(store, lp, at, curve, bounds, Some(pcurves))?;
    store.get(point)?;
    Ok(apply_mev(
        store,
        lp,
        at,
        curve,
        bounds,
        point,
        Some(pcurves),
        preflight,
    ))
}

pub(crate) fn mev_at_position_with_pcurves(
    store: &mut Store,
    lp: LoopId,
    at: usize,
    curve: CurveId,
    bounds: (f64, f64),
    position: Point3,
    pcurves: FinPcurvePair,
) -> Result<(Mev, PointId)> {
    Store::validate_point(position)?;
    let preflight = preflight_mev(store, lp, at, curve, bounds, Some(pcurves))?;
    let point = store.insert_point(position)?;
    let made = apply_mev(
        store,
        lp,
        at,
        curve,
        bounds,
        point,
        Some(pcurves),
        preflight,
    );
    Ok((made, point))
}

fn preflight_mev(
    store: &Store,
    lp: LoopId,
    at: usize,
    curve: CurveId,
    bounds: (f64, f64),
    pcurves: Option<FinPcurvePair>,
) -> Result<MevPreflight> {
    valid_bounds(bounds)?;
    store.get(curve)?;
    let fins = store.get(lp)?.fins.clone();
    if let Some(pcurves) = pcurves {
        let face = store.get(store.get(lp)?.face)?;
        validate_pcurve_pair(store, curve, bounds, [face.surface, face.surface], pcurves)?;
    }
    if fins.is_empty() {
        if at != 0 {
            return topo_err("mev: index must be 0 on a zero-fin loop");
        }
        let shell = shell_of_loop(store, lp)?;
        let seed = store.get(shell)?.vertex;
        let Some(seed) = seed else {
            return topo_err("mev: zero-fin loop's shell holds no seed vertex");
        };
        Ok(MevPreflight {
            sprout: seed,
            empty_shell: Some(shell),
        })
    } else {
        if at >= fins.len() {
            return topo_err("mev: fin index out of range");
        }
        let Some(tail) = store.fin_tail(fins[at])? else {
            return topo_err("mev: cannot sprout at a ring edge fin");
        };
        store.vertex_position(tail)?;
        Ok(MevPreflight {
            sprout: tail,
            empty_shell: None,
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_mev(
    store: &mut Store,
    lp: LoopId,
    at: usize,
    curve: CurveId,
    bounds: (f64, f64),
    point: PointId,
    pcurves: Option<FinPcurvePair>,
    preflight: MevPreflight,
) -> Mev {
    if let Some(shell) = preflight.empty_shell {
        store
            .get_mut(shell)
            .expect("MEV preflight keeps the empty shell live")
            .vertex = None;
    }
    let vertex = store.add(Vertex {
        point,
        tolerance: None,
    });
    let edge = store.add(Edge {
        curve: Some(curve),
        vertices: [Some(preflight.sprout), Some(vertex)],
        bounds: Some(bounds),
        fins: Vec::new(),
        tolerance: None,
    });
    let fin_out = store.add(Fin {
        parent: lp,
        edge,
        sense: Sense::Forward,
        pcurve: pcurves.map(|pair| pair.forward),
    });
    let fin_back = store.add(Fin {
        parent: lp,
        edge,
        sense: Sense::Reversed,
        pcurve: pcurves.map(|pair| pair.reversed),
    });
    store.get_mut(edge).expect("new MEV edge remains live").fins = vec![fin_out, fin_back];
    let ring = &mut store
        .get_mut(lp)
        .expect("MEV preflight keeps the destination loop live")
        .fins;
    ring.insert(at, fin_back);
    ring.insert(at, fin_out);
    Mev {
        edge,
        vertex,
        fin_out,
        fin_back,
    }
}

/// Kill edge and vertex: exact inverse of MEV.
///
/// The edge's two fins must sit consecutively in one loop (the strut
/// shape MEV makes), and the vertex between them must be used by no
/// other edge. If the loop becomes empty, the surviving vertex returns to
/// the shell's seed slot.
pub(crate) fn kev(store: &mut Store, edge: EdgeId) -> Result<Kev> {
    let e = store.get(edge)?;
    let [f0, f1] = e.fins[..] else {
        return topo_err("kev: edge must have exactly two fins");
    };
    if f0 == f1 {
        return topo_err("kev: edge fins must be distinct");
    }
    let (p0, p1) = (store.get(f0)?.parent, store.get(f1)?.parent);
    if p0 != p1 {
        return topo_err("kev: the edge's fins must share one loop");
    }
    let lp = p0;
    let fins = store.get(lp)?.fins.clone();
    let n = fins.len();
    let Some(i) = (0..n).find(|&i| {
        fins[i] == f0 && fins[(i + 1) % n] == f1 || fins[i] == f1 && fins[(i + 1) % n] == f0
    }) else {
        return topo_err("kev: the edge's fins must be consecutive in the loop");
    };
    let first = fins[i];
    // The dying vertex is between the two fins: the head of the first.
    let Some(dying) = store.fin_head(first)? else {
        return topo_err("kev: edge has no vertices (ring edge)");
    };
    let Some(survivor) = store.fin_tail(first)? else {
        return topo_err("kev: edge has no vertices (ring edge)");
    };
    if dying == survivor {
        return topo_err("kev: cannot kill a closed edge's only vertex");
    }
    let vertex_in_use = store
        .iter::<Edge>()
        .any(|(h, other)| h != edge && other.vertices.contains(&Some(dying)))
        || store
            .iter::<crate::entity::Shell>()
            .any(|(_, s)| s.vertex == Some(dying));
    if vertex_in_use {
        return topo_err("kev: the edge's head vertex is still in use");
    }
    let point = store.get(dying)?.point;
    store.get(point)?;
    let now_empty = fins.iter().all(|&fin| fin == f0 || fin == f1);
    let empty_shell = if now_empty {
        let shell = shell_of_loop(store, lp)?;
        if store.get(shell)?.vertex.is_some() {
            return topo_err("kev: shell seed slot already occupied");
        }
        Some(shell)
    } else {
        None
    };
    let ring = &mut store.get_mut(lp)?.fins;
    ring.retain(|f| *f != f0 && *f != f1);
    store.remove(f0)?;
    store.remove(f1)?;
    store.remove(edge)?;
    store.remove(dying)?;
    if let Some(shell) = empty_shell {
        let slot = &mut store.get_mut(shell)?.vertex;
        *slot = Some(survivor);
    }
    Ok(Kev {
        vertex: dying,
        point,
    })
}

/// Entities created by MEF.
#[derive(Debug, Clone, Copy)]
pub struct Mef {
    /// The new edge, directed tail(fins\[i\]) → tail(fins\[j\]).
    pub edge: EdgeId,
    /// The new face (owning `ring`).
    pub face: FaceId,
    /// The new face's loop.
    pub ring: LoopId,
    /// The fin left in the old loop (`Forward`).
    pub fin_old: FinId,
    /// The fin in the new loop (`Reversed`).
    pub fin_new: FinId,
}

/// Make edge and face: split a loop with a new edge.
///
/// A new edge runs from the tail vertex of `fins[i]` to the tail vertex
/// of `fins[j]` (indices into `lp`'s ring; `i == j` makes a closed edge).
/// The new face on `surface` takes the fins from `i` forward (cyclically)
/// up to `j`, closed by the new edge's `Reversed` fin; the old loop keeps
/// the rest, closed by the `Forward` fin. The new face joins the same
/// shell.
#[allow(clippy::too_many_arguments)] // one edge + one face worth of inputs
#[cfg(test)]
pub(crate) fn mef(
    store: &mut Store,
    lp: LoopId,
    i: usize,
    j: usize,
    curve: CurveId,
    bounds: (f64, f64),
    surface: SurfaceId,
    sense: Sense,
) -> Result<Mef> {
    mef_impl(store, lp, i, j, curve, bounds, surface, sense, None)
}

/// Pcurve-bearing MEF. The forward use is validated on the old face;
/// the reversed use is validated on the new face's supporting surface.
/// Existing pcurve-bearing fins moved to the new face are also preflighted.
#[allow(clippy::too_many_arguments)]
pub(crate) fn mef_with_pcurves(
    store: &mut Store,
    lp: LoopId,
    i: usize,
    j: usize,
    curve: CurveId,
    bounds: (f64, f64),
    surface: SurfaceId,
    sense: Sense,
    pcurves: FinPcurvePair,
) -> Result<Mef> {
    valid_bounds(bounds)?;
    let old_face = store.get(lp)?.face;
    let old_surface = store.get(old_face)?.surface;
    validate_pcurve_pair(store, curve, bounds, [old_surface, surface], pcurves)?;
    mef_impl(
        store,
        lp,
        i,
        j,
        curve,
        bounds,
        surface,
        sense,
        Some(pcurves),
    )
}

#[allow(clippy::too_many_arguments)]
fn mef_impl(
    store: &mut Store,
    lp: LoopId,
    i: usize,
    j: usize,
    curve: CurveId,
    bounds: (f64, f64),
    surface: SurfaceId,
    sense: Sense,
    pcurves: Option<FinPcurvePair>,
) -> Result<Mef> {
    valid_bounds(bounds)?;
    store.get(curve)?;
    store.get(surface)?;
    let fins = store.get(lp)?.fins.clone();
    let n = fins.len();
    if n == 0 {
        return topo_err("mef: cannot split a zero-fin loop");
    }
    if i >= n || j >= n {
        return topo_err("mef: fin index out of range");
    }
    let (Some(vi), Some(vj)) = (store.fin_tail(fins[i])?, store.fin_tail(fins[j])?) else {
        return topo_err("mef: cannot split at a ring edge fin");
    };
    let shell = shell_of_loop(store, lp)?;
    let old_face = store.get(lp)?.face;
    let old = store.get(old_face)?;
    let domain = if surface == old.surface {
        old.domain
    } else {
        FaceDomain::natural(store.get(surface)?)
    };
    let tolerance = old.tolerance;
    let new_len = (j + n - i) % n;
    if surface != store.get(old_face)?.surface {
        let moved: Vec<_> = (0..new_len).map(|k| fins[(i + k) % n]).collect();
        validate_fins_on_surface(store, &moved, surface)?;
    }

    let edge = store.add(Edge {
        curve: Some(curve),
        vertices: [Some(vi), Some(vj)],
        bounds: Some(bounds),
        fins: Vec::new(),
        tolerance: None,
    });
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense,
        domain,
        tolerance,
    });
    let ring = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    store.get_mut(face)?.loops.push(ring);
    store.get_mut(shell)?.faces.push(face);

    let fin_old = store.add(Fin {
        parent: lp,
        edge,
        sense: Sense::Forward,
        pcurve: pcurves.map(|pair| pair.forward),
    });
    let fin_new = store.add(Fin {
        parent: ring,
        edge,
        sense: Sense::Reversed,
        pcurve: pcurves.map(|pair| pair.reversed),
    });
    store.get_mut(edge)?.fins = vec![fin_old, fin_new];

    // New loop: fins i, i+1, … (mod n) up to but excluding j, then the
    // Reversed fin (v_j → v_i). Old loop: the remaining fins from j, then
    // the Forward fin (v_i → v_j). Both rings close by construction.
    // When i == j the new-loop segment is *empty* (the new face is a disc
    // bounded by the closed edge alone) and the old loop keeps all fins.
    let mut new_fins = Vec::with_capacity(new_len + 1);
    for k in 0..new_len {
        new_fins.push(fins[(i + k) % n]);
    }
    new_fins.push(fin_new);
    let mut old_fins = Vec::with_capacity(n - new_len + 1);
    for k in 0..(n - new_len) {
        old_fins.push(fins[(j + k) % n]);
    }
    old_fins.push(fin_old);

    for &f in &new_fins {
        store.get_mut(f)?.parent = ring;
    }
    store.get_mut(ring)?.fins = new_fins;
    store.get_mut(lp)?.fins = old_fins;
    let _ = old_face;
    Ok(Mef {
        edge,
        face,
        ring,
        fin_old,
        fin_new,
    })
}

/// Kill edge and face: inverse of MEF.
///
/// The edge's two fins must belong to loops of *different* faces, and the
/// face being absorbed must have exactly one loop. That face and loop are
/// removed and their remaining fins merge into the surviving loop.
pub(crate) fn kef(store: &mut Store, edge: EdgeId) -> Result<()> {
    let e = store.get(edge)?;
    let [fa, fb] = e.fins[..] else {
        return topo_err("kef: edge must have exactly two fins");
    };
    let la = store.get(fa)?.parent;
    let lb = store.get(fb)?.parent;
    let face_a = store.get(la)?.face;
    let face_b = store.get(lb)?.face;
    let face_a_data = store.get(face_a)?.clone();
    let face_b_data = store.get(face_b)?.clone();
    if face_a == face_b {
        return topo_err("kef: the edge's fins must belong to different faces (use kemr)");
    }
    if store.get(face_b)?.loops.len() != 1 {
        return topo_err("kef: the absorbed face must have exactly one loop");
    }
    if store.get(face_a)?.shell != store.get(face_b)?.shell {
        return topo_err("kef: faces must share a shell");
    }
    let fins_a = store.get(la)?.fins.clone();
    let fins_b = store.get(lb)?.fins.clone();
    if store.get(face_a)?.surface != store.get(face_b)?.surface {
        let moved: Vec<_> = fins_b.iter().copied().filter(|&fin| fin != fb).collect();
        validate_fins_on_surface(store, &moved, store.get(face_a)?.surface)?;
    }
    let ia = fins_a.iter().position(|f| *f == fa).expect("fin in loop");
    let ib = fins_b.iter().position(|f| *f == fb).expect("fin in loop");

    // Splice: replace fa by loop B's ring rotated to start after fb.
    let mut merged = Vec::with_capacity(fins_a.len() + fins_b.len() - 2);
    merged.extend_from_slice(&fins_a[..ia]);
    for k in 1..fins_b.len() {
        merged.push(fins_b[(ib + k) % fins_b.len()]);
    }
    merged.extend_from_slice(&fins_a[ia + 1..]);
    for &f in &merged {
        store.get_mut(f)?.parent = la;
    }
    store.get_mut(la)?.fins = merged;
    let (domain, tolerance) = merged_face_metadata(store, &face_a_data, &face_b_data)?;
    let survivor = store.get_mut(face_a)?;
    survivor.domain = domain;
    survivor.tolerance = tolerance;

    let shell = store.get(face_b)?.shell;
    store.get_mut(shell)?.faces.retain(|f| *f != face_b);
    store.remove(lb)?;
    store.remove(face_b)?;
    store.remove(fa)?;
    store.remove(fb)?;
    store.remove(edge)?;
    Ok(())
}

/// Kill edge, make ring: remove an edge whose two fins share a loop,
/// splitting it into the surviving loop and a new inner (ring) loop on
/// the same face.
///
/// The fins between the edge's two fins (in ring order, exclusive) become
/// the ring loop; both resulting rings must be non-empty (killing a strut
/// would need a vertex-only loop, which this model cannot represent —
/// use KEV for struts). Returns the new ring loop.
pub(crate) fn kemr(store: &mut Store, edge: EdgeId) -> Result<LoopId> {
    let e = store.get(edge)?;
    let [fa, fb] = e.fins[..] else {
        return topo_err("kemr: edge must have exactly two fins");
    };
    let lp = store.get(fa)?.parent;
    if store.get(fb)?.parent != lp {
        return topo_err("kemr: the edge's fins must share one loop (use kef)");
    }
    let fins = store.get(lp)?.fins.clone();
    let p = fins.iter().position(|f| *f == fa).expect("fin in loop");
    let q = fins.iter().position(|f| *f == fb).expect("fin in loop");
    let (p, q) = (p.min(q), p.max(q));
    let inner: Vec<FinId> = fins[p + 1..q].to_vec();
    let outer: Vec<FinId> = fins[q + 1..].iter().chain(&fins[..p]).copied().collect();
    if inner.is_empty() || outer.is_empty() {
        return topo_err("kemr: both split rings must be non-empty (use kev for struts)");
    }
    let face = store.get(lp)?.face;
    let ring = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    for &f in &inner {
        store.get_mut(f)?.parent = ring;
    }
    store.get_mut(ring)?.fins = inner;
    store.get_mut(lp)?.fins = outer;
    store.get_mut(face)?.loops.push(ring);
    store.remove(fa)?;
    store.remove(fb)?;
    store.remove(edge)?;
    Ok(ring)
}

/// Entities created by MEKR.
#[derive(Debug, Clone, Copy)]
pub struct Mekr {
    /// The new edge, directed outer tail → ring tail.
    pub edge: EdgeId,
    /// Its fin leaving the outer boundary (`Forward`).
    pub fin_out: FinId,
    /// Its fin returning from the ring (`Reversed`).
    pub fin_back: FinId,
}

/// Make edge, kill ring: join an inner (ring) loop back into another loop
/// of the same face with a new edge. Inverse of KEMR.
///
/// The edge runs from the tail vertex of `outer`'s fin `i` to the tail
/// vertex of `ring`'s fin `j`; `ring` is dissolved into `outer`.
#[cfg(test)]
pub(crate) fn mekr(
    store: &mut Store,
    outer: LoopId,
    i: usize,
    ring: LoopId,
    j: usize,
    curve: CurveId,
    bounds: (f64, f64),
) -> Result<Mekr> {
    mekr_impl(store, outer, i, ring, j, curve, bounds, None)
}

/// Pcurve-bearing MEKR. Both new fin uses are preflighted on the
/// loops' common face before the ring is dissolved.
#[allow(clippy::too_many_arguments)]
pub(crate) fn mekr_with_pcurves(
    store: &mut Store,
    outer: LoopId,
    i: usize,
    ring: LoopId,
    j: usize,
    curve: CurveId,
    bounds: (f64, f64),
    pcurves: FinPcurvePair,
) -> Result<Mekr> {
    valid_bounds(bounds)?;
    if outer == ring {
        return topo_err("mekr: loops must be distinct");
    }
    let face = store.get(outer)?.face;
    if store.get(ring)?.face != face {
        return topo_err("mekr: loops must belong to one face");
    }
    let surface = store.get(face)?.surface;
    validate_pcurve_pair(store, curve, bounds, [surface, surface], pcurves)?;
    mekr_impl(store, outer, i, ring, j, curve, bounds, Some(pcurves))
}

#[allow(clippy::too_many_arguments)]
fn mekr_impl(
    store: &mut Store,
    outer: LoopId,
    i: usize,
    ring: LoopId,
    j: usize,
    curve: CurveId,
    bounds: (f64, f64),
    pcurves: Option<FinPcurvePair>,
) -> Result<Mekr> {
    valid_bounds(bounds)?;
    store.get(curve)?;
    if outer == ring {
        return topo_err("mekr: loops must be distinct");
    }
    let face = store.get(outer)?.face;
    if store.get(ring)?.face != face {
        return topo_err("mekr: loops must belong to one face");
    }
    let outer_fins = store.get(outer)?.fins.clone();
    let ring_fins = store.get(ring)?.fins.clone();
    if i >= outer_fins.len() || j >= ring_fins.len() {
        return topo_err("mekr: fin index out of range");
    }
    let (Some(vi), Some(vj)) = (
        store.fin_tail(outer_fins[i])?,
        store.fin_tail(ring_fins[j])?,
    ) else {
        return topo_err("mekr: cannot join at a ring edge fin");
    };
    let edge = store.add(Edge {
        curve: Some(curve),
        vertices: [Some(vi), Some(vj)],
        bounds: Some(bounds),
        fins: Vec::new(),
        tolerance: None,
    });
    let fin_out = store.add(Fin {
        parent: outer,
        edge,
        sense: Sense::Forward,
        pcurve: pcurves.map(|pair| pair.forward),
    });
    let fin_back = store.add(Fin {
        parent: outer,
        edge,
        sense: Sense::Reversed,
        pcurve: pcurves.map(|pair| pair.reversed),
    });
    store.get_mut(edge)?.fins = vec![fin_out, fin_back];

    // outer[..i] + fin_out + ring[j..] + ring[..j] + fin_back + outer[i..]
    let mut merged = Vec::with_capacity(outer_fins.len() + ring_fins.len() + 2);
    merged.extend_from_slice(&outer_fins[..i]);
    merged.push(fin_out);
    for k in 0..ring_fins.len() {
        merged.push(ring_fins[(j + k) % ring_fins.len()]);
    }
    merged.push(fin_back);
    merged.extend_from_slice(&outer_fins[i..]);
    for &f in &merged {
        store.get_mut(f)?.parent = outer;
    }
    store.get_mut(outer)?.fins = merged;
    store.get_mut(face)?.loops.retain(|l| *l != ring);
    store.remove(ring)?;
    Ok(Mekr {
        edge,
        fin_out,
        fin_back,
    })
}

/// Kill face, make ring hole: delete `kill` (which must have exactly one
/// loop and share `keep`'s shell) and move its loop onto `keep` as an
/// inner loop. Raises the shell's genus by one. Returns the moved loop.
pub(crate) fn kfmrh(store: &mut Store, keep: FaceId, kill: FaceId) -> Result<LoopId> {
    if keep == kill {
        return topo_err("kfmrh: faces must be distinct");
    }
    let keep_face = store.get(keep)?.clone();
    let kill_face = store.get(kill)?.clone();
    let shell = keep_face.shell;
    if kill_face.shell != shell {
        return topo_err("kfmrh: faces must share a shell");
    }
    let loops = kill_face.loops.clone();
    let [ring] = loops[..] else {
        return topo_err("kfmrh: killed face must have exactly one loop");
    };
    if keep_face.surface != kill_face.surface {
        validate_fins_on_surface(store, &store.get(ring)?.fins, keep_face.surface)?;
    }
    let (merged_domain, merged_tolerance) = merged_face_metadata(store, &keep_face, &kill_face)?;
    store.get_mut(ring)?.face = keep;
    let keep = store.get_mut(keep)?;
    keep.loops.push(ring);
    keep.domain = merged_domain;
    keep.tolerance = merged_tolerance;
    store.get_mut(shell)?.faces.retain(|f| *f != kill);
    store.remove(kill)?;
    Ok(ring)
}

/// Make face, kill ring hole: detach an inner loop of a face into a new
/// face on `surface` in the same shell. Inverse of KFMRH; lowers the
/// shell's genus by one. Returns the new face.
pub(crate) fn mfkrh(
    store: &mut Store,
    ring: LoopId,
    surface: SurfaceId,
    sense: Sense,
) -> Result<FaceId> {
    store.get(surface)?;
    let old_face = store.get(ring)?.face;
    if store.get(old_face)?.loops.len() < 2 {
        return topo_err("mfkrh: loop must be an inner loop (face needs another loop)");
    }
    if surface != store.get(old_face)?.surface {
        validate_fins_on_surface(store, &store.get(ring)?.fins, surface)?;
    }
    let shell = store.get(old_face)?.shell;
    let old = store.get(old_face)?;
    let domain = if surface == old.surface {
        old.domain
    } else {
        FaceDomain::natural(store.get(surface)?)
    };
    let tolerance = old.tolerance;
    let face = store.add(Face {
        shell,
        loops: vec![ring],
        surface,
        sense,
        domain,
        tolerance,
    });
    store.get_mut(old_face)?.loops.retain(|l| *l != ring);
    store.get_mut(ring)?.face = face;
    store.get_mut(shell)?.faces.push(face);
    Ok(face)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check_body;
    use crate::entity::{Body, Region, Shell};
    use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
    use crate::make::block;
    use kgeom::curve::Line;
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::param::ParamRange;
    use kgeom::surface::Plane;
    use kgeom::vec::{Point2, Point3, Vec2, Vec3};

    /// Deterministic xorshift (same family as the determinism harness).
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }
        fn below(&mut self, n: usize) -> usize {
            (self.next() % n as u64) as usize
        }
    }

    /// Dummy geometry to attach; Euler operators never look inside it.
    fn dummy(store: &mut Store) -> (crate::entity::PointId, CurveId, SurfaceId) {
        let point = store.add(Point3::new(0.0, 0.0, 0.0));
        let line = Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let curve = store
            .insert_curve(crate::geom::CurveGeom::Line(line))
            .unwrap();
        let surface = store
            .insert_surface(crate::geom::SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        (point, curve, surface)
    }

    fn planar_line_inputs(
        store: &mut Store,
        start: Point3,
        end: Point3,
        uv_shift: Vec2,
    ) -> (CurveId, (f64, f64), FinPcurvePair) {
        let delta = end - start;
        let length = delta.norm();
        let curve = store
            .insert_curve(CurveGeom::Line(Line::new(start, delta).unwrap()))
            .unwrap();
        let uv_start = Point2::new(start.x, start.y) + uv_shift;
        let uv_delta = Vec2::new(delta.x, delta.y);
        let uv_length = uv_delta.norm();
        let make_use = |store: &mut Store| {
            let pcurve = store
                .insert_pcurve(Curve2dGeom::Line(Line2d::new(uv_start, uv_delta).unwrap()))
                .unwrap();
            FinPcurve::new(
                pcurve,
                ParamRange::new(0.0, uv_length),
                crate::entity::ParamMap1d::affine(uv_length / length, 0.0).unwrap(),
            )
            .unwrap()
        };
        (
            curve,
            (0.0, length),
            FinPcurvePair::new(make_use(store), make_use(store)),
        )
    }

    fn pcurved_square(store: &mut Store) -> (BodyId, FaceId, FaceId) {
        let points = [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ];
        let surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let seed = store.add(points[0]);
        let made = mvfs(store, surface, Sense::Forward, seed).unwrap();
        let mut tail = made.vertex;
        for i in 0..3 {
            let (curve, bounds, pcurves) =
                planar_line_inputs(store, points[i], points[i + 1], Vec2::default());
            let point = store.add(points[i + 1]);
            let at = if i == 0 {
                0
            } else {
                index_with_tail(store, made.ring, tail)
            };
            tail = mev_with_pcurves(store, made.ring, at, curve, bounds, point, pcurves)
                .unwrap()
                .vertex;
        }
        let i = index_with_tail(store, made.ring, tail);
        let j = index_with_tail(store, made.ring, made.vertex);
        let (curve, bounds, pcurves) =
            planar_line_inputs(store, points[3], points[0], Vec2::default());
        let split = mef_with_pcurves(
            store,
            made.ring,
            i,
            j,
            curve,
            bounds,
            surface,
            Sense::Reversed,
            pcurves,
        )
        .unwrap();
        (made.body, store.get(made.ring).unwrap().face, split.face)
    }

    /// `V − E + F = 2(S − G) + R`, rearranged over total loop count `L`
    /// (R = L − F): `V − E + 2F − L = 2(S − G)`.
    fn assert_euler_identity(store: &Store, body: BodyId, genus: i64) {
        let v = store.vertices_of_body(body).unwrap().len() as i64;
        let e = store.edges_of_body(body).unwrap().len() as i64;
        let faces = store.faces_of_body(body).unwrap();
        let f = faces.len() as i64;
        let l: i64 = faces
            .iter()
            .map(|&fc| store.get(fc).unwrap().loops.len() as i64)
            .sum();
        let s: i64 = store
            .get(body)
            .unwrap()
            .regions
            .iter()
            .map(|&r| store.get(r).unwrap().shells.len() as i64)
            .sum();
        assert_eq!(
            v - e + 2 * f - l,
            2 * (s - genus),
            "Euler–Poincaré identity violated (V={v} E={e} F={f} L={l} S={s} G={genus})"
        );
    }

    /// Structural health: rings closed, parents consistent, edges paired.
    fn assert_structurally_sound(store: &Store, body: BodyId) {
        for face in store.faces_of_body(body).unwrap() {
            assert_eq!(
                store
                    .get(store.get(face).unwrap().shell)
                    .unwrap()
                    .faces
                    .iter()
                    .filter(|f| **f == face)
                    .count(),
                1
            );
            for &lp in &store.get(face).unwrap().loops {
                assert_eq!(store.get(lp).unwrap().face, face, "loop back-pointer");
                let fins = &store.get(lp).unwrap().fins;
                for (k, &fin) in fins.iter().enumerate() {
                    assert_eq!(store.get(fin).unwrap().parent, lp, "fin back-pointer");
                    let next = fins[(k + 1) % fins.len()];
                    assert_eq!(
                        store.fin_head(fin).unwrap(),
                        store.fin_tail(next).unwrap(),
                        "loop ring must close"
                    );
                }
            }
        }
        for edge in store.edges_of_body(body).unwrap() {
            let e = store.get(edge).unwrap();
            assert_eq!(e.fins.len(), 2, "edge fin pairing");
            for &fin in &e.fins {
                assert_eq!(store.get(fin).unwrap().edge, edge, "fin edge back-pointer");
            }
            let s0 = store.get(e.fins[0]).unwrap().sense;
            let s1 = store.get(e.fins[1]).unwrap().sense;
            assert_ne!(s0, s1, "fins traverse the edge oppositely");
        }
    }

    /// Find the loop index whose fin's tail is `v`.
    fn index_with_tail(store: &Store, lp: LoopId, v: VertexId) -> usize {
        let fins = store.get(lp).unwrap().fins.clone();
        fins.iter()
            .position(|&f| store.fin_tail(f).unwrap() == Some(v))
            .expect("vertex on loop")
    }

    /// Build the topology of a block purely via Euler operators, checking
    /// the identity after every step. Returns (body, per-step counts).
    fn block_via_euler(store: &mut Store) -> BodyId {
        let (pt, cv, sf) = dummy(store);
        let m = mvfs(store, sf, Sense::Forward, pt).unwrap();
        let body = m.body;
        assert_euler_identity(store, body, 0);

        // Bottom square: three struts then a closing mef.
        let v0 = m.vertex;
        let e01 = mev(store, m.ring, 0, cv, (0.0, 1.0), pt).unwrap();
        assert_euler_identity(store, body, 0);
        let at = index_with_tail(store, m.ring, e01.vertex);
        let e12 = mev(store, m.ring, at, cv, (0.0, 1.0), pt).unwrap();
        assert_euler_identity(store, body, 0);
        let at = index_with_tail(store, m.ring, e12.vertex);
        let e23 = mev(store, m.ring, at, cv, (0.0, 1.0), pt).unwrap();
        assert_euler_identity(store, body, 0);
        let i = index_with_tail(store, m.ring, e23.vertex);
        let j = index_with_tail(store, m.ring, v0);
        let close = mef(store, m.ring, i, j, cv, (0.0, 1.0), sf, Sense::Forward).unwrap();
        assert_euler_identity(store, body, 0);
        assert_structurally_sound(store, body);

        // The new loop holds the back fins: it is the growing "outer"
        // face that will become the top. Sprout the four verticals.
        let outer = close.ring;
        let bottom = [v0, e01.vertex, e12.vertex, e23.vertex];
        let mut top = Vec::new();
        for &v in &bottom {
            let at = index_with_tail(store, outer, v);
            let up = mev(store, outer, at, cv, (0.0, 1.0), pt).unwrap();
            top.push(up.vertex);
            assert_euler_identity(store, body, 0);
        }
        // Four side faces: connect consecutive top vertices.
        for k in 0..4 {
            let i = index_with_tail(store, outer, top[k]);
            let j = index_with_tail(store, outer, top[(k + 1) % 4]);
            mef(store, outer, j, i, cv, (0.0, 1.0), sf, Sense::Forward).unwrap();
            assert_euler_identity(store, body, 0);
            assert_structurally_sound(store, body);
        }
        body
    }

    #[test]
    fn block_topology_via_euler_operators() {
        let mut store = Store::new();
        let body = block_via_euler(&mut store);
        assert_eq!(store.vertices_of_body(body).unwrap().len(), 8);
        assert_eq!(store.edges_of_body(body).unwrap().len(), 12);
        assert_eq!(store.faces_of_body(body).unwrap().len(), 6);
        for face in store.faces_of_body(body).unwrap() {
            let loops = &store.get(face).unwrap().loops;
            assert_eq!(loops.len(), 1);
            assert_eq!(store.get(loops[0]).unwrap().fins.len(), 4);
        }
        assert_structurally_sound(&store, body);
    }

    #[test]
    fn pcurve_aware_mev_and_mef_build_checker_clean_incidence() {
        let mut store = Store::new();
        let (body, _, _) = pcurved_square(&mut store);
        for edge in store.edges_of_body(body).unwrap() {
            for &fin in &store.get(edge).unwrap().fins {
                assert!(store.get(fin).unwrap().pcurve.is_some());
            }
        }
        let faults = check_body(&store, body).unwrap();
        assert!(faults.is_empty(), "pcurved Euler square faults: {faults:?}");
    }

    #[test]
    fn bad_pcurve_preflight_leaves_mev_state_unchanged() {
        let mut store = Store::new();
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let seed = store.add(p0);
        let made = mvfs(&mut store, surface, Sense::Forward, seed).unwrap();
        let (curve, bounds, bad_pcurves) =
            planar_line_inputs(&mut store, p0, p1, Vec2::new(0.0, 1.0));
        let point = store.add(p1);
        let counts = (
            store.count::<Edge>(),
            store.count::<Vertex>(),
            store.count::<Fin>(),
        );
        let seed_slot = store.get(made.shell).unwrap().vertex;

        let result = mev_with_pcurves(&mut store, made.ring, 0, curve, bounds, point, bad_pcurves);
        assert!(result.is_err());
        assert_eq!(
            counts,
            (
                store.count::<Edge>(),
                store.count::<Vertex>(),
                store.count::<Fin>()
            )
        );
        assert_eq!(store.get(made.shell).unwrap().vertex, seed_slot);
        assert!(store.get(made.ring).unwrap().fins.is_empty());
    }

    #[test]
    fn face_moves_preflight_existing_pcurves_atomically() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let faces = store.faces_of_body(body).unwrap();
        let shell = store.get(faces[0]).unwrap().shell;
        let shell_faces = store.get(shell).unwrap().faces.clone();
        let keep_loops = store.get(faces[0]).unwrap().loops.clone();
        let kill_loops = store.get(faces[1]).unwrap().loops.clone();

        assert!(kfmrh(&mut store, faces[0], faces[1]).is_err());
        assert_eq!(store.get(shell).unwrap().faces, shell_faces);
        assert_eq!(store.get(faces[0]).unwrap().loops, keep_loops);
        assert_eq!(store.get(faces[1]).unwrap().loops, kill_loops);
    }

    #[test]
    fn pcurve_aware_mekr_and_kemr_preserve_fin_uses() {
        let mut store = Store::new();
        let (body, keep, kill) = pcurved_square(&mut store);
        let ring = kfmrh(&mut store, keep, kill).unwrap();
        let outer = store.get(keep).unwrap().loops[0];
        let outer_fin = store.get(outer).unwrap().fins[0];
        let start_vertex = store.fin_tail(outer_fin).unwrap().unwrap();
        let start = store.vertex_position(start_vertex).unwrap();
        let ring_fins = store.get(ring).unwrap().fins.clone();
        let (j, end) = ring_fins
            .iter()
            .enumerate()
            .find_map(|(j, &fin)| {
                let vertex = store.fin_tail(fin).ok().flatten()?;
                let point = store.vertex_position(vertex).ok()?;
                (point != start).then_some((j, point))
            })
            .unwrap();
        let (curve, bounds, pcurves) = planar_line_inputs(&mut store, start, end, Vec2::default());
        let joined =
            mekr_with_pcurves(&mut store, outer, 0, ring, j, curve, bounds, pcurves).unwrap();
        assert!(store.get(joined.fin_out).unwrap().pcurve.is_some());
        assert!(store.get(joined.fin_back).unwrap().pcurve.is_some());
        let restored_ring = kemr(&mut store, joined.edge).unwrap();
        assert!(store.get(restored_ring).is_ok());
        assert_euler_identity(&store, body, 1);
    }

    #[test]
    fn mvfs_kvfs_roundtrip_restores_empty_topology() {
        let mut store = Store::new();
        let (pt, _, sf) = dummy(&mut store);
        let m = mvfs(&mut store, sf, Sense::Forward, pt).unwrap();
        assert_euler_identity(&store, m.body, 0);
        kvfs(&mut store, m.body).unwrap();
        assert_eq!(store.count::<Body>(), 0);
        assert_eq!(store.count::<Region>(), 0);
        assert_eq!(store.count::<Shell>(), 0);
        assert_eq!(store.count::<Face>(), 0);
        assert_eq!(store.count::<Loop>(), 0);
        assert_eq!(store.count::<Vertex>(), 0);
        // Geometry is never killed.
        assert_eq!(store.count::<crate::geom::SurfaceGeom>(), 1);
    }

    #[test]
    fn kev_inverts_mev() {
        let mut store = Store::new();
        let (pt, cv, sf) = dummy(&mut store);
        let m = mvfs(&mut store, sf, Sense::Forward, pt).unwrap();
        let e1 = mev(&mut store, m.ring, 0, cv, (0.0, 1.0), pt).unwrap();
        let at = index_with_tail(&store, m.ring, e1.vertex);
        let e2 = mev(&mut store, m.ring, at, cv, (0.0, 1.0), pt).unwrap();
        let (edges, vertices) = (store.count::<Edge>(), store.count::<Vertex>());
        kev(&mut store, e2.edge).unwrap();
        assert_eq!(store.count::<Edge>(), edges - 1);
        assert_eq!(store.count::<Vertex>(), vertices - 1);
        assert_euler_identity(&store, m.body, 0);
        kev(&mut store, e1.edge).unwrap();
        // Back to the minimal body: seed vertex restored to the shell.
        assert_eq!(store.get(m.shell).unwrap().vertex, Some(m.vertex));
        assert!(store.get(m.ring).unwrap().fins.is_empty());
        kvfs(&mut store, m.body).unwrap();
    }

    #[test]
    fn kev_failure_is_atomic_when_seed_slot_is_occupied() {
        let mut store = Store::new();
        let (pt, cv, sf) = dummy(&mut store);
        let m = mvfs(&mut store, sf, Sense::Forward, pt).unwrap();
        let made = mev(&mut store, m.ring, 0, cv, (0.0, 1.0), pt).unwrap();
        store.get_mut(m.shell).unwrap().vertex = Some(m.vertex);
        let ring_before = store.get(m.ring).unwrap().fins.clone();
        let counts_before = (
            store.count::<Edge>(),
            store.count::<Vertex>(),
            store.count::<Fin>(),
        );

        assert!(kev(&mut store, made.edge).is_err());
        assert_eq!(store.get(m.ring).unwrap().fins, ring_before);
        assert_eq!(
            counts_before,
            (
                store.count::<Edge>(),
                store.count::<Vertex>(),
                store.count::<Fin>(),
            )
        );
        assert!(store.get(made.edge).is_ok());
        assert!(store.get(made.vertex).is_ok());
    }

    #[test]
    fn kef_inverts_mef() {
        let mut store = Store::new();
        let (pt, cv, sf) = dummy(&mut store);
        let m = mvfs(&mut store, sf, Sense::Forward, pt).unwrap();
        let e1 = mev(&mut store, m.ring, 0, cv, (0.0, 1.0), pt).unwrap();
        let at = index_with_tail(&store, m.ring, e1.vertex);
        mev(&mut store, m.ring, at, cv, (0.0, 1.0), pt).unwrap();
        let split = mef(&mut store, m.ring, 0, 2, cv, (0.0, 1.0), sf, Sense::Forward).unwrap();
        assert_euler_identity(&store, m.body, 0);
        assert_structurally_sound(&store, m.body);
        let (f, l, e) = (
            store.count::<Face>(),
            store.count::<Loop>(),
            store.count::<Edge>(),
        );
        kef(&mut store, split.edge).unwrap();
        assert_eq!(store.count::<Face>(), f - 1);
        assert_eq!(store.count::<Loop>(), l - 1);
        assert_eq!(store.count::<Edge>(), e - 1);
        assert_euler_identity(&store, m.body, 0);
        assert_structurally_sound(&store, m.body);
    }

    #[test]
    fn kemr_and_mekr_are_inverses() {
        let mut store = Store::new();
        let body = block_via_euler(&mut store);
        let (pt, cv, _) = dummy(&mut store);
        let _ = pt;
        // Split one face's loop with a diagonal, then kill that diagonal
        // as kemr: not applicable (fins in different loops after mef).
        // Instead: join two loops with mekr after kfmrh makes a ring.
        let faces = store.faces_of_body(body).unwrap();
        let ring = kfmrh(&mut store, faces[0], faces[1]).unwrap();
        assert_euler_identity(&store, body, 1);
        let outer = store.get(faces[0]).unwrap().loops[0];
        let joined = mekr(&mut store, outer, 0, ring, 0, cv, (0.0, 1.0)).unwrap();
        assert_euler_identity(&store, body, 1);
        assert_structurally_sound(&store, body);
        // kemr splits them apart again.
        let ring2 = kemr(&mut store, joined.edge).unwrap();
        assert_euler_identity(&store, body, 1);
        assert_structurally_sound(&store, body);
        assert!(store.get(ring2).is_ok());
    }

    #[test]
    fn kfmrh_and_mfkrh_track_genus() {
        let mut store = Store::new();
        let body = block_via_euler(&mut store);
        let (_, _, sf) = dummy(&mut store);
        let faces = store.faces_of_body(body).unwrap();
        assert_euler_identity(&store, body, 0);
        let ring = kfmrh(&mut store, faces[0], faces[1]).unwrap();
        assert_euler_identity(&store, body, 1);
        assert_structurally_sound(&store, body);
        mfkrh(&mut store, ring, sf, Sense::Forward).unwrap();
        assert_euler_identity(&store, body, 0);
        assert_structurally_sound(&store, body);
    }

    #[test]
    fn random_operator_sequences_preserve_the_identity() {
        let mut store = Store::new();
        let (pt, cv, sf) = dummy(&mut store);
        let m = mvfs(&mut store, sf, Sense::Forward, pt).unwrap();
        let body = m.body;
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);

        for step in 0..300 {
            let faces = store.faces_of_body(body).unwrap();
            let loops: Vec<LoopId> = faces
                .iter()
                .flat_map(|&f| store.get(f).unwrap().loops.clone())
                .collect();
            let op = rng.below(4);
            match op {
                // mev: sprout somewhere legal.
                0 => {
                    let lp = loops[rng.below(loops.len())];
                    let n = store.get(lp).unwrap().fins.len();
                    if n == 0 {
                        let shell = store.get(store.get(lp).unwrap().face).unwrap().shell;
                        if store.get(shell).unwrap().vertex.is_some() {
                            mev(&mut store, lp, 0, cv, (0.0, 1.0), pt).unwrap();
                        }
                    } else {
                        mev(&mut store, lp, rng.below(n), cv, (0.0, 1.0), pt).unwrap();
                    }
                }
                // mef: split a loop anywhere.
                1 => {
                    let lp = loops[rng.below(loops.len())];
                    let n = store.get(lp).unwrap().fins.len();
                    if n > 0 {
                        let (i, j) = (rng.below(n), rng.below(n));
                        mef(&mut store, lp, i, j, cv, (0.0, 1.0), sf, Sense::Forward).unwrap();
                    }
                }
                // kev: find a strut whose head vertex is free.
                2 => {
                    let candidate = store
                        .edges_of_body(body)
                        .unwrap()
                        .into_iter()
                        .find(|&e| kev_applicable(&store, e));
                    if let Some(e) = candidate {
                        kev(&mut store, e).unwrap();
                    }
                }
                // kef: find an edge separating two faces (absorbee single-loop).
                _ => {
                    let candidate = store
                        .edges_of_body(body)
                        .unwrap()
                        .into_iter()
                        .find(|&e| kef_applicable(&store, e));
                    if let Some(e) = candidate {
                        kef(&mut store, e).unwrap();
                    }
                }
            }
            assert_euler_identity(&store, body, 0);
            assert_structurally_sound(&store, body);
            let _ = step;
        }
    }

    /// Would `kev` accept this edge? (Mirror of its preconditions.)
    fn kev_applicable(store: &Store, edge: EdgeId) -> bool {
        let e = store.get(edge).unwrap();
        let [f0, f1] = e.fins[..] else { return false };
        let lp = store.get(f0).unwrap().parent;
        if store.get(f1).unwrap().parent != lp {
            return false;
        }
        let fins = store.get(lp).unwrap().fins.clone();
        let n = fins.len();
        let adjacent = (0..n).any(|i| {
            (fins[i] == f0 && fins[(i + 1) % n] == f1) || (fins[i] == f1 && fins[(i + 1) % n] == f0)
        });
        if !adjacent {
            return false;
        }
        let first = (0..n)
            .find_map(|i| {
                let a = fins[i];
                let b = fins[(i + 1) % n];
                ((a == f0 && b == f1) || (a == f1 && b == f0)).then_some(a)
            })
            .unwrap();
        let Ok(Some(dying)) = store.fin_head(first) else {
            return false;
        };
        let Ok(Some(survivor)) = store.fin_tail(first) else {
            return false;
        };
        if dying == survivor {
            return false;
        }
        !store
            .iter::<Edge>()
            .any(|(h, other)| h != edge && other.vertices.contains(&Some(dying)))
    }

    /// Would `kef` accept this edge? (Mirror of its preconditions.)
    fn kef_applicable(store: &Store, edge: EdgeId) -> bool {
        let e = store.get(edge).unwrap();
        let [fa, fb] = e.fins[..] else { return false };
        let la = store.get(fa).unwrap().parent;
        let lb = store.get(fb).unwrap().parent;
        let face_a = store.get(la).unwrap().face;
        let face_b = store.get(lb).unwrap().face;
        face_a != face_b && store.get(face_b).unwrap().loops.len() == 1
    }

    #[test]
    fn operators_reject_bad_preconditions() {
        let mut store = Store::new();
        let (pt, cv, sf) = dummy(&mut store);
        let m = mvfs(&mut store, sf, Sense::Forward, pt).unwrap();
        // mev on empty loop demands index 0.
        assert!(mev(&mut store, m.ring, 1, cv, (0.0, 1.0), pt).is_err());
        // Bad bounds.
        assert!(mev(&mut store, m.ring, 0, cv, (1.0, 0.0), pt).is_err());
        assert!(mev(&mut store, m.ring, 0, cv, (0.0, f64::NAN), pt).is_err());
        // mef on the empty loop.
        assert!(mef(&mut store, m.ring, 0, 0, cv, (0.0, 1.0), sf, Sense::Forward).is_err());
        // kvfs refuses a non-minimal body.
        mev(&mut store, m.ring, 0, cv, (0.0, 1.0), pt).unwrap();
        assert!(kvfs(&mut store, m.body).is_err());
    }
}
