//! Topology-owned contractible analytic trims in one certified surface chart.
//!
//! The interval cover encloses entire curve spans. Parity uses a deformation
//! to chords only after proving that every deformation box excludes the
//! query. It therefore never substitutes sampled geometry for a boundary.

use crate::entity::{EdgeId, FaceId, FinId, LoopId};
use crate::geom::Curve2dGeom;
use crate::loop_proof::{
    LoopSimplicity, certify_loop_orientation, certify_loop_simplicity,
    prepare_bounded_analytic_loop,
};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::math;
use kcore::predicates::Orientation;
use kgeom::vec::Point2;

/// An outward chart-coordinate enclosure.
pub type Box2 = [Interval; 2];

/// One complete, bounded topology-owned fin traversal.
#[derive(Debug, Clone)]
pub struct TrimSpan {
    /// Source fin.
    pub fin: FinId,
    /// Source edge.
    pub edge: EdgeId,
    /// Authored chart curve.
    pub curve: Curve2dGeom,
    /// Traversal endpoints in the authored pcurve parameter.
    pub parameters: [f64; 2],
    /// Certified whole-period chart translation.
    pub offset: Point2,
}

/// A simple closed bounded loop with topology-owned chart continuity.
#[derive(Debug, Clone)]
pub struct BoundedTrim {
    spans: Vec<TrimSpan>,
    orientation: Orientation,
    cover: Vec<(Box2, Box2, Box2)>,
}

impl BoundedTrim {
    /// Ordered fin traversals.
    pub fn spans(&self) -> &[TrimSpan] {
        &self.spans
    }
    /// Certified orientation in the lifted surface chart.
    pub fn orientation(&self) -> Orientation {
        self.orientation
    }
    /// Complete curve enclosures, useful for strict separation proofs.
    pub fn cover(&self) -> impl Iterator<Item = Box2> + '_ {
        self.cover.iter().map(|entry| entry.0)
    }
    /// Outward bounds of the complete closed trim.
    pub fn bounds(&self) -> Box2 {
        self.cover
            .iter()
            .skip(1)
            .fold(self.cover[0].0, |a, b| hull(a, b.0))
    }
    /// Maximum structural work for one parity query, including every ray
    /// candidate and every certified join connector.
    pub fn query_work(&self) -> Option<u64> {
        u64::try_from(self.cover.len()).ok()?.checked_mul(12)
    }
    /// Strict winding parity, or an unresolved boundary/arithmetic guard.
    pub fn parity(&self, query: Box2) -> Option<bool> {
        if !finite_box(query)
            || self
                .cover
                .iter()
                .any(|(cover, _, _)| !separated(*cover, query))
        {
            return None;
        }
        // Each fin join has already been certified from shared topology and
        // whole-fin incidence. Include its connector in the same exclusion
        // proof instead of pretending independently rounded endpoints agree.
        let mut chords = Vec::with_capacity(self.cover.len() * 2);
        for (index, (_, start, end)) in self.cover.iter().enumerate() {
            chords.push((*start, *end));
            let next = self.cover[(index + 1) % self.cover.len()].1;
            if !separated(hull(*end, next), query) {
                return None;
            }
            chords.push((*end, next));
        }
        for direction in [[1.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 2.0], [2.0, 1.0]] {
            if let Some(parity) = chord_parity(&chords, query, direction) {
                return Some(parity);
            }
        }
        None
    }
}

/// Conservative structural work bound including finite curve covers and
/// the topology-owned simplicity/incidence readers. Charge before preparing.
pub fn preparation_work(fin_count: usize) -> Option<u64> {
    let n = u64::try_from(fin_count).ok()?;
    n.checked_mul(n)?
        .checked_mul(64)?
        .checked_add(n.checked_mul(256)?)
}

/// Prepare a bounded analytic loop. Unsupported curves, winding, unresolved
/// chart joins, or simplicity/orientation failures return `None`.
pub fn prepare(store: &Store, face: FaceId, loop_id: LoopId) -> Result<Option<BoundedTrim>> {
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let Some(orientation) = certify_loop_orientation(store, face, loop_id)? else {
        return Ok(None);
    };
    let Some(prepared) = prepare_bounded_analytic_loop(store, face, loop_id)? else {
        return Ok(None);
    };
    let mut spans = Vec::with_capacity(prepared.len());
    let mut cover = Vec::new();
    for (&fin, prepared) in store.get(loop_id)?.fins.iter().zip(prepared) {
        let geometry = prepared.geometry();
        let span = TrimSpan {
            fin,
            edge: store.get(fin)?.edge,
            curve: geometry.curve().clone(),
            parameters: [geometry.start(), geometry.end()],
            offset: geometry.chart_offset(),
        };
        let count = match span.curve {
            Curve2dGeom::Line(_) => 1,
            Curve2dGeom::Circle(_) => {
                ((span.parameters[1] - span.parameters[0]).abs() / 0.125).ceil() as usize
            }
            _ => return Ok(None),
        };
        if count == 0 || count > 64 {
            return Ok(None);
        }
        let mut start = span.parameters[0];
        for index in 0..count {
            let end = if index + 1 == count {
                span.parameters[1]
            } else {
                span.parameters[0]
                    + (span.parameters[1] - span.parameters[0])
                        * ((index + 1) as f64 / count as f64)
            };
            let (Some(bounds), Some(a), Some(b)) = (
                span.enclose(Interval::new(start.min(end), start.max(end))),
                span.enclose(Interval::point(start)),
                span.enclose(Interval::point(end)),
            ) else {
                return Ok(None);
            };
            cover.push((bounds, a, b));
            start = end;
        }
        spans.push(span);
    }
    Ok(Some(BoundedTrim {
        spans,
        orientation,
        cover,
    }))
}

impl TrimSpan {
    /// Enclose the complete authored pcurve parameter interval in the lifted chart.
    pub fn enclose(&self, parameter: Interval) -> Option<Box2> {
        let offset = [
            Interval::point(self.offset.x),
            Interval::point(self.offset.y),
        ];
        let value = match &self.curve {
            Curve2dGeom::Line(line) => [
                Interval::point(line.origin().x)
                    + Interval::point(line.dir().x) * parameter
                    + offset[0],
                Interval::point(line.origin().y)
                    + Interval::point(line.dir().y) * parameter
                    + offset[1],
            ],
            Curve2dGeom::Circle(circle) => {
                if !finite(parameter) || parameter.width() > 0.126 {
                    return None;
                }
                let mid = 0.5 * parameter.lo() + 0.5 * parameter.hi();
                let delta = Interval::point(mid) - parameter;
                let radius = delta.lo().abs().max(delta.hi().abs());
                let (sin, cos) = math::sincos(mid);
                let widen = |v: f64| {
                    Interval::new(
                        (v.next_down() - radius).next_down().max(-1.0),
                        (v.next_up() + radius).next_up().min(1.0),
                    )
                };
                let (sin, cos) = (widen(sin), widen(cos));
                let x = circle.x_dir();
                let y = x.perp();
                let r = Interval::point(circle.radius());
                [
                    Interval::point(circle.center().x)
                        + r * (Interval::point(x.x) * cos + Interval::point(y.x) * sin)
                        + offset[0],
                    Interval::point(circle.center().y)
                        + r * (Interval::point(x.y) * cos + Interval::point(y.y) * sin)
                        + offset[1],
                ]
            }
            _ => return None,
        };
        finite_box(value).then_some(value)
    }
}

fn chord_parity(chords: &[(Box2, Box2)], query: Box2, direction: [f64; 2]) -> Option<bool> {
    let mut inside = false;
    let height =
        |p: Box2| Interval::point(direction[0]) * p[1] - Interval::point(direction[1]) * p[0];
    for &(a, b) in chords {
        let a = [a[0] - query[0], a[1] - query[1]];
        let b = [b[0] - query[0], b[1] - query[1]];
        let (ah, bh) = (height(a), height(b));
        if ah.lo() > 0.0 && bh.lo() > 0.0 || ah.hi() < 0.0 && bh.hi() < 0.0 {
            continue;
        }
        let upward = if ah.hi() < 0.0 && bh.lo() > 0.0 {
            true
        } else if bh.hi() < 0.0 && ah.lo() > 0.0 {
            false
        } else {
            return None;
        };
        let side = a[0] * b[1] - a[1] * b[0];
        if side.contains_zero() {
            return None;
        }
        inside ^= (side.lo() > 0.0) == upward;
    }
    Some(inside)
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}
fn finite_box(value: Box2) -> bool {
    value.into_iter().all(finite)
}
fn hull(a: Box2, b: Box2) -> Box2 {
    core::array::from_fn(|i| Interval::new(a[i].lo().min(b[i].lo()), a[i].hi().max(b[i].hi())))
}
fn separated(a: Box2, b: Box2) -> bool {
    (0..2).any(|i| a[i].hi() < b[i].lo() || b[i].hi() < a[i].lo())
}
