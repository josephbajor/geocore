//! Certified membership in topology-owned bounded analytic trims.

use super::*;
use kgeom::frame::Frame;
use ktopo::bounded_trim::BoundedTrim;
use ktopo::entity::LoopId;

pub(super) fn prepare(
    store: &Store,
    face: RawFaceId,
    loop_id: LoopId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<BoundedTrim>> {
    let count = read(store.get(loop_id))?.fins().len();
    let Some(work) = ktopo::bounded_trim::preparation_work(count) else {
        return Ok(None);
    };
    charge(scope, work)?;
    read(ktopo::bounded_trim::prepare(store, face, loop_id))
}

pub(super) fn coordinates(frame: &Frame, point: [Interval; 3]) -> Option<[Interval; 3]> {
    let origin = as_coords(frame.origin());
    let relative = core::array::from_fn::<_, 3, _>(|i| point[i] - Interval::point(origin[i]));
    let basis = [frame.x(), frame.y(), frame.z()].map(|axis| as_coords(axis).map(Interval::point));
    let cross = |a: [Interval; 3], b: [Interval; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let dot = |a: [Interval; 3], b: [Interval; 3]| {
        (0..3)
            .map(|i| a[i] * b[i])
            .fold(Interval::point(0.0), |a, b| a + b)
    };
    let determinant = dot(basis[0], cross(basis[1], basis[2]));
    let mut result = [Interval::point(0.0); 3];
    for i in 0..3 {
        result[i] = dot(relative, cross(basis[(i + 1) % 3], basis[(i + 2) % 3]))
            .checked_div(determinant)?;
    }
    Some(result)
}

pub(super) fn plane_parity(face: &PreparedFace, point: [Interval; 3]) -> Option<bool> {
    if face.bounded_trims.is_empty() {
        return Some(false);
    }
    let uv = coordinates(face.trim_frame.as_deref()?, guard_point(point, face.guard))?;
    let mut parity = false;
    for trim in &face.bounded_trims {
        parity ^= trim.parity([uv[0], uv[1]])?;
    }
    Some(parity)
}

pub(super) fn cylinder_parity(
    frame: &Frame,
    trims: &[BoundedTrim],
    point: [f64; 3],
    guard: f64,
) -> Option<bool> {
    let [x, y, z] = coordinates(frame, guard_point(point.map(Interval::point), guard))?;
    let atan = |ratio: Interval| {
        let mut lo = kcore::math::atan(ratio.lo());
        let mut hi = kcore::math::atan(ratio.hi());
        for _ in 0..4 {
            lo = lo.next_down();
            hi = hi.next_up();
        }
        Interval::new(lo, hi)
    };
    let pi = Interval::new(
        core::f64::consts::PI.next_down(),
        core::f64::consts::PI.next_up(),
    );
    // Select a continuous angle chart before interval evaluation. In
    // particular, the negative-x seam is represented around +pi even when
    // the y enclosure straddles zero.
    let angle = if !x.contains_zero() {
        atan(y.checked_div(x)?)
            + if x.hi() < 0.0 {
                pi
            } else {
                Interval::point(0.0)
            }
    } else if !y.contains_zero() {
        (if y.lo() > 0.0 { pi } else { -pi }) * Interval::point(0.5) - atan(x.checked_div(y)?)
    } else {
        return None;
    };
    if !angle.lo().is_finite() || !angle.hi().is_finite() {
        return None;
    }
    let mut parity = false;
    for trim in trims {
        let bounds = trim.bounds();
        let period = Interval::point(core::f64::consts::TAU);
        let windings = (bounds[0] - angle).checked_div(period)?;
        let first = windings.lo().ceil();
        let last = windings.hi().floor();
        if !first.is_finite()
            || !last.is_finite()
            || first.abs().max(last.abs()) > i32::MAX as f64
            || last - first > 2.0
        {
            return None;
        }
        if first > last {
            continue;
        }
        for shift in first as i32..=last as i32 {
            parity ^= trim.parity([angle + Interval::point(f64::from(shift)) * period, z])?;
        }
    }
    Some(parity)
}

fn guard_point(point: [Interval; 3], guard: f64) -> [Interval; 3] {
    point.map(|value| value + Interval::new(-guard, guard))
}

pub(super) fn charge_query(
    trims: &[BoundedTrim],
    scope: &mut OperationScope<'_, '_>,
) -> Result<()> {
    let work = trims
        .iter()
        .try_fold(0_u64, |sum, trim| sum.checked_add(trim.query_work()?));
    let Some(work) = work else {
        return read(Err(kcore::error::Error::InvalidGeometry {
            reason: "bounded trim query work overflow",
        }));
    };
    charge(scope, work)
}
