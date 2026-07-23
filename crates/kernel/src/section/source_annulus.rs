//! Reusable topology evidence for one whole cylindrical source annulus.
//!
//! The certificate retains the authored cap-ring topology and parameter maps.
//! Consumers may therefore select a ring by its authored axial bound and map
//! a canonical source longitude back to the ring edge's intrinsic parameter
//! without reconstructing topology or comparing derived points.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kgeom::curve::Circle;
use kgeom::curve2d::Line2d;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, FinPcurve, LoopId as RawLoopId,
    ParamMap1d,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::SECTION_WORK;
use crate::FaceId;
use crate::error::{Error, Result as KernelResult};

const PERIOD: f64 = core::f64::consts::TAU;

/// Stable fail-closed result for malformed or unsupported source-ring
/// topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SourceAnnulusTopologyGap;

/// One topology-certified cap ring in its source face's stored loop order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedSourceRing {
    face: RawFaceId,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    circle: Circle,
    pcurve: Line2d,
    fin_pcurve: FinPcurve,
    winding: i32,
    authored_height: f64,
}

impl CertifiedSourceRing {
    /// Raw cylindrical side face that owns the ring loop.
    pub(crate) const fn face(&self) -> RawFaceId {
        self.face
    }

    /// Raw single-fin loop on the cylindrical side face.
    pub(crate) const fn loop_id(&self) -> RawLoopId {
        self.loop_id
    }

    /// Raw fin whose pcurve parameterizes the whole source ring.
    pub(crate) const fn fin(&self) -> RawFinId {
        self.fin
    }

    /// Raw vertex-less ring edge.
    pub(crate) const fn edge(&self) -> RawEdgeId {
        self.edge
    }

    /// Intrinsic circular geometry carried by the source edge.
    pub(crate) const fn circle(&self) -> Circle {
        self.circle
    }

    /// Authored horizontal whole-period pcurve line.
    pub(crate) const fn pcurve(&self) -> Line2d {
        self.pcurve
    }

    /// Complete authored fin pcurve use, including its chart.
    pub(crate) const fn fin_pcurve(&self) -> FinPcurve {
        self.fin_pcurve
    }

    /// Affine intrinsic-edge to pcurve-parameter correspondence.
    pub(crate) fn edge_to_pcurve(&self) -> ParamMap1d {
        self.fin_pcurve.edge_to_pcurve()
    }

    /// Loop traversal winding after composing the fin sense.
    pub(crate) const fn winding(&self) -> i32 {
        self.winding
    }

    /// Exact authored constant `v` coordinate of this ring pcurve.
    pub(crate) const fn authored_height(&self) -> f64 {
        self.authored_height
    }

    /// Map an outward canonical source-longitude enclosure into the
    /// intrinsic source-edge parameter.
    ///
    /// Exactly one integer-period lift must lie strictly inside the fin's
    /// half-open active pcurve range. This rejects seam-straddling and
    /// ambiguous roots. Both the pcurve line and edge map may be reversed;
    /// interval division preserves outward ordering in either case.
    pub(crate) fn intrinsic_edge_parameter_for_longitude(
        &self,
        longitude: Interval,
    ) -> Option<Interval> {
        intrinsic_edge_parameter_for_longitude(longitude, self.pcurve, self.fin_pcurve)
    }
}

/// Certified pair of cap rings bounding one cylindrical source face.
///
/// `face_order` preserves stored topology order. `lower` and `upper` are
/// selected independently by the exact authored constant pcurve height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedSourceAnnulus {
    face_order: [CertifiedSourceRing; 2],
    lower_index: usize,
}

impl CertifiedSourceAnnulus {
    /// Rings in the source face's stored loop order.
    pub(crate) const fn face_order(&self) -> &[CertifiedSourceRing; 2] {
        &self.face_order
    }

    /// Ring at the exact lower authored axial bound.
    pub(crate) const fn lower(&self) -> &CertifiedSourceRing {
        &self.face_order[self.lower_index]
    }

    /// Ring at the exact upper authored axial bound.
    pub(crate) const fn upper(&self) -> &CertifiedSourceRing {
        &self.face_order[1 - self.lower_index]
    }
}

/// Certify exactly two whole circular cap rings on one source face.
///
/// This topology-only proof is uncharged so callers that already precharged
/// their face-local ceiling can reuse it without double accounting.
pub(crate) fn certify_source_annulus_topology(
    store: &Store,
    face: &FaceId,
    linear: f64,
) -> Result<CertifiedSourceAnnulus, SourceAnnulusTopologyGap> {
    let raw = store
        .get(face.raw())
        .map_err(|_| SourceAnnulusTopologyGap)?;
    let [first_loop, second_loop] = raw.loops() else {
        return Err(SourceAnnulusTopologyGap);
    };
    let face_order = [
        certify_source_ring(store, face.raw(), *first_loop, linear)?,
        certify_source_ring(store, face.raw(), *second_loop, linear)?,
    ];
    if face_order[0].winding.signum() == face_order[1].winding.signum()
        || !face_order
            .iter()
            .all(|ring| ring.authored_height.is_finite())
        || face_order[0].authored_height == face_order[1].authored_height
    {
        return Err(SourceAnnulusTopologyGap);
    }
    let lower_index = usize::from(face_order[1].authored_height < face_order[0].authored_height);
    Ok(CertifiedSourceAnnulus {
        face_order,
        lower_index,
    })
}

/// Prove that one cylinder face is exactly the untrimmed full-period annulus
/// described by its conservative face domain.
///
/// Two work units are charged before inspecting topology, so malformed and
/// admitted faces consume the same operation-local source-ring ceiling.
pub(crate) fn certify_source_annulus_window_in_scope(
    store: &Store,
    face: &FaceId,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> KernelResult<Option<CertifiedSourceAnnulus>> {
    scope
        .ledger_mut()
        .charge(SECTION_WORK, 2)
        .map_err(Error::from)?;
    let Ok(annulus) = certify_source_annulus_topology(store, face, linear) else {
        return Ok(None);
    };
    let Ok(raw) = store.get(face.raw()) else {
        return Ok(None);
    };
    let Some(domain) = raw.domain() else {
        return Ok(None);
    };
    if !matches!(store.get(raw.surface()), Ok(SurfaceGeom::Cylinder(_)))
        || !domain.u.is_finite()
        || !domain.v.is_finite()
        || domain.u.width() != PERIOD
    {
        return Ok(None);
    }
    let expected_low_winding = if raw.sense().is_forward() { 1 } else { -1 };
    if annulus.lower().authored_height != domain.v.lo
        || annulus.upper().authored_height != domain.v.hi
        || annulus.lower().winding != expected_low_winding
        || annulus.upper().winding != -expected_low_winding
    {
        return Ok(None);
    }
    Ok(Some(annulus))
}

fn certify_source_ring(
    store: &Store,
    face: RawFaceId,
    loop_id: RawLoopId,
    linear: f64,
) -> Result<CertifiedSourceRing, SourceAnnulusTopologyGap> {
    let loop_ = store.get(loop_id).map_err(|_| SourceAnnulusTopologyGap)?;
    let [fin] = loop_.fins() else {
        return Err(SourceAnnulusTopologyGap);
    };
    if loop_.face() != face {
        return Err(SourceAnnulusTopologyGap);
    }
    let fin_data = store.get(*fin).map_err(|_| SourceAnnulusTopologyGap)?;
    if fin_data.parent() != loop_id
        || certify_whole_fin_incidence(store, face, loop_id, *fin, linear)
            != WholeFinIncidence::Certified
    {
        return Err(SourceAnnulusTopologyGap);
    }
    let edge = store
        .get(fin_data.edge())
        .map_err(|_| SourceAnnulusTopologyGap)?;
    let (Some(curve), Some(fin_pcurve)) = (edge.curve(), fin_data.pcurve()) else {
        return Err(SourceAnnulusTopologyGap);
    };
    let circle = match store.get(curve) {
        Ok(CurveGeom::Circle(circle)) => *circle,
        _ => return Err(SourceAnnulusTopologyGap),
    };
    if edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
        || edge.fins().len() != 2
        || !edge.fins().contains(fin)
        || fin_pcurve.seam().is_some()
        || fin_pcurve.chart().period_shifts()[1] != 0
    {
        return Err(SourceAnnulusTopologyGap);
    }
    let Some(winding) = fin_pcurve.closure_winding() else {
        return Err(SourceAnnulusTopologyGap);
    };
    if !matches!(winding, [1 | -1, 0]) {
        return Err(SourceAnnulusTopologyGap);
    }
    let pcurve = match store.get(fin_pcurve.curve()) {
        Ok(Curve2dGeom::Line(pcurve)) => *pcurve,
        _ => return Err(SourceAnnulusTopologyGap),
    };
    let active = fin_pcurve.range();
    let map = fin_pcurve.edge_to_pcurve();
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
    let rate = pcurve.dir().x * map.scale();
    if pcurve.dir().x == 0.0
        || pcurve.dir().y != 0.0
        || pcurve.dir().x.abs() * active.width() != PERIOD
        || edge_parameters.into_iter().any(|value| !value.is_finite())
        || edge_parameters[0].min(edge_parameters[1]) != 0.0
        || edge_parameters[0].max(edge_parameters[1]) != PERIOD
        || !(winding[0] == 1 && rate > 0.0 || winding[0] == -1 && rate < 0.0)
    {
        return Err(SourceAnnulusTopologyGap);
    }
    let topology_winding = if fin_data.sense().is_forward() {
        winding[0]
    } else {
        -winding[0]
    };
    Ok(CertifiedSourceRing {
        face,
        loop_id,
        fin: *fin,
        edge: fin_data.edge(),
        circle,
        pcurve,
        fin_pcurve,
        winding: topology_winding,
        authored_height: pcurve.origin().y,
    })
}

fn intrinsic_edge_parameter_for_longitude(
    longitude: Interval,
    pcurve: Line2d,
    fin_pcurve: FinPcurve,
) -> Option<Interval> {
    if !finite(longitude) {
        return None;
    }
    let active = fin_pcurve.range();
    let direction = pcurve.dir();
    let map = fin_pcurve.edge_to_pcurve();
    let chart_winding = fin_pcurve.chart().period_shifts()[0];
    let chart_shift = Interval::point(f64::from(chart_winding)) * Interval::point(PERIOD);
    let longitude_at = |parameter: f64| {
        Interval::point(pcurve.origin().x)
            + Interval::point(direction.x) * Interval::point(parameter)
            + chart_shift
    };
    let endpoints = [longitude_at(active.lo), longitude_at(active.hi)];
    if endpoints.into_iter().any(|value| !finite(value)) {
        return None;
    }
    let active_longitudes = Interval::new(
        endpoints[0].lo().min(endpoints[1].lo()),
        endpoints[0].hi().max(endpoints[1].hi()),
    );
    let possible_windings = (active_longitudes - longitude)
        .checked_div(Interval::point(PERIOD))
        .filter(|value| finite(*value))?;
    let first = possible_windings.lo().ceil();
    let last = possible_windings.hi().floor();
    if !first.is_finite()
        || !last.is_finite()
        || first > last
        || first < i32::MIN as f64
        || last > i32::MAX as f64
        || last - first > 2.0
    {
        return None;
    }

    let mut accepted = None;
    for winding in (first as i32)..=(last as i32) {
        let numerator = longitude + Interval::point(f64::from(winding)) * Interval::point(PERIOD)
            - chart_shift
            - Interval::point(pcurve.origin().x);
        let pcurve_parameter = numerator
            .checked_div(Interval::point(direction.x))
            .filter(|value| finite(*value))?;
        if pcurve_parameter.hi() < active.lo || pcurve_parameter.lo() >= active.hi {
            continue;
        }
        if pcurve_parameter.lo() < active.lo || pcurve_parameter.hi() >= active.hi {
            return None;
        }
        let edge_parameter = (pcurve_parameter - Interval::point(map.offset()))
            .checked_div(Interval::point(map.scale()))
            .filter(|value| finite(*value))?;
        if accepted.replace(edge_parameter).is_some() {
            return None;
        }
    }
    accepted
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

#[cfg(test)]
mod tests {
    use kgeom::param::ParamRange;
    use kgeom::vec::{Point2, Vec2};
    use ktopo::entity::{FinPcurve, ParamMap1d, PcurveChart};
    use ktopo::geom::Curve2dGeom;
    use ktopo::store::Store;

    use super::*;

    fn periodic_use(
        store: &mut Store,
        pcurve: Line2d,
        map: ParamMap1d,
        chart: [i32; 2],
    ) -> FinPcurve {
        let curve = store.insert_pcurve(Curve2dGeom::Line(pcurve)).unwrap();
        FinPcurve::new(curve, ParamRange::new(0.0, PERIOD), map)
            .unwrap()
            .with_chart(PcurveChart::shifted(chart))
    }

    #[test]
    fn longitude_mapping_preserves_outward_intervals_through_reversed_maps() {
        let mut store = Store::new();
        let reversed_line = Line2d::new(Point2::new(PERIOD, 0.0), Vec2::new(-1.0, 0.0)).unwrap();
        let reversed_line_use =
            periodic_use(&mut store, reversed_line, ParamMap1d::identity(), [-1, 0]);
        let mapped = intrinsic_edge_parameter_for_longitude(
            Interval::new(1.0_f64.next_down(), 1.0_f64.next_up()),
            reversed_line,
            reversed_line_use,
        )
        .unwrap();
        assert!(mapped.contains(PERIOD - 1.0));

        let forward_line = Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
        let reversed_edge_use = periodic_use(
            &mut store,
            forward_line,
            ParamMap1d::affine(-1.0, PERIOD).unwrap(),
            [3, 0],
        );
        let mapped = intrinsic_edge_parameter_for_longitude(
            Interval::new(1.0_f64.next_down(), 1.0_f64.next_up()),
            forward_line,
            reversed_edge_use,
        )
        .unwrap();
        assert!(mapped.contains(PERIOD - 1.0));
    }

    #[test]
    fn longitude_mapping_refuses_a_source_seam_interval() {
        let mut store = Store::new();
        let pcurve = Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
        let use_ = periodic_use(&mut store, pcurve, ParamMap1d::identity(), [0, 0]);
        assert!(
            intrinsic_edge_parameter_for_longitude(
                Interval::new((-f64::EPSILON).next_down(), f64::EPSILON.next_up()),
                pcurve,
                use_,
            )
            .is_none()
        );
    }
}
