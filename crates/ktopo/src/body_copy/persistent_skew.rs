//! Proof-safe rigid copy of persistent finite-window skew-cylinder spans.

use super::*;
use kgraph::{
    PersistentSkewCylinderFiniteWindowFamilyCertificate, PersistentSkewCylinderOpenSpanPcurve,
    VerifiedSkewCylinderOpenSpanCurveDescriptor,
    reissue_persistent_skew_cylinder_finite_window_family,
};

impl Copier<'_> {
    pub(super) fn copy_persistent_skew_cylinder_open_span(
        &mut self,
        source: CurveId,
        descriptor: &VerifiedSkewCylinderOpenSpanCurveDescriptor,
    ) -> BodyCopyResult<CurveId> {
        let source_surfaces = descriptor.source_surfaces();
        let source_pcurves = descriptor.pcurves();
        let certificate = descriptor.certificate();
        let family = certificate
            .finite_window_family_membership()
            .ok_or_else(unsupported_family)?
            .family();
        if !persistent_dependencies_match(
            self.store,
            source_surfaces,
            source_pcurves,
            certificate,
            family,
        )? {
            return Err(unsupported_family());
        }
        let copied_surfaces = [
            self.copy_surface(source_surfaces[0])?,
            self.copy_surface(source_surfaces[1])?,
        ];
        let copied_source_cylinders = [
            copied_cylinder(self.store, copied_surfaces[0])?,
            copied_cylinder(self.store, copied_surfaces[1])?,
        ];
        let formula_to_source = family.formula_to_source();
        let transformed_formula_cylinders = [
            copied_source_cylinders[formula_to_source[0]],
            copied_source_cylinders[formula_to_source[1]],
        ];
        let reissue = self.reissued_persistent_family(family, transformed_formula_cylinders)?;
        let transformed_endpoint_points = self.transform_points(certificate.endpoint_points())?;
        let certificate = reissue
            .reissue_member(certificate, transformed_endpoint_points)
            .map_err(BodyCopyError::Certificate)?;
        let issued_pcurves = certificate.pcurves();
        let copied_pcurves = [
            self.copy_persistent_skew_pcurve(source_pcurves[0], issued_pcurves[0])?,
            self.copy_persistent_skew_pcurve(source_pcurves[1], issued_pcurves[1])?,
        ];
        let curve = self.store.insert_verified_skew_cylinder_open_span_curve(
            copied_surfaces,
            copied_pcurves,
            certificate,
        )?;
        self.register_curve_copy(source, curve);
        Ok(curve)
    }

    fn reissued_persistent_family(
        &mut self,
        source: PersistentSkewCylinderFiniteWindowFamilyCertificate,
        transformed_formula_cylinders: [Cylinder; 2],
    ) -> BodyCopyResult<kgraph::PersistentSkewCylinderFiniteWindowFamilyReissue> {
        if let Some(reissue) = self
            .persistent_skew_families
            .iter()
            .copied()
            .find(|reissue| reissue.source_family() == source)
        {
            if reissue.certificate().formula_cylinders() != transformed_formula_cylinders {
                return Err(unsupported_family());
            }
            return Ok(reissue);
        }
        let reissue = reissue_persistent_skew_cylinder_finite_window_family(
            source,
            transformed_formula_cylinders,
        )
        .map_err(BodyCopyError::Certificate)?;
        self.persistent_skew_families.push(reissue);
        Ok(reissue)
    }

    fn copy_persistent_skew_pcurve(
        &mut self,
        source: Curve2dId,
        pcurve: PersistentSkewCylinderOpenSpanPcurve,
    ) -> BodyCopyResult<Curve2dId> {
        if let Some(&copied) = self.pcurves.get(&source) {
            if self
                .store
                .get(copied)?
                .as_persistent_skew_cylinder_open_span()
                .copied()
                == Some(pcurve)
            {
                return Ok(copied);
            }
            return Err(unsupported_family());
        }
        let copied = self.store.insert_pcurve(pcurve.into())?;
        self.pcurves.insert(source, copied);
        self.derived(EntityRef::Curve2d(copied), EntityRef::Curve2d(source));
        Ok(copied)
    }
}

fn persistent_dependencies_match(
    store: &Store,
    source_surfaces: [SurfaceId; 2],
    source_pcurves: [Curve2dId; 2],
    certificate: kgraph::PersistentSkewCylinderOpenSpanCertificate,
    family: PersistentSkewCylinderFiniteWindowFamilyCertificate,
) -> BodyCopyResult<bool> {
    let expected_cylinders = family.source_cylinders();
    let expected_pcurves = certificate.pcurves();
    for index in 0..2 {
        if store.get(source_surfaces[index])?.as_cylinder().copied()
            != Some(expected_cylinders[index])
            || store
                .get(source_pcurves[index])?
                .as_persistent_skew_cylinder_open_span()
                .copied()
                != Some(expected_pcurves[index])
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn copied_cylinder(store: &Store, surface: SurfaceId) -> BodyCopyResult<Cylinder> {
    store
        .get(surface)?
        .as_cylinder()
        .copied()
        .ok_or_else(unsupported_family)
}

fn unsupported_family() -> BodyCopyError {
    BodyCopyError::Kernel(Error::InvalidGeometry {
        reason: "rigid body copy requires a live family-bound persistent skew-cylinder span",
    })
}
