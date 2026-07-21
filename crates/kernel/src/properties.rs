//! Facade-safe certified solid-body property interrogation.

use kcore::operation::OperationScope;

use crate::error::{Error, Result, capability};
use crate::operation::{OperationOutcome, OperationSettings, adapt_check_report};
use crate::session::Part;
use crate::{BodyId, CapabilityId, CheckReport, FaceId, PartId, Point3};

/// Typed request for certified solid-body volume, centroid, area, and inertia.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyPropertiesRequest {
    body: BodyId,
    settings: OperationSettings,
}

impl BodyPropertiesRequest {
    /// Construct a request using default operation settings.
    pub fn new(body: BodyId) -> Self {
        Self {
            body,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Body being interrogated.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// A finite certified scalar enclosure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScalarEnclosure {
    lower: f64,
    upper: f64,
}

impl ScalarEnclosure {
    fn from_lower(value: ktopo::body_properties::ScalarEnclosure) -> Self {
        Self {
            lower: value.lower(),
            upper: value.upper(),
        }
    }

    /// Certified inclusive lower bound.
    pub const fn lower(self) -> f64 {
        self.lower
    }

    /// Certified inclusive upper bound.
    pub const fn upper(self) -> f64 {
        self.upper
    }

    /// Deterministic midpoint representative.
    pub fn value(self) -> f64 {
        0.5 * self.lower + 0.5 * self.upper
    }

    /// Radius around [`Self::value`] containing the certified interval.
    pub fn error_bound(self) -> f64 {
        let value = self.value();
        (value - self.lower).max(self.upper - value).next_up()
    }

    /// Whether this enclosure certifies that it contains `value`.
    pub const fn contains(self, value: f64) -> bool {
        self.lower <= value && value <= self.upper
    }
}

/// Per-coordinate certified model-space point enclosure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3Enclosure {
    coordinates: [ScalarEnclosure; 3],
}

impl Point3Enclosure {
    fn from_lower(value: ktopo::body_properties::Point3Enclosure) -> Self {
        let coordinates = value.coordinates();
        Self {
            coordinates: [
                ScalarEnclosure::from_lower(coordinates[0]),
                ScalarEnclosure::from_lower(coordinates[1]),
                ScalarEnclosure::from_lower(coordinates[2]),
            ],
        }
    }

    /// Certified inclusive coordinate enclosures in `(x, y, z)` order.
    pub const fn coordinates(self) -> [ScalarEnclosure; 3] {
        self.coordinates
    }

    /// Deterministic midpoint representative.
    pub fn value(self) -> Point3 {
        Point3::new(
            self.coordinates[0].value(),
            self.coordinates[1].value(),
            self.coordinates[2].value(),
        )
    }

    /// Euclidean radius containing the certified coordinate box.
    pub fn error_bound(self) -> f64 {
        let mut squared_radius = 0.0_f64;
        for coordinate in self.coordinates {
            let radius = coordinate.error_bound();
            squared_radius = (squared_radius + radius * radius).next_up();
        }
        squared_radius.sqrt().next_up()
    }

    /// Whether each coordinate enclosure contains the supplied point.
    pub const fn contains(self, value: Point3) -> bool {
        self.coordinates[0].contains(value.x)
            && self.coordinates[1].contains(value.y)
            && self.coordinates[2].contains(value.z)
    }
}

/// Certified enclosure of one symmetric model-space tensor.
///
/// Components use `(xx, yy, zz, xy, xz, yz)` order. Mirrored matrix entries
/// share exactly the same scalar enclosure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SymmetricTensor3Enclosure {
    components: [ScalarEnclosure; 6],
}

impl SymmetricTensor3Enclosure {
    fn from_lower(value: ktopo::body_properties::SymmetricTensor3Enclosure) -> Self {
        let components = value.components();
        Self {
            components: [
                ScalarEnclosure::from_lower(components[0]),
                ScalarEnclosure::from_lower(components[1]),
                ScalarEnclosure::from_lower(components[2]),
                ScalarEnclosure::from_lower(components[3]),
                ScalarEnclosure::from_lower(components[4]),
                ScalarEnclosure::from_lower(components[5]),
            ],
        }
    }

    /// Certified components in `(xx, yy, zz, xy, xz, yz)` order.
    pub const fn components(self) -> [ScalarEnclosure; 6] {
        self.components
    }

    /// Certified symmetric matrix with bit-identical mirrored entries.
    pub const fn matrix(self) -> [[ScalarEnclosure; 3]; 3] {
        let [xx, yy, zz, xy, xz, yz] = self.components;
        [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    }

    /// Deterministic midpoint representative matrix.
    pub fn value(self) -> [[f64; 3]; 3] {
        self.matrix().map(|row| row.map(ScalarEnclosure::value))
    }

    /// Frobenius radius containing the six-component interval box.
    pub fn error_bound(self) -> f64 {
        let mut squared_radius = 0.0_f64;
        for (index, component) in self.components.into_iter().enumerate() {
            let radius = component.error_bound();
            let multiplicity = if index < 3 { 1.0 } else { 2.0 };
            let term = (multiplicity * radius * radius).next_up();
            squared_radius = (squared_radius + term).next_up();
        }
        squared_radius.sqrt().next_up()
    }

    /// Whether every supplied matrix entry lies in its certified enclosure.
    pub fn contains(self, value: [[f64; 3]; 3]) -> bool {
        let matrix = self.matrix();
        (0..3).all(|row| (0..3).all(|column| matrix[row][column].contains(value[row][column])))
    }
}

/// Certified properties of one opaque facade body.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyProperties {
    body: BodyId,
    volume: ScalarEnclosure,
    centroid: Point3Enclosure,
    surface_area: ScalarEnclosure,
    centroidal_inertia: SymmetricTensor3Enclosure,
}

impl BodyProperties {
    /// Exact body identity used by the query.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Certified positive volume enclosure.
    pub const fn volume(&self) -> ScalarEnclosure {
        self.volume
    }

    /// Certified model-space centroid enclosure.
    pub const fn centroid(&self) -> Point3Enclosure {
        self.centroid
    }

    /// Certified positive area of the complete material boundary.
    pub const fn surface_area(&self) -> ScalarEnclosure {
        self.surface_area
    }

    /// Certified unit-density inertia tensor about the true centroid.
    ///
    /// With `c` the true centroid, this is
    /// `integral (|r-c|^2 I - (r-c)(r-c)^T) dV`; off-diagonals use the
    /// standard negative-product convention and the units are length^5.
    pub const fn centroidal_inertia(&self) -> SymmetricTensor3Enclosure {
        self.centroidal_inertia
    }
}

/// Why a valid facade request did not produce certified properties.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodyPropertiesRefusal {
    /// The body is not a three-dimensional solid.
    NonSolidBody,
    /// Full validation found faults or unresolved proof obligations.
    BodyNotFullValid,
    /// Exact integration does not consume tolerant topology.
    TolerantTopology,
    /// A face uses a supporting surface outside the Plane/Cylinder slice.
    UnsupportedSurface {
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// A face boundary uses a pcurve outside the admitted analytic slice.
    UnsupportedPcurve {
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// Topology-owned loop preparation could not reissue its analytic proof.
    UncertifiedAnalyticBoundary {
        /// Opaque face identity at the proof boundary.
        face: FaceId,
    },
    /// Outward arithmetic did not prove a finite strictly positive volume.
    NonPositiveVolumeEnclosure,
    /// Oriented face-domain integration did not prove positive boundary area.
    NonPositiveSurfaceAreaEnclosure,
    /// Outward arithmetic did not produce a finite centroidal inertia tensor.
    InertiaEnclosureIndeterminate,
}

impl BodyPropertiesRefusal {
    /// Missing finite-support capability, when this is an unsupported case.
    pub const fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::TolerantTopology
            | Self::UnsupportedSurface { .. }
            | Self::UnsupportedPcurve { .. }
            | Self::UncertifiedAnalyticBoundary { .. } => {
                Some(capability::ANALYTIC_BODY_PROPERTIES)
            }
            Self::NonSolidBody
            | Self::BodyNotFullValid
            | Self::NonPositiveVolumeEnclosure
            | Self::NonPositiveSurfaceAreaEnclosure
            | Self::InertiaEnclosureIndeterminate => None,
        }
    }
}

/// Full-check evidence paired with certified properties or a typed refusal.
#[derive(Debug, Clone, PartialEq)]
pub enum BodyPropertiesOutcome {
    /// The Full checker and analytic boundary integral both certified.
    Certified {
        /// Certified properties.
        properties: BodyProperties,
        /// Full checker evidence consumed by the theorem.
        full_check: CheckReport,
    },
    /// The request was valid but outside the current proof boundary.
    Refused {
        /// Typed refusal reason.
        reason: BodyPropertiesRefusal,
        /// Full checker evidence, including non-valid reports.
        full_check: CheckReport,
    },
}

impl BodyPropertiesOutcome {
    /// Full checker report retained by either outcome.
    pub const fn full_check(&self) -> &CheckReport {
        match self {
            Self::Certified { full_check, .. } | Self::Refused { full_check, .. } => full_check,
        }
    }

    /// Certified properties, if the theorem completed.
    pub const fn properties(&self) -> Option<&BodyProperties> {
        match self {
            Self::Certified { properties, .. } => Some(properties),
            Self::Refused { .. } => None,
        }
    }

    /// Typed refusal, if the theorem failed closed.
    pub const fn refusal(&self) -> Option<&BodyPropertiesRefusal> {
        match self {
            Self::Certified { .. } => None,
            Self::Refused { reason, .. } => Some(reason),
        }
    }
}

impl Part<'_> {
    /// Certify one body's volume, centroid, area, and inertia in one scope.
    ///
    /// Wrong-part and stale identities plus invalid or incomplete policy
    /// configuration are rejected before the scope starts. The Full checker
    /// and boundary integrator then share the returned accounting report. The
    /// advertised property record is atomic; no partial record is returned.
    pub fn body_properties(
        &self,
        request: BodyPropertiesRequest,
    ) -> Result<OperationOutcome<BodyPropertiesOutcome>> {
        let BodyPropertiesRequest { body, settings } = request;
        self.body(body.clone())?;
        let defaults = ktopo::body_properties::BodyPropertiesBudgetProfile::v1_defaults();
        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(defaults.clone());
        let effective = context.effective_budget();
        for required in defaults.limits() {
            effective.require_limit(required.stage, required.resource, required.mode)?;
        }

        let mut scope = OperationScope::new(&context);
        let lower = ktopo::body_properties::certify_body_properties_in_scope(
            &self.state.store,
            body.raw(),
            &mut scope,
        );
        let result = lower
            .map_err(Error::from)
            .and_then(|outcome| adapt_outcome(&self.id, &self.state.store, body, outcome));
        Ok(scope.finish_typed(result))
    }
}

fn adapt_outcome(
    part: &PartId,
    store: &ktopo::store::Store,
    body: BodyId,
    outcome: ktopo::body_properties::BodyPropertiesOutcome,
) -> Result<BodyPropertiesOutcome> {
    Ok(match outcome {
        ktopo::body_properties::BodyPropertiesOutcome::Certified {
            properties,
            full_check,
        } => BodyPropertiesOutcome::Certified {
            properties: BodyProperties {
                body,
                volume: ScalarEnclosure::from_lower(properties.volume()),
                centroid: Point3Enclosure::from_lower(properties.centroid()),
                surface_area: ScalarEnclosure::from_lower(properties.surface_area()),
                centroidal_inertia: SymmetricTensor3Enclosure::from_lower(
                    properties.centroidal_inertia(),
                ),
            },
            full_check: adapt_check_report(part, store, full_check)?,
        },
        ktopo::body_properties::BodyPropertiesOutcome::Refused { reason, full_check } => {
            BodyPropertiesOutcome::Refused {
                reason: adapt_refusal(part, reason),
                full_check: adapt_check_report(part, store, full_check)?,
            }
        }
    })
}

fn adapt_refusal(
    part: &PartId,
    refusal: ktopo::body_properties::BodyPropertiesRefusal,
) -> BodyPropertiesRefusal {
    match refusal {
        ktopo::body_properties::BodyPropertiesRefusal::NonSolidBody => {
            BodyPropertiesRefusal::NonSolidBody
        }
        ktopo::body_properties::BodyPropertiesRefusal::BodyNotFullValid => {
            BodyPropertiesRefusal::BodyNotFullValid
        }
        ktopo::body_properties::BodyPropertiesRefusal::TolerantTopology => {
            BodyPropertiesRefusal::TolerantTopology
        }
        ktopo::body_properties::BodyPropertiesRefusal::UnsupportedSurface { face } => {
            BodyPropertiesRefusal::UnsupportedSurface {
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_properties::BodyPropertiesRefusal::UnsupportedPcurve { face } => {
            BodyPropertiesRefusal::UnsupportedPcurve {
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_properties::BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face } => {
            BodyPropertiesRefusal::UncertifiedAnalyticBoundary {
                face: FaceId::new(part.clone(), face),
            }
        }
        ktopo::body_properties::BodyPropertiesRefusal::NonPositiveVolumeEnclosure => {
            BodyPropertiesRefusal::NonPositiveVolumeEnclosure
        }
        ktopo::body_properties::BodyPropertiesRefusal::NonPositiveSurfaceAreaEnclosure => {
            BodyPropertiesRefusal::NonPositiveSurfaceAreaEnclosure
        }
        ktopo::body_properties::BodyPropertiesRefusal::InertiaEnclosureIndeterminate => {
            BodyPropertiesRefusal::InertiaEnclosureIndeterminate
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AccountingMode, BlockRequest, BudgetPlan, CylinderRequest, ErrorClass, Frame, Kernel,
        LimitSpec, ResourceKind,
    };

    fn certified(outcome: BodyPropertiesOutcome) -> BodyProperties {
        match outcome {
            BodyPropertiesOutcome::Certified {
                properties,
                full_check,
            } => {
                assert_eq!(full_check.outcome(), crate::CheckOutcome::Valid);
                properties
            }
            BodyPropertiesOutcome::Refused { reason, .. } => panic!("refused: {reason:?}"),
        }
    }

    fn rotated_diagonal(frame: Frame, diagonal: [f64; 3]) -> [[f64; 3]; 3] {
        let axes = [frame.x(), frame.y(), frame.z()];
        let coordinate = |axis: crate::Vec3, index| match index {
            0 => axis.x,
            1 => axis.y,
            _ => axis.z,
        };
        core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                (0..3)
                    .map(|axis| {
                        diagonal[axis]
                            * coordinate(axes[axis], row)
                            * coordinate(axes[axis], column)
                    })
                    .sum()
            })
        })
    }

    #[test]
    fn block_and_finite_cylinder_are_certified() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let translated_frame = Frame::new(
            Point3::new(300.0, -250.0, 200.0),
            crate::Vec3::new(0.48, 0.64, 0.6),
            crate::Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap();
        let (block, cylinder, translated_block) = {
            let mut part = session.edit_part(part_id.clone()).unwrap();
            let block = part
                .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = part
                .create_cylinder(CylinderRequest::new(Frame::world(), 1.5, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let translated_block = part
                .create_block(BlockRequest::new(translated_frame, [2.0, 3.0, 4.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder, translated_block)
        };
        let part = session.part(part_id).unwrap();

        let block = certified(
            part.body_properties(BodyPropertiesRequest::new(block))
                .unwrap()
                .into_result()
                .unwrap(),
        );
        assert!(block.volume().contains(24.0));
        assert!(block.surface_area().contains(52.0));
        assert!(block.centroidal_inertia().contains([
            [50.0, 0.0, 0.0],
            [0.0, 40.0, 0.0],
            [0.0, 0.0, 26.0],
        ]));
        let block_inertia_matrix = block.centroidal_inertia().matrix();
        assert_eq!(block_inertia_matrix[0][1], block_inertia_matrix[1][0]);
        assert_eq!(block_inertia_matrix[0][2], block_inertia_matrix[2][0]);
        assert_eq!(block_inertia_matrix[1][2], block_inertia_matrix[2][1]);
        assert!(
            block.centroid().contains(Point3::new(0.0, 0.0, 0.0)),
            "{:?}",
            block.centroid()
        );

        let cylinder = certified(
            part.body_properties(BodyPropertiesRequest::new(cylinder))
                .unwrap()
                .into_result()
                .unwrap(),
        );
        assert!(
            cylinder
                .volume()
                .contains(core::f64::consts::PI * 1.5 * 1.5 * 2.0)
        );
        assert!(
            cylinder
                .surface_area()
                .contains(2.0 * core::f64::consts::PI * 1.5 * (1.5 + 2.0))
        );
        assert!(cylinder.centroidal_inertia().contains([
            [129.0 * core::f64::consts::PI / 32.0, 0.0, 0.0],
            [0.0, 129.0 * core::f64::consts::PI / 32.0, 0.0],
            [0.0, 0.0, 81.0 * core::f64::consts::PI / 16.0],
        ]));
        assert!(
            cylinder.centroid().contains(Point3::new(0.0, 0.0, 1.0)),
            "{:?}",
            cylinder.centroid()
        );

        let translated = certified(
            part.body_properties(BodyPropertiesRequest::new(translated_block))
                .unwrap()
                .into_result()
                .unwrap(),
        );
        assert!(translated.volume().contains(24.0));
        assert!((translated.surface_area().value() - 52.0).abs() <= 1.0e-12);
        assert!(translated.centroid().contains(translated_frame.origin()));
        assert!(translated.centroid().error_bound() <= 1.0e-10);
        assert!(translated.surface_area().error_bound() <= 1.0e-10);
        let expected_inertia = rotated_diagonal(translated_frame, [50.0, 40.0, 26.0]);
        assert!(
            translated.centroidal_inertia().contains(expected_inertia),
            "expected {expected_inertia:?}, enclosure {:?}",
            translated.centroidal_inertia(),
        );
        assert!(translated.centroidal_inertia().error_bound() <= 1.0e-8);
    }

    #[test]
    fn analytic_work_budget_accepts_exactly_n_and_rejects_n_minus_one() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let bodies = {
            let mut part = session.edit_part(part_id.clone()).unwrap();
            let block = part
                .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = part
                .create_cylinder(CylinderRequest::new(Frame::world(), 1.5, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            [block, cylinder]
        };
        let part = session.part(part_id).unwrap();
        let settings = |allowed| {
            OperationSettings::new().with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    crate::BODY_PROPERTIES_ANALYTIC_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            )
        };
        let mut consumptions = [0_u64; 2];
        for (index, body) in bodies.into_iter().enumerate() {
            let baseline = part
                .body_properties(BodyPropertiesRequest::new(body.clone()))
                .unwrap();
            let consumed = baseline
                .report()
                .usage()
                .iter()
                .find(|usage| {
                    usage.stage == crate::BODY_PROPERTIES_ANALYTIC_WORK
                        && usage.resource == ResourceKind::Work
                })
                .expect("analytic stage was not charged")
                .consumed;
            assert!(consumed > 0);
            consumptions[index] = consumed;

            let exact = part
                .body_properties(
                    BodyPropertiesRequest::new(body.clone()).with_settings(settings(consumed)),
                )
                .unwrap();
            assert!(exact.result().is_ok());
            let refused = part
                .body_properties(
                    BodyPropertiesRequest::new(body).with_settings(settings(consumed - 1)),
                )
                .unwrap();
            let error = refused.result().unwrap_err();
            assert_eq!(error.class(), ErrorClass::ResourceLimit);
            let limit = error.limit().expect("resource failure lost its limit");
            assert_eq!(limit.stage, crate::BODY_PROPERTIES_ANALYTIC_WORK);
            assert_eq!(limit.consumed, consumed);
            assert_eq!(limit.allowed, consumed - 1);
        }
        assert_ne!(consumptions[0], consumptions[1]);
    }
}
