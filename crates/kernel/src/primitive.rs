//! Checked analytic primitive construction at the supported facade boundary.
//!
//! Curved primitives live here rather than enlarging the already broad
//! operation adapter. Each constructor owns one operation scope, delegates
//! failure-atomic topology creation to `ktopo`, and returns only opaque facade
//! identity plus the committed journal.

use kcore::operation::OperationScope;

use crate::operation::{BodyCreated, ChangeJournal, OperationOutcome, OperationSettings};
use crate::{BodyId, Frame, PartEdit, Result};

/// Typed request to construct one checked finite solid cylinder.
#[derive(Debug, Clone, PartialEq)]
pub struct CylinderRequest {
    frame: Frame,
    radius: f64,
    height: f64,
    settings: OperationSettings,
}

impl CylinderRequest {
    /// Construct a cylinder request using default operation settings.
    ///
    /// The base disc lies in `frame`'s XY plane and the cylinder extends by
    /// `height` along `frame.z()`.
    pub fn new(frame: Frame, radius: f64, height: f64) -> Self {
        Self {
            frame,
            radius,
            height,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Requested placement frame.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Requested cylinder radius.
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// Requested cylinder height along the frame axis.
    pub const fn height(&self) -> f64 {
        self.height
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

impl PartEdit<'_> {
    /// Construct and checked-commit one finite solid cylinder through a
    /// single facade-owned operation context and scope.
    ///
    /// The lower constructor validates finite positive dimensions, creates
    /// exact circle/cylinder/plane geometry with independent pcurves, checks
    /// the completed body, and rolls back all allocation on failure.
    pub fn create_cylinder(
        &mut self,
        request: CylinderRequest,
    ) -> Result<OperationOutcome<BodyCreated>> {
        let CylinderRequest {
            frame,
            radius,
            height,
            settings,
        } = request;
        let context = settings.context(self.policy)?;
        let scope = OperationScope::new(&context);
        let part = self.id.clone();
        let result =
            ktopo::make::cylinder_with_journal(&mut self.state.store, &frame, radius, height)
                .map(|creation| {
                    let (raw_body, raw_journal) = creation.into_parts();
                    BodyCreated::new(
                        BodyId::new(part.clone(), raw_body),
                        ChangeJournal::from_raw(part, raw_journal),
                    )
                })
                .map_err(crate::Error::from);
        Ok(scope.finish_typed(result))
    }
}
