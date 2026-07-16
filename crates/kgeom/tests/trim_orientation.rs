//! Public trim-loop orientation conformance tests.

use kcore::error::{Error, Result};
use kgeom::frame::Frame;
use kgeom::surface::Plane;
use kgeom::tess::{TrimLoop, TrimmedSurface};
use kgeom::vec::Vec2;

fn large_cancellation_square() -> Vec<Vec2> {
    let magnitude = 2f64.powi(52);
    vec![
        Vec2::new(magnitude, magnitude),
        Vec2::new(magnitude + 1.0, magnitude),
        Vec2::new(magnitude + 1.0, magnitude + 1.0),
        Vec2::new(magnitude, magnitude + 1.0),
    ]
}

#[test]
fn exact_trim_winding_accepts_every_rotation_and_rejects_reversal() -> Result<()> {
    let plane = Plane::new(Frame::world());
    let points = large_cancellation_square();
    let magnitude = 2f64.powi(52);
    let enclosing = TrimLoop::new(vec![
        Vec2::new(magnitude - 1.0, magnitude - 1.0),
        Vec2::new(magnitude + 2.0, magnitude - 1.0),
        Vec2::new(magnitude + 2.0, magnitude + 2.0),
        Vec2::new(magnitude - 1.0, magnitude + 2.0),
    ])?;

    for rotation in 0..points.len() {
        let mut rotated = points.clone();
        rotated.rotate_left(rotation);
        let expected = rotated.clone();
        rotated.insert(2, rotated[1]);
        rotated.push(rotated[0]);
        let loop_ = TrimLoop::new(rotated)?;
        assert_eq!(loop_.points, expected);
        assert_eq!(loop_.signed_area(), 0.0, "fixture must defeat shoelace");
        TrimmedSurface::new(&plane, vec![loop_])?;

        let mut reversed = points.clone();
        reversed.rotate_left(rotation);
        reversed.reverse();
        let reversed = TrimLoop::new(reversed)?;
        TrimmedSurface::new(&plane, vec![enclosing.clone(), reversed.clone()])?;
        assert!(matches!(
            TrimmedSurface::new(&plane, vec![reversed]),
            Err(Error::InvalidGeometry {
                reason: "outer trim loop must wind counterclockwise"
            })
        ));
    }

    Ok(())
}

#[test]
fn trim_orientation_rejects_exact_zero_and_nonfinite_public_inputs() {
    assert!(matches!(
        TrimLoop::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(2.0, 2.0),
        ]),
        Err(Error::InvalidGeometry {
            reason: "trim loop has zero area"
        })
    ));
    assert!(matches!(
        TrimLoop::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(f64::NAN, 1.0),
            Vec2::new(1.0, 0.0),
        ]),
        Err(Error::InvalidGeometry {
            reason: "trim loop vertex is not finite"
        })
    ));

    let plane = Plane::new(Frame::world());
    let bypassed_constructor = TrimLoop {
        points: vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(f64::INFINITY, 1.0),
            Vec2::new(1.0, 0.0),
        ],
    };
    assert!(matches!(
        TrimmedSurface::new(&plane, vec![bypassed_constructor]),
        Err(Error::InvalidGeometry {
            reason: "outer trim loop must wind counterclockwise"
        })
    ));
}
