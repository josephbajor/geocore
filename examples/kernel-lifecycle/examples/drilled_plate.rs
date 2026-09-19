//! Facade-only composed modeling sequence. Run with one output X_T path.

use kernel::{
    BooleanBodiesRequest, BooleanOperation, BooleanOutcome, BooleanResult, CylinderRequest,
    ExportXtRequest, ExtrudeProfileRequest, Frame, Kernel, Point2, Point3,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut arguments = std::env::args_os().skip(1);
    let output = arguments.next().ok_or("usage: drilled_plate OUTPUT.x_t")?;
    if arguments.next().is_some() {
        return Err("usage: drilled_plate OUTPUT.x_t".into());
    }
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let mut body = session
        .edit_part(part.clone())?
        .extrude_profile(ExtrudeProfileRequest::new(
            Frame::world(),
            vec![
                Point2::new(-5.0, -4.0),
                Point2::new(5.0, -4.0),
                Point2::new(5.0, 4.0),
                Point2::new(-5.0, 4.0),
            ],
            vec![],
            2.0,
        ))?
        .into_result()?
        .body();
    for (x, y, radius) in [
        (-2.5, -1.0, 0.75),
        (2.0, -1.0, 0.5),
        (0.0, 2.0, 0.625),
        (-2.5, -1.0, 1.25),
    ] {
        let mut edit = session.edit_part(part.clone())?;
        let tool = edit
            .create_cylinder(CylinderRequest::new(
                Frame::world().with_origin(Point3::new(x, y, -1.0)),
                radius,
                4.0,
            ))?
            .into_result()?
            .body();
        let outcome = edit
            .boolean_bodies(BooleanBodiesRequest::new(
                BooleanOperation::Subtract,
                body,
                tool,
            ))?
            .into_result()?;
        let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
            return Err(format!("through cut refused: {outcome:?}").into());
        };
        if created.bodies().len() != 1 {
            return Err("through cut did not create one solid".into());
        }
        body = created.bodies()[0].clone();
    }
    let exported = session
        .part(part)?
        .export_xt(ExportXtRequest::new(body))?
        .into_result()?;
    std::fs::write(output, exported.bytes())?;
    println!("Wrote a Full-checked plate with three holes, then enlarged the first hole.");
    Ok(())
}
