//! Facade-only executable covering one supported application lifecycle.

use std::error::Error;
use std::ffi::OsString;
use std::io;
use std::path::PathBuf;

use kernel::{
    BlockRequest, BodyTessellationBudgetProfile, BoundedCurve, CheckBodyRequest, CheckLevel,
    CheckOutcome, ExportXtRequest, Frame, FullCheckBudgetProfile, ImportXtRequest,
    IntersectCurvesRequest, Kernel, OperationSettings, ParamRange, SurfaceDerivativeOrder,
    SurfaceEvaluationRequest, TessOptions, TessellateBodyRequest,
};

fn output_path() -> Result<PathBuf, io::Error> {
    let mut arguments = std::env::args_os();
    let executable = arguments
        .next()
        .unwrap_or_else(|| OsString::from("kernel-lifecycle"));
    let output = arguments.next().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("usage: {} OUTPUT.x_t", PathBuf::from(executable).display()),
        )
    })?;
    if arguments.next().is_some() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected exactly one output path",
        ));
    }
    Ok(output.into())
}

fn main() -> Result<(), Box<dyn Error>> {
    let output_path = output_path()?;
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();

    let created = session
        .edit_part(part_id.clone())?
        .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))?
        .into_result()?;
    let construction_mutations = created.journal().mutation_count();
    let body_id = created.body();
    let full_check_settings =
        || OperationSettings::new().with_budget_overrides(FullCheckBudgetProfile::v1_defaults());

    let (
        body_kind,
        face_count,
        edge_count,
        vertex_count,
        mesh_vertex_count,
        mesh_triangle_count,
        surface_class,
        point,
        authored_xt,
    ) = {
        let part = session.part(part_id.clone())?;
        let body = part.body(body_id.clone())?;
        let face_ids = body.faces()?.collect::<Vec<_>>();
        let edge_ids = body.edges()?.collect::<Vec<_>>();
        let edge_count = edge_ids.len();
        let vertex_count = body.vertices()?.len();
        let face_id = face_ids
            .first()
            .cloned()
            .ok_or_else(|| io::Error::other("constructed body has no face"))?;
        let face = part.face(face_id)?;
        let surface_id = face.surface();
        let uv = face
            .domain()
            .ok_or_else(|| io::Error::other("constructed face has no finite domain"))?
            .center();
        let surface_class = part.surface(surface_id.clone())?.class_key().as_str();

        let checked = part
            .check_body(
                CheckBodyRequest::new(body_id.clone(), CheckLevel::Full)
                    .with_settings(full_check_settings()),
            )?
            .into_result()?;
        if checked.outcome() != CheckOutcome::Valid {
            return Err(io::Error::other(format!(
                "constructed body did not check as valid: {:?}",
                checked.outcome()
            ))
            .into());
        }

        let mesh = part
            .tessellate_body(
                TessellateBodyRequest::new(
                    body_id.clone(),
                    TessOptions {
                        chord_tol: 1.0e-3,
                        max_edge_len: None,
                    },
                )
                .with_settings(
                    OperationSettings::new()
                        .with_budget_overrides(BodyTessellationBudgetProfile::bounded_v1()),
                ),
            )?
            .into_result()?;
        if mesh.positions().len() != 8
            || mesh.triangles().len() != 12
            || mesh.face_triangle_ranges().len() != face_ids.len()
            || mesh.edge_polylines().len() != edge_count
            || mesh
                .face_triangle_ranges()
                .iter()
                .any(|range| range.range().is_empty() || part.face(range.face()).is_err())
            || mesh
                .edge_polylines()
                .iter()
                .any(|line| line.vertex_indices().is_empty() || part.edge(line.edge()).is_err())
        {
            return Err(io::Error::other("facade tessellation summary changed").into());
        }
        let mesh_vertex_count = mesh.positions().len();
        let mesh_triangle_count = mesh.triangles().len();

        let evaluated = part
            .evaluate_surface(SurfaceEvaluationRequest::new(
                surface_id,
                uv,
                SurfaceDerivativeOrder::First,
            ))?
            .into_result()?;
        let point = evaluated.position();
        if !point
            .to_array()
            .iter()
            .all(|coordinate| coordinate.is_finite())
        {
            return Err(io::Error::other("surface evaluation returned a non-finite point").into());
        }

        let bounded_edges = edge_ids
            .into_iter()
            .map(|edge_id| {
                let edge = part.edge(edge_id)?;
                let (lo, hi) = edge
                    .bounds()
                    .ok_or_else(|| io::Error::other("constructed block has an unbounded edge"))?;
                let curve = edge.curve().ok_or_else(|| {
                    io::Error::other("constructed block has a curve-less tolerant edge")
                })?;
                Ok::<_, Box<dyn Error>>((
                    BoundedCurve::new(curve, ParamRange::new(lo, hi)),
                    edge.vertices(),
                ))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let (first, second) = (0..bounded_edges.len())
            .find_map(|left| {
                ((left + 1)..bounded_edges.len()).find_map(|right| {
                    let adjacent = bounded_edges[left].1.iter().flatten().any(|left_vertex| {
                        bounded_edges[right]
                            .1
                            .iter()
                            .flatten()
                            .any(|right_vertex| right_vertex == left_vertex)
                    });
                    adjacent.then(|| {
                        (
                            bounded_edges[left].0.clone(),
                            bounded_edges[right].0.clone(),
                        )
                    })
                })
            })
            .ok_or_else(|| io::Error::other("constructed block has no adjacent edge pair"))?;
        let intersections = part
            .intersect_curves(IntersectCurvesRequest::new(first, second))?
            .into_result()?;
        if !intersections.is_complete() || intersections.points().is_empty() {
            return Err(io::Error::other(
                "adjacent facade curves did not produce a complete isolated intersection",
            )
            .into());
        }

        let authored_xt = part
            .export_xt(ExportXtRequest::new(body_id.clone()))?
            .into_result()?
            .into_text();
        (
            body.kind(),
            face_ids.len(),
            edge_count,
            vertex_count,
            mesh_vertex_count,
            mesh_triangle_count,
            surface_class,
            point,
            authored_xt,
        )
    };

    let imported_part_id = session.create_part();
    let imported = session
        .edit_part(imported_part_id.clone())?
        .import_xt(ImportXtRequest::new(authored_xt.as_bytes()))?
        .into_result()?;
    let import_mutations = imported.journal().mutation_count();
    let imported_body_id = imported
        .bodies()
        .first()
        .cloned()
        .ok_or_else(|| io::Error::other("facade import returned no body"))?;
    if imported.bodies().len() != 1 {
        return Err(io::Error::other("facade import returned more than one body").into());
    }

    let imported_xt = {
        let part = session.part(imported_part_id)?;
        let imported_body = part.body(imported_body_id.clone())?;
        if imported_body.faces()?.len() != face_count
            || imported_body.edges()?.len() != edge_count
            || imported_body.vertices()?.len() != vertex_count
        {
            return Err(io::Error::other("imported topology summary changed").into());
        }
        let checked = part
            .check_body(
                CheckBodyRequest::new(imported_body_id.clone(), CheckLevel::Full)
                    .with_settings(full_check_settings()),
            )?
            .into_result()?;
        if checked.outcome() != CheckOutcome::Valid {
            return Err(io::Error::other(format!(
                "imported body did not check as valid: {:?}",
                checked.outcome()
            ))
            .into());
        }
        part.export_xt(ExportXtRequest::new(imported_body_id))?
            .into_result()?
            .into_text()
    };

    if authored_xt != imported_xt {
        return Err(io::Error::other("facade X_T round trip changed deterministic bytes").into());
    }
    session.part(part_id)?.body(body_id)?;
    std::fs::write(&output_path, imported_xt.as_bytes())?;

    println!(
        "kind={body_kind:?} faces={} edges={} vertices={} mesh_vertices={} mesh_triangles={} check={:?} surface={} point={:?} bytes={} construction_mutations={} import_mutations={} imported_bodies=1 byte_stable=true original_live=true",
        face_count,
        edge_count,
        vertex_count,
        mesh_vertex_count,
        mesh_triangle_count,
        CheckOutcome::Valid,
        surface_class,
        point.to_array(),
        imported_xt.len(),
        construction_mutations,
        import_mutations,
    );
    Ok(())
}
