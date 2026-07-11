//! JSON-lines corpus inspector for XT parse/reconstruct/check/tess stages.

use ktopo::btess::{TessOptions, tessellate_body};
use ktopo::check::{CheckLevel, CheckOutcome, check_body_report};
use ktopo::store::Store;
use kxt::parse::{Value, XtFile};
use kxt::schema::code;
use kxt::{XtCapability, XtError, read_xt, reconstruct};
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;
use std::process::ExitCode;

#[derive(Default)]
struct Features {
    body_nodes: usize,
    face_nodes: usize,
    nonnull_face_tolerances: usize,
    edge_nodes: usize,
    fin_nodes: usize,
    null_curve_tolerant_edges: usize,
    fin_curves: usize,
    trimmed_sp_fin_curves: usize,
    b_curves: usize,
    b_surfaces: usize,
    intersection_curves: usize,
    procedural_surfaces: usize,
}

fn pointer(file: &XtFile, node: &kxt::Node, field: &'static str) -> Option<u32> {
    file.field(node, field).and_then(Value::as_ptr)
}

fn features(file: &XtFile) -> Features {
    let mut out = Features::default();
    for node in file.nodes.values() {
        match node.code {
            code::BODY => out.body_nodes += 1,
            code::FACE => {
                out.face_nodes += 1;
                if !matches!(file.field(node, "tolerance"), Some(Value::Null) | None) {
                    out.nonnull_face_tolerances += 1;
                }
            }
            code::EDGE => {
                out.edge_nodes += 1;
                let curve_is_null = pointer(file, node, "curve") == Some(0);
                let tolerant = !matches!(file.field(node, "tolerance"), Some(Value::Null) | None);
                if curve_is_null && tolerant {
                    out.null_curve_tolerant_edges += 1;
                }
            }
            code::FIN => {
                out.fin_nodes += 1;
                if let Some(curve) = pointer(file, node, "curve").filter(|&curve| curve != 0) {
                    out.fin_curves += 1;
                    let is_trimmed_sp = file.nodes.get(&curve).is_some_and(|trimmed| {
                        trimmed.code == code::TRIMMED_CURVE
                            && pointer(file, trimmed, "basis_curve")
                                .and_then(|basis| file.nodes.get(&basis))
                                .is_some_and(|basis| basis.code == code::SP_CURVE)
                    });
                    if is_trimmed_sp {
                        out.trimmed_sp_fin_curves += 1;
                    }
                }
            }
            code::B_CURVE => out.b_curves += 1,
            code::B_SURFACE => out.b_surfaces += 1,
            code::INTERSECTION => out.intersection_curves += 1,
            code::SWEPT_SURF
            | code::SPUN_SURF
            | code::OFFSET_SURF
            | code::BLENDED_EDGE
            | code::BLEND_BOUND
            | code::PE_SURF => out.procedural_surfaces += 1,
            _ => {}
        }
    }
    out
}

fn json_string(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    out.push('"');
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ch if ch.is_control() => {
                write!(out, "\\u{:04x}", ch as u32).expect("writing String cannot fail");
            }
            ch => out.push(ch),
        }
    }
    out.push('"');
    out
}

fn error_stage(error: &XtError) -> &'static str {
    if error.capability().is_some() {
        "unsupported"
    } else {
        "fail"
    }
}

fn capability_json(capability: Option<XtCapability>) -> String {
    capability.map_or_else(|| "null".to_owned(), |value| json_string(value.code()))
}

fn failed_row(
    path: &Path,
    bytes: usize,
    stage: &str,
    status: &str,
    capability: Option<XtCapability>,
    error: &str,
) -> String {
    format!(
        "{{\"path\":{},\"bytes\":{bytes},\"schema\":null,\"nodes\":0,\
         \"features\":{{\"body_nodes\":0,\"face_nodes\":0,\"nonnull_face_tolerances\":0,\
         \"edge_nodes\":0,\"fin_nodes\":0,\
         \"null_curve_tolerant_edges\":0,\"fin_curves\":0,\
         \"trimmed_sp_fin_curves\":0,\"b_curves\":0,\"b_surfaces\":0,\
         \"intersection_curves\":0,\"procedural_surfaces\":0}},\
         \"stages\":{{\"parse\":{},\"reconstruct\":\"not_run\",\
         \"checker\":\"not_run\",\"tessellate\":\"not_run\"}},\
         \"reconstructed_bodies\":0,\"checker_faults\":0,\"triangles\":0,\
         \"checker_fault_kinds\":{},\"full_checker_outcome\":\"not_run\",\
         \"full_checker_gaps\":0,\"full_checker_gap_kinds\":{},\
         \"capability\":{},\"failed_stage\":{},\"error\":{}}}",
        json_string(&path.display().to_string()),
        json_string(status),
        "{}",
        "{}",
        capability_json(capability),
        json_string(stage),
        json_string(error),
    )
}

fn inspect(path: &Path) -> (String, bool) {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) => {
            return (
                failed_row(path, 0, "read", "fail", None, &error.to_string()),
                false,
            );
        }
    };
    let file = match read_xt(&bytes) {
        Ok(file) => file,
        Err(error) => {
            return (
                failed_row(
                    path,
                    bytes.len(),
                    "parse",
                    error_stage(&error),
                    error.capability(),
                    &error.to_string(),
                ),
                false,
            );
        }
    };
    let found = features(&file);
    let mut store = Store::new();
    let reconstruction = match reconstruct(&file, &mut store) {
        Ok(reconstruction) => reconstruction,
        Err(error) => {
            let row = format!(
                "{{\"path\":{},\"bytes\":{},\"schema\":{},\"nodes\":{},\
                 \"features\":{},\"stages\":{{\"parse\":\"pass\",\
                 \"reconstruct\":{},\"checker\":\"not_run\",\
                 \"tessellate\":\"not_run\"}},\"reconstructed_bodies\":0,\
                 \"checker_faults\":0,\"triangles\":0,\"checker_fault_kinds\":{},\
                 \"full_checker_outcome\":\"not_run\",\"full_checker_gaps\":0,\
                 \"full_checker_gap_kinds\":{},\"capability\":{},\
                 \"failed_stage\":\"reconstruct\",\
                 \"error\":{}}}",
                json_string(&path.display().to_string()),
                bytes.len(),
                json_string(&file.schema),
                file.nodes.len(),
                feature_json(&found),
                json_string(error_stage(&error)),
                "{}",
                "{}",
                capability_json(error.capability()),
                json_string(&error.to_string()),
            );
            return (row, false);
        }
    };

    let mut checker_faults = 0usize;
    let mut checker_fault_kinds = BTreeMap::new();
    let mut full_checker_outcome = CheckOutcome::Valid;
    let mut full_checker_gaps = 0usize;
    let mut full_checker_gap_kinds = BTreeMap::new();
    let mut checker_error = None;
    for &body in &reconstruction.bodies {
        match check_body_report(&store, body, CheckLevel::Full) {
            Ok(report) => {
                full_checker_outcome =
                    combine_check_outcomes(full_checker_outcome, report.outcome());
                checker_faults += report.faults.len();
                for fault in report.faults {
                    *checker_fault_kinds
                        .entry(format!("{:?}", fault.kind))
                        .or_insert(0usize) += 1;
                }
                full_checker_gaps += report.gaps.len();
                for gap in report.gaps {
                    *full_checker_gap_kinds
                        .entry(format!("{:?}", gap.kind))
                        .or_insert(0usize) += 1;
                }
            }
            Err(error) => checker_error = Some(error.to_string()),
        }
    }
    let checker_pass = checker_faults == 0 && checker_error.is_none();

    let mut triangles = 0usize;
    let mut tessellated = 0usize;
    let mut tessellation_error = None;
    if checker_pass {
        for &body in &reconstruction.bodies {
            if store
                .faces_of_body(body)
                .is_ok_and(|faces| faces.is_empty())
            {
                continue;
            }
            match tessellate_body(
                &store,
                body,
                &TessOptions {
                    chord_tol: 1e-3,
                    max_edge_len: None,
                },
            ) {
                Ok(mesh) => {
                    tessellated += 1;
                    triangles += mesh.triangles.len();
                }
                Err(error) => {
                    tessellation_error = Some(error.to_string());
                    break;
                }
            }
        }
    }
    let tessellation_status = if !checker_pass {
        "not_run"
    } else if tessellation_error.is_some() {
        "fail"
    } else if tessellated == 0 {
        "not_applicable"
    } else {
        "pass"
    };
    let success = checker_pass && tessellation_error.is_none();
    let full_checker_outcome_name = if checker_error.is_some() {
        "error"
    } else {
        check_outcome_name(full_checker_outcome)
    };
    let error = checker_error.or(tessellation_error);
    let failed_stage = if checker_faults != 0 || error.is_some() {
        if checker_pass {
            "tessellate"
        } else {
            "checker"
        }
    } else {
        "none"
    };
    (
        format!(
            "{{\"path\":{},\"bytes\":{},\"schema\":{},\"nodes\":{},\
             \"features\":{},\"stages\":{{\"parse\":\"pass\",\
             \"reconstruct\":\"pass\",\"checker\":{},\"tessellate\":{}}},\
             \"reconstructed_bodies\":{},\"checker_faults\":{},\"triangles\":{},\
             \"checker_fault_kinds\":{},\"full_checker_outcome\":{},\
             \"full_checker_gaps\":{},\"full_checker_gap_kinds\":{},\
             \"capability\":null,\"failed_stage\":{},\"error\":{}}}",
            json_string(&path.display().to_string()),
            bytes.len(),
            json_string(&file.schema),
            file.nodes.len(),
            feature_json(&found),
            json_string(if checker_pass { "pass" } else { "fail" }),
            json_string(tessellation_status),
            reconstruction.bodies.len(),
            checker_faults,
            triangles,
            count_json(&checker_fault_kinds),
            json_string(full_checker_outcome_name),
            full_checker_gaps,
            count_json(&full_checker_gap_kinds),
            json_string(failed_stage),
            error
                .as_deref()
                .map_or_else(|| "null".to_owned(), json_string),
        ),
        success,
    )
}

fn combine_check_outcomes(left: CheckOutcome, right: CheckOutcome) -> CheckOutcome {
    match (left, right) {
        (CheckOutcome::Invalid, _) | (_, CheckOutcome::Invalid) => CheckOutcome::Invalid,
        (CheckOutcome::Indeterminate, _) | (_, CheckOutcome::Indeterminate) => {
            CheckOutcome::Indeterminate
        }
        (CheckOutcome::Valid, CheckOutcome::Valid) => CheckOutcome::Valid,
    }
}

fn check_outcome_name(outcome: CheckOutcome) -> &'static str {
    match outcome {
        CheckOutcome::Valid => "valid",
        CheckOutcome::Invalid => "invalid",
        CheckOutcome::Indeterminate => "indeterminate",
    }
}

fn count_json(counts: &BTreeMap<String, usize>) -> String {
    let mut out = String::from("{");
    for (position, (name, count)) in counts.iter().enumerate() {
        if position != 0 {
            out.push(',');
        }
        write!(out, "{}:{count}", json_string(name)).expect("writing String cannot fail");
    }
    out.push('}');
    out
}

fn feature_json(features: &Features) -> String {
    format!(
        "{{\"body_nodes\":{},\"face_nodes\":{},\"nonnull_face_tolerances\":{},\
         \"edge_nodes\":{},\"fin_nodes\":{},\
         \"null_curve_tolerant_edges\":{},\"fin_curves\":{},\
         \"trimmed_sp_fin_curves\":{},\"b_curves\":{},\"b_surfaces\":{},\
         \"intersection_curves\":{},\"procedural_surfaces\":{}}}",
        features.body_nodes,
        features.face_nodes,
        features.nonnull_face_tolerances,
        features.edge_nodes,
        features.fin_nodes,
        features.null_curve_tolerant_edges,
        features.fin_curves,
        features.trimmed_sp_fin_curves,
        features.b_curves,
        features.b_surfaces,
        features.intersection_curves,
        features.procedural_surfaces,
    )
}

fn render_value(value: &Value) -> String {
    match value {
        Value::Ptr(p) => format!("→{p}"),
        Value::Double(x) => format!("{x}"),
        Value::Int(i) => format!("{i}"),
        Value::Char(c) => format!("'{c}'"),
        Value::Arr(values) => {
            let rendered: Vec<String> = values.iter().map(render_value).collect();
            format!("[{}]", rendered.join(", "))
        }
        other => format!("{other:?}"),
    }
}

/// Dump every node with named fields — the mining tool for diffing this
/// writer's output against real corpus files.
fn dump_nodes(path: &Path) -> Result<(), String> {
    let bytes = std::fs::read(path).map_err(|error| error.to_string())?;
    let file = read_xt(&bytes).map_err(|error| error.to_string())?;
    println!(
        "== {} schema={} usfld={} nodes={}",
        path.display(),
        file.schema,
        file.usfld_size,
        file.nodes.len()
    );
    for (index, node) in &file.nodes {
        let def = file.defs.get(&node.code);
        let name = def.map_or("?", |d| d.name.as_str());
        let mut parts = Vec::new();
        for (position, value) in node.values.iter().enumerate() {
            let field = def
                .and_then(|d| d.fields.get(position))
                .map_or_else(|| format!("f{position}"), |f| f.name.clone());
            parts.push(format!("{field}={}", render_value(value)));
        }
        println!("[{index}] {name}({}) {}", node.code, parts.join(" "));
    }
    Ok(())
}

fn main() -> ExitCode {
    let mut paths: Vec<_> = std::env::args_os().skip(1).collect();
    let dump = paths.first().is_some_and(|arg| arg == "--nodes");
    if dump {
        paths.remove(0);
    }
    if paths.is_empty() {
        eprintln!("usage: xt_inspect [--nodes] FILE.x_t [FILE.x_b ...]");
        return ExitCode::from(2);
    }
    let mut all_passed = true;
    for path in paths {
        if dump {
            if let Err(error) = dump_nodes(Path::new(&path)) {
                eprintln!("{}: {error}", Path::new(&path).display());
                all_passed = false;
            }
        } else {
            let (row, passed) = inspect(Path::new(&path));
            println!("{row}");
            all_passed &= passed;
        }
    }
    ExitCode::from(u8::from(!all_passed))
}
