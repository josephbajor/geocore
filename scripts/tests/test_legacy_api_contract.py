"""Tests for the internal legacy-API retirement ratchet."""

import unittest
from pathlib import Path

from scripts.legacy_api_contract import (
    BODY_TESSELLATION_DEFINITION,
    CURVE_PROJECTION_DEFINITION,
    FACE_TESSELLATION_DEFINITION,
    FACADE_BODY_TESSELLATION_DEFINITION,
    SURFACE_POINT_COMPATIBILITY,
    SURFACE_PROJECTION_DEFINITION,
    audit_repository,
    find_body_tessellation_deprecation_violations,
    find_legacy_body_tessellation_uses,
    find_legacy_curve_projection_uses,
    find_legacy_face_tessellation_uses,
    find_legacy_surface_projection_uses,
)


ROOT = Path(__file__).resolve().parents[2]


class BodyTessellationRatchetTests(unittest.TestCase):
    def test_current_production_sources_are_closed(self) -> None:
        self.assertEqual(audit_repository(ROOT), [])

    def test_production_import_and_call_are_rejected(self) -> None:
        path = Path("crates/kxt/src/bin/new_tool.rs")
        source = """\
use ktopo::btess::tessellate_body;

fn run() {
    let _ = tessellate_body(&store, body, &options);
}
"""
        self.assertEqual(
            find_legacy_body_tessellation_uses({path: source}),
            [f"{path}:1", f"{path}:4"],
        )

    def test_public_wrapper_and_cfg_test_clients_remain_allowed(self) -> None:
        source = """\
pub fn tessellate_body() {}

#[cfg(test)]
mod tests {
    use super::tessellate_body;

    #[test]
    fn compatibility() {
        tessellate_body();
    }
}
"""
        self.assertEqual(
            find_legacy_body_tessellation_uses(
                {BODY_TESSELLATION_DEFINITION: source}
            ),
            [],
        )

    def test_contextual_names_do_not_match_the_legacy_symbol(self) -> None:
        path = Path("crates/kxt/src/bin/contextual.rs")
        source = "use ktopo::btess::tessellate_body_with_context;\n"
        self.assertEqual(find_legacy_body_tessellation_uses({path: source}), [])

    def test_facade_definition_is_allowed_but_legacy_delegation_is_rejected(self) -> None:
        supported = "pub fn tessellate_body(&self) { tessellate_body_in_scope(); }\n"
        self.assertEqual(
            find_legacy_body_tessellation_uses(
                {FACADE_BODY_TESSELLATION_DEFINITION: supported}
            ),
            [],
        )

        reset = """\
pub fn tessellate_body(&self) {
    ktopo::btess::tessellate_body(&store, body, &options);
}
"""
        self.assertEqual(
            find_legacy_body_tessellation_uses(
                {FACADE_BODY_TESSELLATION_DEFINITION: reset}
            ),
            [f"{FACADE_BODY_TESSELLATION_DEFINITION}:2"],
        )

    def test_public_wrapper_deprecation_is_monotonic(self) -> None:
        deprecated = """\
#[deprecated(since = "0.1.0", note = "use the facade")]
pub fn tessellate_body() {}
"""
        self.assertEqual(
            find_body_tessellation_deprecation_violations(
                {BODY_TESSELLATION_DEFINITION: deprecated}
            ),
            [],
        )
        self.assertEqual(
            find_body_tessellation_deprecation_violations(
                {BODY_TESSELLATION_DEFINITION: "pub fn tessellate_body() {}\n"}
            ),
            [f"{BODY_TESSELLATION_DEFINITION}:missing-deprecated"],
        )

    def test_fake_test_modules_and_legacy_names_in_literals_are_ignored(self) -> None:
        path = Path("crates/kxt/src/bin/literal.rs")
        source = '''\
const RAW: &str = r###"#[cfg(test)] mod tests { tessellate_body( }"###;
const NORMAL: &str = "tessellate_body( }";
// #[cfg(test)] mod tests { tessellate_body(
/* nested /* } */ tessellate_body( */
fn production() {
    let _ = '{';
    tessellate_body();
}
'''
        self.assertEqual(
            find_legacy_body_tessellation_uses({path: source}), [f"{path}:7"]
        )

    def test_code_after_test_module_is_still_audited(self) -> None:
        path = Path("crates/kxt/src/bin/trailing.rs")
        source = '''\
#[cfg(test)]
mod tests {
    #[test]
    fn braces_in_literals_do_not_close_the_module() {
        println!("}}");
        let _ = '}';
        let _ = r#"}"#;
        tessellate_body();
    }
}

fn production_after_tests() {
    tessellate_body();
}
'''
        self.assertEqual(
            find_legacy_body_tessellation_uses({path: source}), [f"{path}:13"]
        )

    def test_cfg_test_module_with_intervening_attributes_is_masked(self) -> None:
        path = Path("crates/kgeom/src/attributed_tests.rs")
        source = """\
#[cfg(test)]
#[allow(clippy::disallowed_methods)]
mod tests {
    fn compatibility() {
        tessellate_body();
    }
}
"""
        self.assertEqual(find_legacy_body_tessellation_uses({path: source}), [])


class FaceTessellationRatchetTests(unittest.TestCase):
    def test_production_import_and_call_are_rejected(self) -> None:
        path = Path("crates/ktopo/src/new_face_client.rs")
        source = """\
use kgeom::tess::tessellate;

fn run() {
    let _ = tessellate(&face, &options);
}
"""
        self.assertEqual(
            find_legacy_face_tessellation_uses({path: source}),
            [f"{path}:1", f"{path}:4"],
        )

    def test_public_wrapper_and_cfg_test_clients_remain_allowed(self) -> None:
        source = """\
pub fn tessellate() {}

#[cfg(test)]
mod tests {
    use super::tessellate;

    #[test]
    fn compatibility() {
        tessellate();
    }
}
"""
        self.assertEqual(
            find_legacy_face_tessellation_uses(
                {FACE_TESSELLATION_DEFINITION: source}
            ),
            [],
        )

    def test_contextual_and_in_scope_names_do_not_match(self) -> None:
        path = Path("crates/ktopo/src/contextual_face.rs")
        source = """\
use kgeom::tess::{tessellate_in_scope, tessellate_with_context};
"""
        self.assertEqual(find_legacy_face_tessellation_uses({path: source}), [])

    def test_current_production_sources_are_closed(self) -> None:
        self.assertEqual(audit_repository(ROOT), [])


class SurfaceProjectionRatchetTests(unittest.TestCase):
    def test_production_import_and_call_are_rejected(self) -> None:
        path = Path("crates/kops/src/new_surface_client.rs")
        source = """\
use kgeom::project::project_to_surface;

fn run() {
    let _ = project_to_surface(&surface, point, window);
}
"""
        self.assertEqual(
            find_legacy_surface_projection_uses({path: source}),
            [f"{path}:1", f"{path}:4"],
        )

    def test_public_definition_and_exact_compatibility_wrappers_are_allowed(self) -> None:
        definition = "pub fn project_to_surface() {}\n"
        compatibility = """\
pub fn invert_surface_point() {
    project_to_surface();
}

pub fn distance_to_surface() {
    project_to_surface();
}
"""
        self.assertEqual(
            find_legacy_surface_projection_uses(
                {
                    SURFACE_PROJECTION_DEFINITION: definition,
                    SURFACE_POINT_COMPATIBILITY: compatibility,
                }
            ),
            [],
        )

    def test_new_call_in_compatibility_module_is_still_rejected(self) -> None:
        source = """\
pub fn invert_surface_point() {
    project_to_surface();
}

fn new_production_helper() {
    project_to_surface();
}
"""
        self.assertEqual(
            find_legacy_surface_projection_uses(
                {SURFACE_POINT_COMPATIBILITY: source}
            ),
            [f"{SURFACE_POINT_COMPATIBILITY}:6"],
        )

    def test_contextual_names_do_not_match(self) -> None:
        path = Path("crates/kops/src/contextual_surface.rs")
        source = """\
use kgeom::project::{project_to_surface_in_scope, project_to_surface_with_context};
"""
        self.assertEqual(find_legacy_surface_projection_uses({path: source}), [])

    def test_current_production_sources_are_closed(self) -> None:
        self.assertEqual(audit_repository(ROOT), [])


class CurveProjectionRatchetTests(unittest.TestCase):
    def test_production_import_and_call_are_rejected(self) -> None:
        path = Path("crates/kops/src/new_curve_client.rs")
        source = """\
use kgeom::project::project_to_curve;
fn run() { project_to_curve(&curve, point, range); }
"""
        self.assertEqual(
            find_legacy_curve_projection_uses({path: source}),
            [f"{path}:1", f"{path}:2"],
        )

    def test_public_definition_and_cfg_test_clients_are_allowed(self) -> None:
        source = """\
pub fn project_to_curve() {}
#[cfg(test)]
mod tests { fn compatibility() { project_to_curve(); } }
"""
        self.assertEqual(
            find_legacy_curve_projection_uses({CURVE_PROJECTION_DEFINITION: source}),
            [],
        )

    def test_contextual_names_do_not_match(self) -> None:
        path = Path("crates/kops/src/contextual_curve.rs")
        source = "use kgeom::project::project_to_curve_in_scope;\n"
        self.assertEqual(find_legacy_curve_projection_uses({path: source}), [])


if __name__ == "__main__":
    unittest.main()
