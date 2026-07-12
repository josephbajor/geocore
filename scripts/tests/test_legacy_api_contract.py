"""Tests for the internal legacy-API retirement ratchet."""

import unittest
from pathlib import Path

from scripts.legacy_api_contract import (
    BODY_TESSELLATION_DEFINITION,
    FACE_TESSELLATION_DEFINITION,
    audit_repository,
    find_legacy_body_tessellation_uses,
    find_legacy_face_tessellation_uses,
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


if __name__ == "__main__":
    unittest.main()
