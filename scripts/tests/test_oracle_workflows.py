"""Static contracts separating offline CI from licensed-host catch-up."""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class OracleWorkflowTests(unittest.TestCase):
    def test_automatic_ci_contains_only_offline_oracle_commands(self):
        ci = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
        self.assertNotIn("oracle_loop.py bundle", ci)
        self.assertNotIn("ONSHAPE_ACCESS_KEY", ci)
        self.assertIn("certification-check", ci)
        self.assertIn("offline oracle contracts", ci)

    def test_licensed_host_workflow_is_manual_serial_and_capped(self):
        workflow = (ROOT / ".github" / "workflows" / "oracle-catchup.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn("workflow_dispatch:", workflow)
        self.assertNotIn("pull_request:", workflow)
        self.assertNotIn("schedule:", workflow)
        self.assertNotIn("push:", workflow)
        self.assertIn("cancel-in-progress: false", workflow)
        self.assertIn("environment: onshape-oracle", workflow)
        self.assertIn("permissions:\n  contents: read", workflow)
        self.assertIn("|400)$", workflow)
        self.assertIn("ONSHAPE_REQUEST_LIMIT", workflow)
        self.assertIn("persist-credentials: false", workflow)
        self.assertIn("--completion-file", workflow)
        self.assertIn("--fixtures", workflow)
        self.assertIn("base-identity.json", workflow)
        self.assertIn("run-metadata.json", workflow)
        self.assertIn("[redacted]", workflow)
        self.assertIn('BASE_COMPLETION" != "$BASE_EXIT', workflow)
        self.assertIn("expected_bundle_sha256", workflow)
        self.assertNotIn("          - both\n", workflow)


if __name__ == "__main__":
    unittest.main()
