from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import refusal_frontier  # noqa: E402


class RefusalFrontierTests(unittest.TestCase):
    def test_census_covers_audited_declarations(self) -> None:
        self.assertEqual(refusal_frontier.audit(), [])


if __name__ == "__main__":
    unittest.main()
