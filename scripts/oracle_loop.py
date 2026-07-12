#!/usr/bin/env python3
"""Thin entrypoint for the importable oracle-loop tooling."""

import sys

from oracle.cli import main


if __name__ == "__main__":
    sys.exit(main())
