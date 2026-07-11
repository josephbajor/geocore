#!/usr/bin/env python3
"""Thin entrypoint for the importable kernel benchmark tooling."""

import sys

from benchmark.cli import main


if __name__ == "__main__":
    sys.exit(main())
