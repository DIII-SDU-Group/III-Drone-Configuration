#!/usr/bin/env python3
"""Retired unsafe parameter-file mutation entry point."""

import sys


def main() -> int:
    print(
        "update_installed_parameters.py is retired: use 'iii config sim inspect', "
        "'iii config sim checkpoint', or confirmed 'iii config sim reset'. "
        "Aircraft configuration is reconciled only by the deployment receiver.",
        file=sys.stderr,
    )
    return 64


if __name__ == "__main__":
    raise SystemExit(main())
