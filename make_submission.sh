#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-python}" != python || $# -gt 1 ]]; then
    echo "usage: bash make_submission.sh [python]" >&2
    exit 2
fi
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${PYTHON:-python3}" "$project_root/tools/package_submission.py"
