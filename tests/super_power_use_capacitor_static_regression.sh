#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
header="$module_dir/SuperPower.hpp"

python3 - "$header" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_text(encoding="utf-8")
required = (
    "RegisterUseCapacitorCallback();",
    'FindOrCreate<bool>(',
    '"use_capacitor"',
    "bool use_capacitor_ = true;",
    "use_capacitor_ ? SuperPowerProtocol::ENABLE_DCDC_MASK : 0U",
)
for token in required:
    if token not in source:
        raise SystemExit(f"missing SuperPower use_capacitor contract: {token}")
print("PASS: SuperPower use_capacitor static contract")
PY
