#!/usr/bin/env python3
"""Reject dependency drift before running the fork's complete simulation suite."""
import importlib
import importlib.metadata
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]


def check():
    records = {}
    for raw in (ROOT / ".github/requirements-ci.lock").read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if " @ git+" in line:
            name, source = line.split(" @ git+", 1)
            url, commit = source.rsplit("@", 1)
            if not re.fullmatch(r"[0-9a-f]{40}", commit):
                raise RuntimeError(f"non-immutable VCS pin: {line}")
            dist = importlib.metadata.distribution(name)
            actual = json.loads(dist.read_text("direct_url.json") or "{}")
            if actual.get("url") != url or actual.get("vcs_info", {}).get("commit_id") != commit:
                raise RuntimeError(f"{name}: wrong installed VCS source: {actual}")
            records[name] = dict(version=dist.version, url=url, commit=commit)
        else:
            name, expected = line.split("==", 1)
            actual = importlib.metadata.version(name)
            if actual != expected:
                raise RuntimeError(f"{name}: expected {expected}, found {actual}")
            records[name] = dict(version=actual)
    imports = {name: str(Path(importlib.import_module(name).__file__).resolve())
               for name in ("litenvme", "litepcie", "litex", "migen")}
    if not Path(imports["litenvme"]).is_relative_to(ROOT / "litenvme"):
        raise RuntimeError(f"LiteNVMe import is not the checked-out source: {imports['litenvme']}")
    return dict(python=sys.version, dependencies=records, imports=imports)


if __name__ == "__main__":
    destination = ROOT / "build/ci/provenance.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.unlink(missing_ok=True)
    try:
        evidence = dict(status="passed", **check())
    except Exception as error:
        destination.write_text(json.dumps(dict(status="failed", error=str(error)), indent=2) + "\n")
        raise
    destination.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print(json.dumps(evidence, indent=2, sort_keys=True))
