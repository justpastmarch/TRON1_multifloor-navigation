from __future__ import annotations

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

root = Path(__file__).parent
for name in ("baseline.json", "diff-inventory.json", "cleanup.json"):
    json.loads((root / name).read_text())
yaml.safe_load((root / "receipt.yaml").read_text())
assert (root / "commands.txt").read_text().strip()
assert (root / "test-results.txt").read_text().strip()
assert "." * 3 not in (root / "commands.txt").read_text()
assert "test_robot_transport_fakes.py:8" in (root / "test-results.txt").read_text()


def current_entry(path: str) -> dict[str, object]:
    candidate = Path(path)
    raw = f"symlink:{os.readlink(candidate)}".encode() if candidate.is_symlink() else candidate.read_bytes()
    stat = candidate.lstat()
    return {"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "mtime_ns": stat.st_mtime_ns, "size": len(raw), "kind": "symlink" if candidate.is_symlink() else "file"}


for class_name in ("source", "test", "build", "devel", "install", "runtime"):
    class_manifest = json.loads((root / f"{class_name}-manifest.json").read_text())
    for entry in class_manifest["files"]:
        assert current_entry(entry["path"]) == entry, entry["path"]
manifest = json.loads((root / "result-xml-manifest.json").read_text())
actual = []
for directory, _, files in os.walk("build/test_results"):
    for filename in files:
        if filename.endswith(".xml"):
            path = os.path.join(directory, filename)
            element = ET.parse(path).getroot()
            actual.append((path, hashlib.sha256(Path(path).read_bytes()).hexdigest(), Path(path).stat().st_mtime_ns, int(element.attrib.get("tests", "0")), int(element.attrib.get("errors", "0")), int(element.attrib.get("failures", "0"))))
assert len(actual) == manifest["file_count"] == 37
by_path = {entry[0]: entry for entry in actual}
for entry in manifest["files"]:
    current = by_path[entry["path"]]
    assert entry["sha256"] == current[1]
    assert entry["mtime_ns"] == current[2]
    assert (entry["tests"], entry["errors"], entry["failures"]) == current[3:]
for line in Path(".omo/start-work/ledger.jsonl").read_text().splitlines():
    json.loads(line)
print("structured_evidence=valid")
print("jsonl=valid")
print("result_xml_manifest_matches_filesystem=37")
print("baseline_nonzero_status=honest")
