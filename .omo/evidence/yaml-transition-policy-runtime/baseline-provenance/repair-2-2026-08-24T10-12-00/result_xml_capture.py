import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

entries = []
for directory, _, files in os.walk("build/test_results"):
    for filename in sorted(files):
        if not filename.endswith(".xml"):
            continue
        path = os.path.join(directory, filename)
        root = ET.parse(path).getroot()
        entries.append({"path": path, "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(), "mtime_ns": Path(path).stat().st_mtime_ns, "tests": int(root.attrib.get("tests", "0")), "errors": int(root.attrib.get("errors", "0")), "failures": int(root.attrib.get("failures", "0"))})
entries.sort(key=lambda entry: entry["path"])
manifest = {"schema_version": 1, "root": "build/test_results", "explicit_exclusions": ["non-XML files"], "file_count": len(entries), "files": entries}
canonical = "".join(json.dumps(entry, sort_keys=True) + "\n" for entry in entries).encode()
manifest["manifest_sha256"] = hashlib.sha256(canonical).hexdigest()
Path(__file__).parent.joinpath("result-xml-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps({"file_count": len(entries), "manifest_sha256": manifest["manifest_sha256"]}))
