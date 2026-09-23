"""Read-only check of copied archive bytes; no ROS or source-data dependencies."""
import hashlib
import json
from collections import Counter
from pathlib import Path


def main():
    archive = Path(__file__).resolve().parent
    repository = archive.parent.parent
    manifest = json.loads((archive / "manifest.json").read_text())
    errors = []
    checked = 0

    def check(path, expected):
        nonlocal checked
        checked += 1
        if not path.is_file():
            errors.append({"path": str(path), "error": "missing"})
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            errors.append({"path": str(path), "error": "hash mismatch"})

    counts = dict(Counter(item["status"] for item in manifest["files"]))
    if counts != manifest["counts"] or len(manifest["files"]) != manifest["inventory_count"]:
        errors.append({"error": "inventory/count mismatch"})
    for item in manifest["files"]:
        if item["status"] == "ARCHIVE":
            check(repository / item["destination"], item["archived_sha256"])

    previous = repository / "docs/tron-documentation-20260922"
    previous_manifest = json.loads((previous / "manifest.json").read_text())
    for item in previous_manifest["documents"]:
        check(previous / item["markdown"], item["markdown_sha256"])
    for item in previous_manifest["assets"]:
        check(previous / item["path"], item["sha256"])
    result = {
        "scope": "Copied reports and existing converted Markdown/media hashes only; excludes original HTML, local-only data, links, runtime and physical behavior.",
        "inventory_count": len(manifest["files"]),
        "classified_counts": counts,
        "hashes_checked": checked,
        "errors": errors,
        "passed": not errors,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
