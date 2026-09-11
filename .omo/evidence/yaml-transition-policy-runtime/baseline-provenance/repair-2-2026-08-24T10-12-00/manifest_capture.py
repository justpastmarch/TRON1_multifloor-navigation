from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

OUT = Path(__file__).parent
CLASSES = {
    "source": ["src"],
    "test": ["test", "src/mission_manager/test", "src/multifloor_manager/test", "src/stair_supervisor/test"],
    "build": ["build"],
    "devel": ["devel"],
    "install": ["install"],
    "runtime": ["src/mission_manager/scripts", "src/mission_manager/launch", "src/multifloor_manager/scripts", "src/multifloor_manager/launch", "src/stair_supervisor/scripts", "src/stair_supervisor/launch", "src/mission_manager/package.xml", "src/multifloor_manager/package.xml", "src/stair_supervisor/package.xml"],
}
EXCLUSIONS = [".git/**", ".omo/**", "**/__pycache__/**", "**/*.pyc", "**/*.pyo"]


def excluded(path: str) -> bool:
    return path.startswith(".git/") or path.startswith(".omo/") or "/__pycache__/" in f"/{path}/" or path.endswith((".pyc", ".pyo"))


def collect(item: str) -> list[Path]:
    path = Path(item)
    if path.is_file() or path.is_symlink():
        return [path]
    if not path.is_dir():
        return []
    result: list[Path] = []
    for directory, directories, files in os.walk(path, followlinks=False):
        directories[:] = [name for name in directories if not excluded((Path(directory) / name).as_posix())]
        result.extend(Path(directory) / name for name in files)
        result.extend(Path(directory) / name for name in directories if (Path(directory) / name).is_symlink())
    return result


for name, roots in CLASSES.items():
    paths = sorted({path for item in roots for path in collect(item) if not excluded(path.as_posix())})
    entries = []
    for path in paths:
        raw = f"symlink:{os.readlink(path)}".encode() if path.is_symlink() else path.read_bytes()
        stat = path.lstat()
        entries.append({"path": path.as_posix(), "sha256": hashlib.sha256(raw).hexdigest(), "mtime_ns": stat.st_mtime_ns, "size": len(raw), "kind": "symlink" if path.is_symlink() else "file"})
    canonical = "".join(f"{entry['path']}\0{entry['sha256']}\0{entry['mtime_ns']}\0{entry['size']}\0{entry['kind']}\n" for entry in entries).encode()
    manifest = {"schema_version": 2, "class": name, "roots": roots, "explicit_exclusions": EXCLUSIONS, "file_count": len(entries), "manifest_sha256": hashlib.sha256(canonical).hexdigest(), "files": entries}
    (OUT / f"{name}-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"class": name, "file_count": len(entries), "manifest_sha256": manifest["manifest_sha256"]}))
