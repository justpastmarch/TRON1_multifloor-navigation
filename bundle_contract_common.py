"""Shared filesystem boundaries for offline bundle validation."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ElementTree


GENERATED_DIRS = frozenset({"build", "devel", "install", "logs", "__pycache__"})


def fail(message: str) -> None:
    raise SystemExit("BUNDLE_ERROR: " + message)


def project_files(root: Path) -> tuple[Path, ...]:
    return tuple(
        path
        for path in root.rglob("*")
        if path.is_file()
        and not GENERATED_DIRS.intersection(path.relative_to(root).parts)
        and not any(part.startswith(".") for part in path.relative_to(root).parts)
    )


def parse_xml(path: Path, root: Path) -> ElementTree.Element:
    try:
        return ElementTree.parse(path).getroot()
    except (ElementTree.ParseError, OSError) as error:
        fail(f"invalid XML {path.relative_to(root)}: {error}")
