"""Verify saved documents, local links, copied media and source preservation."""
import hashlib
import json
import re
import subprocess
from pathlib import Path
from urllib.parse import unquote, urlsplit

from bs4 import BeautifulSoup

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent
manifest = json.loads((OUT / "manifest.json").read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


documents = [OUT / item["markdown"] for item in manifest["documents"]]
guides = sorted(OUT.glob("*.md"))
rendered = {}
for path in documents + guides:
    result = subprocess.run(
        ["pandoc", "-f", "gfm", "-t", "html", "--wrap=none", str(path)],
        capture_output=True, text=True, check=True,
    )
    rendered[path.resolve()] = BeautifulSoup(result.stdout, "html.parser")

missing = []
anchors = []
checked = 0
for path, soup in rendered.items():
    for node in soup.find_all(["a", "img", "source", "video"]):
        link = node.get("href") if node.name == "a" else node.get("src")
        if not link:
            continue
        parts = urlsplit(link)
        if parts.scheme not in ("", "file"):
            continue
        checked += 1
        raw = unquote(parts.path)
        target = Path(raw) if raw.startswith("/") else path.parent / raw
        if not raw:
            target = path
        target = Path(re.sub(r":\d+$", "", str(target))).resolve()
        if not target.exists():
            missing.append({"document": str(path.relative_to(OUT)), "link": link})
        elif parts.fragment and target in rendered:
            identifiers = {str(x["id"]) for x in rendered[target].find_all(id=True)}
            if unquote(parts.fragment) not in identifiers:
                anchors.append({"document": str(path.relative_to(OUT)), "link": link})

actual_sources = set(subprocess.check_output([
    "rg", "--files", "--hidden", "--no-ignore", "-g", "*.html", "-g", "*.htm",
    "-g", "!**/.git/**", "-g", "!sources/**", "-g", "!tron-documentation-20260922/**",
], cwd=ROOT, text=True).splitlines())
expected_sources = {item["source"] for item in manifest["documents"]}
result = {
    "documents": len(documents),
    "guides": len(guides),
    "inventory_matches": actual_sources == expected_sources,
    "text_blocks": sum(item["text_blocks_checked"] for item in manifest["documents"]),
    "unmatched_text_blocks": sum(len(item["text_blocks_not_found"]) for item in manifest["documents"]),
    "text_validation_method": "Converter round-trip text comparison, bound to unchanged Markdown hashes.",
    "local_links_checked": checked,
    "missing_paths": missing,
    "broken_internal_anchors": anchors,
    "assets": len(manifest["assets"]),
    "mp4_files": sum(item["path"].endswith(".mp4") for item in manifest["assets"]),
    "asset_hashes_match": all(digest(OUT / item["path"]) == item["sha256"] for item in manifest["assets"]),
    "markdown_hashes_match": all(digest(OUT / item["markdown"]) == item["markdown_sha256"] for item in manifest["documents"]),
    "original_html_unchanged": all(digest(ROOT / item["source"]) == item["source_sha256"] for item in manifest["documents"]),
}
result["passed"] = all(result[key] for key in [
    "inventory_matches", "asset_hashes_match", "markdown_hashes_match", "original_html_unchanged",
]) and not (missing or anchors or result["unmatched_text_blocks"])
(OUT / "conversion-validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
print(json.dumps(result, ensure_ascii=False, indent=2))
raise SystemExit(0 if result["passed"] else 1)
