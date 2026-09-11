"""Load a map_server YAML/PGM target into a pre-call fingerprint identity."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import struct
from typing import List, Tuple

import yaml

from multifloor_manager.map_evidence import MapIdentity, MapOrigin


@dataclass(frozen=True)
class MapFileError(ValueError):
    __slots__ = ("path", "detail")
    path: Path
    detail: str

    def __str__(self) -> str:
        return "{}: {}".format(self.path, self.detail)


def _tokens(content: bytes) -> Tuple[List[bytes], int]:
    tokens: List[bytes] = []
    offset = 0
    while len(tokens) < 4:
        while offset < len(content) and chr(content[offset]).isspace():
            offset += 1
        if offset < len(content) and content[offset] == ord("#"):
            while offset < len(content) and content[offset] not in (10, 13):
                offset += 1
            continue
        start = offset
        while offset < len(content) and not chr(content[offset]).isspace():
            offset += 1
        if start == offset:
            break
        tokens.append(content[start:offset])
    while offset < len(content) and chr(content[offset]).isspace():
        offset += 1
    return tokens, offset


def _read_pgm(path: Path) -> Tuple[int, int, int, Tuple[int, ...]]:
    content = path.read_bytes()
    tokens, payload_at = _tokens(content)
    if len(tokens) != 4 or tokens[0] not in (b"P2", b"P5"):
        raise MapFileError(path, "expected P2 or P5 PGM")
    try:
        width, height, maximum = (int(token) for token in tokens[1:])
    except ValueError:
        raise MapFileError(path, "invalid PGM header") from None
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 255:
        raise MapFileError(path, "invalid PGM dimensions or maximum")
    if tokens[0] == b"P5":
        pixels = tuple(content[payload_at:])
    else:
        try:
            pixels = tuple(int(token) for token in content[payload_at:].split())
        except ValueError:
            raise MapFileError(path, "invalid P2 pixel") from None
    if len(pixels) != width * height or any(pixel < 0 or pixel > maximum for pixel in pixels):
        raise MapFileError(path, "pixel count or range is invalid")
    return width, height, maximum, pixels


def _number(document: dict, key: str, path: Path) -> float:
    value = document.get(key)
    if type(value) not in (int, float) or not math.isfinite(value):
        raise MapFileError(path, "{} must be finite numeric".format(key))
    return float(value)


def load_map_identity(yaml_path: Path, frame_id: str) -> MapIdentity:
    """Resolve map YAML and compute the exact trinary map_server content identity."""
    resolved_yaml = yaml_path.resolve(strict=True)
    try:
        document = yaml.safe_load(resolved_yaml.read_text(encoding="utf-8"))
    except (UnicodeError, yaml.YAMLError) as error:
        raise MapFileError(yaml_path, "invalid YAML: {}".format(error)) from None
    if type(document) is not dict:
        raise MapFileError(yaml_path, "expected YAML mapping")
    image = document.get("image")
    origin = document.get("origin")
    if type(image) is not str or not image or type(origin) is not list or len(origin) != 3:
        raise MapFileError(yaml_path, "image and three-value origin are required")
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in origin):
        raise MapFileError(yaml_path, "origin must be finite numeric")
    resolution = _number(document, "resolution", yaml_path)
    occupied = _number(document, "occupied_thresh", yaml_path)
    free = _number(document, "free_thresh", yaml_path)
    negate = document.get("negate")
    if resolution <= 0 or not 0 <= free < occupied <= 1 or negate not in (0, 1):
        raise MapFileError(yaml_path, "invalid resolution, thresholds, or negate")
    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = resolved_yaml.parent / image_path
    width, height, maximum, pixels = _read_pgm(image_path.resolve(strict=True))
    cells = []
    for row in range(height - 1, -1, -1):
        for column in range(width):
            pixel = pixels[row * width + column]
            occupancy = pixel / maximum if negate else (maximum - pixel) / maximum
            cells.append(100 if occupancy > occupied else 0 if occupancy < free else -1)
    yaw = float(origin[2])
    map_origin = MapOrigin(
        float(origin[0]),
        float(origin[1]),
        0.0,
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )
    data_hash = hashlib.sha256(bytes(value & 0xFF for value in cells)).hexdigest()
    wire_resolution = struct.unpack("<f", struct.pack("<f", resolution))[0]
    return MapIdentity(frame_id, width, height, wire_resolution, map_origin, data_hash)
