"""Fetch only the pinned Go2 assets; never fetch or execute upstream scripts."""

import hashlib
import json
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "assets" / "go2-assets.json"
ASSET_DIR = ROOT / ".assets" / "go2"


def ensure_assets() -> Path:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for entry in manifest["files"]:
        dest = ASSET_DIR / entry["path"]
        if not dest.exists():
            print(f"Downloading Go2 asset: {entry['path']}")
            try:
                with urlopen(entry["url"], timeout=60) as response:
                    content = response.read()
            except (OSError, URLError) as exc:
                raise RuntimeError(f"Cannot download {entry['path']}: {exc}") from exc
            if hashlib.sha256(content).hexdigest() != entry["sha256"]:
                raise RuntimeError(f"Go2 asset checksum mismatch: {entry['path']}")
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(content)
        if hashlib.sha256(dest.read_bytes()).hexdigest() != entry["sha256"]:
            raise RuntimeError(f"Go2 asset checksum mismatch: {dest}; remove this file and retry")
    return ASSET_DIR
