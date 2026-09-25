"""Validate YOLO labels and prevent train/validation session leakage."""

import hashlib
from pathlib import Path

import numpy as np
import yaml

from .results import RACE_CLASSES
from .sources import IMAGE_EXTENSIONS, read_image


def check_dataset(yaml_path):
    yaml_path = Path(yaml_path).resolve()
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("dataset YAML must be a mapping")
    names = data.get("names")
    if isinstance(names, dict):
        names = [names[k] for k in sorted(names)]
    if names != list(RACE_CLASSES):
        raise ValueError(f"names must be {list(RACE_CLASSES)} in this order")
    root = Path(data.get("path", "."))
    root = (yaml_path.parent/root).resolve() if not root.is_absolute() else root.resolve()
    errors = []
    report = {"root": str(root), "splits": {}, "errors": errors}
    sessions, hashes = {}, {}
    resolved = {"path": str(root), "names": list(RACE_CLASSES)}
    for split in ("train", "val"):
        value = data.get(split)
        if not isinstance(value, str):
            raise ValueError(f"{split} must name an image directory")
        folder = (root/value).resolve()
        # Keep a predictable mapping from images to labels, without guessing paths.
        if not folder.is_relative_to(root/"images"):
            raise ValueError(f"{split} must be a directory under <path>/images")
        resolved[split] = str(folder)
        files = sorted(p for p in folder.rglob("*") if p.suffix.lower() in IMAGE_EXTENSIONS)
        if not files:
            errors.append(f"{split}: no images in {folder}")
        counts = [0]*len(RACE_CLASSES)
        negatives = 0
        for image_path in files:
            relative = image_path.relative_to(folder)
            if len(relative.parts) < 2:
                errors.append(f"{relative}: put images in a session subfolder, e.g. images/{split}/day1-run1/")
            else:
                session = relative.parts[0]
                if session in sessions and sessions[session] != split:
                    errors.append(f"session crosses splits: {session}")
                sessions[session] = split
            try:
                image = read_image(image_path)
                # Pixel hash also catches identical images saved with different metadata.
                digest = hashlib.sha256(str(image.shape).encode()+image.tobytes()).hexdigest()
                if digest in hashes and hashes[digest][0] != split:
                    errors.append(f"duplicate image crosses splits: {image_path}, {hashes[digest][1]}")
                hashes[digest] = (split, str(image_path))
            except (ValueError, OSError) as exc:
                errors.append(str(exc))
            label_path = root/"labels"/image_path.relative_to(root/"images").with_suffix(".txt")
            if not label_path.is_file():
                errors.append(f"missing label: {label_path}; confirmed negatives need an empty .txt")
                continue
            lines = label_path.read_text(encoding="utf-8-sig").splitlines()
            if not any(line.strip() for line in lines):
                negatives += 1
            for line_no, line in enumerate(lines, 1):
                if not line.strip():
                    continue
                try:
                    parts = np.asarray([float(v) for v in line.split()])
                    if len(parts) != 5 or not np.isfinite(parts).all():
                        raise ValueError("requires five finite values")
                    cls, x, y, w, h = parts
                    if cls != int(cls) or not 0 <= cls < len(RACE_CLASSES):
                        raise ValueError("class id out of range")
                    if not (0 < w <= 1 and 0 < h <= 1 and 0 <= x-w/2 and x+w/2 <= 1
                            and 0 <= y-h/2 and y+h/2 <= 1):
                        raise ValueError("normalized box extends outside image or has zero size")
                    counts[int(cls)] += 1
                except ValueError as exc:
                    errors.append(f"{label_path}:{line_no}: {exc}")
        for i, count in enumerate(counts):
            if count == 0:
                errors.append(f"{split}: no labeled examples for {RACE_CLASSES[i]}")
        report["splits"][split] = {"images": len(files), "negative_images": negatives,
                                    "objects": dict(zip(RACE_CLASSES, counts))}
    report["valid"] = not errors
    return report, resolved
