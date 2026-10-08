"""Configuration with explicit validation; no hardware capability assumptions."""

import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path


@dataclass
class LaneConfig:
    roi_top: float = 0.45
    sample_bottom: float = 0.94
    sample_top: float = 0.50
    sample_rows: int = 16
    white_s_max: int = 95
    white_v_min: int = 160
    min_run_fraction: float = 0.004
    max_run_fraction: float = 0.065
    min_width_fraction: float = 0.16
    max_width_fraction: float = 0.95
    min_pairs: int = 6
    min_coverage: float = 0.45
    max_fit_residual_fraction: float = 0.035
    lookahead_y: float = 0.70


@dataclass
class YoloConfig:
    image_size: int = 416
    confidence: float = 0.35
    iou: float = 0.5
    device: str = "cpu"


@dataclass
class TemporalConfig:
    confirm_ms: float = 200
    max_gap_ms: float = 250


@dataclass
class LightConfig:
    min_colored_fraction: float = 0.02
    min_bright_fraction: float = 0.008
    min_value: int = 180
    min_saturation: int = 90


@dataclass
class Config:
    lane: LaneConfig = field(default_factory=LaneConfig)
    yolo: YoloConfig = field(default_factory=YoloConfig)
    temporal: TemporalConfig = field(default_factory=TemporalConfig)
    traffic_light: LightConfig = field(default_factory=LightConfig)

    def validate(self):
        c = self.lane
        if not (0 <= c.roi_top <= c.sample_top < c.lookahead_y < c.sample_bottom < 1):
            raise ValueError("lane ROI/sample/lookahead fractions must be ordered within [0,1)")
        if not (3 <= c.min_pairs <= c.sample_rows <= 100):
            raise ValueError("lane needs 3 <= min_pairs <= sample_rows <= 100")
        for low, high in [(c.min_run_fraction, c.max_run_fraction),
                          (c.min_width_fraction, c.max_width_fraction)]:
            if not 0 < low < high <= 1:
                raise ValueError("lane min/max fractions must satisfy 0 < min < max <= 1")
        if not 0 < c.min_coverage <= 1 or not 0 < c.max_fit_residual_fraction < 1:
            raise ValueError("invalid lane coverage/residual")
        for v in [c.white_s_max, c.white_v_min, self.traffic_light.min_value,
                  self.traffic_light.min_saturation]:
            if not 0 <= v <= 255:
                raise ValueError("HSV S/V bounds must be 0..255")
        if not (0 < self.yolo.confidence <= 1 and 0 < self.yolo.iou <= 1):
            raise ValueError("YOLO confidence/iou must be in (0,1]")
        if self.yolo.image_size < 32 or self.yolo.image_size % 32:
            raise ValueError("YOLO image_size must be a positive multiple of 32")
        if self.temporal.confirm_ms <= 0 or self.temporal.max_gap_ms <= 0:
            raise ValueError("temporal intervals must be positive")
        for v in [self.traffic_light.min_colored_fraction, self.traffic_light.min_bright_fraction]:
            if not 0 < v <= 1:
                raise ValueError("light fractions must be in (0,1]")
        return self

    def to_dict(self):
        return asdict(self)


def load_config(path: str | Path | None = None) -> Config:
    cfg = Config()
    if path is not None:
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        for section, values in data.items():
            if section not in {f.name for f in fields(cfg)}:
                raise ValueError(f"unknown config section: {section}")
            current = getattr(cfg, section)
            if not isinstance(values, dict):
                raise ValueError(f"{section} must be an object")
            # Dataclass constructor rejects unknown parameters.
            setattr(cfg, section, type(current)(**(asdict(current) | values)))
    return cfg.validate()
