from dataclasses import asdict, dataclass, field

RACE_CLASSES = ("cone", "blue_board", "traffic_light", "crosswalk")


@dataclass
class Detection:
    label: str
    score: float
    bbox_xyxy: list[float]


@dataclass
class LaneResult:
    valid: bool = False
    reason: str = "not_processed"
    quality: float = 0.0
    offset_normalized: float | None = None
    left: list[list[float]] = field(default_factory=list)
    right: list[list[float]] = field(default_factory=list)
    center: list[list[float]] = field(default_factory=list)
    target: list[float] | None = None


@dataclass
class PerceptionResult:
    frame_id: int
    source_time_ms: float | None
    receive_monotonic_ms: float
    image_size: list[int]
    lane: LaneResult
    detector_status: str
    detections: list[Detection]
    presence: dict[str, str]
    traffic_light_state: str = "unknown"
    schema_version: str = "0.1"
    source_status: str = "ok"
    mode: str = "perception_only"
    processing_ms: float = 0.0
    receive_to_result_ms: float = 0.0

    def to_dict(self):
        return asdict(self)
