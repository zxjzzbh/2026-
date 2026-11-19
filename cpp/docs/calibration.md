# Camera geometry and parking evidence

`ground_road.json` and `ground_parking.json` use the secondary camera. Coordinates
are metres with the rear axle centre as origin, positive x right, positive y
forward. They are separate fitted mappings; never apply one map to the other
camera or to an already cropped/stretched image.

The road mapping comes from the existing fixed-pose 1600/1150 record. The user
previously confirmed that camera angles were unchanged. The parking map comes
from the 2026-11-13 ruler fit, transformed to the rear axle using the recorded
front/rear wheel offsets. Four independent marker centres extend the measured
image envelope. Import verifies original SHA-256 checksums, the rear-axle
transform, marker errors and the reconstructed sample hull. Original JSON
records and hashes are under `sources/` and `provenance.json`.

No `verified` or dynamic-validation flag is promoted. The 1.00 m bay and 1.24 m
lane in the original measurement describe the calibration scene, not the race
configuration. The race bay remains 1.50 m and the outdoor lane 1.17 m.

`GroundMap::project` refuses pixels outside the sampled polygon. Whole-frame
uniform resize is supported with pixel-centre scaling; a changed aspect ratio
is rejected. Cropping cannot be detected from dimensions alone and must never
be passed as a full calibrated frame. `unproject` has the same measured-domain
restriction. Mapping does not claim dynamic accuracy or remove camera blind
areas. The road polygon does not span both 1.17 m lane edges at every distance.

Parking detects warm yellow tape candidates and their geometry. Low saturation
is supported for sunlit tape, but pure white has no yellow chromatic evidence. A yellow
divider and the outer white lane boundaries or yellow perimeter bands identify
two half-bays; yellow transverse bands identify entrance
and end. Initialization requires consecutive observations. If one transverse
band is outside the measured domain but the image structure identifies both,
the 1.50 m length can infer the other distance at lower confidence. The inferred
edge's `entry_visible` or `end_visible` flag remains false. The measured domain
is never expanded to make an unseen edge appear measured.

A visible empty half-bay can initialize even when the blue board hides the other
outer boundary. Actual side/entrance tape and the real far image edge must meet
at supported intersections. Only observed line pixels inside the calibration
domain supply metres; the configured bay width/length infer hidden geometry at
lower confidence. A partly mapped blue board uses only its observed in-domain
base samples. Collinear tape fragments are grouped before checking their total
supported span. The same low-saturation yellow exclusion applies to floor
clearance scoring in both the full-bay and half-bay paths.

After acquisition, measured motion updates the same bay through
`predictMotion(forward_m, lateral_m, yaw_right_rad)`. An entrance disappearing
does not establish that any wheel crossed it. Current transverse observations
are associated against the tracked entry/end distances and matching side
bands. Lost observation confidence decays. Blue boards are assigned by their
ground-contact location in that bay. Overexposed white regions are unknown
surface, not synthesized free ground. A car has not successfully parked merely
because the detector supplies a bay; the controller must also check wheel/body
geometry, measured progress and final stopping evidence.

The detector now clears its internal track when confidence expires, so a newly
visible bay can be initialized again from three current observations instead of
being compared forever against an obsolete position. `invalidateMotion()` marks
a discontinuity in measured progress and discards old coordinates. It is
idempotent within one continuous missing-motion episode: repeated calls do not
erase fresh visual candidate votes, so stationary optical-flow failure does not
prevent visual relocalization. `predictMotion()` returns false for invalid or
out-of-range increments; the caller must not apply such an increment elsewhere.
`generation()` increases after each successful multi-frame acquisition, allowing
the controller to distinguish fresh relocalization from continued prediction of
an old bay. `Bay.generation` carries that value. `Bay.current_geometry` is true
only for a current full/half-bay acquisition or sufficiently matched side and
transverse landmarks; motion prediction alone cannot set it. No generation is
produced by a single recovered motion increment.

Import (on the development computer, no vehicle connection):

```text
python tools/import_calibration.py --vision <current-release>/candidate/vision
```

Synthetic tests cover mapping domains, resize, current tape geometry, wrong
board side, full glare rejection and continued end identity after the entrance
leaves view. They do not replace an actual supervised trial.
