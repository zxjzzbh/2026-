# Road perception

`RoadPerception` accepts full BGR frames from the calibrated secondary camera. It has no camera, vehicle, UI, GPIO or networking side effects. Frame IDs must increase and timestamps must be monotonic seconds. A repeated frame ID returns the cached observation without adding confirmation votes. Empty, incompatible inputs do not provide valid observations. GroundMap rejects an incompatible aspect ratio or coordinates outside its calibration polygon.

## White boundaries

The detector requires light, low-saturation pixels, connected tape geometry, longitudinal extent and plausible tape width. It does not promote Canny/shadow edges to white lines. Ground-domain lines are fitted to measured tape pixels; a real single side can infer the opposite side using the configured 1.17 m lane width. No fixed 200-pixel lane completion remains. Three fresh consistent frames confirm a lane.

`Lane.metric=true` means positions came from pixels inside the measured ground domain. If both genuine tape strips are outside that domain, the detector first attempts controlled extrapolation with the **same homography** recovered from its measured anchors. Each observed strip must have enough longitudinal image support, projected support spanning at least 0.45 m within the original longitudinal measurement range, and no point/reference more than 0.55 m outside the measured polygon. The two fitted sides must agree in heading and normal lane width. This path reports `metric=false`, `model_projected=true`, and a nonzero `lateral_uncertainty_m`; it does not extend `GroundMap::project()` or claim a new calibration.

The decision allowance is 0.03 m plus model-fit residual plus 0.05 times the extrapolation distance. It is an engineering margin for conservative route selection, **not a measured error bound or a guarantee**. The controller must use it when evaluating free width and the right-side 50 cm rule. With the actual supplied homography, a real right boundary at 47.5 cm is reconstructed near 47.5 cm instead of the previous image-centre estimate near 52 cm.

If this controlled projection cannot be supported, the remaining pixel-width estimate has both `metric=false` and `model_projected=false`. It can provide a low-confidence steering direction only; it cannot justify centimetre clearances or obstacle passing-side selection. There is no single-image-side fallback outside the calibration domain.

Broad white glare patches fail the tape-width/geometry checks and dark shadows fail the white-pixel checks. A completely overexposed image cannot recover hidden paint, and thin bright reflections can still resemble tape. Confidence and image/metric status therefore remain explicit. Lighting, fixed camera pose and the correct lane width must be checked using real frames; synthetic tests do not establish field success.

## Cones and launch board

Cones require component shape and calibrated ground contact points. A cone can grow beyond the old 800-pixel cap and remain a target. Unprojectable cone candidates increment `diagnostics().unmapped_cones` and do not receive invented metre coordinates. The blue masks are not changed by debug display switches.

The launch board is a separate **image presence** observation: a large filled convex quadrilateral, suitable aspect ratio, fully inside the image and central front corridor. A mapped base must also have a board-sized width; an unmapped board is only admitted in the lower image and never receives a fictitious ground position. Three overlapping rectangle observations confirm the board. A blue cone triangle cannot satisfy this rectangle requirement.

Cone IDs associate fresh detections with motion-transformed prior targets. Only fresh detections appear in the result; temporary target memory and actual passed-cone decisions belong to the controller. IDs are preserved across short misses but do not prove passage. Board removal is evaluated in the actual previously confirmed rectangle: its lower region must reveal non-blue, non-overexposed floor colour while current white boundaries remain confirmed. Initial absence, black frames, broad glare or a remaining partial board do not establish removal. A long frame gap clears this image reference. The start controller must still latch board presence and then confirm consecutive valid absence; fixed ground points outside this camera's measured field no longer make that condition impossible.

## Zebra crossing

The detector groups at least three ground-plane white strips consistent with the 10.5 × 29.7 cm strips and approximately 21 cm centre spacing. Their near edges and directions must agree. The near edge is reported relative to the rear axle. Confirmation uses new frames only. Stopping distance, front overhang, braking and the ten-second stationary hold are controller responsibilities.

## Motion and stop evidence

Ground optical flow uses forward/backward Lucas–Kanade checks, projection into ground coordinates, a robust partial affine fit, inlier fraction, distribution over a ground patch, scale consistency and residual limits. Blue vertical objects are masked out. A frame gap, insufficient features, concentrated features or a poor fit marks motion invalid instead of converting elapsed time into travelled distance.

`lateral_delta` is rightwards and `forward_delta` forwards, both in metres in the previous rear-axle frame. `yaw_delta` is positive for a right turn. To transform a static target into the new vehicle frame, subtract the translation and apply `[[cos,-sin],[sin,cos]]`. Stop confirmation requires sustained near-zero observed translation and rotation, at least five successful updates and `Config.settle_s`. A neutral/brake PWM command alone is not stopped evidence.

These estimates assume stationary, planar, textured ground and fixed camera pose. They are not wheel odometry and cannot validate motion when those assumptions fail. `tests/test_perception.cpp` includes synthetic lane, glare/shadow rejection, fallback, zebra layout, cone/board discrimination, ID persistence, duplicate-frame handling, ground translation and stationary/textureless cases. It also renders a 47.5 cm right-side boundary with the supplied real homography, checks that the measured map domain remains unchanged, and tests board presence/removal with that same map. These added scenes are synthetic regression inputs, not recordings of a completed field trial. Build with OpenCV `core`, `imgproc`, `video` and `calib3d`.
