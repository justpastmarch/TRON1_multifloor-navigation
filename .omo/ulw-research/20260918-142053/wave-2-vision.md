# Wave 2: Vision

Worker: `bg_85893363`

## Conclusion

Forward RGB is suitable as a secondary stair-presence, edge-consistency, gross-alignment, and conditional heading signal. It is not justified as TRON1's sole primary metric feedback. Monocular lateral centering is possible when both stair boundaries or repeated edges are visible and calibrated, but it remains geometry- and visibility-dependent.

## Key evidence

- Xiong and Matthies, *Autonomous Stair Climbing for Tracked Vehicles*: monocular stair edges plus gyroscope for heading and boundary-ratio centering.
- *Descending-stair Detection, Approach, and Traversal with an Autonomous Tracked Vehicle*: line detection, optical flow, and gyro; line extraction alone lacks depth.
- Gutmann et al., *Stair climbing for humanoid robots using stereo vision*, DOI `10.1109/IROS.2004.1389593`.
- Lee et al., *Vision-based Ascending Staircase Detection*, DOI `10.1109/ICRA46639.2022.9812456`.
- Wang et al., *Deep learning-based ultra-fast stair detection*, DOI `10.1038/s41598-022-20667-w`.

## Caveats

- Detection accuracy is not centering or safe-control accuracy.
- RGB-only methods lack dependable metric depth.
- Blur, vibration, low texture, extreme lighting, shadows, occlusion, and stair-like distractors are material failure modes.
- Strongest closed-loop evidence found is from tracked vehicles or humanoids, not this quadruped.
- Fiducials are appropriate for controlled validation, not arbitrary stairs.
