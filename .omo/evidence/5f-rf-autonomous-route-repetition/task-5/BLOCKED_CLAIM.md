# Task 5: blocked pending independent site survey

## Verdict

`blocked`, not `fully_done`.

Production activation was intentionally not performed. The repository does contain
the 5F and RF map files, recorded mission locations, and tag configuration records
for tags 501 and 600. Those records are still not independently reviewed
commissioning evidence: the Task 2 package contains zero reviewed, spatially
separated map correspondences for either floor, and no transform residuals.
The plan forbids promoting these records, or inventing covariances, tag detections,
stair distances, speed limits, or pass results without that review.

## Recorded inputs found

- `src/multifloor_manager/config/maps/floor_5F.{yaml,pgm}` and
  `floor_RF.{yaml,pgm}` are present. Their fingerprints are recorded in
  `../task-2/map_alignment_block.md`.
- `src/mission_manager/config/locations.yaml` contains `stair_5f_to_rf`,
  `stair_rf_landing`, and the four RF roof-loop locations. The file remains
  `configured: false`.
- `src/multifloor_manager/config/apriltags.yaml` and
  `apriltag_ros_tags.yaml` contain tag 501 on 5F and tag 600 on RF, both
  `tagStandard41h12` and `0.18 m`.
- `docs/stair-up-commissioning.md` records the 5F→RF geometry as 10 steps,
  landing, 9 steps, then 2 exit steps. This is a documented observation, not a
  5F/RF capture-backed measurement.
- `stair_captures/` contains 3F→4F captures, and `manual_captures/` contains
  5F rooftop route captures. The newest attempt is an `.active.invalid` BAG:
  its RGB frames show stair flights, landings, and the RF rooftop. Directly
  enlarged image crops identify printed `600` and `501` markers. It lacks
  `/tag_detections`, mission/floor-state topics, and a finalized artifact, so it
  The operator reports that the physical 5F→RF→5F route completed normally and
  only final bag saving failed. The file therefore supports route-image analysis,
  but lacks mission/floor-state topics and a finalized artifact, so it is not an
  independent finalized acceptance record even though the tag IDs are visually
  supported by the images.
- The adjacent `TRON1_Modular_Navigation` workspace contains replay evidence and
  profile documentation for the same 3F→4F corpus only. Its planning notes also
  state that surveyed pose truth is absent; no 5F/RF anchor, residual, or frozen
  production profile was found there.

## Verification

The production validator rejects use at the first disabled production file:

```text
$ python3 validate_bundle.py --validate-production-config
BUNDLE_ERROR: locations.yaml:configured: must be true before use
```

The local contract and known-good fixture remain healthy:

```text
$ python3 validate_bundle.py --site-config-root test/fixtures/building_valid
SITE_CONFIG_OK floors=3 locations=7 edges=9
BUNDLE_OK

$ ./run.sh --check
SITE_CONFIG_OK floors=3 locations=7 edges=9
BUNDLE_OK
[CHECK] bundle, fixture, packages, and local software: OK
```

## Missing independent inputs

The following must be supplied by stationary on-site survey and review before
Task 5 can be completed:

1. At least two spatially separated reviewed map correspondences on 5F and two
   on RF, with transform residuals and map fingerprints.
2. Independent review of the recorded `stair_5f_to_rf`, `stair_rf_landing`, and
   roof-loop poses against those map correspondences.
3. Independent detector evidence for tag 501 on the 5F landing and tag 600 on
   the RF landing, including family and size.
4. Separate measured ascent and descent stair profiles: signed distances and
   yaws, pitch enter/exit cycle, dwell, tolerances, freshness, sample-gap and
   jump bounds, speeds, and timeout.
5. Reviewed 5F/RF transition readiness evidence: target tag, map generation,
   AMCL covariance/freshness, TF, scan, stationary, and costmap gates.

## Separate physical safety gate

The operator reports that the test area, safety operator, neutral remote, and
reachable physical E-stop are prepared. This preparation is not itself a motion
result; it must be rechecked and recorded immediately before any hardware gate,
even after the site-survey inputs are accepted.

No production YAML was modified. No robot, SSH, WebSocket, or motion process was
started. Production route acceptance remains blocked by the missing independent
commissioning evidence. Task 6 has only partial fixture no-motion evidence;
Tasks 7-10 still require the separate physical safety gate and real hardware
results.
