# Bag coverage and results

READ means the listed topic/sample scope, not a new read of every topic. METADATA_ONLY bags lack raw LiDAR and RGB. Originals were opened read-only.

| ID | Original filename | Scope | LiDAR packets | Geometry samples | Evaluation windows | Guarded dual / V1 / V2 |
|---|---|---|---:|---:|---:|---|
| 00 | manual-5F-rooftop_route_1788917683773755895.bag | READ | 221 | 45 | 0 | no labelled evaluation window |
| 01 | manual-5F-rooftop_route_1788918199558502114.bag.active.invalid | READ | 1930 | 386 | 0 | no labelled evaluation window |
| 02 | manual-5F-rooftop_route_1788918551828137154.bag | READ | 348 | 70 | 0 | no labelled evaluation window |
| 03 | manual-5F-rooftop_route_1788920816079496577.bag.active.invalid | READ | 2690 | 538 | 3 | 38 / 54 / 53 of 200 |
| 04 | stair_3F_to_4F_FULL_20260916_210849.bag | READ | 2892 | 579 | 0 | no labelled evaluation window |
| 05 | stair_3F_to_4F_FULL_20260916_231643.bag | READ | 1341 | 269 | 0 | no labelled evaluation window |
| 06 | stair_3F_to_4F_FULL_20260916_234018.bag | READ | 3054 | 611 | 0 | no labelled evaluation window |
| 07 | stair_3F_to_4F_UP_20260821_155022.bag | READ | 1432 | 287 | 0 | development only |
| 08 | stair_3F_to_4F_UP_20260824_152128.bag | READ | 98 | 20 | 2 | 7 / 11 / 11 of 16 |
| 09 | stair_3F_to_4F_UP_20260824_152223.bag | READ | 141 | 29 | 2 | 13 / 13 / 13 of 18 |
| 10 | stair_3F_to_4F_UP_20260824_152248.bag | READ | 1793 | 359 | 2 | 50 / 65 / 50 of 104 |
| 11 | stair_3F_to_4F_UP_20260827_102101.bag | READ | 2249 | 450 | 2 | 88 / 102 / 88 of 130 |
| 12 | stair_3F_to_4F_UP_20260827_103200.bag | READ | 690 | 138 | 1 | 135 / 136 / 135 of 138 |
| 13 | stair_3F_to_4F_UP_20260827_103313.bag | READ | 2733 | 547 | 3 | 150 / 253 / 167 of 301 |
| 14 | stair_3F_to_4F_UP_20260827_105713.bag | READ | 1818 | 364 | 2 | 72 / 76 / 72 of 103 |
| 15 | stair_3F_to_4F_UP_20260827_150034.bag | READ | 1945 | 389 | 0 | no labelled evaluation window |
| 16 | stair_3F_to_4F_UP_20260827_154342.bag | READ | 2198 | 440 | 0 | no labelled evaluation window |
| 17 | stair_3F_to_4F_UP_CLEAN_REPEAT_20260821_160211.bag | READ | 1410 | 282 | 2 | 53 / 59 / 53 of 78 |
| 18 | tag_calibration_20260827_134853.bag | READ | 0 | 0 | 0 | no labelled evaluation window |
| 19 | yaw_calibration_020_030_040_20260821_134549.bag | METADATA_ONLY | 0 | 0 | 0 | no labelled evaluation window |
| 20 | yaw_calibration_20260821_132716.bag | METADATA_ONLY | 0 | 0 | 0 | no labelled evaluation window |

## Every evaluation window

Coarse RGB stair-view windows include approaches/pauses, not verified physical contact phases. V2 is exploratory after V1 inspection.

| Bag | Window / record-time range | Samples | Guarded dual | V1 | V2 |
|---|---|---:|---:|---:|---:|
| 08 | flight_view1 / 0.0–13.0s | 8 | 6 | 6 | 6 |
| 08 | flight_view2 / 29.0–47.0s | 8 | 1 | 5 | 5 |
| 09 | flight_view1 / 0.0–4.0s | 4 | 1 | 1 | 1 |
| 09 | flight_view2 / 8.0–12.2s | 14 | 12 | 12 | 12 |
| 10 | flight1 / 0.0–31.0s | 62 | 25 | 31 | 25 |
| 10 | flight2 / 51.0–72.0s | 42 | 25 | 34 | 25 |
| 11 | flight1 / 0.0–36.0s | 74 | 61 | 70 | 61 |
| 11 | flight2 / 90.0–118.0s | 56 | 27 | 32 | 27 |
| 12 | stair_view_with_long_pause / 0.0–68.8s | 138 | 135 | 136 | 135 |
| 13 | flight1 / 0.0–67.0s | 136 | 92 | 112 | 105 |
| 13 | flight2 / 106.0–129.0s | 46 | 20 | 28 | 20 |
| 13 | stair_view_with_long_pause / 213.0–272.3s | 119 | 38 | 113 | 42 |
| 14 | flight1 / 0.0–27.0s | 62 | 46 | 47 | 46 |
| 14 | flight2 / 50.0–71.0s | 41 | 26 | 29 | 26 |
| 17 | flight1 / 0.0–18.0s | 36 | 28 | 31 | 28 |
| 17 | flight2 / 40.0–61.0s | 42 | 25 | 28 | 25 |
| 03 | roof_up_flight1 / 72.0–129.0s | 69 | 36 | 36 | 36 |
| 03 | roof_up_flight2 / 168.0–184.0s | 3 | 0 | 0 | 0 |
| 03 | roof_down_candidate_backwards / 320.0–384.0s | 128 | 2 | 18 | 17 |

## Header-time correspondence diagnostic

Same geometry parameters. Map RGB labels through RGB headers and select LiDAR by header. This does not prove cross-sensor clock synchronization. Initial tables above use record-time correspondence.

| Bag | Window | Samples | Guarded dual | V1 | V2 |
|---|---|---:|---:|---:|---:|
| 08 | flight_view1 | 15 | 13 | 13 | 13 |
| 08 | flight_view2 | 0 | 0 | 0 | 0 |
| 09 | flight_view1 | 7 | 4 | 4 | 4 |
| 09 | flight_view2 | 9 | 7 | 7 | 7 |
| 10 | flight1 | 62 | 25 | 31 | 25 |
| 10 | flight2 | 42 | 25 | 34 | 25 |
| 11 | flight1 | 73 | 61 | 70 | 61 |
| 11 | flight2 | 56 | 30 | 35 | 30 |
| 12 | stair_view_with_long_pause | 131 | 128 | 129 | 128 |
| 13 | flight1 | 136 | 92 | 112 | 105 |
| 13 | flight2 | 45 | 20 | 28 | 20 |
| 13 | stair_view_with_long_pause | 110 | 33 | 104 | 37 |
| 14 | flight1 | 59 | 46 | 47 | 46 |
| 14 | flight2 | 41 | 27 | 30 | 27 |
| 17 | flight1 | 35 | 27 | 30 | 27 |
| 17 | flight2 | 42 | 25 | 28 | 25 |
| 03 | roof_up_flight1 | 60 | 46 | 47 | 46 |
| 03 | roof_up_flight2 | 7 | 5 | 5 | 5 |
| 03 | roof_down_candidate_backwards | 128 | 2 | 18 | 17 |

Bag 18: RGB payload and prior numeric arrays; no LiDAR. Bags 19/20: original index and prior numeric arrays only. Bags without labelled windows remain in full-cloud geometry and timing results; corridor/elevator planes are not evidence of stair traversal.

No source bag was replayed into ROS. No robot command was issued. Generated from comparison.json and robustness.json.
