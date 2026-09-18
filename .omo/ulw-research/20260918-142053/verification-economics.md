# Verification Economics

| claim | risk | error cost | verification cost | chosen path | decision | outcome | residual risk |
|---|---|---|---|---|---|---|---|
| Current controller corrects stair-relative lateral error | high | unsafe edge approach | low | trace observations and physical command path | verify now | disproved | none for current source revision |
| Existing bags establish metric lateral drift | high | false controller confidence | medium | inventory independent references and synchronization | verify now | disproved | future surveyed capture required |
| IMU can serve as an absolute stair heading reference | high | biased steering correction | medium | primary literature plus local stationary analysis | research now | pending | no stationary local calibration yet |
| A new controller is superior | high | unsafe deployment | high | staged offline and physical experiment | defer | unverified | repeated ground-truthed trials required |
| Projected `/scan` preserves a unique stair boundary | critical | wrong-line steering near edge | medium | independently surveyed static pose-grid | next mandatory gate | unverified | blocks implementation |
| 3D validates 2D accuracy | high | shared-sensor false confidence | medium | require external surveyed truth | reject internal validation | not independent | common sensor/calibration errors |
| Zero command is physically safe mid-flight | critical | slip or tip after software fault | medium | restrained representative-pose stop test | later mandatory gate | unverified | software zero evidence is insufficient |
