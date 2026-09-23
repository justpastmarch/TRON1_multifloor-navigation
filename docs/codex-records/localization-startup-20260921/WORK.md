Understood as: Automatically initialize localization anywhere on the selected 3F map, with an explicit manual position/yaw override; preserve existing mission/stair ownership and never assume home_3f is the physical start. No motion is authorized by a localization request.

Temporary artifacts: work/ (source snapshot and tests); temporary isolated ROS test master/processes will be registered before launch. Existing live sensor and observation processes are retained.

Completed: 28 files applied with hash checks; 217 testcase + 12 wrapper results and 6 UI/reconnect checks passed. Temporary source/build/log/cache/probe artifacts and owned test bag directories removed (cleanup.json). Existing sensor/observation processes retained. Physical acceptance unverified.
