# stvl_map

`stvl_map` is a FineNav `Map` plugin for a rolling spatio-temporal voxel
window. It buffers `sensor_msgs/msg/PointCloud2` observations through
`MapRuntime`, transforms clouds from `base_link` with the timestamped
`RobotState.pose`, and removes occupied voxels after `decay_time_sec` without a
refresh.

The input contract is deliberately strict:

- A cloud in `map_frame` (default `map`) is inserted directly.
- A cloud in `input_frame` (default `base_link`) is transformed with the
  FineNav robot state.
- Other frame IDs are dropped. No sensor-to-base extrinsic is configured.

Build it alongside the FineNav workspace:

```bash
colcon build --base-paths /home/fins/Desktop/FineNav/FineNav-Engine \
  /home/fins/Desktop/stvl_map --packages-select stvl_map
```

Load the installed plugin before creating a map server:

```cpp
engine->loadAlgoPlugins({"stvl_map/lib"});
engine->createMapServer("stvl_map", "stvl_map_server");
```

The generated parameter namespace is `stvl_map`; see
`config/stvl_map.yaml`. Window size and subscription/frame settings are
read-only after startup. `decay_time_sec` can be changed at runtime.
