# NanoBot Gazebo worlds

Shared Gazebo Classic 11 worlds for the NanoBot ROS 1 simulation packages.

## Run

Build and source the workspace, then select a world by basename:

```bash
cd ~/nanobot_ws
catkin_make
source devel/setup.bash
roslaunch gazebo_worlds world.launch world:=neighborhood
```

Available imported worlds are `neighborhood`, `outdoor`, `small_city`,
`test_city`, `yosemite`, and `inspection`. UAV-oriented worlds are
`uav_training`, `uav_complex_120m`, and `mountain_tea_garden_80m`; the latter
two have detailed README files and deterministic generators in `worlds/`.
The original `world_name` argument is still supported when an absolute path is
needed. Set `gui:=false` for a headless run.

The launch file adds this package's `models` directory to
`GAZEBO_MODEL_PATH`; no `.bashrc` change or online model download is required.

## Licensing

This package contains material under several licenses:

- NanoBot package files and the pre-existing worlds: Apache-2.0.
- The UAV-oriented worlds, generators, and project-authored model resources:
  Apache-2.0.
- The six imported world files, their preview images, and the models listed in
  the GPL section of `third_party/NOTICE.md`: GPL-3.0-only.
- Models imported from the Open Robotics Gazebo Model Database: CC-BY-3.0.

See `third_party/NOTICE.md` for exact provenance, file boundaries, attribution,
and source revisions. Full license texts are in `third_party/licenses`.
