# TYR GeoTIFF Occupancy Grid and Map Editor

`occupancy_gridmap` is a ROS 2 Jazzy Python package that converts a
georeferenced GeoTIFF orthophoto into a `nav_msgs/msg/OccupancyGrid` and
provides a PyQt5 editor for defining allowed areas, forbidden areas, cone
waypoints, and the robot start pose.

It is designed to work with the
[`path_planner_follower`](https://github.com/robotlabor/motion_planning)
package, but the GeoTIFF converter and editor can also be launched separately.

Package version: **0.4.2**

## Features

- GeoTIFF RGB orthophoto to ROS 2 OccupancyGrid conversion;
- map origin derived from the GeoTIFF geotransform;
- configurable grid resolution and map publication period;
- transient-local map publication for late subscribers;
- cell-by-cell free, occupied, and unknown editing;
- rectangular and arbitrary concave polygon editing;
- exact coordinate-based polygon input;
- individual cone placement and removal;
- uniform cone sampling on straight lines and circular arcs;
- exact coordinate-based cone input;
- graphical or coordinate-based robot start pose and yaw;
- optional static `map -> base_link` transform for localization-free tests;
- one-click publication of the edited map, robot pose, and cone list.

## Nodes

| Executable | Purpose |
|---|---|
| `ortho_gridmap_node` | Reads the GeoTIFF, classifies its pixels, and publishes `/map`. |
| `map_editor` | Captures the first map and opens the graphical editing interface. |

## GeoTIFF requirements and map coordinates

The input file must:

- exist at the supplied absolute path;
- contain at least three image bands;
- contain valid pixel-size and geotransform information;
- use a metric projected coordinate reference system if metric world
  coordinates are required.

The node does not reproject coordinates. It copies the GeoTIFF's lower-left
coordinates into the OccupancyGrid origin. For the ZalaZONE map, a suitable
source projection is UTM zone 33N (`EPSG:32633`). Use a north-up GeoTIFF
without rotation or skew, because the published OccupancyGrid orientation is
currently identity.

The requested `grid_resolution` must not be smaller than the source image's
pixel resolution.

## Occupancy classification

The converter groups source pixels into cells and classifies the RGB image in
HSV colour space. The current rules recognise grey, white, and yellow road-like
areas as free.

| Cell value | Meaning | Editor colour |
|---:|---|---|
| `0` | free / allowed | white |
| `100` | occupied / forbidden | black |
| `-1` | unknown | grey |

Small isolated unknown gaps surrounded mainly by free cells are cleaned after
classification. The image row order is then flipped to match the ROS
OccupancyGrid coordinate convention.

## Requirements

- Ubuntu 24.04;
- ROS 2 Jazzy;
- Python 3;
- a desktop session for the PyQt5 editor.

Install the main Python dependencies:

```bash
sudo apt update
sudo apt install \
  python3-numpy \
  python3-opencv \
  python3-pyqt5 \
  python3-rasterio
```

Install all dependencies declared by the workspace packages:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select occupancy_gridmap --symlink-install
source install/setup.bash
```

## Quick start

Start the converter and editor together:

```bash
ros2 launch occupancy_gridmap occupancy_gridmap.launch.py \
  tif_path:=/home/milan/zala.tif \
  grid_resolution:=0.3
```

The converter publishes `/map`. When the editor receives its first map, the
PyQt5 window opens and keeps a local editable copy.

## Launch files

### Converter and editor

```bash
ros2 launch occupancy_gridmap occupancy_gridmap.launch.py \
  tif_path:=/absolute/path/to/map.tif \
  grid_resolution:=0.3
```

Optional arguments:

| Argument | Default | Purpose |
|---|---|---|
| `tif_path` | required | Absolute input GeoTIFF path. |
| `grid_resolution` | `0.3` | OccupancyGrid resolution in metres per cell. |
| `waypoint_topic` | `/waypoints` | Cone `PoseArray` output. |
| `initial_pose_topic` | `/initialpose` | Robot start-pose output. |
| `base_frame` | `base_link` | Child frame of the optional test TF. |
| `publish_static_tf_default` | `true` | Initial state of the editor's static-TF checkbox. |

### Converter only

```bash
ros2 launch occupancy_gridmap ortho_gridmap.launch.py \
  tif_path:=/absolute/path/to/map.tif \
  grid_resolution:=0.3 \
  map_topic:=/map \
  map_frame:=map \
  publish_period:=1.0
```

Direct node execution is also supported:

```bash
ros2 run occupancy_gridmap ortho_gridmap_node --ros-args \
  -p tif_path:=/absolute/path/to/map.tif \
  -p grid_resolution:=0.3
```

### Editor only

```bash
ros2 launch occupancy_gridmap map_editor.launch.py \
  input_map_topic:=/map \
  output_map_topic:=/edited_map
```

The editor captures only the first received map. Restart it to load a different
map source.

## Map editing

### Edit individual cells

- left click: set free (`0`);
- right click: set occupied (`100`);
- middle click or Shift + left click: set unknown (`-1`);
- mouse wheel: zoom;
- Alt + left drag: pan.

### Edit a rectangle

Select rectangle mode and drag in any direction:

- left drag: set the complete rectangle free;
- right drag: set it occupied;
- middle drag or Shift + left drag: set it unknown.

The coloured preview is applied when the mouse button is released.

### Edit a polygon

Select polygon mode, then:

- start with a left, right, or middle click to select free, occupied, or
  unknown cells;
- add further vertices with left clicks;
- click the first vertex, double-click, press Enter, or select
  **Finish polygon** to apply it;
- press Backspace/Delete or right-click to remove the last vertex;
- press Escape or select **Cancel polygon** to discard it.

At least three distinct, non-collinear vertices are required. Concave polygons
are supported.

### Coordinate-defined polygon

Enter one world-coordinate vertex per line in the coordinate panel. Accepted
formats are:

```text
640150.0, 5193600.0
640155.0 5193610.0
640160.0; 5193600.0
```

Choose the target value and select **Preview coordinate polygon** or
**Apply coordinate polygon**. Coordinates are converted with the map origin,
orientation, and resolution. Every vertex must lie inside the map.

## Cone placement

### Individual cones

- left-click a free cell to add a cone;
- right-click near a cone to remove it;
- use **Clear cones** to remove every cone;
- use **Publish cones** to publish `/waypoints`.

Cones are displayed as indexed orange markers. Every pair must be at least
`0.50 m` apart, and every cone must be inside the map on a free cell.

The **Continuously publish cones at 1 Hz** option is enabled by default. The
message timestamp intentionally remains zero, allowing the waypoint sorter to
recognise an unchanged periodic cone list.

### Cones on a line

1. Set the requested maximum cone spacing.
2. Select the line tool.
3. Left-click the start and end points on free cells.

Both endpoints are included. Intermediate cones are distributed uniformly.
The fixed minimum `0.50 m` spacing takes priority over the requested maximum.

### Cones on a circular arc

1. Set the requested maximum cone spacing.
2. Select the arc tool.
3. Left-click the start point, a point on the arc, and the end point.

The three points must be non-collinear. The complete shape is accepted only if
every generated cone lies inside the map on free cells and satisfies the
minimum spacing. One line or arc is limited to 10,000 generated positions.

### Exact cone coordinates

Enter **Cone X** and **Y** in the coordinate panel and select **Add cone**. The
same map-boundary, free-cell, and minimum-spacing rules are applied.

## Robot start pose

### Graphical input

Select the robot tool, then left-click and drag from a free cell:

- the pressed point is the robot position;
- the drag direction defines its yaw.

Right-click or select **Clear robot start** to remove it.

### Exact coordinate input

Enter **Robot X**, **Y**, and **Yaw** in degrees, then select **Set robot**.

Select **Publish robot start** to publish
`geometry_msgs/msg/PoseWithCovarianceStamped` on `/initialpose`.

For localization-free tests, the optional checkbox can also publish a static
`map -> base_link` transform. Disable this option whenever a simulator or real
localization node already publishes the same transform. The integrated motion
planning launch disables it automatically when `simulation:=true`.

## Publishing order

The editor provides separate controls for:

- **Publish map** -> `/edited_map`;
- **Publish cones** -> `/waypoints`;
- **Publish robot start** -> `/initialpose` and optional test TF;
- **Publish all** -> map, robot start, then cone list.

Use **Publish all** for integrated tests. The order ensures that the map and
`map -> base_link` pose are available before the waypoint sorter processes the
cone positions.

Cones and the robot start can be placed only on free cells. If editing later
makes one of their cells occupied or unknown, publication is rejected until
the position or map is corrected.

## ROS interfaces

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | publisher -> editor | Generated map. |
| `/edited_map` | `nav_msgs/msg/OccupancyGrid` | editor output | Edited allowed/forbidden map. |
| `/waypoints` | `geometry_msgs/msg/PoseArray` | editor output | Cone positions. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | editor output | Robot start pose. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | optional editor output | Test-only `map -> base_link` transform. |

The `/map` and `/edited_map` topics use reliable, transient-local QoS so a
late-starting subscriber receives the most recent map.

## Integrated use with motion planning

With both packages in the same workspace, start the full pipeline through the
motion-planning package:

```bash
ros2 launch path_planner_follower integrated_navigation.launch.py \
  tif_path:=/home/milan/zala.tif \
  grid_resolution:=0.3 \
  controller:=pure_pursuit \
  simulation:=true
```

The edited map is consumed by `forbidden_point_avoider`, while the cone list is
sorted and smoothed before occupancy validation.

## Verification

Run the package tests:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon test --packages-select occupancy_gridmap
colcon test-result --verbose
```

Useful runtime checks:

```bash
ros2 node list
ros2 topic echo /map --once
ros2 topic echo /edited_map --once
ros2 topic echo /waypoints --once
ros2 topic echo /initialpose --once
```

If the converter reports `ModuleNotFoundError: No module named 'rasterio'`,
install `python3-rasterio` for the Python environment used by ROS 2 and rebuild
the package.

## License

Apache-2.0
