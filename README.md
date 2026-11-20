# Orthophoto-based Occupancy Grid Map Generator and Editor Interface in ROS 2

This repository provides a complete ROS 2 pipeline for generating 2D occupancy grid maps (`nav_msgs/msg/OccupancyGrid`) directly from high-resolution GeoTIFF orthophotos (e.g., aerial drone imagery). It also features an interactive PyQt5-based graphical interface for real-time map editing, UTM coordinate inspection, local map archiving, and manual post-processing.

Developed at the **Department of Automation and Robotics, Széchenyi István University, Győr, Hungary**.

---

## Table of Contents
- [Version 2.0 (Latest Release)](#version-20-latest-release)
  - [What's New in v2.0](#whats-new-in-v20)
  - [Overview](#overview)
  - [Key Features](#key-features)
  - [System Architecture](#system-architecture)
  - [Prerequisites & Dependencies](#prerequisites--dependencies)
  - [Installation & Building](#installation--building)
  - [Quick Start & Usage](#quick-start--usage)
  - [Node Details & Parameters](#node-details--parameters)
  - [HSV Classification & Noise Filtering](#hsv-classification--noise-filtering)
- [Version 1.0 (Legacy Release)](#version-10-legacy-release)
- [Citation & Reference](#citation--reference)
- [License](#license)

---

# Version 2.0 (Latest Release)

## What's New in v2.0

* **Updated v2.0 Executables & Launch Files:** All v2.0 nodes and launch scripts within the same ROS 2 packages are suffixed with `2` (`ortho_gridmap_node2`, `map_editor2`, `occupancy_gridmap2.launch.py`) so both v1.0 and v2.0 can coexist seamlessly in the workspace.
* **Real-World UTM Georeferencing:** The published `OccupancyGrid` messages now preserve and embed accurate real-world UTM coordinates (`origin.position.x`, `origin.position.y`) extracted from the GeoTIFF affine transformation matrix, aligned to the ROS 2 bottom-left origin standard.
* **Real-Time UTM Coordinate Display:** Hovering the mouse cursor over any cell in the GUI displays its exact real-world UTM coordinates `(Easting, Northing)` in meters in a floating overlay.
* **Advanced GUI Editing Tools:**
  * **Brush Tool:** Continuous line interpolation (`np.linspace`) for smooth freehand drawing without gaps during fast cursor movements.
  * **Polygon Selection Tool:** Multi-vertex interactive polygon drawing with double-click area filling (`cv2.fillPoly`) for rapid editing of large regions.
  * **Undo Stack (`Ctrl+Z`):** 10-step LIFO undo history to revert accidental edits.
* **Standalone Map Archiving (`.npz`):** Save edited maps along with their UTM metadata (`resolution`, `frame_id`, `origin_x`, `origin_y`) into compressed `.npz` archives (`~/ros2_ws/data/maps`) and load them later independently without running the generator node.
* **Nav2-Compatible Topic Remapping & QoS:**
  * Raw generated map topic changed from `/map` to `/raw_map`.
  * Final edited map topic changed from `/edited_map` to `/map` for direct plug-and-play integration with the Nav2 `Static Layer`.
  * Both nodes now use `RELIABLE` + `TRANSIENT_LOCAL` QoS profiles to eliminate periodic publishing overhead while serving late-joining nodes.

---

## Overview

In outdoor mobile robotics, traditional LiDAR-based SLAM can be challenging due to the lack of geometric features over large open areas. High-resolution drone orthophotos offer detailed top-down coverage, but standard ROS navigation tools (`nav2_map_server`) only support basic grayscale intensity thresholding. This often fails when grass, dark asphalt, and shadows share similar grayscale intensities.

This package solves this issue by:
1. Converting GeoTIFF orthophotos to the **HSV color space** for robust pixel-level semantic classification (asphalt roads vs. grass vs. white/yellow lane markings).
2. Rasterizing pixels into cell-based occupancy states (`0` = Free, `100` = Occupied, `-1` = Unknown) with adaptive resolution checking and vertical axis alignment (`np.flipud`).
3. Applying a custom **3×3 neighborhood consensus filter** tailored for discrete 3-state grid maps.
4. Providing an **interactive PyQt5 GUI** (`map_editor2`) for live editing (Brush & Polygon modes), real-time UTM inspection, `.npz` saving/loading, and publishing Nav2-ready maps on `/map`.

---

## Key Features

* **GeoTIFF & UTM Georeferencing:** Uses `rasterio` to extract native pixel resolution (GSD) and embed true UTM origin coordinates into the ROS 2 `OccupancyGrid` metadata.
* **HSV Color Segmentation:** Robust against outdoor lighting variations and shadow degradation.
* **Custom Consensus Filtering:** Smoothes road noise without blurring thin features or distorting obstacle boundaries.
* **Feature-Rich PyQt5 GUI:** Smooth zooming, arrow-key panning, Brush & Polygon editing tools, real-time UTM cursor readout, `Ctrl+Z` undo, and `.npz` map persistence.
* **Nav2 & QoS Compliant:** Uses `RELIABLE` + `TRANSIENT_LOCAL` durability so late-joining navigation nodes (e.g., Nav2 `Static Layer`) automatically receive the latest `/map` without continuous network traffic.

---

## System Architecture

                 +--------------------------------+
                 |    GeoTIFF Orthophoto (.tif)   |
                 +---------------+----------------+
                                 |
                                 v
                 +--------------------------------+
                 |     ortho_gridmap_node2        |
                 | - GeoTIFF & UTM parsing        |
                 | - HSV Thresholding & Filtering |
                 +---------------+----------------+
                                 |
                                 | Topic: /raw_map
                                 | (QoS: Reliable / Transient Local)
                                 v
                 +--------------------------------+      +-----------------------+
                 |       map_editor2 Node         |<---->| Local Map Storage     |
                 | - PyQt5 Interactive GUI        |      | (.npz compressed)     |
                 | - Brush & Polygon editing      |      +-----------------------+
                 | - Real-time UTM hover display  |
                 +---------------+----------------+
                                 |
                                 | Topic: /map
                                 | (QoS: Reliable / Transient Local)
                                 v
                 +--------------------------------+
                 |   Nav2 Static Layer / RViz2 /  |
                 |      Foxglove Visualization    |
                 +--------------------------------+

---

## Prerequisites & Dependencies

### Operating System & ROS Environment
* **ROS 2:** Jazzy Jalisco (recommended), Humble Hawksbill, or newer
* **OS:** Ubuntu 24.04 LTS / 22.04 LTS (or WSL2)

### Python Dependencies
Install required Python libraries:

    pip install numpy opencv-python rasterio PyQt5

---

## Installation & Building

1. Create a ROS 2 workspace (if you haven't already):

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

2. Clone this repository:

    git clone https://github.com/robotlabor/Occupancy_Gridmap.git

3. Build the workspace:

    source /opt/ros/jazzy/setup.bash   # Replace 'jazzy' with your ROS 2 distro
    cd ~/ros2_ws
    colcon build --packages-select occupancy_gridmap --symlink-install
    source install/setup.bash

---

## Quick Start & Usage

> **Note:** Replace `/path/to/your/orthophoto.tif` with the absolute path to your GeoTIFF image file.

### 1. Standalone Orthophoto Gridmap Generator (`ortho_gridmap_node2`)
Run the v2.0 generator node directly by providing the path to your GeoTIFF file and the desired cell resolution (in meters):

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash

    ros2 run ortho_gridmap ortho_gridmap_node2 \
      --ros-args \
      -p tif_path:="/path/to/your/orthophoto.tif" \
      -p grid_resolution:=0.3

### 2. Map Editor GUI (`map_editor2`)
Start the PyQt5 editor GUI to visualize and edit the incoming `/raw_map` topic, or load a previously saved `.npz` map file directly from disk:

    source ~/ros2_ws/install/setup.bash
    ros2 run gridmap_editor map_editor2

#### GUI Controls:
* **Mode Selection (`Brush` / `Polygon`):** Switch between freehand brush drawing and multi-point polygon area filling via radio buttons.
* **Left Click (Brush):** Mark cells as **Free** (`0`, White).
* **Right Click (Brush):** Mark cells as **Occupied** (`100`, Black).
* **Middle Click or `Shift + Left Click` (Brush):** Mark cells as **Unknown** (`-1`, Gray).
* **Polygon Mode:** Single-click to place polygon vertices; **Double Click** to close and fill the selected area with the active state (Left/Right/Middle button).
* **`Ctrl + Z`:** Undo the last editing operation (up to 10 steps).
* **Mouse Wheel:** Zoom in / Zoom out.
* **Arrow Keys:** Pan across the zoomed map view.
* **Top-Left Overlay:** Displays live real-world **UTM coordinates `(X, Y)`** in meters under the cursor.
* **`Save map` / `Load map` Buttons:** Save or load compressed `.npz` map archives (including UTM origin and resolution metadata) in `~/ros2_ws/data/maps`.
* **`Publish map` Button:** Publishes the edited grid map to the `/map` topic.

### 3. Foxglove / RViz2 Visualization
To stream topics to Foxglove Studio via `foxglove_bridge`:

    source ~/ros2_ws/install/setup.bash
    ros2 run foxglove_bridge foxglove_bridge

Connect Foxglove Studio to `ws://localhost:8765` and subscribe to `/raw_map` or `/map`.
(You have to set the origin in Foxglove to align with the occupancy grid map.)

---

## Node Details & Parameters

### `ortho_gridmap_node2` (Package: `ortho_gridmap`)
* **Parameters:**
  * `tif_path` (`string`, required): Absolute file path to the input `.tif` / `.tiff` orthophoto.
  * `grid_resolution` (`double`, default: `0.3`): Desired grid resolution in meters per cell (automatically validated against native GSD).
* **Published Topics:**
  * `/raw_map` (`nav_msgs/msg/OccupancyGrid`, QoS: `Reliable` / `Transient Local`): The automatically generated, UTM-georeferenced raw grid map.

### `map_editor2` / `MapEditorNode` (Package: `gridmap_editor`)
* **Subscribed Topics:**
  * `/raw_map` (`nav_msgs/msg/OccupancyGrid`, QoS: `Reliable` / `Transient Local`): Initial raw map feed from the generator.
* **Published Topics:**
  * `/map` (`nav_msgs/msg/OccupancyGrid`, QoS: `Reliable` / `Transient Local`): Manually edited, Nav2-ready occupancy grid map.

---

## HSV Classification & Noise Filtering

### HSV Color Thresholds
The algorithm applies HSV color masks to classify terrain features into drivable and non-drivable categories:

| Feature Class | Lower Threshold (H, S, V) | Upper Threshold (H, S, V) | Target Cell State |
| :--- | :---: | :---: | :---: |
| **Road Surface (Asphalt)** | `(0, 0, 50)` | `(180, 50, 200)` | Free (`0`) |
| **White Road Markings** | `(0, 0, 200)` | `(180, 40, 255)` | Free (`0`) |
| **Yellow Road Markings** | `(20, 50, 170)` | `(35, 200, 255)` | Free (`0`) |
| **Other (Grass / Buildings)** | — | — | Occupied (`100`) / Unknown (`-1`) |

### Neighborhood Consensus Filter
To eliminate isolated noise without blurring boundaries, a 3×3 sliding window evaluates only `Unknown` (`-1`) cells:
* If a target `Unknown` cell has **≥ 3 `Free` neighbors** and **≤ 1 `Occupied` neighbor**, it is reclassified as `Free` (`0`).

---

# Version 1.0 (Legacy Release)

<details>
<summary><b>Click to expand Version 1.0 Documentation (Initial Release)</b></summary>

### Overview & Differences from v2.0
Version 1.0 represents the initial prototype presented in the SMTS 2026 conference paper. In this version:
* Used the original executable and launch file names (`ortho_gridmap_node`, `map_editor`, `occupancy_gridmap.launch.py`) inside the same `ortho_gridmap`, `gridmap_editor`, and `occupancy_gridmap` packages.
* The generator published directly to `/map`, and the editor published to `/edited_map`.
* Map origin was initialized at local `(0.0, 0.0)` without embedded UTM coordinates or live cursor coordinate readouts.
* The GUI supported basic single-cell/drag painting without polygon selection, undo history, or `.npz` file persistence.

### v1.0 System Architecture

                 +--------------------------------+
                 |    GeoTIFF Orthophoto (.tif)   |
                 +---------------+----------------+
                                 |
                                 v
                 +--------------------------------+
                 |     ortho_gridmap_node         |
                 | - GeoTIFF parsing (rasterio)   |
                 | - HSV Thresholding & Filtering |
                 +---------------+----------------+
                                 |
                                 | Topic: /map
                                 v
                 +--------------------------------+
                 |        map_editor Node         |
                 | - PyQt5 Interactive GUI        |
                 | - User manual corrections      |
                 +---------------+----------------+
                                 |
                                 | Topic: /edited_map
                                 v
                 +--------------------------------+
                 | Path Planner / Nav2 / Visualizer|
                 +--------------------------------+

### v1.0 Node Topics & GUI Controls
* **`ortho_gridmap_node`:** Published to `/map` (`nav_msgs/msg/OccupancyGrid`).
* **`map_editor`:** Subscribed to `/map` (`Reliable`/`Volatile`) and published edited maps to `/edited_map` (`Reliable`/`Transient Local`).
* **GUI Controls (v1.0):**
  * **Left Click:** Mark cell as Free (`0`, White)
  * **Right Click:** Mark cell as Occupied (`100`, Black)
  * **Middle Click or `Shift + Left Click`:** Mark cell as Unknown (`-1`, Gray)
  * **Mouse Wheel:** Zoom in / Zoom out
  * **Click & Drag:** Pan across the map
  * **`Publish map` Button:** Publishes the edited grid to the `/edited_map` topic.

</details>

---

## Citation & Reference

If you use this project in your research or academic work, please cite our paper:

    @inproceedings{kovacs2026orthophoto,
      title     = {Orthophoto-based Occupancy Grid Map Generator and Editor Interface in a ROS2 Environment},
      author    = {Kov{\'a}cs, Levente and Ballagi, {\'A}ron},
      booktitle = {Proceedings of the Sustainable Mobility and Transportation Symposium 2026 (SMTS 2026)},
      year      = {2026},
      address   = {Gy{\H{o}}r, Hungary},
      publisher = {MDPI},
      doi       = {10.3390/engproc2026XXXXX}
    }

## License
This project is open-source under the terms of the **Creative Commons Attribution (CC BY 4.0)** License.
