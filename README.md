# Orthophoto-based Occupancy Grid Map Generator and Editor Interface in ROS 2

This repository provides a complete ROS 2 pipeline for generating 2D occupancy grid maps (`nav_msgs/msg/OccupancyGrid`) directly from high-resolution GeoTIFF orthophotos (e.g., aerial drone imagery). It also features an interactive PyQt5-based graphical interface for real-time map editing and manual post-processing.

Developed at the **Department of Automation and Robotics, Széchenyi István University**, Győr, Hungary.

## Table of Contents

* [Overview](#-overview)

* [Key Features](#-key-features)

* [System Architecture](#-system-architecture)

* [Prerequisites & Dependencies](#-prerequisites--dependencies)

* [Installation & Building](#-installation--building)

* [Quick Start & Usage](#-quick-start--usage)

  * [1. Standalone Orthophoto Gridmap Generator](#1-standalone-orthophoto-gridmap-generator)

  * [2. Map Editor GUI](#2-map-editor-gui)

  * [Full Launch Package](#3-full-launch-package)

  * [Foxglove Visualization](#5-foxglove-visualization)

* [Node Details & Parameters](#-node-details--parameters)

* [HSV Classification & Noise Filtering](#-hsv-classification--noise-filtering)

* [Citation & Reference](#-citation--reference)

## Overview

In outdoor mobile robotics, traditional LiDAR-based SLAM can be challenging due to lack of geometric features over large areas. High-resolution drone orthophotos offer detailed top-down coverage, but standard ROS navigation tools (`nav2_map_server`) only support basic grayscale intensity thresholding. This often fails when grass, dark asphalt, and shadows share similar grayscale values.

This package solves this issue by:

1. Converting GeoTIFF orthophotos to HSV color space for robust pixel-level semantic classification (asphalt roads vs. grass vs. white/yellow lane markings).

2. Rasterizing pixels into cell-based occupancy states ($0 = \text{Free}$, $100 = \text{Occupied}$, $-1 = \text{Unknown}$).

3. Applying a custom $3 \times 3$ neighborhood consensus filter tailored for discrete 3-state grid maps.

4. Providing a PyQt5 GUI for live editing and publishing edited maps over ROS 2 topics with `TRANSIENT_LOCAL` Quality of Service (QoS).

## Key Features

* **GeoTIFF Georeferencing**: Uses `rasterio` to automatically extract spatial scale ($\text{meters/pixel}$) from GeoTIFF metadata.

* **HSV Color Segmentation**: Robust against lighting variations and shadow degradation.

* **Custom Consensus Filtering**: Smoothes boundaries without blurring thin features or distorting obstacles.

* **Interactive PyQt5 GUI**: Pan/zoom interface to manually paint Free, Occupied, or Unknown cells using mouse actions.

* **QoS Compliant**: Map publisher uses `TRANSIENT_LOCAL` durability so late-joining navigation nodes (e.g., Nav2) automatically receive the map.

## System Architecture

```
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
                 |       map_editor Node          |
                 | - PyQt5 Interactive GUI        |
                 | - User manual corrections      |
                 +---------------+----------------+
                                 |
                                 | Topic: /edited_map
                                 v
                 +--------------------------------+
                 | Path Planner / Nav2 / Visualizer|
                 +--------------------------------+

```

## Prerequisites & Dependencies

### Operating System & ROS Environment

* **ROS 2**: Jazzy, Humble, or newer

* **OS**: Ubuntu 22.04 LTS / 24.04 LTS (or WSL2)

### Python Dependencies

Install required Python libraries:

```
pip install numpy opencv-python rasterio PyQt5

```

## Installation & Building

1. Create a ROS 2 workspace (if you haven't already):

   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   
   ```

2. Clone this repository (and optional path planner package):

   ```
   git clone https://github.com/robotlabor/Occupancy_Gridmap.git
   
   ```

3. Build the workspace:

   ```
   source /opt/ros/jazzy/setup.bash   # Replace 'jazzy' with your ROS 2 distro
   cd ~/ros2_ws
   colcon build --packages-select occupancy_gridmap --symlink-install
   source install/setup.bash
   
   ```

## Quick Start & Usage

> **Note:** Replace `/path/to/your/orthophoto.tif` with the absolute path to your GeoTIFF image file.

### 1. Standalone Orthophoto Gridmap Generator

Run the node directly by providing the path to your GeoTIFF file and the desired cell resolution (in meters):

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ortho_gridmap ortho_gridmap_node \
  --ros-args \
  -p tif_path:="/path/to/your/orthophoto.tif" \
  -p grid_resolution:=0.3

```

### 2. Map Editor GUI

Start the PyQt5 editor GUI to visualize and manually edit the incoming `/map` topic:

```
source ~/ros2_ws/install/setup.bash
ros2 run gridmap_editor map_editor

```

#### GUI Controls:

* **Left Click**: Mark cell as **Free** ($0$, White)

* **Right Click**: Mark cell as **Occupied** ($100$, Black)

* **Middle Click** or **Shift + Left Click**: Mark cell as **Unknown** ($-1$, Gray)

* **Mouse Wheel**: Zoom in / Zoom out

* **Click & Drag**: Pan across the map

* **Publish map Button**: Publishes the edited grid to the `/edited_map` topic.

### Full Launch Package

Launch both the generator and interface together using the provided launch configuration:

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch occupancy_gridmap occupancy_gridmap.launch.py \
  tif_path:="/path/to/your/orthophoto.tif" \
  grid_resolution:=0.3

```

### Foxglove Visualization

To stream topics to Foxglove Studio via `foxglove_bridge`:

```
source ~/ros2_ws/install/setup.bash
ros2 run foxglove_bridge foxglove_bridge

```

*Connect Foxglove Studio to `ws://localhost:8765`.*

## Node Details & Parameters

### `ortho_gridmap_node`

* **Parameters**:

  * `tif_path` *(string, required)*: Absolute file path to the input `.tif` / `.tiff` orthophoto.

  * `grid_resolution` *(double, default: `0.3`)*: Desired grid resolution in meters per cell.

* **Published Topics**:

  * `/map` (`nav_msgs/msg/OccupancyGrid`): The automatically generated grid map.

### `map_editor` / `MapEditorNode`

* **Subscribed Topics**:

  * `/map` (`nav_msgs/msg/OccupancyGrid`, QoS: Reliable/Volatile): Raw map feed from the generator.

* **Published Topics**:

  * `/edited_map` (`nav_msgs/msg/OccupancyGrid`, QoS: Reliable/Transient Local): Manually edited map.

## HSV Classification & Noise Filtering

### HSV Color Thresholds

The algorithm applies HSV color masks to classify terrain features into drivable and non-drivable categories:

| Feature Class | Lower Threshold $(H, S, V)$ | Upper Threshold $(H, S, V)$ | Target Cell State | 
 | ----- | ----- | ----- | ----- | 
| **Road Surface (Asphalt)** | $(0, 0, 50)$ | $(180, 50, 200)$ | Free ($0$) | 
| **White Road Markings** | $(0, 0, 200)$ | $(180, 40, 255)$ | Free ($0$) | 
| **Yellow Road Markings** | $(20, 50, 170)$ | $(35, 200, 255)$ | Free ($0$) | 
| **Other (Grass / Buildings)** | — | — | Occupied ($100$) / Unknown ($-1$) | 

### Neighborhood Consensus Filter

To eliminate isolated noise without blurring boundaries, a $3 \times 3$ sliding window evaluates only **Unknown** ($-1$) cells:

* If a target Unknown cell has $\ge 3$ **Free** neighbors and $\le 1$ **Occupied** neighbor, it is reclassified as **Free** ($0$).

## Citation & Reference

If you use this project in your research or academic work, please cite our paper:

```
@inproceedings{kovacs2026orthophoto,
  title     = {Orthophoto-based Occupancy Grid Map Generator and Editor Interface in a ROS2 Environment},
  author    = {Kov{\'a}cs, Levente and Ballagi, {\'A}ron},
  booktitle = {Proceedings of the Sustainable Mobility and Transportation Symposium 2026 (SMTS 2026)},
  year      = {2026},
  address   = {Gy{\H{o}}r, Hungary},
  publisher = {MDPI},
  doi       = {10.3390/engproc2026XXXXX}
}

```

## 📄 License

This project is open-source under the terms of the Creative Commons Attribution (CC BY 4.0) License.
