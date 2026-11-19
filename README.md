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

```text
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
