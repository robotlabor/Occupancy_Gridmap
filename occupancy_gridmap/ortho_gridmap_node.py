#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rasterio
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header


class OrthoGridMapNode(Node):
    """Convert a GeoTIFF orthophoto into a ROS 2 OccupancyGrid."""

    def __init__(self):
        super().__init__('ortho_gridmap_node')

        self.declare_parameter('tif_path', '')
        self.declare_parameter('grid_resolution', 0.3)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_period', 1.0)

        self.tif_path = str(self.get_parameter('tif_path').value)
        self.grid_resolution = float(
            self.get_parameter('grid_resolution').value
        )
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.publish_period = float(
            self.get_parameter('publish_period').value
        )

        self._validate_parameters()

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.map_topic,
            map_qos,
        )

        self.map_msg = self.build_occupancy_grid()
        self.timer = self.create_timer(self.publish_period, self.publish_map)

        # Publish immediately instead of waiting for the first timer callback.
        self.publish_map()

    def _validate_parameters(self):
        if not self.tif_path:
            raise RuntimeError('Parameter "tif_path" is required.')

        if not os.path.isfile(self.tif_path):
            raise RuntimeError(f'TIF file not found: {self.tif_path}')

        if self.grid_resolution <= 0.0:
            raise RuntimeError('grid_resolution must be greater than zero.')

        if self.publish_period <= 0.0:
            raise RuntimeError('publish_period must be greater than zero.')

        self.get_logger().info(
            f'GeoTIFF: {self.tif_path}; grid resolution: '
            f'{self.grid_resolution:.3f} m/cell'
        )

    def build_occupancy_grid(self) -> OccupancyGrid:
        self.get_logger().info('Loading orthophoto...')

        with rasterio.open(self.tif_path) as source:
            if source.count < 3:
                raise RuntimeError('The GeoTIFF must contain at least three bands.')

            image = source.read([1, 2, 3])
            image = np.transpose(image, (1, 2, 0))
            transform = source.transform

        image = self._to_uint8(image)

        meter_per_pixel_x = abs(float(transform.a))
        meter_per_pixel_y = abs(float(transform.e))

        if meter_per_pixel_x <= 0.0 or meter_per_pixel_y <= 0.0:
            raise RuntimeError('Invalid GeoTIFF pixel resolution.')

        if not np.isclose(
            meter_per_pixel_x,
            meter_per_pixel_y,
            rtol=1e-3,
        ):
            self.get_logger().warning(
                'Non-square pixels detected: '
                f'x={meter_per_pixel_x:.4f}, y={meter_per_pixel_y:.4f}'
            )

        meter_per_pixel = (meter_per_pixel_x + meter_per_pixel_y) / 2.0
        pixels_per_cell = int(round(self.grid_resolution / meter_per_pixel))

        if pixels_per_cell < 1:
            raise RuntimeError(
                'grid_resolution is smaller than the GeoTIFF pixel resolution.'
            )

        self.get_logger().info(
            f'GeoTIFF resolution: {meter_per_pixel:.4f} m/px; '
            f'{pixels_per_cell} px/cell'
        )

        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        gray_mask = cv2.inRange(hsv, (0, 0, 50), (180, 50, 200))
        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))
        yellow_mask = cv2.inRange(hsv, (20, 50, 170), (35, 200, 255))

        height_px, width_px = gray_mask.shape
        grid_height = height_px // pixels_per_cell
        grid_width = width_px // pixels_per_cell

        if grid_height < 1 or grid_width < 1:
            raise RuntimeError('The selected grid resolution produces an empty map.')

        grid = np.full((grid_height, grid_width), -1, dtype=np.int8)

        for row in range(grid_height):
            for column in range(grid_width):
                y0 = row * pixels_per_cell
                y1 = y0 + pixels_per_cell
                x0 = column * pixels_per_cell
                x1 = x0 + pixels_per_cell

                gray_ratio = np.mean(gray_mask[y0:y1, x0:x1] > 0)
                white_ratio = np.mean(white_mask[y0:y1, x0:x1] > 0)
                yellow_ratio = np.mean(yellow_mask[y0:y1, x0:x1] > 0)

                if (
                    gray_ratio > 0.5
                    or white_ratio > 0.2
                    or yellow_ratio > 0.1
                ):
                    grid[row, column] = 0
                elif gray_ratio == 0.0:
                    grid[row, column] = 100
                else:
                    grid[row, column] = -1

        # Convert image row order to the OccupancyGrid coordinate convention.
        grid = np.flipud(grid)
        grid = self._clean_unknown_cells(grid)

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.grid_resolution
        msg.info.width = grid_width
        msg.info.height = grid_height

        # GeoTIFF lower-left corner in the source coordinate reference system.
        msg.info.origin.position.x = float(transform.c)
        msg.info.origin.position.y = float(
            transform.f + transform.e * height_px
        )
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten(order='C').astype(int).tolist()

        self.get_logger().info(
            f'Occupancy grid built: {grid_width} x {grid_height} cells; '
            f'origin=({msg.info.origin.position.x:.3f}, '
            f'{msg.info.origin.position.y:.3f})'
        )

        return msg

    @staticmethod
    def _to_uint8(image: np.ndarray) -> np.ndarray:
        if image.dtype == np.uint8:
            return image

        finite_image = np.nan_to_num(image, nan=0.0, posinf=0.0, neginf=0.0)
        minimum = float(np.min(finite_image))
        maximum = float(np.max(finite_image))

        if maximum <= minimum:
            return np.zeros_like(finite_image, dtype=np.uint8)

        normalized = (finite_image - minimum) / (maximum - minimum)
        return np.clip(normalized * 255.0, 0.0, 255.0).astype(np.uint8)

    @staticmethod
    def _clean_unknown_cells(grid: np.ndarray) -> np.ndarray:
        cleaned_grid = grid.copy()
        height, width = grid.shape

        for row in range(1, height - 1):
            for column in range(1, width - 1):
                if grid[row, column] != -1:
                    continue

                neighborhood = grid[row - 1:row + 2, column - 1:column + 2]
                free_count = int(np.sum(neighborhood == 0))
                occupied_count = int(np.sum(neighborhood == 100))

                if free_count >= 3 and occupied_count <= 1:
                    cleaned_grid[row, column] = 0

        return cleaned_grid

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrthoGridMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
