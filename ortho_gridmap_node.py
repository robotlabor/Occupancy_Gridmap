import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import rasterio
import os


class OrthoGridMapNode(Node):

    def __init__(self):
        super().__init__('ortho_gridmap_node')

        # Default parameters
        self.declare_parameter('tif_path', '')
        self.declare_parameter('grid_resolution', 0.3)  

        # Parameters
        self.tif_path = self.get_parameter(
            'tif_path'
        ).get_parameter_value().string_value

        self.grid_resolution = self.get_parameter(
            'grid_resolution'
        ).get_parameter_value().double_value

        # Warnings / Checks
        if not self.tif_path:
            self.get_logger().error('Parameter "tif_path" is required!')
            raise RuntimeError('Missing tif_path')

        if not os.path.exists(self.tif_path):
            self.get_logger().error(f'TIF file not found: {self.tif_path}')
            raise RuntimeError('Invalid tif_path')

        if self.grid_resolution <= 0.0:
            raise RuntimeError('grid_resolution must be > 0')

        self.get_logger().info(f"Requested grid_resolution: {self.grid_resolution:.4f} m")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/raw_map',
            qos_profile
        )

        self.map_msg = self.build_occupancy_grid()


        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)

        self.get_logger().info("Static occupancy grid published successfully to /raw_map.")

    # ======================================================
    # ORTHOPHOTO → OCCUPANCY GRID
    # ======================================================
    def build_occupancy_grid(self) -> OccupancyGrid:
        self.get_logger().info('Loading orthophoto...')

        with rasterio.open(self.tif_path) as src:
            img = src.read([1, 2, 3])   # RGB 
            img = np.transpose(img, (1, 2, 0))  # CHW -> HWC

            transform = src.transform  

            meter_per_pixel_x = transform.a
            meter_per_pixel_y = abs(transform.e)

            if not np.isclose(meter_per_pixel_x, meter_per_pixel_y, rtol=1e-3):
                self.get_logger().warn(
                    f"Non-square pixels detected: "
                    f"x={meter_per_pixel_x:.4f}, y={meter_per_pixel_y:.4f}"
                )

            meter_per_pixel = float((meter_per_pixel_x + meter_per_pixel_y) / 2.0)

            self.get_logger().info(
                f"meter_per_pixel (from TIFF): {meter_per_pixel:.4f} m/px"
            )

        height_px, width_px = img.shape[:2]

        exact_pixels_per_cell = self.grid_resolution / meter_per_pixel
        pixels_per_cell = int(round(exact_pixels_per_cell))

        if pixels_per_cell < 1:
            raise RuntimeError(
                f"The specified grid_resolution ({self.grid_resolution:.4f} m) smaller than "
                f"the native pixel size of GeoTIFF ({meter_per_pixel:.4f} m/px)!"
            )

        adjusted_grid_resolution = pixels_per_cell * meter_per_pixel

     
        if not np.isclose(self.grid_resolution, adjusted_grid_resolution, rtol=1e-4):
            self.get_logger().warn(
                f"The specified cell size ({self.grid_resolution:.4f} m) is not an integer multiple of "
                f"the GeoTIFF pixel size ({meter_per_pixel:.4f} m/px). "
                f"The closest valid cell size will be used instead of the specified value: "
                f"{adjusted_grid_resolution:.4f} m ({pixels_per_cell} px/cell)."
            )
          
            self.grid_resolution = adjusted_grid_resolution

        self.get_logger().info(f"Pixels per grid cell: {pixels_per_cell} x {pixels_per_cell}")
        self.get_logger().info(f'Image shape: {img.shape}')

        # OpenCV
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)  

        gray_mask = cv2.inRange(
            hsv,
            (0, 0, 50),
            (180, 50, 200)
        )

        white_mask = cv2.inRange(
            hsv,
            (0, 0, 200),
            (180, 40, 255)
        )

        yellow_mask = cv2.inRange(
            hsv,
            (20, 50, 170),
            (35, 200, 255)
        )

        height_px, width_px = gray_mask.shape
        grid_h = height_px // pixels_per_cell
        grid_w = width_px // pixels_per_cell

        self.get_logger().info(f'Grid size: {grid_w} x {grid_h}')

        grid = np.full((grid_h, grid_w), -1, dtype=np.int8)

   
        for i in range(grid_h):
            for j in range(grid_w):
                y0 = i * pixels_per_cell
                y1 = y0 + pixels_per_cell
                x0 = j * pixels_per_cell
                x1 = x0 + pixels_per_cell
                gray_ratio = np.mean(gray_mask[y0:y1, x0:x1] > 0)
                white_ratio = np.mean(white_mask[y0:y1, x0:x1] > 0)
                yellow_ratio = np.mean(yellow_mask[y0:y1, x0:x1] > 0)
                if gray_ratio > 0.5 or white_ratio > 0.2 or yellow_ratio > 0.1:
                    grid[i, j] = 0      # FREE
                elif gray_ratio == 0:
                    grid[i, j] = 100    # OCCUPIED
                else:
                    grid[i, j] = -1     # UNKNOWN
        # (Image Y-down -> ROS Y-up)
        grid = np.flipud(grid)

        # Neighborhood Consensus Filter
        cleaned_grid = grid.copy()
        h, w = grid.shape
        for i in range(1, h - 1):
            for j in range(1, w - 1):
                if grid[i, j] == -1:
                    neighborhood = grid[i-1:i+2, j-1:j+2]
                    free_count = np.sum(neighborhood == 0)
                    occupied_count = np.sum(neighborhood == 100)
                    if free_count >= 3 and occupied_count <= 1:
                        cleaned_grid[i, j] = 0
        grid = cleaned_grid

        # ROS OCCUPANCYGRID MSG
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.grid_resolution
        msg.info.width = grid_w
        msg.info.height = grid_h

        origin_x = transform.c
        origin_y = transform.f - (height_px * meter_per_pixel_y)
        # msg.info.origin.position.x = 0.0    #Foxglove
        # msg.info.origin.position.y = 0.0    #Foxglove
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)

        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten(order='C').tolist()

        self.get_logger().info('Occupancy grid built successfully.')

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = OrthoGridMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()