import sys
import threading
import os
import numpy as np
import cv2
import rclpy

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QGraphicsView, QGraphicsScene,
    QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QRadioButton,
    QGraphicsPolygonItem, QFileDialog, QShortcut, QLabel
)
from PyQt5.QtGui import QImage, QPixmap, QPen, QBrush, QColor, QPolygonF, QKeySequence
from PyQt5.QtCore import Qt, pyqtSignal, QPointF
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


COLOR_MAP = {
    0: [255, 255, 255],    
    100: [0, 0, 0],        
    -1: [150, 150, 150]    
}


class MapViewer(QGraphicsView):

    update_signal = pyqtSignal(object)
    hover_coords_signal = pyqtSignal(int, int)

    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.setMouseTracking(True)

        self.pixmap_item = None
        self.grid = None
        self.rgb_img = None  

        self.setDragMode(QGraphicsView.NoDrag)

        self.edit_mode = "brush"  # "brush" or "poly"
        self.is_drawing = False
        self.current_value = 0
        self.start_pos = None
        self.last_pos = None    

        self.poly_points = []
        self.poly_item = None

        # Undo
        self.undo_stack = []
        self.max_undo_depth = 10

        self.update_signal.connect(self.set_grid)

    def _save_undo_state(self):
        if self.grid is not None:
            self.undo_stack.append(self.grid.copy())
            if len(self.undo_stack) > self.max_undo_depth:
                self.undo_stack.pop(0)

    def undo(self):
        if not self.undo_stack:
            return
        self._clear_polygon()
        self.grid = self.undo_stack.pop()
        self.update_full_image()

    def set_edit_mode(self, mode):
        self.edit_mode = mode
        self._clear_polygon()

    def _clear_polygon(self):
        if self.poly_item:
            self.scene.removeItem(self.poly_item)
            self.poly_item = None
        self.poly_points.clear()

    def set_grid(self, grid):
        self.grid = grid
        self.undo_stack.clear()
        self._clear_polygon()
        self.update_full_image()

    def update_full_image(self):
        if self.grid is None:
            return

        h, w = self.grid.shape
        self.rgb_img = np.zeros((h, w, 3), dtype=np.uint8)

        self.rgb_img[self.grid == 0] = COLOR_MAP[0]
        self.rgb_img[self.grid == 100] = COLOR_MAP[100]
        self.rgb_img[self.grid == -1] = COLOR_MAP[-1]

        self.update_pixmap()

    def update_pixmap(self):
        if self.rgb_img is None:
            return

        h, w, _ = self.rgb_img.shape
        qimg = QImage(
            self.rgb_img.data,
            w,
            h,
            3 * w,
            QImage.Format_RGB888
        )

        pix = QPixmap.fromImage(qimg)

        if self.pixmap_item is None:
            self.pixmap_item = self.scene.addPixmap(pix)
        else:
            self.pixmap_item.setPixmap(pix)

    def _draw_brush_line(self, x0, y0, x1, y1):
        n_pts = max(abs(x1 - x0), abs(y1 - y0)) + 1
        xs = np.round(np.linspace(x0, x1, n_pts)).astype(int)
        ys = np.round(np.linspace(y0, y1, n_pts)).astype(int)

        self.grid[ys, xs] = self.current_value
        color = COLOR_MAP.get(self.current_value, COLOR_MAP[-1])
        self.rgb_img[ys, xs] = color

    def wheelEvent(self, event):
        zoom = 1.25
        if event.angleDelta().y() > 0:
            self.scale(zoom, zoom)
        else:
            self.scale(1 / zoom, 1 / zoom)

    def _get_value_from_event(self, event):
        buttons = event.buttons() if event.buttons() else event.button()

        if buttons & Qt.LeftButton:
            if event.modifiers() & Qt.ShiftModifier:
                return -1
            return 0
        elif buttons & Qt.RightButton:
            return 100
        elif buttons & Qt.MiddleButton:
            return -1
        return None

    def mousePressEvent(self, event):
        if self.grid is None:
            return

        val = self._get_value_from_event(event)
        if val is None:
            return

        pos = self.mapToScene(event.pos())
        x = int(pos.x())
        y = int(pos.y())

        h, w = self.grid.shape
        if not (0 <= x < w and 0 <= y < h):
            return

        self.current_value = val

        if self.edit_mode == "brush":
            self._save_undo_state()
            self.is_drawing = True
            self.start_pos = (x, y)
            self.last_pos = (x, y)
            self._draw_brush_line(x, y, x, y)
            self.update_pixmap()
        elif self.edit_mode == "poly":
            if not self.poly_points:
                self._save_undo_state()
            self.poly_points.append((x, y))
            if self.poly_item is None:
                self.poly_item = QGraphicsPolygonItem()
                self.poly_item.setPen(QPen(QColor(255, 0, 0), 1, Qt.DashLine))
                self.poly_item.setBrush(QBrush(QColor(255, 0, 0, 60)))
                self.scene.addItem(self.poly_item)
            self._update_poly_visual(current_mouse_pos=(x, y))

    def mouseMoveEvent(self, event):
        if self.grid is None:
            super().mouseMoveEvent(event)
            return

        pos = self.mapToScene(event.pos())
        x = int(pos.x())
        y = int(pos.y())
        h, w = self.grid.shape
        if 0 <= x < w and 0 <= y < h:
            self.hover_coords_signal.emit(x, y)
        else:
            self.hover_coords_signal.emit(-1, -1)
        x_clamped = max(0, min(x, w - 1))
        y_clamped = max(0, min(y, h - 1))

        if self.edit_mode == "brush":
            if not self.is_drawing:
                super().mouseMoveEvent(event)
                return
            if self.last_pos is not None:
                x0, y0 = self.last_pos
                x1, y1 = x_clamped, y_clamped
                self._draw_brush_line(x0, y0, x1, y1)
                self.last_pos = (x1, y1)
                self.update_pixmap()
        elif self.edit_mode == "poly" and self.poly_points:
            self._update_poly_visual(current_mouse_pos=(x_clamped, y_clamped))

    def leaveEvent(self, event):
        self.hover_coords_signal.emit(-1, -1)
        super().leaveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.grid is None:
            super().mouseReleaseEvent(event)
            return

        if self.edit_mode == "brush" and self.is_drawing:
            self.is_drawing = False
            self.start_pos = None
            self.last_pos = None

    def mouseDoubleClickEvent(self, event):
        if self.edit_mode == "poly":
            val = self._get_value_from_event(event)
            if val is not None:
                self.current_value = val

            pos = self.mapToScene(event.pos())
            x = int(pos.x())
            y = int(pos.y())
            h, w = self.grid.shape

            if 0 <= x < w and 0 <= y < h:
                if not self.poly_points or self.poly_points[-1] != (x, y):
                    self.poly_points.append((x, y))

            if len(self.poly_points) >= 3:
                self._apply_polygon_fill()
        else:
            super().mouseDoubleClickEvent(event)

    def _update_poly_visual(self, current_mouse_pos=None):
        if not self.poly_points or self.poly_item is None:
            return
        pts = [QPointF(px, py) for px, py in self.poly_points]
        if current_mouse_pos is not None:
            pts.append(QPointF(current_mouse_pos[0], current_mouse_pos[1]))
        self.poly_item.setPolygon(QPolygonF(pts))

    def _apply_polygon_fill(self):
        if len(self.poly_points) < 3 or self.grid is None:
            return
        h, w = self.grid.shape
        pts = np.array(self.poly_points, dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 1)
        self.grid[mask == 1] = self.current_value
        self._clear_polygon()
        self.update_full_image()


class MapEditorNode(Node):

    def __init__(self, gui):
        super().__init__('map_editor')

        self.gui = gui
        self.map_received = False

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/raw_map',
            self.receive_map,
            qos_sub
        )

        self.map_msg = None

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_pub
        )

    def receive_map(self, msg):
        if self.map_received:
            return

        self.get_logger().info("Map captured from /raw_map topic")
        self.map_msg = msg

        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height,
            msg.info.width
        )

        grid = np.flipud(grid)

        self.gui.update_signal.emit(grid)
        self.map_received = True

    def publish_edited_map(self, grid):
        if self.map_msg is None:
            self.get_logger().warn("No map metadata available to publish")
            return

        msg = OccupancyGrid()

        now = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_msg.header.frame_id
        #msg.header.frame_id = 'utm'
        msg.header.stamp = now
        msg.info = self.map_msg.info
        msg.info.map_load_time = now

        ros_grid = np.flipud(grid).astype(np.int8)
        msg.data = ros_grid.flatten().astype(int).tolist()

        self.publisher.publish(msg)
        self.get_logger().info("Edited map published to /map")


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS 2 OccupancyGrid Editor")

        self.viewer = MapViewer()

       
        self.coord_label = QLabel("UTM: -", self.viewer)
        self.coord_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.coord_label.setStyleSheet(
            "background-color: rgba(25, 25, 25, 195);"
            "color: #ffffff;"
            "padding: 4px 8px;"
            "border-radius: 4px;"
            "font-family: monospace;"
            "font-size: 12px;"
        )
        self.coord_label.move(10, 10)
        self.coord_label.adjustSize()
        self.viewer.hover_coords_signal.connect(self.update_coord_label)

        self.brush_radio = QRadioButton("Brush")
        self.poly_radio = QRadioButton("Polygon")
        self.brush_radio.setChecked(True)
        self.brush_radio.toggled.connect(self.on_mode_changed)

   
        self.load_button = QPushButton("Load map")
        self.save_button = QPushButton("Save map")
        self.publish_button = QPushButton("Publish map")

        self.load_button.clicked.connect(self.load_map)
        self.save_button.clicked.connect(self.save_map)
        self.publish_button.clicked.connect(self.publish_map)

     
        self.shortcut_undo = QShortcut(QKeySequence("Ctrl+Z"), self)
        self.shortcut_undo.activated.connect(self.viewer.undo)

 
        controls_layout = QHBoxLayout()
        controls_layout.addWidget(self.brush_radio)
        controls_layout.addWidget(self.poly_radio)
        controls_layout.addSpacing(20)
        controls_layout.addWidget(self.load_button)
        controls_layout.addWidget(self.save_button)
        controls_layout.addWidget(self.publish_button)

     
        info_label = QLabel("Free cell: left click,   Unknown cell: wheel button,   Occupied cell: right click,   Polygon: simple clicks then double click,   Undo: ctrl + z.")
        info_label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(self.viewer)
        layout.addLayout(controls_layout)
        layout.addWidget(info_label)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)
        self.resize(1200, 800)

        self.node = None

    def update_coord_label(self, col, row_gui):
        if col < 0 or row_gui < 0 or self.viewer.grid is None or not (self.node and self.node.map_msg):
            self.coord_label.setText("UTM: -")
            self.coord_label.adjustSize()
            return

        h, _ = self.viewer.grid.shape
        ros_row = (h - 1) - row_gui
        info = self.node.map_msg.info
        utm_x = info.origin.position.x + (col + 0.5) * info.resolution
        utm_y = info.origin.position.y + (ros_row + 0.5) * info.resolution

        self.coord_label.setText(f"UTM: ({utm_x:.2f} m, {utm_y:.2f} m)")
        self.coord_label.adjustSize()

    def on_mode_changed(self):
        if self.brush_radio.isChecked():
            self.viewer.set_edit_mode("brush")
        else:
            self.viewer.set_edit_mode("poly")

    def save_map(self):
        if self.viewer.grid is None:
            return
        default_dir = os.path.expanduser('~/ros2_ws/data/maps')
        os.makedirs(default_dir, exist_ok=True)
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save map", default_dir, "NumPy Map Archive (*.npz)"
        )
        if not file_path:
            return
        if not file_path.endswith('.npz'):
            file_path += '.npz'
        res = 0.3
        frame_id = 'map'
        ox, oy = 0.0, 0.0
        if self.node and self.node.map_msg:
            res = float(self.node.map_msg.info.resolution)
            frame_id = str(self.node.map_msg.header.frame_id)
            ox = float(self.node.map_msg.info.origin.position.x)
            oy = float(self.node.map_msg.info.origin.position.y)
        np.savez_compressed(
            file_path,
            grid=self.viewer.grid,
            resolution=res,
            frame_id=frame_id,
            origin_x=ox,
            origin_y=oy
        )

    def load_map(self):
        default_dir = os.path.expanduser('~/ros2_ws/data/maps')
        os.makedirs(default_dir, exist_ok=True)
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load map", default_dir, "NumPy Map Archive (*.npz)"
        )
        if not file_path:
            return
        data = np.load(file_path)
        grid = data['grid']
        res = float(data['resolution']) if 'resolution' in data else 0.3
        frame_id = str(data['frame_id']) if 'frame_id' in data else 'map'
        ox = float(data['origin_x']) if 'origin_x' in data else 0.0
        oy = float(data['origin_y']) if 'origin_y' in data else 0.0
        if self.node:
            msg = OccupancyGrid()
            msg.header.frame_id = frame_id
            #msg.header.frame_id = 'utm'
            msg.info.resolution = res
            msg.info.width = grid.shape[1]
            msg.info.height = grid.shape[0]
            msg.info.origin.position.x = ox
            msg.info.origin.position.y = oy
            msg.info.origin.orientation.w = 1.0
            self.node.map_msg = msg
            self.node.map_received = True
        self.viewer.set_grid(grid)

    def publish_map(self):
        if self.node is None or self.viewer.grid is None:
            return
        self.node.publish_edited_map(self.viewer.grid)


def main():
    rclpy.init()

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    node = MapEditorNode(window.viewer)
    window.node = node

    def ros_spin():
        rclpy.spin(node)

    thread = threading.Thread(target=ros_spin)
    thread.start()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=1.0)


if __name__ == "__main__":
    main()