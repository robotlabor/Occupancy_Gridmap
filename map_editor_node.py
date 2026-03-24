import sys
import threading
import numpy as np
import rclpy

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, QGraphicsScene
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class MapViewer(QGraphicsView):

    update_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        self.pixmap_item = None
        self.grid = None

        self.setDragMode(QGraphicsView.ScrollHandDrag)

        self.update_signal.connect(self.set_grid)


    def set_grid(self, grid):

        self.grid = grid
        self.update_image()


    def update_image(self):

        if self.grid is None:
            return

        h, w = self.grid.shape

        img = np.zeros((h, w, 3), dtype=np.uint8)

        img[self.grid == 0] = [255,255,255]
        img[self.grid == 100] = [0,0,0]
        img[self.grid == -1] = [150,150,150]

        qimg = QImage(
            img.copy().data,
            w,
            h,
            3*w,
            QImage.Format_RGB888
        )

        pix = QPixmap.fromImage(qimg).copy()

        if self.pixmap_item is None:
            self.pixmap_item = self.scene.addPixmap(pix)
        else:
            self.pixmap_item.setPixmap(pix)


    def wheelEvent(self, event):

        zoom = 1.25

        if event.angleDelta().y() > 0:
            self.scale(zoom, zoom)
        else:
            self.scale(1/zoom, 1/zoom)


    def mousePressEvent(self, event):

        if self.grid is None:
            return

        pos = self.mapToScene(event.pos())

        x = int(pos.x())
        y = int(pos.y())

        if x < 0 or y < 0:
            return

        if x >= self.grid.shape[1] or y >= self.grid.shape[0]:
            return

        if event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier:
            self.grid[y, x] = -1
        elif event.button() == Qt.LeftButton:
            self.grid[y,x] = 0
        elif event.button() == Qt.RightButton:
            self.grid[y,x] = 100
        elif event.button() == Qt.MiddleButton:
            self.grid[y, x] = -1
        
        self.update_image()


class MapEditorNode(Node):

    def __init__(self, gui):

        super().__init__('map_editor')

        self.gui = gui

        self.map_received = False

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
            '/map',
            self.map_callback,
            qos_sub
        )

        self.map_msg = None

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/edited_map',
            qos_pub
        )


    def map_callback(self, msg):

        if self.map_received:
            return

        self.get_logger().info("Map captured (one-shot)")

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
            self.get_logger().warn("No map to publish")
            return

        msg = OccupancyGrid()

        msg.header.frame_id = self.map_msg.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info

        # visszaflip ROS koordinátára
        ros_grid = np.flipud(grid).astype(np.int8)      #####################

        msg.data = ros_grid.flatten().astype(int).tolist()      ###############

        print("UNIQUE:", np.unique(ros_grid))       #!!!!!!!!!!!!!!!!!!!!!!!!

        self.publisher.publish(msg)

        self.get_logger().info("Edited map published")

class MainWindow(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("ROS2 OccupancyGrid Editor")

        self.viewer = MapViewer()

        self.publish_button = QPushButton("Publish map")
        self.publish_button.clicked.connect(self.publish_map)

        layout = QVBoxLayout()
        layout.addWidget(self.viewer)
        layout.addWidget(self.publish_button)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

        self.resize(1200, 800)

        self.node = None


    def publish_map(self):
        if self.node is None:
            return

        if self.viewer.grid is None:
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


if __name__ == "__main__":
    main()