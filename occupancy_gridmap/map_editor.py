#!/usr/bin/env python3

import math
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtCore import QPointF, QRectF, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGraphicsScene,
    QGraphicsView,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from occupancy_gridmap.map_geometry import (
    MINIMUM_CONE_SPACING_M,
    MapGeometry,
    cone_spacing_conflicts,
    inclusive_rectangle_bounds,
    polygon_fill_region,
    parse_polygon_coordinate_text,
    quaternion_to_yaw,
    sample_arc_points,
    sample_line_points,
)


class MapViewer(QGraphicsView):
    """Display a map and edit cells, cone positions, and a robot start pose."""

    update_signal = pyqtSignal(object)
    state_changed = pyqtSignal()
    cursor_position_changed = pyqtSignal(str)
    feedback = pyqtSignal(str)

    MODE_MAP = 'map'
    MODE_RECTANGLE = 'rectangle'
    MODE_POLYGON = 'polygon'
    MODE_CONES = 'cones'
    MODE_CONE_LINE = 'cone_line'
    MODE_CONE_ARC = 'cone_arc'
    MODE_ROBOT = 'robot'

    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.pixmap_item = None
        self.grid = None
        self.geometry = None
        self.frame_id = 'map'

        self.mode = self.MODE_MAP
        self.cone_positions = []
        self.robot_start_pose = None
        self.robot_drag_start = None
        self.selection_drag_start = None
        self.selection_drag_current = None
        self.selection_value = None
        self.selection_button = None
        self.polygon_vertices = []
        self.polygon_hover_cell = None
        self.polygon_value = None
        self.cone_shape_points = []
        self.cone_shape_hover = None
        self.cone_spacing = 1.0

        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.update_signal.connect(self.set_map)

    def set_map(self, map_data):
        display_grid, geometry, frame_id = map_data
        self.grid = display_grid
        self.geometry = geometry
        self.frame_id = frame_id or 'map'
        self._clear_map_selection()
        self._clear_polygon()
        self._clear_cone_shape()
        self.update_image()
        self.state_changed.emit()

    def set_mode(self, mode):
        polygon_cancelled = bool(self.polygon_vertices)
        cone_shape_cancelled = bool(self.cone_shape_points)
        self.mode = mode
        self.robot_drag_start = None
        self._clear_map_selection()
        self._clear_polygon()
        self._clear_cone_shape()
        self.update_image()
        self.state_changed.emit()
        if polygon_cancelled:
            self.feedback.emit(
                'Unfinished polygon cancelled because the mouse mode changed.'
            )
        if cone_shape_cancelled:
            self.feedback.emit(
                'Unfinished cone line or arc cancelled because the mouse '
                'mode changed.'
            )

    def set_cone_spacing(self, spacing):
        self.cone_spacing = float(spacing)
        if self.cone_shape_points:
            self.update_image()

    def update_image(self):
        if self.grid is None:
            return

        height, width = self.grid.shape
        image = np.zeros((height, width, 3), dtype=np.uint8)

        image[self.grid == 0] = [255, 255, 255]
        image[self.grid == 100] = [0, 0, 0]
        image[self.grid == -1] = [150, 150, 150]

        qimage = QImage(
            image.data,
            width,
            height,
            3 * width,
            QImage.Format_RGB888,
        ).copy()

        self._draw_overlays(qimage)
        pixmap = QPixmap.fromImage(qimage)

        if self.pixmap_item is None:
            self.pixmap_item = self.scene.addPixmap(pixmap)
        else:
            self.pixmap_item.setPixmap(pixmap)

        self.scene.setSceneRect(self.pixmap_item.boundingRect())

    def _draw_overlays(self, image):
        if self.geometry is None:
            return

        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        self._draw_map_selection(painter)
        self._draw_polygon_selection(painter)
        self._draw_cone_shape(painter)

        cone_pen = QPen(QColor(120, 35, 0))
        cone_pen.setWidth(1)
        painter.setPen(cone_pen)
        painter.setBrush(QBrush(QColor(255, 120, 20)))

        for index, (world_x, world_y) in enumerate(
            self.cone_positions,
            start=1,
        ):
            display_x, display_y = self.geometry.world_to_display(
                world_x,
                world_y,
            )
            centre = QPointF(display_x, display_y)
            radius = 3.2
            painter.drawEllipse(centre, radius, radius)

            label = str(index)
            label_font = QFont(painter.font())
            label_font.setBold(True)
            label_font.setPixelSize(5 if len(label) <= 2 else 4)
            painter.setFont(label_font)
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(
                QRectF(
                    display_x - radius,
                    display_y - radius,
                    radius * 2.0,
                    radius * 2.0,
                ),
                Qt.AlignCenter,
                label,
            )
            painter.setPen(cone_pen)

        if self.robot_start_pose is not None:
            self._draw_robot_start(painter)

        painter.end()

    def _draw_map_selection(self, painter):
        if (
            self.selection_drag_start is None
            or self.selection_drag_current is None
            or self.selection_value is None
        ):
            return

        left, top, right, bottom = inclusive_rectangle_bounds(
            self.selection_drag_start,
            self.selection_drag_current,
        )
        colours = {
            0: QColor(30, 160, 80),
            100: QColor(220, 45, 45),
            -1: QColor(235, 165, 25),
        }
        colour = colours[self.selection_value]
        fill_colour = QColor(colour)
        fill_colour.setAlpha(75)

        selection_pen = QPen(colour)
        selection_pen.setWidth(2)
        painter.setPen(selection_pen)
        painter.setBrush(QBrush(fill_colour))
        painter.drawRect(
            QRectF(
                float(left),
                float(top),
                float(right - left + 1),
                float(bottom - top + 1),
            )
        )

    def _draw_polygon_selection(self, painter):
        if not self.polygon_vertices or self.polygon_value is None:
            return

        colours = {
            0: QColor(30, 160, 80),
            100: QColor(220, 45, 45),
            -1: QColor(235, 165, 25),
        }
        colour = colours[self.polygon_value]
        fill_colour = QColor(colour)
        fill_colour.setAlpha(55)

        points = [
            QPointF(column + 0.5, display_row + 0.5)
            for column, display_row in self.polygon_vertices
        ]
        preview_points = list(points)
        if self.polygon_hover_cell is not None:
            hover_point = QPointF(
                self.polygon_hover_cell[0] + 0.5,
                self.polygon_hover_cell[1] + 0.5,
            )
            if not preview_points or hover_point != preview_points[-1]:
                preview_points.append(hover_point)

        polygon_pen = QPen(colour)
        polygon_pen.setWidth(2)
        painter.setPen(polygon_pen)

        if len(preview_points) >= 3:
            painter.setBrush(QBrush(fill_colour))
            painter.drawPolygon(QPolygonF(preview_points))
        elif len(preview_points) == 2:
            painter.setBrush(Qt.NoBrush)
            painter.drawLine(preview_points[0], preview_points[1])

        painter.setBrush(QBrush(colour))
        for point in points:
            painter.drawEllipse(point, 3.0, 3.0)

        first_point = points[0]
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(first_point, 1.5, 1.5)

    def _draw_cone_shape(self, painter):
        if not self.cone_shape_points:
            return

        control_points = list(self.cone_shape_points)
        if (
            self.cone_shape_hover is not None
            and self.cone_shape_hover != control_points[-1]
        ):
            control_points.append(self.cone_shape_hover)

        preview_points = control_points
        try:
            if self.mode == self.MODE_CONE_LINE and len(control_points) >= 2:
                preview_points = sample_line_points(
                    control_points[0],
                    control_points[-1],
                    self.cone_spacing,
                )
            elif self.mode == self.MODE_CONE_ARC and len(control_points) >= 3:
                preview_points = sample_arc_points(
                    control_points[0],
                    control_points[1],
                    control_points[-1],
                    self.cone_spacing,
                )
        except ValueError:
            preview_points = control_points

        if len(preview_points) > 300:
            stride = math.ceil(len(preview_points) / 300)
            preview_points = preview_points[::stride]
            if preview_points[-1] != control_points[-1]:
                preview_points.append(control_points[-1])

        display_points = [
            QPointF(*self.geometry.world_to_display(*world_point))
            for world_point in preview_points
        ]
        control_display_points = [
            QPointF(*self.geometry.world_to_display(*world_point))
            for world_point in self.cone_shape_points
        ]

        preview_pen = QPen(QColor(0, 165, 215))
        preview_pen.setWidth(2)
        preview_pen.setStyle(Qt.DashLine)
        painter.setPen(preview_pen)
        painter.setBrush(Qt.NoBrush)
        if len(display_points) >= 2:
            painter.drawPolyline(QPolygonF(display_points))

        painter.setPen(QPen(QColor(0, 100, 150), 1))
        painter.setBrush(QBrush(QColor(0, 205, 255)))
        for point in control_display_points:
            painter.drawEllipse(point, 3.0, 3.0)

    def _draw_robot_start(self, painter):
        world_x, world_y, yaw = self.robot_start_pose
        display_x, display_y = self.geometry.world_to_display(world_x, world_y)

        painter.setPen(QPen(QColor(0, 55, 130), 2))
        painter.setBrush(QBrush(QColor(30, 145, 255)))
        centre = QPointF(display_x, display_y)
        painter.drawEllipse(centre, 6.0, 6.0)

        heading_world_x = world_x + math.cos(yaw)
        heading_world_y = world_y + math.sin(yaw)
        heading_display_x, heading_display_y = self.geometry.world_to_display(
            heading_world_x,
            heading_world_y,
        )

        vector_x = heading_display_x - display_x
        vector_y = heading_display_y - display_y
        vector_length = math.hypot(vector_x, vector_y)
        if vector_length <= 1e-9:
            return

        vector_x = vector_x / vector_length * 28.0
        vector_y = vector_y / vector_length * 28.0
        arrow_end = QPointF(display_x + vector_x, display_y + vector_y)
        painter.drawLine(centre, arrow_end)

        unit_x = vector_x / 28.0
        unit_y = vector_y / 28.0
        perpendicular_x = -unit_y
        perpendicular_y = unit_x

        arrow_left = QPointF(
            arrow_end.x() - 8.0 * unit_x + 5.0 * perpendicular_x,
            arrow_end.y() - 8.0 * unit_y + 5.0 * perpendicular_y,
        )
        arrow_right = QPointF(
            arrow_end.x() - 8.0 * unit_x - 5.0 * perpendicular_x,
            arrow_end.y() - 8.0 * unit_y - 5.0 * perpendicular_y,
        )
        painter.drawLine(arrow_end, arrow_left)
        painter.drawLine(arrow_end, arrow_right)

    def wheelEvent(self, event):
        zoom_factor = 1.25

        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1.0 / zoom_factor, 1.0 / zoom_factor)

    def mousePressEvent(self, event):
        if (
            event.button() == Qt.LeftButton
            and event.modifiers() & Qt.AltModifier
        ):
            super().mousePressEvent(event)
            return

        cell = self._event_cell(event)
        if cell is None:
            super().mousePressEvent(event)
            return

        column, display_row = cell

        if self.mode == self.MODE_MAP:
            self._edit_map_cell(event, column, display_row)
        elif self.mode == self.MODE_RECTANGLE:
            self._start_map_selection(event, column, display_row)
        elif self.mode == self.MODE_POLYGON:
            self._edit_polygon(event, column, display_row)
        elif self.mode == self.MODE_CONES:
            self._edit_cones(event, column, display_row)
        elif self.mode in (self.MODE_CONE_LINE, self.MODE_CONE_ARC):
            self._edit_cone_shape(event, column, display_row)
        elif self.mode == self.MODE_ROBOT:
            self._start_robot_pose_edit(event, column, display_row)
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        scene_position = self.mapToScene(event.pos())
        self._emit_cursor_position(scene_position)

        if (
            event.buttons() & Qt.LeftButton
            and event.modifiers() & Qt.AltModifier
        ):
            super().mouseMoveEvent(event)
            return

        if (
            self.mode == self.MODE_RECTANGLE
            and self.selection_drag_start is not None
        ):
            self._update_map_selection(scene_position)
            event.accept()
            return

        if self.mode == self.MODE_POLYGON and self.polygon_vertices:
            self.polygon_hover_cell = self._clamped_scene_cell(scene_position)
            self.update_image()
            event.accept()
            return

        if (
            self.mode in (self.MODE_CONE_LINE, self.MODE_CONE_ARC)
            and self.cone_shape_points
        ):
            self.cone_shape_hover = self.geometry.display_to_world(
                min(max(scene_position.x(), 0.0), self.geometry.width),
                min(max(scene_position.y(), 0.0), self.geometry.height),
            )
            self.update_image()
            event.accept()
            return

        if (
            self.mode == self.MODE_ROBOT
            and self.robot_drag_start is not None
            and event.buttons() & Qt.LeftButton
        ):
            self._update_robot_heading(scene_position)
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if (
            self.mode == self.MODE_RECTANGLE
            and self.selection_drag_start is not None
            and event.button() == self.selection_button
        ):
            self._finish_map_selection(self.mapToScene(event.pos()))
            event.accept()
            return

        if (
            self.mode == self.MODE_ROBOT
            and self.robot_drag_start is not None
            and event.button() == Qt.LeftButton
        ):
            self._update_robot_heading(self.mapToScene(event.pos()))
            self.robot_drag_start = None
            self.state_changed.emit()
            event.accept()
            return

        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if (
            self.mode == self.MODE_POLYGON
            and self.polygon_vertices
            and event.button() == Qt.LeftButton
        ):
            cell = self._event_cell(event)
            if cell is not None and cell != self.polygon_vertices[-1]:
                self.polygon_vertices.append(cell)
            self.finish_polygon()
            event.accept()
            return

        super().mouseDoubleClickEvent(event)

    def keyPressEvent(self, event):
        if self.mode == self.MODE_POLYGON and self.polygon_vertices:
            if event.key() == Qt.Key_Escape:
                self.cancel_polygon()
                event.accept()
                return
            if event.key() in (Qt.Key_Return, Qt.Key_Enter):
                self.finish_polygon()
                event.accept()
                return
            if event.key() in (Qt.Key_Backspace, Qt.Key_Delete):
                self.undo_polygon_vertex()
                event.accept()
                return

        if (
            self.mode in (self.MODE_CONE_LINE, self.MODE_CONE_ARC)
            and self.cone_shape_points
            and event.key() in (
                Qt.Key_Escape,
                Qt.Key_Backspace,
                Qt.Key_Delete,
            )
        ):
            self.cancel_cone_shape()
            event.accept()
            return

        super().keyPressEvent(event)

    def _event_cell(self, event):
        if self.grid is None or self.geometry is None:
            return None

        scene_position = self.mapToScene(event.pos())
        column = math.floor(scene_position.x())
        display_row = math.floor(scene_position.y())

        if not self.geometry.contains_display_cell(column, display_row):
            return None
        return column, display_row

    def _edit_map_cell(self, event, column, display_row):
        edited = False

        if event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier:
            self.grid[display_row, column] = -1
            edited = True
        elif event.button() == Qt.LeftButton:
            self.grid[display_row, column] = 0
            edited = True
        elif event.button() == Qt.RightButton:
            self.grid[display_row, column] = 100
            edited = True
        elif event.button() == Qt.MiddleButton:
            self.grid[display_row, column] = -1
            edited = True

        if edited:
            self.update_image()
            self.state_changed.emit()
            event.accept()
        else:
            super().mousePressEvent(event)

    def _start_map_selection(self, event, column, display_row):
        selection_value = self._map_value_for_event(event)
        if selection_value is None:
            super().mousePressEvent(event)
            return

        self.selection_drag_start = (column, display_row)
        self.selection_drag_current = (column, display_row)
        self.selection_value = selection_value
        self.selection_button = event.button()
        self.update_image()
        event.accept()

    @staticmethod
    def _map_value_for_event(event):
        if event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier:
            return -1
        if event.button() == Qt.LeftButton:
            return 0
        if event.button() == Qt.RightButton:
            return 100
        if event.button() == Qt.MiddleButton:
            return -1
        return None

    def _update_map_selection(self, scene_position):
        self.selection_drag_current = self._clamped_scene_cell(scene_position)
        self.update_image()

    def _finish_map_selection(self, scene_position):
        self._update_map_selection(scene_position)
        left, top, right, bottom = inclusive_rectangle_bounds(
            self.selection_drag_start,
            self.selection_drag_current,
        )
        selection_value = self.selection_value
        self.grid[top:bottom + 1, left:right + 1] = selection_value

        column_count = right - left + 1
        row_count = bottom - top + 1
        value_names = {
            0: 'free',
            100: 'occupied',
            -1: 'unknown',
        }
        self._clear_map_selection()
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit(
            f'{column_count} x {row_count} cells set to '
            f'{value_names[selection_value]} ({selection_value}).'
        )

    def _clamped_scene_cell(self, scene_position):
        column = min(
            max(math.floor(scene_position.x()), 0),
            self.geometry.width - 1,
        )
        display_row = min(
            max(math.floor(scene_position.y()), 0),
            self.geometry.height - 1,
        )
        return column, display_row

    def _clear_map_selection(self):
        self.selection_drag_start = None
        self.selection_drag_current = None
        self.selection_value = None
        self.selection_button = None

    def _edit_polygon(self, event, column, display_row):
        cell = (column, display_row)

        if not self.polygon_vertices:
            polygon_value = self._map_value_for_event(event)
            if polygon_value is None:
                super().mousePressEvent(event)
                return

            self.polygon_vertices = [cell]
            self.polygon_hover_cell = cell
            self.polygon_value = polygon_value
            value_names = {
                0: 'free',
                100: 'occupied',
                -1: 'unknown',
            }
            self.feedback.emit(
                f'Polygon started for {value_names[polygon_value]} cells. '
                'Add vertices with left-click.'
            )
            self.update_image()
            self.state_changed.emit()
            event.accept()
            return

        if event.button() == Qt.RightButton:
            self.undo_polygon_vertex()
            event.accept()
            return

        if event.button() != Qt.LeftButton:
            event.accept()
            return

        if cell == self.polygon_vertices[0] and len(self.polygon_vertices) >= 3:
            self.finish_polygon()
            event.accept()
            return

        if cell != self.polygon_vertices[-1]:
            self.polygon_vertices.append(cell)
            self.polygon_hover_cell = cell
            self.feedback.emit(
                f'Polygon vertex {len(self.polygon_vertices)} added. '
                'Click the first vertex, double-click, or press Enter to finish.'
            )
            self.update_image()
            self.state_changed.emit()
        event.accept()

    def finish_polygon(self):
        if not self.polygon_vertices:
            self.feedback.emit('Start a polygon with a map click first.')
            return False
        if len(set(self.polygon_vertices)) < 3:
            self.feedback.emit('A polygon needs at least three vertices.')
            return False

        try:
            left, top, right, bottom, mask = polygon_fill_region(
                self.polygon_vertices,
                self.geometry.width,
                self.geometry.height,
            )
        except ValueError as error:
            self.feedback.emit(f'Polygon not applied: {error}.')
            return False

        polygon_value = self.polygon_value
        region = self.grid[top:bottom + 1, left:right + 1]
        region[mask] = polygon_value
        edited_cell_count = int(np.count_nonzero(mask))
        vertex_count = len(self.polygon_vertices)
        value_names = {
            0: 'free',
            100: 'occupied',
            -1: 'unknown',
        }

        self._clear_polygon()
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit(
            f'{vertex_count}-vertex polygon applied: {edited_cell_count} cells '
            f'set to {value_names[polygon_value]} ({polygon_value}).'
        )
        return True

    def set_polygon_world(
        self,
        world_vertices,
        polygon_value,
        apply_now=False,
    ):
        """Load or apply a polygon whose vertices use map/world coordinates."""
        if self.grid is None or self.geometry is None:
            self.feedback.emit(
                'Coordinate polygon rejected: no map is loaded.'
            )
            return False
        if polygon_value not in (0, 100, -1):
            self.feedback.emit(
                'Coordinate polygon rejected: invalid cell value.'
            )
            return False

        try:
            display_vertices = self.geometry.world_polygon_to_display_cells(
                world_vertices
            )
            polygon_fill_region(
                display_vertices,
                self.geometry.width,
                self.geometry.height,
            )
        except (TypeError, ValueError) as error:
            self.feedback.emit(f'Coordinate polygon rejected: {error}.')
            return False

        self.polygon_vertices = display_vertices
        self.polygon_hover_cell = display_vertices[-1]
        self.polygon_value = polygon_value
        self.update_image()
        self.state_changed.emit()

        value_names = {
            0: 'free',
            100: 'occupied',
            -1: 'unknown',
        }
        if apply_now:
            return self.finish_polygon()

        self.feedback.emit(
            f'Coordinate polygon preview loaded with '
            f'{len(display_vertices)} vertices for '
            f'{value_names[polygon_value]} cells. Select Finish polygon or '
            'Apply coordinate polygon to edit the map.'
        )
        return True

    def undo_polygon_vertex(self):
        if not self.polygon_vertices:
            self.feedback.emit('There are no polygon vertices to remove.')
            return False

        self.polygon_vertices.pop()
        if not self.polygon_vertices:
            self._clear_polygon()
            self.feedback.emit('Polygon drawing cancelled.')
        else:
            self.polygon_hover_cell = self.polygon_vertices[-1]
            self.feedback.emit(
                f'Last polygon vertex removed; '
                f'{len(self.polygon_vertices)} remain.'
            )

        self.update_image()
        self.state_changed.emit()
        return True

    def cancel_polygon(self):
        if not self.polygon_vertices:
            self.feedback.emit('There is no active polygon drawing.')
            return False

        self._clear_polygon()
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit('Polygon drawing cancelled without changing the map.')
        return True

    def _clear_polygon(self):
        self.polygon_vertices = []
        self.polygon_hover_cell = None
        self.polygon_value = None

    def _edit_cone_shape(self, event, column, display_row):
        if event.button() == Qt.RightButton:
            self.cancel_cone_shape()
            event.accept()
            return

        if event.button() != Qt.LeftButton:
            super().mousePressEvent(event)
            return

        if not self._cell_is_free(column, display_row):
            self.feedback.emit(
                'Line and arc control points must be on free (white) cells.'
            )
            event.accept()
            return

        world_position = self.geometry.display_cell_to_world(
            column,
            display_row,
        )
        self.cone_shape_points.append(world_position)
        self.cone_shape_hover = world_position

        required_point_count = (
            2 if self.mode == self.MODE_CONE_LINE else 3
        )
        if len(self.cone_shape_points) < required_point_count:
            if self.mode == self.MODE_CONE_LINE:
                message = 'Line start set. Left-click the line end.'
            elif len(self.cone_shape_points) == 1:
                message = (
                    'Arc start set. Left-click a point that the arc must '
                    'pass through.'
                )
            else:
                message = 'Arc middle point set. Left-click the arc end.'
            self.feedback.emit(message)
            self.update_image()
            self.state_changed.emit()
            event.accept()
            return

        control_points = list(self.cone_shape_points)
        try:
            if self.mode == self.MODE_CONE_LINE:
                generated_points = sample_line_points(
                    control_points[0],
                    control_points[1],
                    self.cone_spacing,
                )
                shape_name = 'line'
            else:
                generated_points = sample_arc_points(
                    control_points[0],
                    control_points[1],
                    control_points[2],
                    self.cone_spacing,
                )
                shape_name = 'arc'
        except ValueError as error:
            self.cone_shape_points.pop()
            self.cone_shape_hover = self.cone_shape_points[-1]
            self.feedback.emit(f'Cones were not added: {error}.')
            self.update_image()
            self.state_changed.emit()
            event.accept()
            return

        if not self._append_generated_cones(generated_points, shape_name):
            self.cone_shape_points.pop()
            self.cone_shape_hover = self.cone_shape_points[-1]
            self.update_image()
            self.state_changed.emit()
            event.accept()
            return

        self._clear_cone_shape()
        self.update_image()
        self.state_changed.emit()
        event.accept()

    def _append_generated_cones(self, generated_points, shape_name):
        invalid_indices = []
        for index, world_position in enumerate(generated_points):
            cell = self.geometry.world_to_display_cell(*world_position)
            if cell is None or not self._cell_is_free(*cell):
                invalid_indices.append(index)

        if invalid_indices:
            self.feedback.emit(
                f'The {shape_name} was not applied because '
                f'{len(invalid_indices)} generated cone position(s) are '
                'outside the map or not on free cells.'
            )
            return False

        known_positions = {
            (round(point[0], 6), round(point[1], 6))
            for point in self.cone_positions
        }
        unique_points = []
        for point in generated_points:
            key = (round(point[0], 6), round(point[1], 6))
            if key in known_positions:
                continue
            known_positions.add(key)
            unique_points.append((float(point[0]), float(point[1])))

        if not unique_points:
            self.feedback.emit(
                f'The {shape_name} did not add new cones because every '
                'generated position already exists.'
            )
            return False

        combined_positions = list(self.cone_positions) + unique_points
        spacing_conflicts = cone_spacing_conflicts(combined_positions)
        if spacing_conflicts:
            first_index, second_index = spacing_conflicts[0]
            first_position = combined_positions[first_index]
            second_position = combined_positions[second_index]
            actual_distance = math.dist(first_position, second_position)
            self.feedback.emit(
                f'The {shape_name} was not applied because two cone '
                f'positions would be only {actual_distance:.3f} m apart. '
                f'The minimum permitted spacing is '
                f'{MINIMUM_CONE_SPACING_M:.3f} m.'
            )
            return False

        self.cone_positions.extend(unique_points)
        skipped_count = len(generated_points) - len(unique_points)
        skipped_text = (
            f' {skipped_count} duplicate position(s) were skipped.'
            if skipped_count
            else ''
        )
        self.feedback.emit(
            f'{len(unique_points)} cones added along the {shape_name} with '
            f'a requested maximum spacing of {self.cone_spacing:.3f} m and '
            f'a minimum separation of {MINIMUM_CONE_SPACING_M:.3f} m.'
            f'{skipped_text}'
        )
        return True

    def cancel_cone_shape(self):
        if not self.cone_shape_points:
            self.feedback.emit('There is no active cone line or arc.')
            return False

        self._clear_cone_shape()
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit('Cone line or arc drawing cancelled.')
        return True

    def _clear_cone_shape(self):
        self.cone_shape_points = []
        self.cone_shape_hover = None

    def _edit_cones(self, event, column, display_row):
        if event.button() == Qt.LeftButton:
            if not self._cell_is_free(column, display_row):
                self.feedback.emit(
                    'A cone can only be placed on a free (white) cell.'
                )
                event.accept()
                return

            world_position = self.geometry.display_cell_to_world(
                column,
                display_row,
            )
            if not self._cone_has_minimum_spacing(world_position):
                event.accept()
                return
            self.cone_positions.append(world_position)
            self.feedback.emit(
                f'Cone {len(self.cone_positions)} added at '
                f'({world_position[0]:.3f}, {world_position[1]:.3f}).'
            )
            self.update_image()
            self.state_changed.emit()
            event.accept()
            return

        if event.button() == Qt.RightButton:
            removed = self._remove_nearest_cone(event)
            if removed:
                self.update_image()
                self.state_changed.emit()
            event.accept()
            return

        super().mousePressEvent(event)

    def add_cone_world(self, world_x, world_y):
        if self.grid is None or self.geometry is None:
            self.feedback.emit('No map has been received yet.')
            return False
        if not math.isfinite(world_x) or not math.isfinite(world_y):
            self.feedback.emit('Cone coordinates must be finite numbers.')
            return False

        cell = self.geometry.world_to_display_cell(world_x, world_y)
        if cell is None:
            self.feedback.emit('The cone coordinate is outside the map.')
            return False
        if not self._cell_is_free(*cell):
            self.feedback.emit(
                'The cone coordinate is not on a free (white) cell.'
            )
            return False

        world_position = (float(world_x), float(world_y))
        if not self._cone_has_minimum_spacing(world_position):
            return False

        self.cone_positions.append(world_position)
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit(
            f'Cone {len(self.cone_positions)} added from coordinates at '
            f'({world_x:.3f}, {world_y:.3f}).'
        )
        return True

    def _cone_has_minimum_spacing(self, world_position):
        if not self.cone_positions:
            return True

        distances = [
            math.dist(existing_position, world_position)
            for existing_position in self.cone_positions
        ]
        nearest_distance = min(distances)
        if nearest_distance + 1e-9 >= MINIMUM_CONE_SPACING_M:
            return True

        nearest_index = distances.index(nearest_distance)
        self.feedback.emit(
            f'The cone was not added because it is only '
            f'{nearest_distance:.3f} m from cone {nearest_index + 1}. '
            f'The minimum permitted spacing is '
            f'{MINIMUM_CONE_SPACING_M:.3f} m.'
        )
        return False

    def _remove_nearest_cone(self, event):
        if not self.cone_positions:
            self.feedback.emit('There are no cones to remove.')
            return False

        scene_position = self.mapToScene(event.pos())
        zoom = max(
            abs(self.transform().m11()),
            abs(self.transform().m22()),
            1e-9,
        )
        maximum_distance = 14.0 / zoom

        distances = []
        for world_position in self.cone_positions:
            display_position = self.geometry.world_to_display(*world_position)
            distances.append(
                math.hypot(
                    display_position[0] - scene_position.x(),
                    display_position[1] - scene_position.y(),
                )
            )

        nearest_index = int(np.argmin(distances))
        if distances[nearest_index] > maximum_distance:
            self.feedback.emit('Right-click closer to the cone to remove it.')
            return False

        removed = self.cone_positions.pop(nearest_index)
        self.feedback.emit(
            f'Cone {nearest_index + 1} removed from '
            f'({removed[0]:.3f}, {removed[1]:.3f}).'
        )
        return True

    def _start_robot_pose_edit(self, event, column, display_row):
        if event.button() == Qt.RightButton:
            self.clear_robot_start()
            event.accept()
            return

        if event.button() != Qt.LeftButton:
            super().mousePressEvent(event)
            return

        if not self._cell_is_free(column, display_row):
            self.feedback.emit(
                'The robot start can only be placed on a free (white) cell.'
            )
            event.accept()
            return

        world_x, world_y = self.geometry.display_cell_to_world(
            column,
            display_row,
        )
        self.robot_drag_start = (world_x, world_y)
        self.robot_start_pose = (world_x, world_y, 0.0)
        self.update_image()
        self.state_changed.emit()
        event.accept()

    def _update_robot_heading(self, scene_position):
        if self.robot_drag_start is None:
            return

        clamped_x = min(max(scene_position.x(), 0.0), self.geometry.width)
        clamped_y = min(max(scene_position.y(), 0.0), self.geometry.height)
        heading_x, heading_y = self.geometry.display_to_world(
            clamped_x,
            clamped_y,
        )

        start_x, start_y = self.robot_drag_start
        delta_x = heading_x - start_x
        delta_y = heading_y - start_y

        if math.hypot(delta_x, delta_y) > self.geometry.resolution * 0.25:
            yaw = math.atan2(delta_y, delta_x)
        else:
            yaw = 0.0

        self.robot_start_pose = (start_x, start_y, yaw)
        self.update_image()

    def set_robot_start_world(self, world_x, world_y, yaw):
        if self.grid is None or self.geometry is None:
            self.feedback.emit('No map has been received yet.')
            return False
        if not all(math.isfinite(value) for value in (world_x, world_y, yaw)):
            self.feedback.emit(
                'Robot position and orientation must be finite numbers.'
            )
            return False

        cell = self.geometry.world_to_display_cell(world_x, world_y)
        if cell is None:
            self.feedback.emit('The robot coordinate is outside the map.')
            return False
        if not self._cell_is_free(*cell):
            self.feedback.emit(
                'The robot coordinate is not on a free (white) cell.'
            )
            return False

        normalised_yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.robot_start_pose = (
            float(world_x),
            float(world_y),
            normalised_yaw,
        )
        self.robot_drag_start = None
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit(
            f'Robot start set from coordinates: X={world_x:.3f}, '
            f'Y={world_y:.3f}, yaw={math.degrees(normalised_yaw):.1f} deg.'
        )
        return True

    def _emit_cursor_position(self, scene_position):
        if self.geometry is None:
            return
        if not (
            0.0 <= scene_position.x() <= self.geometry.width
            and 0.0 <= scene_position.y() <= self.geometry.height
        ):
            self.cursor_position_changed.emit('')
            return

        world_x, world_y = self.geometry.display_to_world(
            scene_position.x(),
            scene_position.y(),
        )
        self.cursor_position_changed.emit(
            f'Cursor: X={world_x:.3f} m, Y={world_y:.3f} m'
        )

    def _cell_is_free(self, column, display_row):
        return int(self.grid[display_row, column]) == 0

    def invalid_cone_indices(self):
        invalid_indices = []
        for index, (world_x, world_y) in enumerate(self.cone_positions):
            cell = self.geometry.world_to_display_cell(world_x, world_y)
            if cell is None or not self._cell_is_free(*cell):
                invalid_indices.append(index)
        return invalid_indices

    def cone_spacing_conflicts(self):
        return cone_spacing_conflicts(self.cone_positions)

    def robot_start_is_free(self):
        if self.robot_start_pose is None:
            return False

        cell = self.geometry.world_to_display_cell(
            self.robot_start_pose[0],
            self.robot_start_pose[1],
        )
        return cell is not None and self._cell_is_free(*cell)

    def clear_cones(self):
        self.cone_positions = []
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit('All cone positions were cleared.')

    def clear_robot_start(self):
        self.robot_start_pose = None
        self.robot_drag_start = None
        self.update_image()
        self.state_changed.emit()
        self.feedback.emit('The robot start pose was cleared.')


class MapEditorNode(Node):
    """Publish the edited map, graphical waypoints, and graphical start pose."""

    def __init__(self, gui):
        super().__init__('map_editor')

        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/edited_map')
        self.declare_parameter('waypoint_topic', '/waypoints')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_static_tf_default', True)

        input_map_topic = self.get_parameter('input_map_topic').value
        output_map_topic = self.get_parameter('output_map_topic').value
        waypoint_topic = self.get_parameter('waypoint_topic').value
        initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_static_tf_default = bool(
            self.get_parameter('publish_static_tf_default').value
        )

        self.gui = gui
        self.map_received = False
        self.map_msg = None

        subscription_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            input_map_topic,
            self.map_callback,
            subscription_qos,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            output_map_topic,
            publisher_qos,
        )
        self.waypoint_publisher = self.create_publisher(
            PoseArray,
            waypoint_topic,
            10,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            10,
        )
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.get_logger().info(
            f'Waiting for one map on {input_map_topic}; outputs: '
            f'{output_map_topic}, {waypoint_topic}, {initial_pose_topic}.'
        )

    def map_callback(self, msg):
        if self.map_received:
            return

        self.get_logger().info('Map captured in one-shot mode.')
        self.map_msg = msg

        grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height,
            msg.info.width,
        )
        display_grid = np.flipud(grid).copy()

        origin_orientation = msg.info.origin.orientation
        geometry = MapGeometry(
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=quaternion_to_yaw(
                origin_orientation.x,
                origin_orientation.y,
                origin_orientation.z,
                origin_orientation.w,
            ),
        )

        self.gui.update_signal.emit(
            (display_grid, geometry, msg.header.frame_id)
        )
        self.map_received = True

    def publish_edited_map(self, grid):
        if self.map_msg is None:
            self.get_logger().warning('No map has been received yet.')
            return False

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_msg.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info

        ros_grid = np.flipud(grid).astype(np.int8)
        msg.data = ros_grid.flatten(order='C').astype(int).tolist()

        self.map_publisher.publish(msg)
        self.get_logger().info(
            f'Edited map published. Cell values: {np.unique(ros_grid).tolist()}'
        )
        return True

    def publish_waypoints(self, points, frame_id, log=True):
        if not points:
            if log:
                self.get_logger().warning('No cone positions have been set.')
            return False

        msg = PoseArray()
        msg.header.frame_id = frame_id or 'map'

        for world_x, world_y in points:
            pose = Pose()
            pose.position.x = float(world_x)
            pose.position.y = float(world_y)
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        if log:
            self.get_logger().info(
                f'Published {len(msg.poses)} graphical cone positions on '
                f'{self.waypoint_publisher.topic_name}.'
            )
        return True

    def publish_robot_start(self, robot_pose, frame_id, publish_static_tf):
        if robot_pose is None:
            self.get_logger().warning('No robot start pose has been set.')
            return False

        world_x, world_y, yaw = robot_pose
        quaternion_z = math.sin(yaw / 2.0)
        quaternion_w = math.cos(yaw / 2.0)
        map_frame = frame_id or 'map'
        stamp = self.get_clock().now().to_msg()

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = map_frame
        initial_pose.header.stamp = stamp
        initial_pose.pose.pose.position.x = float(world_x)
        initial_pose.pose.pose.position.y = float(world_y)
        initial_pose.pose.pose.orientation.z = quaternion_z
        initial_pose.pose.pose.orientation.w = quaternion_w
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = math.radians(15.0) ** 2
        self.initial_pose_publisher.publish(initial_pose)

        if publish_static_tf:
            transform = TransformStamped()
            transform.header.frame_id = map_frame
            transform.header.stamp = stamp
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(world_x)
            transform.transform.translation.y = float(world_y)
            transform.transform.rotation.z = quaternion_z
            transform.transform.rotation.w = quaternion_w
            self.static_tf_broadcaster.sendTransform(transform)

        tf_status = (
            f' and static {map_frame} -> {self.base_frame} TF'
            if publish_static_tf
            else ''
        )
        self.get_logger().info(
            f'Published robot start on {self.initial_pose_publisher.topic_name}'
            f'{tf_status}: x={world_x:.3f}, y={world_y:.3f}, '
            f'yaw={math.degrees(yaw):.1f} deg.'
        )
        return True


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('ROS 2 Map, Cone and Robot Start Editor')

        self.viewer = MapViewer()
        self.node = None

        self.mode_group = QButtonGroup(self)
        self.map_mode = QRadioButton('Edit cells')
        self.rectangle_mode = QRadioButton('Edit rectangle')
        self.polygon_mode = QRadioButton('Edit polygon')
        self.cone_mode = QRadioButton('Place cones')
        self.cone_line_mode = QRadioButton('Cones on line')
        self.cone_arc_mode = QRadioButton('Cones on arc')
        self.robot_mode = QRadioButton('Set robot start')
        self.map_mode.setChecked(True)

        for mode_button in (
            self.map_mode,
            self.rectangle_mode,
            self.polygon_mode,
            self.cone_mode,
            self.cone_line_mode,
            self.cone_arc_mode,
            self.robot_mode,
        ):
            self.mode_group.addButton(mode_button)

        self.map_mode.toggled.connect(
            lambda checked: self._set_mode(MapViewer.MODE_MAP, checked)
        )
        self.rectangle_mode.toggled.connect(
            lambda checked: self._set_mode(
                MapViewer.MODE_RECTANGLE,
                checked,
            )
        )
        self.polygon_mode.toggled.connect(
            lambda checked: self._set_mode(
                MapViewer.MODE_POLYGON,
                checked,
            )
        )
        self.cone_mode.toggled.connect(
            lambda checked: self._set_mode(MapViewer.MODE_CONES, checked)
        )
        self.cone_line_mode.toggled.connect(
            lambda checked: self._set_mode(
                MapViewer.MODE_CONE_LINE,
                checked,
            )
        )
        self.cone_arc_mode.toggled.connect(
            lambda checked: self._set_mode(
                MapViewer.MODE_CONE_ARC,
                checked,
            )
        )
        self.robot_mode.toggled.connect(
            lambda checked: self._set_mode(MapViewer.MODE_ROBOT, checked)
        )

        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel('Mouse mode:'))
        mode_layout.addWidget(self.map_mode)
        mode_layout.addWidget(self.rectangle_mode)
        mode_layout.addWidget(self.polygon_mode)
        mode_layout.addWidget(self.cone_mode)
        mode_layout.addWidget(self.cone_line_mode)
        mode_layout.addWidget(self.cone_arc_mode)
        mode_layout.addWidget(self.robot_mode)
        mode_layout.addStretch()

        self.cone_spacing_input = QDoubleSpinBox()
        self.cone_spacing_input.setDecimals(3)
        self.cone_spacing_input.setRange(
            MINIMUM_CONE_SPACING_M,
            100000.0,
        )
        self.cone_spacing_input.setSingleStep(0.1)
        self.cone_spacing_input.setValue(1.0)
        self.cone_spacing_input.setSuffix(' m')
        self.cone_spacing_input.valueChanged.connect(
            self.viewer.set_cone_spacing
        )

        spacing_layout = QHBoxLayout()
        spacing_layout.addWidget(
            QLabel(
                'Requested maximum cone spacing on line/arc '
                f'(minimum separation {MINIMUM_CONE_SPACING_M:.2f} m):'
            )
        )
        spacing_layout.addWidget(self.cone_spacing_input)
        spacing_layout.addStretch()

        self.instructions = QLabel(
            'Cells: left=free, right=occupied, middle or Shift+left=unknown; '
            'Alt+left drag=pan.'
        )
        self.cursor_label = QLabel('')
        self.state_label = QLabel(
            'Cones: 0 | Polygon vertices: 0 | Robot start: not set'
        )
        self.feedback_label = QLabel('Waiting for the map...')
        self.feedback_label.setWordWrap(True)

        self.clear_cones_button = QPushButton('Clear cones')
        self.clear_cones_button.clicked.connect(self.viewer.clear_cones)
        self.clear_robot_button = QPushButton('Clear robot start')
        self.clear_robot_button.clicked.connect(self.viewer.clear_robot_start)
        self.finish_polygon_button = QPushButton('Finish polygon')
        self.finish_polygon_button.clicked.connect(self.viewer.finish_polygon)
        self.cancel_polygon_button = QPushButton('Cancel polygon')
        self.cancel_polygon_button.clicked.connect(self.viewer.cancel_polygon)
        self.cancel_cone_shape_button = QPushButton('Cancel line/arc')
        self.cancel_cone_shape_button.clicked.connect(
            self.viewer.cancel_cone_shape
        )

        clear_layout = QHBoxLayout()
        clear_layout.addWidget(self.clear_cones_button)
        clear_layout.addWidget(self.clear_robot_button)
        clear_layout.addWidget(self.finish_polygon_button)
        clear_layout.addWidget(self.cancel_polygon_button)
        clear_layout.addWidget(self.cancel_cone_shape_button)
        clear_layout.addStretch()

        coordinate_group = QGroupBox('Coordinate input')
        coordinate_layout = QGridLayout()

        self.cone_x_input = self._coordinate_spin_box()
        self.cone_y_input = self._coordinate_spin_box()
        self.add_coordinate_cone_button = QPushButton('Add cone')
        self.add_coordinate_cone_button.clicked.connect(
            self._add_coordinate_cone
        )
        coordinate_layout.addWidget(QLabel('Cone X [m]:'), 0, 0)
        coordinate_layout.addWidget(self.cone_x_input, 0, 1)
        coordinate_layout.addWidget(QLabel('Y [m]:'), 0, 2)
        coordinate_layout.addWidget(self.cone_y_input, 0, 3)
        coordinate_layout.addWidget(self.add_coordinate_cone_button, 0, 4)

        self.robot_x_input = self._coordinate_spin_box()
        self.robot_y_input = self._coordinate_spin_box()
        self.robot_yaw_input = QDoubleSpinBox()
        self.robot_yaw_input.setDecimals(2)
        self.robot_yaw_input.setRange(-360.0, 360.0)
        self.robot_yaw_input.setSingleStep(1.0)
        self.robot_yaw_input.setSuffix(' deg')
        self.set_coordinate_robot_button = QPushButton('Set robot')
        self.set_coordinate_robot_button.clicked.connect(
            self._set_coordinate_robot
        )
        coordinate_layout.addWidget(QLabel('Robot X [m]:'), 1, 0)
        coordinate_layout.addWidget(self.robot_x_input, 1, 1)
        coordinate_layout.addWidget(QLabel('Y [m]:'), 1, 2)
        coordinate_layout.addWidget(self.robot_y_input, 1, 3)
        coordinate_layout.addWidget(QLabel('Yaw:'), 1, 4)
        coordinate_layout.addWidget(self.robot_yaw_input, 1, 5)
        coordinate_layout.addWidget(self.set_coordinate_robot_button, 1, 6)

        self.polygon_coordinate_input = QPlainTextEdit()
        self.polygon_coordinate_input.setPlaceholderText(
            'One map/world coordinate per line, for example:\n'
            '640150.000, 5193600.000\n'
            '640160.000, 5193600.000\n'
            '640160.000, 5193610.000'
        )
        self.polygon_coordinate_input.setMaximumHeight(90)
        self.polygon_value_input = QComboBox()
        self.polygon_value_input.addItem('Occupied / forbidden (100)', 100)
        self.polygon_value_input.addItem('Free (0)', 0)
        self.polygon_value_input.addItem('Unknown (-1)', -1)
        self.preview_coordinate_polygon_button = QPushButton(
            'Preview coordinate polygon'
        )
        self.preview_coordinate_polygon_button.clicked.connect(
            lambda: self._load_coordinate_polygon(apply_now=False)
        )
        self.apply_coordinate_polygon_button = QPushButton(
            'Apply coordinate polygon'
        )
        self.apply_coordinate_polygon_button.clicked.connect(
            lambda: self._load_coordinate_polygon(apply_now=True)
        )
        coordinate_layout.addWidget(
            QLabel('Polygon vertices X, Y [m]:'),
            2,
            0,
        )
        coordinate_layout.addWidget(
            self.polygon_coordinate_input,
            2,
            1,
            1,
            6,
        )
        coordinate_layout.addWidget(QLabel('Polygon cell value:'), 3, 0)
        coordinate_layout.addWidget(self.polygon_value_input, 3, 1, 1, 2)
        coordinate_layout.addWidget(
            self.preview_coordinate_polygon_button,
            3,
            3,
            1,
            2,
        )
        coordinate_layout.addWidget(
            self.apply_coordinate_polygon_button,
            3,
            5,
            1,
            2,
        )
        coordinate_group.setLayout(coordinate_layout)

        self.publish_map_button = QPushButton('Publish map')
        self.publish_map_button.clicked.connect(self.publish_map)
        self.publish_cones_button = QPushButton('Publish cones')
        self.publish_cones_button.clicked.connect(self.publish_cones)
        self.publish_robot_button = QPushButton('Publish robot start')
        self.publish_robot_button.clicked.connect(self.publish_robot_start)
        self.publish_all_button = QPushButton('Publish all')
        self.publish_all_button.clicked.connect(self.publish_all)

        publish_layout = QHBoxLayout()
        publish_layout.addWidget(self.publish_map_button)
        publish_layout.addWidget(self.publish_cones_button)
        publish_layout.addWidget(self.publish_robot_button)
        publish_layout.addWidget(self.publish_all_button)

        self.continuous_waypoints = QCheckBox(
            'Continuously publish cones at 1 Hz'
        )
        self.continuous_waypoints.setChecked(True)
        self.publish_static_tf = QCheckBox(
            'Publish static map -> base_link TF (test mode)'
        )
        self.publish_static_tf.setChecked(True)

        option_layout = QHBoxLayout()
        option_layout.addWidget(self.continuous_waypoints)
        option_layout.addWidget(self.publish_static_tf)
        option_layout.addStretch()

        layout = QVBoxLayout()
        layout.addLayout(mode_layout)
        layout.addLayout(spacing_layout)
        layout.addWidget(self.instructions)
        layout.addWidget(self.viewer)
        layout.addWidget(self.cursor_label)
        layout.addWidget(self.state_label)
        layout.addWidget(self.feedback_label)
        layout.addWidget(coordinate_group)
        layout.addLayout(clear_layout)
        layout.addLayout(publish_layout)
        layout.addLayout(option_layout)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.viewer.state_changed.connect(self._update_state_label)
        self.viewer.cursor_position_changed.connect(
            self.cursor_label.setText
        )
        self.viewer.feedback.connect(self.feedback_label.setText)

        self.waypoint_timer = QTimer(self)
        self.waypoint_timer.setInterval(1000)
        self.waypoint_timer.timeout.connect(
            lambda: self.publish_cones(silent=True)
        )
        self.continuous_waypoints.toggled.connect(
            self._set_continuous_waypoint_publish
        )
        self.waypoint_timer.start()

        self.resize(1400, 980)

    def set_node(self, node):
        self.node = node

    @staticmethod
    def _coordinate_spin_box():
        spin_box = QDoubleSpinBox()
        spin_box.setDecimals(3)
        spin_box.setRange(-1000000000.0, 1000000000.0)
        spin_box.setSingleStep(1.0)
        return spin_box

    def _add_coordinate_cone(self):
        self.viewer.add_cone_world(
            self.cone_x_input.value(),
            self.cone_y_input.value(),
        )

    def _set_coordinate_robot(self):
        self.viewer.set_robot_start_world(
            self.robot_x_input.value(),
            self.robot_y_input.value(),
            math.radians(self.robot_yaw_input.value()),
        )

    def _load_coordinate_polygon(self, apply_now):
        try:
            world_vertices = parse_polygon_coordinate_text(
                self.polygon_coordinate_input.toPlainText()
            )
        except ValueError as error:
            self.viewer.feedback.emit(
                f'Coordinate polygon rejected: {error}.'
            )
            return False

        if not self.polygon_mode.isChecked():
            self.polygon_mode.setChecked(True)

        return self.viewer.set_polygon_world(
            world_vertices,
            int(self.polygon_value_input.currentData()),
            apply_now=apply_now,
        )

    def _set_mode(self, mode, checked):
        if not checked:
            return

        self.viewer.set_mode(mode)
        instructions = {
            MapViewer.MODE_MAP: (
                'Cells: left=free, right=occupied, '
                'middle or Shift+left=unknown; Alt+left drag=pan.'
            ),
            MapViewer.MODE_RECTANGLE: (
                'Rectangle: drag left=free, right=occupied, '
                'middle or Shift+left=unknown; Alt+left drag=pan.'
            ),
            MapViewer.MODE_POLYGON: (
                'Polygon: first click selects the value '
                '(left=free, right=occupied, middle or Shift+left=unknown); '
                'then left-click vertices. Finish by clicking the first '
                'vertex, double-clicking, pressing Enter, or using Finish '
                'polygon. Right-click or Backspace=undo; Esc=cancel; '
                'Alt+left drag=pan.'
            ),
            MapViewer.MODE_CONES: (
                'Cones: left=add on a white cell, '
                'right=remove the nearest cone. Every cone must be at least '
                f'{MINIMUM_CONE_SPACING_M:.2f} m from the others; '
                'Alt+left drag=pan.'
            ),
            MapViewer.MODE_CONE_LINE: (
                'Cone line: left-click the start and end on white cells. '
                'Cones are placed uniformly using the selected maximum '
                f'spacing, but never closer than '
                f'{MINIMUM_CONE_SPACING_M:.2f} m; right-click or Esc=cancel; '
                'Alt+left drag=pan.'
            ),
            MapViewer.MODE_CONE_ARC: (
                'Cone arc: left-click the start, a point on the arc, and '
                'the end on white cells. Cones are placed uniformly using '
                'the selected maximum spacing, but never closer than '
                f'{MINIMUM_CONE_SPACING_M:.2f} m; right-click or Esc=cancel; '
                'Alt+left drag=pan.'
            ),
            MapViewer.MODE_ROBOT: (
                'Robot: left-click and drag from a white cell to set '
                'position and heading; right-click=clear; '
                'Alt+left drag=pan.'
            ),
        }
        self.instructions.setText(instructions[mode])

    def _set_continuous_waypoint_publish(self, enabled):
        if enabled:
            self.waypoint_timer.start()
            self.publish_cones(silent=True)
        else:
            self.waypoint_timer.stop()

    def _update_state_label(self):
        cone_count = len(self.viewer.cone_positions)
        polygon_vertex_count = len(self.viewer.polygon_vertices)
        cone_shape_point_count = len(self.viewer.cone_shape_points)
        if self.viewer.robot_start_pose is None:
            robot_text = 'not set'
        else:
            world_x, world_y, yaw = self.viewer.robot_start_pose
            robot_text = (
                f'X={world_x:.3f}, Y={world_y:.3f}, '
                f'yaw={math.degrees(yaw):.1f} deg'
            )

        self.state_label.setText(
            f'Cones: {cone_count} | Polygon vertices: '
            f'{polygon_vertex_count} | Line/arc control points: '
            f'{cone_shape_point_count} | Robot start: {robot_text}'
        )

    def publish_map(self):
        if self.node is None or self.viewer.grid is None:
            self.feedback_label.setText('No map has been received yet.')
            return False

        published = self.node.publish_edited_map(self.viewer.grid)
        if published:
            self.feedback_label.setText('Edited map published on /edited_map.')
        return published

    def publish_cones(self, silent=False):
        if self.node is None or self.viewer.grid is None:
            if not silent:
                self.feedback_label.setText('No map has been received yet.')
            return False

        if not self.viewer.cone_positions:
            if not silent:
                self.feedback_label.setText('Place at least one cone first.')
            return False

        invalid_indices = self.viewer.invalid_cone_indices()
        if invalid_indices:
            invalid_text = ', '.join(str(index + 1) for index in invalid_indices)
            if not silent:
                self.feedback_label.setText(
                    f'Cone(s) {invalid_text} are no longer on free cells.'
                )
            return False

        spacing_conflicts = self.viewer.cone_spacing_conflicts()
        if spacing_conflicts:
            first_index, second_index = spacing_conflicts[0]
            if not silent:
                self.feedback_label.setText(
                    f'Cones {first_index + 1} and {second_index + 1} are '
                    f'closer than the permitted '
                    f'{MINIMUM_CONE_SPACING_M:.3f} m.'
                )
            return False

        published = self.node.publish_waypoints(
            list(self.viewer.cone_positions),
            self.viewer.frame_id,
            log=not silent,
        )
        if published and not silent:
            repeat_text = (
                ' Continuous 1 Hz publishing is active.'
                if self.continuous_waypoints.isChecked()
                else ''
            )
            self.feedback_label.setText(
                f'{len(self.viewer.cone_positions)} cones published on '
                f'/waypoints.{repeat_text}'
            )
        return published

    def publish_robot_start(self):
        if self.node is None or self.viewer.grid is None:
            self.feedback_label.setText('No map has been received yet.')
            return False

        if self.viewer.robot_start_pose is None:
            self.feedback_label.setText('Set the robot start pose first.')
            return False

        if not self.viewer.robot_start_is_free():
            self.feedback_label.setText(
                'The robot start is no longer on a free cell.'
            )
            return False

        published = self.node.publish_robot_start(
            self.viewer.robot_start_pose,
            self.viewer.frame_id,
            self.publish_static_tf.isChecked(),
        )
        if published:
            destinations = '/initialpose'
            if self.publish_static_tf.isChecked():
                destinations += ' and map -> base_link TF'
            self.feedback_label.setText(
                f'Robot start published on {destinations}.'
            )
        return published

    def publish_all(self):
        map_ok = self.publish_map()
        robot_ok = self.publish_robot_start()
        cones_ok = self.publish_cones()

        if map_ok and cones_ok and robot_ok:
            self.feedback_label.setText(
                'Map, cone positions, and robot start were published.'
            )


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    window = MainWindow()
    node = MapEditorNode(window.viewer)
    window.set_node(node)
    window.publish_static_tf.setChecked(node.publish_static_tf_default)
    window.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
