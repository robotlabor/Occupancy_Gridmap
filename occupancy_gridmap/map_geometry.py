#!/usr/bin/env python3

import math
from dataclasses import dataclass

import numpy as np


MINIMUM_CONE_SPACING_M = 0.5
_CONE_SPACING_TOLERANCE_M = 1e-9


def parse_polygon_coordinate_text(text: str) -> list[tuple[float, float]]:
    """Parse one world-coordinate polygon vertex from every non-empty line."""
    vertices = []
    for line_number, raw_line in enumerate(str(text).splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue

        fields = line.replace(',', ' ').replace(';', ' ').split()
        if len(fields) != 2:
            raise ValueError(
                f'line {line_number} must contain exactly one X and one Y '
                'value'
            )

        try:
            world_x = float(fields[0])
            world_y = float(fields[1])
        except ValueError as error:
            raise ValueError(
                f'line {line_number} contains a non-numeric coordinate'
            ) from error

        if not math.isfinite(world_x) or not math.isfinite(world_y):
            raise ValueError(
                f'line {line_number} coordinates must be finite numbers'
            )
        vertices.append((world_x, world_y))

    if len(set(vertices)) < 3:
        raise ValueError(
            'a coordinate polygon needs at least three distinct vertices'
        )
    return vertices


def _validate_curve_sampling(spacing: float, point_count: int) -> None:
    if not math.isfinite(spacing) or spacing < MINIMUM_CONE_SPACING_M:
        raise ValueError(
            'cone spacing must be at least '
            f'{MINIMUM_CONE_SPACING_M:.3f} m'
        )
    if point_count > 10000:
        raise ValueError(
            'the requested spacing would create more than 10000 cones'
        )


def _line_segment_count(length: float, spacing: float) -> int:
    if length + _CONE_SPACING_TOLERANCE_M < MINIMUM_CONE_SPACING_M:
        raise ValueError(
            'the line or arc is shorter than the minimum cone spacing of '
            f'{MINIMUM_CONE_SPACING_M:.3f} m'
        )

    requested_count = max(1, math.ceil(length / spacing))
    maximum_count = max(
        1,
        math.floor(
            (length + _CONE_SPACING_TOLERANCE_M)
            / MINIMUM_CONE_SPACING_M
        ),
    )
    return min(requested_count, maximum_count)


def cone_spacing_conflicts(
    points: list[tuple[float, float]],
    minimum_spacing: float = MINIMUM_CONE_SPACING_M,
) -> list[tuple[int, int]]:
    """Return index pairs whose Euclidean distance is below the limit."""
    if not math.isfinite(minimum_spacing) or minimum_spacing <= 0.0:
        raise ValueError('minimum spacing must be finite and greater than zero')

    buckets = {}
    conflicts = []
    for point_index, point in enumerate(points):
        point_x = float(point[0])
        point_y = float(point[1])
        if not math.isfinite(point_x) or not math.isfinite(point_y):
            raise ValueError('cone coordinates must be finite numbers')

        bucket_x = math.floor(point_x / minimum_spacing)
        bucket_y = math.floor(point_y / minimum_spacing)
        for neighbour_x in range(bucket_x - 1, bucket_x + 2):
            for neighbour_y in range(bucket_y - 1, bucket_y + 2):
                for other_index in buckets.get(
                    (neighbour_x, neighbour_y),
                    [],
                ):
                    if (
                        math.dist(points[other_index], (point_x, point_y))
                        + _CONE_SPACING_TOLERANCE_M
                        < minimum_spacing
                    ):
                        conflicts.append((other_index, point_index))

        buckets.setdefault((bucket_x, bucket_y), []).append(point_index)

    return conflicts


def sample_line_points(
    start: tuple[float, float],
    end: tuple[float, float],
    spacing: float,
) -> list[tuple[float, float]]:
    """Sample a line uniformly, including both endpoints."""
    _validate_curve_sampling(spacing, 0)

    delta_x = float(end[0]) - float(start[0])
    delta_y = float(end[1]) - float(start[1])
    length = math.hypot(delta_x, delta_y)
    if length <= 1e-9:
        raise ValueError('line start and end must be different')

    segment_count = _line_segment_count(length, spacing)
    _validate_curve_sampling(spacing, segment_count + 1)
    return [
        (
            float(start[0]) + delta_x * index / segment_count,
            float(start[1]) + delta_y * index / segment_count,
        )
        for index in range(segment_count + 1)
    ]


def sample_arc_points(
    start: tuple[float, float],
    through: tuple[float, float],
    end: tuple[float, float],
    spacing: float,
) -> list[tuple[float, float]]:
    """Sample the circular arc from start to end that passes through a point."""
    _validate_curve_sampling(spacing, 0)

    start_x = float(start[0])
    start_y = float(start[1])
    through_x = float(through[0]) - start_x
    through_y = float(through[1]) - start_y
    end_x = float(end[0]) - start_x
    end_y = float(end[1]) - start_y

    through_length_squared = through_x ** 2 + through_y ** 2
    end_length_squared = end_x ** 2 + end_y ** 2
    scale_squared = max(through_length_squared, end_length_squared, 1.0)
    cross_product = through_x * end_y - through_y * end_x
    if abs(cross_product) <= 1e-10 * scale_squared:
        raise ValueError('the three arc points must not be collinear')
    if end_length_squared <= 1e-18:
        raise ValueError('arc start and end must be different')

    centre_x_local = (
        through_length_squared * end_y
        - through_y * end_length_squared
    ) / (2.0 * cross_product)
    centre_y_local = (
        through_x * end_length_squared
        - through_length_squared * end_x
    ) / (2.0 * cross_product)
    centre_x = start_x + centre_x_local
    centre_y = start_y + centre_y_local
    radius = math.hypot(centre_x_local, centre_y_local)
    if radius <= 1e-9 or not math.isfinite(radius):
        raise ValueError('the arc radius is invalid')

    start_angle = math.atan2(start_y - centre_y, start_x - centre_x)
    through_angle = math.atan2(
        float(through[1]) - centre_y,
        float(through[0]) - centre_x,
    )
    end_angle = math.atan2(float(end[1]) - centre_y, float(end[0]) - centre_x)

    full_turn = 2.0 * math.pi
    counterclockwise_to_through = (
        through_angle - start_angle
    ) % full_turn
    counterclockwise_to_end = (end_angle - start_angle) % full_turn
    if counterclockwise_to_through <= counterclockwise_to_end + 1e-10:
        sweep = counterclockwise_to_end
    else:
        sweep = -((start_angle - end_angle) % full_turn)

    arc_length = radius * abs(sweep)
    if arc_length <= 1e-9 or not math.isfinite(arc_length):
        raise ValueError('the arc length is invalid')

    endpoint_distance = math.dist(start, end)
    if (
        endpoint_distance + _CONE_SPACING_TOLERANCE_M
        < MINIMUM_CONE_SPACING_M
    ):
        raise ValueError(
            'the arc endpoints are closer than the minimum cone spacing of '
            f'{MINIMUM_CONE_SPACING_M:.3f} m'
        )

    segment_count = _line_segment_count(arc_length, spacing)
    minimum_step_angle = 2.0 * math.asin(
        min(1.0, MINIMUM_CONE_SPACING_M / (2.0 * radius))
    )
    maximum_segment_count = max(
        1,
        math.floor(
            (abs(sweep) + 1e-12) / minimum_step_angle
        ),
    )
    segment_count = min(segment_count, maximum_segment_count)
    _validate_curve_sampling(spacing, segment_count + 1)
    points = [
        (
            centre_x + radius * math.cos(
                start_angle + sweep * index / segment_count
            ),
            centre_y + radius * math.sin(
                start_angle + sweep * index / segment_count
            ),
        )
        for index in range(segment_count + 1)
    ]
    if cone_spacing_conflicts(points):
        raise ValueError(
            'the arc geometry would place cones closer than '
            f'{MINIMUM_CONE_SPACING_M:.3f} m'
        )
    return points


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw angle of a quaternion without extra dependencies."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def inclusive_rectangle_bounds(
    start_cell: tuple[int, int],
    end_cell: tuple[int, int],
) -> tuple[int, int, int, int]:
    """Return inclusive left, top, right, and bottom rectangle bounds."""
    start_column, start_row = start_cell
    end_column, end_row = end_cell
    return (
        min(start_column, end_column),
        min(start_row, end_row),
        max(start_column, end_column),
        max(start_row, end_row),
    )


def polygon_fill_region(
    vertices: list[tuple[int, int]],
    width: int,
    height: int,
) -> tuple[int, int, int, int, np.ndarray]:
    """Return clipped polygon bounds and a mask for enclosed display cells."""
    if width <= 0 or height <= 0:
        raise ValueError('map width and height must be greater than zero')

    normalised_vertices = []
    for vertex in vertices:
        point = (int(vertex[0]), int(vertex[1]))
        if not normalised_vertices or point != normalised_vertices[-1]:
            normalised_vertices.append(point)

    if (
        len(normalised_vertices) > 1
        and normalised_vertices[0] == normalised_vertices[-1]
    ):
        normalised_vertices.pop()

    if len(set(normalised_vertices)) < 3:
        raise ValueError('a polygon needs at least three distinct vertices')

    signed_double_area = 0
    for index, (x_1, y_1) in enumerate(normalised_vertices):
        x_2, y_2 = normalised_vertices[
            (index + 1) % len(normalised_vertices)
        ]
        signed_double_area += x_1 * y_2 - x_2 * y_1

    if signed_double_area == 0:
        raise ValueError('polygon vertices must not be collinear')

    left = max(min(point[0] for point in normalised_vertices), 0)
    top = max(min(point[1] for point in normalised_vertices), 0)
    right = min(max(point[0] for point in normalised_vertices), width - 1)
    bottom = min(max(point[1] for point in normalised_vertices), height - 1)

    if left > right or top > bottom:
        raise ValueError('polygon is outside the map')

    columns, rows = np.meshgrid(
        np.arange(left, right + 1, dtype=np.float64),
        np.arange(top, bottom + 1, dtype=np.float64),
    )
    inside = np.zeros(columns.shape, dtype=bool)
    boundary = np.zeros(columns.shape, dtype=bool)

    for index, (x_1, y_1) in enumerate(normalised_vertices):
        x_2, y_2 = normalised_vertices[
            (index + 1) % len(normalised_vertices)
        ]
        delta_x = x_2 - x_1
        delta_y = y_2 - y_1

        crosses_row = (y_1 > rows) != (y_2 > rows)
        if delta_y != 0:
            crossing_x = x_1 + (rows - y_1) * delta_x / delta_y
            inside ^= crosses_row & (columns < crossing_x)

        cross_product = (
            (columns - x_1) * delta_y
            - (rows - y_1) * delta_x
        )
        within_edge = (
            (columns >= min(x_1, x_2))
            & (columns <= max(x_1, x_2))
            & (rows >= min(y_1, y_2))
            & (rows <= max(y_1, y_2))
        )
        boundary |= np.isclose(cross_product, 0.0) & within_edge

    return left, top, right, bottom, inside | boundary


@dataclass(frozen=True)
class MapGeometry:
    """Convert between GUI pixels and OccupancyGrid world coordinates."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def __post_init__(self):
        if self.resolution <= 0.0:
            raise ValueError('resolution must be greater than zero')
        if self.width <= 0 or self.height <= 0:
            raise ValueError('map width and height must be greater than zero')

    def display_to_world(
        self,
        display_x: float,
        display_y: float,
    ) -> tuple[float, float]:
        """Convert a continuous GUI position to world coordinates."""
        local_x = display_x * self.resolution
        local_y = (self.height - display_y) * self.resolution

        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)

        world_x = self.origin_x + cos_yaw * local_x - sin_yaw * local_y
        world_y = self.origin_y + sin_yaw * local_x + cos_yaw * local_y
        return world_x, world_y

    def display_cell_to_world(
        self,
        column: int,
        display_row: int,
    ) -> tuple[float, float]:
        """Return the world position at the centre of a displayed grid cell."""
        if not self.contains_display_cell(column, display_row):
            raise ValueError('display cell is outside the map')
        return self.display_to_world(column + 0.5, display_row + 0.5)

    def world_to_display(
        self,
        world_x: float,
        world_y: float,
    ) -> tuple[float, float]:
        """Convert a world coordinate to a continuous GUI position."""
        delta_x = world_x - self.origin_x
        delta_y = world_y - self.origin_y

        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)

        local_x = cos_yaw * delta_x + sin_yaw * delta_y
        local_y = -sin_yaw * delta_x + cos_yaw * delta_y

        display_x = local_x / self.resolution
        display_y = self.height - local_y / self.resolution
        return display_x, display_y

    def world_to_display_cell(
        self,
        world_x: float,
        world_y: float,
    ) -> tuple[int, int] | None:
        """Return the displayed grid cell containing a world coordinate."""
        display_x, display_y = self.world_to_display(world_x, world_y)
        column = math.floor(display_x)
        display_row = math.floor(display_y)

        if display_x == self.width:
            column = self.width - 1
        if display_y == self.height:
            display_row = self.height - 1

        if not self.contains_display_cell(column, display_row):
            return None
        return column, display_row

    def world_polygon_to_display_cells(
        self,
        world_vertices: list[tuple[float, float]],
    ) -> list[tuple[int, int]]:
        """Convert world-coordinate polygon vertices to displayed map cells."""
        display_vertices = []
        for vertex_index, (world_x, world_y) in enumerate(
            world_vertices,
            start=1,
        ):
            if not math.isfinite(world_x) or not math.isfinite(world_y):
                raise ValueError(
                    f'polygon vertex {vertex_index} must contain finite values'
                )

            display_cell = self.world_to_display_cell(world_x, world_y)
            if display_cell is None:
                raise ValueError(
                    f'polygon vertex {vertex_index} is outside the map'
                )
            if not display_vertices or display_cell != display_vertices[-1]:
                display_vertices.append(display_cell)

        if (
            len(display_vertices) > 1
            and display_vertices[0] == display_vertices[-1]
        ):
            display_vertices.pop()

        if len(set(display_vertices)) < 3:
            raise ValueError(
                'the coordinates must map to at least three distinct map cells'
            )
        return display_vertices

    def contains_display_cell(self, column: int, display_row: int) -> bool:
        return (
            0 <= column < self.width
            and 0 <= display_row < self.height
        )
