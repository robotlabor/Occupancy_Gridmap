import math
import unittest

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


class TestMapGeometry(unittest.TestCase):
    def test_line_sampling_includes_endpoints_and_limits_spacing(self):
        points = sample_line_points((10.0, 20.0), (20.0, 20.0), 3.0)

        self.assertEqual(points[0], (10.0, 20.0))
        self.assertEqual(points[-1], (20.0, 20.0))
        distances = [
            math.dist(points[index], points[index + 1])
            for index in range(len(points) - 1)
        ]
        self.assertTrue(all(distance <= 3.0 for distance in distances))
        self.assertTrue(
            all(
                distance + 1e-9 >= MINIMUM_CONE_SPACING_M
                for distance in distances
            )
        )
        self.assertTrue(
            all(abs(distance - distances[0]) < 1e-12 for distance in distances)
        )

    def test_line_sampling_rejects_invalid_input(self):
        with self.assertRaises(ValueError):
            sample_line_points((0.0, 0.0), (0.0, 0.0), 1.0)
        with self.assertRaises(ValueError):
            sample_line_points((0.0, 0.0), (1.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            sample_line_points((0.0, 0.0), (1.0, 0.0), 0.499)
        with self.assertRaises(ValueError):
            sample_line_points((0.0, 0.0), (0.499, 0.0), 0.5)
        with self.assertRaises(ValueError):
            sample_line_points((0.0, 0.0), (20000.0, 0.0), 1.0)

    def test_line_sampling_never_places_cones_closer_than_half_a_metre(self):
        points = sample_line_points((0.0, 0.0), (1.49, 0.0), 0.5)

        self.assertEqual(len(points), 3)
        self.assertFalse(cone_spacing_conflicts(points))
        self.assertAlmostEqual(math.dist(points[0], points[1]), 0.745)

    def test_arc_sampling_uses_the_arc_through_the_middle_point(self):
        upper_points = sample_arc_points(
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0),
            0.5,
        )
        lower_points = sample_arc_points(
            (0.0, 0.0),
            (1.0, -1.0),
            (2.0, 0.0),
            0.5,
        )

        self.assertAlmostEqual(upper_points[0][0], 0.0)
        self.assertAlmostEqual(upper_points[-1][0], 2.0)
        self.assertTrue(all(point[1] >= -1e-12 for point in upper_points))
        self.assertTrue(all(point[1] <= 1e-12 for point in lower_points))
        self.assertGreater(max(point[1] for point in upper_points), 0.95)
        self.assertLess(min(point[1] for point in lower_points), -0.95)
        self.assertFalse(cone_spacing_conflicts(upper_points))
        self.assertFalse(cone_spacing_conflicts(lower_points))

    def test_arc_sampling_is_stable_with_utm_coordinates(self):
        points = sample_arc_points(
            (640000.0, 5193000.0),
            (640001.0, 5193001.0),
            (640002.0, 5193000.0),
            0.5,
        )

        self.assertAlmostEqual(points[0][0], 640000.0, places=7)
        self.assertAlmostEqual(points[0][1], 5193000.0, places=7)
        self.assertAlmostEqual(points[-1][0], 640002.0, places=7)
        self.assertAlmostEqual(points[-1][1], 5193000.0, places=7)

    def test_arc_sampling_rejects_collinear_points(self):
        with self.assertRaises(ValueError):
            sample_arc_points(
                (0.0, 0.0),
                (1.0, 0.0),
                (2.0, 0.0),
                0.5,
            )

    def test_arc_sampling_rejects_endpoints_closer_than_half_a_metre(self):
        with self.assertRaises(ValueError):
            sample_arc_points(
                (1.0, 0.0),
                (-1.0, 0.0),
                (math.cos(0.2), -math.sin(0.2)),
                0.5,
            )

    def test_cone_spacing_conflicts_supports_utm_coordinates(self):
        points = [
            (640000.0, 5193000.0),
            (640000.5, 5193000.0),
            (640001.0, 5193000.0),
            (640001.49, 5193000.0),
        ]

        self.assertEqual(cone_spacing_conflicts(points), [(2, 3)])

    def test_inclusive_rectangle_bounds_supports_every_drag_direction(self):
        expected = (2, 3, 8, 10)

        self.assertEqual(inclusive_rectangle_bounds((2, 3), (8, 10)), expected)
        self.assertEqual(inclusive_rectangle_bounds((8, 3), (2, 10)), expected)
        self.assertEqual(inclusive_rectangle_bounds((2, 10), (8, 3)), expected)
        self.assertEqual(inclusive_rectangle_bounds((8, 10), (2, 3)), expected)

    def test_single_cell_rectangle_is_inclusive(self):
        self.assertEqual(
            inclusive_rectangle_bounds((4, 7), (4, 7)),
            (4, 7, 4, 7),
        )

    def test_polygon_fill_region_includes_square_boundary_and_interior(self):
        left, top, right, bottom, mask = polygon_fill_region(
            [(1, 1), (4, 1), (4, 4), (1, 4)],
            width=10,
            height=10,
        )

        self.assertEqual((left, top, right, bottom), (1, 1, 4, 4))
        self.assertEqual(mask.shape, (4, 4))
        self.assertTrue(mask.all())

    def test_polygon_fill_region_supports_concave_shapes(self):
        _, _, _, _, mask = polygon_fill_region(
            [(1, 1), (4, 1), (4, 2), (2, 2), (2, 4), (1, 4)],
            width=10,
            height=10,
        )

        self.assertEqual(int(mask.sum()), 12)
        self.assertFalse(bool(mask[3, 3]))

    def test_polygon_fill_region_is_independent_of_vertex_direction(self):
        vertices = [(2, 2), (6, 2), (4, 6)]
        forward = polygon_fill_region(vertices, 10, 10)
        backward = polygon_fill_region(list(reversed(vertices)), 10, 10)

        self.assertEqual(forward[:4], backward[:4])
        self.assertTrue((forward[4] == backward[4]).all())

    def test_polygon_fill_region_rejects_invalid_shapes(self):
        with self.assertRaises(ValueError):
            polygon_fill_region([(1, 1), (2, 2)], 10, 10)
        with self.assertRaises(ValueError):
            polygon_fill_region([(1, 1), (2, 2), (3, 3)], 10, 10)

    def test_coordinate_polygon_parser_accepts_common_separators(self):
        vertices = parse_polygon_coordinate_text(
            '640000.0, 5193000.0\n'
            '640010.0 5193000.0\n'
            '640010.0;5193010.0\n'
            '\n'
        )

        self.assertEqual(
            vertices,
            [
                (640000.0, 5193000.0),
                (640010.0, 5193000.0),
                (640010.0, 5193010.0),
            ],
        )

    def test_coordinate_polygon_parser_rejects_invalid_input(self):
        with self.assertRaisesRegex(ValueError, 'line 2'):
            parse_polygon_coordinate_text('1, 2\n3, invalid\n4, 5')
        with self.assertRaises(ValueError):
            parse_polygon_coordinate_text('1, 2\n3, 4')
        with self.assertRaises(ValueError):
            parse_polygon_coordinate_text('1, 2\n3, 4\nnan, 5')

    def test_display_cell_round_trip_without_rotation(self):
        geometry = MapGeometry(
            resolution=0.3,
            width=1198,
            height=674,
            origin_x=639976.202,
            origin_y=5193512.087,
        )

        world_x, world_y = geometry.display_cell_to_world(579, 326)

        self.assertAlmostEqual(world_x, 640150.052)
        self.assertAlmostEqual(world_y, 5193616.337)
        self.assertEqual(
            geometry.world_to_display_cell(world_x, world_y),
            (579, 326),
        )

    def test_rotated_map_round_trip(self):
        geometry = MapGeometry(
            resolution=1.0,
            width=20,
            height=10,
            origin_x=100.0,
            origin_y=200.0,
            origin_yaw=math.pi / 2.0,
        )

        world_position = geometry.display_cell_to_world(3, 7)

        self.assertEqual(
            geometry.world_to_display_cell(*world_position),
            (3, 7),
        )

    def test_world_polygon_vertices_convert_on_utm_map(self):
        geometry = MapGeometry(
            resolution=0.5,
            width=20,
            height=20,
            origin_x=640000.0,
            origin_y=5193000.0,
        )
        world_vertices = [
            geometry.display_cell_to_world(2, 3),
            geometry.display_cell_to_world(12, 3),
            geometry.display_cell_to_world(12, 13),
            geometry.display_cell_to_world(2, 13),
        ]

        self.assertEqual(
            geometry.world_polygon_to_display_cells(world_vertices),
            [(2, 3), (12, 3), (12, 13), (2, 13)],
        )

    def test_world_polygon_conversion_rejects_invalid_vertices(self):
        geometry = MapGeometry(
            resolution=1.0,
            width=10,
            height=10,
            origin_x=0.0,
            origin_y=0.0,
        )

        with self.assertRaisesRegex(ValueError, 'outside the map'):
            geometry.world_polygon_to_display_cells(
                [(1.0, 1.0), (2.0, 2.0), (20.0, 20.0)]
            )
        with self.assertRaisesRegex(ValueError, 'distinct map cells'):
            geometry.world_polygon_to_display_cells(
                [(1.1, 1.1), (1.2, 1.2), (2.1, 2.1)]
            )

    def test_quaternion_to_yaw(self):
        yaw = math.radians(70.0)

        result = quaternion_to_yaw(
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

        self.assertAlmostEqual(result, yaw)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            MapGeometry(
                resolution=0.0,
                width=10,
                height=10,
                origin_x=0.0,
                origin_y=0.0,
            )


if __name__ == '__main__':
    unittest.main()
