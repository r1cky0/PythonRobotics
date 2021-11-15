import unittest
from visibility_graph_and_trapezoidal_decomposition import Point, Segment, have_common_point, check_segment_vertical_intersection, \
    check_segment_intersection, is_segment_intersection, check_line_segment_vertical_intersection, \
    check_vertical_line_segment_intersection, check_line_segment_intersection, Polygon, Map, straight_line_segment_intersection_point, segment_segment_intersection_point


class TestVisibilityGraph(unittest.TestCase):
    def test_Point_get_coordinate(self):
        self.assertEqual(Point(1, 2).get_coordinate(), (1, 2))

    def test_Point_get_distance(self):
        self.assertEqual(Point(3, 3).get_distance(Point(6, 7)), 5)
        self.assertEqual(Point(6, 7).get_distance(Point(3, 3)), 5)
        self.assertEqual(Point(3, 3).get_distance(Point(3, 3)), 0)
        self.assertEqual(Point(6, 7).get_distance(Point(6, 7)), 0)

    def test_Point_add_adjacent(self):
        point1 = Point(3, 3)
        point2 = Point(6, 7)
        point1.add_adjacent(point2)
        point2.add_adjacent(point1)
        self.assertEqual(len(point1.adjacent), 1)
        self.assertEqual(point2.adjacent[0][0].get_coordinate(), (3, 3))
        self.assertEqual(point2.adjacent[0][1], 5)

    def test_Point_set_goal_distance(self):
        point1 = Point(4, 4)
        point2 = Point(4, 10)
        point1.set_goal_distance(point2)
        point2.set_goal_distance(point2)
        self.assertEqual(point1.goal_distance, 6)
        self.assertEqual(point2.goal_distance, 0)

    def test_Segment_isVertical_and_get_points(self):
        point1 = Point(1, 1)
        point2 = Point(1, 3)
        point3 = Point(4, 2)
        segment1 = Segment(point1, point2)
        segment2 = Segment(point2, point1)
        segment3 = Segment(point2, point3)
        segment4 = Segment(point3, point2)
        self.assertTrue(segment1.isVertical)
        self.assertTrue(segment2.isVertical)
        self.assertFalse(segment3.isVertical)
        self.assertFalse(segment4.isVertical)
        self.assertEqual(segment2.get_points(), ((1, 1), (1, 3)))
        self.assertEqual(segment1.get_points(), segment2.get_points())
        self.assertEqual(segment4.get_points(), ((1, 3), (4, 2)))
        self.assertEqual(segment3.get_points(), segment4.get_points())

    def test_Segment_line_point_position(self):
        point1 = Point(1, 1)
        point2 = Point(1, 3)
        point3 = Point(4, 4)
        segment1 = Segment(point1, point2)
        segment2 = Segment(point1, point3)
        self.assertEqual(segment1.line_point_position(point1), 0)
        self.assertEqual(segment2.line_point_position(point1), 0)
        self.assertEqual(segment1.line_point_position(Point(0, 5)), -1)
        self.assertEqual(segment1.line_point_position(Point(1, 2)), 0)
        self.assertEqual(segment1.line_point_position(point3), 1)
        self.assertEqual(segment2.line_point_position(Point(4, 5)), 1)
        self.assertEqual(segment2.line_point_position(Point(8, 8)), 0)
        self.assertEqual(segment2.line_point_position(Point(7, 2)), -1)

    def test_Segment_is_point_contained_in_segment(self):
        segment1 = Segment(Point(1, 1), Point(1, 3))
        segment2 = Segment(Point(1, 1), Point(3, 3))
        self.assertFalse(segment1.is_point_contained_in_segment(Point(1, 1)))
        self.assertTrue(segment1.is_point_contained_in_segment(Point(1, 2)))
        self.assertFalse(segment1.is_point_contained_in_segment(Point(2, 2)))
        self.assertTrue(segment2.is_point_contained_in_segment(Point(2, 2)))
        self.assertFalse(segment2.is_point_contained_in_segment(Point(3, 3)))
        self.assertFalse(segment2.is_point_contained_in_segment(Point(1, 2)))

    def test_have_common_point(self):
        point1 = Point(1, 1)
        point2 = Point(1, 3)
        point3 = Point(4, 4)
        point4 = Point(5, 5)
        segment1 = Segment(point1, point2)
        segment2 = Segment(point1, point3)
        segment3 = Segment(point3, point1)
        segment4 = Segment(point2, point3)
        segment5 = Segment(point1, point4)
        self.assertTrue(have_common_point(segment1, segment2))
        self.assertTrue(have_common_point(segment1, segment3))
        self.assertTrue(have_common_point(segment3, segment4))
        self.assertTrue(have_common_point(segment2, segment4))
        self.assertFalse(have_common_point(segment5, segment4))

    def test_check_segment_vertical_intersection(self):
        point1 = Point(1, 1)
        point2 = Point(1, 3)
        point3 = Point(0, 2)
        point4 = Point(2, 2)
        point5 = Point(2, 3)
        point6 = Point(5, 6)
        segment1 = Segment(point1, point2)
        segment2 = Segment(point3, point4)
        segment3 = Segment(point5, point6)
        self.assertTrue(check_segment_vertical_intersection(segment2, segment1))
        self.assertFalse(check_segment_vertical_intersection(segment3, segment1))

    def test_check_segment_intersection(self):
        point1 = Point(1, 1)
        point2 = Point(3, 3)
        point3 = Point(3, 1)
        point4 = Point(1, 3)
        point5 = Point(8, 2)
        segment1 = Segment(point1, point2)
        segment2 = Segment(point3, point4)
        segment3 = Segment(point3, point5)
        self.assertTrue(check_segment_intersection(segment1, segment2))
        self.assertFalse(check_segment_intersection(segment1, segment3))

    def test_is_segment_intersection(self):
        point1 = Point(1, 1)
        point2 = Point(1, 3)
        point3 = Point(4, 4)
        point4 = Point(2, 1)
        point5 = Point(2, 5)
        point6 = Point(2, 2)
        point7 = Point(8, 4)
        point8 = Point(5, 3)
        point9 = Point(7, 6)
        point10 = Point(9, 3)
        point11 = Point(10, 4)
        segment1 = Segment(point1, point2)
        segment2 = Segment(point1, point3)
        segment3 = Segment(point4, point5)
        segment4 = Segment(point1, point6)
        segment5 = Segment(point3, point7)
        segment6 = Segment(point8, point9)
        segment7 = Segment(point10, point11)
        self.assertFalse(is_segment_intersection(segment1, segment2))
        self.assertFalse(is_segment_intersection(segment1, segment3))
        self.assertFalse(is_segment_intersection(segment2, segment4))
        self.assertTrue(is_segment_intersection(segment5, segment6))
        self.assertFalse(is_segment_intersection(segment5, segment7))

    def test_line_segment_vertical_intersection(self):
        self.assertTrue(check_line_segment_vertical_intersection(Segment(Point(-1, 2), Point(0, 2)),
                                                                 Segment(Point(1, 1), Point(1, 3))))
        self.assertFalse(check_line_segment_vertical_intersection(Segment(Point(-1, 4), Point(0, 4)),
                                                                  Segment(Point(1, 1), Point(1, 3))))
        self.assertTrue(check_line_segment_vertical_intersection(Segment(Point(1, 1), Point(2, 2)),
                                                                 Segment(Point(4, 3), Point(4, 5))))
        self.assertFalse(check_line_segment_vertical_intersection(Segment(Point(1, 1), Point(2, 2)),
                                                                  Segment(Point(4, 1), Point(4, 2))))

    def test_check_vertical_line_segment_intersection(self):
        self.assertTrue(check_vertical_line_segment_intersection(Segment(Point(1, 1), Point(1, 3)),
                                                                 Segment(Point(0, 0), Point(3, 0))))
        self.assertFalse(check_vertical_line_segment_intersection(Segment(Point(1, 1), Point(1, 3)),
                                                                  Segment(Point(1, 0), Point(4, 0))))
        self.assertFalse(check_vertical_line_segment_intersection(Segment(Point(1, 1), Point(1, 3)),
                                                                  Segment(Point(2, 0), Point(4, 0))))
        self.assertTrue(check_vertical_line_segment_intersection(Segment(Point(1, 1), Point(1, 3)),
                                                                 Segment(Point(0, 3), Point(5, 16))))
        self.assertFalse(check_vertical_line_segment_intersection(Segment(Point(-1, 2), Point(-1, 4)),
                                                                  Segment(Point(0, 3), Point(5, 16))))

    def test_check_line_segment_intersection(self):
        self.assertTrue(check_line_segment_intersection(Segment(Point(1, 1), Point(3, 3)),
                                                        Segment(Point(4, 3), Point(2, 8))))
        self.assertFalse(check_line_segment_intersection(Segment(Point(1, 1), Point(3, 3)),
                                                         Segment(Point(5, 0), Point(6, 2))))

    def test_Polygon_contains_vertex(self):
        po1 = Polygon([Point(1, 1), Point(5, 1), Point(5, 5), Point(1, 5)])
        self.assertTrue(po1.contains_vertex(Point(1, 5)))
        self.assertFalse(po1.contains_vertex(Point(2, 5)))

    def test_Polygon_contains_segment(self):
        po1 = Polygon([Point(1, 1), Point(5, 1), Point(5, 5), Point(1, 5)])
        self.assertTrue(po1.contains_segment(Segment(Point(1, 1), Point(1, 5))))
        self.assertTrue(po1.contains_segment(Segment(Point(1, 5), Point(1, 1))))
        self.assertFalse(po1.contains_segment(Segment(Point(1, 1), Point(5, 5))))

    def test_Polygon_line_intersection(self):
        po1 = Polygon([Point(1, 1), Point(5, 1), Point(5, 5), Point(1, 5)])
        self.assertTrue(po1.line_intersection(Segment(Point(6, 2), Point(8, 2))))
        self.assertFalse(po1.line_intersection(Segment(Point(9, 5), Point(7, 5))))

    def test_Polygon_segment_intersection(self):
        po1 = Polygon([Point(1, 1), Point(5, 1), Point(5, 5), Point(1, 5)])
        self.assertFalse(po1.segment_intersection(Segment(Point(6, 2), Point(8, 2))))
        self.assertFalse(po1.segment_intersection(Segment(Point(9, 5), Point(7, 5))))
        self.assertTrue(po1.line_intersection(Segment(Point(2, 2), Point(8, 2))))

    def test_Polygon_segment_contains_in_no_vertex(self):
        po1 = Polygon([Point(1, 1), Point(5, 1), Point(5, 5), Point(1, 5)])
        self.assertFalse(po1.segment_contains_in_no_vertex(Segment(Point(2, 1), Point(7, 1))))
        self.assertTrue(po1.segment_contains_in_no_vertex(Segment(Point(5, 1), Point(7, 1))))
        self.assertTrue(po1.segment_contains_in_no_vertex(Segment(Point(6, 1), Point(7, 1))))
        self.assertFalse(po1.segment_contains_in_no_vertex(Segment(Point(0, 1), Point(7, 1))))

    def test_Map_contains_segment(self):
        map1 = Map("default")
        self.assertTrue(map1.contains_segment(Segment(Point(18, 21), Point(14, 16))))
        self.assertFalse(map1.contains_segment(Segment(Point(5, 11), Point(28, 21))))

    def test_Map_contains_vertex(self):
        map1 = Map("default")
        self.assertTrue(map1.contains_vertex(Point(4, 5)))
        self.assertFalse(map1.contains_vertex(Point(1, 1)))

    def test_Map_segment_is_diagonal(self):
        map1 = Map("default")
        self.assertTrue(map1.segment_is_diagonal(Segment(Point(18, 21), Point(15, 12))))
        self.assertFalse(map1.segment_is_diagonal(Segment(Point(9, 14), Point(18, 21))))

    def test_Map_is_reduced_map_intersection(self):
        map1 = Map("default")
        self.assertTrue(map1.is_reduced_map_intersection(Segment(Point(9, 26), Point(14, 16))))
        self.assertFalse(map1.is_reduced_map_intersection(Segment(Point(18, 21), Point(9, 26))))

    def test_Map_is_complete_map_intersection(self):
        map1 = Map("default")
        self.assertFalse(map1.is_complete_map_intersection(Segment(Point(9, 26), Point(14, 16))))
        self.assertTrue(map1.is_complete_map_intersection(Segment(Point(9, 26), Point(28, 2))))
        self.assertFalse(map1.is_complete_map_intersection(Segment(Point(27, 5), Point(28, 2))))


if __name__ == '__main__':
    unittest.main()
