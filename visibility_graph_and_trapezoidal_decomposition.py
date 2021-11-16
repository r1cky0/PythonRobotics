from math import sqrt, pow
import json
import matplotlib.pyplot as plt
import numpy as np

# is possible to do the same thing for trapezoidal decomposition
standard_map_visibility = """
{
  "relevant_points": {
    "start": [3, 28],
    "goal": [28, 2]
  } ,
  "polygons": {
    "p1": [[5, 25], [9, 26], [10, 22], [8, 19], [7, 23]],
    "p2": [[18, 21], [19, 15], [15, 12], [9, 14], [14, 16]],
    "p3": [[22, 25], [24, 27], [28, 21], [25, 17], [21, 20]],
    "p4": [[5, 11], [10, 8], [8, 2], [4, 5], [6, 7]],
    "p5": [[21, 8], [22, 12], [26, 11], [27, 5], [23, 3]]
  }
}
"""


class Point:

    def __init__(self, coordinate_x, coordinate_y):
        self.x = coordinate_x
        self.y = coordinate_y
        self.adjacent = []  # (point, distance)
        self.goal_distance = 0

    def get_coordinate(self):
        return self.x, self.y

    def get_distance(self, point):
        return sqrt(pow(point.y - self.y, 2) + pow(point.x - self.x, 2))

    def add_adjacent(self, point):
        self.adjacent.append((point, self.get_distance(point)))

    def set_goal_distance(self, goal):
        self.goal_distance = self.get_distance(goal)

    def is_equal(self, point):
        if np.isclose(point.x, self.x, atol=1e-08, equal_nan=False) and\
                np.isclose(point.y, self.y, atol=1e-08, equal_nan=False):
            return True
        else:
            return False


class Area:

    def __init__(self, segments):
        self.segments = segments  # lista di segmenti
        self.adjacent = []  # (area, distance)
        self.goal_distance = 0
        cg = self.init_center()
        self.x = cg[0]
        self.y = cg[1]

    def get_coordinate(self):
        return self.x, self.y

    def get_distance(self, area):
        return sqrt(pow(area.y - self.y, 2) + pow(area.x - self.x, 2))

    def add_adjacent(self, area):
        self.adjacent.append((area, self.get_distance(area)))

    def set_goal_distance(self, goal_area):
        self.goal_distance = self.get_distance(goal_area)

    def set_center(self, relevant_point):
        self.x = relevant_point.x
        self.y = relevant_point.y

    def init_center(self):
        if len(self.segments) == 1:
            mp = self.segments[0].get_medium_point()
            x = mp.x
            y = mp.y
            return x, y
        if len(self.segments) == 3:
            median = []
            for s in self.segments:
                me = [s.get_medium_point()]
                for s1 in self.segments:
                    if not s.is_equal_seg(s1):
                        if not s.has_extreme(s1.point1):
                            me.append(s1.point1)
                        if not s.has_extreme(s1.point2):
                            me.append(s1.point2)
                median.append(Segment(me[0], me[1]))
            cg = segment_segment_intersection_point(median[0], median[1])[1]
            x = cg.x
            y = cg.y
            return x, y
        if len(self.segments) == 4:  # baricentro di un trapezio (approximated)
            m = self.get_medians_trz()
            cg = segment_segment_intersection_point(m[0], m[1])[1]
            x = cg.x
            y = cg.y
            return x, y

    def get_medians_trz(self):
        op1 = []
        op2 = []
        op1.append(self.segments[0])
        for s in self.segments:
            if not s.is_equal_seg(op1[0]):
                if s.m == op1[0].m or s.m != "vertical" and np.isclose(s.m, op1[0].m, atol=1e-08, equal_nan=False):
                    op1.append(s)
                else:
                    op2.append(s)
        if len(op1) != 2 and len(op2) != 2:
            if op2[0].m == op2[1].m or op2[0].m != "vertical" and\
                    np.isclose(op2[0].m, op2[1].m, atol=1e-08, equal_nan=False):
                op1.append(op2[2])
                op2.remove(op2[2])
            elif op2[0].m == op2[2].m or op2[0].m != "vertical" and\
                    np.isclose(op2[0].m, op2[2].m, atol=1e-08, equal_nan=False):
                op1.append(op2[1])
                op2.remove(op2[1])
            elif op2[1].m == op2[2].m or op2[1].m != "vertical" and\
                    np.isclose(op2[1].m, op2[2].m, atol=1e-08, equal_nan=False):
                op1.append(op2[0])
                op2.remove(op2[0])
        m1 = Segment(op1[0].get_medium_point(), op1[1].get_medium_point())
        m2 = Segment(op2[0].get_medium_point(), op2[1].get_medium_point())
        return m1, m2


class StraightLine:

    def __init__(self, point, m):
        self.point = point
        if m == "vertical":
            self.x = self.point.x
            self.isVertical = True
        else:
            self.m = m
            self.q = self.point.y - self.m * self.point.x
            self.isVertical = False

    def get_point(self):
        return self.point.get_coordinate()

    def is_point_contained(self, point):
        if self.isVertical:
            if point.x == self.x:
                return True
            else:
                return False
        else:
            if np.isclose(point.y, self.m * point.x + self.q, atol=1e-08, equal_nan=False):
                return True
            else:
                return False


class Segment:

    def __init__(self, point1, point2):
        if point1.x == point2.x:
            self.point1 = min(point1, point2, key=lambda p: p.y)
            self.point2 = max(point1, point2, key=lambda p: p.y)
            self.x = point1.x
            self.m = "vertical"
            self.isVertical = True
        else:
            self.point1 = min(point1, point2, key=lambda p: p.x)
            self.point2 = max(point1, point2, key=lambda p: p.x)
            self.m = (self.point2.y - self.point1.y) / (self.point2.x - self.point1.x)
            self.q = self.point1.y - self.m * self.point1.x
            self.isVertical = False

    def get_points(self):
        return self.point1.get_coordinate(), self.point2.get_coordinate()

    def get_medium_point(self):
        x = (self.point1.x + self.point2.x) / 2
        y = (self.point1.y + self.point2.y) / 2
        return Point(x, y)

    def get_length(self):
        return sqrt(pow(self.point1.x - self.point2.x, 2) + pow(self.point1.y - self.point2.y, 2))

    def line_point_position(self, point):    # 1 = above / dx, 0 = in, -1 = under / sx
        if self.isVertical:
            if point.x > self.x:
                return 1
            if point.x < self.x:
                return -1
            if point.x == self.x:
                return 0
        else:
            if point.y > self.m * point.x + self.q:
                return 1
            if point.y < self.m * point.x + self.q:
                return -1
            if point.y == self.m * point.x + self.q:
                return 0

    def is_point_contained_in_segment(self, point):
        if self.isVertical:
            if point.x == self.x and self.point1.y < point.y < self.point2.y:
                return True
            else:
                return False
        else:
            if point.y == self.m * point.x + self.q and self.point1.x < point.x < self.point2.x:
                return True
            else:
                return False

    def is_point_contained_in_line(self, point):
        if self.isVertical:
            if point.x == self.x:
                return True
            else:
                return False
        else:
            if np.isclose(point.y, self.m * point.x + self.q, atol=1e-08, equal_nan=False):
                return True
            else:
                return False

    def has_extreme(self, point):
        if np.isclose(point.x, self.point1.x, atol=1e-08, equal_nan=False) and\
                np.isclose(point.y, self.point1.y, atol=1e-08, equal_nan=False) or\
                np.isclose(point.x, self.point2.x, atol=1e-08, equal_nan=False) and\
                np.isclose(point.y, self.point2.y, atol=1e-08, equal_nan=False):
            return True
        else:
            return False

    def contains_point(self, point):
        if self.isVertical:
            if (np.isclose(self.x, point.x, atol=1e-08, equal_nan=False)) and (
                    self.point1.y < point.y < self.point2.y or
                    np.isclose(point.y, self.point1.y, atol=1e-08, equal_nan=False) or
                    np.isclose(point.y, self.point2.y, atol=1e-08, equal_nan=False)):
                return True
            else:
                return False
        else:
            if (np.isclose(point.y, self.m * point.x + self.q, atol=1e-08, equal_nan=False)) and (
                    self.point1.x < point.x < self.point2.x or
                    np.isclose(point.x, self.point1.x, atol=1e-08, equal_nan=False) or
                    np.isclose(point.x, self.point2.x, atol=1e-08, equal_nan=False)):
                return True
            else:
                return False

    def is_equal_seg(self, segment):
        if self.point1.is_equal(segment.point1) and self.point2.is_equal(segment.point2):
            return True
        else:
            return False


def have_common_point(segment1, segment2):
    if segment1.point1.get_coordinate() == segment2.point1.get_coordinate() or \
            segment1.point1.get_coordinate() == segment2.point2.get_coordinate():
        return True
    elif segment1.point2.get_coordinate() == segment2.point1.get_coordinate() or \
            segment1.point2.get_coordinate() == segment2.point2.get_coordinate():
        return True
    else:
        return False


def have_common_point_with_is_close(segment1, segment2):
    if segment1.point1.is_equal(segment2.point1) or segment1.point1.is_equal(segment2.point2):
        return True
    elif segment1.point2.is_equal(segment2.point1) or segment1.point2.is_equal(segment2.point2):
        return True
    else:
        return False


def check_segment_vertical_intersection(segment1, segment2):    # segment 2 is vertical, segment 1 is not
    x = segment2.x
    y = segment1.m * x + segment1.q
    if segment2.point1.y < y < segment2.point2.y and segment1.point1.x < x < segment1.point2.x:
        return True
    else:
        return False


def check_segment_intersection(segment1, segment2):
    x = (segment1.q - segment2.q) / (segment2.m - segment1.m)
    if segment1.point1.x < x < segment1.point2.x and segment2.point1.x < x < segment2.point2.x:
        return True
    else:
        return False


def is_segment_intersection(segment1, segment2):
    if have_common_point(segment1, segment2):
        return False
    else:
        if not segment1.isVertical and segment2.isVertical:
            return check_segment_vertical_intersection(segment1, segment2)
        if segment1.isVertical and not segment2.isVertical:
            return check_segment_vertical_intersection(segment2, segment1)
        if (segment1.isVertical and segment2.isVertical) or (segment1.m == segment2.m):
            return False  # include anche il caso in cui i segmenti siano sulla stessa retta
        else:
            return check_segment_intersection(segment1, segment2)


def check_line_segment_vertical_intersection(segment1, segment2):
    x = segment2.x
    y = segment1.m * x + segment1.q
    if segment2.point1.y < y < segment2.point2.y:
        return True
    else:
        return False


def check_vertical_line_segment_intersection(segment1, segment2):
    x = segment1.x
    if segment2.point1.x < x < segment2.point2.x:
        return True
    else:
        return False


def check_line_segment_intersection(segment1, segment2):
    x = (segment1.q - segment2.q) / (segment2.m - segment1.m)
    if segment2.point1.x < x < segment2.point2.x:
        return True
    else:
        return False


def is_line_segment_intersection(segment1, segment2):
    if have_common_point(segment1, segment2):
        return False
    else:
        if not segment1.isVertical and segment2.isVertical:
            return check_line_segment_vertical_intersection(segment1, segment2)
        if segment1.isVertical and not segment2.isVertical:
            return check_vertical_line_segment_intersection(segment1, segment2)
        if (segment1.isVertical and segment2.isVertical) or (segment1.m == segment2.m):
            return False  # include anche il caso in cui i segmenti siano sulla stessa retta
        else:
            return check_line_segment_intersection(segment1, segment2)


class Polygon:

    def __init__(self, points):
        self.isDelimiter = False
        self.vertices = []
        self.segments = []
        for point in points:
            self.vertices.append(point)
        for i in range(len(points)):
            s = Segment(self.vertices[i - 1], self.vertices[i])
            self.segments.append(s)

    def contains_vertex(self, point):
        for vertex in self.vertices:
            if point.get_coordinate() == vertex.get_coordinate():
                return True
        return False

    def contains_segment(self, segment):
        for seg in self.segments:
            if segment.get_points() == seg.get_points():
                return True
        return False

    def line_intersection(self, segment):
        for seg in self.segments:
            if is_line_segment_intersection(segment, seg):
                return True
        return False

    def segment_intersection(self, segment):
        for seg in self.segments:
            if is_segment_intersection(segment, seg):
                return True
        return False

    def segment_contains_in_no_vertex(self, segment):
        count = 0
        for vertex in self.vertices:
            if segment.is_point_contained_in_segment(vertex):
                count += 1
        if count == 0:
            return True
        else:
            return False

    def line_contains_in_one_vertex(self, segment):
        count = 0
        for vertex in self.vertices:
            if segment.is_point_contained_in_line(vertex):
                count += 1
        if count == 1:
            return True
        else:
            return False

    def set_delimiter(self):
        self.isDelimiter = True

    def contains_extremes(self, segment):
        cond1 = False
        cond2 = False
        for seg in self.segments:
            if seg.contains_point(segment.point1):
                cond1 = True
            if seg.contains_point(segment.point2):
                cond2 = True
        if cond1 and cond2:
            return True
        else:
            return False

    def extremes_are_vertices(self, segment):
        cond1 = False
        cond2 = False
        for v in self.vertices:
            if v.is_equal(segment.point1):
                cond1 = True
            if v.is_equal(segment.point2):
                cond2 = True
        if cond1 and cond2:
            return True
        else:
            return False

    def append_segment(self, segment):
        self.segments.append(segment)

    def remove_segment(self, segment):
        for i in range(len(self.segments)):
            if self.segments[i].is_equal_seg(segment):
                self.segments[i] = None
        l1 = []
        for s in self.segments:
            if s is not None:
                l1.append(s)
        self.segments = l1


def create_standard_map_visibility_graph():
    start = Point(3, 28)
    goal = Point(28, 2)
    po1 = [Point(5, 25), Point(9, 26), Point(10, 22), Point(8, 19), Point(7, 23)]
    po2 = [Point(18, 21), Point(19, 15), Point(15, 12), Point(9, 14), Point(14, 16)]
    po3 = [Point(22, 25), Point(24, 27), Point(28, 21), Point(25, 17), Point(21, 20)]
    po4 = [Point(5, 11), Point(10, 8), Point(8, 2), Point(4, 5), Point(6, 7)]
    po5 = [Point(21, 8), Point(22, 12), Point(26, 11), Point(27, 5), Point(23, 3)]
    obstacle1 = Polygon(po1)
    obstacle2 = Polygon(po2)
    obstacle3 = Polygon(po3)
    obstacle4 = Polygon(po4)
    obstacle5 = Polygon(po5)
    return start, goal, [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]


def create_standard_map_trapezoidal_decomposition():  # il primo ostacolo è quello esterno
    start = Point(4, 19)
    goal = Point(11, 17)
    po1 = [Point(8, 2), Point(21, 4), Point(29, 11), Point(24, 27), Point(12, 29), Point(5, 27), Point(2, 17)]
    po2 = [Point(10, 6), Point(19, 8), Point(12, 9), Point(9, 21), Point(20, 23), Point(18, 25), Point(7, 24)]
    po3 = [Point(17, 10), Point(25, 16), Point(16, 20), Point(14, 15)]
    obstacle1 = Polygon(po1)
    obstacle2 = Polygon(po2)
    obstacle3 = Polygon(po3)
    return start, goal, [obstacle1, obstacle2, obstacle3]


def create_map_from_json_file(filename):
    with open(filename) as json_config:
        data_config = json.load(json_config)
    json_config.close()
    start = Point(data_config["relevant_points"]["start"][0], data_config["relevant_points"]["start"][1])
    goal = Point(data_config["relevant_points"]["goal"][0], data_config["relevant_points"]["goal"][1])
    obstacles = []
    for n, p in data_config["polygons"].items():
        points = []
        for coord in p:
            points.append(Point(coord[0], coord[1]))
        obstacle = Polygon(points)
        obstacles.append(obstacle)
    return start, goal, obstacles


class Map:

    def __init__(self, file_name):
        if file_name == "visibility":
            m = create_standard_map_visibility_graph()
            self.start = m[0]
            self.goal = m[1]
            self.obstacles = m[2]
        elif file_name == "trapezoidal":
            m = create_standard_map_trapezoidal_decomposition()
            self.start = m[0]
            self.goal = m[1]
            self.obstacles = m[2]
        else:
            m = create_map_from_json_file(file_name)
            self.start = m[0]
            self.goal = m[1]
            self.obstacles = m[2]

    def contains_segment(self, segment):
        for ob in self.obstacles:
            if ob.contains_segment(segment):
                return True
        return False

    def contains_vertex(self, point):
        for ob in self.obstacles:
            if ob.contains_vertex(point):
                return True
        return False

    def segment_is_diagonal(self, segment):
        for ob in self.obstacles:
            if ob.contains_vertex(segment.point1) and ob.contains_vertex(segment.point2):
                count = 0
                for vertex in ob.vertices:
                    count += segment.line_point_position(vertex)
                if abs(count) != len(ob.vertices) - 2:
                    return True
        return False

    def is_reduced_map_intersection(self, segment):
        cond = False
        if self.segment_is_diagonal(segment):
            cond = True
        for ob in self.obstacles:
            if ob.contains_vertex(segment.point1) and not ob.contains_vertex(segment.point2) or\
                    not ob.contains_vertex(segment.point1) and ob.contains_vertex(segment.point2):
                if not ob.line_contains_in_one_vertex(segment):
                    cond = True
            if ob.contains_vertex(segment.point1) or ob.contains_vertex(segment.point2):
                if ob.line_intersection(segment):
                    cond = True
            elif not ob.contains_vertex(segment.point1) and not ob.contains_vertex(segment.point2):
                if ob.segment_intersection(segment):
                    cond = True
        return cond

    def is_complete_map_intersection(self, segment):
        cond = False
        if self.segment_is_diagonal(segment):
            cond = True
        for ob in self.obstacles:
            if ob.contains_vertex(segment.point1) and not ob.contains_vertex(segment.point2) or\
                    not ob.contains_vertex(segment.point1) and ob.contains_vertex(segment.point2):
                if not ob.segment_contains_in_no_vertex(segment):
                    cond = True
            if ob.segment_intersection(segment):
                cond = True
        return cond

    def segment_extremes_in_one_obstacle(self, segment):
        cond = False
        for ob in self.obstacles:
            if ob.contains_extremes(segment):
                cond = True
        return cond

    def update_segments(self, spl, srm):
        for ob in self.obstacles:
            if ob.contains_segment(srm[0]):
                for s in spl:
                    cond = True
                    for seg in ob.segments:
                        if seg.point1.is_equal(s.point1) and seg.point2.is_equal(s.point2):
                            cond = False
                    if cond:
                        ob.append_segment(s)
                ob.remove_segment(srm[0])


class VisibilityGraph:

    def __init__(self, file_name, mode):   # mode can be "complete" or "reduced"
        self.map = Map(file_name)
        self.start = self.map.start
        self.goal = self.map.goal
        self.node_list = [self.start, self.goal]
        self.init_node_list()
        self.init_adjacent()
        self.new_segments = []
        if mode == "complete":
            self.create_complete_graph()
        else:
            self.create_reduced_graph()

    def init_node_list(self):
        for ob in self.map.obstacles:
            for vertex in ob.vertices:
                self.node_list.append(vertex)

    def init_adjacent(self):
        for node in self.node_list:
            for ob in self.map.obstacles:
                if ob.contains_vertex(node):
                    for se in ob.segments:
                        if node.get_coordinate() == se.point1.get_coordinate():
                            node.add_adjacent(se.point2)
                        elif node.get_coordinate() == se.point2.get_coordinate():
                            node.add_adjacent(se.point1)

    def new_adjacent(self):
        for node in self.node_list:
            for segment in self.new_segments:
                if node.get_coordinate() == segment.point1.get_coordinate():
                    node.add_adjacent(segment.point2)
                elif node.get_coordinate() == segment.point2.get_coordinate():
                    node.add_adjacent(segment.point1)

    def contained_in_new_segments(self, segment):
        for new in self.new_segments:
            if segment.get_points() == new.get_points():
                return True
        return False

    def segment_already_contained(self, segment):
        if self.map.contains_segment(segment) or self.contained_in_new_segments(segment):
            return True
        else:
            return False

    def create_reduced_graph(self):
        for node in self.node_list:
            for node2 in self.node_list:
                if node.get_coordinate() != node2.get_coordinate():
                    s = Segment(node, node2)
                    if not self.segment_already_contained(s):
                        if not self.map.is_reduced_map_intersection(s):
                            self.new_segments.append(s)
        self.new_adjacent()

    def create_complete_graph(self):
        for node in self.node_list:
            for node2 in self.node_list:
                if node.get_coordinate() != node2.get_coordinate():
                    s = Segment(node, node2)
                    if not self.segment_already_contained(s):
                        if not self.map.is_complete_map_intersection(s):
                            self.new_segments.append(s)
        self.new_adjacent()


def straight_line_segment_intersection_point(line, segment):
    if line.isVertical and not segment.isVertical:
        x = line.x
        y = segment.m * x + segment.q
        if segment.point1.x < x < segment.point2.x or np.isclose(x, segment.point1.x, atol=1e-08, equal_nan=False) or\
                np.isclose(x, segment.point2.x, atol=1e-08, equal_nan=False):
            return True, Point(x, y)
        else:
            return False, None
    if not line.isVertical and segment.isVertical:
        x = segment.x
        y = line.m * x + line.q
        if segment.point1.y < y < segment.point2.y or np.isclose(y, segment.point1.y, atol=1e-08, equal_nan=False) or\
                np.isclose(y, segment.point2.y, atol=1e-08, equal_nan=False):
            return True, Point(x, y)
        else:
            return False, None
    if line.isVertical and segment.isVertical or line.m == segment.m:
        return False, None
    else:
        x = (line.q - segment.q) / (segment.m - line.m)
        y = line.m * x + line.q
        if segment.point1.x < x < segment.point2.x or np.isclose(x, segment.point1.x, atol=1e-08, equal_nan=False) or\
                np.isclose(x, segment.point2.x, atol=1e-08, equal_nan=False):
            return True, Point(x, y)
        else:
            return False, None


def segment_segment_intersection_point(segment1, segment2):
    if segment1.isVertical and not segment2.isVertical:
        x = segment1.x
        y = segment2.m * x + segment2.q
        if (segment2.point1.x < x < segment2.point2.x or
            np.isclose(x, segment2.point1.x, atol=1e-08, equal_nan=False)
            or np.isclose(x, segment2.point2.x, atol=1e-08, equal_nan=False)) and (
                segment1.point1.y < y < segment1.point2.y or
                np.isclose(y, segment1.point1.y, atol=1e-08, equal_nan=False) or
                np.isclose(y, segment1.point2.y, atol=1e-08, equal_nan=False)):
            return True, Point(x, y)
        else:
            return False, None
    if not segment1.isVertical and segment2.isVertical:
        x = segment2.x
        y = segment1.m * x + segment1.q
        if (segment1.point1.x < x < segment1.point2.x or
            np.isclose(x, segment1.point1.x, atol=1e-08, equal_nan=False)
            or np.isclose(x, segment1.point2.x, atol=1e-08, equal_nan=False)) and (
                segment2.point1.y < y < segment2.point2.y or
                np.isclose(y, segment2.point1.y, atol=1e-08, equal_nan=False) or
                np.isclose(y, segment2.point2.y, atol=1e-08, equal_nan=False)):
            return True, Point(x, y)
        else:
            return False, None
    if segment1.isVertical and segment2.isVertical or segment1.m == segment2.m:
        return False, None
    else:
        x = (segment1.q - segment2.q) / (segment2.m - segment1.m)
        y = segment1.m * x + segment1.q
        if (segment1.point1.x < x < segment1.point2.x or
            np.isclose(x, segment1.point1.x, atol=1e-08, equal_nan=False)
            or np.isclose(x, segment1.point2.x, atol=1e-08, equal_nan=False)) and (
                segment2.point1.x < x < segment2.point2.x or
                np.isclose(x, segment2.point1.x, atol=1e-08, equal_nan=False) or
                np.isclose(x, segment2.point2.x, atol=1e-08, equal_nan=False)):
            return True, Point(x, y)
        else:
            return False, None


def segment_union(segment1, segment2):  # stessa m e stessa q
    if segment1.isVertical and segment2.isVertical:
        smi = min(segment1, segment2, key=lambda p: p.point1.y)
        sma = max(segment1, segment2, key=lambda p: p.point2.y)
        if np.isclose(smi.point2.y, sma.point1.y, atol=1e-08, equal_nan=False) or smi.point2.y > sma.point1.y:
            return True, Segment(smi.point1, sma.point2)
        else:
            return False, None
    else:
        smi = min(segment1, segment2, key=lambda p: p.point1.x)
        sma = max(segment1, segment2, key=lambda p: p.point2.x)
        if np.isclose(smi.point2.x, sma.point1.x, atol=1e-08, equal_nan=False) or smi.point2.x > sma.point1.x:
            return True, Segment(smi.point1, sma.point2)
        else:
            return False, None


class TrapezoidalDecomposition:

    def __init__(self, file_name, m_new_segments):
        self.m_new_segments = m_new_segments
        self.map = Map(file_name)
        self.map.obstacles[0].set_delimiter()
        self.start = self.map.start
        self.goal = self.map.goal
        self.new_segments = []
        self.square_limit = self.create_limits()
        self.create_trapezoidal_graph()
        self.update_obstacles()
        self.node_list = []
        self.area_list = []
        self.create_areas()
        self.set_start_and_goal_area()
        self.create_node_list_and_adj()

    def create_limits(self):
        x = []
        y = []
        for v in self.map.obstacles[0].vertices:
            x.append(v.x)
            y.append(v.y)
        xmi = min(x)
        xma = max(x)
        ymi = min(y)
        yma = max(y)
        po1 = [Point(xmi - 2, ymi - 2), Point(xmi - 2, yma + 2), Point(xma + 2, yma + 2), Point(xma + 2, ymi - 2)]
        limit = Polygon(po1)
        return limit

    def add_new_segment(self, segment):
        if len(self.new_segments) == 0:
            if not np.isclose(segment.get_length(), 0, atol=1e-08, equal_nan=False):
                self.new_segments.append(segment)
        else:
            cond = True
            for seg in self.new_segments:
                if seg.point1.is_equal(segment.point1) and seg.point2.is_equal(segment.point2):
                    cond = False
            if cond:
                if not np.isclose(segment.get_length(), 0, atol=1e-08, equal_nan=False):
                    self.new_segments.append(segment)

    def create_trapezoidal_graph(self):
        for ob in self.map.obstacles:
            for v in ob.vertices:
                line = StraightLine(v, self.m_new_segments)
                half_line = []
                for edge in self.square_limit.segments:
                    x = straight_line_segment_intersection_point(line, edge)
                    if x[0]:
                        s = Segment(line.point, x[1])
                        if len(half_line) == 0:
                            half_line.append(s)
                        else:
                            count = 0
                            for half in half_line:
                                if half.get_points() == s.get_points():
                                    count += 1
                            if count == 0:
                                half_line.append(s)
                for half in half_line:
                    for ob1 in self.map.obstacles:
                        for s1 in ob1.segments:
                            x = segment_segment_intersection_point(half, s1)
                            if x[0]:
                                s = Segment(line.point, x[1])  # verifico la validità del segmento
                                cond = True
                                for ob2 in self.map.obstacles:
                                    for s2 in ob2.segments:
                                        x2 = segment_segment_intersection_point(s, s2)
                                        if x2[0] and not (s.has_extreme(x2[1]) or s2.has_extreme(x2[1])):
                                            cond = False
                                if cond:
                                    if not self.map.segment_extremes_in_one_obstacle(s):
                                        self.add_new_segment(s)
                                    else:
                                        for ob3 in self.map.obstacles:
                                            if ob3.contains_extremes(s):
                                                if ob3.extremes_are_vertices(s):
                                                    if ob3.isDelimiter:
                                                        self.add_new_segment(s)
                                                else:
                                                    count = 0
                                                    for seg in ob3.segments:
                                                        if is_segment_intersection(seg, half):
                                                            count += 1
                                                    if not ob3.isDelimiter:
                                                        if count % 2 == 0:
                                                            self.add_new_segment(s)
                                                    else:
                                                        if count % 2 != 0:
                                                            self.add_new_segment(s)

    def update_obstacles(self):
        for ns in self.new_segments:
            spl = []
            srm = []
            for ob in self.map.obstacles:
                for seg in ob.segments:
                    if segment_segment_intersection_point(ns, seg)[0] and\
                            not have_common_point_with_is_close(ns, seg) and\
                            not ns.contains_point(seg.point1) and not ns.contains_point(seg.point2):
                        x = segment_segment_intersection_point(ns, seg)
                        spl.append(Segment(seg.point1, x[1]))
                        spl.append(Segment(x[1], seg.point2))
                        srm.append(seg)
            if len(spl) != 0 and len(srm) != 0:
                self.map.update_segments(spl, srm)
            spl.clear()
            srm.clear()

    def create_areas(self):
        for ob in self.map.obstacles:
            for seg in ob.segments:
                ns_point1 = []
                ns_point2 = []
                for ns in self.new_segments:
                    if ns.is_point_contained_in_line(seg.point1):
                        ns_point1.append(ns)
                    if ns.is_point_contained_in_line(seg.point2):
                        ns_point2.append(ns)
                if len(ns_point1) > 1:
                    count = 1
                    while count != 0:
                        count = 0
                        temporary = []
                        for n1 in ns_point1:
                            for n11 in ns_point1:
                                if not n1.is_equal_seg(n11):
                                    u = segment_union(n1, n11)
                                    if u[0]:
                                        temporary.append(u[1])
                        for temp in temporary:
                            cond = True
                            for ns1 in ns_point1:
                                if temp.is_equal_seg(ns1):
                                    cond = False
                            if cond:
                                ns_point1.append(temp)
                                count += 1
                if len(ns_point2) > 1:
                    count = 1
                    while count != 0:
                        count = 0
                        temporary = []
                        for n2 in ns_point2:
                            for n22 in ns_point2:
                                if not n2.is_equal_seg(n22):
                                    u = segment_union(n2, n22)
                                    if u[0]:
                                        temporary.append(u[1])
                        for temp in temporary:
                            cond = True
                            for ns2 in ns_point2:
                                if temp.is_equal_seg(ns2):
                                    cond = False
                            if cond:
                                ns_point2.append(temp)
                                count += 1
                if len(ns_point1) >= 1:
                    for i in range(len(ns_point1)):
                        if not ns_point1[i].point1.is_equal(seg.point1):
                            ns_point1[i] = None
                    l1 = []
                    for el1 in ns_point1:
                        if el1 is not None:
                            l1.append(el1)
                    ns_point1 = l1
                if len(ns_point2) >= 1:
                    for i in range(len(ns_point2)):
                        if not ns_point2[i].point1.is_equal(seg.point2):
                            ns_point2[i] = None
                    l2 = []
                    for el2 in ns_point2:
                        if el2 is not None:
                            l2.append(el2)
                    ns_point2 = l2
                if not (len(ns_point1) == 0 and len(ns_point2) == 0):
                    if len(ns_point1) == 0:
                        for ns2 in ns_point2:
                            s = Segment(seg.point1, ns2.point2)
                            cond = False
                            for ob1 in self.map.obstacles:
                                for seg1 in ob1.segments:
                                    if s.is_equal_seg(seg1):
                                        cond = True
                            if cond:
                                ar = Area([seg, ns2, s])
                                self.area_list.append(ar)
                    if len(ns_point2) == 0:
                        for ns1 in ns_point1:
                            s = Segment(seg.point2, ns1.point2)
                            cond = False
                            for ob1 in self.map.obstacles:
                                for seg1 in ob1.segments:
                                    if s.is_equal_seg(seg1):
                                        cond = True
                            if cond:
                                ar = Area([seg, ns1, s])
                                self.area_list.append(ar)
                    else:
                        for ns1 in ns_point1:
                            for ns2 in ns_point2:
                                s = Segment(ns1.point2, ns2.point2)
                                cond = False
                                for ob1 in self.map.obstacles:
                                    for seg1 in ob1.segments:
                                        if s.is_equal_seg(seg1):
                                            cond = True
                                if cond:
                                    ar = Area([seg, ns1, ns2, s])
                                    self.area_list.append(ar)
        l1 = []
        for i in range(len(self.area_list)):
            if len(self.area_list[i].segments) == 4:
                m = self.area_list[i].get_medians_trz()
                cond = False
                for ob in self.map.obstacles:
                    for s in ob.segments:
                        x0 = segment_segment_intersection_point(m[0], s)
                        x1 = segment_segment_intersection_point(m[1], s)
                        if x0[0] and not m[0].has_extreme(x0[1]):
                            cond = True
                        if x1[0] and not m[1].has_extreme(x1[1]):
                            cond = True
                if cond:
                    self.area_list[i] = None
        for el1 in self.area_list:
            if el1 is not None:
                l1.append(el1)
        self.area_list = l1

    def set_start_and_goal_area(self):
        start_line = StraightLine(self.start, self.m_new_segments)
        goal_line = StraightLine(self.goal, self.m_new_segments)
        start_list = []
        goal_list = []
        for ob in self.map.obstacles:
            for seg in ob.segments:
                x_start = straight_line_segment_intersection_point(start_line, seg)
                x_goal = straight_line_segment_intersection_point(goal_line, seg)
                if x_start[0]:
                    s = Segment(x_start[1], start_line.point)
                    start_list.append(s)
                if x_goal[0]:
                    s = Segment(x_goal[1], goal_line.point)
                    goal_list.append(s)
        start_segment = min(start_list, key=lambda p: p.get_length())
        goal_segment = min(goal_list, key=lambda p: p.get_length())
        is_start_in_area = False
        is_goal_in_area = False
        for ob in self.map.obstacles:
            for seg in ob.segments:
                x_start = segment_segment_intersection_point(start_segment, seg)
                x_goal = segment_segment_intersection_point(goal_segment, seg)
                if x_start[0] and not seg.has_extreme(x_start[1]):
                    for ar in self.area_list:
                        cond = False
                        for seg1 in ar.segments:
                            if seg1.is_equal_seg(seg):
                                cond = True
                        if cond:
                            ar.set_center(self.start)
                            is_start_in_area = True
                if x_goal[0] and not seg.has_extreme(x_goal[1]):
                    for ar in self.area_list:
                        cond = False
                        for seg1 in ar.segments:
                            if seg1.is_equal_seg(seg):
                                cond = True
                        if cond:
                            ar.set_center(self.goal)
                            is_goal_in_area = True
        a_start = []
        if not is_start_in_area:
            for ar in self.area_list:
                for seg2 in ar.segments:
                    if seg2.contains_point(self.start) and not seg2.has_extreme(self.start):
                        a_start.append(seg2)
        if len(a_start) != 0:
            a1 = Area([a_start[0]])
            a1.set_center(self.start)
            self.area_list.append(a1)
        a_goal = []
        if not is_goal_in_area:
            for ar in self.area_list:
                for seg2 in ar.segments:
                    if seg2.contains_point(self.goal) and not seg2.has_extreme(self.goal):
                        a_goal.append(seg2)
        if len(a_goal) != 0:
            a1 = Area([a_goal[0]])
            a1.set_center(self.goal)
            self.area_list.append(a1)

    def create_node_list_and_adj(self):
        for ar in self.area_list:
            if ar.get_coordinate() == self.start.get_coordinate():
                self.node_list.append(ar)
        for ar in self.area_list:
            if ar.get_coordinate() == self.goal.get_coordinate():
                self.node_list.append(ar)
        for ar in self.area_list:
            cond = True
            for node in self.node_list:
                if ar.get_coordinate() == node.get_coordinate():
                    cond = False
            if cond:
                self.node_list.append(ar)
        for node in self.node_list:
            for node1 in self.node_list:
                if not node.get_coordinate() == node1.get_coordinate():
                    for ns in node.segments:
                        for ns1 in node1.segments:
                            if ns.is_equal_seg(ns1):
                                node.add_adjacent(node1)
                            elif not ns.m == "vertical" and not ns1.m == "vertical":
                                if np.isclose(ns.m, ns1.m, atol=1e-08, equal_nan=False) and\
                                        np.isclose(ns.q, ns1.q, atol=1e-08, equal_nan=False):
                                    if segment_union(ns, ns1)[0]:
                                        if segment_union(ns, ns1)[1].is_equal_seg(ns) or \
                                                segment_union(ns, ns1)[1].is_equal_seg(ns1):
                                            node.add_adjacent(node1)
                            elif ns.m == "vertical" and ns1.m == "vertical":
                                if np.isclose(ns.x, ns1.x, atol=1e-08, equal_nan=False):
                                    if segment_union(ns, ns1)[0]:
                                        if segment_union(ns, ns1)[1].is_equal_seg(ns) or \
                                                segment_union(ns, ns1)[1].is_equal_seg(ns1):
                                            node.add_adjacent(node1)


class AStar:

    def __init__(self, graph):
        self.graph = graph
        self.start = self.graph.node_list[0]
        self.goal = self.graph.node_list[1]
        self.open_nodes = []
        self.visited_nodes = []
        self.init_goal_distance()
        self.best_path = []
        self.find_path()

    def init_goal_distance(self):
        for node in self.graph.node_list:
            node.set_goal_distance(self.goal)

    def already_visited(self, point):
        for visited in self.visited_nodes:
            if visited[0].get_coordinate() == point.get_coordinate():
                return True
        return False

    def already_open(self, point):
        for opened in self.open_nodes:
            if opened[0].get_coordinate() == point.get_coordinate():
                return True
        return False

    def possible_expansion(self, point):
        for ad in point.adjacent:
            if not self.already_visited(ad[0]):
                return True
        return False

    def remove_close_nodes(self):
        l1 = []
        for i in range(len(self.open_nodes)):
            if not self.possible_expansion(self.open_nodes[i][0]) and\
                    not self.open_nodes[i][0].get_coordinate() == self.goal.get_coordinate():
                self.open_nodes[i] = None
        for opened in self.open_nodes:
            if opened is not None:
                l1.append(opened)
        self.open_nodes = l1

    def move_to_best_neigh(self):
        if len(self.open_nodes) == 0 and len(self.visited_nodes) == 0:
            self.visited_nodes.append([self.start, 0])
            for adj in self.start.adjacent:
                self.open_nodes.append([adj[0], adj[1], self.start])
            return False
        else:
            self.remove_close_nodes()
            if len(self.open_nodes) == 0:
                return True
            best_neigh = min(self.open_nodes, key=lambda p: p[1] + p[0].goal_distance)
            self.visited_nodes.append(best_neigh)
            if best_neigh[0].get_coordinate() != self.goal.get_coordinate():
                for adj in best_neigh[0].adjacent:
                    if adj[0].get_coordinate() == self.goal.get_coordinate():
                        self.open_nodes.append([adj[0], best_neigh[1] + adj[1], best_neigh[0]])
                    if not self.already_visited(adj[0]) and adj[0].get_coordinate() != self.goal.get_coordinate():
                        if not self.already_open(adj[0]):
                            self.open_nodes.append([adj[0], best_neigh[1] + adj[1], best_neigh[0]])
                        else:
                            distance = 0
                            for opened in self.open_nodes:
                                if opened[0].get_coordinate() == adj[0].get_coordinate():
                                    distance = opened[1]
                            if best_neigh[1] + adj[1] < distance:
                                for opened in self.open_nodes:
                                    if opened[0].get_coordinate() == adj[0].get_coordinate():
                                        opened[1] = best_neigh[1] + adj[1]
                                        opened[2] = best_neigh[0]
                self.open_nodes.remove(best_neigh)
                return False
            else:
                self.open_nodes.remove(best_neigh)
                l2 = []
                for i in range(len(self.open_nodes)):
                    if (self.open_nodes[i][1] + self.open_nodes[i][0].goal_distance) >= best_neigh[1]:
                        self.open_nodes[i] = None
                for opened in self.open_nodes:
                    if opened is not None:
                        l2.append(opened)
                self.open_nodes = l2
                if len(self.open_nodes) == 0:
                    return True
                else:
                    return False

    def find_path(self):
        cond = self.move_to_best_neigh()
        while not cond:
            cond = self.move_to_best_neigh()
        goals = []
        for visited in self.visited_nodes:
            if visited[0].get_coordinate() == self.goal.get_coordinate():
                goals.append(visited)
        best_goal = min(goals, key=lambda p: p[1])
        self.best_path.append(best_goal)
        while len(best_goal) == 3:
            for visited in self.visited_nodes:
                if best_goal[2].get_coordinate() == visited[0].get_coordinate():
                    self.best_path.append(visited)
                    best_goal = visited
                    if best_goal[0].get_coordinate() == self.start.get_coordinate():
                        break
        self.best_path.reverse()


def plot_map(general_map):
    xf = []
    yf = []
    for polygon in general_map.obstacles:
        x = []
        y = []
        for vertex in polygon.vertices:
            x.append(vertex.x)
            y.append(vertex.y)
        x.append(polygon.vertices[0].x)
        y.append(polygon.vertices[0].y)
        xf.append(x)
        yf.append(y)
    for x, y in zip(xf, yf):
        plt.plot(x, y, color="r")
    plt.plot(general_map.start.x, general_map.start.y, color="b", marker="o")
    plt.plot(general_map.goal.x, general_map.goal.y, color="b", marker="o")


def plot_new_segments(new_segments):
    xf = []
    yf = []
    for seg in new_segments:
        x = []
        y = []
        x.append(seg.point1.x)
        x.append(seg.point2.x)
        y.append(seg.point1.y)
        y.append(seg.point2.y)
        xf.append(x)
        yf.append(y)
    for x, y in zip(xf, yf):
        plt.plot(x, y, color="y")
        plt.pause(0.1)


def plot_best_path_trapezoidal(best_path):
    xf = []
    yf = []
    x = []
    y = []
    for node in best_path:
        if len(node) == 3:
            for n in node[0].segments:
                for n1 in node[2].segments:
                    if n.is_equal_seg(n1):
                        xm = n.get_medium_point()
                        x.append(xm.x)
                        y.append(xm.y)
                    elif not n.m == "vertical" and not n1.m == "vertical":
                        if np.isclose(n.m, n1.m, atol=1e-08, equal_nan=False) and \
                                np.isclose(n.q, n1.q, atol=1e-08, equal_nan=False):
                            if segment_union(n, n1)[0]:
                                if segment_union(n, n1)[1].is_equal_seg(n):
                                    xm = n1.get_medium_point()
                                    x.append(xm.x)
                                    y.append(xm.y)
                                if segment_union(n, n1)[1].is_equal_seg(n1):
                                    xm = n.get_medium_point()
                                    x.append(xm.x)
                                    y.append(xm.y)
                    elif n.m == "vertical" and n1.m == "vertical":
                        if np.isclose(n.x, n1.x, atol=1e-08, equal_nan=False):
                            if segment_union(n, n1)[0]:
                                if segment_union(n, n1)[1].is_equal_seg(n):
                                    xm = n1.get_medium_point()
                                    x.append(xm.x)
                                    y.append(xm.y)
                                if segment_union(n, n1)[1].is_equal_seg(n1):
                                    xm = n.get_medium_point()
                                    x.append(xm.x)
                                    y.append(xm.y)
        x.append(node[0].x)
        y.append(node[0].y)
    for i in range(len(x) - 1):
        xf.append((x[i], x[i + 1]))
    for i in range(len(y) - 1):
        yf.append((y[i], y[i + 1]))
    for x, y in zip(xf, yf):
        plt.plot(x, y, color="b", marker=".")
        plt.pause(0.5)


def plot_best_path_visibility(best_path):
    xf = []
    yf = []
    x = []
    y = []
    for node in best_path:
        x.append(node[0].x)
        y.append(node[0].y)
    for i in range(len(x) - 1):
        xf.append((x[i], x[i + 1]))
    for i in range(len(y) - 1):
        yf.append((y[i], y[i + 1]))
    for x, y in zip(xf, yf):
        plt.plot(x, y, color="b", marker=".")
        plt.pause(0.5)


def main_visibility_graph():
    v = VisibilityGraph("visibility", "reduced")
    a = AStar(v)
    # for nod in a.best_path:
    #     print(nod[0].get_coordinate())
    plot_map(v.map)
    plt.pause(1.5)
    plot_new_segments(v.new_segments)
    plt.pause(1.5)
    plot_best_path_visibility(a.best_path)
    plt.show()


def main_trapezoidal_decomposition():
    t = TrapezoidalDecomposition("trapezoidal", 0.5)
    # for n in t.node_list:
    #     print(n.get_coordinate())
    a = AStar(t)
    plot_map(t.map)
    plt.pause(1.5)
    plot_new_segments(t.new_segments)
    plt.pause(1.5)
    plot_best_path_trapezoidal(a.best_path)
    plt.show()


if __name__ == '__main__':
    # main_visibility_graph()
    main_trapezoidal_decomposition()
