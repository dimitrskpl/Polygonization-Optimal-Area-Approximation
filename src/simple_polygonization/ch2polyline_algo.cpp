#include "../../include/simple_polygonization/ch2polyline_algo.hpp"

static bool is_visible(Polygon_2 &polygon, Point_2 &new_point,
                       Point_2 &left_point, Point_2 &right_point) {
  Point_2 middle_point = CGAL::midpoint(left_point, right_point);

  if (!do_intersect_corner(polygon, Segment_2(new_point, left_point)) &&
      !do_intersect_middle(polygon, Segment_2(new_point, middle_point),
                           Segment_2(left_point, right_point)) &&
      !do_intersect_corner(polygon, Segment_2(new_point, right_point))) {
    return true;
  }
  return false;
}

static void expand_polygon_line(Polygon_2 &polygon_line, Point_2 new_point,
                                VISIBLE_EDGE_CHOICE visible_edge_choice) {
  double min_area = -1, max_area = -1;
  double polygon_area = fabs(polygon_line.area());
  int poly_idx_min_area = -1, poly_idx_max_area = -1;
  for (int i = 0; i < polygon_line.size(); ++i) {
    int nxt_idx = (i + 1) % polygon_line.size();
    if (is_visible(polygon_line, new_point, polygon_line[i],
                   polygon_line[nxt_idx])) {
      if (visible_edge_choice == RANDOM_EDGE) {
        polygon_line.insert(polygon_line.begin() + i + 1, new_point);
        return;
      }

      double cur_area =
          CGAL::area(polygon_line[i], polygon_line[nxt_idx], new_point);
      if (cur_area < min_area || min_area == -1) {
        min_area = cur_area;
        poly_idx_min_area = i;
      }
      if (cur_area > max_area || max_area == -1) {
        max_area = cur_area;
        poly_idx_max_area = i;
      }
    }
  }
  if (visible_edge_choice == MAX_AREA) {
    polygon_line.insert(polygon_line.begin() + poly_idx_min_area + 1,
                        new_point);
  } else {
    polygon_line.insert(polygon_line.begin() + poly_idx_max_area + 1,
                        new_point);
  }
}

void ch2polyline_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                      std::vector<Point_2> &points,
                      VISIBLE_EDGE_CHOICE visible_edge_choice) {
  std::vector<std::size_t> indices(points.size()), out;

  std::iota(indices.begin(), indices.end(), 0);

  CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                      Convex_hull_traits_2(CGAL::make_property_map(points)));

  std::vector<bool> points_on_ch(points.size(), false);

  for (int i = 0; i < out.size(); ++i) {
    polygon_line.push_back(points[out[i]]);
    points_on_ch[out[i]] = true;
  }

  convex_hull = polygon_line;

  int out_idx = 0;
  for (int i = 0; i < points.size(); ++i) {
    if (points_on_ch[i] == true) {
      continue;
    }
    expand_polygon_line(polygon_line, points[i], visible_edge_choice);
  }
}

bool in_marked_segments(std::vector<Segment_2> red_segments, Point_2 p1,
                        Point_2 p2) {
  Segment_2 seg_1 = Segment_2(p1, p2);
  Segment_2 seg_2 = Segment_2(p2, p1);
  for (int i = 0; i < red_segments.size(); i++) {
    if (red_segments[i] == seg_1 || red_segments[i] == seg_2) {
      return true;
    }
  }
  return false;
}

static void expand_polygon_line(Polygon_2 &polygon_line, Point_2 new_point,
                                std::vector<Segment_2> marked_segments) {
  for (int i = 0; i < polygon_line.size(); ++i) {
    int nxt_idx = (i + 1) % polygon_line.size();
    if (!in_marked_segments(marked_segments, polygon_line[i],
                            polygon_line[nxt_idx]) &&
        is_visible(polygon_line, new_point, polygon_line[i],
                   polygon_line[nxt_idx])) {
      polygon_line.insert(polygon_line.begin() + i + 1, new_point);
      return;
    }
  }
}

void ch2polyline_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                      std::vector<Point_2> &points,
                      std::vector<Segment_2> marked_segments) {
  std::vector<std::size_t> indices(points.size()), out;

  std::iota(indices.begin(), indices.end(), 0);

  CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                      Convex_hull_traits_2(CGAL::make_property_map(points)));

  std::vector<bool> points_on_ch(points.size(), false);

  for (int i = 0; i < out.size(); ++i) {
    polygon_line.push_back(points[out[i]]);
    points_on_ch[out[i]] = true;
  }

  convex_hull = polygon_line;

  int out_idx = 0;
  for (int i = 0; i < points.size(); ++i) {
    if (points_on_ch[i] == true) {
      continue;
    }
    expand_polygon_line(polygon_line, points[i], marked_segments);
  }
}
