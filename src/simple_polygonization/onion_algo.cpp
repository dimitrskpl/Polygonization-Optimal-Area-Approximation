#include "../../include/simple_polygonization/onion_algo.hpp"

#include "../../include/graham_scan.hpp"

static void make_layer(std::vector<Point_2> &tmp_vec,
                       Polygon_2 &cur_convex_hull) {
  std::vector<int> indices;

  graham_scan(tmp_vec, indices);

  std::vector<bool> points_on_new_layer(tmp_vec.size(), false);
  std::vector<Point_2> rest_points;

  for (int i = 0; i < indices.size(); ++i) {
    cur_convex_hull.push_back(tmp_vec[indices[i]]);
    points_on_new_layer[indices[i]] = true;
  }

  for (int i = 0; i < tmp_vec.size(); ++i) {
    if (!points_on_new_layer[i]) {
      rest_points.push_back(tmp_vec[i]);
    }
  }
  tmp_vec = rest_points;
}

static void build_onion(std::vector<Polygon_2> &onion_layers,
                        std::vector<Point_2> &points) {
  std::vector<Point_2> tmp_vec = points;

  int layer = 0;
  while (!tmp_vec.empty()) {
    if (tmp_vec.size() <= 2) {
      onion_layers.push_back(Polygon_2(tmp_vec.begin(), tmp_vec.end()));
      break;
    } else {
      Polygon_2 cur_convex_hull;
      make_layer(tmp_vec, cur_convex_hull);
      onion_layers.push_back(cur_convex_hull);
    }
  }
}

static int find_closest_point(Segment_2 cur_segment, Polygon_2 &polygon) {
  double min_dist = -1;
  int closest_point_idx = 0;
  for (int i = 0; i < polygon.size(); ++i) {
    double cur_dist = CGAL::squared_distance(cur_segment, polygon[i]);
    if (min_dist < 0 || cur_dist < min_dist) {
      min_dist = cur_dist;
      closest_point_idx = i;
    }
  }
  return closest_point_idx;
}

static bool is_visible(Point_2 p, Point_2 q, Point_2 r,
                       CGAL::Sign Orientation) {
  CGAL::Sign result = CGAL::orientation(p, q, r);
  return (result == Orientation);
}

static Polygon_2 get_inner_hull_in_order(Polygon_2 convex_hull, int start,
                                         int end, bool CCW) {
  Polygon_2 ordered_hull;
  int i = start;
  while (true) {
    ordered_hull.push_back(convex_hull[i]);
    if (i == end) {
      break;
    }

    if (!CCW) {
      i = (i == convex_hull.size() - 1) ? (0) : (i + 1);
    } else {
      i = (i == 0) ? (convex_hull.size() - 1) : (i - 1);
    }
  }
  return ordered_hull;
}

static void unite_convex_hulls(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                               int outer_point_idx, int closest_point_idx,
                               int nxt_2_closest_point_idx, bool ccw) {
  Polygon_2 ordered_ch = get_inner_hull_in_order(convex_hull, closest_point_idx,
                                                 nxt_2_closest_point_idx, ccw);
  polygon_line.insert(polygon_line.begin() + outer_point_idx + 1,
                      ordered_ch.begin(), ordered_ch.end());
}

static int find_next_depth_m_point(Polygon_2 &polygon_line,
                                   Polygon_2 &convex_hull, int idx, int m,
                                   bool ccw) {
  int idx_on_convex_hull =
      (idx + (convex_hull.size() + 1) / 2) % convex_hull.size();

  if (ccw) {
    idx_on_convex_hull = (idx_on_convex_hull == (convex_hull.size() - 1)
                              ? 0
                              : idx_on_convex_hull + 1);
  }

  int ret_value;

  if (convex_hull.size() == 3) {
    return m + 2;
  }

  for (int i = 0; i < convex_hull.size(); ++i) {
    if (polygon_line[m + i + 1] == convex_hull[idx_on_convex_hull]) {
      if (convex_hull.size() % 2) {
        if (i == convex_hull.size() - 1) {
          ret_value = m + 1;
          break;
        } else {
          ret_value = m + i + 2;
          break;
        }
      } else {
        ret_value = m + i + 1;
        break;
      }
    }
  }

  return ret_value;
}

static int select_init_idx(Polygon_2 &convex_hull, int start,
                           std::vector<Segment_2> &marked_segments) {
  for (int i = start; i < convex_hull.size(); ++i) {
    Point_2 point_m = convex_hull[i];
    Point_2 point_nxt_2_m = convex_hull[(i + 1) % convex_hull.size()];
    bool valid_i = true;
    for (int j = 0; j < marked_segments.size(); j++) {
      if ((marked_segments[j].source() == point_m &&
           marked_segments[j].target() == point_nxt_2_m) ||
          (marked_segments[j].source() == point_nxt_2_m &&
           marked_segments[j].target() == point_m)) {
        valid_i = false;
        break;
      }
    }
    if (valid_i) {
      return i;
    }
  }
  return -1;
}

static int select_init_idx(Polygon_2 &convex_hull,
                           INIT_OPTION_ONION onion_initialization) {
  if (onion_initialization == RANDOM_POINT) {
    return 0;
  }
  int min_x_idx = -1, max_x_idx = -1, min_y_idx = -1, max_y_idx = -1;
  int min_x = -1, max_x = -1, min_y = -1, max_y = -1;
  for (int i = 0; i < convex_hull.size(); ++i) {
    int p_x = (int)convex_hull[i].x();
    int p_y = (int)convex_hull[i].y();

    if (min_x_idx == -1 || p_x < min_x) {
      min_x = p_x;
      min_x_idx = i;
    }
    if (max_x_idx == -1 || p_x > max_x) {
      max_x = p_x;
      max_x_idx = i;
    }
    if (min_y_idx == -1 || p_y < min_y) {
      min_y = p_y;
      min_y_idx = i;
    }
    if (max_y_idx == -1 || p_y > max_y) {
      max_y = p_y;
      max_y_idx = i;
    }
  }

  int result;

  switch (onion_initialization) {
    case MIN_X_POINT:
      result = min_x_idx;
      break;
    case MAX_X_POINT:
      result = max_x_idx;
      break;
    case MIN_Y_POINT:
      result = min_y_idx;
      break;
    default:
      result = max_y_idx;
  }

  return result;
}

static void insert_segment_ch(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                              int m, int nxt_m) {
  if (!do_intersect(Segment_2(polygon_line[m], convex_hull[0]),
                    Segment_2(polygon_line[nxt_m], convex_hull[1]))) {
    polygon_line.insert(polygon_line.begin() + m + 1, convex_hull[0]);
    polygon_line.insert(polygon_line.begin() + m + 2, convex_hull[1]);
  } else {
    polygon_line.insert(polygon_line.begin() + m + 1, convex_hull[1]);
    polygon_line.insert(polygon_line.begin() + m + 2, convex_hull[0]);
  }
}

static void insert_last_depth_ch(Polygon_2 &polygon_line,
                                 Polygon_2 &convex_hull, int m, int nxt_m) {
  if (!do_intersect(Segment_2(polygon_line[m], convex_hull[0]),
                    Segment_2(polygon_line[nxt_m],
                              convex_hull[convex_hull.size() - 1]))) {
    polygon_line.insert(polygon_line.begin() + m + 1, convex_hull.begin(),
                        convex_hull.end());
  } else {
    std::reverse(convex_hull.begin(), convex_hull.end());
    polygon_line.insert(polygon_line.begin() + m + 1, convex_hull.begin(),
                        convex_hull.end());
  }
}

static bool insert_nxt_depth_ch(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                                int &m, int nxt_m, bool ccw) {
  bool expanded = true;
  Segment_2 seg_2_brake = Segment_2(polygon_line[m], polygon_line[nxt_m]);

  int closest_point_idx = find_closest_point(seg_2_brake, convex_hull);

  int prev_2_closest_idx = (closest_point_idx == 0) ? (convex_hull.size() - 1)
                                                    : (closest_point_idx - 1);
  int nxt_2_closest_idx = (closest_point_idx + 1) % convex_hull.size();

  Point_2 real_source = seg_2_brake.source();
  Point_2 real_target = seg_2_brake.target();
  if (!ccw) {
    real_source = seg_2_brake.target();
    real_target = seg_2_brake.source();
  }

  Segment_2 candidate_edge_1 = Segment_2(convex_hull[prev_2_closest_idx],
                                         convex_hull[closest_point_idx]);
  Segment_2 candidate_edge_2 =
      Segment_2(convex_hull[closest_point_idx], convex_hull[nxt_2_closest_idx]);

  if (is_visible(real_source, candidate_edge_1.target(),
                 candidate_edge_1.source(), CGAL::Sign::COUNTERCLOCKWISE)) {
    if (ccw) {
      unite_convex_hulls(polygon_line, convex_hull, m, prev_2_closest_idx,
                         closest_point_idx, ccw);
    } else {
      unite_convex_hulls(polygon_line, convex_hull, m, closest_point_idx,
                         prev_2_closest_idx, ccw);
    }
    m = find_next_depth_m_point(polygon_line, convex_hull, prev_2_closest_idx,
                                m, ccw);
  } else if (is_visible(real_target, candidate_edge_2.source(),
                        candidate_edge_2.target(), CGAL::Sign::CLOCKWISE)) {
    if (ccw) {
      unite_convex_hulls(polygon_line, convex_hull, m, closest_point_idx,
                         nxt_2_closest_idx, ccw);
    } else {
      unite_convex_hulls(polygon_line, convex_hull, m, nxt_2_closest_idx,
                         closest_point_idx, ccw);
    }

    m = find_next_depth_m_point(polygon_line, convex_hull, closest_point_idx, m,
                                ccw);
  } else {
    expanded = false;
  }
  return expanded;
}

static bool expand_polygon_line(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                                int depth, int &m, int nxt_m, int max_depth,
                                bool ccw) {
  bool expanded = true;
  if ((depth == max_depth - 2) && ((ll)fabs(convex_hull.area()) == 0LL)) {
    insert_last_depth_ch(polygon_line, convex_hull, m, nxt_m);
  } else {
    expanded = insert_nxt_depth_ch(polygon_line, convex_hull, m, nxt_m, ccw);
  }
  return expanded;
}

void onion_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                std::vector<Point_2> &points,
                INIT_OPTION_ONION onion_initialization) {
  std::vector<Polygon_2> onion_layers;

  build_onion(onion_layers, points);

  polygon_line = onion_layers[0];
  convex_hull = onion_layers[0];

  int m = select_init_idx(onion_layers[0], onion_initialization);
  int depth = 0, max_depth = onion_layers.size();
  while (depth < max_depth - 1) {
    bool ccw = !(depth % 2);
    int nxt_m = (m + 1) % polygon_line.size();
    int nxt_depth = depth + 1;

    if (!expand_polygon_line(polygon_line, onion_layers[nxt_depth], depth, m,
                             nxt_m, onion_layers.size(), ccw)) {
      m = nxt_m;
    } else {
      depth++;
    }
  }
}

void onion_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                std::vector<Point_2> &points,
                std::vector<Segment_2> &marked_segments) {
  std::vector<Polygon_2> onion_layers;
  build_onion(onion_layers, points);
  polygon_line = onion_layers[0];
  convex_hull = onion_layers[0];
  int m = select_init_idx(convex_hull, 0, marked_segments);
  int depth = 0, max_depth = onion_layers.size();
  int prev_depth_m = -1;

  while (depth < max_depth - 1) {
    bool ccw = !(depth % 2);
    int nxt_depth = depth + 1;
    int prev_depth_size = onion_layers[depth].size();

    if (m == prev_depth_m + prev_depth_size - 1) {
      m = prev_depth_m + 1;
    }
    int nxt_m = m + 1;
    int m_value = m;

    if (!expand_polygon_line(polygon_line, onion_layers[nxt_depth], depth, m,
                             nxt_m, onion_layers.size(), ccw)) {
      if (depth == 0) {
        m = select_init_idx(onion_layers[0], nxt_m, marked_segments);
      } else {
        m = nxt_m;
      }
    } else {
      prev_depth_m = m_value;
      depth++;
    }
  }
}