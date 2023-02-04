#include "../../include/optimal_polygonization/subdivision.hpp"

#define M 100

static void make_subsets(std::vector<Point_2> &points,
                         std::vector<std::vector<Point_2>> &subsets) {
  int start = 0;
  while (start < points.size()) {
    std::vector<Point_2> cur_subset;

    if (start + M - 1 >= points.size()) {
      if (!subsets.empty()) {
        subsets[subsets.size() - 1].insert(subsets[subsets.size() - 1].end(),
                                           points.begin() + start + 1,
                                           points.end());
      } else {
        cur_subset.insert(cur_subset.end(), points.begin() + start,
                          points.end());
        subsets.push_back(cur_subset);
      }
      break;
    }

    if (start + M - 1 == points.size() - 1) {
      cur_subset.insert(cur_subset.end(), points.begin() + start, points.end());
      subsets.push_back(cur_subset);
      break;
    }

    if (start != 0) {
      cur_subset.push_back(points[start]);
    }
    int min_y = INT_MAX;
    for (int i = (start != 0); i < M - 1; ++i) {
      min_y = std::min(min_y, (int)points[start + i].y());
      cur_subset.push_back(points[start + i]);
    }
    cur_subset.push_back(points[start + M - 1]);
    std::multiset<int> y_values;
    for (int i = 1; (i < M - 1) && (start + M - 1 + i < points.size()); ++i) {
      y_values.insert((int)points[start + M - 1 + i].y());
    }

    int shift = 1;

    while ((start + M - 1 + shift) < points.size()) {
      int q_y = (int)points[start + M - 1 + shift - 1].y();

      if ((min_y < q_y && q_y > *y_values.begin()) ||
          (start + M - 1 + shift - 1 + M - 1 >= points.size())) {
        subsets.push_back(cur_subset);
        break;
      }
      min_y = std::min(min_y, (int)points[start + M - 1 + shift - 1].y());
      cur_subset.push_back(points[start + M - 1 + shift]);
      y_values.erase(y_values.find((int)points[start + M - 1 + shift].y()));
      if (start + M - 1 + shift - 1 + M - 1 < points.size()) {
        y_values.insert((int)points[(start + M - 1 + shift - 1) + M - 1].y());
      }
      shift++;
    }
    start += (M - 1 + shift - 1);
  }
}

static void merge_polygon_lines(Polygon_2 &polygon_line,
                                Polygon_2 sub_polygon_line, Point_2 p,
                                Point_2 q, Point_2 r) {
  int size_before = polygon_line.size() + sub_polygon_line.size();
  int sub_q_idx = -1, sub_r_idx = -1;

  for (int i = 0; i < sub_polygon_line.size(); ++i) {
    if (sub_polygon_line[i] == q) {
      sub_q_idx = i;
    }
    if (sub_polygon_line[i] == r) {
      sub_r_idx = i;
    }
  }

  std::vector<Point_2> to_be_inserted;
  if (((sub_r_idx + 1) % sub_polygon_line.size()) == sub_q_idx) {
    while (sub_r_idx != sub_q_idx) {
      to_be_inserted.push_back(sub_polygon_line[sub_r_idx]);
      sub_r_idx =
          (sub_r_idx == 0) ? (sub_polygon_line.size() - 1) : (sub_r_idx - 1);
    }
  } else {
    while (sub_r_idx != sub_q_idx) {
      to_be_inserted.push_back(sub_polygon_line[sub_r_idx]);
      sub_r_idx = (sub_r_idx + 1) % sub_polygon_line.size();
    }
  }

  int poly_q_idx = -1, poly_p_idx = -1;
  for (int i = 0; i < polygon_line.size(); ++i) {
    if (polygon_line[i] == q) {
      poly_q_idx = i;
    }
    if (polygon_line[i] == p) {
      poly_p_idx = i;
    }
  }

  polygon_line.insert(polygon_line.begin() + poly_p_idx + 1,
                      to_be_inserted.begin(), to_be_inserted.end());
}

void subdivision(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                 OPTIMIZATION optimization, int L, INIT_ALGO init_algo) {
  merge_sort(points, 0, points.size() - 1, X_ASCENDING);

  std::vector<std::vector<Point_2>> subsets;
  make_subsets(points, subsets);

  std::vector<Polygon_2> polygon_lines(subsets.size(), Polygon_2());
  std::vector<std::vector<Point_2>> subsets_marked_points;

  for (int i = 0; i < subsets.size(); ++i) {
    std::vector<int> indices;
    graham_scan(subsets[i], indices);

    std::vector<Point_2> marked_points;
    if (i != 0) {
      int q_idx = -1;
      for (int j = 0; j < indices.size(); ++j) {
        if (subsets[i][indices[j]] == subsets[i][0]) {
          q_idx = j;
          break;
        }
      }

      int q_nxt = (q_idx + 1) % indices.size();
      marked_points.push_back(subsets[i][indices[q_idx]]);
      marked_points.push_back(subsets[i][indices[q_nxt]]);
    }
    if (i != subsets.size() - 1) {
      int last_point = -1;

      Point_2 last_p = subsets[i].back();
      for (int j = 0; j < indices.size(); ++j) {
        if (subsets[i][indices[j]] == subsets[i].back()) {
          last_point = j;
          break;
        }
      }

      int last_point_prev =
          (last_point == 0) ? (indices.size() - 1) : (last_point - 1);
      marked_points.push_back(subsets[i][indices[last_point]]);
      marked_points.push_back(subsets[i][indices[last_point_prev]]);
    }
    subsets_marked_points.push_back(marked_points);

    std::vector<Segment_2> marked_segments;
    for (int j = 0; j < marked_points.size(); j += 2) {
      Segment_2 cur_segment = Segment_2(marked_points[j], marked_points[j + 1]);
      marked_segments.push_back(cur_segment);
    }

    simulated_annealing_algo(subsets[i], polygon_lines[i], optimization, L,
                             marked_segments, init_algo);
  }

  polygon_line = polygon_lines[0];

  for (int i = 1; i < polygon_lines.size(); ++i) {
    Point_2 point_p, point_q;
    if (i == 1) {
      point_p = subsets_marked_points[i - 1][1],
      point_q = subsets_marked_points[i - 1][0];
    } else {
      point_p = subsets_marked_points[i - 1][3],
      point_q = subsets_marked_points[i - 1][2];
    }

    merge_polygon_lines(polygon_line, polygon_lines[i], point_p, point_q,
                        subsets_marked_points[i][1]);
  }

  optimize_subdivision_polygon(points, polygon_line, optimization, L);

  Polygon_2 polygon_line_onion;
  Polygon_2 convex_hull;
}