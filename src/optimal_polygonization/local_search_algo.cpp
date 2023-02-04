#include "../../include/optimal_polygonization/local_search_algo.hpp"

void local_search_algo(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                       int L, OPTIMIZATION optimization, double threshold,
                       double &area_initial, double &ratio_initial,
                       INIT_ALGO init_algo) {
  Polygon_2 convex_hull;

  if (init_algo == CH2POLY_ALGO) {
    ch2polyline_algo(polygon_line, convex_hull, points, RANDOM_EDGE);
  } else {
    onion_algo(polygon_line, convex_hull, points, RANDOM_POINT);
  }

  double DA = fabs(convex_hull.area());
  area_initial = fabs(polygon_line.area());
  ratio_initial = area_initial / DA;

  Polygon_2 transition_polygon = polygon_line;

  while (DA > threshold) {
    double cur_area = fabs(polygon_line.area());
    double best_area = cur_area;

    for (int u1_idx = 0; u1_idx < polygon_line.size(); ++u1_idx) {
      for (int v1_idx = 0; v1_idx < polygon_line.size(); ++v1_idx) {
        for (int l = 0; l < L; ++l) {
          int vk_idx = (v1_idx + l) % polygon_line.size();
          int u2_idx = (u1_idx + 1) % polygon_line.size();
          if (L == 1 && (v1_idx == u1_idx || v1_idx == u2_idx)) {
            continue;
          } else if (L != 1 && ((v1_idx <= vk_idx &&
                                 (u1_idx >= v1_idx - 1 && u1_idx <= vk_idx)) ||
                                (vk_idx <= v1_idx && ((u1_idx >= v1_idx - 1) ||
                                                      (u1_idx <= vk_idx))))) {
            continue;
          }

          Polygon_2 new_polygon;

          bool reached_vk = false;
          int p = 0;
          while (p < polygon_line.size()) {
            if (p == u1_idx) {
              new_polygon.push_back(polygon_line[u1_idx]);

              for (int q = l; q >= 0; --q) {
                int idx = (v1_idx + q) % polygon_line.size();
                new_polygon.push_back(polygon_line[idx]);
              }
            } else if (p == v1_idx) {
              p = (p + l);
            } else {
              new_polygon.push_back(polygon_line[p]);
            }
            p++;
          }

          double new_area = fabs(new_polygon.area());

          if (new_polygon.is_simple()) {
            if ((optimization == MAXIMAL_AREA && best_area < new_area) ||
                (optimization == MINIMAL_AREA && best_area > new_area)) {
              best_area = new_area;
              transition_polygon = new_polygon;
            }
          }
        }
      }
    }

    polygon_line = transition_polygon;
    DA = fabs(cur_area - best_area);
  }
}