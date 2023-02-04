#include "../include/pick_algo.hpp"

static ll lattice_points(Polygon_2 &polygon) {
  // find max and min x's
  ll min_x = -1, max_x = -1;
  ll min_y = -1, max_y = -1;

  for (int i = 0; i < polygon.size(); ++i) {
    int p_x = (int)polygon[i].x();
    int p_y = (int)polygon[i].y();
    if (min_x == -1 || p_x < min_x) {
      min_x = p_x;
    }
    if (max_x == -1 || p_x > max_x) {
      max_x = p_x;
    }
    if (min_y == -1 || p_y < min_y) {
      min_y = p_y;
    }
    if (max_y == -1 || p_y > max_y) {
      max_y = p_y;
    }
  }
  ll res = 0;
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      if (CGAL::bounded_side_2(polygon.begin(), polygon.end(), Point_2(x, y)) ==
          CGAL::ON_BOUNDED_SIDE) {
        res++;
      }
    }
  }
  return res;
}

static int mygcd(int a, int b) {
  if (a == 0) return b;
  return mygcd(b % a, a);
}

static ll boundary_points(Polygon_2 &polygon) {
  ll res = polygon.size();
  for (int i = 0; i < polygon.size(); ++i) {
    int nxt_point = (i + 1) % polygon.size();
    int dx = abs((int)polygon[i].x() - (int)polygon[nxt_point].x());
    int dy = abs((int)polygon[i].y() - (int)polygon[nxt_point].y());
    res += mygcd(dx, dy) - 1;
  }
  return res;
}

ll pick_algo(Polygon_2 &polygon) {
  ll I = lattice_points(polygon), B = boundary_points(polygon);
  return (I + B / 2 - 1);
}