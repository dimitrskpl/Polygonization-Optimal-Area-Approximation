#include "../../include/optimal_polygonization/local_transition.hpp"

static bool on_box_boundary(Point_2 point, std::vector<Segment_2> &box_segs) {
  for (int i = 0; i < box_segs.size(); ++i) {
    if (do_intersect(box_segs[i], point)) {
      return true;
    }
  }
  return false;
}

bool local_transition(Polygon_2 &polygon_line, KD_Tree &kdtree,
                      std::map<Point_2, int> &original_idx,
                      std::vector<int> &offset) {
  int q = rand() % polygon_line.size();
  int p = (q == 0) ? (polygon_line.size() - 1) : (q - 1);
  int r = (q + 1) % polygon_line.size();
  int s = (r + 1) % polygon_line.size();
  int s_nxt = (s + 1) % polygon_line.size();
  int p_prev = (p == 0) ? (polygon_line.size() - 1) : (p - 1);

  Segment_2 pr = Segment_2(polygon_line[p], polygon_line[r]);
  Segment_2 qs = Segment_2(polygon_line[q], polygon_line[s]);
  Segment_2 p_prevp = Segment_2(polygon_line[p_prev], polygon_line[p]);
  Segment_2 ss_nxt = Segment_2(polygon_line[s], polygon_line[s_nxt]);

  if (CGAL::do_intersect(pr, qs)) {
    return false;
  }

  int min_x = std::min(std::min(polygon_line[p].x(), polygon_line[q].x()),
                       std::min(polygon_line[r].x(), polygon_line[s].x()));

  int max_x = std::max(std::max(polygon_line[p].x(), polygon_line[q].x()),
                       std::max(polygon_line[r].x(), polygon_line[s].x()));

  int min_y = std::min(std::min(polygon_line[p].y(), polygon_line[q].y()),
                       std::min(polygon_line[r].y(), polygon_line[s].y()));

  int max_y = std::max(std::max(polygon_line[p].y(), polygon_line[q].y()),
                       std::max(polygon_line[r].y(), polygon_line[s].y()));

  Point_2 most_left = Point_2(min_x, min_y);
  Point_2 most_right = Point_2(max_x, max_y);

  std::list<Point_2> points_in_box;
  Fuzzy_iso_box exact_range(most_left, most_right);
  kdtree.search(std::back_inserter(points_in_box), exact_range);

  std::vector<Segment_2> box_segs;
  Segment_2 left = Segment_2(Point_2(min_x, max_y), Point_2(min_x, min_y));
  box_segs.push_back(left);
  Segment_2 down = Segment_2(Point_2(min_x, min_y), Point_2(max_x, min_y));
  box_segs.push_back(down);
  Segment_2 right = Segment_2(Point_2(max_x, min_y), Point_2(max_x, max_y));
  box_segs.push_back(right);
  Segment_2 up = Segment_2(Point_2(max_x, max_y), Point_2(min_x, max_y));
  box_segs.push_back(up);

  for (std::list<Point_2>::iterator it = points_in_box.begin();
       it != points_in_box.end(); ++it) {
    Point_2 cur_point = *it;

    if (on_box_boundary(cur_point, box_segs) &&
        (cur_point == polygon_line[p] || cur_point == polygon_line[q] ||
         cur_point == polygon_line[r] || cur_point == polygon_line[s])) {
      continue;
    }

    int orig_idx = original_idx[cur_point];
    int real_idx = (offset[orig_idx] + orig_idx) % polygon_line.size();
    int prev_idx = (real_idx == 0) ? (polygon_line.size() - 1) : (real_idx - 1);
    int nxt_idx = (real_idx + 1) % polygon_line.size();
    Point_2 prev_2_cur_point = polygon_line[prev_idx];
    Point_2 nxt_2_cur_point = polygon_line[nxt_idx];

    Segment_2 seg1 = Segment_2(prev_2_cur_point, cur_point);
    Segment_2 seg2 = Segment_2(cur_point, nxt_2_cur_point);

    if ((prev_2_cur_point != polygon_line[q] && cur_point != polygon_line[q] &&
         prev_2_cur_point != polygon_line[s] && cur_point != polygon_line[s] &&
         CGAL::do_intersect(seg1, qs)) ||
        (cur_point != polygon_line[q] && nxt_2_cur_point != polygon_line[q] &&
         cur_point != polygon_line[s] && nxt_2_cur_point != polygon_line[s] &&
         CGAL::do_intersect(seg2, qs)) ||
        (prev_2_cur_point != polygon_line[p] && cur_point != polygon_line[p] &&
         prev_2_cur_point != polygon_line[r] && cur_point != polygon_line[r] &&
         CGAL::do_intersect(seg1, pr)) ||
        (cur_point != polygon_line[p] && nxt_2_cur_point != polygon_line[p] &&
         cur_point != polygon_line[r] && nxt_2_cur_point != polygon_line[r] &&
         CGAL::do_intersect(seg2, pr))) {
      return false;
    }
  }

  offset[original_idx[polygon_line[q]]]++;
  offset[original_idx[polygon_line[r]]]--;
  if (offset[original_idx[polygon_line[r]]] < 0) {
    offset[original_idx[polygon_line[r]]] += polygon_line.size();
  }

  std::swap(polygon_line[q], polygon_line[r]);

  return true;
}
