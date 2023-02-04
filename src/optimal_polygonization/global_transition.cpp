#include "../../include/optimal_polygonization/global_transition.hpp"

bool global_transition(Polygon_2 &polygon_line) {
  int q = rand() % polygon_line.size();
  int s;
  while ((s = (rand() % polygon_line.size())) == q)
    ;

  int p = (q == 0) ? (polygon_line.size() - 1) : (q - 1);
  int r = (q + 1) % polygon_line.size();
  int t = (s + 1) % polygon_line.size();

  Segment_2 pr = Segment_2(polygon_line[p], polygon_line[r]);
  Segment_2 sq = Segment_2(polygon_line[s], polygon_line[q]);
  Segment_2 qt = Segment_2(polygon_line[q], polygon_line[t]);

  if (CGAL::do_intersect(pr, sq) || CGAL::do_intersect(pr, qt)) {
    return false;
  }

  Point_2 point_t = polygon_line[t];
  Point_2 point_q = polygon_line[q];
  polygon_line.erase(polygon_line.begin() + q);
  int t_pos = std::find(polygon_line.begin(), polygon_line.end(), point_t) -
              polygon_line.begin();
  polygon_line.insert(polygon_line.begin() + t_pos, point_q);

  if (!polygon_line.is_simple()) {
    return false;
  }

  return true;
}

/* === Global Transition version for "subdivision to subsets" === */
bool global_transition(Polygon_2 &polygon_line,
                       std::vector<Point_2> &marked_points) {
  int q = rand() % polygon_line.size();
  int s;

  while ((s = (rand() % polygon_line.size())) == q)
    ;

  int p = (q == 0) ? (polygon_line.size() - 1) : (q - 1);
  int r = (q + 1) % polygon_line.size();
  int t = (s + 1) % polygon_line.size();

  std::vector<Segment_2> check_segments;
  check_segments.push_back(Segment_2(polygon_line[s], polygon_line[t]));
  check_segments.push_back(Segment_2(polygon_line[r], polygon_line[q]));
  check_segments.push_back(Segment_2(polygon_line[q], polygon_line[p]));

  for (int i = 0; i < marked_points.size(); i += 2) {
    for (int j = 0; j < 3; ++j) {
      Point_2 source = check_segments[j].source();
      Point_2 target = check_segments[j].target();
      if ((source == marked_points[i] || source == marked_points[i + 1]) &&
          (target == marked_points[i] || target == marked_points[i + 1])) {
        return false;
      }
    }
  }

  Segment_2 pr = Segment_2(polygon_line[p], polygon_line[r]);
  Segment_2 sq = Segment_2(polygon_line[s], polygon_line[q]);
  Segment_2 qt = Segment_2(polygon_line[q], polygon_line[t]);

  if (CGAL::do_intersect(pr, sq) || CGAL::do_intersect(pr, qt)) {
    return false;
  }

  Point_2 point_t = polygon_line[t];
  Point_2 point_q = polygon_line[q];
  polygon_line.erase(polygon_line.begin() + q);
  int t_pos = std::find(polygon_line.begin(), polygon_line.end(), point_t) -
              polygon_line.begin();
  polygon_line.insert(polygon_line.begin() + t_pos, point_q);

  if (!polygon_line.is_simple()) {
    return false;
  }

  return true;
}