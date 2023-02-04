#include "../include/graham_scan.hpp"

static int orientation(Point_2 a, Point_2 b, Point_2 c) {
  ll v = (ll)a.x() * ((ll)b.y() - (ll)c.y()) +
         (ll)b.x() * ((ll)c.y() - (ll)a.y()) +
         (ll)c.x() * ((ll)a.y() - (ll)b.y());
  if (v < 0) return -1;  // clockwise
  if (v > 0) return +1;  // counter-clockwise
  return 0;
}

static bool cw(Point_2 a, Point_2 b, Point_2 c) {
  int o = orientation(a, b, c);
  return (o < 0) || (o == 0);
}

static bool collinear(Point_2 a, Point_2 b, Point_2 c) {
  return orientation(a, b, c) == 0;
}

void graham_scan(std::vector<Point_2> &points, std::vector<int> &indices) {
  std::vector<Point_2> tmp_vec = points;
  Point_2 p0 =
      *min_element(points.begin(), points.end(), [](Point_2 a, Point_2 b) {
        return std::make_pair(a.y(), a.x()) < std::make_pair(b.y(), b.x());
      });

  sort(points.begin(), points.end(), [&p0](const Point_2 &a, const Point_2 &b) {
    int o = orientation(p0, a, b);
    if (o == 0)
      return (p0.x() - a.x()) * (p0.x() - a.x()) +
                 (p0.y() - a.y()) * (p0.y() - a.y()) <
             (p0.x() - b.x()) * (p0.x() - b.x()) +
                 (p0.y() - b.y()) * (p0.y() - b.y());
    return o < 0;
  });

  int i = (int)points.size() - 1;
  while (i >= 0 && collinear(p0, points[i], points.back())) i--;
  std::reverse(points.begin() + i + 1, points.end());

  std::vector<Point_2> st;
  for (int i = 0; i < (int)points.size(); i++) {
    while (st.size() > 1 && !cw(st[st.size() - 2], st.back(), points[i]))
      st.pop_back();
    st.push_back(points[i]);
  }
  points = tmp_vec;
  std::map<Point_2, int> mp;
  for (int i = 0; i < points.size(); ++i) {
    mp[points[i]] = i;
  }
  for (int i = 0; i < st.size(); ++i) {
    indices.push_back(mp[st[i]]);
  }
  std::reverse(indices.begin(), indices.end());
}