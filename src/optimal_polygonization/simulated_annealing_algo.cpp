#include "../../include/optimal_polygonization/simulated_annealing_algo.hpp"

static double polygon_energy(double convex_hull_area, Polygon_2 &polygon_line,
                             OPTIMIZATION optimization) {
  double minimization_energy =
      polygon_line.size() * (fabs(polygon_line.area()) / convex_hull_area);

  if (optimization == MINIMAL_AREA) {
    return minimization_energy;
  } else {
    return polygon_line.size() - minimization_energy;
  }
}

void simulated_annealing_algo(std::vector<Point_2> &points,
                              Polygon_2 &polygon_line, ANNEALING_OPTION option,
                              OPTIMIZATION optimization, int L,
                              double &area_initial, double &ratio_initial,
                              INIT_ALGO init_algo) {
  Polygon_2 convex_hull;
  if (init_algo == ONION_ALGO) {
    onion_algo(polygon_line, convex_hull, points, RANDOM_POINT);
  } else {
    ch2polyline_algo(polygon_line, convex_hull, points, RANDOM_EDGE);
  }

  area_initial = fabs(polygon_line.area());
  ratio_initial = area_initial / fabs(convex_hull.area());

  std::map<Point_2, int> original_idx;
  std::vector<int> offset(points.size(), 0);

  KD_Tree kdtree;
  for (int i = 0; i < points.size(); i++) {
    kdtree.insert(points[i]);
  }

  for (int i = 0; i < polygon_line.size(); ++i) {
    original_idx[polygon_line[i]] = i;
  }

  double convex_hull_area = fabs(convex_hull.area());
  double last_energy =
      polygon_energy(convex_hull_area, polygon_line, optimization);
  double T = 1.0;

  srand(time(NULL));
  double R = ((double)rand() / (double)RAND_MAX);
  while (T >= 0) {
    Polygon_2 transition_polygon = polygon_line;
    std::vector<int> transition_offset = offset;

    bool transitioned;
    switch (option) {
      case LOCAL:
        transitioned = local_transition(transition_polygon, kdtree,
                                        original_idx, transition_offset);
        break;
      case GLOBAL:
        transitioned = global_transition(transition_polygon);
        break;
      default:
        break;
    }
    if (transitioned) {
      double cur_energy =
          polygon_energy(convex_hull_area, transition_polygon, optimization);
      double DE = cur_energy - last_energy;
      if (DE < 0 || ((exp(-DE / T) >= R))) {
        polygon_line = transition_polygon;
        if (option == LOCAL) {
          offset = transition_offset;
        }
      }
    }
    T = T - 1.0 / L;
  }
}

/* === Simulated Annealing version for Subdivision to Subproblems === */
void simulated_annealing_algo(std::vector<Point_2> &points,
                              Polygon_2 &polygon_line,
                              OPTIMIZATION optimization, int L,
                              std::vector<Segment_2> &marked_segments,
                              INIT_ALGO init_algo) {
  Polygon_2 convex_hull;

  std::vector<Point_2> marked_points;
  for (int i = 0; i < marked_segments.size(); ++i) {
    marked_points.push_back(marked_segments[i].source());
    marked_points.push_back(marked_segments[i].target());
  }

  if (init_algo == ONION_ALGO) {
    onion_algo(polygon_line, convex_hull, points, marked_segments);
  } else {
    ch2polyline_algo(polygon_line, convex_hull, points, marked_segments);
  }

  double convex_hull_area = fabs(convex_hull.area());

  double last_energy =
      polygon_energy(convex_hull_area, polygon_line, optimization);
  double T = 1.0;
  srand(0);
  double R = ((double)rand() / (double)RAND_MAX);

  while (T >= 0) {
    Polygon_2 transition_polygon = polygon_line;

    bool transitioned = global_transition(transition_polygon, marked_points);
    if (transitioned) {
      double cur_energy =
          polygon_energy(convex_hull_area, transition_polygon, optimization);
      double DE = cur_energy - last_energy;
      if (DE < 0 || ((exp(-DE / T) >= R))) {
        polygon_line = transition_polygon;
      }
    }
    T = T - 1.0 / L;
  }
}

void optimize_subdivision_polygon(std::vector<Point_2> &points,
                                  Polygon_2 &polygon_line,
                                  OPTIMIZATION optimization, int L) {
  Polygon_2 convex_hull;
  get_convex_hull(points, convex_hull);

  std::map<Point_2, int> original_idx;
  std::vector<int> offset(points.size(), 0);

  KD_Tree kdtree;
  for (int i = 0; i < points.size(); i++) {
    kdtree.insert(points[i]);
  }

  for (int i = 0; i < polygon_line.size(); ++i) {
    original_idx[polygon_line[i]] = i;
  }

  double convex_hull_area = fabs(convex_hull.area());

  double last_energy =
      polygon_energy(convex_hull_area, polygon_line, optimization);
  double T = 1.0;
  double R = ((double)rand() / (double)RAND_MAX);
  int iter = 0;
  while (T >= 0) {
    iter++;
    Polygon_2 transition_polygon = polygon_line;
    std::vector<int> transition_offset = offset;

    bool transitioned;
    transitioned = local_transition(transition_polygon, kdtree, original_idx,
                                    transition_offset);
    if (transitioned) {
      double cur_energy =
          polygon_energy(convex_hull_area, transition_polygon, optimization);
      double DE = cur_energy - last_energy;
      if (DE < 0 || ((exp(-DE / T) >= R))) {
        polygon_line = transition_polygon;
        offset = transition_offset;
      }
    }
    T = T - 1.0 / L;
  }
}