#include "../../include/optimal_polygonization/ant_colony_algo.hpp"

#define T_0 1        // Initial pheromone
#define MAX_M 10010  // Max nodes for construction graph
#define PRIME 251

double T[MAX_M][MAX_M];   // Pheromones for each edge
double dT[MAX_M][MAX_M];  // Pheromone change for each edge
double H[MAX_M][MAX_M];   // Heuristic value for each edge
double P[MAX_M][MAX_M];   // Probability of each edge

int N_ANTS;

ll szudzik_pair(int x, int y) {
  return (x >= y ? (x * x) + x + y : (y * y) + x);
}

ll szudzik_hash(int x, int y) {
  int a = (x >= 0 ? 2 * x : (-2 * x) - 1);
  int b = (y >= 0 ? 2 * y : (-2 * y) - 1);
  int c = szudzik_pair(a, b) * 0.5;

  if (((a >= 0) && (b < 0)) || ((a < 0) && (b >= 0))) {
    return -c - 1;
  }
  return c;
}

ll get_hash(Polygon_2 &polygon_line) {
  ll polygon_hash = 0;
  int start_idx;
  int max_x = -1, max_y = -1;
  for (int i = 0; i < polygon_line.size(); ++i) {
    if (max_x < (int)polygon_line[i].x() ||
        (max_x == (int)polygon_line[i].x() &&
         max_y < (int)polygon_line[i].y())) {
      start_idx = i;
      max_x = (int)polygon_line[i].x();
      max_y = (int)polygon_line[i].y();
    }
  }

  int p_idx = start_idx;
  int points_cnt = 1;
  while (points_cnt <= polygon_line.size()) {
    int x = (int)polygon_line[p_idx].x();
    int y = (int)polygon_line[p_idx].y();

    int point_hash = szudzik_hash(x, y);
    polygon_hash = polygon_hash * PRIME + point_hash;
    p_idx = (p_idx + 1) % polygon_line.size();
    points_cnt++;
  }

  return polygon_hash;
}

std::map<ll, int> poly_idx_map;

static void initialize_triangle(std::vector<Point_2> &points,
                                Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                                std::vector<bool> &in_poly_line) {
  int idx = rand() % points.size();
  double min_dist_1 = -1, min_dist_2 = -1;
  int min_idx_1 = -1, min_idx_2 = -1;

  for (int i = 0; i < points.size(); ++i) {
    if (i == idx) {
      continue;
    }
    double dist = CGAL::squared_distance(points[idx], points[i]);

    if (min_dist_1 == -1) {
      min_dist_1 = dist;
      min_idx_1 = i;
    } else if (min_dist_2 == -1) {
      min_dist_2 = dist;
      min_idx_2 = i;
    } else if (min_dist_1 >= dist) {
      min_dist_2 = min_dist_1;
      min_idx_2 = min_idx_1;
      min_dist_1 = dist;
      min_idx_1 = i;
    } else if (min_dist_2 > dist) {
      min_dist_2 = dist;
      min_idx_2 = i;
    }
  }

  polygon_line.push_back(points[idx]);
  polygon_line.push_back(points[min_idx_1]);
  polygon_line.push_back(points[min_idx_2]);
  in_poly_line[idx] = true;
  in_poly_line[min_idx_1] = true;
  in_poly_line[min_idx_2] = true;

  convex_hull = polygon_line;
}

static void get_feasible_points(std::vector<Point_2> &points,
                                Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                                std::vector<int> &feasible_points) {
  for (int i = 0; i < points.size(); ++i) {
    bool in_poly_line = false;
    for (int j = 0; j < polygon_line.size(); ++j) {
      if (polygon_line[j] == points[i]) {
        in_poly_line = true;
        break;
      }
    }

    if (in_poly_line) {
      continue;
    }

    Polygon_2 tmp_convex_hull = convex_hull;
    std::vector<Segment_2> red_segments;

    expand_convex_hull(tmp_convex_hull, red_segments, points[i]);

    bool feasible = true;
    for (int j = 0; j < points.size(); ++j) {
      in_poly_line = false;
      for (int k = 0; k < polygon_line.size(); ++k) {
        if (polygon_line[k] == points[j]) {
          in_poly_line = true;
          break;
        }
      }

      if (in_poly_line || (j == i)) {
        continue;
      }

      if (CGAL::bounded_side_2(tmp_convex_hull.begin(), tmp_convex_hull.end(),
                               points[j]) != CGAL::ON_UNBOUNDED_SIDE) {
        feasible = false;
        break;
      }
    }

    if (feasible == true) {
      feasible_points.push_back(i);
    }
  }
}

static int expand_solution(Polygon_2 &polygon_line,
                           std::vector<Segment_2> &red_segments,
                           Point_2 new_point, std::vector<int> &next_solutions,
                           std::vector<Polygon_2> &solutions_poly, bool ccw) {
  int poly_seg_src_idx = -1;
  int red_idx = 0;

  for (int i = 0; i < polygon_line.size(); ++i) {
    if (red_segments[red_idx].source() == polygon_line[i]) {
      poly_seg_src_idx = i;
      break;
    }
  }

  while (true) {
    int nxt_idx = (poly_seg_src_idx + 1) % polygon_line.size();
    Segment_2 poly_segment =
        Segment_2(polygon_line[poly_seg_src_idx], polygon_line[nxt_idx]);

    int insert_offset_idx = poly_seg_src_idx + 1;

    if (poly_segment == red_segments[red_idx]) {
      Polygon_2 solution = polygon_line;

      solution.insert(solution.begin() + insert_offset_idx, new_point);
      if (poly_idx_map.find(get_hash(solution)) == poly_idx_map.end()) {
        int idx = poly_idx_map.size();
        if (idx >= MAX_M) {
          return -1;
        }
        poly_idx_map[get_hash(solution)] = idx;
      }
      next_solutions.push_back(poly_idx_map[get_hash(solution)]);
      solutions_poly.push_back(solution);
    } else {
      if (is_visible(polygon_line, new_point, polygon_line[poly_seg_src_idx],
                     polygon_line[nxt_idx])) {
        Polygon_2 solution = polygon_line;
        solution.insert(solution.begin() + insert_offset_idx, new_point);
        if (poly_idx_map.find(get_hash(solution)) == poly_idx_map.end()) {
          int idx = poly_idx_map.size();
          if (idx >= MAX_M) {
            return -1;
          }
          poly_idx_map[get_hash(polygon_line)] = idx;
        }
        next_solutions.push_back(poly_idx_map[get_hash(solution)]);
        solutions_poly.push_back(solution);
      }
    }
    if (poly_segment.target() == red_segments[red_idx].source()) {
      if (++red_idx == red_segments.size()) {
        break;
      }
    }
    poly_seg_src_idx = (poly_seg_src_idx + 1) % polygon_line.size();
  }
  return 0;
}

// Based on incremental algorithm, adjusted for Ant Colony //
static int build_solution(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                          std::vector<Polygon_2> &path, double alpha,
                          double beta, double rho, OPTIMIZATION optimization,
                          bool elitism) {
  Polygon_2 convex_hull;
  std::vector<bool> in_poly_line(points.size(), false);

  bool ccw = false;
  initialize_triangle(points, polygon_line, convex_hull, in_poly_line);

  if (poly_idx_map.find(get_hash(polygon_line)) == poly_idx_map.end()) {
    int idx = poly_idx_map.size();
    if (idx >= MAX_M) {
      return -1;
    }
    poly_idx_map[get_hash(polygon_line)] = idx;
  }

  int points_left = points.size() - 3;

  while (points_left > 0) {
    ccw = false;
    int start_idx = poly_idx_map[get_hash(polygon_line)];
    if (start_idx >= MAX_M) {
      return -1;
    }
    if (!path.empty() && !elitism) {
      Polygon_2 prev_sol = path.back();
      int prev_sol_idx = poly_idx_map[get_hash(prev_sol)];
      if (optimization == MINIMAL_AREA) {
        dT[prev_sol_idx][start_idx] += 1 / fabs(prev_sol.area());
      } else {
        dT[prev_sol_idx][start_idx] += fabs(prev_sol.area());
      }
    }
    path.push_back(polygon_line);

    double start_area = fabs(polygon_line.area());
    std::vector<int> feasible_points;
    get_feasible_points(points, polygon_line, convex_hull, feasible_points);
    std::vector<double> probabilities;
    std::vector<Polygon_2> all_solutions_poly;

    for (int j = 0; j < feasible_points.size(); ++j) {
      Polygon_2 tmp_polygon = polygon_line;
      Polygon_2 tmp_convex_hull = convex_hull;

      std::vector<Segment_2> red_segments;
      expand_convex_hull(tmp_convex_hull, red_segments,
                         points[feasible_points[j]]);

      std::vector<int> next_solutions_idxs;
      std::vector<Polygon_2> solutions_poly;

      if (expand_solution(tmp_polygon, red_segments, points[feasible_points[j]],
                          next_solutions_idxs, solutions_poly, ccw) == -1) {
        return -1;
      }

      all_solutions_poly.insert(all_solutions_poly.end(),
                                solutions_poly.begin(), solutions_poly.end());

      for (int sol = 0; sol < next_solutions_idxs.size(); ++sol) {
        int sol_idx = next_solutions_idxs[sol];
        if (H[start_idx][sol_idx] == 0) {
          H[start_idx][sol_idx] = fabs(solutions_poly[sol].area()) - start_area;
        }
        double prob = pow(T[start_idx][sol_idx], alpha) *
                      pow(H[start_idx][sol_idx], beta);
        probabilities.push_back(prob);
      }
    }
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(probabilities.begin(), probabilities.end());

    polygon_line = all_solutions_poly[d(gen)];
    std::vector<Point_2> polygon_points;
    Polygon_2 new_convex_hull;

    for (int p = 0; p < polygon_line.size(); ++p) {
      polygon_points.push_back(polygon_line[p]);
    }

    get_convex_hull(polygon_points, new_convex_hull);
    convex_hull = new_convex_hull;
    points_left--;
  }
  return 0;
}

static void initialize_parameters() {
  for (int i = 0; i < MAX_M; ++i) {
    for (int j = 0; j < MAX_M; ++j) {
      T[i][j] = T_0;
    }
  }
}

static double evaluate_solution(double solution_area, double ch_area) {
  return solution_area / ch_area;
}

static double compute_tij(double rho, double tij, double delta_tij) {
  return (1 - rho) * tij + delta_tij;
}

static void update_trails(double rho, bool elitism,
                          std::vector<Polygon_2> &optimal_path) {
  int M = poly_idx_map.size();
  if (!elitism) {
    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < M; ++j) {
        T[i][j] = compute_tij(rho, T[i][j], dT[i][j] / (double)N_ANTS);
      }
    }
  } else {
    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < M; ++j) {
        T[i][j] = compute_tij(rho, T[i][j], 0);
      }
    }
    int prev_idx = poly_idx_map[get_hash(optimal_path[0])];
    for (int i = 1; i < optimal_path.size(); ++i) {
      int cur_idx = poly_idx_map[get_hash(optimal_path[i])];
      T[prev_idx][cur_idx] += dT[prev_idx][cur_idx];
      prev_idx = cur_idx;
    }
  }
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < M; ++j) {
      dT[i][j] = 0;
    }
  }
}

void ant_colony_algo(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                     OPTIMIZATION optimization, int L, double alpha,
                     double beta, double rho, bool elitism) {
  srand(time(NULL));
  initialize_parameters();
  N_ANTS = (int)ceil(points.size() / 4);
  Polygon_2 convex_hull, optimal_polygon;
  get_convex_hull(points, convex_hull);
  std::vector<Polygon_2> optimal_path;
  double convex_hull_area = fabs(convex_hull.area());
  double optimal_area = -1, cycle0_ratio = -1, optimal_ratio = -1;
  bool m_out_of_bounds = false;

  int cycle;
  for (cycle = 0; cycle < L; ++cycle) {
    for (int k = 0; k < N_ANTS; ++k) {
      Polygon_2 solution;
      std::vector<Polygon_2> path;
      if (build_solution(points, solution, path, alpha, beta, rho, optimization,
                         elitism) == -1) {
        printf("\n----- SOLUTIONS OUT OF BOUNDS -----\n");
        printf("Generated %ld solutions!\n", poly_idx_map.size());
        fflush(stdout);
        m_out_of_bounds = true;
        break;
      }

      double solution_area = fabs(solution.area());

      double area_ratio = evaluate_solution(solution_area, convex_hull_area);
      if (optimization == MINIMAL_AREA &&
          (optimal_area == -1 || solution_area < optimal_area)) {
        optimal_area = solution_area;
        optimal_polygon = solution;
        optimal_ratio = area_ratio;
        optimal_path = path;
        if (!cycle) {
          cycle0_ratio = area_ratio;
        }
      } else if (optimization == MAXIMAL_AREA &&
                 (optimal_area == -1 || solution_area > optimal_area)) {
        optimal_area = solution_area;
        optimal_polygon = solution;
        optimal_ratio = area_ratio;
        optimal_path = path;
        if (!cycle) {
          cycle0_ratio = area_ratio;
        }
      }
    }
    if (m_out_of_bounds) {
      ++cycle;
      break;
    }
    update_trails(rho, elitism, optimal_path);
  }

  polygon_line = optimal_polygon;
}