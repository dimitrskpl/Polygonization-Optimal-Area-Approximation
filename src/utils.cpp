#include "../include/utils.hpp"

void get_convex_hull(std::vector<Point_2> &points, Polygon_2 &convex_hull) {
  std::vector<int> indices;

  graham_scan(points, indices);

  for (int i = 0; i < indices.size(); ++i) {
    convex_hull.push_back(points[indices[i]]);
  }
}

void print_polygon(Polygon_2 &polygon, bool plotter_format) {
  if (!plotter_format) {
    for (int i = 0; i < polygon.size(); ++i) {
      printf("(%d, %d)\t", (int)polygon[i].x(), (int)polygon[i].y());
    }
  } else {
    for (int i = 0; i < polygon.size(); ++i) {
      printf("%d, %d; ", (int)polygon[i].x(), (int)polygon[i].y());
    }
  }
  puts("");
}

bool parse_arguments(int argc, char *argv[], char *input_file,
                     char *output_file, ALGORITHM *algorithm,
                     INIT_ALGO *init_algo, int *L, OPTIMIZATION *optimization,
                     double *threshold, ANNEALING_OPTION *annealing,
                     double *alpha, double *beta, double *rho, bool *elitism,
                     bool *testing_mode) {
  bool local_search = false, simulated_annealing = false, ant_colony = false;
  bool threshold_given = false, annealing_given = false, alpha_given = false;
  bool beta_given = false, rho_given = false, elitism_given = false;
  bool optimization_given = false, L_given = false;
  int i = 1;

  while (i < argc) {
    if (!strcmp(argv[i], "-i")) {
      strcpy(input_file, argv[i + 1]);
    } else if (!strcmp(argv[i], "-o")) {
      strcpy(output_file, argv[i + 1]);
    } else if (!strcmp(argv[i], "-L")) {
      *L = atoi(argv[i + 1]);
      L_given = true;
    } else if (!strcmp(argv[i], "-alpha")) {
      *alpha = (double)atof(argv[i + 1]);
      alpha_given = true;
    } else if (!strcmp(argv[i], "-beta")) {
      *beta = (double)atof(argv[i + 1]);
      beta_given = true;
    } else if (!strcmp(argv[i], "-rho")) {
      *rho = (double)atof(argv[i + 1]);
      rho_given = true;
    } else if (!strcmp(argv[i], "-elitism")) {
      *elitism = (bool)atoi(argv[i + 1]);
      elitism_given = true;
    } else if (!strcmp(argv[i], "-max")) {
      if (optimization_given) {
        fprintf(stderr,
                "Argument error: You have to select only one optimization "
                "option (min or max).\n");
        return false;
      }
      *optimization = MAXIMAL_AREA;
      optimization_given = true;
      i++;
      continue;
    } else if (!strcmp(argv[i], "-min")) {
      if (optimization_given) {
        fprintf(stderr,
                "Argument error: You have to select only one optimization "
                "option (min or max).\n");
        return false;
      }
      *optimization = MINIMAL_AREA;
      optimization_given = true;
      i++;
      continue;
    } else if (!strcmp(argv[i], "-threshold")) {
      *threshold = (double)atof(argv[i + 1]);
      threshold_given = true;
    } else if (!strcmp(argv[i], "-annealing")) {
      if (!strcmp(argv[i + 1], "local")) {
        *annealing = LOCAL;
      } else if (!strcmp(argv[i + 1], "global")) {
        *annealing = GLOBAL;
      } else if (!strcmp(argv[i + 1], "subdivision")) {
        *annealing = SUBDIVISION;
      } else {
        fprintf(stderr, "Argument error: Invalid annealing option.\n");
        return false;
      }
      annealing_given = true;
    } else if (!strcmp(argv[i], "-algorithm")) {
      if (!strcmp(argv[i + 1], "local_search")) {
        local_search = true;
        *algorithm = LOCAL_SEARCH;
      } else if (!strcmp(argv[i + 1], "simulated_annealing")) {
        simulated_annealing = true;
        *algorithm = SIMULATED_ANNEALING;
      } else if (!strcmp(argv[i + 1], "ant_colony")) {
        ant_colony = true;
        *algorithm = ANT_COLONY;
      } else {
        fprintf(stderr, "Argument error: Invalid algorithm option.\n");
        return false;
      }
    } else if (!strcmp(argv[i], "-init_algo")) {
      if (!strcmp(argv[i + 1], "onion")) {
        *init_algo = ONION_ALGO;
      } else if (!strcmp(argv[i + 1], "ch2poly")) {
        *init_algo = CH2POLY_ALGO;
      } else {
        fprintf(stderr,
                "Argument error: Invalid initialization algorithm option.\n");
        return false;
      }
    } else if (!strcmp(argv[i], "-t")) {
      *testing_mode = true;
      i++;
      continue;
    } else {
      fprintf(stderr, "Argument error: Wrong argument given!\n");
      fprintf(stderr, "%s\n", argv[i]);
      return false;
    }
    i += 2;
  }
  if (!L_given) {
    fprintf(stderr,
            "Argument error: L argument needs to be specified for every "
            "algorithm.\n");
    return false;
  }
  if (!local_search && threshold_given) {
    fprintf(stderr,
            "Argument error: threashold argument only "
            "used with the "
            "local search algorithm.\n");
    return false;
  } else if (annealing_given && !simulated_annealing) {
    fprintf(stderr,
            "Argument error: annealing argument only "
            "used with the "
            "simulated annealing algorithm.\n");
    return false;
  } else if ((alpha_given | beta_given | rho_given | elitism_given) &&
             !ant_colony) {
    fprintf(stderr,
            "Argument error: alpha, beta, ro and elitism arguments only used "
            "with the ant colony algorithm.\n");
    return false;
  } else if (local_search && !threshold_given) {
    fprintf(
        stderr,
        "Argument error: Local Search algorithm needs threshold argument.\n");
    return false;
  } else if (simulated_annealing && !annealing_given) {
    fprintf(stderr,
            "Argument error: Simulated annealing algorithm needs annealing "
            "argument.\n");
    return false;
  } else if (ant_colony &&
             !(alpha_given && beta_given && rho_given && elitism_given)) {
    fprintf(stderr,
            "Argument error: Ant colony algorithm needs alpha, beta, ro and "
            "elitism arguments.\n");
    return false;
  }
  return true;
}

void print_output(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                  char *str_algo, OPTIMIZATION optimization,
                  ANNEALING_OPTION annealing, char *output_file,
                  double area_initial, double ratio_initial,
                  int construction_time) {
  FILE *fp = fopen(output_file, "w");
  fprintf(fp, "Optimal Area Polygonization\n");
  for (int i = 0; i < polygon_line.size(); ++i) {
    fprintf(fp, "%d %d\n", (int)polygon_line[i].x(), (int)polygon_line[i].y());
  }

  for (int i = 0; i < polygon_line.size(); ++i) {
    int nxt = (i + 1) % polygon_line.size();
    fprintf(fp, "%d %d %d %d\n", (int)polygon_line[i].x(),
            (int)polygon_line[i].y(), (int)polygon_line[nxt].x(),
            (int)polygon_line[nxt].y());
  }

  fprintf(fp, "Algorithm: %s", str_algo);
  if (optimization == MINIMAL_AREA) {
    fprintf(fp, "_min\n");
  } else {
    fprintf(fp, "_max\n");
  }

  double area = fabs(polygon_line.area());
  double ratio = area / fabs(convex_hull.area());

  if (strcmp(str_algo, "ant_colony") && annealing != SUBDIVISION) {
    fprintf(fp, "area_initial: %lf\n", area_initial);
    fprintf(fp, "area: %lf\n", area);
    fprintf(fp, "ratio_initial: %lf\n", ratio_initial);
    fprintf(fp, "ratio: %lf\n", ratio);
  } else {
    fprintf(fp, "area: %lf\n", area);
    fprintf(fp, "ratio: %lf\n", ratio);
  }
  fprintf(fp, "Construction time: %d ms\n", construction_time);
}

void read_input(FILE *fp, char *line1, char *line2,
                std::vector<Point_2> &points) {
  fgets(line1, MAXBUF, fp);
  fgets(line2, MAXBUF, fp);

  int i, x, y;
  while (fscanf(fp, "%d %d %d", &i, &x, &y) != EOF) {
    points.push_back(Point_2(x, y));
  }
}

static bool compare_option(const Point_2 &lhs, const Point_2 &rhs,
                           INIT_OPTION_INCR initialization) {
  if (initialization == X_DESCENDING) {
    return (lhs.x() == rhs.x()) ? (lhs.y() >= rhs.y()) : (lhs.x() > rhs.x());
  } else if (initialization == X_ASCENDING) {
    return (lhs.x() == rhs.x()) ? (lhs.y() <= rhs.y())
                                : (lhs.x() < rhs.x());  // change later
  } else if (initialization == Y_DESCENDING) {
    return (lhs.y() == rhs.y()) ? (lhs.x() >= rhs.x()) : (lhs.y() > rhs.y());
  } else {
    return (lhs.y() == rhs.y()) ? (lhs.x() <= rhs.x()) : (lhs.y() < rhs.y());
  }
}

static void merge(std::vector<Point_2> &points, int left, int mid, int right,
                  INIT_OPTION_INCR initialization) {
  int subarray_1 = mid - left + 1;
  int subarray_2 = right - mid;

  std::vector<Point_2> left_vec, right_vec;

  for (int i = 0; i < subarray_1; i++) left_vec.push_back(points[left + i]);
  for (int j = 0; j < subarray_2; j++) right_vec.push_back(points[mid + 1 + j]);

  int subarray_1_idx = 0, subarray_2_idx = 0;
  int merged_array_idx = left;

  while (subarray_1_idx < subarray_1 && subarray_2_idx < subarray_2) {
    if (compare_option(left_vec[subarray_1_idx], right_vec[subarray_2_idx],
                       initialization)) {
      points[merged_array_idx] = left_vec[subarray_1_idx];
      subarray_1_idx++;
    } else {
      points[merged_array_idx] = right_vec[subarray_2_idx];
      subarray_2_idx++;
    }
    merged_array_idx++;
  }

  while (subarray_1_idx < subarray_1) {
    points[merged_array_idx] = left_vec[subarray_1_idx];
    subarray_1_idx++;
    merged_array_idx++;
  }

  while (subarray_2_idx < subarray_2) {
    points[merged_array_idx] = right_vec[subarray_2_idx];
    subarray_2_idx++;
    merged_array_idx++;
  }
}

void merge_sort(std::vector<Point_2> &points, int begin, int end,
                INIT_OPTION_INCR initialization) {
  if (begin >= end) return;

  int mid = begin + (end - begin) / 2;
  merge_sort(points, begin, mid, initialization);
  merge_sort(points, mid + 1, end, initialization);
  merge(points, begin, mid, end, initialization);
}

bool do_intersect_corner(Polygon_2 &polygon, Segment_2 segment) {
  for (int i = 0; i < polygon.size(); ++i) {
    int nxt_idx = (i == polygon.size() - 1) ? (0) : (i + 1);  // loop cyclicly
    Segment_2 cur_segment = Segment_2(polygon[i], polygon[nxt_idx]);

    if (do_intersect(cur_segment, segment) &&
        !(cur_segment.source() == segment.target() ||
          cur_segment.target() == segment.target())) {
      return true;
    }
  }
  return false;
}

bool do_intersect_middle(Polygon_2 &polygon, Segment_2 segment,
                         Segment_2 edge_lr) {
  for (int i = 0; i < polygon.size(); ++i) {
    int nxt_idx = (i == polygon.size() - 1) ? (0) : (i + 1);  // loop cyclicly
    Segment_2 cur_segment = Segment_2(polygon[i], polygon[nxt_idx]);

    if (cur_segment == edge_lr) continue;
    if (CGAL::intersection(cur_segment, segment)) {
      return true;
    }
  }
  return false;
}