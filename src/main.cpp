#include <chrono>

#include "../include/optimal_polygonization/ant_colony_algo.hpp"
#include "../include/optimal_polygonization/local_search_algo.hpp"
#include "../include/optimal_polygonization/simulated_annealing_algo.hpp"
#include "../include/optimal_polygonization/subdivision.hpp"
#include "../include/simple_polygonization/ch2polyline_algo.hpp"
#include "../include/simple_polygonization/incremental_algo.hpp"
#include "../include/simple_polygonization/onion_algo.hpp"
#include "../include/utils.hpp"

int main(int argc, char *argv[]) {
  std::set<int> valid_argcs = {12, 13, 14, 15, 18, 19};
  if (valid_argcs.find(argc) == valid_argcs.end()) {
    fprintf(stderr,
            "Usage: $./optimal_polygon -i <point set input file>\n"
            "\t-o <output file>\n"
            "\t-algorithm <local_search/simulated_annealing/ant_colony>\n"
            "\t-init_algo <onion/ch2poly> [except ant_colony]\n"
            "\t-L [L parameter according to algorithm]\n"
            "\t-max [maximal area polygonization]\n"
            "\t-min [minimal area polygonization]\n"
            "\t-threshold <double> [in local search]\n"
            "\t-annealing <local/global/subdivision in simulated annealing>\n"
            "\t-alpha <double> -beta <double> -rho <double> -elitism <0 or 1> "
            "[alpha, beta, rho, elitism only in ant_colony algorithm]\n");
    return -1;
  }

  char input_file[MAXBUF], output_file[MAXBUF];

  ALGORITHM algorithm;
  INIT_ALGO init_algo = ONION_ALGO;
  OPTIMIZATION optimization;
  ANNEALING_OPTION annealing;
  int L;
  double threshold, alpha, beta, rho;
  bool elitism;

  bool testing_mode = false;
  if (!parse_arguments(argc, argv, input_file, output_file, &algorithm,
                       &init_algo, &L, &optimization, &threshold, &annealing,
                       &alpha, &beta, &rho, &elitism, &testing_mode)) {
    return EXIT_FAILURE;
  }
  int i, x, y;
  char line1[MAXBUF], line2[MAXBUF];
  FILE *fp = fopen(input_file, "r");
  std::vector<Point_2> points;

  if (fp == NULL) {
    fprintf(stderr, "Error opening input file.\n");
    return EXIT_FAILURE;
  }

  read_input(fp, line1, line2, points);

  Polygon_2 polygon_line, convex_hull;
  double area_initial, ratio_initial;
  char str_algo[50];

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  switch (algorithm) {
    case LOCAL_SEARCH:
      local_search_algo(points, polygon_line, L, optimization, threshold,
                        area_initial, ratio_initial, init_algo);
      sprintf(str_algo, "local_search");
      break;
    case SIMULATED_ANNEALING:
      if (annealing != SUBDIVISION) {
        simulated_annealing_algo(points, polygon_line, annealing, optimization,
                                 L, area_initial, ratio_initial, init_algo);
      } else {
        subdivision(points, polygon_line, optimization, L, init_algo);
      }
      sprintf(str_algo, "simulated_annealing");
      break;
    case ANT_COLONY:
      ant_colony_algo(points, polygon_line, optimization, L, alpha, beta, rho,
                      elitism);
      sprintf(str_algo, "ant_colony");
      break;
    default:
      fprintf(stderr, "Invalid algorithm given!\n");
  }

  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_ms = (end - start);
  int construction_time =
      (int)(std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                .count());

  if (!testing_mode) {
    get_convex_hull(points, convex_hull);
    print_output(polygon_line, convex_hull, str_algo, optimization, annealing,
                 output_file, area_initial, ratio_initial, construction_time);
  } else {  // Only in testing mode!
    // print some info to check if test case passed
    FILE *rfp = fopen("optimal_results.txt", "w");
    fprintf(rfp, "%lu ", polygon_line.size());
    fprintf(rfp, "%c", (polygon_line.is_simple()) ? ('Y') : ('N'));
  }

  return EXIT_SUCCESS;
}