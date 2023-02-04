#pragma once
#include "common.hpp"
#include "graham_scan.hpp"
#include "pick_algo.hpp"

void print_polygon(Polygon_2 &polygon, bool plotter_format);

/**
 * @brief Parses arguments from stdin and performs some validity check.
 *
 * @param argc                  Num of command line arguments
 * @param argv                  Command line arguments
 * @param input_file            Name of input file
 * @param output_file           Name of output file
 * @param algorithm             Algorithm to run
 * @param edge_selection        Edge selection for the algorithm
 * @param initialization        Init option of incremental/onion algorithm
 * @param onion_initialization  Init option of onion algorithm
 * @param testing_mode          Testing the algorithms (print on results.txt)
 * @return                      True iff all arguments are correctly given
 * @return                      False otherwise
 */
bool parse_arguments(int argc, char *argv[], char *input_file,
                     char *output_file, ALGORITHM *algorithm,
                     INIT_ALGO *init_algo, int *L, OPTIMIZATION *optimization,
                     double *threshold, ANNEALING_OPTION *annealing,
                     double *alpha, double *beta, double *ro, bool *elitism,
                     bool *testing_mode);
/**
 * @brief Prints vertices and edges, area, and the ratio of polygon to convex
 * hull area
 *
 * @param polygon_line Polygon constructed from one of the algorithms
 * @param str_algo Algorithm's name
 * @param edge_selection Edge selection of the algorithm
 * @param initialization Initialization of the incremental or onion algorithm
 * @param onion_initialization Init option of the onion algorithm
 * @param construction_time Construction time of polygon line
 */
void print_output(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                  char *str_algo, OPTIMIZATION optimization,
                  ANNEALING_OPTION annealing, char *output_file,
                  double area_initial, double ratio_initial,
                  int construction_time);

/**
 * @brief               Reads the input data
 *
 * @param fp            Fp refers to the input file
 * @param line1         First line of input (contains some metadata)
 * @param line2         Second line of input (some more metadata)
 * @param points        Point-set to polygonize
 */
void read_input(FILE *fp, char *line1, char *line2,
                std::vector<Point_2> &points);

/**
 * @brief Merge sort the points according to the initialization option
 *
 * @param points            Given point-set to sort
 * @param begin             Start index on points vector
 * @param end               End index on points vector
 * @param initialization    How to sort the points option
 */
void merge_sort(std::vector<Point_2> &points, int begin, int end,
                INIT_OPTION_INCR initialization);

/**
 * @brief           Checks if the given segment intersects with any of the
 *                  polygon edges at their corner
 *
 * @param polygon   Polygon to check its edges for intersection
 * @param segment   Segment to check if intersects with any polygon edge
 * @return true     Iff intersect
 * @return false    Otherwise
 */
bool do_intersect_corner(Polygon_2 &polygon, Segment_2 segment);

/**
 * @brief           Checks if the given segment intersects with any of the
 *                  polygon edges at their middle
 *
 * @param polygon   Polygon to check its edges for intersection
 * @param segment   Segment to check if intersects with any polygon edge
 * @param edge_lr   The segment where the middle lies on in the polygon
 * @return true     Iff intersect
 * @return false    Otherwise
 */
bool do_intersect_middle(Polygon_2 &polygon, Segment_2 segment,
                         Segment_2 edge_lr);

/**
 * @brief Get the convex hull from the given point-set using
 *        custom Graham Scan implementation

 * @param points Point-set
 * @param convex_hull Resulted convex hull
 */
void get_convex_hull(std::vector<Point_2> &points, Polygon_2 &convex_hull);