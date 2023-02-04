#pragma once

#include "../common.hpp"
#include "../graham_scan.hpp"
#include "../simple_polygonization/ch2polyline_algo.hpp"
#include "../simple_polygonization/onion_algo.hpp"
#include "../utils.hpp"
#include "global_transition.hpp"
#include "local_transition.hpp"

/**
 * @brief Simulated Annealing algorithm for optimal area polygonization.
 *
 * @param points the point set
 * @param polygon_line the resulted polygon line
 * @param option        annealing option (local, global, subdivision)
 * @param optimization  min or max
 * @param L             number of iterations
 * @param area_initial  for metrics
 * @param ratio_initial for metrics
 * @param init_algo     Initial polygon from onion or ch2poly algorithm
 */
void simulated_annealing_algo(std::vector<Point_2> &points,
                              Polygon_2 &polygon_line, ANNEALING_OPTION option,
                              OPTIMIZATION optimization, int L,
                              double &area_initial, double &ratio_initial,
                              INIT_ALGO init_algo);

/**
 * @brief Overloading implementation of SA algo,
 *        for Subdivision to Subproblems (annealing = subdivision)
 *
 * @param points the point set
 * @param polygon_line the resulted polygon line
 * @param optimization min or max
 * @param L number of iterations
 * @param marked_segments edges that must be in the polygonn line
 * @param init_algo Initial polygon from onion or ch2poly algorithm
 */
void simulated_annealing_algo(std::vector<Point_2> &points,
                              Polygon_2 &polygon_line,
                              OPTIMIZATION optimization, int L,
                              std::vector<Segment_2> &marked_segments,
                              INIT_ALGO init_algo);

/**
 * @brief implements simulated anealing with local transition to the
 * polygon_line
 *
 * @param points the point set
 * @param polygon_line the resulted polygon line
 * @param optimization min or max
 * @param L number of iterations
 */
void optimize_subdivision_polygon(std::vector<Point_2> &points,
                                  Polygon_2 &polygon_line,
                                  OPTIMIZATION optimization, int L);