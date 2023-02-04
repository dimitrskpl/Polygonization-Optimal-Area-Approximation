#pragma once

#include "../common.hpp"
#include "../simple_polygonization/ch2polyline_algo.hpp"
#include "../simple_polygonization/onion_algo.hpp"
#include "../utils.hpp"

/**
 * @brief Local Search optimization algorithm.
 *
 * @param points        the point-set to polygonize
 * @param polygon_line  the resulted polygon line
 * @param L             number of iterations
 * @param optimization  min or max
 * @param threshold     threshold
 * @param area_initial  for metrics
 * @param ratio_initial for metrics
 * @param init_algo     Initial polygon from onion or ch2poly algorithm
 */
void local_search_algo(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                       int L, OPTIMIZATION optimization, double threshold,
                       double &area_initial, double &ratio_initial,
                       INIT_ALGO init_algo);