#pragma once
#include "../common.hpp"
#include "../utils.hpp"

/**
 * @brief               Implementation of the convex_hull algorithm for
 *                      point-set polygonization
 *
 * @param polygon_line  Polygon line formed by the ch2polyline algo
 * @param convex_hull   Convex hull formed by the ch2polyline algo
 * @param points        The point-set to polygonize
 */
void ch2polyline_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                      std::vector<Point_2> &points,
                      VISIBLE_EDGE_CHOICE visible_edge_choice);

/**
 * @brief overloading the implementation of ch2polyline_algo
 * with the restriction that marked segments must be in the polygone line
 * @param polygon_line the resulted polygon line
 * @param convex_hull Convex hull formed by the ch2polyline algo
 * @param points The point-set to polygonize
 * @param marked_segments edges that must be in the polygonn line
 */
void ch2polyline_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                      std::vector<Point_2> &points,
                      std::vector<Segment_2> marked_segments);