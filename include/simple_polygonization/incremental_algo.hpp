#pragma once
#include "../common.hpp"
#include "../utils.hpp"

/**
 * @brief                           Implements the incremental algorithm for
 *                                  point-set polygonization
 *
 * @param polygon_line              Polygon line formed by the incremental algo
 * @param convex_hull               Convex hull formed by the incremental algo
 * @param points                    The point-set given
 * @param initialization            Option for how to sort the point-set
 * @param visible_edge_selection    Option for how to choose the visible edge
 */
void run_incremental_algo(Polygon_2 &polygon_line, Polygon_2 &convex_hull,
                          std::vector<Point_2> &points,
                          INIT_OPTION_INCR initialization,
                          VISIBLE_EDGE_CHOICE visible_edge_selection);

/**
 * @brief Checks if new point is visible in respect to an edge in the polygon
 *
 * @param polygon  the polygon
 * @param new_point new point to check
 * @param left_point source of the edge (segment)
 * @param right_point target of the edge (segment)
 * @return true if is visible
 * @return false otherwise
 */
bool is_visible(Polygon_2 &polygon, Point_2 &new_point, Point_2 &left_point,
                Point_2 &right_point);

/**
 * @brief Expands convex_hull given a new point by breaking a red segment
 *
 * @param convex_hull the expanded convex hull
 * @param red_segments red (breakable) segments
 * @param new_point new point to insert
 */
void expand_convex_hull(Polygon_2 &convex_hull,
                        std::vector<Segment_2> &red_segments,
                        Point_2 new_point);

/**
 * @brief Expands polygon line given a new point
 *
 *
 * @param polygon_line The expanded polygon line
 * @param red_segments red (breakable) segments on CH
 * @param new_point new point to insert
 * @param initialization ossible choice for next visible edge
 */
void expand_polygon_line(Polygon_2 &polygon_line,
                         std::vector<Segment_2> &red_segments,
                         Point_2 new_point, VISIBLE_EDGE_CHOICE initialization);