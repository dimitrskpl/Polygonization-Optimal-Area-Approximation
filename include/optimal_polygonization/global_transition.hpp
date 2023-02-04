#include "../common.hpp"

/**
 * @brief Performs global transition in Local Search optimization
 *
 * @param polygon_line the resulted polygon line
 * @return true if transition applied
 * @return false otherwise
 */
bool global_transition(Polygon_2 &polygon_line);

/**
 * @brief An overload version of Global Transition for the
 *        "Subdivision to subproblems"
 *
 * @param polygon_line the resulted polygon line
 * @param marked_points marked points
 * @return true if transition applied
 * @return false otherwise
 */
bool global_transition(Polygon_2 &polygon_line,
                       std::vector<Point_2> &marked_points);