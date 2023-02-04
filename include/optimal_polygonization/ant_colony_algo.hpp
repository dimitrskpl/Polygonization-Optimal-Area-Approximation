#include "../common.hpp"
#include "../simple_polygonization/incremental_algo.hpp"
#include "../utils.hpp"

/**
 * @brief Implementation of Ant Colony Optimization for
 *        optimal area polygonizations.
 *
 * @param points the point set
 * @param polygon_line the resulted optimal polygon line
 * @param optimization min or max
 * @param L            number of cycles to perform
 * @param alpha        alpha parameter - double
 * @param beta         beta parameter - double in [1,5]
 * @param rho          rho parameter - double
 * @param elitism      elitism <0 or 1>
 */
void ant_colony_algo(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                     OPTIMIZATION optimization, int L, double alpha,
                     double beta, double rho, bool elitism);