#include "../common.hpp"
#include "../graham_scan.hpp"
#include "../utils.hpp"
#include "simulated_annealing_algo.hpp"

/**
 * @brief implements simulated anealing with subdivision
 *
 * @param points the point set
 * @param polygon_line the resulted polygon line
 * @param optimization min or max
 * @param L number of iterations
 * @param init_algo  Initial polygon from onion or ch2poly algorithm
 */
void subdivision(std::vector<Point_2> &points, Polygon_2 &polygon_line,
                 OPTIMIZATION optimization, int L, INIT_ALGO init_algo);