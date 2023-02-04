#include "../common.hpp"

/**
 * @brief Performs global transition in Local Search optimization
 *
 * @param polygon_line the resulted polygon line
 * @param kdtree        KD-Tree for optimization
 * @param original_idx  Keeping indexes and offsets
 * @param offset        for further optimization
 * @return true if transition applied
 * @return false otherwise
 */
bool local_transition(Polygon_2 &polygon_line, KD_Tree &kdtree,
                      std::map<Point_2, int> &original_idx,
                      std::vector<int> &offset);
