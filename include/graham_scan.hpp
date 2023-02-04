#pragma once

#include "common.hpp"

/**
 * @brief           Custom implementation of Graham Scan algorithm to include
 *                  collinear points on the convex hull
 *
 * @param points    The point-set
 * @param indices   Indices of points (in the points vector) that belong to the
 *                  convex hull
 */
void graham_scan(std::vector<Point_2> &points, std::vector<int> &indices);
