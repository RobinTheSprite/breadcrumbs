//
// Created by Mark on 5/23/2020.
//

#ifndef BREADCRUMBS_BREADCRUMBS_H
#define BREADCRUMBS_BREADCRUMBS_H

#include <vector>
#include <deque>
#include <memory>

/*
 * Stores all the information necessary to complete
 * a run of the algorithm.
 */
struct MatrixPoint
{
    long x = -1;
    long y = -1;
    double movementCost = 0;
    double totalCost = 0;
    std::pair<long, long> parent = {-1, -1};
    bool visited = false;
};

/*
 * All of the weights given in params.json
 */
struct Weights
{
    double unitsPerPixel;
    int gradeBase;
    int gradeRadius;
    double movementCostXY;
    double movementCostZ;
    double heuristicXY;
    double heuristicZ;
};

// A 2D matrix of floats
using Matrix = std::vector<std::vector<float>>;

/*
 * Using a set of weights, a set of points to pass through, a matrix of elevation
 * data, and a matrix of extra accumulated weighted data layers, computes the shortest
 * path between each consecutive point.
 */
std::vector<std::vector<int>> getShortestPath(const Matrix & elevationMatrix,
                                              const Matrix & costMatrix,
                                              std::deque<MatrixPoint> controlPoints,
                                              const Weights &weights);

#endif //BREADCRUMBS_BREADCRUMBS_H
