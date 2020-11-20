//
// Created by Mark on 5/23/2020.
//

#ifndef BREADCRUMBS_BREADCRUMBS_H
#define BREADCRUMBS_BREADCRUMBS_H

#include <vector>
#include <deque>
#include <memory>

struct MatrixPoint
{
    long x = -1;
    long y = -1;
    double movementCost = 0;
    double totalCost = 0;
    std::pair<long, long> parent = {-1, -1};
    bool visited = false;
};
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

using Matrix = std::vector<std::vector<float>>;

std::vector<std::vector<int>> getShortestPath(const Matrix & elevationMatrix,
                                              const Matrix & costMatrix,
                                              std::deque<MatrixPoint> controlPoints,
                                              const Weights &weights);

#endif //BREADCRUMBS_BREADCRUMBS_H
