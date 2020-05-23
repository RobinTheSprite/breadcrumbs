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
    long x;
    long y;
    double movementCost;
    double totalCost;
    std::shared_ptr<MatrixPoint> parent;
};
struct Weights
{
    int unitsPerPixel;
    int gradeCost;
    double movementCostXY;
    double movementCostZ;
    double heuristicXY;
    double heuristicZ;
};

std::vector<std::vector<int>> getShortestPath(const std::vector<std::vector<float>> & terrainMatrix,
                                              std::deque<MatrixPoint> controlPoints,
                                              Weights weights);

#endif //BREADCRUMBS_BREADCRUMBS_H
