//
// Created by Mark on 5/23/2020.
//

#include <memory>
#include <vector>
#include <cmath>
#include <queue>
#include <deque>
#include "breadcrumbs.h"

using std::vector;
using std::deque;
using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::priority_queue;

bool inBounds(const vector<vector<float>>& matrix, const MatrixPoint& p)
{
    return (p.x > -1 && p.y > -1) && p.y < matrix.size() && p.x < matrix[0].size();
}

void getSurroundingPoints(const vector<vector<float>>& matrix, const MatrixPoint& currentPoint, vector<MatrixPoint> & surroundingPoints)
{
    auto stepSize = 1;

    auto writeIndex = 0;
    for (long x = currentPoint.x - stepSize; x <= currentPoint.x + stepSize; x += stepSize)
    {
        for (long y = currentPoint.y - stepSize; y <= currentPoint.y + stepSize; y += stepSize)
        {
            if(!(x == currentPoint.x && y == currentPoint.y) && inBounds(matrix, {x, y}))
            {
                surroundingPoints[writeIndex] = {x, y, 0, 0, make_shared<MatrixPoint>(currentPoint)};
                ++writeIndex;
            }
        }
    }

    surroundingPoints.resize(writeIndex);
}

double distance(const MatrixPoint &a, const MatrixPoint &b, double height, double xScale, double yScale, double zScale)
{
    auto xScaled = (double)(b.x - a.x) * xScale;
    auto yScaled = (double)(b.y - a.y) * yScale;
    auto zScaled = height * zScale;

    return std::sqrt(xScaled*xScaled + yScaled*yScaled + zScaled*zScaled);
}

double scaledDifference(const vector<vector<float>> & matrix, const MatrixPoint& a, const MatrixPoint& b, double unitsPerPixel)
{
    double scaleFactor = 1 / unitsPerPixel;
    return std::abs(matrix[b.y][b.x] - matrix[a.y][a.x]) * scaleFactor;
}

double gradeCost(const MatrixPoint &currentPoint, const MatrixPoint &successor, const vector<vector<float>> & matrix, int base, double unitsPerPixel)
{
    long x = currentPoint.x;
    long y = currentPoint.y;
    const long xDiff = successor.x - currentPoint.x;
    const long yDiff = successor.y - currentPoint.y;

    double worstHeight = 0;

    const unsigned int radius = 4;
    for (auto i = 0u; i < radius; ++i)
    {
        if (inBounds(matrix, {x + xDiff, y + yDiff}) && inBounds(matrix, {x, y}))
        {
            worstHeight = std::max(worstHeight, scaledDifference(matrix, {x, y}, {x + xDiff, y + yDiff}, unitsPerPixel));
        }

        x += xDiff;
        y += yDiff;
    }

    return std::pow(base, worstHeight / distance(currentPoint, successor, 0, 1, 1, 1));
}

//controlPoints needs to be a deque because the algorithm needs to pop things off the front quickly but also have
//random access. std::queue does not have random access.
vector<vector<int>> getShortestPath(const vector<vector<float>> & terrainMatrix,
                                    const vector<vector<float>> & costMatrix,
                                    deque<MatrixPoint> controlPoints,
                                    Weights weights)
{
    auto finalMatrix = vector<vector<int>>(terrainMatrix.size(), vector<int>(terrainMatrix[0].size(), 0));
    auto visitedPoint = 100;
    shared_ptr<MatrixPoint> finishingPoint;
    while (controlPoints.size() >= 2)
    {
        MatrixPoint startingPoint = controlPoints[0];
        MatrixPoint target = controlPoints[1];

        auto differenceGreater = [](auto a, auto b) { return a.totalCost > b.totalCost; };
        using PointQueue = priority_queue<MatrixPoint, vector<MatrixPoint>, decltype(differenceGreater)>;
        PointQueue openSet(differenceGreater);

        startingPoint.movementCost = 0;
        startingPoint.totalCost = 0;
        startingPoint.parent = nullptr;
        openSet.push(startingPoint);

        auto pathMatrix = vector<vector<int>>(terrainMatrix.size(), vector<int>(terrainMatrix[0].size(), 0));
        pathMatrix[startingPoint.y][startingPoint.x] = visitedPoint;

        vector<MatrixPoint> surroundingPoints(8, {0, 0, 0, 0, nullptr});

        bool stop = false;
        while (!stop && !openSet.empty())
        {
            auto currentPoint = openSet.top();
            openSet.pop();

            surroundingPoints.resize(8);
            getSurroundingPoints(terrainMatrix, currentPoint, surroundingPoints);

            for (auto &successor : surroundingPoints)
            {
                finishingPoint = make_shared<MatrixPoint>(successor);
                if (successor.x == target.x && successor.y == target.y)
                {
                    stop = true;
                    break;
                }

                if (pathMatrix[successor.y][successor.x] == 0)
                {
                    pathMatrix[successor.y][successor.x] = visitedPoint;
                    double heightToSuccessor = scaledDifference(
                        terrainMatrix,
                        currentPoint,
                        successor,
                        weights.unitsPerPixel
                    );

                    successor.movementCost = (
                            distance(
                                currentPoint,
                                successor,
                                heightToSuccessor,
                                weights.movementCostXY,
                                weights.movementCostXY,
                                weights.movementCostZ
                            )
                          + currentPoint.movementCost
                          + gradeCost(currentPoint, successor, terrainMatrix, weights.gradeCost, weights.unitsPerPixel)
                          + costMatrix[successor.y][successor.x]
                    );

                    double heightToTarget = scaledDifference(terrainMatrix, successor, target, weights.unitsPerPixel);
                    double distToTarget = distance(successor, target, heightToTarget,
                            weights.heuristicXY, weights.heuristicXY, weights.heuristicZ);

                    successor.totalCost = successor.movementCost + distToTarget;

                    successor.parent = make_unique<MatrixPoint>(currentPoint);
                    openSet.push(successor);
                }
            }
        }
        fill(pathMatrix.begin(), pathMatrix.end(), vector<int>(pathMatrix.size(), 0));
        controlPoints.pop_front();
        while(finishingPoint)
        {
            finalMatrix[finishingPoint->y][finishingPoint->x] = visitedPoint;
            finishingPoint = finishingPoint->parent;
        }
    }

    return finalMatrix;
}