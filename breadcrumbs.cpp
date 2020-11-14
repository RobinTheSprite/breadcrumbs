//
// Created by Mark on 5/23/2020.
//

#include <memory>
#include <vector>
#include <cmath>
#include <queue>
#include <deque>
#include <utility>
#include "breadcrumbs.h"

using std::vector;
using std::deque;
using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::priority_queue;

bool inBounds(const Matrix &matrix, const MatrixPoint &p)
{
    return (p.x > -1 && p.y > -1) && p.y < matrix.size() && p.x < matrix[0].size();
}

void getSurroundingPoints(const Matrix &matrix, const MatrixPoint &currentPoint, vector<MatrixPoint> &surroundingPoints)
{
    auto stepSize = 1;

    auto writeIndex = 0;
    for (long x = currentPoint.x - stepSize; x <= currentPoint.x + stepSize; x += stepSize)
    {
        for (long y = currentPoint.y - stepSize; y <= currentPoint.y + stepSize; y += stepSize)
        {
            if(!(x == currentPoint.x && y == currentPoint.y) && inBounds(matrix, {x, y}))
            {
                surroundingPoints[writeIndex].x = x;
                surroundingPoints[writeIndex].y = y;
                surroundingPoints[writeIndex].movementCost = 0;
                surroundingPoints[writeIndex].totalCost = 0;
                ++writeIndex;
            }
        }
    }

    surroundingPoints.resize(writeIndex);
}

double distance(const MatrixPoint &a, const MatrixPoint &b, double height = 0, double xScale = 1, double yScale = 1, double zScale = 1)
{
    auto xScaled = (double)(b.x - a.x) * xScale;
    auto yScaled = (double)(b.y - a.y) * yScale;
    auto zScaled = height * zScale;

    return std::sqrt(xScaled*xScaled + yScaled*yScaled + zScaled*zScaled);
}

double scaledHeight(const Matrix &matrix, const MatrixPoint &a, const MatrixPoint &b, const double &unitsPerPixel)
{
    double scaleFactor = 1 / unitsPerPixel;
    return std::abs(matrix[b.y][b.x] - matrix[a.y][a.x]) * scaleFactor;
}

double gradeCost(const MatrixPoint &currentPoint, const MatrixPoint &successor, const Matrix &matrix, const Weights &weights)
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
            worstHeight = std::max(worstHeight,
                                   scaledHeight(matrix, {x, y}, {x + xDiff, y + yDiff}, weights.unitsPerPixel));
        }

        x += xDiff;
        y += yDiff;
    }

    return std::pow(weights.gradeCost, worstHeight / distance(currentPoint, successor));
}

//controlPoints needs to be a deque because the algorithm needs to pop things off the front quickly but also have
//random access. std::queue does not have random access.
vector<vector<int>> getShortestPath(const Matrix &elevationMatrix,
                                    const Matrix &costMatrix,
                                    deque<MatrixPoint> &controlPoints,
                                    const Weights &weights)
{
    auto finalMatrix = vector<vector<int>>(elevationMatrix.size(), vector<int>(elevationMatrix[0].size(), 0));
    auto visitedPoint = 100;
    MatrixPoint finishingPoint;
    while (controlPoints.size() >= 2)
    {
        MatrixPoint startingPoint = controlPoints[0];
        MatrixPoint target = controlPoints[1];

        auto differenceGreater = [](auto a, auto b) { return a.totalCost > b.totalCost; };
        using PointQueue = priority_queue<MatrixPoint, vector<MatrixPoint>, decltype(differenceGreater)>;
        PointQueue pointQueue(differenceGreater);

        pointQueue.push(startingPoint);

        auto pathMatrix = vector<vector<MatrixPoint>>(elevationMatrix.size(), vector<MatrixPoint>(elevationMatrix[0].size(), MatrixPoint{}));
        startingPoint.visited = true;
        pathMatrix[startingPoint.y][startingPoint.x]= startingPoint;

        vector<MatrixPoint> surroundingPoints(8, MatrixPoint{});

        bool stop = false;
        while (!stop && !pointQueue.empty())
        {
            auto currentPoint = pointQueue.top();
            pointQueue.pop();
            finishingPoint = currentPoint;

            surroundingPoints.resize(8);
            getSurroundingPoints(elevationMatrix, currentPoint, surroundingPoints);

            for (auto &successor : surroundingPoints)
            {
                if (currentPoint.x == target.x && currentPoint.y == target.y)
                {
                    stop = true;
                    break;
                }

                if (!pathMatrix[successor.y][successor.x].visited)
                {
                    successor.visited =true;
                    double heightToSuccessor = scaledHeight(
                            elevationMatrix,
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
                          + gradeCost(
                                    currentPoint,
                                    successor,
                                    elevationMatrix,
                                    weights
                            )
                          + currentPoint.movementCost
                          + costMatrix[successor.y][successor.x]
                    );

                    const double heightToTarget = scaledHeight(
                            elevationMatrix,
                            successor,
                            target,
                            weights.unitsPerPixel
                    );
                    double distToTarget = distance(
                        successor,
                        target,
                        heightToTarget,
                        weights.heuristicXY,
                        weights.heuristicXY,
                        weights.heuristicZ
                    );

                    successor.totalCost = successor.movementCost + distToTarget;

                    successor.parent = {currentPoint.x, currentPoint.y};
                    pointQueue.push(successor);
                    pathMatrix[successor.y][successor.x] = successor;
                }
            }
        }

        controlPoints.pop_front();
        while(finishingPoint.parent != std::make_pair<long, long>(-1, -1))
        {
            finalMatrix[finishingPoint.y][finishingPoint.x] = visitedPoint;
            finishingPoint = pathMatrix[finishingPoint.parent.second][finishingPoint.parent.first];
        }

        fill(pathMatrix.begin(), pathMatrix.end(), vector<MatrixPoint>(pathMatrix.size(), MatrixPoint{}));
    }

    return finalMatrix;
}