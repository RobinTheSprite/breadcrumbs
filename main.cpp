#include <iostream>
#include <vector>
#include <string>
#include <numeric>
#include <tiffio.h>
#include <memory>

#include "LasOps.h"
#include "TiffOps.h"

using std::cout;
using std::endl;
using std::vector;

using namespace pdal;

using PointCloud = vector<Point>;

PointCloud mergeClouds(vector<PointCloud> && clouds)
{
    cout << "Merging clouds..." << endl;

    auto foldCloudSizes = [](auto a, auto b)
    {
        return a + b.size();
    };

    PointCloud outCloud;
    outCloud.reserve(std::accumulate(clouds.begin(), clouds.end(), (size_t)0, foldCloudSizes));

    for (auto cloud : clouds)
    {
        outCloud.insert(outCloud.end(), cloud.begin(), cloud.end());
    }

    return outCloud;
}


struct MatrixPoint
{
    long x;
    long y;
    double movementCost;
    double totalCost;
    std::shared_ptr<MatrixPoint> parent;
};


bool inBounds(const vector<vector<float>>& matrix, long n)
{
    return n > -1 && n < matrix.size() && n < matrix[0].size();
}


void getSurroundingPoints(const vector<vector<float>>& matrix, const MatrixPoint& currentPoint, vector<MatrixPoint> & surroundingPoints)
{
    auto stepSize = 1;

    auto writeIndex = 0;
    for (auto x = currentPoint.x - stepSize; x <= currentPoint.x + stepSize; x += stepSize)
    {
        for (auto y = currentPoint.y - stepSize; y <= currentPoint.y + stepSize; y += stepSize)
        {
            if(!(x == currentPoint.x && y == currentPoint.y) && inBounds(matrix, x) && inBounds(matrix, y))
            {
                surroundingPoints[writeIndex] = {x, y, 0, 0, std::make_shared<MatrixPoint>(currentPoint)};
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

    return sqrt(xScaled*xScaled + yScaled*yScaled + zScaled*zScaled);
}


double scaledDifference(const vector<vector<float>> & matrix, const MatrixPoint& a, const MatrixPoint& b, double unitsPerPixel)
{
    double scaleFactor = 1 / unitsPerPixel;
    return abs(matrix[b.y][b.x] - matrix[a.y][a.x]) * scaleFactor;
}


vector<vector<int>> getShortestPath(const vector<vector<float>> & terrainMatrix, MatrixPoint startingPoint, const MatrixPoint& target)
{
    auto differenceGreater = [](auto a, auto b){return a.totalCost > b.totalCost;};
    using PointQueue = priority_queue<MatrixPoint, vector<MatrixPoint>, decltype(differenceGreater)>;
    PointQueue openSet(differenceGreater);
    vector<vector<double>> closedSet(terrainMatrix.size(), vector<double>(terrainMatrix.size(), std::numeric_limits<double>::max()));

    startingPoint.movementCost = 0;
    startingPoint.totalCost = 0;
    startingPoint.parent = nullptr;
    openSet.push(startingPoint);

    auto pathMatrix = vector<vector<int>>(terrainMatrix.size(), vector<int>(terrainMatrix[0].size(), 0));
    auto visitedPoint = 100;
    pathMatrix[startingPoint.y][startingPoint.x] = visitedPoint;

    vector<MatrixPoint> surroundingPoints(8, {0, 0, 0, 0, nullptr});

    shared_ptr<MatrixPoint> finishingPoint;
    bool stop = false;
    while (!stop && !openSet.empty())
    {
        auto currentPoint = openSet.top();
        openSet.pop();

        surroundingPoints.resize(8);
        getSurroundingPoints(terrainMatrix, currentPoint, surroundingPoints);

        for (auto &successor : surroundingPoints)
        {
            finishingPoint = std::make_shared<MatrixPoint>(successor);
            if (successor.x == target.x && successor.y == target.y)
            {
                stop = true;
                break;
            }

            if (pathMatrix[successor.y][successor.x] == 0)
            {
                pathMatrix[successor.y][successor.x] = visitedPoint;
                double heightToSuccessor = scaledDifference(terrainMatrix, currentPoint, successor, 5);
                successor.movementCost = (distance(currentPoint, successor, heightToSuccessor, 1, 1, 1)
                                        + currentPoint.movementCost);

                double heightToTarget = scaledDifference(terrainMatrix, successor, target, 5);
                double distToTarget = distance(successor, target, heightToTarget, 1, 1, 1);

                successor.totalCost = successor.movementCost + distToTarget;

                if (heightToSuccessor / distance(currentPoint, successor, 0, 1, 1, 1) < 0.08)
                {
                    successor.parent = std::make_unique<MatrixPoint>(currentPoint);
                    openSet.push(successor);
                }
            }
        }
        closedSet[currentPoint.y][currentPoint.x] = currentPoint.totalCost;
    }

    std::fill(pathMatrix.begin(), pathMatrix.end(), std::vector<int>(pathMatrix.size(), 0));
    while(finishingPoint)
    {
        pathMatrix[finishingPoint->y][finishingPoint->x] = visitedPoint;
        finishingPoint = finishingPoint->parent;
    }

    return pathMatrix;
}

int main()
{
    auto matrix = getMatrix("donnelly_dome.tif"); //../bh_FB17_3764.tif

    cout << "Rows: " << matrix.size() << endl;

    cout << "Columns: " << matrix[0].size() << endl;

    auto pathMatrix = getShortestPath(matrix, {470, 420}, {0, 0}); //{470, 420}, {200, 230}

    writePathToTIFF(pathMatrix, "path.tif");

    return 0;
}