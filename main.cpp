#include <iostream>
#include <vector>
#include <string>
#include <numeric>
#include <tiffio.h>

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
    int x;
    int y;
    float movementCost;
    float heuristic;
    float totalCost;
};

vector<vector<int>> getShortestPath(const vector<vector<float>> & terrainMatrix, MatrixPoint startingPoint, MatrixPoint target)
{
    auto differenceGreater = [](auto a, auto b){return a.totalCost > b.totalCost;};
    using PointQueue = priority_queue<MatrixPoint, vector<MatrixPoint>, decltype(differenceGreater)>;
    PointQueue openSet(differenceGreater);

    startingPoint.heuristic = 0;
    startingPoint.movementCost = 0;
    startingPoint.totalCost = 0;
    openSet.push(startingPoint);

    auto pathMatrix = vector<vector<int>>(terrainMatrix.size(), vector<int>(terrainMatrix[0].size(), 0));
    auto visitedPoint = 100;

    vector<MatrixPoint> surroundingPoints(4);

    bool stop = false;
    while (!stop)
    {
        auto currentPoint = openSet.top();
        openSet.pop();

        surroundingPoints = {{currentPoint.x + 1, currentPoint.y},
                             {currentPoint.x - 1, currentPoint.y},
                             {currentPoint.x,     currentPoint.y + 1},
                             {currentPoint.x,     currentPoint.y - 1},
                             {currentPoint.x + 1, currentPoint.y + 1},
                             {currentPoint.x - 1, currentPoint.y - 1},
                             {currentPoint.x - 1, currentPoint.y + 1},
                             {currentPoint.x + 1, currentPoint.y - 1}
        };

        auto inBounds = [terrainMatrix](auto n) { return n > -1 && n < terrainMatrix.size() && n < terrainMatrix[0].size(); };
        for (const auto &point : surroundingPoints)
        {
            if (point.x == target.x && point.y == target.y)
            {
                pathMatrix[point.y][point.x] = visitedPoint;
                stop = true;
                break;
            }

            if (inBounds(point.x) && inBounds(point.y))
            {
                if (pathMatrix[point.y][point.x] == 0)
                {
                    auto difference = abs(terrainMatrix[point.y][point.x] - terrainMatrix[currentPoint.y][currentPoint.x]);
                    auto heightToTarget = abs(terrainMatrix[target.y][target.x] - terrainMatrix[currentPoint.y][currentPoint.x]);
                    auto movementCost = difference + point.movementCost;
//                    float heuristic = std::max(abs(target.x - currentPoint.x), abs(target.y - currentPoint.y)); //Diagonal distance
                    float heuristic = sqrt((target.x - currentPoint.x)*(target.x - currentPoint.x) +
                            (target.y - currentPoint.y)*(target.y - currentPoint.y) +
                                                   (int)(heightToTarget*heightToTarget)); //Euclidean Distance

                    openSet.push({point.x, point.y, movementCost, heuristic, movementCost + heuristic});

                    pathMatrix[currentPoint.y][currentPoint.x] = visitedPoint;
                }
            }
        }
    }

    return pathMatrix;
}

int main()
{
    auto matrix = getMatrix("donnelly_dome.tif"); //../bh_FB17_3764.tif

    cout << "Rows: " << matrix.size() << endl;

    cout << "Columns: " << matrix[0].size() << endl;

    auto pathMatrix = getShortestPath(matrix, {285, 125}, {250, 100});

    writePathToTIFF(pathMatrix);

    return 0;
}