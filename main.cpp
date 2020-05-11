#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <numeric>
#include <tiffio.h>
#include <memory>

#include "json.h"
#include "LasOps.h"
#include "TiffOps.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::ifstream;

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


struct Weights
{
    int unitsPerPixel;
    int gradeCost;
    double movementCostXY;
    double movementCostZ;
    double heuristicXY;
    double heuristicZ;
};


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

double gradeCost(const MatrixPoint &currentPoint, const MatrixPoint &successor, double heightDifference, int base)
{
    return pow(base, heightDifference / distance(currentPoint, successor, 0, 1, 1, 1));
}

vector<vector<int>> getShortestPath(const vector<vector<float>> & terrainMatrix, deque<MatrixPoint> controlPoints, Weights weights)
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
                finishingPoint = std::make_shared<MatrixPoint>(successor);
                if (successor.x == target.x && successor.y == target.y)
                {
                    stop = true;
                    break;
                }

                if (pathMatrix[successor.y][successor.x] == 0)
                {
                    pathMatrix[successor.y][successor.x] = visitedPoint;
                    double heightToSuccessor = scaledDifference(terrainMatrix, currentPoint, successor, weights.unitsPerPixel);
                    successor.movementCost = (distance(currentPoint, successor, heightToSuccessor,
                            weights.movementCostXY, weights.movementCostXY, weights.movementCostZ)
                                              + currentPoint.movementCost
                                              + gradeCost(currentPoint, successor, heightToSuccessor, weights.gradeCost));

                    double heightToTarget = scaledDifference(terrainMatrix, successor, target, weights.unitsPerPixel);
                    double distToTarget = distance(successor, target, heightToTarget,
                            weights.heuristicXY, weights.heuristicXY, weights.heuristicZ);

                    successor.totalCost = successor.movementCost + distToTarget;

                    successor.parent = std::make_unique<MatrixPoint>(currentPoint);
                    openSet.push(successor);
                }
            }
        }
        std::fill(pathMatrix.begin(), pathMatrix.end(), std::vector<int>(pathMatrix.size(), 0));
        controlPoints.pop_front();
        while(finishingPoint)
        {
            finalMatrix[finishingPoint->y][finishingPoint->x] = visitedPoint;
            finishingPoint = finishingPoint->parent;
        }
    }

    return finalMatrix;
}

int main(int argc, char * argv [])
{
    if (argc < 2)
    {
        cout << "Specify a TIFF to use" << endl;
        return -1;
    }

    auto matrix = getMatrix(argv[1]); //../bh_FB17_3764.tif

    if (matrix.empty())
    {
        return -1;
    }

    cout << "Rows: " << matrix.size() << endl;

    cout << "Columns: " << matrix[0].size() << endl;

    ifstream jsonFile("params.json");
    if (!jsonFile)
    {
        cout << "Error Reading Input from params.json" << endl;
        return -1;
    }

    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    Json::Value root;
    string errors;
    Json::parseFromStream(builder, jsonFile, &root, &errors);

    deque<MatrixPoint> points;
    for (Json::ArrayIndex i = 0; i < root["points"].size(); ++i)
    {
        MatrixPoint m = {root["points"][i]["x"].asInt64(), root["points"][i]["y"].asInt64()};
        points.push_back(m);
    }

    auto weightsJson = root["weights"];

    Weights weights = {weightsJson["unitsPerPixel"].asInt(),
                       weightsJson["gradeCost"].asInt(),
                       weightsJson["movementCost"]["xy"].asDouble(),
                       weightsJson["movementCost"]["z"].asDouble(),
                       weightsJson["heuristic"]["xy"].asDouble(),
                       weightsJson["heuristic"]["z"].asDouble()};

    auto pathMatrix = getShortestPath(matrix, points, weights); //{470, 420}, {200, 230}

    writePathToTIFF(pathMatrix, "path.tif");

    return 0;
}