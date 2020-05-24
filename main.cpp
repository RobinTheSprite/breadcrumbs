#include <iostream>
#include <string>
#include <sys/stat.h>
#include <deque>
#include <fstream>

#include "json.h"
#include "LasOps.h"
#include "TiffOps.h"
#include "breadcrumbs.h"

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


int main(int argc, char * argv [])
{
    if (argc < 2)
    {
        cout << "Specify a TIFF to use" << endl;
        return -1;
    }

    auto matrix = getMatrix(argv[1]);

    if (matrix.empty())
    {
        return -1;
    }

    cout << argv[1] << endl;

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

    if (argc == 3)
    {
        if (!strcmp(argv[2], "-testsuite"))
        {
            string dequeString;
            for (const auto &point : points)
            {
                dequeString += "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")";
            }

            cout << "Begin full test suite for " << argv[1] << endl;
            cout << "at points " << dequeString << endl;

            string filepath = "./data/" + dequeString + "/";
            int mkdirResult = mkdir("./data/", S_IRWXU);
            if (mkdirResult != 0 && mkdirResult != EEXIST)
            {
                cout << "ERROR: Unable to create data directory" << endl;
                return -1;
            }

            mkdirResult = mkdir(filepath.data(), S_IRWXU);
            if (mkdirResult != 0 && mkdirResult != EEXIST)
            {
                cout << "ERROR: Unable to create directory for test run" << endl;
                return -1;
            }

            int gradeCosts [] = {0, 10, 100, 1000};
            for (const auto &gradeCost : gradeCosts)
            {
                for (int movementCostXY = 0; movementCostXY <= 10; movementCostXY += 5)
                {
                    for (int movementCostZ = 0; movementCostZ <= 10; movementCostZ += 5)
                    {
                        for (int heuristicXY = 0; heuristicXY <= 10; heuristicXY += 5)
                        {
                            for (int heuristicZ = 0; heuristicZ <= 10; heuristicZ += 5)
                            {
                                Weights weights = {
                                        weightsJson["unitsPerPixel"].asInt(),
                                        gradeCost,
                                        static_cast<double>(movementCostXY),
                                        static_cast<double>(movementCostZ),
                                        static_cast<double>(heuristicXY),
                                        static_cast<double>(heuristicZ)
                                };

                                auto pathMatrix = getShortestPath(matrix, points, weights);

                                writePathToTIFF(pathMatrix, filepath +
                                    "grade(" + std::to_string(gradeCost) + ")" +
                                    "g(xy=" + std::to_string(movementCostXY) + ", z=" + std::to_string(movementCostZ) + ")" +
                                    "h(xy=" + std::to_string(heuristicXY) + ", z=" + std::to_string(heuristicZ) + ").tif");
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        Weights weights = {weightsJson["unitsPerPixel"].asInt(),
                           weightsJson["gradeCost"].asInt(),
                           weightsJson["movementCost"]["xy"].asDouble(),
                           weightsJson["movementCost"]["z"].asDouble(),
                           weightsJson["heuristic"]["xy"].asDouble(),
                           weightsJson["heuristic"]["z"].asDouble()};

        auto pathMatrix = getShortestPath(matrix, points, weights); //{470, 420}, {200, 230}

        writePathToTIFF(pathMatrix, "path.tif");
    }

    return 0;
}