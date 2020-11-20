#include <iostream>
#include <string>
#include <sys/stat.h>
#include <deque>
#include <fstream>

#include "json.hpp"
#include "TiffOps.h"
#include "breadcrumbs.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::ifstream;
using std::deque;

template <typename T>
void addMatrices(vector<vector<T>> &accumulatedLayers, const vector<vector<T>> &layer)
{
    for (size_t y = 0; y < layer.size(); ++y)
    {
        for (size_t x = 0; x < layer[0].size(); ++x)
        {
            accumulatedLayers[y][x] += layer[y][x];
        }
    }
}

int runTestSuite(char *const *argv,
                 const vector<std::vector<float>> &matrix,
                 const vector<std::vector<float>> &costMatrix,
                 deque<MatrixPoint> &points,
                 const double &unitsPerPixel)
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
    if (mkdirResult != 0 && errno != EEXIST)
    {
        cout << "ERROR: Unable to create data directory" << endl;
                return -1;
    }

    mkdirResult = mkdir(filepath.data(), S_IRWXU);
    if (mkdirResult != 0 && errno != EEXIST)
    {
        cout << "ERROR: Unable to create directory for test run" << endl;
                return -1;
    }

    auto heatMap = std::vector<vector<int>>(matrix.size(), vector<int>(matrix[0].size(), 0));

    int gradeCosts [] = {0, 10, 100, 1000};
    int xyzWeights [] = {0, 1, 10, 100};
    for (const auto &gradeCost : gradeCosts)
    {
        for (const int &movementCostXY : xyzWeights)
        {
            for (const int &movementCostZ : xyzWeights)
            {
                for (const int &heuristicXY : xyzWeights)
                {
                    for (const int &heuristicZ : xyzWeights)
                    {
                        Weights weights = {
                                unitsPerPixel,
                                gradeCost,
                                5,
                                static_cast<double>(movementCostXY),
                                static_cast<double>(movementCostZ),
                                static_cast<double>(heuristicXY),
                                static_cast<double>(heuristicZ)
                        };

                        auto pathMatrix = getShortestPath(matrix, costMatrix, points, weights);

                        addMatrices<int>(heatMap, pathMatrix);

                        writePathToTIFF(pathMatrix, filepath +
                            "grade(" + std::to_string(gradeCost) + ")" +
                            "g(xy=" + std::to_string(movementCostXY) + ", z=" + std::to_string(movementCostZ) + ")" +
                            "h(xy=" + std::to_string(heuristicXY) + ", z=" + std::to_string(heuristicZ) + ").tif");
                    }
                }
            }
        }
    }

    writePathToTIFF(heatMap, filepath + "heatmap.tif");

    return 0;
}

nlohmann::json readJSON(const string& filename)
{
    nlohmann::json root;
    ifstream jsonFile(filename);
    if (!jsonFile)
    {
        throw std::runtime_error("Error Reading Input from params.json");
    }
    jsonFile >> root;
    return root;
}

vector<vector<vector<float>>> getLayers(const nlohmann::json& layersJson)
{
    std::vector<std::vector<vector<float>>> layers = std::vector<std::vector<vector<float>>>();
    for (const auto & layerInfo : layersJson)
    {
        auto layer = readTIFF(layerInfo["filename"]);
        float layerWeight = layerInfo["weight"];
        for (auto & row : layer)
        {
            for (auto & point : row)
            {
                point *= layerWeight;
            }
        }

        layers.push_back(layer);
    }

    return layers;
}

vector<vector<float>> accumulateLayers(vector<vector<vector<float>>> layers)
{
    vector<vector<float>> accumulatedLayers(layers[0].size(), vector<float>(layers[0][0].size(), 0));
    for (const auto & layer : layers)
    {
        addMatrices<float>(accumulatedLayers, layer);
    }

    return accumulatedLayers;
}

Weights getWeights(const nlohmann::json &json)
{
    return {
            json["unitsPerPixel"].get<double>(),
            json["grade"]["base"].get<int>(),
            json["grade"]["radius"].get<int>(),
            json["movementCost"]["xy"].get<double>(),
            json["movementCost"]["z"].get<double>(),
            json["heuristic"]["xy"].get<double>(),
            json["heuristic"]["z"].get<double>()

    };
}

vector<vector<float>> getCostMatrix(const Matrix &elevationMatrix, const nlohmann::json &layersJson)
{
    std::vector<Matrix> layers;
    if (layersJson.empty())
    {
        layers = std::vector<std::vector<vector<float>>>(1, std::vector<vector<float>>(elevationMatrix.size(), vector<float>(elevationMatrix[0].size(), 0)));
    }
    else
    {
        layers = getLayers(layersJson);
    }

    return accumulateLayers(layers);
}

deque<MatrixPoint> getControlPoints(const nlohmann::json &json)
{
    deque<MatrixPoint> points;
    for (const auto & point : json)
    {
        MatrixPoint m = {point["x"].get<int>(), point["y"].get<int>()};
        points.push_back(m);
    }

    return points;
}

int main(int argc, char * argv [])
{
    if (argc < 2)
    {
        cout << "Specify a TIFF to use" << endl;
        return -1;
    }

    auto elevationMatrix = readTIFF(argv[1]);

    if (elevationMatrix.empty())
    {
        cout << "Failed to read TIFF " << argv[1] << endl;
        return -1;
    }

    cout << argv[1] << endl;

    cout << "Rows: " << elevationMatrix.size() << endl;

    cout << "Columns: " << elevationMatrix[0].size() << endl;

    nlohmann::json json;
    try
    {
        json = readJSON("params.json");
    }
    catch(std::runtime_error &e)
    {
        cout << e.what() << endl;
        return -1;
    }

    auto points = getControlPoints(json["points"]);

    auto costMatrix = getCostMatrix(elevationMatrix, json["layers"]);

    if (argc == 3)
    {
        if (!strcmp(argv[2], "--testsuite"))
        {
            return runTestSuite(argv, elevationMatrix, costMatrix, points, json["weights"]["unitsPerPixel"].get<double>());
        }
    }
    else
    {
        auto weights = getWeights(json["weights"]);

        auto pathMatrix = getShortestPath(elevationMatrix, costMatrix, points, weights);

        writePathToTIFF(pathMatrix, "path.tif");
    }

    return 0;
}