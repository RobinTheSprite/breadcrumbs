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
                                unitsPerPixel,
                                gradeCost,
                                static_cast<double>(movementCostXY),
                                static_cast<double>(movementCostZ),
                                static_cast<double>(heuristicXY),
                                static_cast<double>(heuristicZ)
                        };

                        auto pathMatrix = getShortestPath(matrix, costMatrix, points, weights);

                        writePathToTIFF(pathMatrix, filepath +
                            "grade(" + std::to_string(gradeCost) + ")" +
                            "g(xy=" + std::to_string(movementCostXY) + ", z=" + std::to_string(movementCostZ) + ")" +
                            "h(xy=" + std::to_string(heuristicXY) + ", z=" + std::to_string(heuristicZ) + ").tif");
                    }
                }
            }
        }
    }

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
        for (size_t y = 0; y < layer.size(); ++y)
        {
            for (size_t x = 0; x < layer[0].size(); ++x)
            {
                accumulatedLayers[y][x] += layer[y][x];
            }
        }
    }

    return accumulatedLayers;
}

int main(int argc, char * argv [])
{
    if (argc < 2)
    {
        cout << "Specify a TIFF to use" << endl;
        return -1;
    }

    auto matrix = readTIFF(argv[1]);

    if (matrix.empty())
    {
        return -1;
    }

    cout << argv[1] << endl;

    cout << "Rows: " << matrix.size() << endl;

    cout << "Columns: " << matrix[0].size() << endl;

    nlohmann::json root;
    try
    {
        root = readJSON("params.json");
    }
    catch(std::runtime_error &e)
    {
        cout << e.what() << endl;
        return -1;
    }

    //load deque with points to run the algorithm between
    deque<MatrixPoint> points;
    for (int i = 0; i < root["points"].size(); ++i)
    {
        MatrixPoint m = {root["points"][i]["x"].get<int>(), root["points"][i]["y"].get<int>()};
        points.push_back(m);
    }

    auto layersJson = root["layers"];
    vector<Matrix> layers;
    if (layersJson.empty())
    {
        layers = vector<vector<vector<float>>>(1, vector<vector<float>>(matrix.size(), vector<float>(matrix[0].size(), 0)));
    }
    else
    {
        layers = getLayers(layersJson);
    }

    auto accumulatedLayer = accumulateLayers(layers);

    auto weightsJson = root["weights"];

    if (argc == 3)
    {
        if (!strcmp(argv[2], "-testsuite"))
        {
            return runTestSuite(argv, matrix, accumulatedLayer, points, weightsJson["unitsPerPixel"].get<double>());
        }
    }
    else
    {
        Weights weights = {
            weightsJson["unitsPerPixel"].get<double>(),
            weightsJson["gradeCost"].get<int>(),
            weightsJson["movementCost"]["xy"].get<double>(),
            weightsJson["movementCost"]["z"].get<double>(),
            weightsJson["heuristic"]["xy"].get<double>(),
            weightsJson["heuristic"]["z"].get<double>()
        };

        auto pathMatrix = getShortestPath(matrix, accumulatedLayer, points, weights); //{470, 420}, {200, 230}

        writePathToTIFF(pathMatrix, "path.tif");
    }

    return 0;
}