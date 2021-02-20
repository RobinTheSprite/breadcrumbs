/*
 * main.cpp
 * Mark Underwood
 * www.github.com/RobinTheSprite
 * Parses input and runs the algorithm
 */

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

// Adds each cell of a 2D matrix to an identically sized matrix.
// The second parameter is added to the first parameter.
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

// Stores various settings for the test suite function.
// Cuts down on needed parameters
struct TestSuiteSettings
{
    string filepath;
    bool heatmap = false;
    bool writeImages = false;
    double unitsPerPixel = 0;
};

/*
 * Systematically runs the algorithm on a combination of parameter settings.
 * Each run is individually written to a TIFF with the parameter settings
 * encoded into the filename.
 * The results of the test suite are written to a folder named after the points
 * that were traversed.
 * Optionally, generate a heatmap of every run and output to a single TIFF.
 */
int runTestSuite(const vector<std::vector<float>> &matrix, const vector<std::vector<float>> &costMatrix,
                 deque<MatrixPoint> &points, const TestSuiteSettings &settings)
{
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
                                settings.unitsPerPixel,
                                gradeCost,
                                5,
                                static_cast<double>(movementCostXY),
                                static_cast<double>(movementCostZ),
                                static_cast<double>(heuristicXY),
                                static_cast<double>(heuristicZ)
                        };

                        auto pathMatrix = getShortestPath(matrix, costMatrix, points, weights);

                        if (settings.heatmap)
                        {
                            addMatrices<int>(heatMap, pathMatrix);
                        }

                        if (settings.writeImages)
                        {
                            writePathToTIFF
                            (
                                pathMatrix,
                                settings.filepath +
                                "grade(" + std::to_string(gradeCost) + ")" +
                                "g(xy=" + std::to_string(movementCostXY) + ", z=" + std::to_string(movementCostZ) + ")" +
                                "h(xy=" + std::to_string(heuristicXY) + ", z=" + std::to_string(heuristicZ) + ").tif"
                            );
                        }
                    }
                }
            }
        }
    }

    if (settings.heatmap)
    {
        writePathToTIFF(heatMap, settings.filepath + "heatmap.tif");
    }

    return 0;
}

/*
 * Reads a JSON file with the given name into a JSON object.
 */
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

/*
 * Reads, and weights, the extra cost layers specified in params.json.
 */
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

/*
 * Adds all cost layers, except the elevation data, into a single matrix.
 */
vector<vector<float>> accumulateLayers(vector<vector<vector<float>>> layers)
{
    vector<vector<float>> accumulatedLayers(layers[0].size(), vector<float>(layers[0][0].size(), 0));
    for (const auto & layer : layers)
    {
        addMatrices<float>(accumulatedLayers, layer);
    }

    return accumulatedLayers;
}

/*
 * Read the weights out of the JSON object and into a Weights object.
 */
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

/*
 * Calls all necessary functions to create an accumulated cost matrix for all cost layers
 * given in params.json
 */
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

/*
 * Reads the points which the algorithm must pass through from
 * the JSON object.
 * Returns a deque of MatrixPoints.
 */
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

    if (argc > 2)
    {
        TestSuiteSettings settings;
        settings.unitsPerPixel = json["weights"]["unitsPerPixel"].get<double>();
        for (int i = 2; i < argc; ++i)
        {
            if (strcmp(argv[i], "--testsuite") == 0)
            {
                settings.writeImages = true;
            } else if (strcmp(argv[i], "--heatmap") == 0)
            {
                settings.heatmap = true;
            }
        }

        string dequeString;
        for (const auto &point : points)
        {
            dequeString += "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")";
        }

        cout << "Begin full test suite for " << argv[1] << endl;
        cout << "at points " << dequeString << endl;

        string filename = argv[1];
        auto const posOfSlash = filename.find_last_of('/');
        auto const posOfDot = filename.find_last_of('.');
        filename = filename.substr(posOfSlash + 1, posOfDot - posOfSlash - 1);

        string filepath = "./data/" + filename + dequeString + "/";
        settings.filepath = filepath;
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

        return runTestSuite(elevationMatrix, costMatrix, points, settings);
    }
    else
    {
        auto weights = getWeights(json["weights"]);

        auto pathMatrix = getShortestPath(elevationMatrix, costMatrix, points, weights);

        writePathToTIFF(pathMatrix, "path.tif");
    }

    return 0;
}