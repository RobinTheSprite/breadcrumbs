#include <iostream>
#include <vector>
#include <string>
#include <numeric>
#include <tiffio.h>

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