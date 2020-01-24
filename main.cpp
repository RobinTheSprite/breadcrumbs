#include <iostream>
#include <vector>
#include <numeric>
#include <map>
#include <pdal/filters/SMRFilter.hpp>
#include "GDALGrid.hpp"
#include <tiffio.h>

#include "LasOps.h"

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

vector<vector<float>> getMatrix(const string & filename)
{
    TIFF * tiff = TIFFOpen(filename.data(), "r");
    vector<vector<float>> matrix;
    if (tiff)
    {
        uint32 imageWidth, imageLength;
        uint32 tileWidth, tileLength;
        uint32 x, y;
        tdata_t buf;

        TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &imageWidth);
        TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &imageLength);
        TIFFGetField(tiff, TIFFTAG_TILEWIDTH, &tileWidth);
        TIFFGetField(tiff, TIFFTAG_TILELENGTH, &tileLength);
        buf = _TIFFmalloc(TIFFTileSize(tiff));
        matrix = vector<vector<float>>(imageLength, vector<float>(imageWidth));
        for (y = 0; y < imageLength; y += tileLength)
        {
            for (x = 0; x < imageWidth; x += tileWidth)
            {
                TIFFReadTile(tiff, buf, x, y, 0, 0);
                for (auto tileY = y; tileY < y + tileLength && tileY < imageLength; ++tileY)
                {
                    for (auto tileX = x; tileX < x + tileWidth && tileX < imageWidth; ++tileX)
                    {
                        matrix[tileY][tileX] = ((float *)buf)[tileX - x + (tileY - y) * tileWidth];
                    }
                }
            }
        }

        _TIFFfree(buf);
        TIFFClose(tiff);
    }

    return matrix;
}

void writeMatrixToTIFF(vector<vector<float>> matrix, const string & filename)
{
    TIFF * out = TIFFOpen(filename.data(), "w");

    int width = matrix[0].size();
    int height = matrix.size();
    int samplesPerPixel = 1;

    TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);
    TIFFSetField(out, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, samplesPerPixel);
    TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 32);
    TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);

    size_t bytesPerLine = width * sizeof(float);
    float * buf = nullptr;

    if (TIFFScanlineSize(out))
    {
        buf = (float *)_TIFFmalloc(bytesPerLine);
    }
    else
    {
        buf = (float *)_TIFFmalloc(TIFFScanlineSize(out));
    }

    TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, 1);

    for (auto i = 0ul; i < matrix.size(); ++i)
    {
        memcpy(buf, matrix[i].data(), bytesPerLine);
        if (TIFFWriteScanline(out, buf, i, 0) < 0)
        {
            cout << "Error Writing TIFF" << endl;
            break;
        }
    }

    TIFFClose(out);
    _TIFFfree(buf);
}

void writePathToTIFF(vector<vector<int>> matrix)
{
    TIFF * out = TIFFOpen("path.tif", "w");

    int width = matrix[0].size();
    int height = matrix.size();
    int samplesPerPixel = 1;

    TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);
    TIFFSetField(out, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, samplesPerPixel);
    TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 32);
    TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_INT);

    size_t bytesPerLine = width * sizeof(int);
    int * buf = nullptr;

    if (TIFFScanlineSize(out))
    {
        buf = (int *)_TIFFmalloc(bytesPerLine);
    }
    else
    {
        buf = (int *)_TIFFmalloc(TIFFScanlineSize(out));
    }

    TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, 1);

    for (auto i = 0ul; i < matrix.size(); ++i)
    {
        memcpy(buf, matrix[i].data(), bytesPerLine);
        if (TIFFWriteScanline(out, buf, i, 0) < 0)
        {
            cout << "Error Writing TIFF" << endl;
            break;
        }
    }

    TIFFClose(out);
    _TIFFfree(buf);
}

struct MatrixPoint
{
    int x;
    int y;
    float movementCost;
    float heuristic;
    float totalCost;
};

vector<vector<int>> getShortestPath(const vector<vector<float>> & matrix, MatrixPoint startingPoint, MatrixPoint target)
{
    auto differenceGreater = [](auto a, auto b){return a.totalCost > b.totalCost;};
    using PointQueue = priority_queue<MatrixPoint, vector<MatrixPoint>, decltype(differenceGreater)>;
    PointQueue openSet(differenceGreater);

    startingPoint.heuristic = 0;
    startingPoint.movementCost = 0;
    startingPoint.totalCost = 0;
    openSet.push(startingPoint);

    auto pathMatrix = vector<vector<int>>(matrix.size(), vector<int>(matrix[0].size(), 0));
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

        auto inBounds = [matrix](auto n) { return n > -1 && n < matrix.size() && n < matrix[0].size(); };
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
                    auto difference = abs(matrix[point.y][point.x] - matrix[currentPoint.y][currentPoint.x]);
                    auto movementCost = difference + point.movementCost;
                    float heuristic = std::max(abs(target.x - currentPoint.x), abs(target.y - currentPoint.y));
                    openSet.push({point.x, point.y, movementCost, heuristic, movementCost + heuristic});
                }
            }
        }

        pathMatrix[currentPoint.y][currentPoint.x] = visitedPoint;
    }

    return pathMatrix;
}

int main()
{
//    auto cloud1 = getTIFF("../bh_FB17_3764.tif");
//
//    auto cloud2 = getTIFF("../bh_FB17_3765.tif");
//
//    auto cloud_out = mergeClouds({cloud1, cloud2});
//
//    writeGdal(cloud_out, "merge_test.tif");

    auto matrix = getMatrix("../bh_FB17_3764.tif");

    cout << "Rows: " << matrix.size() << endl;

    cout << "Columns: " << matrix[0].size() << endl;

    auto pathMatrix = getShortestPath(matrix, {0, 0}, {200, 100});

    writePathToTIFF(pathMatrix);

    return 0;
}