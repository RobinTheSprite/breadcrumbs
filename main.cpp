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

void writeMatrixToTIFF(vector<vector<float>> matrix)
{
    TIFF * out = TIFFOpen("matrix_test.tif", "w");

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

    for (auto row : matrix)
    {
        if (!std::all_of(row.begin(), row.end(), [](auto f){return f > 500 && f < 1500;}))
        {
            cout << "Matrix Broken!" << endl;
            cout << *std::find_if(row.begin(), row.end(), [](auto f){return f < 500 || f > 1500;}) << endl;
        }
    }

    writeMatrixToTIFF(matrix);

    return 0;
}