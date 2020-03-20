//
// Created by Mark on 2/2/2020.
//
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include "tiffio.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

vector<vector<float>> getMatrix(const string & filename)
{
    TIFFSetWarningHandler(nullptr);
    TIFF * tiff = TIFFOpen(filename.data(), "r");
    vector<vector<float>> matrix;
    if (tiff)
    {
        uint32 imageWidth, imageLength;
        TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &imageWidth);
        TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &imageLength);
        matrix = vector<vector<float>>(imageLength, vector<float>(imageWidth));
        tdata_t buf;

        if (TIFFIsTiled(tiff))
        {
            uint32 tileWidth, tileLength;
            uint32 x, y;

            TIFFGetField(tiff, TIFFTAG_TILEWIDTH, &tileWidth);
            TIFFGetField(tiff, TIFFTAG_TILELENGTH, &tileLength);
            buf = _TIFFmalloc(TIFFTileSize(tiff));
            for (y = 0; y < imageLength; y += tileLength)
            {
                for (x = 0; x < imageWidth; x += tileWidth)
                {
                    TIFFReadTile(tiff, buf, x, y, 0, 0);
                    for (auto tileY = y; tileY < y + tileLength && tileY < imageLength; ++tileY)
                    {
                        for (auto tileX = x; tileX < x + tileWidth && tileX < imageWidth; ++tileX)
                        {
                            matrix[tileY][tileX] = ((float *) buf)[tileX - x + (tileY - y) * tileWidth];
                        }
                    }
                }
            }
        }
        else
        {
            buf = _TIFFmalloc(TIFFScanlineSize(tiff));
            for (uint32 imageRow = 0; imageRow < imageLength; imageRow++)
            {
                TIFFReadScanline(tiff, buf, imageRow, 0);
                for (auto i = 0; i < imageWidth; ++i)
                {
                    matrix[imageRow][i] = ((float *)buf)[i];
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

void writePathToTIFF(std::vector<std::vector<int>> matrix, std::string filename)
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