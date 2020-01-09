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

int main()
{
    auto cloud1 = getTIFF("../bh_FB17_3764.tif");

    auto cloud2 = getTIFF("../bh_FB17_3765.tif");

    auto cloud_out = mergeClouds({cloud1, cloud2});

    writeGdal(cloud_out, "merge_test.tif");

    return 0;
}