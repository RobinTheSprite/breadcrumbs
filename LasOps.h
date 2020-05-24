//
// Created by Mark on 1/1/2020.
//

#ifndef BREADCRUMBS_LASOPS_H
#define BREADCRUMBS_LASOPS_H

#include <vector>
#include <numeric>
#include <map>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/filters/ReprojectionFilter.hpp>
#include <pdal/filters/AssignFilter.hpp>
#include <pdal/filters/SMRFilter.hpp>
#include <pdal/filters/RangeFilter.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

using namespace pdal;

struct Point
{
    double x = 0;
    double y = 0;
    double z = 0;

    std::map<Dimension::Id, uint16_t> rgb{};
};

using PointCloud = vector<Point>;

PointCloud mergeClouds(vector<PointCloud> && clouds);

Point pointProcessor(const PointRef & pdalPoint);

PointCloud getAndFilterCloud(const string &filename);

PointCloud getCloud(const string &filename);

void writeLAZ(const PointCloud &cloud, const string &filename);

void writeGdal(PointCloud & cloud, const string &filename);

PointCloud getTIFF(const string &filename);

#endif //BREADCRUMBS_LASOPS_H
