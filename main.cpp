#include <iostream>
#include <vector>
#include <chrono>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/filters/ReprojectionFilter.hpp>
#include <pdal/filters/AssignFilter.hpp>
#include <pdal/filters/SMRFilter.hpp>
#include <pdal/filters/RangeFilter.hpp>
#include <pdal/filters/MergeFilter.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

using std::cout;
using std::endl;
using std::vector;

using namespace pdal;

vector<PointRef> getAndFilterCloud(const string &filename)
{
    LasReader reader;

    Options options;
    options.add("filename", filename);
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    std::cout << "Reading LAZ file..." << std::endl;

    string utmProj4 = "+proj=utm +zone=6 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    SpatialReference newCoordSystem(utmProj4);

    LasHeader header = reader.header();

    cout << "File has " << reader.preview().m_pointCount << " points" << endl;

    cout << "Setting Options" << endl;
    Options filterOptions;
    filterOptions.add("in_srs", header.srs());
    filterOptions.add("out_srs", newCoordSystem.getProj4());

    ReprojectionFilter repro;
    repro.addOptions(filterOptions);
    repro.setInput(reader);
    repro.prepare(table);

    Options assignOptions;
    assignOptions.add("assignment", "Classification[:]=0");
    AssignFilter assign;
    assign.addOptions(assignOptions);
    assign.setInput(repro);

    Options smrfOptions;
    smrfOptions.add("ignore", "Classification[7:7]");
    smrfOptions.add("slope", 0.2);
    smrfOptions.add("window", 16);
    smrfOptions.add("threshold", 0.45);
    smrfOptions.add("scalar", 1.2);
    SMRFilter smrf;
    smrf.setInput(assign);
    smrf.addOptions(smrfOptions);

    Options rangeOptions;
    rangeOptions.add("limits", "Classification[2:2]");
    RangeFilter range;
    range.setInput(smrf);
    range.addOptions(rangeOptions);

    range.prepare(table);
    PointViewSet set = range.execute(table);

    PointViewPtr view = *(set.begin());
    vector<PointRef> cloud;
    cloud.reserve(view->size());

    for (PointId i = 0; i < view->size(); ++i)
    {
        cloud.push_back(view->point(i));
    }

    return cloud;
}

vector<PointRef> getCloud(const string &filename)
{
    LasReader reader;

    Options options;
    options.add("filename", filename);
    reader.setOptions(options);

    FixedPointTable table(100);
    reader.prepare(table);

    std::cout << "Reading LAZ file..." << std::endl;

    string utmProj4 = "+proj=utm +zone=6 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    SpatialReference newCoordSystem(utmProj4);

    LasHeader header = reader.header();

    cout << "File has " << reader.preview().m_pointCount << " points" << endl;

    cout << "Setting Options" << endl;
    Options filterOptions;
    filterOptions.add("in_srs", header.srs());
    filterOptions.add("out_srs", newCoordSystem.getProj4());

    ReprojectionFilter repro;
    repro.addOptions(filterOptions);
    repro.setInput(reader);
    repro.prepare(table);

    vector<PointRef> cloud;
    cloud.reserve(repro.preview().m_pointCount);
    auto processOne = [&] (PointRef & point)
    {
        cloud.push_back(point);

        return true;
    };

    StreamCallbackFilter callback;
    callback.setCallback(processOne);
    callback.prepare(table);
    callback.execute(table);

    return cloud;
}

int main()
{
    //TODO: Try and do a file merge myself

    auto start = std::chrono::system_clock::now();

    auto end = std::chrono::system_clock::now();

    vector<PointRef> cloud = getAndFilterCloud("../FB17_3764.laz");

    cout << "Cloud Size: " << cloud.size() << endl;

    std::chrono::duration<double> difference = end - start;

    cout << difference.count() << endl;

    return 0;
}