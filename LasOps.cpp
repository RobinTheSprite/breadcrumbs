//
// Created by Mark on 1/1/2020.
//

#include "LasOps.h"

Point pointProcessor(const PointRef & pdalPoint)
{
    Point p;

    p.x = pdalPoint.getFieldAs<double>(Dimension::Id::X);
    p.y = pdalPoint.getFieldAs<double>(Dimension::Id::Y);
    p.z = pdalPoint.getFieldAs<double>(Dimension::Id::Z);

    p.rgb.insert({Dimension::Id::Red, pdalPoint.getFieldAs<int>(Dimension::Id::Red)});
    p.rgb.insert({Dimension::Id::Green, pdalPoint.getFieldAs<int>(Dimension::Id::Green)});
    p.rgb.insert({Dimension::Id::Blue, pdalPoint.getFieldAs<int>(Dimension::Id::Blue)});

    return p;
}


PointCloud getAndFilterCloud(const string &filename)
{
    cout << "Reading " << filename << "..." << endl;

    LasReader reader;

    Options options;
    options.add("filename", filename);
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    string utmProj4 = "+proj=utm +zone=6 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    SpatialReference newCoordSystem(utmProj4);

    LasHeader header = reader.header();

    cout << "File has " << reader.preview().m_pointCount << " points" << endl;

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

    PointCloud cloud;
    cloud.reserve(view->size());

    for (PointId i = 0; i < view->size(); ++i)
    {
        Point p = pointProcessor(view->point(i));
        cloud.push_back(p);
    }

    return cloud;
}


PointCloud getCloud(const string &filename)
{
    cout << "Reading " << filename << "..." << endl;

    LasReader reader;

    Options options;
    options.add("filename", filename);
    reader.setOptions(options);

    FixedPointTable table(100);
    reader.prepare(table);

    string utmProj4 = "+proj=utm +zone=6 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    SpatialReference newCoordSystem(utmProj4);

    LasHeader header = reader.header();

    cout << "File has " << reader.preview().m_pointCount << " points" << endl;

    Options filterOptions;
    filterOptions.add("in_srs", header.srs());
    filterOptions.add("out_srs", newCoordSystem.getProj4());

    ReprojectionFilter repro;
    repro.addOptions(filterOptions);
    repro.setInput(reader);
    repro.prepare(table);

    PointCloud cloud;
    cloud.reserve(header.pointCount());
    auto counter = 0.0;
    int lastPercentage = 0;
    auto processOne = [&] (PointRef & pdalPoint)
    {
        Point p = pointProcessor(pdalPoint);

        auto currentPercentage = static_cast<int>(counter / header.pointCount() * 100);
        if (lastPercentage < currentPercentage)
        {
            lastPercentage = currentPercentage;
            cout << currentPercentage << endl;
        }
        counter++;

        cloud.push_back(p);

        return true;
    };

    StreamCallbackFilter callback;
    callback.setCallback(processOne);
    callback.setInput(repro);
    callback.prepare(table);
    callback.execute(table);

    return cloud;
}


void writeLAZ(const PointCloud &cloud, const string &filename)
{
    cout << "Writing to " << filename << "..." << endl;

    if (cloud.empty())
    {
        cout << "Error: Point Cloud Empty" << endl;
        return;
    }

    FixedPointTable table(100);
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);
    table.layout()->registerDim(Dimension::Id::Red);
    table.layout()->registerDim(Dimension::Id::Green);
    table.layout()->registerDim(Dimension::Id::Blue);

    auto index = 0u;
    auto counter = 0.0;
    auto lastPercentage = 0.0;
    auto processOne = [&](PointRef & pdalPoint)
    {
        auto currentPercentage = static_cast<int>(counter / cloud.size() * 100);
        if (lastPercentage < currentPercentage)
        {
            lastPercentage = currentPercentage;
            cout << currentPercentage << endl;
        }
        ++counter;

        if (index < cloud.size())
        {
            auto point = cloud[index];

            pdalPoint.setField(Dimension::Id::X, point.x);
            pdalPoint.setField(Dimension::Id::Y, point.y);
            pdalPoint.setField(Dimension::Id::Z, point.z);

            for (auto [dimension, value] : point.rgb)
            {
                pdalPoint.setField(dimension, value);
            }

            ++index;

            return true;
        }
        else
        {
            return false;
        }
    };

    StreamCallbackFilter callback;
    callback.setCallback(processOne);

    Options options;
    options.add("filename", filename);
    options.add("a_srs", "EPSG:32606");
    LasWriter writer;
    writer.setInput(callback);
    writer.addOptions(options);
    writer.prepare(table);
    writer.execute(table);
}


void writeGdal(PointCloud & cloud, const string &filename)
{
    cout << "Writing to " << filename << "..." << endl;

    if (cloud.empty())
    {
        cout << "Error: Point Cloud Empty" << endl;
        return;
    }

    FixedPointTable table(100);

    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);
    table.layout()->registerDim(Dimension::Id::Red);
    table.layout()->registerDim(Dimension::Id::Green);
    table.layout()->registerDim(Dimension::Id::Blue);

    auto index = 0u;
    auto counter = 0.0;
    auto lastPercentage = 0.0;
    auto processOne = [&](PointRef & pdalPoint)
    {
        auto currentPercentage = static_cast<int>(counter / cloud.size() * 100);
        if (lastPercentage < currentPercentage)
        {
            lastPercentage = currentPercentage;
            cout << currentPercentage << endl;
        }
        ++counter;

        if (index < cloud.size())
        {
            auto point = cloud[index];

            pdalPoint.setField(Dimension::Id::X, point.x);
            pdalPoint.setField(Dimension::Id::Y, point.y);
            pdalPoint.setField(Dimension::Id::Z, point.z);

            ++index;

            return true;
        }
        else
        {
            return false;
        }
    };

    StreamCallbackFilter callback;
    callback.setCallback(processOne);

    Options options;
    options.add("gdaldriver", "GTiff");
    options.add("output_type", "all");
    options.add("filename", filename);
    options.add("resolution", 2.0);
    GDALWriter writer;
    writer.setInput(callback);
    writer.addOptions(options);
    writer.prepare(table);
    writer.execute(table);
}

PointCloud getTIFF(const string &filename)
{
    cout << "Reading " << filename << "..." << endl;

    GDALReader reader;

    Options options;
    options.add("filename", filename);
    options.add("header", "z");
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    string utmProj4 = "+proj=utm +zone=6 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    SpatialReference newCoordSystem(utmProj4);

    cout << "File has " << reader.preview().m_pointCount << " points" << endl;

    Options filterOptions;
    filterOptions.add("in_srs", reader.getSpatialReference());
    filterOptions.add("out_srs", newCoordSystem.getProj4());

    ReprojectionFilter repro;
    repro.addOptions(filterOptions);
    repro.setInput(reader);
    repro.prepare(table);

    PointCloud cloud;
    cloud.reserve(reader.preview().m_pointCount);

    PointViewSet set = repro.execute(table);
    PointViewPtr view = *(set.begin());

    cloud.reserve(view->size());

    for (PointId i = 0; i < view->size(); ++i)
    {
        Point p = pointProcessor(view->point(i));

        cloud.push_back(p);
    }

    return cloud;
}