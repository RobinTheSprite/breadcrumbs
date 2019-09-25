#include <iostream>
#include <pdal/io/LasReader.hpp>
#include <pdal/filters/ReprojectionFilter.hpp>

using std::cout;
using std::endl;

using namespace pdal;

int main()
{
    LasReader reader;

    Options options;
//    options.add("filename", "../test_1.laz");
    options.add("filename", "../FB17_3764.laz");
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    std::cout << "Reading LAZ file..." << std::endl;

    string ddProj4 = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
    SpatialReference newCoordSystem(ddProj4);

    LasHeader header = reader.header();

    cout << "Setting Options" << endl;
    Options filterOptions;
    filterOptions.add("in_srs", header.srs());
    filterOptions.add("out_srs", newCoordSystem.getProj4());

    ReprojectionFilter repro;
    repro.addOptions(filterOptions);
    repro.setInput(reader);

    cout << "Executing" << endl;
    repro.prepare(table);
    PointViewSet set = repro.execute(table);

    std::cout << "Finished Reading" << std::endl;

    PointViewPtr view = *set.begin();

    cout << endl << "Data:" << endl;
    cout << "(";
    cout << view->getFieldAs<double>(Dimension::Id::X, 0) << ", ";
    cout << view->getFieldAs<double>(Dimension::Id::Y, 0) << ", ";
    cout << view->getFieldAs<double>(Dimension::Id::Z, 0) << ")" << endl;

    return 0;
}