#include <iostream>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/filters/ReprojectionFilter.hpp>
#include <pdal/filters/AssignFilter.hpp>
#include <pdal/filters/ELMFilter.hpp>
#include <pdal/filters/OutlierFilter.hpp>
#include <pdal/filters/SMRFilter.hpp>
#include <pdal/filters/RangeFilter.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

using std::cout;
using std::endl;

using namespace pdal;

int main()
{
    LasReader reader;

    Options options;
    options.add("filename", "../FB17_3764.laz");
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

    Options assignOptions;
    assignOptions.add("assignment", "Classification[:]=0");
    AssignFilter assign;
    assign.addOptions(assignOptions);
    assign.setInput(repro);

    ELMFilter elm;
    elm.setInput(assign);

    OutlierFilter outlier;
    outlier.setInput(elm);

    Options smrfOptions;
    smrfOptions.add("ignore", "Classification[7:7]");
    smrfOptions.add("slope", 0.2);
    smrfOptions.add("window", 16);
    smrfOptions.add("threshold", 0.45);
    smrfOptions.add("scalar", 1.2);
    SMRFilter smrf;
    smrf.setInput(elm);
    smrf.addOptions(smrfOptions);

    Options rangeOptions;
    rangeOptions.add("limits", "Classification[2:2]");
    RangeFilter range;
    range.setInput(smrf);
    range.addOptions(rangeOptions);

    StreamCallbackFilter stream;
    stream.setInput(assign);

    double pointCount = 0;
    int percentage = 0;
    int savedPercentage = 0;
    auto processOne = [&](PointRef & point)
    {
       ++pointCount;
       percentage = (int)(pointCount/header.pointCount() * 100);

       if (percentage != savedPercentage)
       {
           savedPercentage = percentage;
           cout << percentage << "%" << endl;
       }

        return true;
    };

    cout << "Executing" << endl;

    stream.setCallback(processOne);
    stream.prepare(table);
    stream.execute(table);

    return 0;
}