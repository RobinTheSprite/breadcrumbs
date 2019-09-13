//
// Created by Mark on 9/11/2019.
//

#include "MyReader.h"
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{



    static const PluginInfo stageInfo
            {
                    "readers.myreader",
                    "My first LAS reader",
                    "https://pdal.io/development/writing-reader.html"
            };

    CREATE_SHARED_PLUGIN(0,1,MyReader, Reader, stageInfo)

    point_count_t MyReader::read(PointViewPtr view, point_count_t count)
    {
        return 0;
    }

    void pdal::MyReader::addDimensions(pdal::PointLayoutPtr layout)
    {

    }

    void pdal::MyReader::addArgs(pdal::ProgramArgs &args)
    {

    }

    void pdal::MyReader::ready(pdal::PointTableRef table)
    {

    }

    void pdal::MyReader::done(PointTableRef & table)
    {
        _stream.reset();
    }

    std::string MyReader::getName() const
    {
        return stageInfo.name;
    }

}