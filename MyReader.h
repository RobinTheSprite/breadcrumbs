//
// Created by Mark on 9/11/2019.
//

#ifndef PATHFINDER_MYREADER_H
#define PATHFINDER_MYREADER_H

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>

namespace pdal
{

    class MyReader : public pdal::Reader
    {
    public:
        MyReader() : pdal::Reader()
        {};
    private:
        void addDimensions(PointLayoutPtr layoutPtr) override;
        void addArgs(ProgramArgs &) override;
        pdal::point_count_t read(PointViewPtr view, point_count_t count) override;
    };
}

#endif //PATHFINDER_MYREADER_H
