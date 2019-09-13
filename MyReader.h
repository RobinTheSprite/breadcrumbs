//
// Created by Mark on 9/11/2019.
//

#ifndef PATHFINDER_MYREADER_H
#define PATHFINDER_MYREADER_H

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>
#include <memory>

namespace pdal
{

    class MyReader : public pdal::Reader
    {
    public:
        MyReader() : _Zscale{1.0}, _stream{nullptr}, _index{0}, Reader(){};
        std::string getName() const;

        static void * create();
        static int32_t destroy(void *);
    private:
        std::unique_ptr<ILeStream> _stream;
        point_count_t _index;
        double _Zscale;

        void addDimensions(PointLayoutPtr layoutPtr) override;
        void addArgs(ProgramArgs &) override;
        void ready(PointTableRef) override;
        pdal::point_count_t read(PointViewPtr view, point_count_t count) override;
        void done(PointTableRef) override;
    };
}

#endif //PATHFINDER_MYREADER_H
