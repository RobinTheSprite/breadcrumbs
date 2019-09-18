#include <iostream>
#include <pdal/io/LasReader.hpp>

using std::cout;
using std::endl;

using namespace pdal;

int main()
{
    LasReader reader;

    Options options;
//    options.add("filename", "../test_1.laz");
    options.add("filename", "../test_1.laz");
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);

    std::cout << "Reading LAZ file..." << std::endl;
    PointViewSet set = reader.execute(table);
    std::cout << "Finished Reading" << std::endl;

    PointViewPtr view = *set.begin();

    cout << endl << "Data:" << endl;
    for (unsigned int i = 0; i < view->size(); ++i)
    {
        cout << "(";
        cout << view->getFieldAs<int>(Dimension::Id::X, i) << ", ";
        cout << view->getFieldAs<int>(Dimension::Id::Y, i) << ", ";
        cout << view->getFieldAs<int>(Dimension::Id::Z, i) << ")" << endl;
    }

    return 0;
}