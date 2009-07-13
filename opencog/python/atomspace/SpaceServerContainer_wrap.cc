#include "SpaceServerContainer_wrap.h"
#include <boost/python.hpp>

using namespace opencog;
using namespace boost::python;

void init_SpaceServerContainer_py()
{
    class_<SpaceServerContainerWrap, boost::noncopyable>("SpaceServerContainer", 
    no_init)
        .def("mapRemoved", pure_virtual(&SpaceServerContainer::mapRemoved))
        .def("mapPersisted",
            pure_virtual(&SpaceServerContainer::mapPersisted))
        .def("getMapIdString",
            pure_virtual(&SpaceServerContainer::getMapIdString))
    ;
}

// For the pure virtual functions.

void SpaceServerContainerWrap::mapRemoved(Handle mapId)
{
    this->get_override("mapRemoved")();
}
void SpaceServerContainerWrap::mapPersisted(Handle mapId)
{
    this->get_override("mapPersisted")();
}
std::string SpaceServerContainerWrap::getMapIdString(Handle mapId)
{
    return this->get_override("getMapIdString")();
}
