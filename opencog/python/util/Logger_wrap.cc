#include <boost/python/class.hpp>

#include <opencog/util/Logger.h>

#include "Logger_wrap.h"

using namespace boost::python;
using namespace opencog;

void init_Logger_py()
{
    class_<Logger>("Logger", no_init)
        .def(init< optional<const std::string, Logger::Level, bool> >())
        .def(init<const Logger&>())
    ;
}
