#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/copy_const_reference.hpp>

#include <opencog/util/Logger.h>

#include "Logger_wrap.h"

using namespace opencog;
using namespace boost::python;

void init_Logger_py()
{
    // Expose Logger class.
    class_<Logger> logger("Logger", no_init);
    logger.def(init< optional<const std::string, Logger::Level, bool> >());
    logger.def(init<const Logger&>());
    logger.def("setLevel", &Logger::setLevel);
    logger.def("getLevel", &Logger::getLevel);
    logger.def("setBackTraceLevel", &Logger::setBackTraceLevel);
    logger.def("getBackTraceLevel", &Logger::getBackTraceLevel);
    logger.def("setFilename", &Logger::setFilename);
    logger.def("getFilename", &Logger::getFilename,
        return_value_policy<copy_const_reference>());
    //logger.def("log", &Logger::log);
    logger.def("enable", &Logger::enable);
    logger.def("disable", &Logger::disable);

    // Change scope to be within the just-exposed Logger class.
    scope within(logger);

    // Now expose Level enum (under new scope).
    enum_<Logger::Level>("Level")
        .value("NONE", Logger::NONE)
        .value("ERROR", Logger::ERROR)
        .value("WARN", Logger::WARN)
        .value("INFO", Logger::INFO)
        .value("DEBUG", Logger::DEBUG)
        .value("FINE", Logger::FINE)
    ;
}
