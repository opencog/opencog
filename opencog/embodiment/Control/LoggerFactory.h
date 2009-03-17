/**
 * LoggerFactory.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jul 10 12:34:20 BRT 2007
 */

#ifndef LOGGERFACTORY_H
#define LOGGERFACTORY_H

#include <string>
#include "util/Logger.h"
#include "SystemParameters.h"

namespace Control {

class LoggerFactory {

    private:

        LoggerFactory();

    public:

        static opencog::Logger* getLogger(const SystemParameters &parameters, const std::string &id);

}; // class
}  // namespace

#endif
