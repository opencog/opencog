/*
 * opencog/embodiment/Control/MessagingSystem/NetworkElementCommon.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include "NetworkElementCommon.h"
#include <boost/algorithm/string.hpp>
#include <errno.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/Logger.h>

using namespace opencog::messaging;

const std::string NetworkElementCommon::OK_MESSAGE = "OK";
const std::string NetworkElementCommon::FAILED_MESSAGE = "FAILED";
const std::string NetworkElementCommon::FAILED_TO_CONNECT_MESSAGE = "FAILED_TO_CONNECT";

void NetworkElementCommon::parseCommandLine(const std::string &line,
        std::string &command, std::queue<std::string> &args)
{
    std::vector<std::string> parameters;
    boost::split( parameters, line, boost::is_any_of( " " ) );
    if ( parameters.size( ) > 0 ) {
        command = parameters[0];
    }

    unsigned int j;
    for ( j = 1; j < parameters.size( ); ++j ) {
        args.push( parameters[j] );
    }
}

void NetworkElementCommon::reportThreadError(const int &errorCode)
{

    std::string errorMsg;
    switch (errorCode) {
    case EAGAIN:
        errorMsg = "The system lacked the necessary resources to create another thread,"
                   " or the system-imposed limit on the total number of threads in a "
                   "process PTHREAD_THREADS_MAX would be exceeded.";
        break;
    case EINVAL:
        errorMsg = "The value specified by attr is invalid.";
        break;
    case EPERM:
        errorMsg = "The caller does not have appropriate permission to set the required "
                   "scheduling parameters or scheduling policy.";
        break;
    default:
        errorMsg = "Unknown error code: ";
        errorMsg += opencog::toString(errorCode);
        // When pthread_join is not called, this gets error code 12 after 380 created threads!
    }
    opencog::logger().error(
                 "Thread error: \n%s.", errorMsg.c_str());
}

