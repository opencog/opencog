/*
 * opencog/embodiment/Control/MessagingSystem/NetworkElementCommon.h
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


#ifndef NETWORKELEMENTCOMMON_H
#define NETWORKELEMENTCOMMON_H

#include <string>
#include <queue>

namespace opencog { namespace messaging {

/**
 * Common definitions used by classes using NetworkElement protocol
 */
class NetworkElementCommon
{

public:

    static const std::string OK_MESSAGE;
    static const std::string FAILED_MESSAGE;
    static const std::string FAILED_TO_CONNECT_MESSAGE;

    /**
     * Convenience method to parse command lines into commands and arguments.
     */
    static void parseCommandLine(const std::string &line, std::string &command, std::queue<std::string> &args);

    /**
     * Log thread error message based on standard error codes.
     *
     * @param erroCode The errorCode that should be translated into some meaningfull message
     */
    static void reportThreadError(const int &errorCode);

}; // class

} } // namespace opencog::messaging

#endif
