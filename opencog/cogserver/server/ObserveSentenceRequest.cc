/*
 * opencog/cogserver/server/ObserveSentenceRequest.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Curtis Faith
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

#include "ObserveSentenceRequest.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/types.h>
#include <opencog/cogserver/server/CogServer.h>

#include <opencog/nlp/learn/ObserveSentence.h>

using namespace opencog;

ObserveSentenceRequest::ObserveSentenceRequest(CogServer& cs) : Request(cs)
{
}

ObserveSentenceRequest::~ObserveSentenceRequest()
{
    logger().debug("[ObserveSentenceRequest] destructor");
}

bool ObserveSentenceRequest::syntaxError()
{
    _error << "invalid syntax" << std::endl;
    sendError();
    return false;
} 

bool ObserveSentenceRequest::execute()
{
    std::list<std::string>::const_iterator it;

    if (_parameters.empty())
    {
        _error << "Error: observe - a sentence is required" << std::endl;
        sendError();
        return false;
    }

    std::ostringstream oss;

    // Extract the parameters.
    int parameter_count = 0;
    int pair_distance = 0;
    bool noop = false;
    bool verbose = false;
    for (it = _parameters.begin(); it != _parameters.end(); ++it)
    {
        parameter_count++;

        // -pair_distance <limit> - limit distance of word pairs
        if (*it == "-pair_distance")
        {
            ++it;
            if (it == _parameters.end()) return syntaxError();
            pair_distance = atoi(it->c_str());       
        }
        // -noop - do not perform any operations
        else if (*it == "-noop")
        { 
            noop = true;
        }
        // -verbose - send return confirmation of sentence processed
        else if (*it == "-verbose")
        { 
            verbose = true;
        }
        // This should be a sentence.
        else
        {
            _sentence = *it;
        }
    }

    if (!noop)
    {
        AtomSpace& as = _cogserver.getAtomSpace();
        observe_sentence(&as, _sentence, pair_distance);
    }
    if (verbose)
        sendOutput();
    return true;
}

void ObserveSentenceRequest::sendOutput()
{
    std::ostringstream oss;
    oss << "Processed sentence: \"" << _sentence << "\"" << std::endl;
    send(oss.str());
}

void ObserveSentenceRequest::sendError()
{
    _error << "Format: observe [-pair_distance <limit>] \"<sentence-in-quotes>\"" << std::endl;
    _error << "Supported options:" << std::endl;
    _error << "    -pair_distance <limit>   Create pairs up to <limit> distance apart." << std::endl;
    _error << "    -quiet                   Do not return status over telnet." << std::endl;
    _error << "    -noop                    Perform no op-erations (useful for timing)." << std::endl;
    _error << "Options may be combined" << std::endl << std::endl;
    send(_error.str());
}
