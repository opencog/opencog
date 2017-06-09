/*
 * opencog/cogserver/server/LearnSentenceRequest.cc
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

#include "LearnSentenceRequest.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/types.h>
#include <opencog/cogserver/server/CogServer.h>

#include <opencog/nlp/learn/LearnSentence.h>

using namespace opencog;

LearnSentenceRequest::LearnSentenceRequest(CogServer& cs) : Request(cs)
{
}

LearnSentenceRequest::~LearnSentenceRequest()
{
    logger().debug("[LearnSentenceRequest] destructor");
}

bool LearnSentenceRequest::syntaxError()
{
    _error << "invalid syntax" << std::endl;
    sendError();
    return false;
} 

bool LearnSentenceRequest::execute()
{
    AtomSpace& as = _cogserver.getAtomSpace();

    if (_parameters.empty()) {
        _error << "Error: a sentence is required" << std::endl;
        sendError();
        return false;
    }
    _sentence = _parameters.front();
    learn_sentence(&as, _sentence);
    sendOutput();
    return true;
}

void LearnSentenceRequest::sendOutput()
{
    std::ostringstream oss;
    oss << "Processed sentence: \"" << _sentence << "\"" << std::endl;
    send(oss.str());
}

void LearnSentenceRequest::sendError()
{
    _error << "Format: learn \"Some sentence.\"" << std::endl;
    if (false) {
        _error << "Supported options:" << std::endl;
        _error << "-a <algorithm>   Learn with the specific algorithm." << std::endl;
        _error << "Options may be combined" << std::endl;
    }

    send(_error.str());
}
