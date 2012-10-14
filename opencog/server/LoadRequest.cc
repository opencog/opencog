/*
 * opencog/server/LoadRequest.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "LoadRequest.h"

#include <vector>

#include <opencog/server/CogServer.h>
#include <opencog/persist/xml/FileXMLBufferReader.h>
#include <opencog/util/Logger.h>
#include <opencog/persist/xml/NMXmlParser.h>
#include <opencog/persist/xml/XMLBufferReader.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

LoadRequest::LoadRequest()
{
}

LoadRequest::~LoadRequest()
{
}

bool LoadRequest::execute()
{
    logger().debug("[LoadRequest] execute");
    std::ostringstream oss;

    if (_parameters.empty()) {
        oss << "invalid syntax: load <filename>" << std::endl;
        send(oss.str());
        return false;
    }

    std::string& filename = _parameters.front();
    std::vector<XMLBufferReader*> readers(1, new FileXMLBufferReader(filename.c_str()));
    try {
        NMXmlParser::loadXML(readers, &server().getAtomSpace());
    } catch (StandardException &e) {
        oss << "Error: unable to load inline xml data (" << e.getMessage() << ")" << std::endl;
        send(oss.str());
        return false;
    }
    oss << "done" << std::endl;
    send(oss.str());
    return true;
}
