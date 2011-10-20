/*
 * opencog/server/DataRequest.cc
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

#include "DataRequest.h"

#include <vector>

#include <opencog/server/CogServer.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>
#include <opencog/persist/xml/NMXmlParser.h>
#include <opencog/persist/xml/StringXMLBufferReader.h>
#include <opencog/persist/xml/XMLBufferReader.h>

using namespace opencog;

DataRequest::DataRequest()
{
}

DataRequest::~DataRequest()
{
}

void DataRequest::setRequestResult(RequestResult* rr)
{
    Request::setRequestResult(rr);
    rr->SetDataRequest();
}

bool DataRequest::execute()
{
    std::string& xmldata = _parameters.front();
    // remove trailing white space and line breaks
    xmldata.erase(xmldata.find_last_not_of(" \t\n\r"));
    std::vector<XMLBufferReader*> readers(1, new StringXMLBufferReader(xmldata.c_str()));
    try {
        NMXmlParser::loadXML(readers, server().getAtomSpace());
    } catch (StandardException &e) {
        std::ostringstream oss;
        oss << "Error: unable to load inline xml data (" << e.getMessage() << ")" << std::endl;
        send(oss.str());
        return false;
    }
    send("done\n");
    return true;
}
