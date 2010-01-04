/*
 * opencog/rest/GetAtomRequest.cc
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include "GetAtomRequest.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/types.h>
#include <opencog/server/CogServer.h>

#include <boost/algorithm/string.hpp>

using namespace opencog;

GetAtomRequest::GetAtomRequest()
{
}

GetAtomRequest::~GetAtomRequest()
{
    logger().debug("[GetAtomRequest] destructor");
}

bool GetAtomRequest::execute()
{
    Handle handle = Handle::UNDEFINED;

    std::list<std::string>::const_iterator it;
    for (it = _parameters.begin(); it != _parameters.end(); ++it) {
        std::vector<std::string> keyvalue;
        boost::split(keyvalue, *it, boost::is_any_of("="));
        if (keyvalue.size() != 2) {
            _output << "Bad syntax" << std::endl;
            return false;
        }
        if (keyvalue[0] == "handle") { // get by handle
            UUID uuid = strtol(keyvalue[1].c_str(), NULL, 0);
            handle = Handle(uuid);
        }
    }
    if (TLB::isInvalidHandle(handle)) {
        _output << "Invalid handle: " << handle.value() << std::endl;
        return false;
    }
    makeOutput(handle);
    return true;
}

#define SERVER_PLACEHOLDER "REST_SERVER_ADDRESS"
void GetAtomRequest::makeOutput(Handle h)
{
    AtomSpace* as = server().getAtomSpace();
    // Make output from atom objects so we can access and create output from
    // them
    _output << "<table border=\"1\"><tr>";

    _output << "<th>Name</th> <th>Type</th> <th>STI</th> <th>LTI</th>"
        "<th>TruthValue</th> <th>Outgoing</th> <th>Incoming</th> </tr>" << std::endl; 

    _output << "<tr>" << std::endl;
    _output << "<td>" << as->getName(h) << "</td>";
    _output << "<td>" << classserver().getTypeName(as->getType(h)) << "</td> ";
    AttentionValue::sti_t the_sti = as->getSTI(h) ;
    AttentionValue::lti_t the_lti = as->getLTI(h) ;
    //! @todo make the sti/lti color scaled instead of just -ve/+ve
    if (the_sti > 0)
        _output << "<td style=\"background-color:#99FF66\">" << the_sti << "</td> ";
    else
        _output << "<td style=\"background-color:#99FFFF\">" << the_sti << "</td> ";
    if (the_lti > 0)
        _output << "<td style=\"background-color:#99FF66\">" << the_lti << "</td> ";
    else
        _output << "<td style=\"background-color:#99FFFF\">" << the_lti << "</td> ";
    _output << "<td>" << as->getTV(h).toString() << "</td> ";

    // Here the outgoing targets string is made
    HandleSeq outgoing = as->getOutgoing(h);
    _output << "<td>";
    for (uint i = 0; i < outgoing.size(); i++) {
        Handle ho = outgoing[i];
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/list?type=" <<
            classserver().getTypeName(as->getType(ho)) << "\">";
        _output << classserver().getTypeName(as->getType(ho));
        _output << "</a>:";
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/atom?handle=" <<
            ho.value() << "\">";
        if (as->getName(ho) == "")
            _output << "#" + ho.value();
        else
            _output << as->getName(ho);
        _output << "</a><br/>";
    }
    _output << "</td>";

    // Here the incoming string is made.
    HandleSeq incoming = as->getIncoming(h);
    _output << "<td>";
    for (uint i = 0; i < incoming.size(); i++) {
        Handle ho = incoming[i];
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/list?type=" <<
            classserver().getTypeName(as->getType(ho)) << "\">";
        _output << classserver().getTypeName(as->getType(ho));
        _output << "</a>:";
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/atom?handle=" <<
            ho.value() << "\">";
        if (as->getName(ho) == "")
            _output << "#" << ho.value();
        else
            _output << as->getName(ho) << ":";
        _output << "</a><br/>";
    }
    _output << "</td>";
    _output << "</tr>" << std::endl;

}

std::string GetAtomRequest::getHTML(std::string server_string) 
{
    std::string output = _output.str();
    boost::replace_all(output, SERVER_PLACEHOLDER,
            server_string);
    return output;
}
