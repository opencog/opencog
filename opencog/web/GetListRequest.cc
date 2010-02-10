/*
 * opencog/rest/GetListRequest.cc
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

#include "GetListRequest.h"
#include "BaseURLHandler.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/types.h>
#include <opencog/server/CogServer.h>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>

using namespace opencog;

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by LTI
 */
bool getListSortByLTIPredicate (bool descending,
                                const Handle& h1, const Handle& h2) {
    AttentionValue::lti_t lti1, lti2;
    AtomSpace* as = server().getAtomSpace();

    lti1 = as->getLTI(h1);
    lti2 = as->getLTI(h2);

    if (descending)
        return lti1 > lti2;
    else
        return lti1 < lti2;
}

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by STI
 */
bool getListSortBySTIPredicate (bool descending,
                                const Handle& h1, const Handle& h2) {
    AttentionValue::sti_t sti1, sti2;
    AtomSpace* as = server().getAtomSpace();

    sti1 = as->getSTI(h1);
    sti2 = as->getSTI(h2);

    if (descending)
        return sti1 > sti2;
    else
        return sti1 < sti2;
}

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by TV Strength
 */
bool getListSortByTVStrengthPredicate (bool descending,
                                const Handle& h1, const Handle& h2) {
    float tv1, tv2;
    AtomSpace* as = server().getAtomSpace();

    tv1 = as->getTV(h1).getMean();
    tv2 = as->getTV(h2).getMean();

    if (descending)
        return tv1 > tv2;
    else
        return tv1 < tv2;
}

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by TV Confidence
 */
bool getListSortByTVConfidencePredicate (bool descending,
                                const Handle& h1, const Handle& h2) {
    float tv1, tv2;
    AtomSpace* as = server().getAtomSpace();

    tv1 = as->getTV(h1).getConfidence();
    tv2 = as->getTV(h2).getConfidence();

    if (descending)
        return tv1 > tv2;
    else
        return tv1 < tv2;
}


GetListRequest::GetListRequest() : order_by(""), descending(true), name(""),
    type(NOTYPE), subtypes(false)
{
    
}

GetListRequest::~GetListRequest()
{
    logger().debug("[GetListRequest] destructor");
}

bool GetListRequest::execute()
{
    Handle handle = Handle::UNDEFINED;
    HandleSeq _handles;
    AtomSpace* as = server().getAtomSpace();

    std::list<std::string>::const_iterator it;
    for (it = _parameters.begin(); it != _parameters.end(); ++it) {
        std::vector<std::string> keyvalue;
        boost::split(keyvalue, *it, boost::is_any_of("="));
        if (keyvalue.size() != 2) {
            _output << "Bad syntax" << std::endl;
            send(_output.str());
            return false;
        }
        if (keyvalue[0] == "handle") { // get by handle
            UUID uuid = strtol(keyvalue[1].c_str(), NULL, 0);
            handle = Handle(uuid);
            if (TLB::isInvalidHandle(handle)) {
                _output << "Invalid handle: " << uuid << std::endl;
                send(_output.str());
                return false;
            }
            requestHandles.push_back(handle);
            _handles.push_back(handle);
        } else if (keyvalue[0] == "name")  { // filter by name
            name.assign(keyvalue[1]);
        } else if (keyvalue[0] == "type") { // filter by type, excluding subtypes
            type = classserver().getType(keyvalue[1].c_str());
            if (type == NOTYPE) {
                _output << "Invalid type: " << keyvalue[1] << std::endl;
                send(_output.str());
                return false;
            }
        }
        else if (keyvalue[0] == "subtype") { // include subtypes
            boost::to_lower(keyvalue[1]);
            if (keyvalue[1] == "1" || keyvalue[1]  == "true") {
                subtypes = true;
            }
        }
        else if (keyvalue[0] == "order") { // sort list by... 
            order_by = keyvalue[1];
            boost::to_lower(order_by);
        }
        else if (keyvalue[0] == "ascend") { // sort in the reverse direction
            boost::to_lower(keyvalue[1]);
            if (keyvalue[1] == "1" || keyvalue[1]  == "true") {
                descending = false;
            }
        }
        //! @todo deal with refresh and unknown parameters
    }
    if (name != "" && type != NOTYPE) { // filter by name & type
        as->getHandleSet
            (std::back_inserter(_handles), type, name.c_str(), subtypes);
    } else if (name != "") {     // filter by name
        as->getHandleSet(std::back_inserter(_handles), ATOM, name.c_str(), true);
    } else if (type != NOTYPE) { // filter by type
        as->getHandleSet(std::back_inserter(_handles), type, subtypes);
    }
    if (order_by != "") sortHandles(_handles, order_by, descending);
    makeOutput(_handles);
    send(_output.str()); // send output to RequestResult instance
    return true;
}

void GetListRequest::sortHandles(HandleSeq &hs, std::string order_by,
        bool descend)
{
    if (order_by == "lti") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortByLTIPredicate, descend, _1, _2));
    } else if (order_by == "sti") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortBySTIPredicate, descend, _1, _2));
    } else if (order_by == "tv.s") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortByTVStrengthPredicate, descend, _1, _2));
    } else if (order_by == "tv.c") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortByTVConfidencePredicate, descend, _1, _2));
    }

}

void GetListRequest::makeListHeader()
{
    std::vector<std::string> params;
    std::ostringstream querystring;
    std::ostringstream tmpstring;

    // Build the base query string without the ordering behaviour
    querystring << SERVER_PLACEHOLDER << "/list?";
    for (uint i = 0; i < requestHandles.size(); i++) {
        tmpstring << "handle=" << requestHandles[i].value();
        params.push_back(tmpstring.str());
        tmpstring.str("");
    }
    if (type != NOTYPE) {
        tmpstring << "type=" << classserver().getTypeName(type);
        params.push_back(tmpstring.str());
        tmpstring.str("");
    }
    if (subtypes) {
        tmpstring << "subtypes=1";
        params.push_back(tmpstring.str());
        tmpstring.str("");
    }
    if (name != "") {
        tmpstring << "name=" << name;
        params.push_back(tmpstring.str());
        tmpstring.str("");
    }
    for (uint i = 0; i < params.size(); i++) {
        if (i > 0) querystring << "&";
        querystring << params[i];
    }
    
    // Make the header for the table
    _output << "<th>Name</th> <th>Type</th> ";

    // STI
    tmpstring << querystring.str() << "&order=STI";
    _output << "<th>STI [<a href=\"" << tmpstring.str() << "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << "&order=STI&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>]</th>";
    tmpstring.str("");
    
    // LTI
    tmpstring << querystring.str() << "&order=LTI";
    _output << "<th>LTI [<a href=\"" << tmpstring.str() << "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << "&order=LTI&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>]</th>";
    tmpstring.str("");

    // TV.s
    tmpstring << querystring.str() << "&order=TV.s";
    _output << "<th>TruthValue [S: <a href=\"" << tmpstring.str() <<
        "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << "&order=TV.s&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << "&order=TV.c";
    _output << " conf: <a href=\"" << tmpstring.str() << "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << "&order=TV.c&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>]</th>";
    tmpstring.str("");
        
    _output << " <th>Outgoing</th> <th>Incoming</th> </tr>" << std::endl; 

}

void GetListRequest::makeOutput(HandleSeq &hs)
{
    AtomSpace* as = server().getAtomSpace();
    // Make output from atom objects so we can access and create output from
    // them
    _output << "<table border=\"1\"><tr>";

    makeListHeader();

    std::vector<Handle>::const_iterator it;
    for (it = hs.begin(); it != hs.end(); ++it) {
        Handle h = *it;
        _output << "<tr>" << std::endl;
        _output << "<td><a href=\"" << SERVER_PLACEHOLDER
            << "/atom?handle=" << h.value() << "\">"
            << as->getName(h) << "</a></td>";
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

}
