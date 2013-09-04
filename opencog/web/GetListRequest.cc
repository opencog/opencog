/*
 * opencog/rest/GetListRequest.cc
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * Copyright (C) 2010 by Joel Pitt
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

#include "GetAtomRequest.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/types.h>
#include <opencog/server/CogServer.h>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

using namespace opencog;

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by LTI
 */
bool getListSortByLTIPredicate (CogServer* cs, bool descending,
                                const Handle& h1, const Handle& h2)
{
    AttentionValue::lti_t lti1, lti2;
    AtomSpace& as = cs->getAtomSpace();

    lti1 = as.getLTI(h1);
    lti2 = as.getLTI(h2);

    if (descending)
        return lti1 > lti2;
    else
        return lti1 < lti2;
}

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by STI
 */
bool getListSortBySTIPredicate (CogServer* cs, bool descending,
                                const Handle& h1, const Handle& h2)
{
    AttentionValue::sti_t sti1, sti2;
    AtomSpace& as = cs->getAtomSpace();

    sti1 = as.getSTI(h1);
    sti2 = as.getSTI(h2);

    if (descending)
        return sti1 > sti2;
    else
        return sti1 < sti2;
}

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by TV Strength
 */
bool getListSortByTVStrengthPredicate (CogServer* cs, bool descending,
                                const Handle& h1, const Handle& h2)
{
    float tv1, tv2;
    AtomSpace& as = cs->getAtomSpace();

    tv1 = as.getMean(h1);
    tv2 = as.getMean(h2);

    if (descending)
        return tv1 > tv2;
    else
        return tv1 < tv2;
}

/**
 * Comparison operator for using qsort on a vector of Handles.
 * Sorts by TV Confidence
 */
bool getListSortByTVConfidencePredicate (CogServer* cs, bool descending,
                                const Handle& h1, const Handle& h2)
{
    float tv1, tv2;
    AtomSpace& as = cs->getAtomSpace();

    tv1 = as.getConfidence(h1);
    tv2 = as.getConfidence(h2);

    if (descending)
        return tv1 > tv2;
    else
        return tv1 < tv2;
}


GetListRequest::GetListRequest(CogServer& cs) :
    Request(cs),
    output_format(html_tabular_format), 
    order_by(""), descending(true), name(""),
    type(NOTYPE), subtypes(false), maximum(GETLIST_MAXIMUM_RESULTS), skip(0)
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
    std::ostringstream errbuff;
    AtomSpace& as = _cogserver.getAtomSpace();

    std::list<std::string>::const_iterator it;
    for (it = _parameters.begin(); it != _parameters.end(); ++it) {
        std::vector<std::string> keyvalue;
        boost::split(keyvalue, *it, boost::is_any_of("="));
        if (keyvalue.size() != 2) {
            errbuff << "Bad syntax";
            sendError(errbuff.str());
            return false;
        }
        if (keyvalue[0] == "handle") { // get by handle
            UUID uuid = strtol(keyvalue[1].c_str(), NULL, 0);
            handle = Handle(uuid);
            if (!as.isValidHandle(handle)) {
                errbuff << "Invalid handle: " << uuid;
                sendError(errbuff.str());
                return false;
            }
            requestHandles.push_back(handle);
            _handles.push_back(handle);
        } else if (keyvalue[0] == "name")  { // filter by name
            name.assign(keyvalue[1]);
        } else if (keyvalue[0] == "type") { // filter by type, excluding subtypes
            type = classserver().getType(keyvalue[1].c_str());
            if (type == NOTYPE) {
                errbuff << "Invalid type: " << keyvalue[1];
                sendError(errbuff.str());
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
        else if (keyvalue[0] == "max") { // maximum results to return
            try {
                maximum = boost::lexical_cast<int>(keyvalue[1]);
            } catch (boost::bad_lexical_cast) {
                errbuff << "Couldn't interpret max parameter";
                sendError(errbuff.str());
                return false;
            }
        }
        else if (keyvalue[0] == "skip") { // maximum results to return
            try {
                skip = boost::lexical_cast<int>(keyvalue[1]);
                if (skip < 0) skip = 0;
            } catch (boost::bad_lexical_cast) {
                errbuff << "Couldn't interpret skip parameter";
                sendError(errbuff.str());
                return false;
            }
        }
        else if (keyvalue[0] == "ascend") { // sort in the reverse direction
            boost::to_lower(keyvalue[1]);
            if (keyvalue[1] == "1" || keyvalue[1]  == "true") {
                descending = false;
            }
        }
        // URL handler adds a format parameter to get JSON output.
        else if (keyvalue[0] == "format") {
            boost::to_lower(keyvalue[1]);
            if (keyvalue[1] == "json") output_format = json_format;
        }
        //! @todo deal with refresh and unknown parameters
    }
    if(_handles.empty()){
    	if (name != "" && type != NOTYPE) { // filter by name & type
			as.getHandleSet
				(std::back_inserter(_handles), type, name.c_str(), subtypes);
		} else if (name != "") {     // filter by name
			as.getHandleSet(std::back_inserter(_handles), ATOM, name.c_str(), true);
		} else if (type != NOTYPE) { // filter by type
			as.getHandleSet(std::back_inserter(_handles), type, subtypes);
		}
    }
    if (order_by != "") {
        bool sortResult = sortHandles(_handles, order_by, descending);
        if (!sortResult) {
            send(_output.str()); 
            return false;
        }
    }
    if (output_format == json_format) json_makeOutput(_handles);
    else html_makeOutput(_handles);
    send(_output.str()); // send output to RequestResult instance
    return true;
}

void GetListRequest::sendError(std::string err)
{
    //! @todo: different responses for different formats json/html
    _output << err << std::endl;
    send(_output.str());
}

bool GetListRequest::sortHandles(HandleSeq &hs, std::string order_by,
        bool descend)
{
    if (order_by == "lti") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortByLTIPredicate, &_cogserver, descend, _1, _2));
    } else if (order_by == "sti") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortBySTIPredicate, &_cogserver, descend, _1, _2));
    } else if (order_by == "tv.s") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortByTVStrengthPredicate, &_cogserver, descend, _1, _2));
    } else if (order_by == "tv.c") {
        std::sort(hs.begin(), hs.end(),
                boost::bind(getListSortByTVConfidencePredicate, &_cogserver, descend, _1, _2));
    } else {
        _output << "unknown sort order" << std::endl;
        return false;
    }
    return true;

}

void GetListRequest::html_makeListHeader(unsigned int total_results)
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
        tmpstring << "subtype=1";
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
    
    std::ostringstream orderstr;
    if (order_by != "") {
        orderstr << "&order=" << order_by;
        if (!descending) orderstr << "&ascend=1";
    }
    _output << "<p><small>Viewing atoms " << skip+1 << " to ";
    if ((uint)skip+maximum < total_results)
        _output << skip+maximum;
    else
        _output << total_results;
    _output  << " (of " << total_results << ")</small></p>" << std::endl;
    std::ostringstream maxstring;
    maxstring << "&max=" << maximum;
    if (skip > 0) {
        int lskip = skip-maximum;
        if (lskip < 0) lskip = 0;
        _output << "<a href=\"" << querystring.str() << "&skip=0" <<
            maxstring.str() << orderstr.str() << "\">&lArr;" << "</a> ";
        _output << "<a href=\"" << querystring.str() << "&skip=" << lskip <<
            maxstring.str() << orderstr.str() << "\">&larr;" << "</a> ";
    } else {
        _output << "&lArr; &larr; ";
    }
    if ((unsigned int)skip+maximum < total_results) {
        int rskip = skip+maximum;
        _output << "<a href=\"" << querystring.str() << "&skip=" << rskip <<
            maxstring.str() << orderstr.str() << "\">&rarr;" << "</a> ";
        _output << "<a href=\"" << querystring.str() << "&skip=" <<
            total_results-maximum <<
            maxstring.str() << orderstr.str() << "\">&rArr;" << "</a> ";
    } else {
        _output << "&rarr; &rArr;";
    }
    _output << "<br/>" << std::endl;
    _output << "<small>Per page: ";
    int maximums[] = { 10, 20, 50, 100 };
    for (int m = 0; m < 4; m++) {
        if (maximums[m] == maximum) {
            _output << maximums[m] << " ";
        } else {
            _output << "<a href=\"" << querystring.str() << orderstr.str() << "&max=" << maximums[m]
                << "&skip=" << skip << "\">" << maximums[m] << "</a> ";
        }
    }
    _output << "</small>" << std::endl;

    // Make the header for the table
    _output << "<th>Name</th> <th>Type</th> ";

    // STI
    tmpstring << querystring.str() << maxstring.str() << "&order=STI";
    _output << "<th>STI [<a href=\"" << tmpstring.str() << "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << maxstring.str() << "&order=STI&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>]</th>";
    tmpstring.str("");
    
    // LTI
    tmpstring << querystring.str() << maxstring.str() << "&order=LTI";
    _output << "<th>LTI [<a href=\"" << tmpstring.str() << "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << maxstring.str() << "&order=LTI&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>]</th>";
    tmpstring.str("");

    // TV.s
    tmpstring << querystring.str() << maxstring.str() << "&order=TV.s";
    _output << "<th>TruthValue [S: <a href=\"" << tmpstring.str() <<
        "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << maxstring.str() << "&order=TV.s&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>";
    tmpstring.str("");
    // TV.c
    tmpstring << querystring.str() << maxstring.str() << "&order=TV.c";
    _output << " conf: <a href=\"" << tmpstring.str() << "\">&uarr</a>";
    tmpstring.str("");
    tmpstring << querystring.str() << maxstring.str() << "&order=TV.c&ascend=1";
    _output << "<a href=\"" << tmpstring.str() << "\">&darr</a>]</th>";
    tmpstring.str("");
        
    _output << " <th>Outgoing</th> <th>Incoming</th> </tr>" << std::endl; 

}

void GetListRequest::html_makeOutput(HandleSeq &hs)
{
    AtomSpace& as = _cogserver.getAtomSpace();
    // Make output from atom objects so we can access and create output from
    // them
    _output << "<table border=\"1\"><tr>";

    html_makeListHeader(hs.size());

    std::vector<Handle>::const_iterator it;
    int counter=0;
    for (it = hs.begin(); it != hs.end() && counter < skip+maximum; ++it) {
        counter++;
        if (counter <= skip) continue;
        Handle h = *it;
        _output << "<tr>" << std::endl;
        _output << "<td><a href=\"" << SERVER_PLACEHOLDER
            << "/atom?handle=" << h.value() << "\">"
            << as.getName(h) << "</a></td>";
        _output << "<td>" << classserver().getTypeName(as.getType(h)) << "</td> ";
        AttentionValue::sti_t the_sti = as.getSTI(h) ;
        AttentionValue::lti_t the_lti = as.getLTI(h) ;
        //! @todo make the sti/lti color scaled instead of just -ve/+ve
        if (the_sti > 0)
            _output << "<td style=\"background-color:#99FF66\">" << the_sti << "</td> ";
        else
            _output << "<td style=\"background-color:#99FFFF\">" << the_sti << "</td> ";
        if (the_lti > 0)
            _output << "<td style=\"background-color:#99FF66\">" << the_lti << "</td> ";
        else
            _output << "<td style=\"background-color:#99FFFF\">" << the_lti << "</td> ";
        _output << "<td>" << as.getTV(h)->toString() << "</td> ";

        // Here the outgoing targets string is made
        HandleSeq outgoing = as.getOutgoing(h);
        _output << "<td>";
        for (uint i = 0; i < outgoing.size(); i++) {
            Handle ho = outgoing[i];
            _output << "<a href=\"" << SERVER_PLACEHOLDER << "/list/" <<
                classserver().getTypeName(as.getType(ho)) <<
                "?max=" << maximum << "\">";
            _output << classserver().getTypeName(as.getType(ho));
            _output << "</a>:";
            _output << "<a href=\"" << SERVER_PLACEHOLDER << "/atom/" <<
                ho.value() << "\">";
            if (as.getName(ho) == "")
                _output << "#" + ho.value();
            else
                _output << as.getName(ho);
            _output << "</a><br/>";
        }
        _output << "</td>";

        // Here the incoming string is made.
        HandleSeq incoming = as.getIncoming(h);
        _output << "<td>";
        for (uint i = 0; i < incoming.size(); i++) {
            Handle ho = incoming[i];
            _output << "<a href=\"" << SERVER_PLACEHOLDER << "/list/" <<
                classserver().getTypeName(as.getType(ho)) <<
                "?max=" << maximum << "\">";
            _output << classserver().getTypeName(as.getType(ho));
            _output << "</a>:";
            _output << "<a href=\"" << SERVER_PLACEHOLDER << "/atom/" <<
                ho.value() << "\">";
            if (as.getName(ho) == "")
                _output << "#" << ho.value();
            else
                _output << as.getName(ho) << ":";
            _output << "</a><br/>";
        }
        _output << "</td>";
        _output << "</tr>" << std::endl;
    }

}

void GetListRequest::json_makeOutput(HandleSeq &hs)
{
    // Make output from atom objects so we can access and create output from
    // them
    _output << "{ \"complete\":";
    if (hs.size() > (unsigned int)maximum)
        _output << "false," << std::endl;
    else
        _output << "true," << std::endl;
    _output << "\"skipped\":" << skip << "," << std::endl;
    _output << "\"total\":" << hs.size() << "," << std::endl;
    _output << "\"result\": [ ";

    std::vector<Handle>::const_iterator it;
    int counter=0;
    bool first = true;
    for (it = hs.begin(); it != hs.end() && counter < skip+maximum; ++it) {
        counter++;
        if (counter <= skip) continue;
        Handle h = *it;
        if (!first) _output << ", ";
        else first = false;
        //_output << h.value();
        _output << GetAtomRequest::json_makeOutput(_cogserver, h);
    }
    _output << " ]" << std::endl << 
        "}" << std::endl;

}
