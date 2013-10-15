/*
 * opencog/rest/GetAtomRequest.cc
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

#include "GetAtomRequest.h"
#include "BaseURLHandler.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/CompositeTruthValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/server/CogServer.h>

#include <boost/algorithm/string.hpp>

using namespace opencog;

GetAtomRequest::GetAtomRequest(CogServer& cs) : Request(cs)
{
    output_format = html_tabular_format;
}

GetAtomRequest::~GetAtomRequest()
{
    logger().debug("[GetAtomRequest] destructor");
}

bool GetAtomRequest::execute()
{
    Handle handle = Handle::UNDEFINED;
    AtomSpace& as = _cogserver.getAtomSpace();

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
        // URL handler adds a format parameter to get JSON output.
        else if (keyvalue[0] == "format") {
            boost::to_lower(keyvalue[1]);
            if (keyvalue[1] == "json") output_format = json_format;
        }
    }
    if (!as.isValidHandle(handle)) {
    if (output_format != json_format){
        _output << "Invalid handle: " << handle.value() << std::endl;
    }else{
        _output << "{\"error\": \"Invalid handle "<<handle.value()<<"\"}"<<std::endl;
    }
        send(_output.str());
        return false;
    }
    if (output_format == json_format)
        _output << json_makeOutput(_cogserver, handle);
    else html_makeOutput(handle);
    send(_output.str());
    return true;
}

std::string GetAtomRequest::json_makeOutput(CogServer& cs, Handle h)
{
    std::ostringstream output;
    
    AtomSpace& as = cs.getAtomSpace();
    output << "{\"handle\":" << h.value() << ",";// << std::endl;

    output << "\"type\":\"" << classserver().getTypeName(as.getType(h)) <<
        "\",";// << std::endl;
    output << "\"name\":\"" << as.getName(h) << "\",";// << std::endl;

    // Here the outgoing targets string is made
    HandleSeq outgoing = as.getOutgoing(h);
    output << "\"outgoing\":[";
    for (uint i = 0; i < outgoing.size(); i++) {
        if (i != 0) output << ",";
        output << outgoing[i].value();
    }
    output << "],";// << std::endl;

    // Here the incoming string is made.
    HandleSeq incoming = as.getIncoming(h);
    output << "\"incoming\":[";
    for (uint i = 0; i < incoming.size(); i++) {
        if (i != 0) output << ",";
        output << incoming[i].value();
    }
    output << "],"; // << std::endl;
    output << "\"sti\":" << as.getSTI(h) << ",";// <<std::endl;
    output << "\"lti\":" << as.getLTI(h) << ",";// <<std::endl;

    TruthValuePtr tvp = as.getTV(h);
    output << "\"truthvalue\":" << tvToJSON(tvp);// << std::endl;
    output << "}";

    return output.str();
}

std::string GetAtomRequest::tvToJSON(TruthValuePtr tv)
{
    std::ostringstream jtv;
    if (tv->getType() == SIMPLE_TRUTH_VALUE) {
        jtv << "{\"simple\":{";
        jtv << "\"str\":" << tv->getMean() << ",";
        jtv << "\"count\":" << tv->getCount() << ",";
        jtv << "\"conf\":" << tv->getConfidence() << "}}";
    } else if (tv->getType() == COUNT_TRUTH_VALUE) {
        jtv << "{\"count\":{";
        jtv << "\"str\":" << tv->getMean() << ",";
        jtv << "\"conf\":" << tv->getConfidence() << ",";
        jtv << "\"count\":" << tv->getCount() << "}}";
    } else if (tv->getType() == INDEFINITE_TRUTH_VALUE) {
        IndefiniteTruthValuePtr itv = IndefiniteTVCast(tv);
        jtv << "{\"indefinite\":{";
        jtv << "\"str\":" << itv->getMean() << ",";
        jtv << "\"L\":" << itv->getL() << ",";
        jtv << "\"U\":" << itv->getU() << ",";
        jtv << "\"conf\":" << itv->getConfidenceLevel() << ",";
        //jtv << "\"diff\":" << itv->getDiff() << ",";
        jtv << "\"symmetric\":" << itv->isSymmetric() << "}}";
    } else if (tv->getType() == COMPOSITE_TRUTH_VALUE) {
        CompositeTruthValuePtr ctv = CompositeTVCast(tv);
        jtv << "{\"composite\":";
        jtv << "{\"primary\":" <<
            tvToJSON(ctv->getPrimaryTV());
        foreach(VersionHandle vh, ctv->vh_range()) {
            jtv << "\"" << VersionHandle::indicatorToStr(vh.indicator) << "\":" << std::endl;
            jtv << "[" << vh.substantive << ",";
            jtv << tvToJSON(ctv->getVersionedTV(vh));
            jtv << "]";
        }
        jtv << "}}";
    } else {
        jtv << "{\"unknown\":0}";
    }
    return jtv.str();

}

void GetAtomRequest::html_makeOutput(Handle h)
{
    AtomSpace& as = _cogserver.getAtomSpace();
    // Make output from atom objects so we can access and create output from
    // them
    _output << "<table border=\"1\"><tr>";

    _output << "<th>Name</th> <th>Type</th> <th>STI</th> <th>LTI</th>"
        "<th>TruthValue</th> <th>Outgoing</th> <th>Incoming</th> </tr>" << std::endl; 

    _output << "<tr>" << std::endl;
    _output << "<td>" << as.getName(h) << "</td>";
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
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/list?type=" <<
            classserver().getTypeName(as.getType(ho)) << "\">";
        _output << classserver().getTypeName(as.getType(ho));
        _output << "</a>:";
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/atom?handle=" <<
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
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/list?type=" <<
            classserver().getTypeName(as.getType(ho)) << "\">";
        _output << classserver().getTypeName(as.getType(ho));
        _output << "</a>:";
        _output << "<a href=\"" << SERVER_PLACEHOLDER << "/atom?handle=" <<
            ho.value() << "\">";
        if (as.getName(ho) == "")
            _output << "#" << ho.value();
        else
            _output << as.getName(ho) << ":";
        _output << "</a><br/>";
    }
    _output << "</td>";
    _output << "</tr></table>" << std::endl;

    generateProcessingGraph(h);

}

void GetAtomRequest::generateProcessingGraph(Handle h)
{
    _output << "<p>"
        "</script><canvas datasrc=\"/resources/processhg.pjs\" "
        "width=\"200px\" height=\"200px\"></canvas></p>\n";
}

