/*
 * opencog/rest/UpdateAtomRequest.cc
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

#include "UpdateAtomRequest.h"
#include "BaseURLHandler.h"
#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/server/CogServer.h>

#include <boost/algorithm/string.hpp>

#include <opencog/web/json_spirit/json_spirit.h>
#include "JsonUtil.h"

using namespace opencog;
using namespace json_spirit;
using namespace std;

UpdateAtomRequest::UpdateAtomRequest(CogServer& cs) :
    Request(cs),
    sti_mod(sti_none), lti_mod(lti_none),
    tv_mod(tv_none), tv(NULL)
{
}

UpdateAtomRequest::~UpdateAtomRequest()
{
    logger().debug("[UpdateAtomRequest] destructor");
}

void UpdateAtomRequest::setRequestResult(RequestResult* rr)
{
    Request::setRequestResult(rr);
    //rr->SetDataRequest();
}

bool UpdateAtomRequest::execute()
{
    AtomSpace* as = &_cogserver.getAtomSpace();

    Handle h;
    if (_parameters.size() != 2) {
        _output << "{\"error\":\"incorrect number of parameters\"}" << std::endl;
        send(_output.str());
        return false;
    }
    std::string& handleStr = _parameters.front();
    std::vector<std::string> keyvalue;
    boost::split(keyvalue, handleStr, boost::is_any_of("="));
    if (keyvalue.size() != 2) {
        _output << "{\"error\":\"parsing handle value\"}" << std::endl;
        send(_output.str());
        return false;
    }
    if (keyvalue[0] == "handle") { // get by handle
        UUID uuid = strtol(keyvalue[1].c_str(), NULL, 0);
        h = Handle(uuid);
        if (!as->isValidHandle(h)) {
            _output << "{\"error\":\"invalid handle\"}" << std::endl;
            send(_output.str());
            return false;
        }
    } else {
        _output << "{\"error\":\"unknown parameter\"}" << std::endl;
        send(_output.str());
        return false;
    }

    std::string& json_data = _parameters.back();

    decode(json_data);

    Value json_top;
    // Instead of applying changes while parsing, we store them
    // in case later changes parsed are invalid.
    AttentionValue::sti_t sti_x=0;
    AttentionValue::lti_t lti_x=0;
    try {
        read( json_data, json_top);
        const Object &json_obj = json_top.get_obj();
        for( Object::size_type i = 0; i != json_obj.size(); ++i ) {
            std::string s;
            const Pair& pair = json_obj[i];
            const std::string& name = pair.name_;
            const Value&  value = pair.value_;
            if (name.substr(0,3) == "sti") {
                // parse STI change
                if (sti_mod != 0) {
                    _output << "{\"error\":\"multiple STI changes\"}" << std::endl;
                    send(_output.str());
                    return false;
                }
                std::string suffix = name.substr(3);
                if (suffix == "") {
                    sti_mod = sti_replace;
                } else if (suffix == "_add") {
                    sti_mod = sti_add;
                } else if (suffix == "_subtract") {
                    sti_mod = sti_subtract;
                } else if (suffix == "_merge") {
                    sti_mod = sti_merge;
                } else {
                    _output << "{\"error\":\"unknown STI modifier\"}" << std::endl;
                    send(_output.str());
                    return false;
                }
                sti_x = value.get_int();
            } else if (name.substr(0,3) == "lti") {
                // parse lti
                if (lti_mod != 0) {
                    _output << "{\"error\":\"multiple LTI changes\"}" << std::endl;
                    send(_output.str());
                    return false;
                }
                std::string suffix = name.substr(3);
                if (suffix == "") {
                    lti_mod = lti_replace;
                } else if (suffix == "_add") {
                    lti_mod = lti_add;
                } else if (suffix == "_subtract") {
                    lti_mod = lti_subtract;
                } else if (suffix == "_merge") {
                    lti_mod = lti_merge;
                } else {
                    _output << "{\"error\":\"unknown LTI modifier\"}" << std::endl;
                    send(_output.str());
                    return false;
                }
                lti_x = value.get_int();
            } else if (name.substr(0,2) == "tv") {
                // parse tv
                if (tv_mod != 0) {
                    _output << "{\"error\":\"multiple TV changes\"}" << std::endl;
                    send(_output.str());
                    return false;
                }
                std::string suffix = name.substr(2);
                if (suffix == "") {
                    tv_mod = tv_replace;
                } else if (suffix == "_merge") {
                    tv_mod = tv_merge;
                } else {
                    _output << "{\"error\":\"unknown TV modifier\"}" << std::endl;
                    send(_output.str());
                    return false;
                }
                tv = JSONToTV(value, _output);
                // Null TV returned on error and usually they'll be
                // an error message already sent to output
                if (tv == NULL) {
                    if (_output.str().size() == 0) {
                        _output << "{\"error\":\"parsing truthvalue\"}" << std::endl;
                    }
                    send(_output.str());
                    return false;
                }
            } else {
                _output << "{\"error\":\"invalid property\"}" << std::endl;
                send(_output.str());
                return false;
            }
        }
    } catch (std::runtime_error e) {
        // json spirit probably borked at parsing bad javascript
        if (_output.str().size() == 0) {
            //_output << "{\"error\":\"parsing json\"}" << std::endl;
            _output << "{\"error\":"<<e.what()<<"\"}" << std::endl;
            send(_output.str());
        }
        return false;
    }

    doSTIChanges(as, h, sti_x);
    doLTIChanges(as, h, lti_x);
    doTVChanges(as, h, tv);

    _output << "{\"result\":\"success\"}" << std::endl;
    send(_output.str());
    return true;
}

bool UpdateAtomRequest::doSTIChanges(AtomSpace* as, Handle h, AttentionValue::sti_t sti_x ) {
    if (sti_mod == sti_replace) {
        as->setSTI(h,sti_x);
    } else if (sti_mod == sti_add) {
        as->setSTI(h,as->getSTI(h) + sti_x);
    } else if (sti_mod == sti_subtract) {
        as->setSTI(h,as->getSTI(h) - sti_x);
    } else if (sti_mod == sti_merge) {
        AttentionValue::sti_t oldSTI = as->getSTI(h);
        float average = (oldSTI + sti_x) / 2;
        as->setSTI(h,(AttentionValue::sti_t)average);
    }
    return true;
}

bool UpdateAtomRequest::doLTIChanges(AtomSpace* as, Handle h, AttentionValue::lti_t lti_x ) {
    if (lti_mod == lti_replace) {
        as->setLTI(h,lti_x);
    } else if (lti_mod == lti_add) {
        as->setLTI(h,as->getLTI(h) + lti_x);
    } else if (lti_mod == lti_subtract) {
        as->setLTI(h,as->getLTI(h) - lti_x);
    } else if (lti_mod == lti_merge) {
        AttentionValue::lti_t oldLTI = as->getLTI(h);
        float average = (oldLTI + lti_x) / 2;
        as->setLTI(h,(AttentionValue::lti_t)average);
    }
    return true;
}

bool UpdateAtomRequest::doTVChanges(AtomSpace* as, Handle h, TruthValuePtr tv) {
    if (tv_mod == tv_replace) {
        as->setTV(h, tv);
    } else if (tv_mod == tv_merge) {
        TruthValuePtr oldTV = as->getTV(h);
        TruthValuePtr newTV = oldTV->merge(tv);
        as->setTV(h, newTV);
    }
    return true;

}


void UpdateAtomRequest::decode(std::string& str) 
{
 string str2("%");   
 string str3("%7B");
 string str4("+"); 
 string str5("%22"); 
 string str6("%3A"); 
 string str7("%2C");
 string str8("%5B");
 string str9("%5D");
 string str10("%7D");
 
while (str.find(str3)!=string::npos )
{
  str.replace(str.find(str3),str3.length(),"{");    
}


while (str.find(str4)!=string::npos )
{
  str.replace(str.find(str4),str4.length()," ");    
}

while (str.find(str5)!=string::npos )
{
  str.replace(str.find(str5),str5.length(),"\"");    
}

while (str.find(str6)!=string::npos )
{
  str.replace(str.find(str6),str6.length(),":");    
}
 
while (str.find(str7)!=string::npos )
{
  str.replace(str.find(str7),str7.length(),",");    
}

while (str.find(str8)!=string::npos )
{
  str.replace(str.find(str8),str8.length(),"[");    
}


while (str.find(str9)!=string::npos )
{
  str.replace(str.find(str9),str9.length(),"]");    
}


while (str.find(str10)!=string::npos )
{
  str.replace(str.find(str10),str10.length(),"}");    
}

}
