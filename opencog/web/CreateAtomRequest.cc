/*
 * opencog/rest/CreateAtomRequest.cc
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

#include "CreateAtomRequest.h"
#include "BaseURLHandler.h"
#include "JsonUtil.h"
#include <string>

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

#include <opencog/web/json_spirit/json_spirit.h>

using namespace opencog;
using namespace json_spirit;
using namespace std;

CreateAtomRequest::CreateAtomRequest(CogServer& cs) : Request(cs)
{
}

CreateAtomRequest::~CreateAtomRequest()
{
    logger().debug("[CreateAtomRequest] destructor");
}

void CreateAtomRequest::setRequestResult(RequestResult* rr)
{
    Request::setRequestResult(rr);
    rr->SetDataRequest();
}

bool CreateAtomRequest::execute()
{
    logger().debug("json-create-atom execute called");
    std::string& data = _parameters.front();
    decode(data);
    AtomSpace& as = _cogserver.getAtomSpace();
    Type t = NOTYPE;
    std::string atomName;
    HandleSeq outgoing;
    TruthValue* tv = NULL;
    Value json_top;

    try {
	logger().debug("start parsing");
        read( data, json_top);
	logger().debug("data is read");
        const Object &json_obj = json_top.get_obj();
	logger().debug("json_obj gotten; parsing");
	logger().debug("json_obj size: %d", json_obj.size());
        for( Object::size_type i = 0; i != json_obj.size(); ++i ) {
            std::string s;
            const Pair& pair = json_obj[i];
            const std::string& name = pair.name_;
            const Value&  value = pair.value_;
            std::cout << name << std::endl;
            if (name == "type") {
		logger().debug("json type: ");//, value.get_str());
                t = classserver().getType(value.get_str());
            } else if (name == "name") {
		logger().debug("json name: ");//%s", value.get_str());
                atomName = value.get_str();
            } else if (name == "outgoing") {
                const Array &outArray = value.get_array();
                for( unsigned int j = 0; j < outArray.size(); ++j ) {
                    outgoing.push_back(Handle(outArray[j].get_uint64()));
                }
            } else if (name == "truthvalue") {
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
            }

        }
    } catch (std::runtime_error e) {
        // json spirit probably borked at parsing bad javascript
        if (tv) delete tv;
        if (_output.str().size() == 0) {
            //_output << "{\"error\":\"parsing json\"}" << std::endl;
	    logger().debug("json_spirit error: %s", e.what());
            _output << "{\"error\":\"json_spirit error "<<e.what()<<"\"}" << std::endl;
            send(_output.str());
        }
        return false;
    }
    if (t == NOTYPE) {
        _output << "{\"error\":\"no type\"}" << std::endl;
        send(_output.str());
        if (tv) delete tv; // remember to clean up TV if it's around
        return false;
    }
    if (tv == NULL) {
        _output << "{\"error\":\"no truthvalue\"}" << std::endl;
        send(_output.str());
        return false;
    }
    Handle h;
    bool exists = false;
    if (classserver().isLink(t) ) {
        if (atomName != "") {
            _output << "{\"error\":\"links can't have name\"}" << std::endl;
            send(_output.str());
            delete tv; // remember to clean up TV if it's around
            return false;
        }
        h = as.getHandle(t, outgoing);
        if (as.isValidHandle(h)) exists = true;
        h = as.addLink(t, outgoing, *tv);
    } else {
        h = as.getHandle(t, atomName);
        if (as.isValidHandle(h)) exists = true;
        h = as.addNode(t,atomName, *tv);
    }
    delete tv;

    if (!as.isValidHandle(h)) {
        _output << "{\"error\":\"invalid handle returned\"}" << std::endl;
        send(_output.str());
        return false;
    } else {
        json_makeOutput(h,exists);
        send(_output.str());
    }
    return true;
}
void CreateAtomRequest::decode(std::string& str) 
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


void CreateAtomRequest::json_makeOutput(Handle h, bool exists)
{
    if (exists)
        _output << "{\"result\":\"merged\",";
    else
        _output << "{\"result\":\"created\",";
    _output << "\"handle\":" << h.value() << "}" << std::endl;

}


