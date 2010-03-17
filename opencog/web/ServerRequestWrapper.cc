/*
 * opencog/rest/ServerRequestWrapper.cc
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

#include "ServerRequestWrapper.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

#include "WebModule.h"

#include <opencog/web/json_spirit/json_spirit.h>

using namespace opencog;
using namespace json_spirit;

ServerRequestWrapper::ServerRequestWrapper() : BaseURLHandler("text/plain"),
    isJSON(false)
{
}

ServerRequestWrapper::~ServerRequestWrapper()
{
}

void ServerRequestWrapper::handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    boost::regex reg("request/([^/]*)");
    boost::cmatch m;
    _conn = conn;

    // If we are passed data, then this is a json request
    if (data) {
        isJSON = true;
    }
    if (boost::regex_search(ri->uri,m,reg)) {
        requestName = std::string(m[1].first, m[1].second);
        std::list<const char*> commands = cogserver.requestIds();
        Request* request = cogserver.createRequest(requestName.c_str());
        if (request == NULL) {
            if (isJSON)
                request_output << "{\"error\":\"unknown request\"}" << std::endl;
            else
                request_output << "unknown request " << requestName << std::endl;
            mg_printf(conn, request_output.str().c_str());
            completed = true;
            return;
        }
        // Ensure it isn't a shell request or anything else that will break
        const RequestClassInfo& cci = cogserver.requestInfo(requestName);
        if (cci.is_shell || cci.hidden) {
            if (isJSON)
                request_output << "{\"error\":\"request not callable via REST\"}" << std::endl;
            else
                request_output << "Request not callable via REST: " << requestName << std::endl;
            mg_printf(conn, request_output.str().c_str());
            completed = true;
            return;
        }

        // Deal with parameters if they exist
        char* var_data;
        std::list<std::string> params;
        var_data = mg_get_var(conn, "params");
        if (var_data) {
            boost::split(params, var_data, boost::is_any_of(" "));
            mg_free(var_data);
        } else if (isJSON) {
            std::string json_str;
            if (ri->post_data_len > 0) {
                json_str = std::string(ri->post_data, ri->post_data_len);
                Value json_top;
                try {
                    read( json_str, json_top);
                    const Object &json_obj = json_top.get_obj();
                    if (json_obj.size() != 1) {
                        request_output << "{\"error\":\"incorrect size for json\"}" << std::endl;
                        mg_printf(conn, request_output.str().c_str());
                        completed = true;
                        return;
                    }
                    const Pair& pair = json_obj[0];
                    const std::string& name = pair.name_;
                    const Value&  value = pair.value_;
                    if (name == "params") {
                        params.push_back(value.get_str());
                    } else {
                        request_output << "{\"error\":\"expected 'params' key\"}" << std::endl;
                        mg_printf(conn, request_output.str().c_str());
                        completed = true;
                        return;
                    }
                } catch (std::runtime_error e) {
                    // json spirit probably borked at parsing bad javascript
                    request_output << "{\"error\":\"parsing json\"}" << std::endl;
                    mg_printf(conn, request_output.str().c_str());
                    completed = true;
                    return;
                }
            }
        }
        request->setRequestResult(this);
        request->setParameters(params);
        cogserver.pushRequest(request);
    } else {
        if (isJSON)
            request_output << "{\"error\":\"url malformed\"}" << std::endl;
        else
            request_output << "URL malformed?" << requestName << std::endl;
        mg_printf(conn, request_output.str().c_str());
        completed = true;
        return;
    }
}

void ServerRequestWrapper::OnRequestComplete() {

    //! @todo replace with configured server
    std::stringstream result;

    if (isJSON) {
        std::string nodoublequotes = request_output.str();
        boost::replace_all(nodoublequotes, "\"", "\'");
        result.str("");
        result << "{\"result\":\"";
        result << nodoublequotes;
        result << "\"}" << std::endl;
    } else {
        result << WebModule::openHtmlHeader();
        result << WebModule::closeHtmlHeader();
        result << "Result of running request '" << requestName << "':<br/>";
        result << "<pre>";

        // Escape angle brackets
        //! @todo move removal of angle brackets to BaseURLHandler
        std::string noanglebrackets = request_output.str();
        boost::replace_all(noanglebrackets, "<", "&lt;");
        boost::replace_all(noanglebrackets, ">", "&gt;");
        result.str("");
        result << noanglebrackets;

        result << "</pre>";
        result << WebModule::htmlFooter();
    }
    mg_printf(_conn, result.str().c_str());

    completed = true;

}

