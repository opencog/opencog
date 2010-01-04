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

#include "RESTModule.h"

using namespace opencog;

ServerRequestWrapper::ServerRequestWrapper()
{
}

ServerRequestWrapper::~ServerRequestWrapper()
{
}

void ServerRequestWrapper::handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::ostringstream oss;
    CogServer& cogserver = static_cast<CogServer&>(server());
    boost::regex reg("request/([^/]*)");
    boost::cmatch m;

    if (boost::regex_search(ri->uri,m,reg)) {
        std::string requestName(m[1].first, m[1].second);
        std::list<const char*> commands = cogserver.requestIds();
        Request* request = cogserver.createRequest(requestName.c_str());
        if (request == NULL) {
            mg_printf(conn, "unknown request %s\n", requestName.c_str());
            return;
        }
        request->cleanUp = false;
        // Ensure it isn't a shell request or anything else that will break
        const RequestClassInfo& cci = cogserver.requestInfo(requestName);
        if (cci.is_shell || cci.hidden) {
            mg_printf(conn, "Request not callable via REST: %s\n", requestName.c_str());
            return;
        }

        // Deal with parameters if they exist
        char* var_data;
        std::list<std::string> params;
        var_data = mg_get_var(conn, "params");
        if (var_data) {
            boost::split(params, var_data, boost::is_any_of(" "));
            mg_free(var_data);
        }
        request->setParameters(params);

        cogserver.pushRequest(request);

        for (;;) if (request->complete) break;

        //! @todo replace with configured server
        std::stringstream result;
        std::string serverAdd("http://localhost:17034");
        serverAdd += PATH_PREFIX;

        result << RESTModule::html_header;
        result << "Result of running request '" << requestName << "':<br/>";
        result << "<pre>";

        // Escape angle brackets
        std::string noanglebrackets = request->_output.str();
        boost::replace_all(noanglebrackets, "<", "&lt;");
        boost::replace_all(noanglebrackets, ">", "&gt;");
        result << noanglebrackets;

        result << "</pre>";
        result << RESTModule::html_footer;
        mg_printf(conn, result.str().c_str());

    } else {
        mg_printf(conn, "URL malformed? %s\n", ri->uri);
    }

}

