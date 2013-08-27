/*
 * opencog/rest/ListURLHandler.cc
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

#include "ListURLHandler.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

#include "WebModule.h"

using namespace opencog;

ListURLHandler::ListURLHandler() : BaseURLHandler("text/plain"),
    refreshPage(false), isJSON(false)
{
}

ListURLHandler::~ListURLHandler()
{
}

void ListURLHandler::handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = BaseURLHandler::splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());
    Request* request = cogserver.createRequest("json-get-list");
    if (request == NULL) {
        WebModule::return500( conn, std::string("unknown request"));
        return;
    }
    _conn = conn;
    call_url = ri->uri;
    if (ri->query_string) query_string = ri->query_string;

    // If we are passed data, then this is a json request
    if (data) {
        isJSON = true;
        params.push_back(std::string("format=json"));
    }
    else {
        // Check for refresh option
        //! @todo make refresh time specifiable on the page.
        std::list<std::string>::const_iterator it;
        for (it = params.begin(); it != params.end(); ++it) {
            if (*it == "refresh=1" or *it == "refresh=true") {
                refreshPage = true;
                break;
            }
        }
    }
    // Get list parameters from URL if they exist
    // take /list/x/ as wanting type x
    boost::regex reg("list/([^/]*)");
    boost::cmatch m;
    bool hasHandle;
    if (boost::regex_search(ri->uri,m,reg)) {
        std::string typeName(m[1].first, m[1].second);
        typeName = "type=" + typeName;
        params.push_back(typeName);
        hasHandle = true;
    }
    bool hasType=false;
    if (!hasHandle){
		// take /list?*type=x* as wanting type x
		boost::regex typeSpecRegex("type=([^&]*)");
		if (boost::regex_search(ri->query_string,m,typeSpecRegex)) {
			hasType = true;
		}
    }
    if (!hasHandle && !hasType) {
        std::string p;
        p = "type=Atom";
        params.push_back(p);
        p = "subtype=true";
        params.push_back(p);
    }

    request->setRequestResult(this);
    request->setParameters(params);
    cogserver.pushRequest(request);
}

void ListURLHandler::OnRequestComplete()
{
    std::stringstream result;
    std::string serverAdd("http://localhost:17034");
    serverAdd += UI_PATH_PREFIX;
    std::cout << request_output;
    
    if (!isJSON) {
        result << WebModule::openHtmlHeader();
        if (refreshPage)
        result << WebModule::HtmlrefreshHeader();
        result << WebModule::closeHtmlHeader();
    } else {
        result << WebModule::jsonHeader();
    }

    result << replaceURL(serverAdd);
//mg_printf(conn, result.str().c_str());
    char buffer[512];
    for (uint i = 0; i < result.str().size(); i+=511) {
        memset(buffer,'\0',512);
        result.str().copy(buffer,511,i);
        if (i+511 > result.str().size())
            buffer[(result.str().size() % 511) + 1] = '\0';
        mg_printf(_conn, buffer);
    }

    if (!isJSON) {
        result.str("");
        result << "\r\n\r\n<small>You requested the url: %s<br/> With query string:"
            "%s</small>";
        if (refreshPage) {
            result << "<br/><small>Page will refresh every 5 seconds</small>";
        }
        result << WebModule::htmlFooter();
        mg_printf(_conn, result.str().c_str(), call_url.c_str(),
            query_string.c_str());
    }

    completed=true;
}


