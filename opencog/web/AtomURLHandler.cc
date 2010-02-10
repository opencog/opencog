/*
 * opencog/rest/AtomURLHandler.cc
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

#include "AtomURLHandler.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

#include "WebModule.h"

#include "GetAtomRequest.h"

using namespace opencog;

AtomURLHandler::AtomURLHandler() : BaseURLHandler("text/plain"),
    refreshPage(false)
{
}

AtomURLHandler::~AtomURLHandler()
{
}

void AtomURLHandler::handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = BaseURLHandler::splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());
    _conn = conn;
    call_url = ri->uri;
    if (ri->query_string) query_string = ri->query_string;

    // Get handle UUID from URL if it exists
    boost::regex reg("atom/([^/]*)");
    boost::cmatch m;
    if (boost::regex_search(ri->uri,m,reg)) {
        std::string handleUUID(m[1].first, m[1].second);
        handleUUID = "handle=" + handleUUID;
        params.push_back(handleUUID);
    }

    Request* request = cogserver.createRequest("get-atom");
    if (request == NULL) {
        WebModule::return500( conn, std::string("unknown request"));
        return;
    }
    // Check for refresh option
    //! @todo make refresh time specifiable on the page.
    std::list<std::string>::const_iterator it;
    for (it = params.begin(); it != params.end(); ++it) {
        if (*it == "refresh=1" or *it == "refresh=true") {
            refreshPage = true;
            break;
        }
    }
    request->setRequestResult(this);
    request->setParameters(params);
    cogserver.pushRequest(request);
}

std::string AtomURLHandler::getHTMLHeader()
{
    std::ostringstream oss;
    oss << "<script language=\"javascript\" src=\"../processing.js\">"
        "</script>" << std::endl;
    oss << "<script language=\"javascript\" src=\"../init.js\">"
        "</script>" << std::endl;
    return oss.str();
}

void AtomURLHandler::OnRequestComplete()
{
    //! @todo replace with configured server
    std::stringstream result;
    std::string serverAdd("http://localhost:17034");
    serverAdd += UI_PATH_PREFIX;
    
    result << WebModule::open_html_header;
    if (refreshPage)
        result << WebModule::html_refresh_header;
    result << getHTMLHeader();
    result << WebModule::close_html_header;

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
        
    result.str("");
    result << "\r\n\r\n<small>You requested the url: %s<br> With query string:"
        "%s</small>";
    if (refreshPage) {
        result << "<br/><small>Page will refresh every 5 seconds</small>";
    }
    result << WebModule::html_footer;
    mg_printf(_conn, result.str().c_str(), call_url.c_str(),
            query_string.c_str());
    completed = true;

}

