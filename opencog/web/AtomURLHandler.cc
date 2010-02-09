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

AtomURLHandler::AtomURLHandler()
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
    // Prevent CogServer from deleting this request
    // after it executes so we can find details about it.
    request->cleanUp = false;
    request->setParameters(params);
    cogserver.pushRequest(request);
    
    for (;;) {
        //! @todo - implement time out (requires lock on the request cleanUp variable
        if (request->complete) break;
    }
    GetAtomRequest *gar = dynamic_cast<GetAtomRequest *>(request);

    //! @todo replace with configured server
    std::stringstream result;
    std::string serverAdd("http://localhost:17034");
    serverAdd += UI_PATH_PREFIX;
    
    // Check for refresh option
    //! @todo make refresh time specifiable on the page.
    bool refresh = false;
    std::list<std::string>::const_iterator it;
    for (it = params.begin(); it != params.end(); ++it) {
        if (*it == "refresh=1" or *it == "refresh=true") {
            refresh = true;
            break;
        }
    }
    result << WebModule::open_html_header;
    if (refresh)
        result << WebModule::html_refresh_header;
    result << gar->getHTMLHeader();
    result << WebModule::close_html_header;

    result << gar->getHTML(serverAdd).c_str();
//mg_printf(conn, result.str().c_str());
    char buffer[512];
    for (uint i = 0; i < result.str().size(); i+=511) {
        memset(buffer,'\0',512);
        result.str().copy(buffer,511,i);
        if (i+511 > result.str().size())
            buffer[(result.str().size() % 511) + 1] = '\0';
        mg_printf(conn, buffer);
    }
        
    result.str("");
    result << "\r\n\r\n<small>You requested the url: %s<br> With query string:"
        "%s</small>";
    if (refresh) {
        result << "<br/><small>Page will refresh every 5 seconds</small>";
    }
    result << WebModule::html_footer;
    mg_printf(conn, result.str().c_str(), ri->uri, ri->query_string);

    // Clean up
    delete request;
    
}

