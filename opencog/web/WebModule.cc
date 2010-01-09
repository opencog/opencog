/*
 * opencog/rest/WebModule.cc
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

#include "WebModule.h"

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>
#include <opencog/util/Config.h>

#include <sstream>
#include <boost/bind.hpp>
#include <boost/mem_fn.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

using namespace opencog;

WebModule *rest_mod;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()   { return WebModule::id(); }
extern "C" Module*     opencog_module_load() {
    rest_mod = new WebModule();
    return rest_mod;
}
extern "C" void        opencog_module_unload(Module* module) { delete module; }

const char* WebModule::DEFAULT_SERVER_ADDRESS = "http://localhost";

//! @todo create a function to generate header
const char* WebModule::open_html_header = "HTTP/1.1 200 OK\r\n"
    "content-Type: text/html\r\n\r\n"
    "<html><head>";
const char* WebModule::close_html_header = "</head><body>";

const char* WebModule::html_refresh_header = 
    "<META HTTP-EQUIV=\"Refresh\" CONTENT=\"5\">";

const char* WebModule::html_footer = "</body></html>\r\n";

WebModule::WebModule() : _port(DEFAULT_PORT), serverAddress(DEFAULT_SERVER_ADDRESS)
{
    logger().debug("[WebModule] constructor");

    if (config().has("Web_PORT"))
        _port = config().get_int("Web_PORT");
    if (config().has("Web_SERVER"))
        serverAddress = config().get("Web_SERVER");

    // Register all requests with CogServer
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerRequest(GetAtomRequest::info().id, &getAtomFactory); 
    cogserver.registerRequest(GetListRequest::info().id, &getListFactory); 

    timeout = 100;

}

WebModule::~WebModule()
{
    rest_mod = NULL;
    logger().debug("[WebModule] destructor");
    //CogServer& cogserver = static_cast<CogServer&>(server());
    mg_stop(ctx);
}

static const char* DEFAULT_WEB_PATH[] =
{
    DATADIR,
    "../opencog/web", // For running from bin dir that's in root of src
#ifndef WIN32
    "/usr/share/opencog/www",
    "/usr/local/share/opencog/www",
#endif // !WIN32
    NULL
};

void WebModule::init()
{
    logger().debug("[WebModule] init");
    // Set the port that the embedded mongoose webserver will listen on.
    std::stringstream port_str;
    port_str << _port;
    ctx = mg_start();
    mg_set_option(ctx, "ports", port_str.str().c_str());
    // Turn on admin page
    //mg_set_option(ctx, "admin_uri", "/admin/");
    // Find and then set path for web resource files
    int i = 0;
    for (; DEFAULT_WEB_PATH[i] != NULL; ++i) {
        boost::filesystem::path webPath(DEFAULT_WEB_PATH[i]);
        webPath /= "processing.js";
        if (boost::filesystem::exists(webPath)) {
            break;
        }
    }
    mg_set_option(ctx, "root", DEFAULT_WEB_PATH[i]);
    // Turn off directory listing
    mg_set_option(ctx, "dir_list", "no");
    // Set up the urls
    setupURIs();
}

void viewAtomPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
void viewListPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
void makeRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);

void WebModule::setupURIs()
{
    setupURIsForREST();
    setupURIsForUI();
}

void WebModule::setupURIsForUI()
{
    // Support both "atom/UUID" and "atom?handle=UUID"
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/atom/*", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/atom", viewAtomPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/list", viewListPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/list/*", viewListPage, NULL);
    mg_set_uri_callback(ctx, UI_PATH_PREFIX "/request/*", makeRequest, NULL);
}

void WebModule::setupURIsForREST()
{
    static char rest_str[] = "rest";
    // atom/type/* support GET atoms of type.
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/atom/type/*", viewListPage,
            rest_str);
    // atom/ support GET/PUT/POST == get info/create/create
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/atom/", viewListPage,
            rest_str);
    // atom/* support GET, get atom info
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/atom/*", viewAtomPage,
            rest_str);
    // server/request/request_name, POST
    mg_set_uri_callback(ctx, REST_PATH_PREFIX "/server/request/*", makeRequest,
            rest_str);
}

void WebModule::return400(mg_connection* conn, const std::string& message)
{
    mg_printf(conn, "HTTP/1.1 400 %s\r\n", message.c_str());
}

void WebModule::return404(mg_connection* conn)
{
    mg_printf(conn, "HTTP/1.1 404 Not found.\r\n");
}

void WebModule::return500(mg_connection* conn, const std::string& message)
{
    mg_printf(conn, "HTTP/1.1 500 %s\r\n", message.c_str());
}

std::list<std::string> splitQueryString(char* query) {
    using namespace std;
    list<string> params;
    if (query == NULL) return params;
    string query_string(query);
    boost::split(params, query_string, boost::is_any_of("&;"));
    return params;
}

std::map<std::string,std::string> paramsToMap(const std::list<std::string>& params) 
{
    using namespace std;
    map<string,string> query_map;
    list<string>::const_iterator p_i;
    for (p_i = params.begin(); p_i != params.end(); p_i++) {
        std::vector<string> keyAndValue;
        boost::split(keyAndValue, *p_i, boost::is_any_of("="));
        if (keyAndValue.size() != 2) continue;
        query_map.insert(pair<string,string>(keyAndValue[0], keyAndValue[1]));
    }
    return query_map;
}

void viewAtomPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = splitQueryString(ri->query_string);
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

void viewListPage( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{
    std::list<std::string> params = splitQueryString(ri->query_string);
    CogServer& cogserver = static_cast<CogServer&>(server());
    Request* request = cogserver.createRequest("get-list");
    if (request == NULL) {
        WebModule::return500( conn, std::string("unknown request"));
        return;
    }
    // Get list parameters from URL if they exist
    boost::regex reg("list/([^/]*)");
    boost::cmatch m;
    if (boost::regex_search(ri->uri,m,reg)) {
        std::string typeName(m[1].first, m[1].second);
        typeName = "type=" + typeName;
        params.push_back(typeName);
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
    GetListRequest *glr = dynamic_cast<GetListRequest *>(request);

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
    result << WebModule::close_html_header;

    result << glr->getHTML(serverAdd).c_str();
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
    result << "\r\n\r\n<small>You requested the url: %s<br/> With query string:"
        "%s</small>";
    if (refresh) {
        result << "<br/><small>Page will refresh every 5 seconds</small>";
    }
    result << WebModule::html_footer;
    mg_printf(conn, result.str().c_str(), ri->uri, ri->query_string);

    // Clean up
    delete request;
    
}

void makeRequest ( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data)
{ rest_mod->requestWrapper.handleRequest(conn, ri, data); }


