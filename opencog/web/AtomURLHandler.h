/*
 * opencog/rest/AtomURLHandler.h
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

#ifndef _OPENCOG_ATOM_URL_HANDLER_H
#define _OPENCOG_ATOM_URL_HANDLER_H

#include <vector>
#include <string>

#include <opencog/atomspace/Handle.h>
#include <opencog/server/Request.h>

#include "BaseURLHandler.h"
#include "GetAtomRequest.h"

namespace opencog
{

class AtomURLHandler : public BaseURLHandler
{

    bool isJSON;
    bool refreshPage;
    std::string call_url;
    std::string query_string;
    std::string method;
    bool handleInQuery;
    bool handleInURL;
    Handle h;

public:

    AtomURLHandler();
    ~AtomURLHandler();
    void handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
    void handleGET( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
    void handlePOSTUpdate( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
    void handlePOSTCreate( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
    void handlePUT( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);
    void handleDELETE( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);

    virtual void OnRequestComplete();
    std::string getHTMLHeader();
};

} // namespace 

#endif // _OPENCOG_ATOM_URL_HANDLER_H
