/*
 * opencog/rest/RESTModule.h
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

#ifndef _OPENCOG_REST_MODULE_H
#define _OPENCOG_REST_MODULE_H

#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

#include "mongoose.h"
#include <opencog/rest/GetAtomRequest.h>
#include <opencog/rest/GetListRequest.h>

#define PATH_PREFIX "/rest/0.1"

namespace opencog
{

class RESTModule : public Module
{

private:

    unsigned short _port;

    // Mongoose HTTP server context
    struct mg_context *ctx;

    // Timeout - how long we should wait for CogServer to fulfill requests
    // before giving up.
    int timeout;

    std::string serverAddress;

    // Register AtomSpace API requests. We can't directly access the AtomSpace
    // due to the REST Http server running in it's own set of threads.
    Factory<GetAtomRequest, Request> getAtomFactory;
    Factory<GetListRequest, Request> getListFactory;

public:

    static const unsigned int DEFAULT_PORT = 17034;
    static const char* DEFAULT_SERVER_ADDRESS;

    //! @todo create a function to generate header
    static const char* html_header;
    static const char* html_refresh_header;
    static const char* html_footer;

    static inline const char* id() {
        static const char* _id = "opencog::RESTModule";
        return _id;
    }

    RESTModule();
    virtual ~RESTModule();
    virtual void init  (void);
    
    void setupURIs();
    static void return400(mg_connection* conn, const std::string& message);
    static void return404(mg_connection* conn);
    static void return500(mg_connection* conn, const std::string& message);


}; // class

}  // namespace

#endif // _OPENCOG_REST_MODULE_H
