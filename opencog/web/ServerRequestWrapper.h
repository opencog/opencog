/*
 * opencog/rest/ServerRequestWrapper.h
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

#ifndef _OPENCOG_REST_REQUEST_WRAPPER_H
#define _OPENCOG_REST_REQUEST_WRAPPER_H

#include <vector>
#include <string>

#include <opencog/server/Request.h>

#include "BaseURLHandler.h"
#include "mongoose.h"

namespace opencog
{

class ServerRequestWrapper : public BaseURLHandler
{
    std::string requestName;
    bool isJSON;

public:

    ServerRequestWrapper();
    ~ServerRequestWrapper();
    void handleRequest( struct mg_connection *conn,
        const struct mg_request_info *ri, void *data);

    virtual void OnRequestComplete();
};

} // namespace 

#endif // _OPENCOG_REST_REQUEST_WRAPPER_H
