/*
 * opencog/server/RequestResult.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi <welter@vettalabs.com>
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

#ifndef _OPENCOG_REQUEST_RESULT_H
#define _OPENCOG_REQUEST_RESULT_H

#include <string>
#include <tr1/memory>

#include <opencog/server/IHasMimeType.h>
#include <opencog/server/IRequestComplete.h>

namespace opencog
{

class Request;


/**
 * This abstract class defines the API needed to handle a request result.
 *
 * We provide a callback method from the IRequestComplete interface: 
 * 'OnRequestCompleted()'. This callback signals the RequestResult object 
 * that request processing has finished (so that we may 
 * synchronously send the request result to the client but process the 
 * request 'asynchronously' on the cogserver's main thread).
 *
 * As required by the Request abstract class, the RequestResult class
 * also defines the IHasMimeType interface, defining its mime-type
 * as 'text/plain'.
 *
 */
class RequestResult : public IHasMimeType,
                      public IRequestComplete
{

public:

    RequestResult(const std::string& mimeType) : IHasMimeType(mimeType) {}
    virtual ~RequestResult() {}

    /** SetDataRequest: called when this is assigned to a DataRequest */
    virtual void SetDataRequest() {}

    /** Exit: called when a Request exits the connection */
    virtual void Exit() {}

    /** Sends result to the client */
    virtual void SendResult(const std::string& res) = 0;

    /** OnRequestComplete: called when a request has finished. */
    virtual void OnRequestComplete() = 0;

}; // class

}  // namespace

#endif // _OPENCOG_REQUEST_RESULT_H
