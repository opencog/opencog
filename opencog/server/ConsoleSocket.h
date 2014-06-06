/*
 * opencog/server/ConsoleSocket.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_CONSOLE_SOCKET_H
#define _OPENCOG_CONSOLE_SOCKET_H

#include <string>
#include <tr1/memory>

#include <opencog/server/RequestResult.h>
#include <opencog/shell/GenericShell.h>
#include <opencog/server/ServerSocket.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class Request;


/**
 * This class implements the ServerSocket that handles the primary
 * interface of the cogserver: the plain text command line. 
 *
 * There may be multiple instances of ConsoleSocket to support multiple 
 * simultaneous clients. This is done by creating a separate thread and 
 * dispatching a client socket for each client that connects to the 
 * server socket.
 *
 * We provide a callback method from the IRPCSocket interface: 
 * 'OnRequestCompleted()'. This callback signals the server socket 
 * that request processing has finished (so that the we may 
 * synchronously send the command prompt to the client but process the 
 * request 'asynchronously' on the cogserver's main thread.
 *
 * As required by the Request abstract class, the ConsoleSocket class
 * also implements the IHasMimeType interface, defining its mime-type
 * as 'text/plain'.
 *
 */
class ConsoleSocket : public ServerSocket,
                      public RequestResult
{

private:

    Request* _request;
    std::string _buffer;
    GenericShell *_shell;

public:

    /** ConsoleSocket's constructor. Defines the socket's mime-type as
     *  'text/plain' and the enables the Socket's 'line protocol'
     */
    ConsoleSocket(boost::asio::io_service& _io_service);

    /** ConsoleSocket's destructor. */
    ~ConsoleSocket();

    /** Connection callback: called whenever a new connection arrives
     */
    void OnConnection(void);

    /** OnLine callback: called when a new command/request is revieved
     *  from the client. It parses the command line by splitting it into
     *  space-separated tokens.
     *
     *  The first token is used as the requests id/name and the
     *  remaining tokens are used as the request's parameters. The
     *  request name is used to retrieve the request class from the
     *  cogserver.
     *
     *  If the class is not found, we execute the 'HelpRequest' which
     *  will return a useful message to the client.
     *
     *  If the request class is found, we instantiate a new request,
     *  set its parameters and push it to the cogserver's request queue
     *  (*unless* the request instance has disabled the line protocol;
     *  see the OnRawData() method documentation).
     */
    void OnLine            (const std::string& line);

    /** Some requests may require input that spans multiple lines. To
     *  handle these cases, the request should disable the socket's
     *  line protocol when the method 'Request.setSocket' is called.
     *
     *  If the line protocol is disabled, this callback will be called
     *  whenever the client sends data to server. The current
     *  implementation pushes the input to a buffer until a special
     *  character sequence is sent: "^D\r\n" (or "^D\n"). Then, the
     *  server adds the input buffer to the request's parameter list
     *  and pushes the request to the cogserver's request queue.
     *  Parsing the contents of input buffer is naturally, up to
     *  the request itself.
     */
    void OnRawData         (const char * buf, size_t len);

    /** OnRequestComplete: called when a request has finished. It
     *  just sends another command prompt (configuration parameter
     *  "PROMPT") to the client.
     */
    void OnRequestComplete ();

    /** Called when this is assigned to a DataRequest
     */ 
    void SetDataRequest();

    /** Sends a request result to the client
      */
    void SendResult(const std::string& res);

    void sendPrompt();

    /** Called when a Request exits the connection
     */ 
    void Exit();

    /** SetShell: Declare an alternate shell, that will perform all
     *  command line processing.
     */
    void SetShell(GenericShell *);

    /** Gets the tcp socket 
     */
    tcp::socket& getSocket(void);

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_CONSOLE_SOCKET_H
