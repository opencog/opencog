/*
 * opencog/cogserver/server/ConsoleSocket.h
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

#include <condition_variable>
#include <mutex>
#include <string>

#include <opencog/cogserver/shell/GenericShell.h>
#include <opencog/cogserver/server/ServerSocket.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class implements the ServerSocket that handles the primary
 * interface of the cogserver: the plain text command line.
 *
 * There may be multiple instances of ConsoleSocket to support multiple
 * simultaneous clients. This is done by creating a separate thread and
 * dispatching a client socket for each client that connects to the
 * server socket.
 *
 * We provide a callback method: 'OnRequestCompleted()'. This callback
 * tells the server socket that request processing has finished (so that
 * the command prompt can be sent to the client immediately, while the
 * request itself is processed 'asynchronously'.
 */
class ConsoleSocket : public ServerSocket
{
private:
    GenericShell *_shell;

    static std::string _prompt;

    // We need the use-count and the condition variables to avoid races
    // between asynchronous socket closures and unsent replies. So, for
    // example, the user may send a command, but then close the socket
    // before the reply is sent.  The boost::asio code notices the
    // closed socket, and ServerSocket::handle_connection() exits it's
    // loop, and then tries to destruct this class and exit the thread
    // that has been handling the socket i/o. We have to hold off this
    // destruction, until all of the users of this class have completed
    // thier work. We accomplish this with a use-count: each in-flight
    // request increments the use-count, and then decrements it when done.
    // The destructor can run only when the use-count has dropped to zero.
    //
    // Sockets with a shell on them will typically have a use-count of
    // zero already; these use a different method of holding off the dtor.
    // Basically, the shell dtor has to run first, before the
    // ServerSocket::handle_connection() invokes this dtor.
    volatile unsigned int _use_count;
    std::mutex _in_use_mtx;
    std::condition_variable _in_use_cv;

    // A count of the number of concurrent open sockets. This is used
    // to limit the number of connections to the cogserver, so that it
    // doesn't crash with a `accept: Too many open files` error.
    static unsigned int _max_open_sockets;
    static volatile unsigned int _num_open_sockets;
    static std::mutex _max_mtx;
    static std::condition_variable _max_cv;

protected:

    /**
     * Connection callback: called whenever a new connection arrives
     */
    void OnConnection(void);

    /**
     * OnLine callback: called when a new command/request is received
     * from the client. It parses the command line by splitting it into
     * space-separated tokens.
     *
     * The first token is used as the request's id/name and the
     * remaining tokens are used as the request's parameters. The
     * request name is used to identify the request type.
     *
     * If the request type is not found, we execute the 'HelpRequest',
     * which will return a useful message to the client.
     *
     * If the request class is found, we instantiate a new request,
     * set its parameters and push it to the cogserver's request queue.
     */
    void OnLine(const std::string&);

public:
    /**
     * Ctor. Defines the socket's mime-type as 'text/plain' and then
     * configures the Socket to use line protocol.
     */
    ConsoleSocket(void);
    ~ConsoleSocket();

    void get() { std::unique_lock<std::mutex> lck(_in_use_mtx); _use_count++; }
    void put() { std::unique_lock<std::mutex> lck(_in_use_mtx); _use_count--; _in_use_cv.notify_all(); }

    /**
     * OnRequestComplete: called when a request has finished. It
     * just sends another command prompt (configuration parameter
     * "PROMPT") to the client.
     */
    void OnRequestComplete();

    /**
     * Sends a request result to the client,
     */
    void SendResult(const std::string&);

    void sendPrompt();

    /**
     * Called when a Request exits the connection
     */
    void Exit();

    /**
     * SetShell: Declare an alternate shell, that will perform all
     * command line processing.
     */
    void SetShell(GenericShell *);

    /**
     * Assorted debugging utilities.
     */
    unsigned int get_use_count() const { return _use_count; }
    unsigned int get_max_open_sockets() const { return _max_open_sockets; }
    unsigned int get_num_open_sockets() const { return _num_open_sockets; }
}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_CONSOLE_SOCKET_H
