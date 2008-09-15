/*
 * opencog/server/SimpleNetworkServer.h
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

#ifndef _OPENCOG_SIMPLE_NETWORK_SERVER_H
#define _OPENCOG_SIMPLE_NETWORK_SERVER_H

#include <string>
#include <queue>

#include <pthread.h>

#include <opencog/server/CallBackInterface.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/NetworkServer.h>

namespace opencog
{

class SimpleNetworkServer : public NetworkServer
{
private:

    std::string prompt;
    bool started;
    static bool stopListenerThreadFlag;
    int portNumber;
    CogServer *cogServer;
    bool shell_mode;

    pthread_t socketListenerThread; // thread which will listen to the port
    pthread_attr_t socketListenerAttr;

    static void *portListener(void *arg);
    static void parseCommandLine(const std::string &cmdLine,
                                 std::string &command,
                                 std::queue<std::string> &args);
public:

    ~SimpleNetworkServer();
    SimpleNetworkServer(CogServer *cogServer, int portNumber);

    void processCommandLine(CallBackInterface *callBack, const std::string &line);
    std::string getCommandPrompt();

    void start();

}; // class
}  // namespace

#endif // _OPENCOG_SIMPLE_NETWORK_SERVER_H
