/*
 * opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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

#ifndef _EMBODIMENT_COG_SERVER_H
#define _EMBODIMENT_COG_SERVER_H

#include <opencog/server/CogServer.h>

#include "Message.h"
#include "NetworkElement.h"

namespace opencog { namespace messaging {

/** Abstract CogServer class for embodiment servers which needs a NetworkElement
 */
class EmbodimentCogServer : public opencog::CogServer
{

private:
    NetworkElement* ne;

protected:
    bool externalTickMode;

    /** controls the interval (in number of cycles) in which the serverLoop will
     * check for messages
     */ 
    int unreadMessagesCheckInterval;

    /** sets the limit of messages per request to be retrieved from the
     * router (-1 for unlimited)
     */
    int unreadMessagesRetrievalLimit;

    /**
     * Maximum size of the message queue. Some old messages would be discarded,
     * if the message queue size exceeds the limit. 
     */
    int maxMessageQueueSize; 

public:

    EmbodimentCogServer();
    virtual ~EmbodimentCogServer();

    //! override CogServer methods
    virtual bool customLoopRun(void);

    void setNetworkElement(NetworkElement*);
    NetworkElement& getNetworkElement(void);

    /**
     * This is called when this servers needs to process a Message received
     * from router. Subclasses must override this to perform something useful.
     *
     * @return true, if the CogServer must be stopped (e.g. it received a
     * SAVE_AND_EXIT message)
     */
    virtual bool processNextMessage(Message *message) = 0;

    //! convenience method - delegated to NetworkElement
    virtual bool sendMessage(Message &msg);
    //! convenience method - delegated to NetworkElement
    virtual bool sendCommandToRouter(const std::string &cmd);

    const std::string& getID(void);
    int getPortNumber(void);
    bool isElementAvailable(const std::string& id);
    void logoutFromRouter();

};

} } // namespace opencog::messaging

#endif // _EMBODIMENT_COG_SERVER_H
