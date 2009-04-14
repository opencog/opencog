/*
 * opencog/embodiment/PetaverseProxySimulator/MessageSenderAgent.h
 *
 * Copyright (C) 2007-2008 Welter Luigi
 * All Rights Reserved
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

#ifndef MESSAGE_SENDERAGENT_H
#define MESSAGE_SENDERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.h>

namespace PetaverseProxySimulator
{

using namespace opencog;

class MessageSenderAgent : public Agent
{

public:

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("PetaverseProxySimulator::MessageSenderAgent");
        return _ci;
    }

    // ***********************************************/
    // Constructors/destructors

    ~MessageSenderAgent();
    MessageSenderAgent();

    void run(CogServer *ne);

}; // class
}  // namespace

#endif
