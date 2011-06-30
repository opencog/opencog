/*
 * opencog/embodiment/Control/Procedure/ComboShellServer.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef COMBO_SHELL_SERVER_H
#define COMBO_SHELL_SERVER_H

#include <string>
#include <opencog/embodiment/Control/MessagingSystem/Message.h>
#include <opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.h>

namespace opencog { namespace messaging {

class ComboShellServer : public EmbodimentCogServer
{

public:
    static BaseServer* createInstance();
    ComboShellServer();
    void init();

    // overrides
    bool customLoopRun();
    bool processNextMessage(Message *message);
private:
    bool _waiting;
}; // class
} } // namespace opencog::messaging

#endif
