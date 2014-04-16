/*
 * opencog/embodiment/Control/Procedure/ComboShellServer.cc
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

#include <opencog/comboreduct/combo/vertex.h>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include "ComboShellServer.h"
#include "ComboInterpreter.h"
#include "ComboProcedureRepository.h"
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

using namespace Procedure;
using namespace AvatarCombo;
using namespace boost;
using namespace std;
using namespace opencog::messaging;

BaseServer* ComboShellServer::createInstance()
{
    return new ComboShellServer;
}

ComboShellServer::ComboShellServer()
{
}

void ComboShellServer::init(const string& OAC_ID)
{
    setNetworkElement(new NetworkElement(opencog::config().get("COMBO_SHELL_ID"),
                                         opencog::config().get("COMBO_SHELL_IP"),
                                         opencog::config().get_int("COMBO_SHELL_PORT")));
    _waiting = false;
    _OAC_ID = OAC_ID;
}

bool ComboShellServer::processNextMessage(opencog::messaging::Message *msg)
{
    if (msg->getTo() != getID())
        return false;
    string res = msg->getPlainTextRepresentation();

    if (res == "action_failure")
        cout << "execution failed!" << endl;
    else {
        if (res != "action_success")
            cout << "result: " << res << endl;
        cout << "execution succeeded!" << endl;
    }
    _waiting = false;
    return false;
}

bool ComboShellServer::customLoopRun()
{
    if (_waiting) {
        return MessageCogServer::customLoopRun();
    }

    combo_tree tr;
    if (!cin.good()) {
        cout << endl;
        exit(0);
    }

start:
    cout << "> ";
    if (cin.peek() == ' ' ||
            cin.peek() == '\n' ||
            cin.peek() == '\t')
        cin.get();
    while (cin.peek() == ' ' ||
            cin.peek() == '\n' ||
            cin.peek() == '\t') {
        if (cin.peek() == '\n')
            cout << "> ";
        cin.get();
    }
    if (cin.peek() == '#') { //a comment line
        char tmp[1024];
        cin.getline(tmp, 1024);
        goto start;
    }
    cin >> tr;
    if (!cin.good()) {
        cout << endl;
        exit(0);
    }

    try {
        stringstream ss;
        ss << tr;
        StringMessage msg(opencog::config().get("COMBO_SHELL_ID"),
                          _OAC_ID,
                          ss.str());
        cout << "sending schema " << ss.str() << "..." << endl;
        sendMessage(msg);
        cout << "schema sent to OAC, waiting for result ..." << endl;
        _waiting = true;
    } catch (...) {
        cout << "execution failed (threw exception)" << endl;
    }

    return true;
}
