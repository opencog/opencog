/*
 * opencog/embodiment/Control/MessagingSystem/RouterMessage.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#include "RouterMessage.h"

using namespace opencog::messaging;

RouterMessage::~RouterMessage()
{
}

RouterMessage::RouterMessage(const std::string &from, const std::string &to,
                             int _encapsulateType) : Message(from, to, Message::ROUTER)
{
    encapsulateType = _encapsulateType;
    message.assign("");
}

RouterMessage::RouterMessage(const std::string &from, const std::string &to,
                             int _encapsulateType, const std::string& msg)
        : Message(from, to, Message::ROUTER)
{
    encapsulateType = _encapsulateType;
    message.assign(msg);
}

const char *  RouterMessage::getPlainTextRepresentation()
{
    return message.c_str();
}


void RouterMessage::loadPlainTextRepresentation(const char *strMessage)
{
    message.assign(strMessage);
}

void RouterMessage::setMessage(const std::string& msg)
{
    message.assign(msg);
}

const std::string& RouterMessage::getMessage()
{
    return message;
}

void RouterMessage::setEncapsulateType(int type)
{
    encapsulateType = type;
}

int RouterMessage::getEncapsulateType()
{
    return encapsulateType;
}

