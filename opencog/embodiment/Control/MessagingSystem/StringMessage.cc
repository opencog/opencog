/*
 * opencog/embodiment/Control/MessagingSystem/StringMessage.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "StringMessage.h"

using namespace opencog::messaging;

// ***********************************************/
// Constructors/destructors

StringMessage::~StringMessage()
{
}

StringMessage::StringMessage(const std::string &from, const std::string &to) : Message(from, to, Message::STRING)
{
    message.assign("");
}

StringMessage::StringMessage(const std::string &from, const std::string &to, const std::string &msg) : Message(from, to, Message::STRING)
{
    message.assign(msg);
}

StringMessage::StringMessage(const std::string &from, const std::string &to, const char *msg) : Message(from, to, Message::STRING)
{
    message.assign(msg);
}

// ***********************************************/
// Overwritten from message

const char *StringMessage::getPlainTextRepresentation()
{
    return message.c_str();
}

void StringMessage::loadPlainTextRepresentation(const char *strMessage)
{
    message.assign(strMessage);
}

// ***********************************************/
// Getters and setters

void StringMessage::setMessage(const std::string &msg)
{
    message.assign(msg);
}

void StringMessage::setMessage(const char *msg)
{
    message.assign(msg);
}

const std::string &StringMessage::getMessage()
{
    return message;
}
