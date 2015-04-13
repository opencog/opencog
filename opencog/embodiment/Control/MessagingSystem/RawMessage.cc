/*
 * opencog/embodiment/Control/MessagingSystem/RawMessage.cc
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


#include "MessageFactory.h"
#include "RawMessage.h"

using namespace opencog::messaging;

// ***********************************************/
// Constructors/destructors

RawMessage::~RawMessage()
{
}

RawMessage::RawMessage(const std::string &from, const std::string &to) : Message(from, to, RAW)
{
    message.assign("");
}

RawMessage::RawMessage(const std::string &from, const std::string &to, const std::string &msg) : Message(from, to, RAW)
{
    message.assign(msg);
}

RawMessage::RawMessage(const std::string &from, const std::string &to, const char *msg) : Message(from, to, RAW)
{
    message.assign(msg);
}

// ***********************************************/
// Overwritten from message

const char *RawMessage::getPlainTextRepresentation()
{
    return message.c_str();
}

void RawMessage::loadPlainTextRepresentation(const char *rawText)
{
    message.assign(rawText);
}

// ***********************************************/
// Getters and setters

void RawMessage::setMessage(const std::string &msg)
{
    message.assign(msg);
}

void RawMessage::setMessage(const char *msg)
{
    message.assign(msg);
}

const std::string &RawMessage::getMessage()
{
    return message;
}
