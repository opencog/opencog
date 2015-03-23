/*
 * opencog/embodiment/Control/MessagingSystem/TickMessage.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Elvys Borges
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
#include "TickMessage.h"

using namespace opencog::messaging;

// ***********************************************/
// Constructors/destructors

TickMessage::~TickMessage()
{
}

TickMessage::TickMessage(const std::string &from, const std::string &to) : Message(from, to, TICK)
{
}

// ***********************************************/
// Overwritten from message

const char *TickMessage::getPlainTextRepresentation()
{
    static const char* tick = "TICK_MESSAGE";
    return tick;
}

void TickMessage::loadPlainTextRepresentation(const char *strMessage)
{

}

