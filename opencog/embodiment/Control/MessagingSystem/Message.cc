/*
 * opencog/embodiment/Control/MessagingSystem/Message.cc
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


#include <stdio.h>

#include "Message.h"
#include "MessagingSystemExceptions.h"

namespace opencog { namespace messaging {
    
Message::~Message()
{
}

Message::Message(const std::string &from, const std::string &to, int type)
{
    this->from = from;
    this->to = to;
    this->type = type;
}

// Setters and getters

void Message::setFrom(const std::string &from)
{
    this->from = from;
}

void Message::setTo(const std::string &to)
{
    this->to = to;
}

void Message::setType(int type)
{
    this->type = type;
}

const std::string &Message::getFrom() const
{
    return from;
}

const std::string &Message::getTo() const
{
    return to;
}

int Message::getType() const
{
    return type;
}

} } // namespace opencog::messaging

