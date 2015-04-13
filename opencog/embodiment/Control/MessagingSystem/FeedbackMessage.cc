/*
 * opencog/embodiment/Control/MessagingSystem/FeedbackMessage.cc
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

#include <opencog/util/StringTokenizer.h>

#include "MessageFactory.h"
#include "FeedbackMessage.h"

using namespace opencog::messaging;

// ***********************************************/
// Constructors/destructors

FeedbackMessage::~FeedbackMessage()
{
}

FeedbackMessage::FeedbackMessage(const std::string &from, const std::string &to) :
        Message(from, to, FEEDBACK)
{
    this->petId.assign("");
    this->feedback.assign("");
}

FeedbackMessage::FeedbackMessage(const std::string &from, const std::string &to,
                                 const std::string &msg) :
        Message(from, to, FEEDBACK)
{

    loadPlainTextRepresentation(msg.c_str());
}

FeedbackMessage::FeedbackMessage(const std::string &from, const std::string &to,
                                 const std::string &petId, const std::string &feedback) :
        Message(from, to, FEEDBACK)
{

    this->petId.assign(petId);
    this->feedback.assign(feedback);
}

// ***********************************************/
// Overwritten from message

const char *FeedbackMessage::getPlainTextRepresentation()
{
    buffer.assign("");

    buffer.append(petId);
    buffer.append(END_TOKEN);
    buffer.append(feedback);
    return buffer.c_str();
}

void FeedbackMessage::loadPlainTextRepresentation(const char *strMessage)
{
    opencog::StringTokenizer stringTokenizer((std::string)strMessage,
            (std::string)END_TOKEN);

    petId = stringTokenizer.nextToken();
    feedback = stringTokenizer.nextToken();
}

// ***********************************************/
// Getters and setters

void FeedbackMessage::setFeedback(const std::string &msg)
{
    feedback.assign(msg);
}

void FeedbackMessage::setFeedback(const char *msg)
{
    feedback.assign(msg);
}

const std::string &FeedbackMessage::getFeedback()
{
    return feedback;
}

void FeedbackMessage::setPetId(const std::string &oId)
{
    petId.assign(oId);
}

const std::string &FeedbackMessage::getPetId()
{
    return petId;
}

