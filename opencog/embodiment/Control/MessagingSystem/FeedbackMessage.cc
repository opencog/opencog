/**
 * FeedbackMessage.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 14:19:25 BRT 2007
 */

#include "FeedbackMessage.h"
#include "util/StringTokenizer.h"

using namespace MessagingSystem;

// ***********************************************/
// Constructors/destructors

FeedbackMessage::~FeedbackMessage() {
} 

FeedbackMessage::FeedbackMessage(const std::string &from, const std::string &to) :
                                 Message(from, to, Message::FEEDBACK) {
    this->petId.assign("");
    this->feedback.assign("");
}

FeedbackMessage::FeedbackMessage(const std::string &from, const std::string &to, 
                                 const std::string &msg) :
                                 Message(from, to, Message::FEEDBACK) {
                                     
    loadPlainTextRepresentation(msg.c_str());
}

FeedbackMessage::FeedbackMessage(const std::string &from, const std::string &to, 
                                 const std::string &petId, const std::string &feedback) : 
                                 Message(from, to, Message::FEEDBACK) {

    this->petId.assign(petId);                                
    this->feedback.assign(feedback);
}

// ***********************************************/
// Overwritten from message

const char *FeedbackMessage::getPlainTextRepresentation() {
    buffer.assign("");

    buffer.append(petId);
    buffer.append(END_TOKEN);
    buffer.append(feedback);
    return buffer.c_str();
}

void FeedbackMessage::loadPlainTextRepresentation(const char *strMessage) {
    opencog::StringTokenizer stringTokenizer((std::string)strMessage,
					      (std::string)END_TOKEN);

    petId = stringTokenizer.nextToken();
    feedback = stringTokenizer.nextToken();
}
  
// ***********************************************/
// Getters and setters

void FeedbackMessage::setFeedback(const std::string &msg) {
    feedback.assign(msg);
}

void FeedbackMessage::setFeedback(const char *msg) {
    feedback.assign(msg);
}

const std::string &FeedbackMessage::getFeedback() {
    return feedback;
}

void FeedbackMessage::setPetId(const std::string &oId){
    petId.assign(oId);
}

const std::string &FeedbackMessage::getPetId(){
    return petId;
}

