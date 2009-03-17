#ifndef PVPACTIONPLANSENDER_H_
#define PVPACTIONPLANSENDER_H_

/** 
 * PVPActionPlanSender.h
 * 
 * Concrete subclass of ActionPlanSender that sends action plans to the PVP 
 * 
 * Author: Welter Luigi 
 * Copyright(c), 2007
 */

#include "ActionPlanSender.h"
#include "NetworkElement.h"
#include "ActionPlan.h"

namespace OperationalPetController {

using namespace MessagingSystem;
 
class PVPActionPlanSender: public PerceptionActionInterface::ActionPlanSender {
	
    private: 

        /**
         * The id of the pet which the action plans will be sent for   
         */
        std::string petId;
         
        /**
         * A network element object used to send the action plan in a message to PVP
         */
        NetworkElement* ne;

        /**
         * Indicates if the pvp messages should be logged or not.
         */
        bool logPVPMessage;
	
    public:
    
        /**
         * Constructor
         */
        PVPActionPlanSender(const std::string& petId, NetworkElement *);
        ~PVPActionPlanSender();
	
        /**
         * Sends the action plan to the target Virtual World.
         * 
         * @param ationPlan	a reference to the object that contains the action plan to be sent. Note that 
         *        this reference may not be valid after the call of this method because caller may release the object. 
         * 
         * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
         */ 	
        bool sendActionPlan(const PerceptionActionInterface::ActionPlan& actionPlan);

        /**
         * Sends the emotional feelings meessage to the target Virtual World.
         * 
         * @param feelings	The XML message already converted int a string representation 
         * 
         * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
         */ 	
        bool sendEmotionalFeelings(const std::string& emotionalFeelings);

};

} // namespace

#endif // PVPACTIONPLANSENDER_H_
