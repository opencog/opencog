#ifndef ACTIONPLANSENDER_H_
#define ACTIONPLANSENDER_H_

/** 
 * ActionPlanSender.h
 * 
 * Author: Welter Luigi 
 * Copyright(c), 2007
 */
 
#include "ActionPlan.h" 
 
namespace PerceptionActionInterface {


class ActionPlanSender {
	
	public: 
	
		virtual ~ActionPlanSender() {}
		
		/**
		 * Sends the action plan to the target Virtual World.
		 * 
		 * @param ationPlan	a reference to the object that contains the action plan to be sent. Note that 
		 * 		  this reference may not be valid after the call of this method because caller may release the object. 
		 *        So, if the implementation of this method is going to store the action plan for further use, it must 
		 *        clone/copy the ActionPlan object.
		 * 
		 * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
		 */ 	
		virtual bool sendActionPlan(const ActionPlan& actionPlan)=0;

        /**
         * Sends the emotional feelings meessage to the target Virtual World.
         * 
         * @param feelings	The XML message already converted int a string representation 
         * 
         * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
         */ 	
        virtual bool sendEmotionalFeelings(const std::string& emotionalFeelings)=0;

};

} // namespace

#endif /*ACTIONPLANSENDER_H_*/
