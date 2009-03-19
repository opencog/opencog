/**
 * IdleTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jul  3 16:17:00 BRT 2007
 */

#ifndef IDLETASK_H
#define IDLETASK_H

#include "util/Logger.h"

namespace MessagingSystem {

class NetworkElement;

/**
 * Interface implemented by tasks supposed to be acted in NetworkElement's  idleTime()
 */
class IdleTask {

    private:

        /**
         * Used to control if the task is active or not.
         */
        bool taskActive;

    public:
		
		virtual ~IdleTask() {}
		
        /**
         * Perform the task.
         * 
         * @param ne The NetworkElement this task belongs to.
         */
        virtual void run(NetworkElement *ne) = 0;
        
        /**
         * Geters and seters
         */
        void setTaskActive(const bool& state){
            taskActive = state;
            opencog::logger().log(opencog::Logger::DEBUG, "IdleTask - taskActive set to = %d.", taskActive);
        }
    
        bool isTaskActive(){
            return taskActive;
        }

}; // class
}  // namespace

#endif
