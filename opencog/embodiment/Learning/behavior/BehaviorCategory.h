/**
 * BehaviorCategory.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Aug 22 12:57:33 BRT 2007
 */

#ifndef BEHAVIORCATEGORY_H
#define BEHAVIORCATEGORY_H

#include "CompositeBehaviorDescription.h"
#include <string>
#include <vector>

namespace behavior {

class BehaviorCategory {

    private:

        std::vector<CompositeBehaviorDescription> entries;
	//Note from Nil : apparently timelineSets and timelineIntervals
	//are not used so I commented them
        //std::vector<std::vector<Handle> > timelineSets;
        //std::vector<long> timelineIntervals;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~BehaviorCategory();
        BehaviorCategory();
          
        // ***********************************************/
        // Building API

        void addCompositeBehaviorDescription(const CompositeBehaviorDescription &bd);

        // ***********************************************/
        // Manipulation API

        /**
         * @return the number of exemplars in BehaviorCategory
         */
        int getSize() const;

        const std::vector<CompositeBehaviorDescription> &getEntries() const;

	bool empty(); //check if empty
	void clear(); //clear the behavior category

        // ***********************************************/
        // Test/debug
        
        std::string toString();
        std::string toStringHandles();
        std::string toStringTimeline();

}; // class
}  // namespace

#endif
