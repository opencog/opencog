/**
 * CallBackInterface.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef CALLBACKINTERFACE_H
#define CALLBACKINTERFACE_H

#include <string>

namespace opencog {

class CallBackInterface {

    public:
		
        virtual ~CallBackInterface() {}
        virtual void callBack(const std::string &message) = 0;

}; // class
}  // namespace

#endif
