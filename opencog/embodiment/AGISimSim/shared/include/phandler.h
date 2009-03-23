/***************************************************************************
 *          
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 	
 ****************************************************************************/

#ifndef __PHANDLER_H__
#define __PHANDLER_H__

#include <string>

//------------------------------------------------------------------------------------------------------------
class PerformativeHandler {
  public:
    PerformativeHandler(); //(RemoteAgent* ra);
    
	// Action may be redundant, since sensation handles most reponses to actions
    virtual int action 		(std::string action, std::string parameters) = 0;	
	virtual int sensation   (std::string type, std::string parameters) = 0;
    virtual int mate		() = 0;    
    virtual int addAgent    (std::string name, std::string nick) = 0;
    virtual int removeAgent (std::string name) = 0;
};

#endif
