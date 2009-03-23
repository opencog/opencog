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
#ifndef __WXPHANDLER_H__
#define __WXPHANDLER_H__

#include "phandler.h"

class iGUIProvider;

//------------------------------------------------------------------------------------------------------------
class WXPHandler : public PerformativeHandler
{
	iGUIProvider* gui;
public:
    WXPHandler(iGUIProvider* _gui);
    
    virtual int action     (std::string action, std::string parameters);
    virtual int sensation  (std::string type, std::string parameters);
    virtual int mate	   ();

    virtual int addAgent   (std::string name,std::string nick);
    virtual int removeAgent(std::string name);    
};


#endif
