/***************************************************************************
 *  VOS property helper class.        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 * 26.01.06  FP   formatting
 ****************************************************************************/
#ifndef _LOCALPROPERTY_HH_
#define _LOCALPROPERTY_HH_

#include <property.h>

//---------------------------------------------------------------------------------------
class LocalProperty : public Property {
	
public:
    LocalProperty();

    virtual void write(int start, const std::string& newdata)
        { write( start, newdata); }

    virtual void replace(const std::string& newdata, const std::string& newtype = "?")
        { replace(0, newdata, newtype); }
};

#endif
