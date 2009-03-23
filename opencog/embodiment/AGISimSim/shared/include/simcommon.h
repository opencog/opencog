/***************************************************************************
 *  Definitions common to all AgiSim source files. Should be included in
 *  the beginning of each source file.
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

#ifndef SIMCOMMON_H
#define SIMCOMMON_H

#ifdef WIN32
	#define HAVE_SNPRINTF 
#endif

#include <math.h>
#include <stdarg.h>
#include <string>
#include <map>

#include "simconfig.h"

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

using namespace boost;

#include "utils.h"
#include "log.h"

//------------------------------------------------------------------------------------------------------------
/// "Variable" Singletons are sometimes created with this structure
//------------------------------------------------------------------------------------------------------------
#define SINGLETON(TYPE, VAR) TYPE& VAR() { \
    static TYPE* s = new TYPE(); \
    return *s; \
}

//------------------------------------------------------------------------------------------------------------
/**	Some helper macros to define a 'bridge' design pattern.
	They're ugly, but the resulting bridges are pretty. */
/// Put this inside the bridge header definition:
//------------------------------------------------------------------------------------------------------------
#define DEFINE_BRIDGE(__iclass) 	 \
	protected: 						 \
	static __iclass* implementation; \
	public: 						 \
	static __iclass& Get(); 		 \
	static __iclass& ResetBridge();  \
	protected:

//------------------------------------------------------------------------------------------------------------	
/// These 3 definitions in cpp file, in this order:	
//------------------------------------------------------------------------------------------------------------
#define IMPLEMENT_BRIDGE(__iclass, __par) 		\
	__iclass* __iclass::implementation = NULL;	\
	__iclass& __iclass::ResetBridge() 			\
	{ 											\
		if (implementation) { 					\
			delete implementation; 				\
			implementation = NULL; 				\
		} 										\
		return Get(); 							\
	} 											\
	__iclass& __iclass::Get() 					\
	{ 											\
		if (!implementation) 					\
		{ 

//------------------------------------------------------------------------------------------------------------
/** BRIDGE_PROVIDER for each implementation (__imp)
 which is selected if __condition is true.
 You can pass constructor parameters in __par or leave it empty.*/
//------------------------------------------------------------------------------------------------------------			
#define BRIDGE_PROVIDER(__imp, __condition, __par) 	\
	if (__condition) 								\
		implementation = new __imp(__par);

#define END_BRIDGE } 		\
	return *implementation; \
	}

#endif
