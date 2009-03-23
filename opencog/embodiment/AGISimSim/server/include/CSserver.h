/***************************************************************************
 *  CS operations server-side implementation.        
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

#ifndef CS_SERVER
#define CS_SERVER

#include "CSops.h"
#include "CSagent.h"
#if CRYSTAL
 #include "CSanimation.h"
#endif
//------------------------------------------------------------------------------------------------------------
/** @class CSserver
	\brief CSbox implementation on the server-side. */
//------------------------------------------------------------------------------------------------------------
class CSserver : public CSbox {
protected:
	static csRef<iCollideSystem> cd_sys; //A plugin
	bool   LoadCollisionDetector ();
public:
	static csRef<iCollideSystem> GetColliderSystem();

	void DrawViews();
	bool   InitcsPlugins  (iGUIProvider* _GUIprovider );
	bool   SetupFrame     ();
	bool   ConnectToWorld (const char* worldURL);
	bool   DisconnectFromWorld ();
};

#endif
