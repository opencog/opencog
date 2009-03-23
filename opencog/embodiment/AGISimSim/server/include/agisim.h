/***************************************************************************
 *  The main server application classes.
 * 
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka & Joel Pitt / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *																			
 *	19.01.06	FP	formatting 
 ****************************************************************************/

#ifndef __AGISIM_H__
#define __AGISIM_H__

#include "CSproxy.h"

//------------------------------------------------------------------------------------------------------------
/** @class SimpleGUIProvider
	\brief The CSProxy callback class on the server-side*/
//------------------------------------------------------------------------------------------------------------
class SimpleGUIProvider : public iGUIProvider {
private:
	bool connected;
	bool updateOnNextNotify;
	long counter;
public:
	SimpleGUIProvider();
	virtual ~SimpleGUIProvider();

	bool IsConnected			();
	bool SetConnected			(bool c);
	void SetStatusText			(std::string s);
	void SetEnergy				(int energy);
	void LogMessage				(std::string data, std::string type);	
	void OnSetupFrameBegin		(shared_ptr<unsigned char>& pixelsrc, int& w, int& h) { }
	void OnSetupFrameEnd		() { } //gibt es einen Grund für die Klammern anstatt einfach einen ";" zu setzten ?	
	void PleaseUpdateFrame	 	();
	void PleaseSkipUpdateFrame	();

	virtual void Notify();
	
	DECLARE_HANDLE_CS_EVENT_METHOD;
};

#endif // __AGISIM_H__
