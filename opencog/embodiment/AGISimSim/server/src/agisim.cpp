/*
 * opencog/embodiment/AGISimSim/server/src/agisim.cpp
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include "simcommon.h"
#include "space.h"
#include "agisim.h"
#include <iostream>

#ifndef NOWX
  #include <wx/wx.h>
  #include <wx/image.h>
#endif

#include "simserver.h"
#include "CSagent.h"

#if CRYSTAL
 #include <CSops.h>

 CS_IMPLEMENT_APPLICATION
#endif

#ifdef WIN32
 #define WIN32_LEAN_AND_MEAN
 #include "Windows.h"
#endif
//--------------------------------------------------------------------------------------------------------------
void SimpleGUIProvider::Notify()
{
	int  cycle = IntConfig ("FrameUpdateDelay");

#if CRYSTAL
	if (IntConfig("EnableAnimation"))
	  {
	    if (SimServer::Get()->GetAgent(0))
	      if (SimServer::Get()->GetAgent(0)->GetAnimationControl().AnimationActive())
		{
		  CSproxy::Get()->PushFrame();
		}
	  }
	else
#else
	  {
	    if ( updateOnNextNotify || !(counter % cycle) )  {
	      LOG("Pump", 4, "Updating.");
	      updateOnNextNotify = false;
	      counter            = 0;
	      CSproxy::Get()->PushFrame();
	    }
	    else { LOG("Pump", 5, "Skipped update."); }
	  }
#endif
	counter++;
}

//--------------------------------------------------------------------------------------------------------------
void SimpleGUIProvider::SetStatusText (std::string s) { }

//--------------------------------------------------------------------------------------------------------------
void SimpleGUIProvider::LogMessage (std::string data, std::string type) {}

//--------------------------------------------------------------------------------------------------------------
bool SimpleGUIProvider::SetConnected (bool c)
{
	//if (c)  CSproxy::Get()->ForceRelight();
	connected = c;
	
	return true;
}

//--------------------------------------------------------------------------------------------------------------
bool SimpleGUIProvider::HandleEvent (csEventID evtype, unsigned int evcode, unsigned int cooked_code)
{
#if CRYSTAL
	csRef<iEventNameRegistry>  name_reg = csEventNameRegistry::GetRegistry ( ((CSbox*)CSbox::Get())->GetObjReg() );
	
	if ( evtype == csevProcess( ((CSbox*)CSbox::Get())->GetObjReg() ) )  {
		CSbox::Get()->SetupFrame();
		return true;
	}
	else if ( evtype == csevFinalProcess ( ((CSbox*)CSbox::Get())->GetObjReg()) )  {
		CSbox::Get()->FinishFrame ();
		return true;
	}
	else if ( name_reg->IsKindOf (evtype, csevKeyboardEvent(name_reg)) )
		return true; ///Keyboard is disabled on the server
	else if ( evtype == csevQuit ( ((CSbox*)CSbox::Get())->GetObjReg() ) ) {
		 LOG("!",1,"CS asked for exit...");
		SetConnected (false);
		return true;
	}

	/*case csevMouseMove:
		LOG("AGISIM", 2, "Mouse move to " << ev.Mouse.x << " " << ev.Mouse.y);
		break;
	case csevMouseDown:
		LOG("AGISIM", 2, "Mouse button "<< ev.Mouse.Button <<" down at " <<  ev.Mouse.x <<" " << ev.Mouse.y);
		break;
	case csevMouseUp:
		LOG("AGISIM", 2, "Mouse button "<< ev.Mouse.Button <<" up at " <<  ev.Mouse.x <<" " << ev.Mouse.y);
		break;*/
#endif
	return false;
}

//--------------------------------------------------------------------------------------------------------------
SimpleGUIProvider::SimpleGUIProvider ()
									 : connected(false), updateOnNextNotify(false), counter(0)
{}

//--------------------------------------------------------------------------------------------------------------
SimpleGUIProvider::~SimpleGUIProvider()
{
	LOG("SimpleGUIProvider", 4, "Destroying...");
}

//--------------------------------------------------------------------------------------------------------------
void SimpleGUIProvider::PleaseUpdateFrame()
{
	updateOnNextNotify = true;
	 LOG("Pump", 3, "Received update hint");
}

//--------------------------------------------------------------------------------------------------------------
void SimpleGUIProvider::PleaseSkipUpdateFrame()
{
	counter = 1;	
}

//--------------------------------------------------------------------------------------------------------------
bool SimpleGUIProvider::IsConnected()
{
	return connected;
}


void CreateSprites();
//--------------------------------------------------------------------------------------------------------------
//--- Main function																							   -	
//--------------------------------------------------------------------------------------------------------------
#ifndef ___WIN32
int main (int argc, char* argv[])
{
#else
int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    int    argc;
    char** argv;


    char*  arg;
    int    index;
    int    result;


    // count the arguments
    
    argc = 1;
    arg  = lpCmdLine;
    
    while (arg[0] != 0) {


        while (arg[0] != 0 && arg[0] == ' ') {
            arg++;
        }


        if (arg[0] != 0) {
        
            argc++;
        
            while (arg[0] != 0 && arg[0] != ' ') {
                arg++;
            }
        
        }
    
    }    
    
    // tokenize the arguments

    argv = (char**)malloc(argc * sizeof(char*));


    arg = lpCmdLine;
    index = 1;


    while (arg[0] != 0) {


        while (arg[0] != 0 && arg[0] == ' ') {
            arg++;
        }


        if (arg[0] != 0) {
        
            argv[index] = arg;
            index++;
        
            while (arg[0] != 0 && arg[0] != ' ') {
                arg++;
            }
        
            if (arg[0] != 0) {
                arg[0] = 0;    
                arg++;
            }
        
        }
    
    }    


    // put the program name into argv[0]

    char filename[_MAX_PATH];
    
    GetModuleFileName(NULL, filename, _MAX_PATH);
    argv[0] = filename;

HANDLE hOutput = (HANDLE)GetStdHandle( STD_OUTPUT_HANDLE ); 

// Set the text output position to (5,10) 
COORD sPos; 
sPos.X = 5; 
sPos.Y = 10; 
SetConsoleCursorPosition( hOutput, sPos ); 

// Set the color to bright green 
SetConsoleTextAttribute( hOutput, 
FOREGROUND_INTENSITY | FOREGROUND_GREEN ); 

// Write the text 
DWORD nWritten; 
WriteConsole( hOutput, "This is a test", 14, &nWritten, NULL );

#endif

	try  {
		LOG("AGISIM", 0, "Starting...");
                initReferenceTime();

puts(".");
		Config::Instance().Create();

		CSproxy*           cs       = CSproxy::Get();
		SimpleGUIProvider* cshelper = new SimpleGUIProvider();

 		cs->OnGUIinit(argc, argv);
		#ifndef NOWX
			wxImage::AddHandler (new wxJPEGHandler);
		#endif
		LOG("AGISIM", 1, "Initializing CS...");
	  
  		if ( !cs->InitcsPlugins(cshelper) )  return -1;
		
		CSstatus*          cs_status = NULL;
  		if (NULL == (cs_status = cs->OpenMainSystem()))
			return false;
		
		/*  We currently don't support command line arguments, but
			we could do that here: */
		/*  if (!cs_status->cmd_url.empty())
			if (!cs->ConnectToWorld(cs_status->cmd_url.c_str()))
				return false;*/

		Log::masterLogLevel = IntConfig ("LogLevel");	
		LOG("AGISIM", 1, "Initialize ok.");
		
		//	CreateSprites();
		//  gets(0);
  		cs->ConnectToWorld("simworld.def");	
  		LOG("AGISIM", 1, "World connection ok. Entering run loop.");

		iEngine::Instance().MeshDump();
		
		while ( cshelper->IsConnected() ) {
			cshelper->Notify();
			usleep (2000); //Sleep 2000 micro sec... until the end of eternity!
		}
		//Alternative way would be:
		//cs->RunLoop();
	} catch(std::string s) { puts(s.c_str()); }
	catch(...) { puts("Unknown exception in run loop."); }

	return 0;
}
