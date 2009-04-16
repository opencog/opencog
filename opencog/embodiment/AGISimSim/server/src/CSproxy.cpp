/*
 * opencog/embodiment/AGISimSim/server/src/CSproxy.cpp
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

/**

Copyright Ari A. Heljakka / GenMind Ltd.
Not part of AGI-Sim.
Do not distribute.

*/

#include "simcommon.h"
#include "space.h"

#if CRYSTAL

#include "CSproxy.h"
#include "CSserver.h"


//--- Proxy methods -------------------------------------------------------------------------------------------
iGUIProvider*     CSbox::GUIprovider	  = NULL;
iObjectRegistry*  CSbox::object_reg	      = NULL;
csRef<iView>      CSbox::view;
CSproxy*          CSproxy::implementation = NULL;

//-------------------------------------------------------------------------------------------------------------
CSbox::~CSbox() { }

//-------------------------------------------------------------------------------------------------------------
void CSbox::OnGUIinit (int argc, char const * const argv[])
{
	 LOG("CSbox::OnGuiInit", 1, argv[0]);
	object_reg = csInitializer::CreateEnvironment(argc, argv);
}

//-------------------------------------------------------------------------------------------------------------
void CSbox::OnGUIexit()
{
	 LOG("CSbox::OnExit", 1, "Destroying CS application... ");
	csInitializer::DestroyApplication(object_reg);
	 LOG("CSbox::OnExit", 1, "Done. ");
}

//-------------------------------------------------------------------------------------------------------------
bool CSbox::MyEventHandler (iEvent& ev)
{
	//LOG("CSbox::MyEventHandler (iEvent& ev)", 0, "Started!");
	//CSclient* self_ref = static_cast<CSclient*>(CSclient::Get());
    CSserver*  self_ref = static_cast<CSserver*>(CSserver::Get());
	
	if ( !GUIprovider || !self_ref )  return false;
	csRef<iEventNameRegistry> name_reg = csEventNameRegistry::GetRegistry(object_reg);

	//LOG("CSbox::MyEventHandler (iEvent& ev)", 0, name_reg->GetString(ev.Name));
	//LOG("CSbox::MyEventHandler (iEvent& ev) SHOULD_HAVE", 0, name_reg->GetString(csevProcess(object_reg)));
	
	if (1)	{
		//if (!self_ref->GUI_driven_frames)	
		//LOG("!", 0, "Towards setupframe!");		  		
		//if (ev.Type == csevBroadcast && ev.Command.Code == cscmdProcess)
		//if (ev.Type == csevBroadcast && csCommandEventHelper::GetCode(&ev) == cscmdProcess)

		if (ev.Name == csevProcess(object_reg))  {
			//LOG("!", 0, "SetupFrame CALLED!");		  
			self_ref->SetupFrame();		  
			return GUIprovider->HandleEvent (ev.Name, csCommandEventHelper::GetCode(&ev), 0);
	  }

	  else if( ev.Name == csevFinalProcess(object_reg) )  {
	    self_ref->FinishFrame ();
	    return true;
	  }
	}
	
	if ( name_reg->IsKindOf(ev.Name, csevKeyboardEvent(name_reg)) )  {
		if (csKeyEventHelper::GetCookedCode (&ev) == CSKEY_ESC) {
			csRef<iEventQueue>  q (CS_QUERY_REGISTRY (object_reg, iEventQueue));
			if (q)  q->GetEventOutlet()->Broadcast(csevQuit(object_reg));
		}
		return GUIprovider->HandleEvent(ev.Name, csKeyEventHelper::GetEventType (&ev), csKeyEventHelper::GetCookedCode (&ev));
	}

	/*case csevMouseMove: return GUIprovider->HandleEvent(ev.Type, csMouseEventHelper::GetX(&ev), csMouseEventHelper::GetY(&ev));						
		break;
	case csevMouseDown: return GUIprovider->HandleEvent(ev.Type, csMouseEventHelper::GetX(&ev), csMouseEventHelper::GetY(&ev));
		break;
	case csevMouseUp: return GUIprovider->HandleEvent(ev.Type,  csMouseEventHelper::GetX(&ev), csMouseEventHelper::GetY(&ev));
		break;*/

	return false;
}

//-------------------------------------------------------------------------------------------------------------
void CSbox::RunLoop()
{
	csDefaultRunLoop (object_reg);
}

//-------------------------------------------------------------------------------------------------------------
void CSbox::PushFrame()
{
	if (GUIprovider->IsConnected()) {
		csRef<iEventQueue>    q  (CS_QUERY_REGISTRY (object_reg, iEventQueue));
		if (!q)  return ;
		csRef<iVirtualClock>  vc (CS_QUERY_REGISTRY (object_reg, iVirtualClock));

		if (vc)  vc->Advance();

		q->Process();
  }
}

//-------------------------------------------------------------------------------------------------------------
bool CSbox::InitcsPlugins (iGUIProvider* _GUIprovider )
{
	GUIprovider = _GUIprovider;
	
	 LOG("CSbox::InitcsPlugins", 1, "Initializing Crystal Space");

	csRef<iConfigManager>  confmgr = CS_QUERY_REGISTRY (object_reg, iConfigManager);
	pluginmanager = CS_QUERY_REGISTRY (object_reg, iPluginManager);
	confmgr->SetDynamicDomainPriority (iConfigManager::PriorityMax);
	
	if (!csInitializer::RequestPlugins (object_reg, CS_REQUEST_VFS		  , CS_REQUEST_OPENGL3D	 , CS_REQUEST_ENGINE,
													CS_REQUEST_FONTSERVER , CS_REQUEST_IMAGELOADER,
													CS_REQUEST_LEVELLOADER, CS_REQUEST_END))
					//CS_REQUEST_REPORTER, CS_REQUEST_REPORTERLISTENER,
                    //CS_REQUEST_PLUGIN("crystalspace.mesh.crossbuilder", iCrossBuilder),
                    //CS_REQUEST_PLUGIN("crystalspace.modelconverter.multiplexer", iModelConverter),
                                     
	{
		 LOG("CSProxy",0, "Can't initialize plugins!");
		return false;
	}

	 LOG("AGISIM", 1, "Plugins requested.");

	if (!csInitializer::SetupEventHandler (object_reg, MyEventHandler))	{
		 LOG("CSProxy",0, "Can't setup even handler.");
		return false;
	}

	// The virtual clock.
	vc = CS_QUERY_REGISTRY (object_reg, iVirtualClock);
	if (vc == 0) {
		 LOG("crystalspace.application.agisim", 0,  "Can't find the virtual clock!");	return false;
	}

	g2d = CS_QUERY_REGISTRY (object_reg, iGraphics2D);
	if (!g2d) {
		 LOG("CS",0, "No iGraphics2D plugin!");	return false;
	}
	  
	// Find the pointer to engine plugin
	engine = CS_QUERY_REGISTRY (object_reg, iEngine);
	if (engine == 0) {
		 LOG("crystalspace.application.agisim", 0, "No iEngine plugin!");            return false;
	}

	loader = CS_QUERY_REGISTRY (object_reg, iLoader);
	if (loader == 0) {
		 LOG("crystalspace.application.agisim", 0, "No iLoader plugin!");            return false;
	}

	g3d = CS_QUERY_REGISTRY (object_reg, iGraphics3D);
	if (g3d == 0) {
		LOG("crystalspace.application.agisim", 0, "No iGraphics3D plugin!");         return false;
	}
	  
	if (use_keyboard)  {
		kbd = CS_QUERY_REGISTRY (object_reg, iKeyboardDriver);
		if (kbd == 0)  {
			LOG("crystalspace.application.agisim",0, "No iKeyboardDriver plugin!");  return false;
		}
	}
	  
	 LOG("CSProxy", 1, "Shared plugins loaded.");	  
	return true;
}  

//--- Open the main system. This will open all the previously loaded plug-ins  ---------------------------------
CSstatus* CSbox::OpenMainSystem()
{
	 LOG("CSProxy", 1, "Opening application...");

	if (!csInitializer::OpenApplication (object_reg))  {
		 LOG("CSbox", 0, "OpenApplication failure.");
    	return NULL;
  	}
	 LOG("CSproxy",1,"Opened.");

  	CSstatus*  ret = new CSstatus;

  	// Parse command line options for CS
  	csRef<iCommandLineParser>  cmdline = CS_QUERY_REGISTRY (object_reg, iCommandLineParser);

 	 LOG("CSProxy", 1, "Using engine.");
	// Light cache is disabled!
  	engine->SetLightingCacheMode (CS_ENGINE_CACHE_NOUPDATE);

 	 LOG("CSProxy", 1, "Preparing engine.");

  	engine->Prepare ();

  	 LOG("CSProxy", 1, "Engine prepared.");
  
  	iGraphics2D* g2d = g3d->GetDriver2D ();

  	 LOG("CSProxy", 1, "2D ok.");
  
  	neutral_view = csPtr<iView> (new csView (engine, g3d));
	if ( neutral_view && neutral_view->GetCamera() ) {
  		neutral_view->GetCamera()->SetSector (engine->CreateSector("_tmp"));
  		 LOG("CSProxy", 1, "Camera ok."); 

  		neutral_view->SetRectangle ( 0, 0, g2d->GetWidth(), g2d->GetHeight() );  
  		TakeView( neutral_view ); 
	}
	else {
		 LOG("CSProxy", 0, "Neutral view could not be created!");
	}
  
  	// Get URL from command line
  	char*  vosWorldURL;
  	if ( cmdline->GetName() ) {
	    vosWorldURL  = strdup(cmdline->GetName());
    	ret->cmd_url = vosWorldURL;
    	free (vosWorldURL);
  	}

  	return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------------------
bool CSbox::TakeView(csRef<iView>  _view)
{
  view = _view;

  if ( IntConfig("DrawGUI") )  {	
	if ( !g3d->BeginDraw(engine->GetBeginDrawFlags() | CSDRAW_3DGRAPHICS | CSDRAW_CLEARZBUFFER | CSDRAW_CLEARSCREEN) )  {
		 LOG("CSProxy", 1, "ERROR: in 3D Drawing.");	  
	  	return false;
  	}
	 LOG("CSProxy", 1, "3D Drawing ok.");
	view->Draw();
	 LOG("CSProxy", 1, "View Drawing ok.");
	return true;
  }
  else
	  return true;
}

//----------------------------------------------------------------------------------------------------------------------------------------------
void CSbox::FinishFrame ()
{
  	if ( IntConfig("DrawGUI") )  {
 		g3d->FinishDraw ();
  		g3d->Print (0);
	}
}

//----------------------------------------------------------------------------------------------------------------------------------------------
bool CSbox::OnConnectToWorld()
{
	 LOG("CSBox",0,"OnConnectToWorld");

	double x, y, z;
	x = y = z = 0.0;	

	view->GetCamera()->GetTransform().SetOrigin( csVector3(x,y,z) );

  	if (IntConfig("DrawGUI"))  {
		if (!g3d->BeginDraw(engine->GetBeginDrawFlags() | CSDRAW_3DGRAPHICS | CSDRAW_CLEARZBUFFER | CSDRAW_CLEARSCREEN))
			LOG("CSBox",2, "g3d Error...");			
		view->Draw();
	}

	GUIprovider->SetConnected  (true);	
	GUIprovider->SetStatusText ("Connected");
	 LOG("CSBox",2, "Push 1st frame...");
	
	PushFrame();
	
	 LOG("CSBox",0, "OnConnectToWorld ok");
	
	return true;
}

//----------------------------------------------------------------------------------------------------------------------------------------------
iObjectRegistry*  CSbox::GetObjReg() 
{ 
	return object_reg; 
}

//----------------------------------------------------------------------------------------------------------------------------------------------
void CSbox::ForceRelight()  
{ 
	if (engine) engine->ForceRelight(); 
}

#else

#endif
