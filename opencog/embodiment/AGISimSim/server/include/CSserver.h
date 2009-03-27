/*
 * opencog/embodiment/AGISimSim/server/include/CSserver.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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
