/*
 * opencog/embodiment/AGISimSim/server/src/sensationxml.cpp
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

/**

Copyright Ari A. Heljakka 03/2006
Not part of AGI-Sim.
Do not distribute.

TO_COMPLETE

*/


/***************************************************************************
 *  Sensation implementation files that are shared between
 *	AgiSim and SimCreator (simcreator.cc).
 *
 *  Project: Novamente
 ****************************************************************************/


#include "simcommon.h"
#include "space.h"
#include "sensors.h"
#include "world_vocabulary.h"


//---------------------------------------------------------------------------------------------------------------
std::string  CustomSensation::AsXML() const
{
	if ( !STLhas( properties, toupper(WINTENSITY) ) )  {
		 LOG("CustomSensation", 0, "Has no intensity!");
		return "";
	}
	string  intensity = properties.find( toupper(WINTENSITY) )->second;
	
	return  XMLembed( "custom",
				XMLembed( "msg", name)
				+ XMLembed( WQUALITY, Get(WQUALITY) )
				+ XMLembed( WINTENSITY, intensity )
			);	
}

//--------------------------------------------------------------------------------------------------------------
std::string  Sound::AsXML() const
{
	if ( !STLhas( properties, toupper(WINTENSITY) ) )  {
		 LOG("Sound", 0, "Has no intensity!");
		return "";
	}
	string  intensity = properties.find( toupper(WINTENSITY) )->second;
	
	return XMLembed( WSOUND,
				XMLembed( WFREQ, Get(WFREQ) )
				+ XMLembed( WINTENSITY, intensity )
			);
}

//--------------------------------------------------------------------------------------------------------------
std::string Smell::AsXML() const
{
	if ( !STLhas( properties, toupper(WINTENSITY) ) )  {
		 LOG("Smell", 0, "Has no intensity!");
		return "";
	}
	string  intensity = properties.find(toupper(WINTENSITY))->second;
	
	return XMLembed(WSMELL,
				XMLembed( WQUALITY, Get(WQUALITY) )
				+ XMLembed( WINTENSITY, intensity )
			);	
}

//--------------------------------------------------------------------------------------------------------------
std::string Taste::AsXML() const
{
	if ( !STLhas( properties, toupper(WINTENSITY) ) )  {
		 LOG("Taste", 0, "Has no intensity!");
		return "";
	}
	string  intensity = properties.find(toupper(WINTENSITY))->second;
	
	return XMLembed( WTASTE,
				XMLembed( WQUALITY, Get(WQUALITY) )
				+ XMLembed( WINTENSITY, intensity )
			);
}

//--------------------------------------------------------------------------------------------------------------
std::string Proprioception::AsXML() const
{
	return XMLembed( WENERGY, toString(energy) );
}

//--------------------------------------------------------------------------------------------------------------
Smell::Smell(int _volume, int _quality)
{
	 LOG("!", 0, toString(_volume) + "/" + toString(_quality))
	Set( WINTENSITY, _volume  );
	Set( WQUALITY  , _quality );
}

//--------------------------------------------------------------------------------------------------------------
Sound::Sound( int _volume, int _freq, int _duration )
{
	Set( WINTENSITY, _volume   );
	Set( WFREQ     , _freq     );
	Set( WDURATION , _duration );
}

//--------------------------------------------------------------------------------------------------------------
Smell::Smell()
{
	Set( WINTENSITY, 0 );
	Set( WQUALITY  , 0 );
}

Sound::Sound()
{
	Set( WINTENSITY, 0 );
	Set( WFREQ     , 0 );
	Set( WDURATION , 0 );
}

//--------------------------------------------------------------------------------------------------------------
Proprioception::Proprioception(int _energy)
{
	Set(WENERGY  , _energy);
}
