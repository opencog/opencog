/*
 * opencog/embodiment/AGISimSim/server/src/sensation.cpp
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


/***************************************************************************
 *  Sensation classes.
 *
 *  Project: Novamente
 *
 ****************************************************************************/

#include "simcommon.h"
#include "space.h"
#include <math.h>

#include "sensors.h"
#include "world_vocabulary.h"
#include "demon.h"
#if CRYSTAL
 #include "CSagent.h"
#else
#endif

using std::string;

//--------------------------------------------------------------------------------------------------------------
Sensation::Sensation()  {
	Set( WQUALITY  , 0 );
	Set( WINTENSITY, 0 );
}

//--------------------------------------------------------------------------------------------------------------
Sensation::Sensation( int _i, int _q )  {
	Set( WQUALITY  , _i );
	Set( WINTENSITY, _q );
}

//--------------------------------------------------------------------------------------------------------------
Field::Field() {
	Set( WINTENSITY, 0 );
	//Volume an sich isn't knowable (Kant, I., 1781;)
	MakePrivateGet( WINTENSITY );
}
//--------------------------------------------------------------------------------------------------------------
Taste::Taste( XMLNode &node ){
	try  {
		XMLVALIDATE( node, WTASTE, csPrintf( "Bad taste entry." ) );
		XMLITERATE ( node, property )  {
			string  nodename = ( *property )->TagData().name;
			if ( nodename != WINTENSITY  &&  nodename != WQUALITY )  {
				 LOG( "Taste", 1, "Bad taste entry." );
			}
			else
				Set( nodename, ( *property )->TagData().textcontent );
		}
	}
	catch ( ... ) {}
}

//--------------------------------------------------------------------------------------------------------------
Sound::Sound(std::string  _source, int  _volume, int  _freq, int	_duration )
{
	Field::source        = _source;

	Set( WINTENSITY, _volume );
	Set( WFREQ, _freq );
	Set( WDURATION, _duration );
}

//--------------------------------------------------------------------------------------------------------------
Sound::Sound(std::string  _source, XMLNode&  node )
{
	Field::source        = _source;

	Set( WFREQ    , 0 );
	Set( WDURATION, 0 );

	try  {
		XMLVALIDATE( node, WSOUND, csPrintf( "Bad sound entry." ) );
		XMLITERATE ( node, property )  {
			string  nodename = (*property)->TagData().name;
			if ( nodename != WQUALITY  &&  nodename != WFREQ  &&  nodename != WINTENSITY )  {
				 LOG( "Taste", 1, "Bad sound entry." );
			}
			else
				Set( nodename, ( *property )->TagData().textcontent );
		}
	}
	catch ( ... ){}
}

//--------------------------------------------------------------------------------------------------------------
int Field::VolumeAt( double x, double y, double z, int volumeFadePerDistance, std::string wIntensityPropertyName) const  
{
    //PrintList(); // PrintList is time-consuming
	if ( !STLhas( properties, wIntensityPropertyName) )  {
		 LOG( "Field", 0, "Has no: " + wIntensityPropertyName );
		return 0;
	}
	int  volume = _Int( properties.find( wIntensityPropertyName )->second );

	if ( nocase_equal( source.c_str(), "Ambient" ) )
		return volume;
	//ambient volume doesn't fade

	if ( !volume)
		return 0;

	iMeshList     *meshes      = FROM_CS(iEngine)->GetMeshes();
	iMeshWrapper  *source_mesh = meshes->FindByName( source.c_str() );
	if ( source_mesh == NULL ){
		 LOG( "Field", 1, ( "Mesh " + source + " not found from engine!" ).c_str() );
		return 0;
	}
	//double sx,sy,sz;
	csVector3  spos = source_mesh->GetMovable()->GetPosition();
	float      dist = sqrt( ( spos - csVector3( x, y, z ) ).SquaredNorm() );

	int        perceived_volume = volume - ( int ) (volumeFadePerDistance *dist);

	 LOG( "Field", 4, "Volume: " + toString( perceived_volume ) );

	return perceived_volume;
}

//--------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
Smell::Smell (std::string  _source, int  _volume, int  _quality )
{
	Field::source        = _source;
	//Field::volume = _volume;
	Set( WINTENSITY, _volume  );
	Set( WQUALITY  , _quality );
}

//--------------------------------------------------------------------------------------------------------------
Smell::Smell(std::string  _source, XMLNode&  node )
{
	Field::source        = _source;

	try  {
		XMLVALIDATE( node, WSMELL, csPrintf( "Bad smell entry." ) );
		XMLITERATE ( node, property )  {
			string  nodename = (*property)->TagData().name;
			if ( nodename != WINTENSITY  &&  nodename != WQUALITY )  {
				 LOG( "Taste", 1, "Bad smell entry." );
			}
			else
				Set( nodename, ( *property )->TagData().textcontent );
		}
	}
	catch ( ... ){}
}

//--------------------------------------------------------------------------------------------------------------
WorldObjectProperty::WorldObjectProperty(){
	Set( WENERGY ,   0  );
	Set( WDENSITY, 0.0f );
}

//--------------------------------------------------------------------------------------------------------------
string WorldObjectProperty::AsXML() const  
{
	string indepependentNodes = RemoteObject::AsXML();
	string totalsound;

		for ( unsigned int s = 0; s < sound.size(); s++ )
			totalsound += sound[s].AsXML();

	return XMLembed( "Properties", indepependentNodes + totalsound + smell.AsXML() + taste.AsXML() );
}
