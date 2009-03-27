/*
 * opencog/embodiment/AGISimSim/server/src/sensor.cpp
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
 *  Sensor method implementations.
 *
 *  Project: Novamente
 *
 *  03/2006
 *  Copyright  2006  Ari A. Heljakka / GenMind Ltd
 *  Email [heljakka at iki dot fi]
 *
 *	FP	20.02.06	Reformatting
 *      AH      29.03.06        Converted to AGISimSim: PseudoSensor.
 ****************************************************************************/


#include "simcommon.h"
#include "simconfig.h"
#include "space.h"
#include "sensors.h"
#include "world_vocabulary.h"
#include "demon.h"
#if CRYSTAL
 #include "CSops.h"
#endif
#include "simworld.h"
#include "CSagent.h"
#include "simserver.h"

#ifndef NOWX
 #include <wx/image.h>
#endif

const nocase_string  Sensor::MODE    = "MODE";
const nocase_string  Sensor::POWERON = "POWERON";

//-----------------------------------------------------------------------------------------------------------------
map< iMeshWrapper*, string >  mesh_name_cache;

/*void Sensor::Set(string property, string value)
{
	properties[toupper(property)] = toupper(value);
}*/

//-----------------------------------------------------------------------------------------------------------------
void Sensor::ForceUpdate()
{
	elapsed_tics = (unsigned int)IntConfig( "TicsPerFrame" ) + 1;
	//	AllowUpdate();
}

//-----------------------------------------------------------------------------------------------------------------
Sensor::Sensor( Demon*  _demon, string  _sense_name, bool  updateOnFirstCall)
			  : sense_name( _sense_name ), elapsed_tics(0),	firstCall( updateOnFirstCall )
{
	demon = _demon;
	Set( POWERON, "1" );
	
	 LOG( sense_name, 3, "Initialized." );
	//		mutex->Create();
}

//-----------------------------------------------------------------------------------------------------------------
Sensor::~Sensor()
{
	 LOG( "Sensor", 3, "Destroying " + sense_name);
}

//-----------------------------------------------------------------------------------------------------------------
void Sensor::AddTics( unsigned long  ticks) 
{
	elapsed_tics += ticks; 
}

//-----------------------------------------------------------------------------------------------------------------
void Sensor::AllowUpdate()
{
	int   powerstate    = 0;
	bool  haspowerstate = Get( POWERON, powerstate );
	if ( haspowerstate  &&  powerstate == 0 ) {
		 LOG( sense_name, 4, "Turned off, returning..." );
		return;
	}
	
	try  {
		if (elapsed_tics > (unsigned int)IntConfig( "TicsPerFrame" )  ||  firstCall) {
			 LOG(sense_name, 3, "Updating..." );
			Update();
			 LOG(sense_name, 3, "Update ok..." );
			
			elapsed_tics = 0;			
			firstCall    = false;
		}
	} catch(string s) { LOG(sense_name, 0, "Exception in AllowUpdate: "+s ); }
	catch(...) { LOG(sense_name, 0, "Exception in AllowUpdate.\n" ); }
}

//-----------------------------------------------------------------------------------------------------------------
PseudoObjectVisionSensor::PseudoObjectVisionSensor(Demon* _demon, bool updateOnFirstCall)
: Sensor( _demon, "PseudoObjectVision", updateOnFirstCall)
{
}
PseudoObjectVisionSensor::~PseudoObjectVisionSensor() {}

void PseudoObjectVisionSensor::Update()
{
	nmap  objnames;
	map< string, FOVpos >  name2pos;
	std::string            senseData;
	
	w = SCREEN_W;
	h = SCREEN_H;

	double  __x, __y, __z, phi;

	if (!demon || !demon->getRepresentation())
		return;

	demon->getRepresentation()->getOrientation( __x, __y, __z, phi);
	demon->getRepresentation()->getPosition( __x, __y, __z );
	float  eye_phi = 0.0, eye_theta = 0.0;
	demon->Get( EYE_THETA, eye_theta);
	demon->Get( EYE_PHI  , eye_phi  );

	map< int, map<int, meshData> >  vobjs;

	float dphi = 3.14/2; //3.14*2

	 LOG( "PVU", 2, "GetVisibleObjects...");

	 float netsize = 0.05f; //0.01

/*	int x0 = Xcoord2index(__x);
	int y0 = Ycoord2index(__y);
	if (x0>=0 && y0>=0 && x0 < iEngine::discrete_w && y0 < iEngine::discrete_h && !iEngine::blocks[x0][y0].empty())
		vobjs[x0][y0] = meshData((*iEngine::blocks[x0][y0].begin())->GetMesh(),0,0);
*/
	for (float next_phi = phi + eye_phi-dphi/2; next_phi < phi+eye_phi+dphi/2; next_phi += netsize)
	{
		map<int, map<int, meshData> > o = iEngine::Instance().VisibleObjects(csVector3(__x,__y,__z), next_phi);

		if (!o.empty())
			vobjs[o.begin()->first][o.begin()->second.begin()->first] = o.begin()->second.begin()->second;

/*		for (map<int, map<int, meshData> >::iterator xi = o.begin(); xi != o.end(); xi++)
			for (map<int, meshData>::iterator y = xi->second.begin(); y != xi->second.end(); y++)
*/
				//objs.insert(y->second->QueryObject()->GetName());
	}
 
	map< int, map<int, float> >     bdata;

	 LOG( "PVU", 2, "GetVisibleObjects ok with "+toString(vobjs.size())+" elements." );
	
	 int  objtotal = 0;
	
try {
	for (map< int, map<int, meshData> >:: iterator  xi = vobjs.begin(); xi != vobjs.end(); xi++)  {
		for (map< int, meshData > ::iterator  yi = xi->second.begin(); yi != xi->second.end(); yi++)
		{
		  LOG("OVSensor", 4, "Y 1");
			iMeshWrapper*         mesh = yi->second.wrapper;	
			if (mesh)
			  {		  LOG("OVSensor", 3, "wrapper was NULL!"); continue; }		
			std::string  comboName,  meshname="null",  polygonname="null";
			try {
				map< iMeshWrapper*, string >:: iterator  mi	= mesh_name_cache.find( mesh );
				if ( mi != mesh_name_cache.end() )  meshname = mi->second;
				else  {
					meshname = std::string( mesh->QueryObject() ? mesh->QueryObject()->GetName() : "null" );
					//	+	std::string(psobj ? psobj->GetName () : "<null>") + "/"
					mesh_name_cache[mesh] = meshname;
				}
									
			} catch(...) { }

			comboName =	meshname;
			
			int   dist = (int)( yi->second.distance * 1000 );			
			//float bness = 0.0f;
			
#ifdef WIN32
	#define CREATE_SENSE_DATA(x,z,d,name)  senseData += XMLembed( "object", FOVpos(x-__x,z-__z,d).AsXML() + XMLembed( "name", name ) + XMLembed("brightness", toString(bdata[(int)x][(int)z]*100)) )
#else
	#define CREATE_SENSE_DATA(x,z,d,name)  \
					senseData += XMLembed( "object", \
						FOVpos(x-__x,z-__z,d).AsXML()         \
					+	XMLembed( "name", name )        \
					+ 	XMLembed( "brightness", toString((bdata[(int)x][(int)z]*100)) ) )			
						
#endif
			
			/*	// OLD IMPLEMENTATION:
			if (properties[MODE] == MULTI_COMPONENT)
			{
				CREATE_SENSE_DATA(xi->first, yi->first, dist, comboName);
				objtotal++;
			}*/
		  LOG("OVSensor", 4, "Y 2");
			if ( ( name2pos[comboName].dist < 0.001 * 1000 ||  //=doesn't exist
				      name2pos[comboName].dist > dist) )          //=is closer than previous edges
			{
				// Saves the shortest beam to each object (=> distance)				
				name2pos[comboName] = FOVpos( xi->first, yi->first, dist );				
				//	csPrintf( "%.1f... ", bdata[xi->first][yi->first]);
			}
		  LOG("OVSensor", 4, "Y 3");
		} //for yi
	} //for xi

} catch(exception e) { puts(e.what()); }
catch(string s) { puts(s.c_str()); }
catch(...) { LOG("OVSensor",1, "Exception in GVO"); }

	 LOG( "OVSensor", 2, "Creating single component data... name cache size was " + toString(mesh_name_cache.size()) );
		
	for (map< string, FOVpos >:: iterator  i = name2pos.begin(); i != name2pos.end(); i++)  {
		CREATE_SENSE_DATA(i->second.x, i->second.y, i->second.dist, i->first);

		/* csPrintf( "%s: ", i->first.c_str());
			csPrintf( "distance %.2f ", i->second.dist);
			csPrintf( "\n" );*/

		objtotal++;
	}
	 LOG( "PVSensor", 2, "Created. Adding the new sensation..." );		
		
	AddNewSensation( XMLembed( "objectvisual", senseData ) );

	 LOG( "PVSensor", 2, "Added the new sensation." );
}

//-----------------------------------------------------------------------------------------------------------------
MapInfoSensor::MapInfoSensor(Demon* _demon, bool updateOnFirstCall)
: Sensor( _demon, "MapInfo", updateOnFirstCall)
{
     LOG( "MapInfoSensor", 2, "Created");		
}
MapInfoSensor::~MapInfoSensor() {}

void MapInfoSensor::Update()
{
    std::string senseData;
    if (!demon || !demon->getRepresentation())
        return;

     LOG( "MapInfoSensor", 2, "Adding the new sensation..." );		

    iEngine*  engine = FROM_CS( iEngine );
    iMeshList* meshes = engine->GetMeshes();
    for (int i = 0; i < meshes->GetCount(); i++)  {
        iMeshWrapper*  mesh = meshes->Get(i);
        if ( mesh->QueryObject() )  {
            const char* objName = mesh->QueryObject()->GetName();
            std::string objData = XMLembed("name", objName); 
            objData += XMLembed("type", mesh->material); 
            objData += XMLembed("remove", "false"); 
            iMovable* movable = mesh->GetMovable();
            if (movable) {
                csVector3 pos = movable->GetPosition();
                objData += XMLembed("pos", toString(pos.x) + "," + toString(pos.y) + "," + toString(pos.z));
                csVector3 rot = movable->GetRotation();
                objData += XMLembed("rot", toString(rot.x) + "," + toString(rot.y) + "," + toString(rot.z));
            }
            csVector3 dim = mesh->dim;
            objData += XMLembed("dim", toString(dim.x) + "," + toString(dim.y) + "," + toString(dim.z));
            if ( STLhas( TheSimWorld.pmap, objName ) )  {
                WorldObjectProperty prop = TheSimWorld.pmap[objName];
                int energy, tasteQuality; 
                prop.Get(WENERGY, energy);
                prop.taste.Get(WQUALITY, tasteQuality);
                objData += XMLembed("edible", (energy > 0 && tasteQuality == 2)?"true":"false"); 
                objData += XMLembed("drinkable", (energy > 0 && tasteQuality == 3)?"true":"false"); 
            }
            
            std::string profile(objName);
            objData += XMLembed("petHome", (profile.find("petHome", 0) != std::string::npos)?"true":"false"); 
            objData += XMLembed("foodBowl", (profile.find("foodBowl", 0) != std::string::npos)?"true":"false"); 
            objData += XMLembed("waterBowl", (profile.find("waterBowl", 0) != std::string::npos)?"true":"false"); 

            senseData += XMLembed("object", objData);
        }
    }
    for (std::map<std::string, std::string>::const_iterator it = removedObjects.begin(); it != removedObjects.end(); it++) {
        std::string objData = XMLembed("name", it->first.c_str()); 
        objData += XMLembed("type", it->second.c_str()); 
        objData += XMLembed("remove", "true"); 
        senseData += XMLembed("object", objData);
	printf("senseData = %s\n", senseData.c_str());
    }
    removedObjects.clear();

    AddNewSensation( XMLembed( "map-info", senseData ) );

     LOG( "MapInfoSensor", 2, "Added the new sensation." );
}

void MapInfoSensor::objectRemoved(const std::string& objName, const std::string& objType) {
    removedObjects[objName] = objType;
}


#if CRYSTAL
//-----------------------------------------------------------------------------------------------------------------
	/* - Scans the whole camera view by throwing a net of "beams"
	   - Gets all names of all objects hit by the beams
	   - Calculates the length of each beam to the object	*/
//-----------------------------------------------------------------------------------------------------------------
map< int, map<int, meshData> >  GetVisibleObjects( csRef<iCamera>  cam, int  w, int  h, int  netsize )
{
	return GetVisibleObjects( cam, 0, 0, w-1, h-1, netsize );
}

//-----------------------------------------------------------------------------------------------------------------
map<int, map<int, meshData> >  GetVisibleObjects( csRef< iCamera >  cam, const int  sx, const int  sy, const int  ex, 
							 					  const int ey, const int  netsize )
{
	map< int, map<int, meshData> >  ret;
	int  dx = netsize,  dy = netsize;

	for (int  x = sx; x <= ex; x += dx)  {
		for (int  y = sy; y < ey; y += dy)
		{
			meshData  data;
			data.index   = 0;
			data.wrapper = LocateObject( cam, x, y, &data.index, data.intersect );

			if (data.wrapper)  {
				float  dist = ( cam->GetTransform().GetOrigin() - data.intersect ).Norm();
				
				if (dist > 0.001)  {// To avoid strange errors				
					data.distance = dist;
					ret[x][y]     = data;
				}
			}				
		}
	}
	return ret;
}

//-----------------------------------------------------------------------------------------------------------------
float GetBrightness( csVector3  point, iLightList*  ll)
{
	float  max_bness = 0.0f;
	
	for (int  i = 0; i < ll->GetCount(); i++)  {
		csRef< iLight >  light = ll->Get(i);
		
		if ( light )  {
			csVector3  origo = light->GetCenter();
			float      dist  = (origo-point).Norm();
			float      bness = light->GetBrightnessAtDistance(dist); 

			if ( bness > max_bness )  max_bness = bness;
		}
	}
	
	return max_bness;
}

//-----------------------------------------------------------------------------------------------------------------
iMeshWrapper*  LocateObject ( iCamera*  camera, int x, int y, int* poly, csVector3&  intersect)
{
	// Setup a 2D vector with our mouse position.  We invert the y (based on vertical screen dimension) because 
	// CS assumes y=0 is down for 3D calculations.
	csVector2  v2d (x, camera->GetShiftY () * 2 - y);
	
	// We calculate the inverse perspective of this 2D point at z=100.  This results in a 3D position in camera space at
	// z=100 that directly corresponds to the 2D position we clicked on.  We use z=500 to ensure that we will at least
	// hit all objects that are before that distance.
	csVector3  v3d;
	camera->InvPerspective ( v2d, 500, v3d );
	
	// We are going to cast a beam in the current sector of the camera from our camera position in the direction of the
	// 'v3d' point.  First we transform the v3d camera space location to world space.
	csVector3  startbeam = camera->GetTransform ().GetOrigin ();
	csVector3  endbeam   = csVector3( camera->GetTransform ().This2Other( v3d ) );
	
	// Now do the actual intersection.
	*poly = -1;
	iSector*       beamsector = camera->GetSector ();
	iMeshWrapper*  mesh       = beamsector->HitBeamPortals( startbeam, endbeam, intersect, poly );

	return mesh;
}

//-----------------------------------------------------------------------------------------------------------------
// Use sign +1 or -1, for enabling and disabling the sensor, respectively.

/*void PrepareViewForVisionSensor(csRef<iView> view,int sign)
{
		csVector3 eyeSensorCorrection(
			sign*0,
			sign*1.0,
			sign*0); //agentR+0.05);

#warning "Single agent solution"
	
		iCamera* c = view->GetCamera();
	
		simplecoord sc;
		Obj3D* obj = SimServer::Get()->GetAgentBody();
	  	obj->getPosition(sc.x, sc.y, sc.z);
	  
      	csVector3 cam_pos(sc.x, sc.y, sc.z);
		cam_pos += eyeSensorCorrection;
		
		csOrthoTransform ot = c->GetTransform();
		ot.SetOrigin(cam_pos);
		c->SetTransform(ot);
}*/


/* O.V. Sensor */

//-----------------------------------------------------------------------------------------------------------------
const nocase_string  ObjectVisionSensor::SINGLE_COMPONENT = "SINGLE_COMPONENT";
const nocase_string  ObjectVisionSensor::MULTI_COMPONENT  = "MULTI_COMPONENT";

//-----------------------------------------------------------------------------------------------------------------
ObjectVisionSensor::ObjectVisionSensor ( Demon*  _demon,	csRef< iGraphics2D >  _myG2D, csRef< iView >  _view, 
										/*OVUmode _mode,*/   	 int _netsize, bool updateOnFirstCall )
										: Sensor( _demon, "ObjectVision", updateOnFirstCall ), view( _view ),
												 myG2D( _myG2D ), netsize( _netsize )
{
	//	Set(MODE, SINGLE_COMPONENT);
	Set( MODE, MULTI_COMPONENT );
}

//-----------------------------------------------------------------------------------------------------------------
void ObjectVisionSensor::Update()
{
	 LOG( "ObjectVisionSensor", 0, "WARNING! HACK: Forcing MULTI_COMPONENT mode!" );
#ifdef WIN32
	#pragma message ("WARNING! HACK: Forcing MULTI_COMPONENT mode!")
#else
	#warning "WARNING! HACK: Forcing MULTI_COMPONENT mode!"
#endif
	
	properties[MODE] = MULTI_COMPONENT;
	
	nmap  objnames;
	map< string, FOVpos >  name2pos;
	std::string            senseData;
	
	if ( properties[MODE] != MULTI_COMPONENT	&& 
		 properties[MODE] != SINGLE_COMPONENT )
	{
		 LOG( "ObjectVisionSensor", 1, "Unknown mode!" );
		return;
	}
	
	if ( !myG2D.IsValid()  ||  !view.IsValid() ) {
		 csPrintf( "G2D or iView was null!\n" );
		return;
	}

	//	csRef<iImage> im = myG2D->ScreenShot();
	w = myG2D->GetWidth();
	h = myG2D->GetHeight();

	LOG( "OVU", 2, "GetVisibleObjects with netsize = " + toString(netsize));
	
	//	PrepareViewForVisionSensor(view, +1);
	map< int, map<int, meshData> >  vobjs = GetVisibleObjects( view->GetCamera(), w, h, netsize );
	//	PrepareViewForVisionSensor(view, -1);	
	map< int, map<int, float> >     bdata;

	 LOG( "OVU", 2, "GetVisibleObjects ok with "+toString(vobjs.size())+" elements." );
	
	int  objtotal = 0;
	
	map< pair<string,string>, vector<csVector3> >  polygons;
	
	int  use_polygonnames = IntConfig( "UsePolygonNameInObjectVision" );
	
	for (map< int, map<int, meshData> >:: iterator  xi = vobjs.begin(); xi != vobjs.end(); xi++)  {
		for (map< int, meshData > ::iterator  yi = xi->second.begin(); yi != xi->second.end(); yi++)  {
			iMeshWrapper*         mesh = yi->second.wrapper;			
			csRef< iThingState >  objState;
			if ( use_polygonnames )  {
				objState = SCF_QUERY_INTERFACE( mesh->GetMeshObject(), iThingState );
				 LOG( "PolygonVision", 2, "Using polygonnames!" );
			}

			std::string  comboName,  meshname="null",  polygonname="null";
			try {
				map< iMeshWrapper*, string >:: iterator  mi	= mesh_name_cache.find( mesh );
				if ( mi != mesh_name_cache.end() )  meshname = mi->second;
				else  {
					meshname = std::string( mesh->QueryObject() ? mesh->QueryObject()->GetName() : "null" );
					//	+	std::string(psobj ? psobj->GetName () : "<null>") + "/"
					mesh_name_cache[mesh] = meshname;
				}
				
				 LOG( "PolygonVision", 2, "Mesh: " + meshname);
				
				csRef<iThingFactoryState> tfs;
				
				if (use_polygonnames)
				{
					tfs = scfQueryInterface<iThingFactoryState>(
						mesh->GetMeshObject()->GetFactory());
					
					polygonname = std::string( tfs
						? tfs->GetPolygonName(yi->second.index) : "null" );
				}

				 LOG( "PolygonVision", 2, "Polygon: " + polygonname);
						
				if ( !objState )  { LOG( "PolygonVision", 3, "objState NULL" ); }	
				
				pair< string,string >  pnamepair( meshname, polygonname );
				
				if ( objState  &&  tfs  &&  !STLhas( polygons, pnamepair ) )  {
					int   vsc = tfs->GetPolygonVertexCount  ( yi->second.index );
					int*  vs  = tfs->GetPolygonVertexIndices( yi->second.index );
					
					vector< csVector3 >  vset;
					for (int  v=0; v < vsc; v++)  {
						const csVector3&  pv = tfs->GetPolygonVertex( yi->second.index, vs[v] );
						 LOG( "PolygonVision", 2, "X: " + toString(pv.x) + " Y: " + toString(pv.y) + " Z: " + toString(pv.z) );
						vset.push_back( csVector3(pv) );
					}
					polygons[pnamepair] = vset;
				}
				else { LOG( "PolygonVision", 2, "objState && objState->GetFactory() NULL" ); }
			} catch(...) { }

			//continue;			
			comboName =	meshname;
			
			if ( use_polygonnames )  comboName += "/" + polygonname;

			int   dist = (int)( yi->second.distance * 1000 );			
			//LOG( "Sensor", 4, "Object "+comboName + " @dist="+toString(dist));
			float bness = 0.0f;

			if ( IntConfig("SenseBrightnessInObjectVision") )  {
				bness = GetBrightness( yi->second.intersect, view->GetCamera()->GetSector()->GetLights() );
				bdata[xi->first][yi->first] = bness;
			}
			
#ifdef WIN32
	#define CREATE_SENSE_DATA(x,y,d,name)  senseData += XMLembed( "object", FOVpos(x,y,d).AsXML() + XMLembed( "name", name ) + XMLembed("brightness", toString(bdata[(int)x][(int)y]*100)) )
#else
	#define CREATE_SENSE_DATA(x,y,d,name)  \
					senseData += XMLembed( "object", \
						FOVpos(x,y,d).AsXML()         \
					+	XMLembed( "name", name )        \
					+ 	XMLembed( "brightness", toString(bdata[(int)x][(int)y]*100) ) )			
						
#endif
			
			/*	// OLD IMPLEMENTATION:
			if (properties[MODE] == MULTI_COMPONENT)
			{
				CREATE_SENSE_DATA(xi->first, yi->first, dist, comboName);
				objtotal++;
			}*/

			if ( properties[MODE] == SINGLE_COMPONENT  && 
					( name2pos[comboName].dist < 0.001 * 1000 ||  //=doesn't exist
				      name2pos[comboName].dist > dist) )          //=is closer than previous edges
			{
				// Saves the shortest beam to each object (=> distance)				
				name2pos[comboName] = FOVpos( xi->first, yi->first, dist );				
				//	csPrintf( "%.1f... ", bdata[xi->first][yi->first]);
			}
		} //for yi
	} //for xi

	if ( properties[MODE] == MULTI_COMPONENT )  {
		for (map< pair<string,string>, vector<csVector3> >::iterator  pi = polygons.begin(); pi != polygons.end(); pi++)  {
			string  pvs;			
			 LOG( "PolygonVision", 2, "Vertex vector size: " + toString(pi->second.size()));

			for ( vector< csVector3 >:: iterator  pv = pi->second.begin(); pv != pi->second.end(); pv++)
			{
#ifdef WIN32
				#pragma message ("These coordinates are in WORLD system, not relative to agent!")
#else
				#warning "These coordinates are in WORLD system, not relative to agent!"
#endif				
				pvs += XMLembed( "corner", "(" + toString(pv->x) + "," + toString(pv->y) + "," + toString(pv->z) + ")" );				
				 LOG( "PolygonVision", 2, "X: "+toString(pv->x)+" Y: "+toString(pv->y)+" Z: "+toString(pv->z));
			}
			
			senseData += XMLembed("object", XMLembed( "name", pi->first.first ) + XMLembed( "polygonname", pi->first.second ) \
											+ 	XMLembed("brightness", "0" ) + 	pvs	);
		}
	}		
		
	 LOG( "OVSensor", 2, "Creating single component data... name cache size was " + toString(mesh_name_cache.size()) );
		
	if ( properties[MODE] == SINGLE_COMPONENT )  {
		for (map< string, FOVpos >:: iterator  i = name2pos.begin(); i != name2pos.end(); i++)  {
			CREATE_SENSE_DATA(i->second.x, i->second.y, i->second.dist, i->first);

			/* csPrintf( "%s: ", i->first.c_str());
			csPrintf( "distance %.2f ", i->second.dist);
			csPrintf( "\n" );*/

			objtotal++;
		}
	}
	 LOG( "OVSensor", 2, "Created. Adding the new sensation..." );		
		
	AddNewSensation( XMLembed( "objectvisual", senseData ) );
	///	f("%d items pushed to sensation Queue by ObjectVision.\n",objtotal);
}

//-----------------------------------------------------------------------------------------------------------------
/*	void ObjectVisionSensor::rprint(csRef<iMeshList> objs)
	{
		if (objs)
		{
			int n = objs->GetCount();
			csPrintf( "%d Objects\n", n);

			for (int i = 0; i < n; i++)
			{
				csRef<iMeshWrapper> mesh = objs->Get(i);

				iMeshList *children = mesh->GetChildren();

				if (children && children->GetCount() > 0)
					rprint(children);

				csRef<iObject> obj = mesh->QueryObject();
				csPrintf( "%s\n", obj->GetName());
			}
		}
	}*/


/* V. Sensor */

//-----------------------------------------------------------------------------------------------------------------
const nocase_string VisionSensor::RAW         = "RAW";
const nocase_string VisionSensor::MEAN        = "MEAN";
const nocase_string VisionSensor::REMOTE      = "REMOTE";
const nocase_string VisionSensor::SCALEFACTOR = "SCALEFACTOR";

//-----------------------------------------------------------------------------------------------------------------
VisionSensor::VisionSensor( Demon*  _demon, csRef< iGraphics2D >  _myG2D, csRef< iView >  _view, int _netsize, 
						    bool updateOnFirstCall)
						  : Sensor( _demon, "PixelVision", updateOnFirstCall ), view( _view ),	netsize( _netsize )
//                            ,myG2D( _myG2D )
{
	//properties[MODE] = RAW;
	properties[MODE]        = MEAN;	
	properties[SCALEFACTOR] = "1";
	properties[REMOTE]      = "1";
}

//-----------------------------------------------------------------------------------------------------------------
#ifdef WIN32
	#include <sys/timeb.h>
#else
	#include <sys/time.h>
#endif

//-----------------------------------------------------------------------------------------------------------------
void VisionSensor::Update()
{
	//	csPrintf( "---\nImage load. Ticking...\n" );	
	if ( !myG2D.IsValid() )  {
		csPrintf( "G2D was null!\n" );
		return;
	}
	
	 LOG( "VisionSensor", 3, "ScreenShooting..." );

	csRef< iImage >  im = myG2D->ScreenShot();
	/*csRef<csImageMemory> imme = csPtr<csImageMemory>(new csImageMemory(im));
	  imme->SetFormat(CS_IMGFMT_JPEG);*/
	
	//int nw = 640/4, nh = 480/4;
	//im = csImageManipulate::Rescale(im, nw, nh);

	//    struct timeval tv1;
	//    struct timezone tz;
	//    gettimeofday(&tv1, &tz);
	im = csImageManipulate::Mipmap(im, atoi(properties[SCALEFACTOR].c_str()));

	//    struct timeval tv2;
	//    gettimeofday(&tv2, &tz);
	//    printf("RESCALE TIME => %lu\n", (tv2.tv_sec-tv1.tv_sec)*1000 + (tv2.tv_usec-tv1.tv_usec)/1000);
	
	w = im->GetWidth();
	h = im->GetHeight();	
	 LOG( "VisionSensor", 3, "Shot ok. Scaled to " + properties[SCALEFACTOR]);

	/*	int nw = w, nh = h;
	csImageMemory* imme = new csImageMemory(nw, nh, im->GetFormat());
	//imme->SetFormat();
	imme->Copy(im, 0,0, nw, nh);
	
	LOG( "VS",0, toString(((unsigned char*)imme->GetImageData())[0]));
	
	//	w = nw;
	//	h = nh;
	
	im = csPtr<iImage>(imme);
	w = im->GetWidth();
	h = im->GetHeight();
	*/
	//	PrepareViewForVisionSensor(view, -1);		
	//	csPrintf( "Size: %d x %d\n", w, h);
	//	csPrintf( "Format: %d\n", im->GetFormat());
	
	if ( firstCall ) {
		bufferIm = im;
		OnChange( im, w, h );
	}
	else if ( Changed( im, w, h, bufferIm, bufferW, bufferH ) )  {
		OnChange( im, w, h );
		bufferIm = im;
		bufferW  = w;
		bufferH  = h;
	}
	
	firstCall = false;
}

//-----------------------------------------------------------------------------------------------------------------
//#include <direct.h>
#ifdef WIN32
//????????????????
#else
	#include <unistd.h>
#endif

//-----------------------------------------------------------------------------------------------------------------
void VisionSensor::OnChange( csRef< iImage >  im, uint  w, uint  h)
{
	 LOG( "VisionSensor", 2, "MODE: " + properties[MODE]);
	
	if ( properties[MODE] == RAW )  {
		char  pathbuf[1000];
		getcwd(pathbuf, 999);
		std::string  pixelpath = pathbuf + string("/pixels.dat" );
		//	std::string distpath = "dist.dat";
		
		 LOG( "VisionSensor", 2, std::string("Changed. Saving to file ") + pixelpath );		
		std::string  distData;
		
		if ( IntConfig( "SenseDistanceInPixelVision" ) )  GetDistanceData(distData,w,h);
		
		//		DistanceSave(distpath,w,h);		
		string  dataXML;
		
		if ( !atoi( properties[REMOTE].c_str() ) )  {
			Save( im, w, h, pixelpath );
			 LOG( "VisionSensor", 2, "Image saved to " + pixelpath + " as "	+ toString(w) + "x" + toString(h));
			ClearCache();  //The changes in pixeldata aren't visible in XML.
			dataXML = XMLembed( "rawdatafile", pixelpath );
		}
		else
		{
			 LOG( "VisionSensor", 2, "Image UU-encoding..." );
			shared_array<unsigned char> data = AsciiImage(im,w,h);
			 LOG( "VisionSensor", 3, "UU data retrieved." );
			 LOG( "VisionSensor", 3, "Adding " + toString(strlen( (char*)data.get() )) + " while " + toString( dataXML.max_size() ) + " is max." );
			dataXML = "<rawdata>";

            string  imageData = string( (char*)data.get() ); 
            //printf("********** imageData(size=%d,strlen(c_str)=%d)\n", imageData.size(), strlen(imageData.c_str()));
			dataXML += imageData;
			dataXML += "</rawdata>";
			
			/*	for (int n = 0; n < dataXML.size(); n++)
				if (dataXML[n] >= 0 && dataXML[n] < 31)
				{
					LOG( "VisionSensor", 0, "Bad char found: " + toString(dataXML[n]) + " @"+toString(n));
					//return;
				}
                else 
                	dataXML[n] = '0';
            dataXML = XMLembed("rawdata", dataXML);		*/
		
			 LOG( "VisionSensor", 3, "Tag size = " + toString(dataXML.size()));
		}
		
		AddNewSensation(XMLembed("visual",
			XMLembed( "distances", distData )
			+ dataXML
			+ XMLembed( "w", toString(w) )
			+ XMLembed( "h", toString(h) )
			) );
	}
	else if ( properties[MODE] == MEAN )  {
		std::string  distData;

		 LOG( "VisionSensor", 3, "Getting distance & color data.." );
		GetDistanceColorData( im, distData, w, h );	
		 LOG( "VisionSensor", 3, "Adding sensation..." );
		
		AddNewSensation( XMLembed( "visual",distData ) );
	}
}

//-----------------------------------------------------------------------------------------------------------------
bool VisionSensor::Changed( csRef< iImage >  im1, uint  w1, uint  h1, csRef< iImage >  im2, uint  w2, uint  h2)
{
	if ( w1 != w2  ||  h1 != h2 )  return true;
	
	if ( CS_IMGFMT_TRUECOLOR != im1->GetFormat()  ||  CS_IMGFMT_TRUECOLOR != im2->GetFormat() )  {
		 csPrintf( "ScreenShot format not supported... Try to get TrueColor.\n" );
		return false;
	}
	
	const csRGBpixel*  buf1 = static_cast<const csRGBpixel*>( im1->GetImageData() );
	const csRGBpixel*  buf2 = static_cast<const csRGBpixel*>( im2->GetImageData() );

	if ( buf1 == NULL  ||  buf2 ==NULL )  {
		 csPrintf( "Image Data wasn't available!\n" );
		return false;
	}
	
	int size = w1 * h1;
	for (long i = 0; i < size; i++)	{
		if ( buf1[i] != buf2[i] )  return true;
	}
	
	return false;
}

//-----------------------------------------------------------------------------------------------------------------
int Min( int a, int b)
{
	if ( a > b )  return b;

	return a;
}

//-----------------------------------------------------------------------------------------------------------------
/** UU coding produces tag characters which confuse
	the XML parser, so we modify them!     */
//----------------------------------------------------------------------------------------------------------------- 
int  asciicode( unsigned char*  bufin, unsigned int  nbytes, unsigned char*  bufout)
{	
	// The first bytes until ':' give the size. Size is given w/o the changed characters in the end of this whole block!
    sprintf( (char*)bufout, "%d", nbytes );
    int  offset      = strlen( (char*)bufout ); 
	bufout[offset++] = ':';

	memcpy( bufout + offset, bufin, nbytes);
	long size = nbytes + offset;
	
	// Change all problem characters to #148 and add them to the end in the right order
#ifdef WIN32
	for (long i = offset; i < (int)( nbytes + offset ); i++)
#else
	for (long i = offset; i < nbytes + offset; i++)
#endif
	{
		if ( bufout[i] == 31  ||  bufout[i] == '<'  ||  bufout[i] < 32 )  {
			bufout[size++] = bufout[i] + 63; //63 passes over '<' neatly.
			bufout[i]      = 31;
		}
	}
	bufout[size] = 0;
    return size;
}

//-----------------------------------------------------------------------------------------------------------------
#ifndef NOWX
  #define JPEG_COMPRESSION
#else
  #define ZLIB_COMPRESSION
#endif

#ifdef ZLIB_COMPRESSION
    #include "zlib.h"
#else
    #ifdef JPEG_COMPRESSION
//        #include <wx/wx.h>
        #include <wx/mstream.h>
    #endif
#endif

int seq_bmp_number = 0;

//-----------------------------------------------------------------------------------------------------------------
int encodeData( unsigned char*  bufin, unsigned int  nbytes, unsigned char*  bufout) 
{
     //printf("********** rawData size= %d\n", nbytes);
    int result;

#ifdef ZLIB_COMPRESSION
    uLongf         compressedSize = nbytes;
    unsigned char* bufcompress    = (unsigned char*)malloc( compressedSize * sizeof(char) );
    int            compressResult = compress2( (Bytef*) bufcompress, &compressedSize, (Bytef*)bufin, nbytes, Z_BEST_SPEED); //Z_BEST_COMPRESSION
    if ( compressResult == Z_MEM_ERROR )  {
         printf( "Error compressing image data: Z_MEM_ERROR\n" );
        exit(-1);
    } else if ( compressResult == Z_BUF_ERROR ) {
         printf( "Error compressing image data: Z_BUF_ERROR\n" );
        exit(-1);
    }
     //printf("********** Compressed size = %ld\n", compressedSize);
    result = asciicode( bufcompress, compressedSize, bufout );
    free(bufcompress);
#else
    result = asciicode( bufin, nbytes, bufout );
#endif        
     //printf("********** encoded size = %d\n", result);
    return result;
}

//-----------------------------------------------------------------------------------------------------------------
#include "csgfx/packrgb.h"
 
//-----------------------------------------------------------------------------------------------------------------
shared_array<unsigned char>  VisionSensor::AsciiImage( csRef< iImage >  im, uint  w, uint  h) const
{
	if ( CS_IMGFMT_TRUECOLOR == im->GetFormat() )  {
		const csRGBpixel*  buf1 = static_cast<const csRGBpixel*>( im->GetImageData() );
        unsigned char*     buf  = (unsigned char*)csPackRGB::PackRGBpixelToRGB( buf1, w*h );
		if ( buf )  {
            int  bufsize = w * h * 3;

			//    struct timeval tv1;
			//    struct timezone tz;
			//    gettimeofday(&tv1, &tz);
			 //    printf("STARTED ENCODING/COMPRESSION => %lu\n", tv1.tv_sec*1000 + tv1.tv_usec/1000);

#if defined(JPEG_COMPRESSION) && !defined(ZLIB_COMPRESSION)
             //printf("**** Creating new wxImage: width = %d, height = %d\n", w, h);
            unsigned char*  currentPixel = buf;
            wxImage*        newImage     = new wxImage( w, h );

            for (unsigned int  y = 0; y < h; y++) {
                for (unsigned int  x = 0; x < w; x++) {
                    newImage->SetRGB( x, y, *currentPixel, *(currentPixel+1), *(currentPixel+2) );
                    currentPixel += 3;
                }
            }
            // Set the quality for JPEG compression according with config variable.
            wxString qualityOption((wxChar*)"quality");
            wxString qualityValue((wxChar*)StringConfig("ImageQualityOption").c_str());
            newImage->SetOption(qualityOption, qualityValue);
            qualityValue = newImage->GetOption(qualityOption);
            //printf("IMAGE QUALITY = %s\n", qualityValue.c_str()); 

            //printf("**** Saving jpeg format of the new image to buffer, bufSize = %d\n", bufsize);
            char*                  jpegBuf     = (char*) malloc( bufsize * sizeof(char) );
            wxMemoryOutputStream*  out         = new wxMemoryOutputStream( jpegBuf, bufsize );

            newImage->SaveFile( *out, wxBITMAP_TYPE_JPEG );
            delete newImage;

            int                    jpegBufSize = out->TellO();
             //printf("**** Calling encodeData on the jpeg buffer, jpegBufSize = %d\n", jpegBufSize);
            unsigned char*         bigbuf      = new unsigned char[2 * jpegBufSize + 10];
            int                    bigbufsize  = encodeData( (unsigned char*)jpegBuf, jpegBufSize, bigbuf ); 
            delete out;
            free( jpegBuf );
#else
			unsigned char*  bigbuf     = new unsigned char[2 * bufsize + 10];
            int             bigbufsize = encodeData( buf, bufsize, bigbuf ); 
#endif            
            delete[] buf;

			//    struct timeval tv2;
			//    gettimeofday(&tv2, &tz);
			//    printf("ENCODING/COMPRESSION TIME => %lu\n", (tv2.tv_sec-tv1.tv_sec)*1000 + (tv2.tv_usec-tv1.tv_usec)/1000);
    
			 LOG( "VisionSensor", 3, "Image size: " + toString(bufsize) );
             LOG( "VisionSensor", 3, "UU buffer size: " + toString(bigbufsize) );

			shared_array<unsigned char> result(bigbuf, checked_array_deleter<unsigned char>());
			return result;
		}
		else
			 csPrintf( "Image Data wasn't available!\n" );
	}
	else
		 csPrintf( "The ScreenShot format not supported... Try to get TrueColor.\n" );
	
	shared_array< unsigned char >  result;
	return result;
}

//-----------------------------------------------------------------------------------------------------------------
bool  VisionSensor::Save( csRef< iImage >  im, uint  w, uint  h, std::string  path) const
{
	if ( CS_IMGFMT_TRUECOLOR == im->GetFormat() )  {
		//im->SetFormat(CS_IMGFMT_JPEG);
		const csRGBpixel*  buf = static_cast<const csRGBpixel*>( im->GetImageData() );
		
		if ( buf )  {
			int  bufsize = w * h *sizeof(csRGBpixel);
			//	 csPrintf( "Left Top (blue): %X, %X, %X, ...",
			//	buf[0].blue,buf[1].blue,buf[2].blue);
			FILE*  f = fopen(path.c_str(), "wb" );
			fwrite( buf, bufsize, 1, f );
			fclose( f );			
			 LOG( "VisionSensor", 3, "Image size: " + toString(bufsize) );

			return true;
		}
		else
			 csPrintf( "Image Data wasn't available!\n" );
	}
	else
		 csPrintf( "The ScreenShot format not supported... Try to get TrueColor.\n" );
	
	return false;
}

//-----------------------------------------------------------------------------------------------------------------
bool  VisionSensor::DistanceSave( std::string  path, uint  w, uint  h) const
{
	FILE*  f = fopen( path.c_str(), "wb" );
	if ( !f )  return false;
	
	map< int, map<int, meshData> >  vobjs = GetVisibleObjects( view->GetCamera(), w, h, netsize );

	std::string  senseData;
	
	for (map< int, map<int, meshData> >:: iterator  xi = vobjs.begin(); xi != vobjs.end(); xi++)  {
		for (map<int, meshData>::iterator yi = xi->second.begin(); yi!=xi->second.end(); yi++)  {
			fprintf(f, "%d %d %d\n", xi->first, yi->first, (int)(1000*yi->second.distance));
		}
	}
	fclose( f );

	return true;
}

//-----------------------------------------------------------------------------------------------------------------
bool  VisionSensor::GetDistanceData( std::string&  senseData, uint  w, uint  h) const
{
	map< int, map<int, meshData> >  vobjs = GetVisibleObjects( view->GetCamera(), w, h, netsize );

	std::string  senseDistData;

	for (map< int, map<int, meshData> >:: iterator  xi = vobjs.begin(); xi != vobjs.end(); xi++) {
		for (map< int, meshData>:: iterator  yi = xi->second.begin(); yi != xi->second.end(); yi++)  {
			senseDistData += FOVpos( xi->first, yi->first, 1000 * ( (int)yi->second.distance) ).AsXML();
		}
	}

	senseData += XMLembed( "distances", senseDistData );

	return true;
}

//-----------------------------------------------------------------------------------------------------------------
csRGBpixel  AverageColor( const csRGBpixel*  buf, int x, int y, int size, int w)
{
	int blue = 0,  red = 0,  green = 0;

	for ( int  xi = x; xi < x + size; xi++)  {
		for (int yi = y; yi < y+size; yi++)  {
			red   += buf[xi + yi*w].red;
			green += buf[xi + yi*w].green;
			blue  += buf[xi + yi*w].blue;
		}
	}

	return  csRGBpixel( red / size*size, green / size*size, blue / size*size );
}

//-----------------------------------------------------------------------------------------------------------------
bool  VisionSensor::GetDistanceColorData( csRef< iImage >  im, std::string&  senseData, uint  w, uint  h) const
{
	if ( CS_IMGFMT_TRUECOLOR  ==  im->GetFormat() )  {
		 LOG( "VisionSensor", 3, "Getting color data.." );		
		const csRGBpixel*  buf = static_cast<const csRGBpixel*>( im->GetImageData() );
		
		if ( buf ) {
			//  csPrintf( "Left Top (blue): %X, %X, %X, ...",
			// buf[0].blue,buf[1].blue,buf[2].blue);
			 LOG( "VisionSensor", 3, "Getting distance data.." );
			map< int, map<int, meshData> >  vobjs = GetVisibleObjects( view->GetCamera(), w, h, netsize );
			std::string  sensePixData;

			 LOG( "VisionSensor", 3, "Combining data.." );
			
			for (map<int, map<int, meshData> >:: iterator  xi = vobjs.begin(); xi != vobjs.end(); xi++)  {
				for (map<int, meshData>::iterator yi = xi->second.begin(); yi!=xi->second.end(); yi++)  {
					csRGBpixel  pix = AverageColor( buf, xi->first, yi->first, netsize, w );

					sensePixData += XMLembed( "pixel",
										XMLembed( "color", 
											XMLembed( "r", toString(pix.red) )
									  + XMLembed( "g", toString(pix.green) )
									  + XMLembed( "b", toString(pix.blue) )
										)
										+FOVpos(xi->first, yi->first, (int)(1000*yi->second.distance)).AsXML()
									);
				}
			 }
			senseData = sensePixData;

			return true;
		}
		else  csPrintf( "Image Data wasn't available!\n" );
	}
	else  csPrintf( "The ScreenShot format not supported... Try to get TrueColor.\n" );

	return true;
}

//-----------------------------------------------------------------------------------------------------------------
VisionSensor::~VisionSensor() {}

ObjectVisionSensor::~ObjectVisionSensor() {}

#endif


//-----------------------------------------------------------------------------------------------------------------
void  Sensor::ClearCache()
{
	old_sense_data = "";
}
	
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
void  Sensor::AddNewSensation( std::string  new_sense_data)
{
	if ( old_sense_data == new_sense_data )  {
        //std::cout << "Sensor::AddNewSensation(): skiping because old_sense_data == new_sense_data" << std::endl;
        return;
    }

	demon->sensationQueue.push( new_sense_data );
	
	char  temp[500];
	 sprintf( temp, "%.1f kb data pushed to sensation queue by %s.\n", new_sense_data.size() / 1024.0f, sense_name.c_str() );
	 LOG( "Sensor", 3, temp );
	
	old_sense_data = new_sense_data;
}

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
SmellSensor::SmellSensor( Demon*  _demon, bool  updateOnFirstCall)
						: Sensor( _demon, "SmellSense", updateOnFirstCall )
{
}

//-----------------------------------------------------------------------------------------------------------------
SmellSensor::~SmellSensor()
{
}

//-----------------------------------------------------------------------------------------------------------------
void SmellSensor::Update() 
{
	std::string  senseData;
	Obj3D*       agentbody = demon->getRepresentation();
	
    int volumeFadePerDistance = IntConfig( "VolumeFadePerDistance" );	
    std::string wIntensityPropertyName = toupper(WINTENSITY);
	for (PropertyMap::iterator  i = TheSimWorld.pmap.begin(); i != TheSimWorld.pmap.end(); i++)  {
		double  x, y, z;
		agentbody->getPosition( x, y ,z );
		
		WorldObjectProperty&  p = i->second;		
		int  volume = p.smell.VolumeAt( x, y, z, volumeFadePerDistance, wIntensityPropertyName);
		
		if ( volume > 0 )  {
			//Note: we can't take p.smell.AsXML() because that'd use the object's internal volume - not the phenomenal volume!
			int  quality;
			p.smell.Get( WQUALITY, quality );
			senseData += Smell(	volume,quality ).AsXML();
		}
	}

	AddNewSensation(XMLembed("smelldata", senseData));
}

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
TasteSensor::TasteSensor( Demon*  _demon, bool  updateOnFirstCall )
						: Sensor( _demon, "TasteSense",updateOnFirstCall )
{
}

//-----------------------------------------------------------------------------------------------------------------
TasteSensor::~TasteSensor()
{
}

//-----------------------------------------------------------------------------------------------------------------
void TasteSensor::OnTaste( Taste taste )
{
	tastes.push( taste );
}

//-----------------------------------------------------------------------------------------------------------------
void TasteSensor::Update()
{
	std::string  senseData;
	
	while( !tastes.empty() )  {
		Taste  taste = tastes.front();		
		senseData += taste.AsXML();
				
		tastes.pop();
	}	
	
	AddNewSensation( XMLembed( "tastedata", senseData ) );
}

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
TouchSensor::TouchSensor( CSAgent*  _demon, bool  updateOnFirstCall )
						: Sensor( _demon, "TouchSense", updateOnFirstCall )
{
}

//-----------------------------------------------------------------------------------------------------------------
TouchSensor::~TouchSensor()
{
}

//-----------------------------------------------------------------------------------------------------------------
void TouchSensor::Update()
{
}

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
AudioSensor::AudioSensor( Demon*  _demon, bool  updateOnFirstCall )
						: Sensor( _demon, "AudioSense", updateOnFirstCall )
{
}

//-----------------------------------------------------------------------------------------------------------------
AudioSensor::~AudioSensor()
{
}

//-----------------------------------------------------------------------------------------------------------------
void AudioSensor::Update()
{
    //std::cout << "AudioSensor::Update()" << std::endl;
	std::string  senseData;
	Obj3D*       agentbody = demon->getRepresentation();
	
    int volumeFadePerDistance = IntConfig( "VolumeFadePerDistance" );   
    std::string wIntensityPropertyName = toupper(WINTENSITY);
    //std::cout << "wIntensityPropertyName: " << wIntensityPropertyName << std::endl;        
	for (PropertyMap::iterator  i = TheSimWorld.pmap.begin(); i != TheSimWorld.pmap.end(); i++)  {
		double  x, y, z;
		agentbody->getPosition( x, y, z );
		
		WorldObjectProperty&  p = i->second;
				
        //std::cout << "SimWorld element: " << i->first << " =>  Got property: Sound size = " << p.sound.size() << std::endl;        
		for (unsigned int s = 0; s < p.sound.size(); s++)  {
            //std::cout << "sound " << s << std::endl;
			int  volume = p.sound[s].VolumeAt( x, y, z, volumeFadePerDistance, wIntensityPropertyName);

			if ( volume > 0 )  {
                //std::cout << "volume > 0 " << std::endl;
				//Note: we can't take p.smell.AsXML() because that'd use the object's internal volume - not the phenomenal volume!
				int  freq;
				int  duration;
				
				p.sound[s].Get( WFREQ    , freq);
				p.sound[s].Get( WDURATION, duration);
				
				senseData += Sound(	volume,freq, duration).AsXML();
                //std::cout << "senseData after creating and appending a new Sound: " << senseData << std::endl;
                
			}
            //else  std::cout << "volume <= 0 " << std::endl;
		}
	}
	AddNewSensation( XMLembed( "audiodata", senseData ) );
}

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
SelfSensor::SelfSensor( Demon*  _demon )
					  : Sensor( _demon, "SelfSensor", false )
{
}

//-----------------------------------------------------------------------------------------------------------------
void  SelfSensor::Update()
{
    printf("SelfSensor::Update()\n");
	CSAgent*  bodied = dynamic_cast<CSAgent*>( demon );
	int       energy = 0;

	if ( bodied )  
		bodied->Get(WENERGY, energy);
    printf("SelfSensor::Update(): energy = %d\n", energy);
	
	double  x, y, z, phi;
	demon->getRepresentation()->getPosition( x, y, z );
	char  pos[30];
	 sprintf( pos, "(%.2f,%.2f,%.2f)", x, y, z );
    printf("SelfSensor::Update(): pos = %s\n", pos);

	demon->getRepresentation()->getOrientation( x, y, z, phi);
	char   ori[30];
	sprintf( ori, "(%.2f,%.2f,%.2f,%.2f)", x , y, z, phi);
    printf("SelfSensor::Update(): oru = %s\n", ori);

	float  eye_phi = 0.0, eye_theta = 0.0;
	demon->Get( EYE_THETA, eye_theta);
	demon->Get( EYE_PHI  , eye_phi  );
	char   eye_ori[30];
	sprintf( eye_ori, "(%.2f,%.2f)", eye_theta, eye_phi );
    printf("SelfSensor::Update(): eye_ori = %s\n", eye_ori);
	
	string  holding_str;
	if ( bodied )  {
		shared_ptr<meshData>  holding = bodied->GetLifted();

		if ( holding.get()  &&  holding->wrapper  &&  holding->wrapper->QueryObject() )
			holding_str = XMLembed( "holding", holding->wrapper->QueryObject()->GetName() );
	}
	
	string nodes = 	XMLembed( WENERGY, toString(energy) )
					+ holding_str
					+ XMLembed( "position", pos )
					+ XMLembed( "orientation", ori )
					+ XMLembed( "eye", 
						XMLembed( "orientation", eye_ori ) );

    printf("SelfSensor::Update(): nodes = %s\n", nodes.c_str());
    boost::mutex::scoped_lock  lock( demon->lock_provider );
    
//printf("initial selfsensor XML:\n%s\n", nodes.c_str());    
    while ( !demon->customSensationQueue.empty() )  {
        nodes += demon->customSensationQueue.front().AsXML();
//printf("selfsensor XML after add a customSensation:\n%s\n", nodes.c_str());    
        demon->customSensationQueue.pop();
    }
    printf("SelfSensor::Update(): nodes after customSensationQueue = %s\n", nodes.c_str());
	
	AddNewSensation( XMLembed( "selfdata",nodes ) );
}

//-----------------------------------------------------------------------------------------------------------------
SelfSensor::~SelfSensor()
{
}
