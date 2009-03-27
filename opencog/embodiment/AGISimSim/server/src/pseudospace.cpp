/*
 * opencog/embodiment/AGISimSim/server/src/pseudospace.cpp
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

Copyright Ari A. Heljakka / GenMind Ltd 03/2006.
Not part of public AGI-Sim.
Do not distribute.

TO_COMPLETE

*/

#include "simcommon.h"
#include "space.h"
#include "qsqrt.h"
#include "demon.h"
#include "simserver.h"
#include "simworld.h"
#include "CSagent.h"

iGUIProvider*	CSpseudo::GUIprovider;
CSproxy*        CSproxy::implementation = NULL;

float csVector3::Norm () const
{
  return qsqrt (x * x + y * y + z * z);
}

void csVector3::Normalize ()
{
  float sqlen = x * x + y * y + z * z;
  if (sqlen < SMALL_EPSILON) return ;

  float invlen = qisqrt (sqlen);
  *this *= invlen;
}

/*csVector3::csVector3 (const csDVector3 &v)
{
  x = (float)v.x;
  y = (float)v.y;
  z = (float)v.z;
}*/

CSproxy* CSproxy::Get()
{
	if ( !implementation ) implementation = new CSpseudo();
	return implementation;
}

map< string, meshData >  CSpseudo::GetMovableVisibleObjects( CSAgent*  agent )
{
	//int  w = Screen::Instance().w;
	//int  h = Screen::Instance().h;
	
/*	// Define Field-of-eatables:	
	int  sx = w / 4;
	int  sy = h / 4;
	int  ex = w - sx;
	int  ey = h - sy;*/

	double  __x, __y, __z, phi;

	if (!agent || !agent->getRepresentation())
		return map< string, meshData >();

	agent->getRepresentation()->getOrientation( __x, __y, __z, phi);
	agent->getRepresentation()->getPosition( __x, __y, __z );
	float  eye_phi = 0.0, eye_theta = 0.0;
	agent->Get( EYE_THETA, eye_theta);
	agent->Get( EYE_PHI  , eye_phi  );
	map< int, map<int, meshData> >  objs;
	float dphi = 3.14/2; //3.14*2
	 LOG( "Pseudo", 2, "GetVisibleObjects...");
	 float netsize = 0.05f; //0.01

	for (float next_phi = phi + eye_phi-dphi/2; next_phi < phi+eye_phi+dphi/2; next_phi += netsize)
	{
		map<int, map<int, meshData> > o = iEngine::Instance().VisibleObjects(csVector3(__x,__y,__z), next_phi);

		if (!o.empty())
			objs[o.begin()->first][o.begin()->second.begin()->first] = o.begin()->second.begin()->second;
	}
 
	set< std::string >              movables = TheSimWorld.GetMovableNames();
	map< string, meshData >         current_edibles;
	
	for (map< int, map<int, meshData> > :: iterator  x = objs.begin(); x!=objs.end(); x++)  {
		for (map<int, meshData >::iterator y = x->second.begin(); y != x->second.end(); y++)  {
			if ( y->second.wrapper )  {
				string  oname = ( y->second.wrapper->QueryObject() ? y->second.wrapper->QueryObject()->GetName() : "<null>" );
				// TODO: "1 agent soln"						
				if ( STLhas( movables, oname )  &&  oname != AGENT_NAME )
					current_edibles[oname] = ( y->second );
			}
		}
	}
	return current_edibles;
}


map< int, map<int, meshData> > iEngine::GetVisibleObjects(float sx, float ex)
{
// TODO: "1-Demon soln"

	shared_ptr< Demon >  demon = SimServer::Get()->GetDemon( 0 );
	Obj3D* body = demon->getRepresentation();
	double ox,oy,oz;
	body->getPosition(ox,oy,oz);

	return map< int, map<int, meshData> >();

/*	for (map<csVector3,csRef<iMeshWrapper> >::iterator p2m = pos2mesh.begin(); p2m != pos2mesh.end(); p2m++)
	{
		float msx = p2m->first.x - p2m->second.dim.x;
		float mex = p2m->first.x + p2m->second.dim.x;
	}

	map< int, map<int, meshData> > ret;

	int netsize = 1;

	int  dx = netsize;

	for (int  x = sx; x <= ex; x += dx)
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


	for (map<std::string,csRef<iMeshWrapper> >::iterator m = meshes.begin(); m != meshes.end(); m++)
	{

		meshes->meshes
	}*/
}


void iMovable::UpdateMove()
{
	iEngine* eng = FROM_CS(iEngine);
	eng->UpdateMove(this);
}

static float RoomScaleFactor;

void iEngine::UpdateMove(iMovable* m)
{
//	printf("No longer at: ");
	for (set<xy,lessxy>::iterator i = m2blocks[m].begin(); i != m2blocks[m].end(); i++)
	{
//		printf("(%d,%d) ", i->x, i->y);
		//blocks[i->x][i->y] = NULL;
		blocks[i->x][i->y].erase(m);
	}
//	printf("\n");

	m2blocks.erase(m);

	// Remove knowledge of anything I overlap with
	overlap.erase(m);

	// Remove knowledge of anything overlapping with me
	for (map<iMovable*, map<iMovable*, set<xy,lessxy> > >::iterator mm = overlap.begin(); mm != overlap.end(); mm++)
	{
		map<iMovable*, set<xy,lessxy> >::iterator mmi = mm->second.find(m);
		if (mmi != mm->second.end())
			mm->second.erase(mmi);
	}

	set<xy,lessxy> myblocks;
	
	csVector3 pos = m->GetFullPosition();

	float bsx = pos.x - m->GetMesh()->dim.x;
	float bex = pos.x + m->GetMesh()->dim.x;
	float bsy = pos.z - m->GetMesh()->dim.z;
	float bey = pos.z + m->GetMesh()->dim.z;

	float  w = FloatConfig( "RoomSizeX" )/dx;
	float  h = FloatConfig( "RoomSizeZ" )/dy;

//	const float x_buffer = (dx-0.02*RoomScaleFactor), y_buffer = (dy-0.02*RoomScaleFactor);
	const float x_buffer = 0.001f, y_buffer = 0.001f;

	#define Xcoord2index( _x ) (unsigned int)( (FloatConfig( "RoomSizeX" ) /2+_x) / dx )
	#define Ycoord2index( _y ) (unsigned int)( (FloatConfig( "RoomSizeZ" ) /2+_y) / dy )

//puts("Occupation:");

	const float floor_w = FloatConfig( "RoomSizeX" );
	const float floor_h = FloatConfig( "RoomSizeY" );

	for (float	ix = bsx - x_buffer;
				ix < bex + x_buffer;
				ix+= dx / RoomScaleFactor)
	{
		for (float	iy = bsy - y_buffer;
					iy < bey + y_buffer;
					iy+= dy / RoomScaleFactor)
		{
//			unsigned int iix = Xcoord2index(ix);
			unsigned int iix = (unsigned int)( (floor_w /2+ix) / dx );
//			unsigned int iiy = Ycoord2index(iy);
			unsigned int iiy = (unsigned int)( (floor_h /2+iy) / dy );
			
			if ( iix < 0 || iiy < 0) //May happen due to buffer deltas.
				continue;
			
			if ( iix >= w  || iiy >=h)
			{
/*				LOG( "TEST",0,"OVERFLOW Block #"+i2str(i)+ " blocking "
					+i2str(ix / dx) + "," + i2str(iy / dy));*/
				continue;
			}
			
			/// Save the overlap info:

			for (set<iMovable*>::iterator	imi = blocks[iix][iiy].begin();
											imi!= blocks[iix][iiy].end(); imi++)
			{
				overlap[*imi][m].insert(xy(iix,iiy));
				overlap[m][*imi].insert(xy(iix,iiy));
			}

			//blocks[iix][iiy] = m;
			blocks[iix][iiy].insert(m);

			myblocks.insert(xy(iix,iiy)); //y => z

//			printf("(%d,%d): %s\n", iix, iiy, m->GetMesh()->QueryObject()->GetName());
		}
//		printf("%f\n",ix);
	}

	m2blocks[m] = myblocks;
}

#define round(a) (    (( (int)((a)+0.5) ) > (a))    ?    (((int)(a))+1)    :    ((int)(a)) )

//map<int, map<int, iMeshWrapper*> > iEngine::VisibleObjects(csVector3 orig, float sphi) //, float ephi)
//map< int, map<int, meshData> > iEngine::VisibleObjects(csVector3 orig, float sphi, float ephi)
map< int, map<int, meshData> > iEngine::VisibleObjects(csVector3 orig, float sphi)
{
	const bool OBJECT_TRANSPARENCY = false;

	int x0 = Xcoord2index(orig.x);
	int y0 = Ycoord2index(orig.y);

	float r = (15/2) * RoomScaleFactor;
	// y/r = sin sphi
	
	int y1 = Ycoord2index(orig.y + r * sin(sphi));
	int x1 = Ycoord2index(orig.x + r * cos(sphi));

/*	if (x0>x1)
		swap(x1,x0);
	if (y0>y1)
		swap(y1,y0);*/

	set<xy,lessxy> visible_coords;
	map<int, map<int, meshData> > objs;

	int deltax = x1 - x0;
	int deltay = y1 - y0;

	const float floor_w = FloatConfig( "RoomSizeX" );
	const float floor_h = FloatConfig( "RoomSizeY" );

	/// See yourself!
	if (x0>=0 && y0>=0 && x0 < discrete_w && y0 < discrete_h && !blocks[x0][y0].empty())
		objs[x0][y0] = meshData((*blocks[x0][y0].begin())->GetMesh(),0,0);

	int xo = x0, yo = y0;

	if (deltax != 0) {
		float m = (float) deltay / (float) deltax;
		float b = yo - m*xo;
		deltax = (x1 > xo) ? 1 : -1;
		while (xo != x1)
		{
			xo += deltax;
			yo = round(m*xo + b);

			float this_dy = (yo-y0)*dy - floor_h/2;
			float this_dx = (xo-x0)*dx - floor_w/2;
			float dist = sqrt(this_dx*this_dx+this_dy*this_dy);

			if (xo>=0 && yo>=0 && xo < discrete_w && yo < discrete_h
				&& !blocks[xo][yo].empty())
			{
				objs[xo][yo] = meshData((*blocks[xo][yo].begin())->GetMesh(), 0, dist);
				if (!OBJECT_TRANSPARENCY)
					return objs;
			}
//			printf("(%d,%d)\t", xo,yo);
		}
	}
//	printf("\n\n\n\n\n\n\n\n\n\n\n\n");

	return objs;
}


bool CSpseudo::OnConnectToWorld()
{
	 LOG("CSBox",0,"OnConnectToWorld");

	double x, y, z;
	x = y = z = 0.0;	

  	if (IntConfig("DrawGUI"))  {
                 iEngine::Instance().graphDump();
/*		if (!g3d->BeginDraw(engine->GetBeginDrawFlags() | CSDRAW_3DGRAPHICS | CSDRAW_CLEARZBUFFER | CSDRAW_CLEARSCREEN))
			LOG("CSBox",2, "g3d Error...");			
		view->Draw();*/
	}

	GUIprovider->SetConnected  (true);	
	GUIprovider->SetStatusText ("Connected");
	 LOG("CSBox",2, "Push 1st frame...");
	
	PushFrame();
	
	 LOG("CSBox",0, "OnConnectToWorld ok");
	
	return true;
}

bool CSpseudo::SetupFrame	 ()
{
	 LOG("CSbox",4, "SetupFrame");
	
	// Enable command processing on other threads by protecting the CS pump by this mutex.
	boost::mutex::scoped_lock serverlock( SimServer::Get()->AcquireLock() );
	LOG("CSbox",4, "SetupFrame passed server LOCK");
	
	// TODO: "1 agent soln"

	if ( !SimServer::Get()->GetDemon(0) )  {
	  serverlock.unlock();
		SimServer::Get()->OnIdleEvent();		
		 LOG("CSbox",4, "SetupFrame ending. Was idle.");
		return true;
	}

//	DrawViews();
  
	GUIprovider->OnSetupFrameEnd();
	  
	 LOG("CSbox",4, "Idle event...");
	serverlock.unlock();

	  SimServer::Get()->OnIdleEvent();
	 LOG("CSbox",4, "SetupFrame ok.");

	return true;
}

void CSpseudo::PushFrame   () {
	if (GUIprovider->IsConnected())
		SetupFrame();
}
void CSpseudo::RunLoop     (){ }
bool CSpseudo::ConnectToWorld		 (const char* worldURL)
{
	return SimServer::Get()->ConnectToWorld();
}
bool CSpseudo::DisconnectFromWorld ()
{
	return SimServer::Get()->DisconnectFromWorld();
}

iEngine::iEngine()
: dx(0.25f), dy(0.25f), discrete_w(0), discrete_h(0)
{
  LOG("HardWorld", 0, "Creating iEngine...");
	RoomScaleFactor = 1.0f; //FloatConfig( "RoomSizeX" ) / 15.0f;

	dx *= RoomScaleFactor;
	dy *= RoomScaleFactor;

	discrete_w  =  (int) ((FloatConfig( "RoomSizeX" )) / dx +1);
	discrete_h  =  (int) ((FloatConfig( "RoomSizeZ" )) / dy +1);
	
	meshes = csRef<iMeshList>(new iMeshList);
	blocks = new set<iMovable*>* [discrete_w];

	printf("dicrete_w %d\n", discrete_w);

	for (int k=0;k< discrete_w;k++)
	{
		blocks[k] = new set<iMovable*>[discrete_h];
/*		for (int p=0;p<discrete_h;p++)
			blocks[k][p] = ;*/
	}
	LOG("HardWorld", 0, "iEngine created.");
}

void iEngine::MeshDump()
{
	map<string,csRef<iMeshWrapper> >::iterator it;
		 for (it=meshes->meshes.begin(); it!=meshes->meshes.end(); it++)
		 {
			 csVector3 pos(it->second->GetMovable()->GetPosition());
			 csVector3 rot(it->second->GetMovable()->GetRotation());
			 string s = it->first;
			 printf("%s @(%f,%f) @(%f)\n", it->first.c_str(), pos.x, pos.z, rot.z);
		 }
}

static int gcount=0;

void iEngine::graphDump() const
{
//	system("cls");

	int    w  =  (int) ((FloatConfig( "RoomSizeX" )) / dx +1);
	int    h  =  (int) ((FloatConfig( "RoomSizeZ" )) / dy +1);
	
	int px = w/79, py = w/40;

	for (int i=0;i<70;i++)
		printf("-");
	printf("\n");


	printf("Discrete: %d %d\n", discrete_w, discrete_h);
	/*	int discrete_w = 20;
	int discrete_h=20;*/

	if (gcount++ < 1)
	  return;

	for (int p=0;p<h;p+=py)
	{
		for (int k=0;k< w;k+=px)
		{
			char c = ' ';

			for (int p2=0;p2<py && p+p2<h;p2++)
			{
				for (int k2=0;k2<px && k+k2<w;k2++)
				{
				  if (k+k2 >= discrete_w || p+p2>=discrete_h)
				    {
				      printf("%d+%d / %d,%d+%d / %d\n",
					     k,k2,discrete_w,p,p2,discrete_h);
				      continue;
}

					if (!blocks[k+k2][p+p2].empty())
					{
					  if ((*blocks[k+k2][p+p2].begin()) &&
(*blocks[k+k2][p+p2].begin())->GetMesh() &&
						    (*blocks[k+k2][p+p2].begin())->GetMesh()->QueryObject()
&&  strlen((*blocks[k+k2][p+p2].begin())->GetMesh()->QueryObject()->GetName())>0)
							c = (*blocks[k+k2][p+p2].begin())->GetMesh()->QueryObject()->GetName()[0];
					}
				}
			}

			printf("%c", c);
		}
		printf("\n");
	}

	for (int j=0;j<70;j++)
		printf("-");
	printf("\n");
}
