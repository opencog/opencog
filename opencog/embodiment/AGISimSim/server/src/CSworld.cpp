/*
 * opencog/embodiment/AGISimSim/server/src/CSworld.cpp
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
 *  CS object manipulation and loading.
 * 
 *  Project: Novamente
 *
 ****************************************************************************/


#include "simcommon.h"
#include "space.h"
#include "command.h"
#include "simworld.h"
#include "CSworld.h"
#include "simserver.h"
#include "demon.h"

#if CRYSTAL
 #include "CSops.h"
 #include "CSserver.h"
 #include "CSMeshTool.h"
#endif



void CleanSpace    (string& s);
bool SelectCommand (string vcmd,  ReportProvider *reporter,  shared_ptr<Command>& com,  
					ServerSocket* _socket = NULL);


//--------------------------------------------------------------------------------------------------------------
class LogReportProvider : public ReportProvider
{
public:
	LogReportProvider() {}
	virtual void SendMsg (string msg) {
		 LOG("LogReportProvider", 1, msg);
	}
	virtual void SendError(string msg) {
		 LOG("LogReportProvider", 0, msg);
	}
};

//--------------------------------------------------------------------------------------------------------------
iSector* CSWorld::GetRoom() const
{
	return room;
}

//--------------------------------------------------------------------------------------------------------------
string CSWorld::GetFreeName(string base)
{
	iEngine*  engine = FROM_CS( iEngine );
	iMeshList*      meshes = engine->GetMeshes();			  
	string  name = base;
	int     suffix = 1;
	
	while(true) {
		bool ok = true;
		for (int i = 0; i < meshes->GetCount(); i++) {
			iMeshWrapper*  mesh = meshes->Get(i);
			if ( mesh->QueryObject() ) {
				 if ( nocase_equal( mesh->QueryObject()->GetName(), name.c_str() ) )
					ok = false;
			}
		}
		if (ok) break;
		name = base + toString(suffix++);
	};

	return name;
}


float normalizeAngle(float angle) {
    const double  pi = 3.14159265;
    float result = angle;
    while ( result > 2*pi )  result -= 2*pi;
    while ( result < 0    )  result += 2*pi;
    return result;
}

#if CRYSTAL

//--------------------------------------------------------------------------------------------------------------
bool  CSWorld::RotateObject ( csRef<iMeshWrapper>  mesh, float rotX, float rotY, float rotZ) {
    if ( mesh->QueryObject() )  {
        iMovable*  movable = mesh->GetMovable();
        string meshName = string(mesh->QueryObject()->GetName());
        if ( movable != NULL ) {
             LOG("CSWorld::RotateObject",0,"Object is movable: " + meshName);

            printf("Rotating object by (%f,%f,%f)\n", rotX, rotY, rotZ);
            
            csMatrix3  rot = csXRotMatrix3( rotX ) * csYRotMatrix3( rotY ) * csZRotMatrix3( rotZ );
            csOrthoTransform  ot( rot, movable->GetTransform().GetOrigin() );
            movable->SetTransform(ot);      
            return true;
        }      
        else  {
             LOG("CSWorld::RotateObject",0,"Object is not movable: " + meshName);
        }
    }
    else { 
         LOG("CSWorld::RotateObject",0,"Mesh not queryble."); 
    }
    return false;
}

#define ADULT_HAND_NAME "Adult_Torso:Adult_ArmLeft:Adult_ForearmLeft:Adult_HandLeft"
#define CHILD_HAND_NAME "Child_Torso:Child_ArmLeft:Child_ForearmLeft:Child_HandLeft"
#define ADULT_ARM_NAME "Adult_Torso:Adult_ArmLeft"
#define CHILD_ARM_NAME "Child_Torso:Child_ArmLeft"
#define ADULT_LEG_NAME "Adult_Torso:Adult_ThighLeft"
#define CHILD_LEG_NAME "Child_Torso:Child_ThighLeft"

//--------------------------------------------------------------------------------------------------------------
bool  CSWorld::MoveArm ( csRef<iMeshWrapper>  mesh, float angle) {
    if ( mesh->QueryObject() )  {
        iMeshWrapper* armMesh = mesh->FindChildByName(ADULT_ARM_NAME);
        if (!armMesh) {
            armMesh = mesh->FindChildByName(CHILD_ARM_NAME);
        }
        if (armMesh) {
            RotateObject(armMesh, angle, 0, 0);
            return true;
        }      
        else  {
             LOG("CSWorld::RotateObject",0,"Arm mesh was not found for object: " + string(mesh->QueryObject()->GetName()));
        }
    }
    else { 
         LOG("CSWorld::RotateObject",0,"Mesh not queryble."); 
    }
    return false;
}

//--------------------------------------------------------------------------------------------------------------
bool  CSWorld::MoveLeg ( csRef<iMeshWrapper>  mesh, float angle) {
    if ( mesh->QueryObject() )  {
        iMeshWrapper* legMesh = mesh->FindChildByName(ADULT_LEG_NAME);
        if (!legMesh) {
            legMesh = mesh->FindChildByName(CHILD_LEG_NAME);
        }
        if (legMesh) {
            RotateObject(legMesh, angle, 0, 0);
            return true;
        }      
        else  {
             LOG("CSWorld::RotateObject",0,"Arm mesh was not found for object: " + string(mesh->QueryObject()->GetName()));
        }
    }
    else { 
         LOG("CSWorld::RotateObject",0,"Mesh not queryble."); 
    }
    return false;
}

#endif

//--------------------------------------------------------------------------------------------------------------
#define MAXIMAL_DISTANCE_TO_LIFT_AN_OBJECT 70
bool  CSWorld::LiftObject (iMeshWrapper* actorMesh, iMeshWrapper* targetMesh) {
    const char* actorName = actorMesh->QueryObject()->GetName();
    const char* liftedObjName = liftedObjMap[actorName];
    if (liftedObjName != NULL) {
        printf("Cannot lift an object when another one is already lifted.\n");
        // Do not allow to lift an object if another object is still lifted.
        return false;
    }
    csVector3 cur_target_pos = targetMesh->GetMovable()->GetPosition();
    csVector3 delta_pos = actorMesh->GetMovable()->GetFullPosition();
    delta_pos -= cur_target_pos;
    float distance = sqrt(delta_pos.x*delta_pos.x + delta_pos.z*delta_pos.z);
    if (distance > MAXIMAL_DISTANCE_TO_LIFT_AN_OBJECT) {
        printf("Cannot lift an object whose distance is : %f\n", distance);
        // Too far away to lift object
        return false;
    }

    liftedObjMap[actorName] = targetMesh->QueryObject()->GetName();
    originalLiftedHeightMap[actorName] = cur_target_pos.y;
    printf("actor = %s, target =%s\n", actorName, targetMesh->QueryObject()->GetName());
    
#if CRYSTAL
    iMeshWrapper* handMesh = actorMesh->FindChildByName(ADULT_HAND_NAME);
    if (!handMesh) {
        handMesh = actorMesh->FindChildByName(CHILD_HAND_NAME);
    }
#else
	iMeshWrapper* handMesh = NULL;
#endif
    if (handMesh) {
        UpdateLiftedObject(actorMesh, 0, cur_target_pos.y, 0);
    } else {
        targetMesh->GetMovable()->SetPosition(csVector3(cur_target_pos.x, cur_target_pos.y + FloatConfig("LiftHeight"), cur_target_pos.z));
        targetMesh->GetMovable()->UpdateMove();
    }

	LOG("CSWorld::LiftObject",1,"Lifted.");
	return true;
}

//--------------------------------------------------------------------------------------------------------------
void  CSWorld::UpdateLiftedObject (iMeshWrapper* actorMesh, float deltaX, float deltaY, float deltaZ) {
    const char* liftedObjName = liftedObjMap[actorMesh->QueryObject()->GetName()];
    if (liftedObjName != NULL) {
        iMeshWrapper* liftedMesh = FROM_CS(iEngine)->FindMeshObject(liftedObjName);
        if (liftedMesh != NULL)
		{
#if CRYSTAL
		    iMeshWrapper* handMesh = actorMesh->FindChildByName(ADULT_HAND_NAME);
            if (!handMesh) {
                handMesh = actorMesh->FindChildByName(CHILD_HAND_NAME);
            }
#else
	iMeshWrapper* handMesh = NULL;
#endif

            if (handMesh) {
                // Put object in the actor's hand.
                csVector3 hand_full_pos = handMesh->GetMovable()->GetFullPosition();
                printf("HAND FULL POS = (%f, %f, %f)\n", hand_full_pos.x, hand_full_pos.y, hand_full_pos.z);
                csBox3 cbox;
                liftedMesh->GetWorldBoundingBox(cbox);
                liftedMesh->GetMovable()->SetPosition(csVector3(hand_full_pos.x, hand_full_pos.y - (cbox.MaxY() - cbox.MinY())/2, hand_full_pos.z));
            } else {
                csVector3 cur_pos = liftedMesh->GetMovable()->GetPosition();
                liftedMesh->GetMovable()->SetPosition(csVector3(deltaX + cur_pos.x, deltaY + cur_pos.y, deltaZ + cur_pos.z));
            }
            liftedMesh->GetMovable()->UpdateMove();
        }
    }
}

//--------------------------------------------------------------------------------------------------------------
bool  CSWorld::DropObject ( iMeshWrapper*  actorMesh) {
	const char* actorName = actorMesh->QueryObject()->GetName();
	const char* liftedObjName = liftedObjMap[actorName];

	if (liftedObjName != NULL) {
		iMeshWrapper* liftedMesh = FROM_CS(iEngine)->FindMeshObject(liftedObjName);
		if (liftedMesh != NULL) {
			csVector3 cur_pos = liftedMesh->GetMovable()->GetPosition();
			float originalHeight = originalLiftedHeightMap[actorMesh->QueryObject()->GetName()];
			liftedMesh->GetMovable()->SetPosition(csVector3(cur_pos.x, originalHeight, cur_pos.z));
			liftedMesh->GetMovable()->UpdateMove();
		}
		liftedObjMap.erase(actorMesh->QueryObject()->GetName());
		originalLiftedHeightMap.erase(actorMesh->QueryObject()->GetName());
		return true;
    }
	return false;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------
void CSWorld::ResetObjects()
{
	iEngine*  engine = FROM_CS( iEngine );
	iMeshList*      meshes = engine->GetMeshes();
			
	if ( !meshes )  {
		 LOG("CSWorld", 1, "NO MESHES! Should MakeWorld() first!");
		return;
	}
	
	//TODO: Orientation is not reset.
	
	for (int i = 0; i < meshes->GetCount(); i++)  {
		iMeshWrapper*  mesh = meshes->Get(i);
		if ( mesh->QueryObject() )  {
			string    label( mesh->QueryObject()->GetName() );
			CSObject  o = TheSimWorld.GetMovable(label);
			
			if ( (mesh->GetMovable()->GetFullPosition() - o.startpos).SquaredNorm() > 0.01 )  {
				mesh->GetMovable()->SetPosition( o.startpos );
				mesh->GetMovable()->UpdateMove ();
			
				 LOG("CSWorld::ResetObjects",2,"Repositioned: " + label);
			}
			else  {
				LOG("CSWorld::ResetObjects",3,"Was immobile: " + label);
			}
		}
		else  { 
			LOG("CSWorld::ResetObjects",2,"Unknown mesh not reset."); 
		}
	}
}

#if CRYSTAL

//--------------------------------------------------------------------------------------------------------------
csRef<iMeshWrapper>  CSWorld::InsertBall( csVector3 pos, std::string name, std::string materialname, 
										  float radiusX, float radiusY, float radiusZ, bool inert, bool collidable)
{
  const char * ballmodel = "/sim/ball";
  csRef<iMeshWrapper> ball = InsertMesh(pos, ballmodel, name,
					materialname, 1.0, inert, collidable);
  
  csMatrix3 scaler;
  scaler.Identity();
  scaler.m11 = radiusX;
  scaler.m22 = radiusY;
  scaler.m33 = radiusZ;
  ball->GetMeshObject()->HardTransform(csReversibleTransform(scaler, csVector3(0.0f, 0.0f, 0.0f)));
  
  return ball;
}

//--------------------------------------------------------------------------------------------------------------
csRef<iMeshWrapper> CSWorld::VisibleFovMesh(float fov_angle, float near_z, float far_z)
{
	csRef<iGraphics2D>  g2d;
	float  view_width  = g2d->GetWidth();
	float  view_height = g2d->GetHeight();

	string     name( "FOVconstruct" );
	csVector3  pos(0.0, 0.0, 0.0);
	iEngine*  engine = FROM_CS( iEngine );
	csRef<iCamera>  cam    = engine->CreateCamera();
	cam->SetFOVAngle( fov_angle, (int)view_width );
	// Should camera transformation be set?

	csVector3 tlf( cam->InvPerspective( csVector2(0.0	    , 0.0        ), far_z  ) );
	csVector3 tln( cam->InvPerspective( csVector2(0.0	    , 0.0        ), near_z ) );

	csVector3 trf( cam->InvPerspective( csVector2(view_width, 0.0        ), far_z  ) );
	csVector3 trn( cam->InvPerspective( csVector2(view_width, 0.0	     ), near_z ) );

	csVector3 brf( cam->InvPerspective( csVector2(view_width, view_height), far_z  ) );
	csVector3 brn( cam->InvPerspective( csVector2(view_width, view_height), near_z ) );

	csVector3 blf( cam->InvPerspective( csVector2(0.0       , view_height), far_z  ) );
	csVector3 bln( cam->InvPerspective( csVector2(0.0       , view_height), near_z ) );

	csRef<iMeshWrapper>         genmesh;
	csRef<iMeshFactoryWrapper>  genmesh_fact ( csPtr<iMeshFactoryWrapper>  ( engine->CreateMeshFactory(
  											   "crystalspace.mesh.object.genmesh", ( name + ".fact" ).c_str())) );
	if ( !genmesh_fact )  {
		 LOG("CSWorld", 1, "Can't make genmesh factory!");
		return genmesh;
	}
	csRef<iGeneralFactoryState>  factstate = SCF_QUERY_INTERFACE ( genmesh_fact->GetMeshObjectFactory (), iGeneralFactoryState );
	if ( !factstate ) {
		 LOG("CSWorld", 1, "Strange, genmesh_fact doesn't implement iGeneralFactoryState!");
		return genmesh; //!!!FP not factstate ?
	}

	//  factstate->SetMaterialWrapper ( engine->FindMaterial(materialname.c_str()) );
	factstate->SetVertexCount   (8);
	factstate->SetTriangleCount (8);

	csVector3*   verts     = factstate->GetVertices  ();
	csColor4*    colors    = factstate->GetColors    ();
	csTriangle*  triangles = factstate->GetTriangles ();

	verts[0] = tln;
	colors[0].Set(0.0, 1.0, 0.0, 1.0);

	verts[1] = tlf;
	colors[1].Set(0.0, 1.0, 0.0, 0.0);

	verts[2] = trn;
	colors[2].Set(0.0, 1.0, 0.0, 1.0);

	verts[3] = trf;
	colors[3].Set(0.0, 1.0, 0.0, 0.0);

	verts[4] = brn;
	colors[4].Set(0.0, 1.0, 0.0, 1.0);

	verts[5] = brf;
	colors[5].Set(0.0, 1.0, 0.0, 0.0);

	verts[6] = bln;
	colors[6].Set(0.0, 1.0, 0.0, 1.0);

	verts[7] = blf;
	colors[7].Set(0.0, 1.0, 0.0, 0.0);

	triangles[0].Set(0, 2, 1);
	triangles[1].Set(1, 3, 2);
	triangles[2].Set(2, 4, 3);
	triangles[3].Set(3, 5, 4);
	triangles[4].Set(4, 6, 5);
	triangles[5].Set(5, 7, 6);
	triangles[6].Set(6, 0, 7);
	triangles[7].Set(7, 1, 0);

	factstate->CalculateNormals ();
	factstate->Invalidate       ();

	genmesh = csPtr<iMeshWrapper> ( engine->CreateMeshWrapper (genmesh_fact, name.c_str(), room, pos) );
	if ( !genmesh )  {
		 LOG("CSWorld",1, "Can't make genmesh object!");
		return genmesh;
	}
	csRef<iGeneralMeshState>  state( SCF_QUERY_INTERFACE( genmesh->GetMeshObject(), iGeneralMeshState ) );
	if ( !state )  {
		 LOG("CSWorld",1, "Strange, genmesh doesn't implement iGeneralMeshState!");
		return genmesh;
	}

	state->SetLighting     (true);
	state->SetManualColors (true);

	return genmesh;
}

//-------------------------------------------------------------------------------------------------------------
csRef< iMeshWrapper >  CSWorld::InsertBox ( csVector3 pos, std::string name, std::string materialname, 
										  csVector3 dim, bool inert,bool collidable )
{
	float dz = 0.02;
	if (inert)  dz = 0.001;
    
	if ( dim.x+dz > FloatConfig( "RoomSizeX" ) || 
		 dim.y+dz > FloatConfig( "RoomSizeY" ) || 
		 dim.z+dz > FloatConfig( "RoomSizeZ" ) )
	{
		 LOG("CSWorld", 1, "Inserted object doesn't fit the room.");
		csRef<iMeshWrapper>  emptyw;

		return emptyw;
	}
	
	float  minH = dim.y / 2 + dz;
	float  maxH = FloatConfig( "RoomSizeY" ) - dim.y/2;

	if (pos.y < minH)  pos.y = minH; //Don't touch the ground.
	if (pos.y > maxH)  pos.y = maxH; //Don't touch the ceiling.
	
	iEngine*       engine = FROM_CS(iEngine);
	csRef<iMeshWrapper>  genmesh;

	csRef<iMeshFactoryWrapper>   genmesh_fact( csPtr<iMeshFactoryWrapper> (engine->CreateMeshFactory (
  											  "crystalspace.mesh.object.genmesh", ( name + ".fact" ).c_str())) );
	if ( !genmesh_fact )  {
		 LOG("CSWorld",1, "Can't make genmesh factory!");
		return genmesh;
	}

	csRef<iGeneralFactoryState>  factstate = SCF_QUERY_INTERFACE ( genmesh_fact->GetMeshObjectFactory(), iGeneralFactoryState );
	if ( !factstate )  {
		 LOG("CSWorld", 1, "Strange, genmesh_fact doesn't implement iGeneralFactoryState!");
		return genmesh;
	}

	factstate->SetMaterialWrapper ( engine->FindMaterial(materialname.c_str()) ); 
	factstate->SetVertexCount     (8);
	factstate->SetTriangleCount   (12);

	csVector3*   verts     = factstate->GetVertices ();
	csVector2*   texels    = factstate->GetTexels   ();
	csTriangle*  triangles = factstate->GetTriangles();

	verts[0].Set(-dim.x/2, dim.y/2, dim.z/2);
	texels[0].Set(0,0 );

	verts[1].Set(-dim.x/2, dim.y/2, -dim.z/2);
	texels[1].Set(1,0);

	verts[2].Set(dim.x/2, dim.y/2, -dim.z/2);
	texels[2].Set(0, 1);

	verts[3].Set(dim.x/2, dim.y/2, dim.z/2);
	texels[3].Set(1,1);

	verts[4].Set(-dim.x/2, -dim.y/2, dim.z/2);
	texels[4].Set(1,0);

	verts[5].Set(-dim.x/2, -dim.y/2, -dim.z/2);
	texels[5].Set(0, 1);

	verts[6].Set(dim.x/2, -dim.y/2, -dim.z/2);
	texels[6].Set(1,1);

	verts[7].Set(dim.x/2, -dim.y/2, dim.z/2);
	texels[7].Set(0, 0);

	triangles[0].Set(0,3,1);
	triangles[1].Set(3,2,1);
	triangles[2].Set(4,5,7);
	triangles[3].Set(5,6,7);
	triangles[4].Set(0,4,3);
	triangles[5].Set(4,7,3);
	triangles[6].Set(1,6,5);
	triangles[7].Set(1,2,6);
	triangles[8].Set(0,1,5);
	triangles[9].Set(0,5,4);
	triangles[10].Set(2,3,7);
	triangles[11].Set(2,7,6);

	factstate->CalculateNormals ();
	factstate->Invalidate       ();

	genmesh = csPtr<iMeshWrapper> ( engine->CreateMeshWrapper( genmesh_fact, name.c_str(), room, pos ) );
	if ( !genmesh )  {
		 LOG("CSWorld",1, "Can't make genmesh object!");
		return genmesh;
	}

	csRef<iGeneralMeshState>  state( SCF_QUERY_INTERFACE( genmesh->GetMeshObject(),	iGeneralMeshState ) );
	if ( !state )	{
		 LOG("CSWorld",1, "Strange, genmesh doesn't implement iGeneralMeshState!");
		return genmesh;
	}

	state->SetLighting     (true);
	state->SetManualColors (false);
	genmesh->SetZBufMode   (CS_ZBUF_USE); //CS_ZBUF_FILL
	//genmesh->SetRenderPriority (engine->GetWallRenderPriority ());

  	genmesh->GetMovable()->SetPosition( pos );
	genmesh->GetMovable()->UpdateMove ();

	// Registration	
	if ( !inert )  {
		CSObject  o;
		o.startpos = pos;
		o.volume   = dim.x*dim.y*dim.z;
		o.meshtype = "box";
		o.label    = name;
		
		TheSimWorld.AddMovableInfo(o);
	}
	else if ( collidable )  RegisterInert( name );

  return genmesh;
}

//-------------------------------------------------------------------------------------------------------------
void CreatePolygon (iThingFactoryState *th,	int v1, int v2, int v3, int v4, iMaterialWrapper *mat)
{
  th->AddPolygon               (4, v1, v2, v3, v4);
  th->SetPolygonMaterial       ( CS_POLYRANGE_LAST, mat );
  th->SetPolygonTextureMapping ( CS_POLYRANGE_LAST,   6 );
}

// This doesn't really work! TODO: FIX.
//-------------------------------------------------------------------------------------------------------------
csRef < iMeshWrapper >  CSWorld::InsertCyl( csVector3 pos, std::string name, std::string materialname, float
  						   				    radius, float height, bool inert, bool collidable )
{
	float dz = 0.02;
	if ( inert )  dz = 0.001;
	if ( radius*2 + dz > FloatConfig( "RoomSizeX" ) || 
		 height   + dz > FloatConfig( "RoomSizeY" ) || 
		 radius*2 + dz > FloatConfig( "RoomSizeZ" ) )
	{
		 LOG( "CSWorld", 1, "Inserted object doesn't fit the room." );
		csRef< iMeshWrapper >  emptyw;
		return emptyw;
	}

	iEngine*         engine        = FROM_CS( iEngine );
	csRef< iPluginManager >  pluginmanager = FROM_CS( iPluginManager );
	csRef< iLoader >         loader        = FROM_CS( iLoader );
	csRef< iGraphics3D >     g3d           = FROM_CS( iGraphics3D );
	csRef< iMeshWrapper >    genmesh;
	csRef< iMeshFactoryWrapper >  genmesh_fact( csPtr< iMeshFactoryWrapper >  ( engine->CreateMeshFactory(
											    "crystalspace.mesh.object.genmesh", ( name + ".fact" ).c_str() ) ) );
	if ( !genmesh_fact )  {
		 LOG( "CSWorld", 1, "Can't make genmesh factory!" );
		return genmesh;
	}

	csRef< iGeneralFactoryState >  factstate = SCF_QUERY_INTERFACE( genmesh_fact->GetMeshObjectFactory(),
																	iGeneralFactoryState );
	if ( !factstate )  {
		 LOG( "CSWorld", 1, "Strange, genmesh_fact doesn't implement iGeneralFactoryState!" );
		return genmesh;
	}

	factstate->SetMaterialWrapper( engine->FindMaterial( materialname.c_str() ) );

	int m_nSegments = 24;

	factstate->SetVertexCount  ( ( m_nSegments + 1 )* 2 + ( m_nSegments + 1 ) );
	factstate->SetTriangleCount( ( m_nSegments )    * 2 +  m_nSegments );

	csVector3*   verts     = factstate->GetVertices();
	csVector2*   texels    = factstate->GetTexels();
	csTriangle*  triangles = factstate->GetTriangles();
	csVector3*   pVertex   = &verts[0];
	
	csVector2*   pTex      = &texels[0];
	csTriangle*  pTri      = &triangles[0];

	int   v = 0;
	int   nCurrentSegment;
	float rDeltaSegAngle = 2.0f * 3.141592 / m_nSegments;
	float rSegmentLength = 1.0f / (float)m_nSegments;

	//Create the sides triangle strip
	for ( nCurrentSegment = 0; nCurrentSegment < ( m_nSegments ); nCurrentSegment++ )  {
		float  x0 = radius * sinf( nCurrentSegment* rDeltaSegAngle );
		float  z0 = radius * cosf( nCurrentSegment* rDeltaSegAngle );

		//	printf("%d: %f, %f (%f rad)\n", v, x0, z0, nCurrentSegment * rDeltaSegAngle);
		pVertex->x = x0;
		pVertex->y = 0.0f + ( height / 2.0f );
		pVertex->z = z0;

		pTex->x    = 1.0f - ( rSegmentLength * (float)nCurrentSegment );
		pTex->y    = 0.0f;

		pVertex++;
		pTex++;
		v++;

		pVertex->x = x0;
		pVertex->y = 0.0f - ( height / 2.0f );
		pVertex->z = z0;

		pTex->x = 1.0f - ( rSegmentLength * (float)nCurrentSegment );
		pTex->y = 1.0f;

		pVertex++;
		pTex++;
		v++;
	}


	for ( nCurrentSegment = 0; nCurrentSegment < m_nSegments - 1; nCurrentSegment++ ){
		pTri->Set( nCurrentSegment *   2, nCurrentSegment * 2+1, nCurrentSegment * 2+2 );
		pTri++;
		pTri->Set( nCurrentSegment * 2+3, nCurrentSegment * 2+2, nCurrentSegment * 2+1 );
		pTri++;
	}

	//Last one, connect to the 1st one:
	pTri->Set( nCurrentSegment * 2, nCurrentSegment * 2+1, 0 );
	pTri++;
	pTri->Set( 1, 0, nCurrentSegment* 2+1 );
	pTri++;

	int topmid = m_nSegments * 2;

	//Create the top triangle fan: Center
	pVertex->x = 0.0f;
	pVertex->y = 0.0f + ( height / 2.0f );
	pVertex->z = 0.0f;
	pTex->x    = 0.5f;
	pTex->y    = 0.5f;

	pVertex++;
	pTex++;
	v++;

	//Create the top triangle fan: Edges
	for ( nCurrentSegment = 0; nCurrentSegment < m_nSegments; nCurrentSegment++ ){
		float  x0 = radius * sinf( nCurrentSegment * rDeltaSegAngle );
		float  z0 = radius * cosf( nCurrentSegment*  rDeltaSegAngle );

		pVertex->x = x0;
		pVertex->y = 0.0f + ( height / 2.0f );
		pVertex->z = z0;

		float  tu0 = ( 0.5f* sinf( nCurrentSegment * rDeltaSegAngle ) ) + 0.5f;
		float  tv0 = ( 0.5f* cosf( nCurrentSegment * rDeltaSegAngle ) ) + 0.5f;

		pTex->x = tu0;
		pTex->y = tv0;

		pVertex++;
		pTex++;
		v++;
	}

	for ( nCurrentSegment = 0; nCurrentSegment < m_nSegments - 1; nCurrentSegment++ ){
		pTri->Set( topmid, topmid + 1+nCurrentSegment, topmid + 1+nCurrentSegment + 1 );
		pTri++;
	}

	//Last one, connect to the 1st one:
	pTri->Set( topmid, topmid + 1 + ( m_nSegments - 1 ), topmid + 1 );
	pTri++;

	/*	int bottommid = topmid+1+m_nSegments;

	//Create the bottom triangle fan: Center

	pVertex->x = 0.0f;
	pVertex->y = 0.0f - (height / 2.0f);
	pVertex->z = 0.0f;
	pTex->x = 0.5f;
	pTex->y = 0.5f;

	pVertex++;
	pTex++;
	v++;

	//#if 0
	//Create the bottom triangle fan: Edges
	for(nCurrentSegment = 0; nCurrentSegment < m_nSegments; nCurrentSegment++)
	{
	float x0 = radius * sinf(nCurrentSegment * rDeltaSegAngle);
	float z0 = radius * cosf(nCurrentSegment * rDeltaSegAngle);

	pVertex->x = x0;
	pVertex->y = 0.0f - (height / 2.0f);
	pVertex->z = z0;


	float tu0 = (0.5f * sinf(nCurrentSegment * rDeltaSegAngle)) + 0.5f;
	float tv0 = (0.5f * cosf(nCurrentSegment * rDeltaSegAngle)) + 0.5f;

	pTex->x = tu0;
	pTex->y = tv0;

	pVertex++;
	pTex++;
	v++;
	}

	for(nCurrentSegment = 0; nCurrentSegment < m_nSegments-1; nCurrentSegment++)
	{
	pTri->Set(bottommid, bottommid+1+nCurrentSegment, bottommid+1+nCurrentSegment+1);
	pTri++;
	}

	//Last one, connect to the 1st one:

	pTri->Set(bottommid, bottommid+1+(m_nSegments-1), bottommid+1);
	pTri++;*/

	/*#endif

	for(nCurrentSegment = m_nSegments-1-1; nCurrentSegment >= 0; nCurrentSegment--)
	{
	pTri->Set(bottommid, nCurrentSegment*2+1, (nCurrentSegment+1)*2+1);
	pTri++;
	}

	//Last one, connect to the 1st one:

	pTri->Set(bottommid, (m_nSegments-1)*2+1, 1);
	pTri++;*/

	//  factstate->CalculateNormals ();
	factstate->Invalidate();

	genmesh = csPtr< iMeshWrapper >  ( engine->CreateMeshWrapper( genmesh_fact, "genmesh", room, pos ) );
	if ( !genmesh )  {
		 LOG( "CSWorld", 1, "Can't make genmesh object!" );
		return genmesh;
	}
	csRef< iGeneralMeshState >  state( SCF_QUERY_INTERFACE( genmesh->GetMeshObject(), iGeneralMeshState ) );
	if ( !state )  {
		 LOG( "CSWorld", 1, "Strange, genmesh doesn't implement iGeneralMeshState!" );
		return genmesh;
	}

	state->SetLighting    ( true );
	state->SetManualColors( false );
	genmesh->SetZBufMode  ( CS_ZBUF_FILL );
	//genmesh->SetRenderPriority (engine->GetWallRenderPriority ());

	/*	csRef<iSprite3DFactoryState> factstate3D = SCF_QUERY_INTERFACE (genmesh_fact->GetMeshObjectFactory(), iSprite3DFactoryState);

	//	csRef<iSprite3DState> spriteState = SCF_QUERY_INTERFACE(genmesh->GetMeshObject(), iSprite3DState );
	iSpriteSocket* socket = factstate3D->AddSocket();
	socket->SetName("tag");*/

	genmesh->GetMovable()->SetPosition( pos );
	genmesh->GetMovable()->UpdateMove();

	// Registration
	if ( !inert ){
		CSObject o;
		o.startpos = pos;
		o.volume   = CylVolume( radius, height );
		o.meshtype = "cylinder";
		o.label    = name;

		TheSimWorld.AddMovableInfo( o );
	}
	else if ( collidable )	RegisterInert( name );

	return genmesh;
}

//----------------------------------------------------------------------------------------------------------------------------
float CSWorld::CylVolume( float radius, float h )const{
	return h* PI *( radius* radius );
}
#else
csRef<iMeshWrapper>  CSWorld::InsertBall( csVector3 pos, std::string name, std::string materialname, 
										  float radiusX, float radiusY, float radiusZ, bool inert, bool collidable)
{
	return InsertBox(pos,name,materialname,csVector3(radiusX*2, radiusY*2, radiusZ*2),inert,collidable);
}
csRef< iMeshWrapper >  CSWorld::InsertBox ( csVector3 pos, std::string name, std::string materialname, 
										  csVector3 dim, bool inert,bool collidable )
{
	float dz = 0.02;
	if (inert)  dz = 0.001;
    
	if ( dim.x+dz > FloatConfig( "RoomSizeX" ) || 
		 dim.y+dz > FloatConfig( "RoomSizeY" ) || 
		 dim.z+dz > FloatConfig( "RoomSizeZ" ) )
	{
		 LOG("CSWorld", 1, "Inserted object doesn't fit the room.");
		csRef<iMeshWrapper>  emptyw;

		return emptyw;
	}
	
	float  minH = dim.y / 2 + dz;
	float  maxH = FloatConfig( "RoomSizeY" ) - dim.y/2;

	if (pos.y < minH)  pos.y = minH; //Don't touch the ground.
	if (pos.y > maxH)  pos.y = maxH; //Don't touch the ceiling.
	
	iEngine*       engine = FROM_CS(iEngine);
	csRef<iMeshWrapper>  genmesh= engine->CreateMeshWrapper( materialname, name.c_str(), pos, dim );

  	genmesh->GetMovable()->SetPosition( pos );
//	genmesh->GetMovable()->UpdateMove ();

	// Registration	
	if ( !inert )  {
		CSObject  o;
		o.startpos = pos;
		o.volume   = dim.x*dim.y*dim.z;
		o.meshtype = "box";
		o.label    = name;
		
		TheSimWorld.AddMovableInfo(o);
	}
	else if ( collidable )  RegisterInert( name );

  return genmesh;
}

#endif

//----------------------------------------------------------------------------------------------------------------------------
float CSWorld::BallVolume( float radius )const{
	return ( 4.0f / 3 )* 3.141 *( radius* radius * radius );
}

//----------------------------------------------------------------------------------------------------------------------------
void CSWorld::RegisterAll( bool allow_re_registration )
{
	iEngine*  engine = FROM_CS( iEngine );
	iMeshList*        meshes = engine->GetMeshes();

	if ( !meshes ){
		 LOG( "CSWorld", 1, "NO MESHES! Should MakeWorld() first!" );
		return ;
	}

	for ( int i = 0; i < meshes->GetCount(); i++ )  {
		iMeshWrapper*  mesh = meshes->Get( i );
		string         name( mesh->QueryObject() ? mesh->QueryObject()->GetName(): "<null>" );
		string         proname = name;

		// Profile name is the lower-case and numberless
		// version of the mesh name (eg. "Ball1" => "ball")

		for ( unsigned int i = 0; i < proname.size(); i++ ) {
			if ( !isalpha( proname[i] ) )
				proname.erase( i, proname.size() - i );
		    else if ( proname[i] >= 'A' && proname[i] <= 'Z' )
				proname[i] += ( 'a' - 'A' );
		}

		if ( !IsInert( proname ) )  {
			 LOG( "CSWorld", 3, ("Registering " + name + " with " + proname ).c_str() );
			TheSimWorld.Register( name, proname, allow_re_registration );
		}
		else  { LOG( "CSWorld", 3, "Skipping reserved profile 'walls' initialization." ); }
	}
	TheSimWorld.Register( "Ambient", "ambient", allow_re_registration );
}

//----------------------------------------------------------------------------------------------------------------------------
CSWorld::~CSWorld(){
	 LOG( "CSWorld", 3, "Destroying..." );

	iEngine*  engine = FROM_CS( iEngine );
	engine->DeleteAll();

	 LOG( "CSWorld", 3, "Destroyed." );
}

#if CRYSTAL

//----------------------------------------------------------------------------------------------------------------------------
bool MapWorld::MakeWorld(){
	// Set VFS current directory to the level we want to load.
	csRef< iVFS >     VFS    = FROM_CS( iVFS );
	csRef< iLoader >  loader = FROM_CS( iLoader );
	iEngine*  engine = FROM_CS( iEngine );

    engine->SetLightingCacheMode( 0 );

#ifdef ENV_VAR

		VFS->ChDir ("/appdata/Worlds/");
		if (!loader->LoadMapFile( (StringConfig("world")).c_str()) ) {
			LOG("LocalServer", 0, "ENV_VAR:: Error: couldn't load CS map file ");
		}
		else  {
			LOG("LocalServer", 0, "ENV_VAR:: Sucess: Loaded CS map file ");
		}
#else

	// Load the level file which is called 'world'.
	VFS->ChDir( "/sim" );
	if ( !loader->LoadMapFile( ( "/sim/" + StringConfig( "world" ) ).c_str() ) ){
		 LOG( "LocalServer", 0, "Error: couldn't load CS map file " + StringConfig( "world" ) + 	" from the executable directory." );
		VFS->ChDir( StringConfig( "world" ).c_str() );

		if ( !loader->LoadMapFile( ( StringConfig( "world" ) + "/world" ).c_str() ) ) 
		{
			 LOG( "LocalServer", 0, string( "Error couldn't load level!" ) + "\nNext I'll try a CS map file, lev/flarge." );
			VFS->ChDir( "/lev/flarge" );

			if ( !loader->LoadMapFile( "/lev/flarge/world" ) )  {
				 LOG( "MapWorld", 0, "Error couldn't load level!" );
			}
		}
	}

#endif //ENV_VAR

  // TODO: Check if this is really needed. 	
  // Initialize collision objects for all loaded objects.
  // csColliderHelper::InitializeCollisionWrappers (CSserver::GetColliderSystem(), engine);

	engine->Prepare();

	// Find the starting position in this level.
	csVector3  pos( 0, 0, 0 );
	if ( engine->GetCameraPositions()->GetCount() > 0 ){
		// There is a valid starting position defined in the level file.
		iCameraPosition*  campos = engine->GetCameraPositions()->Get( 0 );

		room = engine->GetSectors()->FindByName( campos->GetSector() );
		pos  = campos->GetPosition();
	}
	else{
		// We didn't find a valid starting position. So we default to going to room called 'room' at position (0,0,0).
		room = engine->GetSectors()->FindByName( "room" );
	}
	 LOG( "MapWorld", 0, "World loaded." );

	iMeshList*  meshes = FROM_CS( iEngine )->GetMeshes();
	for ( int i = 0; i < meshes->GetCount(); i++ )  {
		iMeshWrapper*  mesh = meshes->Get( i );

		string  name( mesh->QueryObject() ? mesh->QueryObject()->GetName(): "<null>" );
		string  profile;

		// Everything is inert & collidable by default
		bool inert      = true;
		bool collidable = true;

		CSObject o;
		o.startpos = pos;
		#ifdef WIN32
			#pragma message ("Volume is unknown for map-loaded objects!")
		#else
			#warning "Volume is unknown for map-loaded objects!"
		#endif
		o.volume   = 0;
		o.label    = name;

		if ( strstr( name.c_str(), "Knob" ) )  {
			inert      = false;
			o.meshtype = "knob";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile knob (movable!)" );
		}
		else if ( strstr( name.c_str(), "Wall" ) )  {
			inert      = true;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else if ( strstr( name.c_str(), "Floor" ) )  {
			inert      = true; 
			o.meshtype = "floor";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Floor" );
		}
		else if ( strstr( name.c_str(), "Ceiling" ) )  {
			inert      = true;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else if ( strstr( name.c_str(), "Window" ) )  {
			inert      = true;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else if ( strstr( name.c_str(), "Door" ) )  {
			inert      = true;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else if ( strstr( name.c_str(), "BookCase" ) )  {
			inert      = true;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else if ( strstr( name.c_str(), "BookShelf" ) )  {
			inert      = true;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else if ( strstr( name.c_str(), "Painting" ) )  {
			inert      = true;
			collidable = false;
			o.meshtype = "wall";
			 LOG( "MapWorld", 0, name + "registered w/undefined profile Wall" );
		}
		else{
			o.meshtype = "map_mesh";
			 LOG( "MapWorld", 0, name + "registered w/o profile" );
		}

		if ( !inert )	        TheSimWorld.AddMovableInfo( o );
		else if ( collidable )  RegisterInert( name );

		// TheSimWorld.Register(name, profile, allow_re_registration);
	}

#if 1    
// MapWorld is also scripted    
    string  script;
    LoadTextFile( StringConfig( "SimScriptFilename" ).c_str(), script );
     LOG( "LocalScriptWorld", 2, ( string( "Start parsing..." ) + StringConfig( "SimScriptFilename" ) ).c_str() );
    // Will never be deleted, but who cares.
    LogReportProvider*  reporter = new LogReportProvider;

    StringTokenizer  lines( script, "\n" );

    for ( unsigned int i = 0; i < lines.size(); i++ )  {
        string  thisline = lines[i];
        CleanSpace( thisline );

        if ( thisline[0] == '#' )  continue;

        if ( !thisline.empty() )  {
             LOG( "CSWorld", 1, "Commencing: " + thisline );
            shared_ptr< Command >  com;
            if ( SelectCommand( thisline, reporter, com ) )  SimServer::Get()->PerformCommand( com );
             LOG( "CSWorld", 3, "Command ok." );
        }
    }

  // TODO: Check if this is really needed
  // Initialize collision objects for all loaded objects.
  // csColliderHelper::InitializeCollisionWrappers (CSserver::GetColliderSystem(), engine);
  
    engine->Prepare();
     LOG( "CSWorld", 2, "Visual CS World was created." );
#endif

	return true;
}

#endif

//---------------------------------------------------------------------------------------------------------------------------
void CSWorld::RegisterInert( string name )
{
	inertObjects.insert( name );
}

//---------------------------------------------------------------------------------------------------------------------------
bool CSWorld::IsInert( string name ) const
{
	return ( inertObjects.find( name ) != inertObjects.end() );
}

//---------------------------------------------------------------------------------------------------------------------------
void HardWorld::MakeWalls( string name, csVector3 corner1, csVector3 corner2 )
{
#if CRYSTAL
	iEngine* engine = FROM_CS( iEngine );

	iMaterialWrapper*      tm   = engine->GetMaterialList()->FindByName( "stone" );
	csRef< iMeshWrapper >  walls( engine->CreateSectorWallsMesh( room, name.c_str() ) );

	csRef< iThingFactoryState > walls_state = scfQueryInterface < iThingFactoryState > ( walls->GetMeshObject()	->GetFactory() );

	walls_state->AddInsideBox            ( corner1, corner2 );
	walls_state->SetPolygonMaterial      ( CS_POLYRANGE_LAST, tm );
	walls_state->SetPolygonTextureMapping( CS_POLYRANGE_LAST, 3 );
#endif
}

//---------------------------------------------------------------------------------------------------------------------------
bool HardWorld::MakeWorld()
{
	iEngine* engine = FROM_CS( iEngine );
	iLoader* loader = FROM_CS( iLoader );
	engine->SetLightingCacheMode( 0 );

	if ( !loader )  {
		 LOG( "Server", 1, "Loader failure." );
		return false;
	}

#if CRYSTAL
	room = engine->CreateSector( "room" );
#endif

/*
    if (room->SetVisibilityCullerPlugin("crystalspace.culling.dynavis")) {
        printf("VISIBILITY PLUGIN LOADED SUCCESSFULLY\n");
    } else {
        printf("*************************** ERROR LOADING VISIBILITY PLUGIN!!!\n");
    }
*/

	float  sx =  - FloatConfig( "RoomSizeX" ) / 2;
	float  ex =  + FloatConfig( "RoomSizeX" ) / 2;
	float  sy =  0; //-FloatConfig("RoomSizeY")/2;
	float  ey =  + FloatConfig( "RoomSizeY" );
	float  sz =  - FloatConfig( "RoomSizeZ" ) / 2;
	float  ez =  + FloatConfig( "RoomSizeZ" ) / 2;

	// And "infinitesimal" box is our layer of paint, of wall!
	float  dz = 0.01;

#if !CRYSTAL
	dz = 1;
#endif

	if (IntConfig("MakeWalls")) {
		MakeWalls    ( "walls", csVector3( sx, sy, sz ), csVector3( ex, ey, ez ) );
		RegisterInert( "walls" );

		InsertBox( csVector3( 0, FloatConfig( "RoomSizeY" ) / 2, sz + dz ), "S", "stone", csVector3( FloatConfig(
   				   "RoomSizeX" ) - dz, FloatConfig( "RoomSizeY" ) - dz, dz ), true, true );
		//RegisterInert("S");
		InsertBox( csVector3( sx + dz, FloatConfig( "RoomSizeY" ) / 2, 0 ), "W", "stone", csVector3( dz, FloatConfig(
   				   "RoomSizeY" ) - dz, FloatConfig( "RoomSizeZ" ) - dz ), true, true );
		InsertBox( csVector3( ex - dz, FloatConfig( "RoomSizeY" ) / 2, 0 ), "E", "stone", csVector3( dz, FloatConfig(
   				   "RoomSizeY" ) - dz, FloatConfig( "RoomSizeZ" ) - dz ), true, true );
		InsertBox( csVector3( 0, FloatConfig( "RoomSizeY" ) / 2, ez - dz ), "N", "stone", csVector3( FloatConfig(
   				   "RoomSizeX" ) - dz, FloatConfig( "RoomSizeY" ) - dz, dz ), true, true );
#if CRYSTAL
		csRef< iLight >  light;
		iLightList*      ll = room->GetLights();

		light = engine->CreateLight( 0, csVector3( ( ex + sx ) / 2, ey, ( ez + sz ) / 2 ), 20, csColor( 1, 1, 1 ) );
		ll->Add( light );
#endif
	} else {
        /* SPECIFIC LIGHTS MUST BE CREATED IN THE SCRIPT FILE
		csRef< iLight >  light;
		iLightList*      ll = room->GetLights();

	    light = engine->CreateLight( "demoLight1", csVector3( -1.5, 2.5, 3), 200, csColor( 0.5, 0.5, 0.5 ) );
    	ll->Add( light );
    	light = engine->CreateLight( "demoLight2", csVector3( -4.5, 2.5,-3), 200, csColor( 0.5, 0.5, 0.5 ) );
	    ll->Add( light );
    	//light = engine->CreateLight( "demoLight3", csVector3( -1.5, 2.5,-3), 200, csColor( 0.5, 0.5, 0.5 ) );
    	//ll->Add( light );
    	//light = engine->CreateLight( "demoLight4", csVector3( -4.5, 2.5, 3), 200, csColor( 0.5, 0.5, 0.5 ) );
    	//ll->Add( light );
         */
        engine->SetAmbientLight(csColor(0.1, 0.1, 0.1));
	}    

	return true;

}

//---------------------------------------------------------------------------------------------------------------------------
bool PredefinedWorld::MakeWorld()
{
	HardWorld::MakeWorld();

	InsertBall( csVector3( 2, .51, 2 ), "HeavyBall1", "stone", .5,.5,.5 );
	InsertBall( csVector3( 3, .51, 4 ), "LightBall1", "stone", .5,.5,.5 );

	string wallname = "MidWall";
	InsertBox    ( csVector3(  - FloatConfig( "RoomSizeX" ) / 2+3.5, 0.42, 1 ), wallname, "wood", csVector3( 1, 0.8, 1.95 ), true );
	RegisterInert( wallname );

	for ( int ifood = 0; ifood < 5; ifood++ )  {
		string                 name = "Food" + toString( ifood + 1 );
		csRef< iMeshWrapper >  mw   = InsertBall( csVector3( 0.5* ifood, .11, 1 ), name, "purple", .10, .10, .10 );
	}

/*	string wallN = "OWallN";
	string wallS = "OWallS";
	string wallE = "OWallE";
	string wallW = "OWallW";
	iEngine::Instance().UpdateMove(InsertBox    ( csVector3(-FloatConfig( "RoomSizeX" )/2+1.1, 0.42, 0), wallW, "stone", csVector3(1, 0.8, FloatConfig( "RoomSizeZ" )-0.1), true )->GetMovable());
	iEngine::Instance().UpdateMove(InsertBox    ( csVector3(+FloatConfig( "RoomSizeX" )/2-1.1, 0.42, 0), wallE, "stone", csVector3(1, 0.8, FloatConfig( "RoomSizeZ" )-0.1), true )->GetMovable());
	iEngine::Instance().UpdateMove(InsertBox    ( csVector3(0, 0.42, -FloatConfig( "RoomSizeZ" )/2+1.1), wallS, "stone", csVector3(FloatConfig( "RoomSizeX" )-0.1, 0.8, 1), true )->GetMovable());
	iEngine::Instance().UpdateMove(InsertBox    ( csVector3(0, 0.42, +FloatConfig( "RoomSizeZ" )/2-1.1), wallN, "stone", csVector3(FloatConfig( "RoomSizeX" )-0.1, 0.8, 1), true )->GetMovable());
*/
	//CSMeshTool cst;
	//cst.CreateSprites();
	FROM_CS(iEngine)->Prepare();
	LOG( "CSWorld", 2, "Visual CS World was created." );

	return true;
}

//---------------------------------------------------------------------------------------------------------------------------
set< string >  CSWorld::GetInertObjects() const
{
	return inertObjects;
}

//---------------------------------------------------------------------------------------------------------------------------
map< string, string >  CSWorld::LoadTextureFileNames() const
{
	map< string, string > ret;

	string buf;

#ifdef ENV_VAR
		char   fullpath[1024];//can be [_MAX_PATH]
		char*  AgisimEnv = getenv( "AGISIM" );
		if( AgisimEnv == NULL ) {
			 LOG("ENV_VAR", 0, "No Enviournment Variable Found. Can't load textures.def"); 
			return ret;
		}
		for(int i=0;i<=(int)strlen(AgisimEnv);i++){fullpath[i]=AgisimEnv[i];} //<=
#ifdef WIN32
		strcat( fullpath,"\\appdata\\defs\\textures.def" );
#else
		strcat( fullpath,"/appdata/defs/textures.def" );
#endif //WIN32
	if ( !LoadTextFile( fullpath, buf ) )  {
		 LOG( "CSWorld", 0, "Texture definition file not found." );
		return ret;
	}
#else
	if ( !LoadTextFile( "data/textures.def", buf ) )  {
		 LOG( "CSWorld", 0, "Texture definition file not found." );
		return ret;
	}
#endif //ENV_VAR

	XMLNode  xml( buf );
	int count = 0;

	XMLITERATE( xml, property )  {
		string nodename = ( *property )->TagData().name;
		ret[nodename]   = ( *property )->TagData().textcontent;

		 LOG( "CSWorld", 3, "Loaded texture: " + ret[nodename] );
		count++;
	}
	 LOG( "CSWorld", 1, toString( count ) + " textures loaded." );

	return ret;
}

//---------------------------------------------------------------------------------------------------------------------------
void CSWorld::Init(){
	 LOG( "CSWorld", 1, "Initializing..." );
#if CRYSTAL
	csRef< iLoader >  loader  = FROM_CS( iLoader );
	csRef< iVFS >     filesys = FROM_CS( iVFS );

	if ( !loader )  {
		 LOG( "Server", 1, "Loader failure." );
		return ;
	}
	if ( !filesys )  {
		 LOG( "Server", 1, "VFS failure." );
		return ;
	}

	csRef< iPluginManager >   pluginmanager = FROM_CS( iPluginManager );
	csRef< iMeshObjectType >  ball_type     = CS_QUERY_PLUGIN_CLASS( pluginmanager, "crystalspace.mesh.object.ball", iMeshObjectType );

	if ( !ball_type.IsValid() )
		ball_type = CS_LOAD_PLUGIN( pluginmanager, "crystalspace.mesh.object.ball", iMeshObjectType );
	if ( !ball_type.IsValid() )  {
		 LOG( "CSWorld", 1, "No ball type plug-in found!\n" );
	}
	else  ball_factory = ball_type->NewFactory();

        string simPath; // "../../appdata/"
        simPath = simPath + ".." + CS_PATH_SEPARATOR + ".." + CS_PATH_SEPARATOR + "appdata" + CS_PATH_SEPARATOR;
	bool  ok = FROM_CS( iVFS )->Mount( "sim", simPath.c_str());
	string worldsPath; // "../../appdata/Worlds/"
        worldsPath = simPath + "Worlds" + CS_PATH_SEPARATOR; 
        ok |= FROM_CS( iVFS )->Mount( "Worlds", worldsPath.c_str());
        ok |= FROM_CS( iVFS )->Mount( "sim/Worlds", worldsPath.c_str());
	string texturesPath; // "../../appdata/Worlds/Textures"
        texturesPath = worldsPath + "Textures" + CS_PATH_SEPARATOR; 
	ok |= FROM_CS( iVFS )->Mount( "Textures", texturesPath.c_str());
	ok |= FROM_CS( iVFS )->Mount( "sim/Textures", texturesPath.c_str());
	ok |= FROM_CS( iVFS )->Mount( "Worlds/Textures", texturesPath.c_str());
	ok |= FROM_CS( iVFS )->Mount( "sim/Worlds/Textures", texturesPath.c_str());

#ifdef ENV_VAR

	char fullpath[1024];//can be [_MAX_PATH]
	char * AgisimEnv = getenv( "AGISIM" );
    if( AgisimEnv != NULL )
	{
		for(int i=0;i<=(int)strlen(AgisimEnv);i++){fullpath[i]=AgisimEnv[i];} //<=
#ifdef WIN32
		strcat(fullpath,"\\appdata\\Worlds\\");
#else
		strcat(fullpath,"/appdata/Worlds/");
#endif //WIN32
		csRef<iVFS> VFS = FROM_CS(iVFS);
		ok |= VFS->Mount("appdata/Worlds", fullpath);
	}
	else  {
		LOG("ENV_VAR", 0, "No Enviournment Variable Found. Using default paths."); 
	}
#endif //ENV_VAR

	 LOG( "CSWorld", 0, FROM_CS( iVFS )->GetCwd() );
	csRef< iDataBuffer >   texpath = FROM_CS( iVFS )->GetRealPath( "/Textures/" );
	 LOG( "CSWorld", 0, texpath->GetData() );
	csRef< iStringArray >  files   = FROM_CS( iVFS )->FindFiles  ( "/Textures/" );
	#ifdef WIN32
		for ( int i = 0; i < ( int ) ( files->Length() ); i++ )
	#else
		for ( int i = 0; i < files->Length(); i++ )
	#endif
	{
		 LOG( "CSWorld", 0, files->Get( i ) );
	}

	if ( FROM_CS( iVFS )->Exists( "/Textures/" ) ){
		 LOG( "CSWorld", 0, "FOUND" );
	}
	else {
		 LOG( "CSWorld", 0, "Path Not found!" );
	}

	/*
	csRef<iStringArray> mounts = FROM_CS(iVFS)->GetMounts();
	for (int i = 0; i < mounts->Length(); i++)
	{
	LOG("CSWorld", 0, mounts->Get(i));
	csRef<iStringArray> paths = FROM_CS(iVFS)->GetRealMountPaths(mounts->Get(i));
	for (int ii = 0; ii < paths->Length(); ii++)
	LOG("CSWorld", 0, paths->Get(ii));
	}
	 */

	if ( !ok ){
		 LOG( "Server", 1, "VFS mounting unsuccessful." );
	}

	map< string, string >  textures = LoadTextureFileNames();

	for ( map< string, string > ::iterator  t = textures.begin(); t != textures.end(); t++ )  {
		if ( !loader->LoadTexture( t->first.c_str(), t->second.c_str() ) )  {
			LOG( "Server", 1, "Error loading " + t->first + "texture!" );
		}
	}
#endif
	 LOG( "CSWorld", 1, "Initialization ok." );
}

//---------------------------------------------------------------------------------------------------------------------------
CSWorld::CSWorld(){
	Init();
}

#if CRYSTAL
//---------------------------------------------------------------------------------------------------------------------------
csRef<iMeshWrapper> CSWorld::InsertMesh( csVector3 pos,
					 std::string model,
					 std::string name,
					 std::string materialname,
					 float size,
					 bool inert,
					 bool collidable)
{
  iEngine* engine = FROM_CS(iEngine);
  csRef<iLoader> loader = FROM_CS(iLoader);

  iBase *result;
  csRef<iMeshWrapper> mesh_w(NULL);
  csRef<iMeshFactoryWrapper> factory_w(NULL);
  bool no_lib = loader->Load(model.c_str(), result);


  if (!no_lib)
    {
      LOG("CSworld", 2, "Don't load library as mesh...");
    }
  factory_w = SCF_QUERY_INTERFACE(result, iMeshFactoryWrapper);
  if (!factory_w)
    {
      mesh_w = SCF_QUERY_INTERFACE(result, iMeshWrapper);
    }
  else
    {
      mesh_w = ( engine->CreateMeshWrapper(factory_w, name.c_str(), room, pos));
    }

  if (!mesh_w)
    {
      LOG("CSWorld", 1, (std::string("Couldn't load mesh:")+model).c_str());
      return csRef<iMeshWrapper>(NULL);
    }
	
  LOG("CSWorld", 2, "Creating meshobject: " + name);

  csRef<iMaterialWrapper> obj_mat = engine->FindMaterial(materialname.c_str());

  
  csRef<iSpriteCal3DFactoryState>cal3d (SCF_QUERY_INTERFACE(factory_w->GetMeshObjectFactory(), iSpriteCal3DFactoryState));


  if (cal3d)
    {
      LOG("CSworld", 2, "Mesh object is Cal3D type!");
      if (!nocase_equal(materialname.c_str(), "nullmaterial"))
	LOG("CSworld", 1, "Cal3D model material changing not yet implemented!");
    }
  else
    {
      csRef<iMeshObject> mesh = mesh_w->GetMeshObject();

      if (!nocase_equal(materialname.c_str(), "nullmaterial"))
	{
	  csRef<iThingState> tstate = SCF_QUERY_INTERFACE (mesh, iThingState);
	  tstate->ReplaceMaterial( engine->FindMaterial("checkboard"),obj_mat );
	  tstate->ReplaceMaterial( engine->FindMaterial("no"),obj_mat );
	}
    }

  // Try to scale object to wanted proportions...
  csBox3 bbox = mesh_w->GetWorldBoundingBox();
  csVector3 delta = bbox.GetSize();

  csMatrix3 hardtrans; hardtrans.Identity();
  hardtrans *= 1.0/size;
  csReversibleTransform transu(hardtrans, csVector3(0, 0, 0));
	
  csRef<iMeshObject> mesh = mesh_w->GetMeshObject();
  csRef<iMeshObjectFactory> fact = factory_w->GetMeshObjectFactory();
  if (mesh->SupportsHardTransform())
    {
      LOG("CSworld", 2, (std::string("Scaled mesh object by ")).c_str());
      mesh->HardTransform(transu);
    }
  else if (fact->SupportsHardTransform())
    {
      LOG("CSworld", 2, (std::string("Scaled mesh factory by ")).c_str());
    }
  else
    {
      LOG("CSworld", 2, "Mesh can't be scaled!");
    }

  mesh_w->GetMovable()->SetPosition(pos);
  mesh_w->GetMovable()->UpdateMove();

  if (!inert)
    {
      // Registration
	
      CSObject o;
      o.startpos = pos;
      o.volume = delta.x*delta.y*delta.z;
      o.meshtype = "sprite";
      o.label = name;
      
      TheSimWorld.AddMovableInfo(o);
    }
  else if (collidable)
    RegisterInert(name);

  return mesh_w;
}

#endif

//---------------------------------------------------------------------------------------------------------------------------
bool LocalScriptWorld::MakeWorld()
{
	HardWorld::MakeWorld();

	string  script;
	LoadTextFile( StringConfig( "SimScriptFilename" ).c_str(), script );
	 LOG( "LocalScriptWorld", 2, ( string( "Start parsing..." ) + StringConfig( "SimScriptFilename" ) ).c_str() );
	// Will never be deleted, but who cares.
	LogReportProvider*  reporter = new LogReportProvider;

	StringTokenizer  lines( script, "\n" );

	for ( unsigned int i = 0; i < lines.size(); i++ )  {
		string  thisline = lines[i];
		CleanSpace( thisline );

		if ( thisline[0] == '#' )  continue;

		if ( !thisline.empty() )  {
			 LOG( "CSWorld", 1, "Commencing: " + thisline );
			shared_ptr< Command >  com;
			if ( SelectCommand( thisline, reporter, com ) )  
				SimServer::Get()->PerformCommand( com ); 
			 LOG( "CSWorld", 3, "Command ok." );
		}
	}

  // TODO: Check if this is really needed
  // Initialize collision objects for all loaded objects.
  // csColliderHelper::InitializeCollisionWrappers (CSserver::GetColliderSystem(), engine);

	FROM_CS(iEngine)->Prepare();
	 LOG( "CSWorld", 2, "Visual CS World was created." );

	return true;
}

#if CRYSTAL
//-----------------------------------------------------------------------------------------------------------------
csRef< iView >  CSWorld::CreateView( csVector3  pos )
{
	 LOG( "CSWorld", 3, string( "Create view at " )+ toString(pos.x)+", " + toString(pos.y)+", " + toString(pos.z) );	
	iSector*  room = GetRoom();
	
	csRef<iEngine>      engine = FROM_CS( iEngine );
	csRef<iGraphics3D>  g3d    = FROM_CS( iGraphics3D );
	csRef<iGraphics2D>  g2d    = FROM_CS( iGraphics2D );
	
	csRef< iView >  view = csPtr<iView>( new csView( engine, g3d ) );
	
  	if ( !room ) {
		 LOG( "CSWorld", 0, "Can't find a valid starting position!" );
		return view;
	}
	else {
  		if ( engine->GetCameraPositions()->GetCount() > 0 )  {
	    	
			// There is a valid starting position defined in the level file.
    		iCameraPosition*  campos = engine->GetCameraPositions()->Get(0);
    		pos = campos->GetPosition ();
		}

	  	view->GetCamera()->SetSector( room );
		view->GetCamera()->GetTransform().SetOrigin( pos );	  		
  		view->SetRectangle( 0, 0, g2d->GetWidth(), g2d->GetHeight() );

		return view;
	}
}
#endif

//---------------------------------------------------------------------------------------------------------------------------
IMPLEMENT_BRIDGE( CSWorld,  )
	BRIDGE_PROVIDER( LocalScriptWorld, StringConfig( "World" ) == "scripted",  )
	BRIDGE_PROVIDER( PredefinedWorld, StringConfig( "World" ) == "pre",  )
#if CRYSTAL
	BRIDGE_PROVIDER(MapWorld, ( StringConfig( "World" ) != "scripted" && StringConfig( "World" ) != "pre" ),  )
#endif
END_BRIDGE
