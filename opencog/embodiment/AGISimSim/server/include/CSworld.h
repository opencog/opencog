/***************************************************************************
 *  CS object manipulation and loading.        
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

#include "collisionsystem.h"
class CSbox;	

//------------------------------------------------------------------------------------------------------------
/** @class CSWorld
	\brief The crystal space geography (visual properties) loader.

The class is a bridge with three implementations:
a script-created world, a predefined world and a dynamically loaded map.

The variable \b StringConfig["World"] determines which one is used.
Use the value "pre" for the pre-defined hard-coded map,
the value "scripted" to create the objects in the file simscript.def;
otherwise use your CS map file name as the value.
NOTE: Using CS map file is _not_ recommended, since it generally won't */
//------------------------------------------------------------------------------------------------------------
class CSWorld : public CollisionSystem {
	DEFINE_BRIDGE(CSWorld);

protected:
	//	map<string, csVector3> object_positions;
	csRef<iMeshObjectFactory>  ball_factory;
	set<string> 			   inertObjects;
	iSector* 				   room;
    std::map<const char*, const char*> liftedObjMap;
    std::map<const char*, float> originalLiftedHeightMap;
	void Init();

public:	
	CSWorld ();
	virtual ~CSWorld();

	/** Attaches a profile to each initialized mesh object.
	The profile name is the mesh name converted to lower case and
	stripped of any numbers in the end.
	Eg. a mesh with a name "Myobject23" will be given the profile "myobject". */
	void RegisterAll(bool allow_re_registration = false);
	
	/** Return all objects to their original positions & orientations. */
	void ResetObjects();

	/** Create mesh object representing fov...
		@param fov_angle Simulated FOV
		@param near_z Near end of FOV structure
		@param far_z Far end of FOV structure */
	csRef<iMeshWrapper> VisibleFovMesh(float fov_angle, float near_z, float far_z);

    /** Get a mesh object. 
        @param name The meshname of the mesh object. */
	//THE FOLLOWING WILL CAUSE MEMORY SEG FAULT! MUST USE RAW POINTER HERE!
    //csRef<iMeshWrapper>  GetObject (std::string meshname);
	iMeshWrapper*  GetObject (std::string meshname);

#if CRYSTAL
    /** Rotate a mesh object. 
        @param mesh The mesh object to be rotated
        @param rotX The rotate value in X-axis 
        @param rotY The rotate value in Y-axis 
        @param rotZ The rotate value in Z-axis 
        */
    bool  RotateObject (csRef<iMeshWrapper> mesh, float rotX, float rotY, float rotZ);

    /** Apply rotation movement to the arm mesh of the object. 
        @param mesh The mesh object
        @param angle The rotate angle value to be applied (positive = up, negative = down).
        */
    bool  MoveArm (csRef<iMeshWrapper> mesh, float angle);

    /** Apply rotation movement to the leg mesh of the object. 
        @param mesh The mesh object
        @param angle The rotate angle value to be applied (positive = up, negative = down).
        */
    bool  MoveLeg (csRef<iMeshWrapper> mesh, float angle);
#endif

    /** Lift a mesh object. 
        @param actorMesh The mesh object that will make the action of lifting
        @param targetMesh The mesh object to be lifted
        @return true if the operation was successfully executed
        */
    bool  LiftObject (iMeshWrapper* actorMesh, iMeshWrapper* targetMesh);

    /** Update the position of Lifted objects from a actor mesh object. 
        @param actorMesh The mesh object that has lifted any object
        @param deltaX, deltaY, deltaZ: the relative changes in the position of the actor object
        */
    void  UpdateLiftedObject (iMeshWrapper* actorMesh, float deltaX, float deltaY, float deltaZ);

    /** Drop any lifted mesh object. 
        @param actorMesh The mesh object that will make the action of droping
        @return true if the operation was successfully executed
        */
    bool  DropObject (iMeshWrapper* actorMesh);

	/** Insert a mesh (without registering it). 
		@param pos The position in the room.
		@param meshtype The mesh type (eg. "ball")	*/
	csRef<iMeshWrapper>  InsertObject (csVector3 pos, std::string meshtype);

	/** Insert a ball mesh (without registering it). 
		@param pos The position in the room.
		@param name The mesh name.
		@param materialname The name of the texture. The texture must be loaded first.
		@param radiusXYZ The radius of the sphere in CS spatial units.
		@param inert Whether the mesh moves upon collision
		@param collidable Whether the mesh reacts at all to collision */
	csRef<iMeshWrapper>  InsertBall (csVector3 pos,	std::string name, std::string materialname,
									 float radiusX, float radiusY=0.0f, float radiusZ=0.0f,
									 bool inert = false,	bool collidable = true);

	/** Insert a cylinder mesh (without registering it). 
		@param pos The position in the room.
		@param name The mesh name.
		@param materialname The name of the texture. The texture must be loaded first.
		@param radius The radius of the cylinder in CS spatial units.
		@param h The height of the cylinder in CS spatial units.
		@param inert Whether the mesh moves upon collision
		@param collidable Whether the mesh reacts at all to collision 	*/
	csRef<iMeshWrapper>  InsertCyl(csVector3 pos, std::string name, std::string materialname,
								  float radius, float h,
								  bool inert = false, bool collidable = true);

	/** Insert a cube mesh (without registering it). 
		@param pos The position in the room.
		@param name The mesh name.
		@param materialname The name of the texture. The texture must be loaded first.
		@param dim The dimensions of the cube in CS spatial units.
		@param inert Whether the mesh moves upon collision
		@param collidable Whether the mesh reacts at all to collision*/
	csRef<iMeshWrapper>  InsertBox (csVector3 pos, std::string name, std::string materialname,
									csVector3 dim, bool inert = false, bool collidable = true);

	/** Insert mesh loaded from file.
		@param pos The position in the room
		@param model file with path
		@param materialname Material used with model
		@param size Scaling of normalized mesh. (Lenth from corner to corner is 1)
		@param inert Whether the mesh moves upon collision
		@param collidable Whether the mesh reacts at all to collision	**/
	csRef<iMeshWrapper>  InsertMesh (csVector3 pos,	std::string model, std::string name, std::string materialname, 
									 float size, bool inert = false, bool collidable = true);
											
	float 	 BallVolume (float radius) const;
	float 	 CylVolume  (float radius, float h) const;	
	iSector* GetRoom	() const;
	
	/** Load the color/material/texture names from texture.def */
	map<string, string> LoadTextureFileNames () const;
	
	/** Get the next free mesh name based on padding index number to the
	end of the argument. */
	string GetFreeName( string base);
	
	/** Create a neutral view to this sector */
	csRef<iView> CreateView (csVector3 pos);
	
	/** Tell the collision system to recognize the object with the given
	name as a static bouncer (ie. wall) */
	void RegisterInert (string name);
	
	bool IsInert(string name) const;
	set<string> GetInertObjects() const;

	/** The initial creation of the CS meshes of the world.	Varies for each implementation */
	virtual bool MakeWorld() = 0;
};

//------------------------------------------------------------------------------------------------------------
/** @class MapWorld
	\brief World loaded from Crystal Space XML format. Not recommended.*/
//------------------------------------------------------------------------------------------------------------
struct MapWorld : public CSWorld {
	bool MakeWorld();
};

//------------------------------------------------------------------------------------------------------------
/** @class HardWorld
	\brief Superclass of worlds at least partially hard-coded.*/
//------------------------------------------------------------------------------------------------------------
struct HardWorld : public CSWorld {
	void MakeWalls (string name, csVector3 corner1, csVector3 corner2);
	bool MakeWorld ();
};

//------------------------------------------------------------------------------------------------------------
/** @class LocalScriptWorld
	\brief The world loaded from 'simscript.def'. */
//------------------------------------------------------------------------------------------------------------
struct LocalScriptWorld : public HardWorld {
	bool MakeWorld();
};

//------------------------------------------------------------------------------------------------------------
/** @class PredefinedWorld
	\brief A hard-coded world with 2 big balls and 5 small ones. */
//------------------------------------------------------------------------------------------------------------
struct PredefinedWorld : public HardWorld {
	bool MakeWorld();
};
