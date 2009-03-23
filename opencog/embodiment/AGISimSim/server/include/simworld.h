/***************************************************************************
 *  Object sensation source management.        
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
 
#ifndef SIMWORLD_H
#define SIMWORLD_H

#include "world_vocabulary.h"
#include "sensation.h"
#include "singleton.h"

class SimWorld;

//------------------------------------------------------------------------------------------------------------
/** 'mprofile' class is _only_ meant to prevent us from accidentally
	using mesh names instead of profile names.
	Profiles for meshes are accessible thru SimWorld registry.*/
//------------------------------------------------------------------------------------------------------------
class mprofile {
	friend class SimWorld;
		
private:	
	std::string name;
	mprofile (std::string _name) : name(_name) { }
	
public:
	mprofile() {}	
	std::string GetName() const { return name; }
};

//------------------------------------------------------------------------------------------------------------
struct less_mprofile : public binary_function<mprofile, mprofile, bool> { //!!! was ist das ???
    bool operator()(const mprofile& oa, const mprofile& ob) const;
};

typedef map <mprofile, WorldObjectProperty, less_mprofile>  ProfileMap;
typedef map <std::string, WorldObjectProperty> 			   PropertyMap;

//------------------------------------------------------------------------------------------------------------
struct CSObject {
	CSObject() {}
	CSObject(string _label, string _meshtype, float _volume, csVector3 _startpos);
	
	string 	   label;
	string 	   meshtype;
	float  	   volume;
	csVector3  startpos;
    double     phi;
};

//------------------------------------------------------------------------------------------------------------
/** @class SimWorld
	\brief The storage of noumenal object data. */
//------------------------------------------------------------------------------------------------------------
#ifdef WIN32
class SimWorld : public Singleton<SimWorld> {
#else
struct SimWorld : public Singleton<SimWorld> {
#endif
protected:
	static ProfileMap  reg;
	set<std::string>   movablenames;
	map<std::string, CSObject>  objs;

public:
	virtual ~SimWorld();	

	/** Load a world definition file */
	bool Load(std::string path);

	/** The mapping from \b mesh \b names to their attached \b profiles */
	PropertyMap pmap;

	/** Create a new profile with a specific name */
	bool RegisterProfile(mprofile profilename, WorldObjectProperty profile);

	/** Attach a profile to a specific mesh */
	bool Register(	std::string meshname, std::string profilename, bool allow_re_registration = false);

	mprofile   GetProfile   (const std::string meshname) const;
	ProfileMap GetProfileMap() const;

	/** Calculates weight based on object volume and density */	
	float GetWeight (std::string mesh_name);

	/** A temporary helper function.
		Informs the system about a non-wall object in the world.
		\param label The label of the object.
		\param obj The SimObject data of the object.	*/
	void AddMovableInfo(const CSObject& obj);
	
	//void AddMovableInfo(std::string meshname, float radius);	
	set<std::string>  GetMovableNames() const;
	CSObject	      GetMovable (std::string label) const;
	bool			  IsMovable  (std::string label) const;
	float 			  GetRadius  (std::string s);
	
	friend class Singleton<SimWorld>;
};

#define TheSimWorld (SimWorld::Instance())

#endif
