/*
 * opencog/embodiment/AGISimSim/server/include/simworld.h
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
