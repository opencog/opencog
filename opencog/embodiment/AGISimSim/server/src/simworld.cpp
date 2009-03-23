/**

Copyright Ari A. Heljakka 03/2006
Not part of AGI-Sim.
Do not distribute.

*/

/***************************************************************************
 *  
 *
 *  Project: Novamente
 *
 ****************************************************************************/

#include "simcommon.h"
#include "space.h"
//#include "CSops.h"
#include "XMLNode.h"
#include "simworld.h"

ProfileMap  SimWorld::reg;

//------------------------------------------------------------------------------------------------------------------
void SimWorldLog(std::string e)
{
	 LOG( "SimWorld", 3, e.c_str());
}

//---------------------------------------------------------------------------------------------------------------
CSObject::CSObject (string  _label, string  _meshtype, float  _volume,	csVector3  _startpos)
				   : label( _label ), meshtype( _meshtype ), volume( _volume ),	startpos( _startpos )  
{ 
    phi = 0;
}

//---------------------------------------------------------------------------------------------------------------
SimWorld::~SimWorld()
{
	reg.clear  ();
	SimWorldLog( "Destroyed." );
}

//----------------------------------------------------------------------------------------------------------------
CSObject SimWorld::GetMovable(std::string  label) const
{
	CSObject  ret;
	
	map< string, CSObject >::const_iterator  i = objs.find( label );
	if ( i != objs.end() )  ret = i->second;
		
	return ret;
}

//---------------------------------------------------------------------------------------------------------------
bool SimWorld::IsMovable(std::string label) const
{
	return STLhas(objs, label);
}

//---------------------------------------------------------------------------------------------------------------
void SimWorld::AddMovableInfo(const CSObject& o)
{
	 LOG( "SimWorld", 4, "Movable: " + o.label );
     //printf("SimWorld::AddMovableInfo: phi = %f\n" , o.phi);
	
	movablenames.insert(o.label);
	objs[o.label] = o;
    
    //CSObject o1 = GetMovable(o.label);
    // printf("SimWorld::AddMovableInfo: stored phi = %f\n" , o1.phi);
}

//---------------------------------------------------------------------------------------------------------------
set< std::string >  SimWorld::GetMovableNames() const
{ 
	return movablenames; 
}

//---------------------------------------------------------------------------------------------------------------
bool  less_mprofile::operator() ( const mprofile&  oa, const mprofile&  ob) const
{
	std::string  a = oa.GetName();
	std::string  b = ob.GetName();
	return a < b;
}

//---------------------------------------------------------------------------------------------------------------
bool  SimWorld::RegisterProfile( mprofile  profilename, WorldObjectProperty  profile)
{
	if ( !STLhas(reg, profilename) )  {
		reg[profilename] = profile;
		return true;
	}
	else  {
		SimWorldLog( "Warning! Multiply registering the same profile name: " + profilename.GetName() );		
		return false;
	}
}

//---------------------------------------------------------------------------------------------------------------
bool  SimWorld::Register(std::string meshname, std::string profilename,
						bool allow_re_registration)
{
	if (allow_re_registration || !STLhas(pmap, meshname))
	{
		WorldObjectProperty  newprop = reg[mprofile(profilename)];
		pmap[meshname] = newprop;
		
		for (unsigned int s = 0; s < pmap[meshname].sound.size(); s++)  {		
			pmap[meshname].sound[s].source = ( meshname ); //Bad design here.
		}

		pmap[meshname].smell.source = meshname; //Bad design here.
		return true;
	}
	else  {
		SimWorldLog( "Warning! Multiply registering the same mesh name: " + meshname);	
		return false;
	}
}

//---------------------------------------------------------------------------------------------------------------
ProfileMap  SimWorld::GetProfileMap() const
{
	return reg;
}

//---------------------------------------------------------------------------------------------------------------
Sensation::Sensation( XMLNode&  node )
{
	try {
		XMLITERATE( node, property )  {
			string  nodename = (*property)->TagData().name;
			if ( nodename != WINTENSITY  &&  nodename != WQUALITY )  {  
				LOG( "Sensation", 1, "Bad sensation entry." );  
			}
			else  Set( nodename, (*property)->TagData().textcontent );
		}
	} catch(...) { }	
}

//---------------------------------------------------------------------------------------------------------------
struct WorldObject
{
	std::string  name;
	WorldObjectProperty  args;
};

//---------------------------------------------------------------------------------------------------------------
bool SimWorld::Load(std::string path)
{
	try  {
		 SimWorldLog( "Loading world definition file: " + path);

#ifdef ENV_VAR
		char fullpath[1024];//can be [_MAX_PATH]
		char * AgisimEnv = getenv( "AGISIM" );
		if( AgisimEnv == NULL )  {
			 LOG("ENV_VAR", 0, "No Enviournment Variable Found. Can't load simworld.def"); 
			return false;
		}
		for(int i=0;i<=(int)strlen(AgisimEnv);i++){fullpath[i]=AgisimEnv[i];} //<=
#ifdef WIN32
		strcat(fullpath,"\\appdata\\defs\\simworld.def");
#else
		strcat(fullpath,"/appdata/defs/simworld.def");
#endif //WIN32
		path = fullpath;
#else
		if ( path.empty() )  
			path = "simworld.def";
#endif //ENV_VAR

		FILE*  f = fopen( path.c_str(), "rt" );
		if ( f == NULL )  {
			 SimWorldLog( path + " wasn't a valid for file." );
			return false;
		}
		fseek ( f, 0L, SEEK_END );
		long  size = ftell(f);
		fseek ( f, 0L, SEEK_SET );
		
		char*  buf = new char[size+2];
		fread ( buf, size, 1, f );
		fclose( f );
		
		XMLNode  world( buf );		
		delete buf;
		
		if ( !world.ok ) {
			 SimWorldLog( path + " wasn't a valid (simple) XML file. Should only contain nested blocks." );
			return false;
		}
		
		XMLVALIDATE( world, WROOT,SimWorldLog( "World def: Bad root tag" ) );
		XMLITERATE ( world, o )  {
			WorldObject  wo;
			XMLVALIDATE( **o, WOBJ, SimWorldLog( "World def: Bad object tag" ) );
			XMLITERATE ( **o, ochild )  {
				XMLHANDLE( **ochild, WNAME )  {
					wo.name = (**ochild).TagData().textcontent;
					 SimWorldLog( "Object name: " + wo.name);
				}
				XMLHANDLE( **ochild, WARG )  {
					XMLITERATE( **ochild, argnode )  {
						XMLHANDLE(**argnode, WSOUND)  {
							if ( wo.name.empty() )
								 SimWorldLog( "Error: A name block must be provided BEFORE the sound entry." );
							else  {
								wo.args.sound.push_back( Sound(wo.name, **argnode) );
								 SimWorldLog( "Sound found." );
							}
						}
						XMLHANDLE( **argnode, WSMELL )  {
							if ( wo.name.empty() )
								 SimWorldLog( "Error: A name block must be provided BEFORE the smell entry." );
							else
								wo.args.smell = Smell(wo.name, **argnode);
						}
						XMLHANDLE( **argnode, WTASTE )  {
							wo.args.taste = Taste( **argnode );
							 SimWorldLog( "Taste found." );
						}			
						XMLHANDLE( **argnode, WENERGY )  {
							wo.args.Set ( WENERGY, (*argnode)->TagData().textcontent );
							 SimWorldLog( "Energy found." );
						}
						XMLHANDLE( **argnode, WDENSITY ) {
							wo.args.Set(WDENSITY, (*argnode)->TagData().textcontent);							
							 SimWorldLog( "Density found." );
						}
					}
				}
			}
			
			RegisterProfile( mprofile(wo.name), wo.args );
		}
	
		 SimWorldLog( "World definition loaded ok." );
	} catch(string s) {
		 SimWorldLog( "World file: " + s);
		return false;
	}
	catch(...) {
		 SimWorldLog( "World file failed." );
		return false;
	}
	return true;
}

//---------------------------------------------------------------------------------------------------------------
float SimWorld::GetWeight(std::string mesh_name)
{
	float  volume  = objs[mesh_name].volume;
	float  density = 0;

	pmap[mesh_name].Get( WDENSITY, density );

	if ( density < 0.01f )
		density = FloatConfig( "DefaultDensity" );
	
	// Return density x volume of a sphere
	return density*  volume;
}

