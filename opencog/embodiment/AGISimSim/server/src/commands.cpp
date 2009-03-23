#include "command.h"
#include "space.h"
#include "simserver.h"
#include "CSagent.h"
#include "sensors.h"
#include "CSworld.h"
#include <stack>
#include <boost/thread/thread.hpp>

/*
#include "simcommon.h"
#include "CSserver.h"
#include "simworld.h"
#include "serversocket.h"
*/


//----------------------------------------------------------------------------------------------------------------
Command::Command( std::string  _name,  ReportProvider* _reporter)
				: reporter( _reporter ), name( _name )
{
	if ( reporter)  reporter->SetName(_name);
}

Command::~Command()
{ }

//----------------------------------------------------------------------------------------------------------------
void  Command::addArguments( std::string  _arguments)
{
	begin_us  =  getusec();	
	arguments = _arguments;
}
	
//----------------------------------------------------------------------------------------------------------------
ParseResult Command::parse() const
{
	return parse(arguments);
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandUpdate::parse( std::string  arguments ) const  {
         LOG( name,3,"Tokenizing..." );
        
        StringTokenizer  args( arguments, " " );
        
        if ( args.size() > 2 )  {
             reporter->SendError( "Too many arguments." );
            return PARSE_FAILED;
        }
        if ( args.size() < 2 )  {
             reporter->SendError( "Too few arguments." );
            return PARSE_FAILED;
        }
        
        string           objname = args[0];
        if ( nocase_equal( args[1].c_str(), "no_init")) {
            LOG( name,3,"'no_init' update command received. Just discarding it...");
            reporter->SendMsg( MSG_OK );        
            return PARSE_OK;
        }
	
        StringTokenizer  params( args[1], "&" );
#ifdef WIN32        
         LOG( name,3,"Update parameters: " + toString(params.size()) );
#else
         LOG( name,3,"Update parameters: " + toString(params.size()) );
#endif
        
        // Moves are given coordinate by coordinate, so we save them and commit them all at once in the end.
        map< string, csVector3 >  moves;
        
        for (uint  p = 0; p < params.size(); p++)  {
            StringTokenizer  elems( params[p], "=" );
            if ( elems.size() != 2 )  {
                 reporter->SendError( "Syntax error: " + params[p] );
                return PARSE_FAILED;
            }
            string  propertyname  = elems[0];
            string  propertyvalue = elems[1];   
            string  objname2      = objname;        
            
            StringTokenizer  objectparts( objname, "." );
            if ( objectparts.size() > 1 ) {
                objname  = objectparts[0];
                objname2 = objectparts[1];
            }
        
             LOG( name,3,"Updating device: " + objname + "." + objname2 );
            
            bool isDemon = false;
            for (int i = 0; i < SimServer::Get()->DemonCount(); i++) {
                shared_ptr< Demon >  demon = SimServer::Get()->GetDemon(i);
                if ( demon.get() && nocase_equal( objname.c_str(), demon->GetName().c_str())) {
                    std::map< std::string, shared_ptr<Sensor> >            ss = demon->GetSenses();                             
                    std::map< std::string, shared_ptr<Sensor> >::iterator  si = ss.find( toupper(objname2) );
                    if ( si != ss.end() )  {
                         LOG( "CommandUpdate", 3, propertyname + " = " + propertyvalue );                       
                        si->second->Set( propertyname, propertyvalue );
                    } else {
                         reporter->SendError( "Unknown device: "+objname2 );
                    }                   
                    isDemon = true;
                    break;
                }
            }
            if (!isDemon) {
                // LOG( "CommandUpdate",4,objname+": "+propertyname+" = "+propertyvalue);
                string  sensationName = propertyname;
                string  propName;
                StringTokenizer  proparts(propertyname, "." );
                if ( proparts.size() == 2 )  {
                    sensationName = proparts[0];
                    propName      = proparts[1];
                }
                
                 LOG( "CommandUpdate",1,sensationName+"."+propName+" = "+propertyvalue);

                if ( STLhas(TheSimWorld.pmap, objname) )  {
                    if ( sensationName == WSMELL )
                        TheSimWorld.pmap[objname].smell.Set( propName, propertyvalue );
                    else if ( sensationName == WTASTE )
                        TheSimWorld.pmap[objname].taste.Set( propName, propertyvalue );
                    else if ( sensationName.substr( 0, strlen(WSOUND) ) == WSOUND  )  {
                        StringTokenizer  soundtok( sensationName, "[]" );
                        int              sound_index = 0;
                        vector<Sound>&   sound       = TheSimWorld.pmap[objname].sound;
                        if ( soundtok.size() == 2 )
                            sound_index = _Int(soundtok[1]);
#ifdef WIN32
                        if ( sound.size() <= (uint)sound_index )
#else
                        if ( sound.size() <= (unsigned int) sound_index )
#endif
                        {
                             reporter->SendError( "Bad sound index (was fixed): " + toString( sound_index ) );
#ifdef WIN32
                            sound_index = (int)sound.size() - 1;
#else
                            sound_index =      sound.size() - 1;
#endif
                        }
                        if ( sound_index >= 0)  sound[sound_index].Set( propName, propertyvalue );
                        else  {                 reporter->SendError( "Bad sound index (was fixed): " + toString(sound_index)); }
                    }
                    else if ( toupper( sensationName ) == "X" )  moves[objname].x = atof( propertyvalue.c_str() );
                    else if ( toupper( sensationName ) == "Y" )  moves[objname].y = atof( propertyvalue.c_str() );
                    else if ( toupper( sensationName ) == "Z" )  moves[objname].z = atof( propertyvalue.c_str() );
                    else
                        TheSimWorld.pmap[objname].Set( sensationName, propertyvalue );
                }
            }
        }
        
        for (map< string, csVector3 >::iterator  m = moves.begin(); m!=moves.end(); m++)  {
            iEngine*  engine = FROM_CS( iEngine );
            iMeshWrapper*   mesh   = engine->FindMeshObject( m->first.c_str() );
            if ( mesh )  {
                /// Verify that the height remains sane.
                csVector3  oldpos = mesh->GetMovable()->GetPosition();
                csVector3  newpos = m->second;
                newpos.y = oldpos.y;
                
                mesh->GetMovable()->SetPosition( newpos );
                mesh->GetMovable()->UpdateMove ();
            } else {
                 reporter->SendError( "ERROR: Moving an invalid object: " + m->first);
                return PARSE_FAILED;
            }
        }
        if ( !moves.empty() ) {
            for (int i = 0; i < SimServer::Get()->DemonCount(); i++ )  {
                SimServer::Get()->GetDemon(i)->UpdateView();
                SimServer::Get()->GetDemon(i)->ForceSensorUpdate();
            }
        }       
        reporter->SendMsg( MSG_OK );        

        return PARSE_OK;
}

//--- The valid form is "new objectname" ---------------------------------------------------------------------
ParseResult CommandNew::parse( std::string  arguments ) const
    {
         LOG( "CommandNew", 0, "parse..." );
        StringTokenizer tok(arguments, "\t\n\r ,();" );
        vector<string> a = tok.WithoutEmpty();
        string init_string;
      
        if ( !a.empty()) 
        {
             LOG(name, 3, "Creating object..." );
	    /* command parameters: 
	     * <x> <y> <z> <color> <mesh_type> <profile> <init_string> <others> 
	     * <others>: 
	     *  - when <mesh_type> ==  ball => radius
	     *  - when <mesh_type> ==  box => x_dim, y_dim, zdim 
	     */

            float  x = 0,  z = 0,  y = 0;
            string  color = "purple",  mesh_type = "ball",  profile = "Agent";
                
            float  other[a.size()-7];
#ifdef WIN32
            int    others = (int)a.size()-7;
#else
            int    others = a.size()-7;
#endif
          
            if ( a.size() >=8 )  { //0,0,0,purple,ball,Agent,init_str,...
                for (uint j = 7; j < a.size(); j++) {
                    if ( !nocase_equal(a[j].c_str(), "null" ))
                        other[j-7] = atof(a[j].c_str());
                }             
                 LOG(name, 3, "Dimension specifiers: " + toString(others));
            }
              
            if ( a.size() >=7)  { //0,0,0,purple,ball,Agent,init_str
                if ( !nocase_equal( a[6].c_str(), "null" ) )  {
                    init_string = a[6];                 
                     LOG(name, 3, "Initializer: " + init_string );
                }
            }
              
            if ( a.size() >=6)  {//0,0,0,purple,ball,Agent
                if ( !nocase_equal( a[5].c_str(), "null" ) )  {
                    profile = a[5];
                     LOG(name, 3, "Profile: " + profile);
                }
            }

            if ( a.size() >=5)  { //0,0,0,purple,ball
                if ( !nocase_equal( a[4].c_str(), "null" ) )  {
                    mesh_type = a[4];               
                     LOG( name, 3, "MeshType: " + mesh_type );
                }
            }
          
            if ( a.size() >=4 )  { //0,0,0,purple
                if ( !nocase_equal( a[3].c_str(), "null" ))
                {
                    color = a[3];
                     LOG( name, 3, "Color: " + color);
                }
            }

            if ( a.size() >=3)  { //0,0,0
                if ( !nocase_equal( a[2].c_str(), "null" ) )  {
                    z = atof( a[2].c_str() );
                     LOG( name, 3, "Z (mm): " + toString(z) );
                }
            }
              
            if ( a.size() >=2 ) { //0,0
                if ( !nocase_equal(a[1].c_str(), "null" ) )  {
                    y = atof( a[1].c_str());
                     LOG( name, 3, "Y (mm): " + toString(y) );
                }
            }

            if ( a.size() >=1)  { //0
                if ( !nocase_equal( a[0].c_str(), "null" ))
                {
                    if ( nocase_equal( a[0].c_str(), DEMON_NAME ) )
                        profile = DEMON_NAME;
                    else  {
                        x = atof(a[0].c_str());     
                         LOG(name, 3, "X (mm): " + toString(x) );
                    }
                }
            }
          
            bool  isdemon = ( nocase_equal( mesh_type.c_str(), DEMON_NAME ) ) || 
                            ( nocase_equal( profile.c_str()  , DEMON_NAME ) )  ||
                            ( nocase_equal( color.c_str()    , DEMON_NAME ) );
              
            bool  isball  = nocase_equal(mesh_type.c_str(), "ball" );
            bool  iscyl   = nocase_equal(mesh_type.c_str(), "cyl" );
            bool  isbox   = nocase_equal(mesh_type.c_str(), "box" );
            bool  ismesh  = false;
              
            // haxx:: Model recognized if name starts with standard path specifiers
            if ( mesh_type[0] == '.'  ||  mesh_type[0] == '/'  ||  color[0] == '/' )  {
                ismesh = true;
                 LOG( "CommandNew", 3, "Path recognized, try to load mesh..." );
            }
            if (ismesh) { ismesh=false; isball=true; puts("MESHes not supported!"); }
          
            // this mesh may be used for agent also..
            csRef< iMeshWrapper >  mesh;
            if ( !isdemon  &&  ( isball  ||  iscyl  ||  isbox  ||  ismesh) )  {
                 LOG( "!", 0, "isball || iscyl || isbox || ismesh" );
                string objname = (nocase_equal( profile.c_str(), AGENT_NAME) ? profile : CSWorld::Get().GetFreeName(profile));

                if ( ismesh )  {
#if CRYSTAL
                    if ( color[0] == '/' )  {
                        string       world = color;
                        csRef<iVFS>  vfs   = FROM_CS( iVFS );
                        
                        if ( vfs->Exists(world.c_str()) )
                        {

                          if (IntConfig( "LibraryLoading" ))
                          {
                        // Load the object from a model library file
                            mesh = CSMeshTool::Instance().ReadModels(world.c_str(), csVector3(x, y, z), mesh_type, objname);
                            if (mesh) {
                                CSObject o;
                                o.startpos = csVector3(x, y, z);
                                o.volume = 0;
                                o.meshtype = "mapload";
                                o.label = mesh_type;
                          
                                TheSimWorld.AddMovableInfo(o);
                            // TODO: Handle cases where the loaded mesh is inert.
                            //CSWorld::Get().RegisterInert(mesh_type);
                            }
                          }
                          else
                          {

                            CSMeshTool & tool = CSMeshTool::Instance();
                            if (!tool.LoadModelFrom(world.c_str(), mesh_type))
                            {
                              LOG("CSworld", 0, "Errorororor!");
                            }
                            iEngine* engine = FROM_CS(iEngine);
                            csRef<iMeshWrapper> mesh_w = tool.InsertInstance(mesh_type.c_str());
                          //                          tool.ReportGeom(mesh_w);

                        // TODO: "hack, no actual size defined!" 

                          // tool.ScaleByY(mesh_w, 0.1);
                            csMatrix3  hardtrans; hardtrans.Identity();
                            hardtrans *= 1.0/0.01;            
                            csReversibleTransform  transu (hardtrans, csVector3(0, 0, 0));
                            mesh_w->GetMeshObject()->HardTransform(transu);

                            csVector3 disp(0, 0, 0);
                            //disp = tool.ReCenter(mesh_w);
                            //disp += csVector3(x, y, z);
                            disp = csVector3(x, y, z);

                            mesh_w->GetMovable()->SetPosition(disp);
                            mesh_w->GetMovable()->UpdateMove();

                            tool.ReportGeom(mesh_w);

                            CSObject o;
                            o.startpos = csVector3(x, y, z);
                            o.volume = mesh_w->GetWorldBoundingBox().Volume();
                            o.meshtype = "mapload";
                            o.label = mesh_type;
                                  
                            TheSimWorld.AddMovableInfo(o);
                          }
                        }
                        else  { //if NOT ( vfs->Exists(world.c_str()) )
                            LOG( "!",0,"Mesh store not found!" );   
                        }
                    }
                    else  {
                         LOG( name, 1, "Insert mesh object" );
                        float  unitsize = 1.0;
                        if ( others >= 1 )  {
                            unitsize = other[0];
                        }
                        else {  
                            LOG( name, 0, "Mesh scalesize not specified, using value 1.0!" ); 
                        }
                        mesh = CSWorld::Get().InsertMesh( csVector3(x, y, z), mesh_type, objname, color, unitsize, 
                                                          nocase_equal( profile.c_str(), "static" ) );
                    }
#else 
                    // CRYSTAL required for mesh usage.
                    reporter->SendError( "Operation not supported." );
                    return PARSE_FAILED;
#endif
                }
                else if ( iscyl )  {
                     reporter->SendError( "Cylinders are not implemented." );
#if 0
                    float  h       = 0.1;
                    float  radius = 0.15;
                      
                    if ( others >= 2 )  h      = other[1];
                    if ( others >= 1 )  radius = other[0];
                      
                    y = 0.5 + h/2;                    
                     LOG( objname, 3, "Inserting cylinder with h=" + toString(h) + " & r=" + toString(radius) );
                      
                    mesh = CSWorld::Get().InsertCyl(csVector3(x,y,z), objname, color, radius,h);
#endif
                }
                else if ( isball )  {
                    float radius = FloatConfig("AgentBodyRadius");
                    
                    if ( others >= 1)  radius = other[0];
                    
                     LOG(name, 3, "Inserting ball with r=" + toString(radius));
                        
                    mesh = CSWorld::Get().InsertBall( csVector3(x, y, z), objname, color, radius, radius, radius, nocase_equal( profile.c_str(), "static" ) );  
                    y = mesh->GetMovable()->GetPosition().y;
                }                 
                else if ( isbox)  {
                    // TODO: Remove this hacky
                    csVector3 dim(0.5f,0.5f,0.5f);
                    if ( others >= 3 )  dim.z = other[2];
                    if ( others >= 2 )  dim.y = other[1];
                    if ( others >= 1 )  dim.x = other[0];
                        
                     LOG(name, 3, "Inserting box." );                       
                    mesh = CSWorld::Get().InsertBox(csVector3(x,y,z), objname, color, dim, nocase_equal(profile.c_str(), "static" ));
                    y = mesh->GetMovable()->GetPosition().y;
                }                 

                if ( mesh.get() == NULL )  return PARSE_FAILED;

                shared_ptr<CSAgent> csa;
                if (nocase_equal(init_string.c_str(), AGENT_NAME )) {
                     LOG(name, 1, "Trying to add an agent..." );

                    if ( !socket )  {
                         LOG(name, 1, "Tried to create an agent outside sockets!" );
                        return PARSE_FAILED;
                    }             
                  
                    if ( SimServer::Get()->AgentCount()   < IntConfig( "AgentMax" ) &&
                         SimServer::Get()->DemonCount() < IntConfig( "DemonMax" ) )  {
                        csVector3 startpos(x, y, z);

                        csa = SimServer::Get()->NewAgent(mesh,x,y,z, socket);
			csa->SetPort(reporter->GetSocket()->GetParent()->GetPort());
                         LOG( "CommandNew", 4, "Sending label..." );
                        reporter->SendMsg(mesh->QueryObject()->GetName());
                         LOG( "CommandNew", 4, "Label sent." );             
                    }
                    else  {
                        reporter->SendError( "Maximum number of agents reached." );
                        return PARSE_FAILED;
                    }
                }
		
                CSWorld::Get().RegisterAll ( false );
                if (IntConfig("CollisionDetection")) CSWorld::Get().InitCollider( mesh.get() );
                  
                if ( !init_string.empty() && !nocase_equal(init_string.c_str(), AGENT_NAME))  {
                     LOG(name, 3, "Initializing the profile." );
                    CommandUpdate upd(reporter);
                    upd.parse(objname + " " + init_string);
                }
                  
                //reporter->SendMsg(MSG_OK);
                reporter->SendMsg(mesh->QueryObject()->GetName());                
                return PARSE_OK;                  
            }
            else if ( isdemon )  
            {
                if ( !socket )  {
                     LOG(name, 1, "Tried to create a demon outside sockets!" );
                    return PARSE_FAILED;
                }             
                    
                if ( SimServer::Get()->DemonCount() < IntConfig( "DemonMax" ) )  {  
                    shared_ptr< Demon > demon = SimServer::Get()->NewDemon( profile, x, y, z, socket );
		    if (demon.get()) {
	                demon->SetPort( reporter->GetSocket()->GetParent()->GetPort() );
                         LOG( "CommandNew", 4, "Sending label..." );
                        reporter->SendMsg(demon->GetName());
                         LOG( "CommandNew", 4, "Label sent." );
                        return PARSE_OK;
                    }
                    else  {
                        reporter->SendError( "Got an internal error when tried to create a new Demon." );
                        return PARSE_FAILED;
                    }
                }
                else  {
                    reporter->SendError( "Maximum number of demons reached." );
                    return PARSE_FAILED;
                }
            }           
        }

         LOG( "CommandNew", 1, "ERROR: Invalid arguments: "+arguments);
        return PARSE_FAILED;
    }
    
    
//--- The valid form is "rotate objectname value" -------------------------------------------------------------
ParseResult CommandRotate::parse( std::string  arguments ) const
{
#if CRYSTAL
       LOG( "CommandRotate", 0, "parse..." );
      StringTokenizer   tok( arguments, "\t\n\r ,();" );
      vector< string >  a = tok.WithoutEmpty();
      string  init_string;
      
      if ( a.size() == 4 )  {
           LOG( name, 3, "Rotating object..." );
          string objname = a[0];
          float rotX = atof(a[1].c_str());
          float rotY = atof(a[2].c_str());
          float rotZ = atof(a[3].c_str());
          //printf( "arguments = %s %f %f %f\n", objname.c_str(), rotX, rotY, rotZ);
          
          csRef<iMeshWrapper>  mesh = CSWorld::Get().GetObject(objname);
          if ( mesh.get() != NULL ) {
              if ( CSWorld::Get().RotateObject( mesh, rotX, rotY, rotZ ) ) {
                  CSWorld::Get().UpdateLiftedObject(mesh, 0, 0, 0);
                  //CSproxy::Get()->ForceRelight(); Causing segfault
                  reporter->SendMsg(MSG_OK);
                  return PARSE_OK;
              } 
              else  {
                   LOG( "CommandRotate", 1, "ERROR: Could not rotate the object with value "+a[1]);
              }
          } 
          else {
               LOG( "CommandRotate", 1, "ERROR: Could not find object named "+a[0]);
          }
      }
       LOG( "CommandRotate", 1, "ERROR: Invalid arguments: "+arguments);
#else
      reporter->SendError( "Operation not supported." );
      return PARSE_FAILED;
#endif
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is "light lightName x y z radius"
ParseResult CommandLight::parse(std::string arguments) const
{
      LOG("CommandLight", 0, "parse...");
#if CRYSTAL
      StringTokenizer tok(arguments, "\t\n\r ,();");
      vector<string> a = tok.WithoutEmpty();
      string init_string;
      
      if (a.size() == 5)
        {
          LOG(name, 3, "Creating light...");
          printf("arguments = %s %s %s %s %s\n", a[0].c_str(), a[1].c_str(), a[2].c_str(), a[3].c_str(), a[4].c_str());
          char* lightName = (char*) a[0].c_str();
          float x = atof(a[1].c_str());
          float y = atof(a[2].c_str());
          float z = atof(a[3].c_str());
          float radius = atof(a[4].c_str());
          
          iEngine* engine = FROM_CS( iEngine );
          csRef<iSector> room = engine->FindSector( "room" );
          csRef< iLight >  light;
          iLightList*      ll = room->GetLights();
          light = engine->CreateLight( lightName, csVector3(x, y, z), radius, csColor( 0.5, 0.5, 0.5 ) );
          ll->Add( light );
          // engine->ForceRelight(light); Causing SegFault!!!
          reporter->SendMsg(MSG_OK);
          printf("Sent OK message\n");
          return PARSE_OK;
        }
      LOG("CommandLight", 1, "ERROR: Invalid arguments: "+arguments);
#endif
      return PARSE_FAILED;
}


//----------------------------------------------------------------------------------------------------------------
// The valid form is "lift actorObjName targetObjName"
ParseResult CommandLift::parse(std::string arguments) const
{
#if 1 //CRYSTAL
      LOG("CommandLift", 0, "parse...");
      StringTokenizer tok(arguments, "\t\n\r ,();");
      vector<string> a = tok.WithoutEmpty();
      string init_string;

      if (a.size() == 2)
        {
          LOG(name, 3, "Lifting object...");
          printf("arguments: actor = %s, target = %s \n", a[0].c_str(), a[1].c_str());
          char* actorObjName = (char*) a[0].c_str();
          char* targetObjName = (char*) a[1].c_str();

          iMeshWrapper*  actorMesh = CSWorld::Get().GetObject(actorObjName);
          if ( actorMesh != NULL ) {
              iMeshWrapper*  targetMesh = CSWorld::Get().GetObject(targetObjName);
              if ( targetMesh != NULL ) {
                  CSWorld::Get().LiftObject(actorMesh, targetMesh);
//                if (nocase_string(actorObjName) == nocase_string(AGENT_NAME))
                  reporter->SendMsg(MSG_OK);
                  return PARSE_OK;
              } 
              else  {
               LOG( "CommandLift", 1, "ERROR: Could not find object named "+a[1]);
              }
          } 
          else {
               LOG( "CommandLift", 1, "ERROR: Could not find object named "+a[0]);
          }
          
          reporter->SendMsg(MSG_OK);
          printf("Sent OK message\n");
          return PARSE_OK;
        }
      LOG("CommandLift", 1, "ERROR: Invalid arguments: "+arguments);
#endif
      return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is "Drop actorObjName"
ParseResult CommandDrop::parse(std::string arguments) const
{
#if CRYSTAL
      LOG("CommandDrop", 0, "parse...");
      StringTokenizer tok(arguments, "\t\n\r ,();");
      vector<string> a = tok.WithoutEmpty();
      string init_string;
      
      if (a.size() == 1)
        {
          LOG(name, 3, "Droping objects lifted by a given actor ...");
          printf("arguments: actor = %s\n", a[0].c_str());
          char* actorObjName = (char*) a[0].c_str();

          csRef<iMeshWrapper>  actorMesh = CSWorld::Get().GetObject(actorObjName);
          if ( actorMesh != NULL ) {
              CSWorld::Get().DropObject(actorMesh);
              reporter->SendMsg(MSG_OK);
              return PARSE_OK;
          } 
          else {
               LOG( "CommandRotate", 1, "ERROR: Could not find object named "+a[0]);
          }
          
          reporter->SendMsg(MSG_OK);
          printf("Sent OK message\n");
          return PARSE_OK;
        }
      LOG("CommandLight", 1, "ERROR: Invalid arguments: "+arguments);
#endif
      return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is just "relight" with no arguments
ParseResult CommandRelight::parse(std::string arguments) const
{
      LOG("CommandRelight", 0, "executing...");
      reporter->SendMsg(MSG_OK);
#if CRYSTAL
      CSproxy::Get()->ForceRelight();
#endif
      return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is just "relight" with no arguments
ParseResult CommandDump::parse(std::string arguments) const
    {
      LOG("CommandDump", 0, "executing...");

      iEngine::Instance().MeshDump();
      
      reporter->SendMsg(MSG_OK);
      return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandDelete::parse( std::string  arguments ) const
{
         LOG( "ParseDeleteCommand", 1, "arguments: "+arguments);
        StringTokenizer  tok( arguments, "\t\n\r ,();" );
        vector<string>   a = tok.WithoutEmpty();
        
        if ( a.empty() )  {
              LOG( "ParseDeleteCommand", 1, "Empty name" );
             reporter->SendError( "Empty name" );
            return PARSE_FAILED;
        }

        // Remove the perceptual properties
        TheSimWorld.pmap.erase( a[0] );
        
        if ( nocase_equal( a[0].c_str(), AGENT_NAME ) )  {
            if ( SimServer::Get()->AgentCount() > 0 )  {
                 LOG( "ParseDeleteCommand", 1, "Agent deletion ok" );
                SimServer::Get()->DeleteDemon(a[0]);
                 reporter->SendMsg(MSG_OK);     
            } 
            else {
                  LOG( "ParseDeleteCommand", 1, "Agent deletion failed" );
                 reporter->SendError( "No agents to kill." );
            }
            return PARSE_OK;
        }
        else if ( nocase_equal( a[0].c_str(), DEMON_NAME ) )  {
            if ( SimServer::Get()->DemonCount() > 0 )  {
                 LOG( "ParseDeleteCommand", 1, "Demon deletion ok" );
                SimServer::Get()->DeleteDemon(a[0]);
                reporter->SendMsg(MSG_OK);      
            } 
            else  {
                 LOG( "ParseDeleteCommand", 1, "Demon deletion failed" );
                reporter->SendError( "No demons to kill." );
            }
            
            return PARSE_OK;
        }
        else  {
            iEngine*  engine = FROM_CS( iEngine );
            iMeshWrapper*   mesh   = engine->FindMeshObject( a[0].c_str() );
            if ( mesh )  {
                 LOG( "ParseDeleteCommand", 1, "Object deletion ok" );
                reporter->SendMsg   ( MSG_OK );
                engine->RemoveObject( mesh );

                return PARSE_OK;
            }
            else  {
                  LOG( "ParseDeleteCommand", 1, "ERROR: Invalid object: " + a[0] );
                 reporter->SendError( "Invalid object" );
                return PARSE_FAILED;
            }
        }
        
          LOG( "ParseDeleteCommand", 1, "ERROR: Invalid argument: " + arguments);
         reporter->SendError( "Invalid argument" );
        return PARSE_FAILED;   
}


//----------------------------------------------------------------------------------------------------------------
//--- The valid form is "hand.forward objectname value" -------------------------------------------------------------
ParseResult CommandMoveArm::parse( std::string  arguments ) const
{
#if CRYSTAL
       LOG( "CommandMoveArm", 0, "parse..." );
      StringTokenizer   tok( arguments, "\t\n\r ,();" );
      vector< string >  a = tok.WithoutEmpty();
      string  init_string;
      
      if ( a.size() == 2 )  {
           LOG( name, 3, "Hand.Up of object..." );
          string objname = a[0];
          float angle = atof(a[1].c_str());
          ( "arguments = %s %f\n", objname.c_str(), angle);
          
          csRef<iMeshWrapper>  mesh = CSWorld::Get().GetObject(objname);
          if ( mesh != NULL ) {
              if ( CSWorld::Get().MoveArm( mesh, angle ) ) {
                  CSWorld::Get().UpdateLiftedObject(mesh, 0, 0, 0);
                  //CSproxy::Get()->ForceRelight(); Causing segfault
                  reporter->SendMsg(MSG_OK);
                  return PARSE_OK;
              } 
              else  {
                   LOG( "CommandMoveArm", 1, "ERROR: Could not apply hand.up to the object with value "+a[1]);
              }
          } 
          else {
               LOG( "CommandMoveArm", 1, "ERROR: Could not find object named "+a[0]);
          }
      }
       LOG( "CommandMoveArm", 1, "ERROR: Invalid arguments: "+arguments);
#endif
      return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
//--- The valid form is "hand.forward objectname value" -------------------------------------------------------------
ParseResult CommandMoveLeg::parse( std::string  arguments ) const
{
       LOG( "CommandMoveLeg", 0, "parse..." );
#if CRYSTAL
      StringTokenizer   tok( arguments, "\t\n\r ,();" );
      vector< string >  a = tok.WithoutEmpty();
      string  init_string;
      
      if ( a.size() == 2 )  {
           LOG( name, 3, "Hand.Up of object..." );
          string objname = a[0];
          float angle = atof(a[1].c_str());
          ( "arguments = %s %f\n", objname.c_str(), angle);
          
          csRef<iMeshWrapper>  mesh = CSWorld::Get().GetObject(objname);
          if ( mesh != NULL ) {
              if ( CSWorld::Get().MoveLeg( mesh, angle ) ) {
                  //CSproxy::Get()->ForceRelight(); Causing segfault
                  reporter->SendMsg(MSG_OK);
                  return PARSE_OK;
              } 
              else  {
                   LOG( "CommandMoveLeg", 1, "ERROR: Could not apply hand.up to the object with value "+a[1]);
              }
          } 
          else {
               LOG( "CommandMoveLeg", 1, "ERROR: Could not find object named "+a[0]);
          }
      }
       LOG( "CommandMoveLeg", 1, "ERROR: Invalid arguments: "+arguments);

#endif

      return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandMark::parse( std::string arguments ) const
{
    /** TODO */
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
//--- The valid form is "new objectname" ---------------------------------------------------------------------
    ParseResult CommandPaint::parse( std::string arguments ) const  {
#if CRYSTAL
        StringTokenizer  args(arguments, " \t," );
    
    if ( args.size() > 2 )  {
         reporter->SendError( "Too many arguments." );
        return PARSE_FAILED;
    }
    if ( args.size() < 2 )  {
         reporter->SendError( "Too few arguments." );
        return PARSE_FAILED;
    }
    
    iEngine*  engine = FROM_CS( iEngine );

    iMeshList*     meshes       = engine->GetMeshes();
    iMeshWrapper*  mesh         = meshes->FindByName( args[0].c_str() );
    string         materialname = args[1];
        
    if ( mesh )  {
        iMaterialWrapper*  mw = engine->FindMaterial( materialname.c_str() );
        if ( mw )  {
            mesh->GetMeshObject()->SetMaterialWrapper( mw );
             reporter->SendMsg( MSG_OK );
        }
        else  reporter->SendError( "Unknown color / material." );           
    }
    else  reporter->SendError( "Unknown target." );
#endif      
        return PARSE_OK;
    }

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandConfig::parse( std::string arguments ) const
{
    StringTokenizer  args( arguments, " =" );
    
    if ( args.size() > 2 )  {
         reporter->SendError( "Too many arguments." );
        return PARSE_FAILED;
    }
    if ( args.size() < 2)  {
         reporter->SendError( "Too few arguments." );
        return PARSE_FAILED;
    }
    Config::Instance().InsertVariable(args[0], args[1]);
    reporter->SendMsg(MSG_OK);
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandGetPos::parse( std::string  arguments ) const
{
    CleanSpace( arguments );
     LOG( "CommandGetPos",4,"Queried object: '" + arguments + "'" );
    iEngine*  engine = FROM_CS( iEngine );
    
    iMeshWrapper*   mesh   = engine->FindMeshObject( arguments.c_str() );
    if ( mesh )  {
        mesh->GetMovable()->UpdateMove();           
        csVector3  pos = mesh->GetMovable()->GetPosition();
        string posstr = "Position of "+arguments+": ( " + toString(pos.x) + "," + toString(pos.y) + "," + toString(pos.z) + " )";
        printf("Position = (%f,%f,%f)\n", pos.x, pos.y, pos.z);
        
        reporter->SendMsg( posstr );
         LOG( "CommandGetPos", 1, posstr );
    } else {
        reporter->SendError( "Unknown object.");
    }
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is "setpos objectname x y z"
ParseResult CommandSetPos::parse(std::string arguments) const
{
  LOG("CommandSetPos", 0, "parse...");
  StringTokenizer tok(arguments, "\t\n\r ,();");
  vector<string> a = tok.WithoutEmpty();
  string init_string;
  
  if (a.size() == 4) {
      LOG(name, 3, "Set possition of object...");
      printf("objetname: %s, position: (%s,%s,%s)\n", a[0].c_str(), a[1].c_str(), a[2].c_str(), a[3].c_str());
      
      iEngine* engine = FROM_CS(iEngine);
      const char* objName = a[0].c_str();
      iMeshWrapper* mesh = engine->FindMeshObject(objName);
      if (mesh) {
        csVector3 cur_pos = mesh->GetMovable()->GetPosition();
        float newX = atof(a[1].c_str());
        float newY = atof(a[2].c_str());
        float newZ = atof(a[3].c_str());
        mesh->GetMovable()->SetPosition(csVector3(newX, newY, newZ));

        float deltaX = newX - cur_pos.x;
        float deltaY = newY - cur_pos.y; 
        float deltaZ = newZ - cur_pos.z;
        CSWorld::Get().UpdateLiftedObject(mesh, deltaX, deltaY, deltaZ);

        for (int i = 0; i < SimServer::Get()->AgentCount(); i++) {
            shared_ptr<CSAgent> agent = SimServer::Get()->GetAgent(i);
            if (nocase_equal(objName, agent->getCSBody()->QueryObject()->GetName())) {
                agent->getRepresentation()->setPosition (newX, newY, newZ);
                break;
            }
        }

        mesh->GetMovable()->UpdateMove();
        reporter->SendMsg(MSG_OK);
        //CSproxy::Get()->ForceRelight(); causing segfault
        return PARSE_OK;
      } else {
          LOG("CommandSetPos", 1, "ERROR: Could not find object named "+a[0]);
          reporter->SendError( "Unknown object.");
      }
  }
  LOG("CommandSetPos", 1, "ERROR: Invalid arguments: "+arguments);
  reporter->SendError( "Invalid arguments.");
  return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is "getrot objectname"
ParseResult CommandGetRot::parse( std::string  arguments ) const
{
    CleanSpace( arguments );
     LOG( "CommandGetRot",4,"Queried object: '" + arguments + "'" );
    iEngine*  engine = FROM_CS( iEngine );
    
    iMeshWrapper*   mesh   = engine->FindMeshObject( arguments.c_str() );
    if ( mesh )  {
        mesh->GetMovable()->UpdateMove();           
        csVector3  rot = mesh->GetMovable()->GetRotation();
        string posstr = "Rotation of "+arguments+": ( " + toString(rot.x) + "," + toString(rot.y) + "," + toString(rot.z) + " )";
        printf("Rotation = (%f,%f,%f)\n", rot.x, rot.y, rot.z);
        
        reporter->SendMsg( posstr );
         LOG( "CommandGetRot", 1, posstr );
    } else {
        reporter->SendError( "Unknown object.");
    }
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is "setrot objectname x y z"
ParseResult CommandSetRot::parse(std::string arguments) const
{
  LOG("CommandSetRot", 0, "parse...");
  StringTokenizer tok(arguments, "\t\n\r ,();");
  vector<string> a = tok.WithoutEmpty();
  string init_string;
  
  if (a.size() == 4) {
      LOG(name, 3, "Set rotation of object...");
      printf("objetname: %s, rotation: (%s,%s,%s)\n", a[0].c_str(), a[1].c_str(), a[2].c_str(), a[3].c_str());
      
      iEngine* engine = FROM_CS(iEngine);
      const char* objName = a[0].c_str();
      iMeshWrapper* mesh = engine->FindMeshObject(objName);
      if (mesh) {
        float newX = atof(a[1].c_str());
        float newY = atof(a[2].c_str());
        float newZ = atof(a[3].c_str());
        mesh->GetMovable()->SetRotation(csVector3(newX, newY, newZ));

        for (int i = 0; i < SimServer::Get()->AgentCount(); i++) {
            shared_ptr<CSAgent> agent = SimServer::Get()->GetAgent(i);
            if (nocase_equal(objName, agent->getCSBody()->QueryObject()->GetName())) {
                float newPhi = newY; // For now, phi == y rotation
                agent->getRepresentation()->setOrientation(newX, newY, newZ, newPhi);
                break;
            }
        }

        mesh->GetMovable()->UpdateMove();
        reporter->SendMsg(MSG_OK);
        //CSproxy::Get()->ForceRelight(); causing segfault
        return PARSE_OK;
      } else {
          LOG("CommandSetRot", 1, "ERROR: Could not find object named "+a[0]);
          reporter->SendError( "Unknown object.");
      }
  }
  LOG("CommandSetRot", 1, "ERROR: Invalid arguments: "+arguments);
  reporter->SendError( "Invalid arguments.");
  return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
// The valid form is "getsize objectname"
ParseResult CommandGetSize::parse( std::string  arguments ) const
{
    CleanSpace( arguments );
     LOG( "CommandGetSize",4,"Queried object: '" + arguments + "'" );
    iEngine*  engine = FROM_CS( iEngine );
    
    iMeshWrapper*   mesh   = engine->FindMeshObject( arguments.c_str() );
    if ( mesh )  {
        csVector3  dim = mesh->dim;
        string posstr = "Size of "+arguments+": ( " + toString(dim.x) + "," + toString(dim.y) + "," + toString(dim.z) + " )";
        printf("Size = (%f,%f,%f)\n", dim.x, dim.y, dim.z);
        
        reporter->SendMsg( posstr );
         LOG( "CommandGetSize", 1, posstr );
    } else {
        reporter->SendError( "Unknown object.");
    }
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandExplain::parse( std::string  arguments ) const  {
    CleanSpace(arguments);
     LOG( "CommandExplain",4,"Queried object: '" + arguments + "'" );

    if ( STLhas( TheSimWorld.pmap, arguments ) )  {
         reporter->SendMsg( TheSimWorld.pmap[arguments].AsXML() );
        return PARSE_OK;
    }
    else  {
         reporter->SendError( arguments + " object not found." );
        return PARSE_FAILED;
    }
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandSense::parse( std::string arguments ) const  {
    if ( SimServer::Get()->DemonCount() == 0  &&  SimServer::Get()->AgentCount() == 0 )  {
         reporter->SendError( "No demons available." );
        return PARSE_FAILED;
    }
    
                
    shared_ptr< Demon >  demon = SimServer::Get()->GetDemonByPort( reporter->GetSocket()->GetParent()->GetPort() );		
    if (!demon.get()) {
        reporter->SendError( "Sense command error: could not find the demon associated to the given socket port");
        return PARSE_FAILED;
    }

    demon->ResendSensorData();
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandReset::parse( std::string  arguments ) const  
{
     LOG( "CommandReset", 3, "Reset with args " + arguments);       
    bool  hard = !( nocase_equal( arguments.substr( 0,4 ).c_str(), "soft" ) );      

    if ( hard )  { LOG( "CommandReset", 3, "HARD" ); }
    else         { LOG( "CommandReset", 3, "SOFT" ); }
    
     LOG( "CommandReset", 3, "Resetting Server/World..." );
    SimServer::Get()->ResetWorld( hard );
    reporter->SendMsg( MSG_OK );
     LOG( "CommandReset", 3, "Reset ok." );     

    return PARSE_OK;            
}

//------------------------------------------------------------------------------------------------------------
/*    
ParseResult CommandQuit::parse(std::string arguments) const
{
    socket->Send( "BYE\n\n" );
    socket->SetCloseAndDelete();
    
     LOG( "CommandQuit", 0, "The socket has quitted." );
    
    return PARSE_OK;
}
*/

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandListen::parse( std::string  arguments ) const
{
    if ( !socket )  {
         LOG( "CommandListen", 1, "'Listen' called outside sockets!" );
        reporter->SendError( "'Listen' called outside sockets!" );
        return PARSE_FAILED;
    }
    if ( arguments.empty() )  {
         LOG( "CommandListen", 1, "'Listen' called without arguments!" );
        reporter->SendError( "'Listen' called without arguments!" );
        return PARSE_FAILED;
    }
    
    StringTokenizer  tok( arguments, "\t\n\r ,();" );
    
    CSWorld::Get().ListenObjectPosition( socket, tok[0] );
    
     LOG( "CommandListen", 1, "Listening to '" + arguments + "'" );     
     reporter->SendMsg( MSG_OK );
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandGraphDump::parse( std::string  arguments ) const
{
//      reporter->SendMsg( MSG_OK );
    iEngine::Instance().graphDump();
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
ParseResult CommandBroadcast::parse( std::string  arguments ) const
{
    if ( !socket )  {
         LOG( "CommandBroadcast", 1, "'Broadcast' called outside sockets!" );
        return PARSE_FAILED;
    }
    if ( arguments.empty() )  {
         LOG( "CommandListen", 1, "'Broadcast' called without arguments!" );
        return PARSE_FAILED;
    }
    
    // Broadcast adds custom sensation to all demons
    int port = socket->GetParent()->GetPort(); 
    shared_ptr<Demon> sourceDemon = SimServer::Get()->GetDemonByPort(port);
    std::string message = sourceDemon->GetName() + ": " + arguments;
    printf("***************** BROADCAST message = %s\n", message.c_str());
     
    for (int  i = 0; i < SimServer::Get()->DemonCount(); i++) {
        shared_ptr<Demon> demon = SimServer::Get()->GetDemon(i);
        // Check if this demon has socket before adding sensations to it.
        if (demon->GetSocket()) {
            printf( "ADDING CUSTOM SENSATION IN AGENT %d (port = %d)\n", i, demon->GetSocket()->GetPort());
            demon->AddSensation( CustomSensation( message, 1, CUSTOM_SENSATION_BROADCAST_MESSAGE ) );
        }
    }

    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
bool  CommandAction::validCommand( std::string  objectname ) const
{
    return true; //nocase_equal(objectname.c_str(), "" );
}

//----------------------------------------------------------------------------------------------------------------
bool  CommandAction::validMove( int  dx ) const  { 
    return ( dx > -FloatConfig( "MaxMoveLength" )  &&  dx < FloatConfig( "MaxMoveLength" ) ); 
}

static int command_action_count = 0;

//----------------------------------------------------------------------------------------------------------------
//--- The valid form is "action (action name, action paramater) )" -------------------------------------------
ParseResult CommandAction::parse(std::string arguments) const
{
     LOG( "CommandAction", 2, arguments);
	printf("ACTION Commands processed so far: %d\n", command_action_count++);
    try {       
        if ( SimServer::Get()->DemonCount() == 0  && 
             SimServer::Get()->AgentCount() == 0 )
        {
             reporter->SendError( "No demons available." );
            return PARSE_FAILED;
        }
        
        if ( !reporter->GetSocket() )  {
             reporter->SendError( "Reported is not attached to a demon!" );
            return PARSE_FAILED;
        }
        
        shared_ptr< Demon >  demon = SimServer::Get()->GetDemonByPort( reporter->GetSocket()->GetParent()->GetPort() );
	printf("reporter->GetSocket() = %p\n", reporter->GetSocket());
	printf("reporter->GetSocket()->GetParent() = %p\n", reporter->GetSocket()->GetParent());
	printf("reporter->GetSocket()->GetParent()->GetPort() = %d\n", reporter->GetSocket()->GetParent()->GetPort());
	printf("demon = %p\n", demon.get());
        
         LOG( "CommandAction", 1, "About to tokenize... at port " + toString(reporter->GetSocket()->GetParent()->GetPort()));
        StringTokenizer      tok( arguments, "\t\n\r ,();" );
        vector<string>       a     = tok.WithoutEmpty();
        
        if ( !a.empty() && validCommand(a[0]))
        {
             LOG( "CommandAction", 3, "About to call agent action..." );                
            ParseResult result;
            if ( a.size() <= 4)        result = demon->action(a);
            else                       reporter->SendError( "WARNING: Too many arguments on socket command: " + arguments);
             LOG( "CommandAction", 3, "Agent action done." );

             if (IntConfig("DrawGUI"))
                 iEngine::Instance().graphDump();

            return result;
        }

    } catch(...)  { LOG( "CommandAction",0,"parse failure" ); }
    return PARSE_FAILED;
}

//----------------------------------------------------------------------------------------------------------------
//--- The valid form is "action (action name, action paramater) )" -------------------------------------------
ParseResult CommandMeshes::parse( std::string arguments ) const
{
    iEngine*  engine = FROM_CS( iEngine );
    iMeshList*        meshes = engine->GetMeshes();
    string            mlist  = "Listing all mesh names: ";
    for (int i = 0; i < meshes->GetCount(); i++)  {
        iMeshWrapper*  mesh = meshes->Get(i);
        if ( mesh->QueryObject() )  {
	    if (i > 0) mlist +=  ", ";
            mlist += mesh->QueryObject()->GetName();
        }
    }                
    printf("%s", mlist.c_str());
    reporter->SendMsg( mlist ); 
    
    return PARSE_OK;
}

//----------------------------------------------------------------------------------------------------------------
float RoomScaleFactor = 100.0f;

//----------------------------------------------------------------------------------------------------------------
bool  ContainedIn( const csVector3&  v, float  x, float  y, float  z)
{
    return ( v.x > - x+0.1  &&  v.y > 0.0001  &&  v.z > - z+0.1  &&  v.x <  x-0.1  &&   v.y < y - 0.0001  &&  v.z < z - 0.1);
}

//----------------------------------------------------------------------------------------------------------------
struct dcoord {
    dcoord() 
          : x(0), y(0)  {
    }

    dcoord( int  _x, int  _y)
          : x(_x), y(_y)  {
    }       

    bool  operator < ( const dcoord&  rhs ) const {
        return x < rhs.x  ||  ( rhs.x == x  &&  y < rhs.y );
    }

    int  x, y;
};

//----------------------------------------------------------------------------------------------------------------
bool  within( const dcoord&  c, int  w, int  h, int  body_units )
{
#ifdef WIN32
    return c.x-body_units/2 >=0  &&  c.y >=0  &&  c.x+body_units/2< w  &&  c.y < h;

#else
    return c.x-body_units/2 >=0  &&  c.y >=0  &&  
           c.x+body_units/2 < w  &&  
	   c.y < h;
#endif
}

//----------------------------------------------------------------------------------------------------------------
bool  FreeArea( bool**  F, const dcoord&  next, int  body_units)
{
    int  y = next.y;

#ifdef WIN32
    for (int  x = next.x-body_units/2; x < next.x+body_units/2; x++)
#else
    for (int   x = next.x-body_units/2; x < next.x+body_units/2; x++)
#endif
            //      for (int y = next.y-body_units/2; y < next.y+body_units/2; y++)
            if ( !F[x][y] )
                return false;

    return true;
}

//----------------------------------------------------------------------------------------------------------------
bool  find_path( const dcoord&  start, const dcoord&  target, bool**  F, int  w, int  h, int  body_units,
                 stack< dcoord >&  path,    set< dcoord >&  used_coords)
{
    int  idx = target.x - start.x;
    int  idy = target.y - start.y;

    path.push         ( start );
    used_coords.insert( start );

    //   LOG( "GoTo",0,string( "Next: " ) + toString(start.x) + string( "," )+ toString(start.y));    
    if ( !idx  &&  !idy )  return true;
    
    if ( idx > 0 )         idx =  1;
    else if ( idx< 0)      idx = -1;
    
    if ( idy > 0)          idy =  1;
    else if ( idy< 0)      idy = -1;
    
    dcoord  next( start.x, start.y + idy );
    
    if ( idy == 0  ||  !within( next,w,h, body_units )  ||  !FreeArea( F, next, body_units )  || 
         STLhas(used_coords,next)  ||  !find_path( next, target, F, w, h, body_units, path, used_coords ) )
    {
        next = dcoord( start.x + idx, start.y );
        
        if ( idx == 0  ||  !within( next, w, h, body_units )  ||  !FreeArea( F, next, body_units )  || 
             STLhas( used_coords,next )  ||  !find_path( next, target, F, w, h, body_units, path, used_coords ) )
        {
            next = dcoord( start.x - idx, start.y);
            if ( idx == 0  ||  !within( next, w, h, body_units )  ||  !FreeArea( F, next, body_units )  || 
                STLhas( used_coords, next )  ||  !find_path( next, target, F, w, h, body_units, path, used_coords ) )
            {
                next = dcoord( start.x - idy, start.y );
                if ( idy == 0  ||  !within( next, w, h, body_units )  ||  !FreeArea( F, next, body_units )  || 
                     STLhas( used_coords, next )  || !find_path( next, target, F, w, h, body_units, path, used_coords ) )
                {
                    path.pop();
                    return false;
                }
            }
        }
    }
    
    return true;
}

//----------------------------------------------------------------------------------------------------------------
stack< dcoord >  revert_stack( stack< dcoord >  rhs)
{
    stack< dcoord >  ret;
    while ( !rhs.empty() )  {
        ret.push( rhs.top() );
        rhs.pop ();
    }
    
    return ret;
}

//----------------------------------------------------------------------------------------------------------------
#define _abs(x) (((x)>0) ? (x) : -(x))

//----------------------------------------------------------------------------------------------------------------
struct AgentMoveThread
{
    ServerSocket*    socket;
    stack< dcoord >  path;
    float  dx, dy;
    int     w,  h;
    
    AgentMoveThread( ServerSocket*  _socket, const stack< dcoord >&  _path, int  _w, int  _h, float  _dx, float  _dy)
                   : socket( _socket ), path( _path ), dx( _dx ), dy( _dy ), w( _w ), h( _h ) { 
    }
    
    //-------------------------------------------------------------------------------------------------------------
    void operator() ()
    {
         LOG( "AgentMoveThread",1,"Up and running!" );
        // TODO:  "Resource leak! SocketReportProvider object."
        SocketReportProvider*  reporter = new SocketReportProvider( socket );       
        Obj3D*                 body     = SimServer::Get()->GetDemonByPort( socket->GetParent()->GetPort() )->getRepresentation();        

         LOG( "AgentMoveThread",3,"Body found." );      
        if ( path.empty() )  {
             LOG( "AgentMoveThread",1,"Empty path?" );
            return;
        }
            
        while ( !path.empty() )  {
            double x, y, z, phi, nphi = 0.0;        
            const double  step        = 0.05 * RoomScaleFactor;
            
            dcoord  next = path.top();
            path.pop();

             LOG( "AgentMoveThread",3,"Next step..." );     
            
            while (1)  {
                shared_ptr<Command> com(new CommandAction(reporter));

                /*com->addArguments( "move.forward 1" );
                SimServer::Get()->EnqueueCommand(com);
                return;*/
                
                body->getOrientation( x, y, z, phi);
                body->getPosition   ( x, y, z);             
                 LOG( "AgentMoveThread", 3, "phi: " + toString(phi));
                    
                double  diffx = ( (float)next.x + 0.5 - w/2.0f ) * dx - x;
                double  diffy = ( (float)next.y + 0.5 - h/2.0f ) * dy - z;
                
                // Manages the loop. Yes, this is the easiest way.
                if ( _abs( diffx ) < 0.04*RoomScaleFactor  &&  _abs( diffy ) < 0.04*RoomScaleFactor )
                    break;
                 LOG( "AgentMoveThread", 3, "x: " + toString(x) + " z: " + toString(z) + " nx: " + toString(next.x) + " ny: " + toString(next.y));          
                 LOG( "AgentMoveThread", 3, "diffx: " + toString(diffx) + " diffy: " + toString(diffy) );
            
                char          combuf[500];          
                const double  pi = 3.14159265;
        
                if ( diffx > 0.01 )        nphi =  pi/2 - atan( diffy / diffx );                
                else if ( diffx < -0.01 )  nphi = -pi/2 - atan( diffy / diffx );
                else if ( diffy > 0.01  ||  diffy < -0.01 )  nphi = ( (diffy > 0) ? 0 : pi );
            
                //  if ( diffy < 0.01 && diffx < 0.01)
                //  nphi = -nphi;
            
                if ( nphi > 2*pi)  nphi -= 2*pi;
                if ( nphi < 0   )  nphi += 2*pi;
                
                double  dphi = nphi - phi;      
                        
                 LOG( "AgentMoveThread", 2, "dphi: " + toString(dphi) + " nphi: " + toString(nphi));
            
                if ( dphi > 0.04  ||  dphi < -0.04 )   
                    sprintf( combuf, "turn.right  %f" , dphi );
                else                                   
                    sprintf( combuf, "move.forward %f", step );

                com->addArguments( combuf );
            
                 LOG( "AgentMoveThread",3,string( "Enquing action..." ) + combuf );     
                SimServer::Get()->EnqueueCommand( com ); //Parse later
                 LOG( "AgentMoveThread",3,"Enque ok..." );      
                usleep(500000);
                 LOG( "AgentMoveThread",3,"Sleep over..." );        
            }
        }   
         LOG( "AgentMoveThread",3,"Path traversed ok..." );
    }

}; //class

//----------------------------------------------------------------------------------------------------------------
void goto_test( ServerSocket*  _socket, float tsx, float tsy )
{
    iEngine*  engine = FROM_CS( iEngine );  
    iMeshList*        meshes = engine->GetMeshes();
    vector< pair<csVector3, csBox2> >  blocks;  
    ServerSocket*  socket = _socket;
    if (socket == NULL) {
        socket = SimServer::Get()->GetAgent(0)->GetSocket();
    }
    Obj3D*            body   = SimServer::Get()->GetDemonByPort( socket->GetParent()->GetPort() )->getRepresentation();
    double  _foo;
    float   asx, asy;
    double  dasx, dasy;
    
    RoomScaleFactor = FloatConfig( "RoomSizeX" ) / 15.0f;

    body->getPosition(dasx,_foo,dasy);
    asx = dasx;  asy = dasy;

    for (int m = 0; m < meshes->GetCount(); m++)  {
        iMeshWrapper*  mesh = meshes->Get(m);
        if ( !mesh  ||  !mesh->QueryObject()  ||  !mesh->GetMeshObject() )
            continue;
        string name = mesh->QueryObject()->GetName();
        // TODO:  "1-agent soln"
        if ( !nocase_equal( name.c_str(), AGENT_NAME )  &&  
             ( TheSimWorld.IsMovable(name)  ||  CSWorld::Get().IsInert(name) ) )
        {
            csBox3         box;
            iObjectModel*  iom = mesh->GetMeshObject()->GetObjectModel();
            
            if ( !iom  ||  !mesh->GetMovable() )
                continue;
            
            iom->GetObjectBoundingBox (box);
        
            // You can actually use GetMovable even for inert objects in this manner.
            csVector3 pos = mesh->GetMovable()->GetPosition();

            // Check that the object is not a room container itself!
            if ( ContainedIn( pos, FloatConfig( "RoomSizeX" ) /2, 
                                   FloatConfig( "RoomSizeY" ) /2,
                                   FloatConfig( "RoomSizeZ" ) /2) )
            {
                char  buf[300];
                 sprintf( buf, "%s: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", name.c_str(),
                          box.MinX(), box.MinY(), box.MinZ(), box.MaxX(), box.MaxY(), box.MaxZ());
                 LOG( "TEST", 0, buf );             
                blocks.push_back( pair< csVector3,csBox2 >( pos, csBox2(box.MinX(), box.MinZ(), box.MaxX(), box.MaxZ())) );
            }
        }
    }

//    float  dx = FloatConfig( "RoomSizeX" )/100;
    float  dx = 0.1f * RoomScaleFactor; //FloatConfig( "RoomSizeX" )/2;
    float  dy = 2 * FloatConfig( "AgentBodyRadius" );
    
    int    body_units = (int)(dy/dx)+1;

     LOG( "TEST", 3, "Dx: " + toString(dx) + "mm DY: " + toString(dy)+"mm" );

    float  sx = -FloatConfig( "RoomSizeX" )/2,  ex = FloatConfig( "RoomSizeX" )/2;
    float  sz = -FloatConfig( "RoomSizeZ" )/2,  ez = FloatConfig( "RoomSizeZ" )/2;
    int    w  = (int) ((ex-sx) / dx);
    int    h  = (int) ((ez-sz) / dy);
    
    bool  **F = new bool* [w];

    for (int k=0;k< w;k++)
        F[k] = new bool[h];
    
    for (int x=0;x< w;x++)
        for (int y=0;y< h;y++)
            F[x][y] = true;

    #define Xcoord2index( _x ) (int)( w/2+_x / dx )
    #define Ycoord2index( _y ) (int)( h/2+_y / dy )
        
    const float x_buffer = (dx-0.02*RoomScaleFactor), y_buffer = (dy-0.02*RoomScaleFactor);

    //printf( "Blocks %d\n", blocks.size());
        
#ifdef WIN32
    for (uint i=0;i< blocks.size();i++)
#else
    for (unsigned int i=0;i< blocks.size();i++)
#endif
    {
        float bsx = blocks[i].first.x+blocks[i].second.MinX();
        float bex = blocks[i].first.x+blocks[i].second.MaxX();
        float bsy = blocks[i].first.z+blocks[i].second.MinY();
        float bey = blocks[i].first.z+blocks[i].second.MaxY();
        for (float  ix = bsx - x_buffer;
                    ix < bex + x_buffer;
                    ix+= dx)
            for (float  iy = bsy - y_buffer;
                        iy < bey + y_buffer;
                        iy+= dy)
            {
                int iix = Xcoord2index(ix);
                int iiy = Ycoord2index(iy);
                
                if ( iix < 0 || iiy < 0) //May happen due to buffer deltas.
                    continue;
                
                if ( iix >= w  || iiy >=h)
                {
                     LOG( "TEST",0,"OVERFLOW Block #"+toString(i)+ " blocking "
                          +toString(ix / dx) + "," + toString(iy / dy));
                    continue;
                }
//               LOG( "TEST",0,toString(w)+":"+toString(h)+":"+toString(iix)+"/"+toString(iiy));
                F[iix][iiy] = false;
//               LOG( "TEST",0,"Block #"+toString(i)+ " blocking "
//                  +toString(ix / dx) + "," + toString(iy / dy));
            }
    }

    
    //  CSWorld::Get().InsertBox(csVector3(-4,1,-2),"b1","purple",csVector3(0.5,0.5,0.5),true,true);
    /*  
    /// TODO: There's a bug in the path visualization! Segfaults both here and at
    /// the later visualization loop
        // Visualization:
        for (int x=0;x< w;x++)
            for (int y=0;y< h;y++)
                if ( !F[x][y])
                {
                    float rx = ((float)x-w/2.0f)*dx;                                
                    float rz = ((float)y-h/2.0f)*dy;
                    
    //               LOG( "TEST",0,toString(x)+" = > "+toString(rx)+" mm" );
    //               LOG( "TEST",0,toString(y)+" = > "+toString(rz)+" mm" );
                    
                    CSWorld::Get().InsertBox(csVector3(rx,0.1,rz),
                                    "b"+toString(y*w+x),
                                    "purple",
                                    csVector3(dx,1.5,dy),
                                    true,true);
                }
                else
                {
                    float rx = ((float)x-w/2.0f)*dx;                                
                    float rz = ((float)y-h/2.0f)*dy;
                    
    //              float rx = Xcoord2index(x);                             
    //              float rz = Ycoord2index(y);
                    
    //               LOG( "TEST",0,toString(x)+" = > "+toString(rx)+" mm" );
    //               LOG( "TEST",0,toString(y)+" = > "+toString(rz)+" mm" );
                    
                    CSWorld::Get().InsertBox(csVector3(rx,0.1,rz),
                                    "b"+toString(y*w+x),
                                    "green",
                                    csVector3(dx,0.2,dy),
                                    true,true);
                }
    */
            
            
    dcoord  target( Xcoord2index( tsx ), Ycoord2index( tsy ) );
    dcoord  start ( Xcoord2index( asx ), Ycoord2index( asy ) );
    stack< dcoord >  path,  path_store;
    set< dcoord >    used_coords;

    printf( "Trying to find path from (%f,%f) to (%f,%f) ...\n", asx, asy, tsx, tsy);
    if ( find_path(start, target, F, w,h, body_units, path_store, used_coords))
    {
        path = path_store;
         LOG( "TEST",0,"Path found!" );
        
        /*      //Visualization
                while (!path.empty())
                {
                    dcoord d = path.top();
                    path.pop();
        //           LOG( "TEST",0,toString(d.x) + "," + toString(d.y));

                        float rx = ((float)d.x-w/2.0f)*dx;                              
                        float rz = ((float)d.y-h/2.0f)*dy;
                    
        //           LOG( "TEST",0,toString(rx) + "," + toString(rz) + " / "+toString(w)+","+toString(h));
                    
                        CSWorld::Get().InsertBox(csVector3(rx,0.1,rz),
                                        "P"+toString(d.y*w+d.x),
        //                              "black",
                                        "lightpurple",
                                        csVector3(dx,0.25,dy),
                                        true,true);
                }
                */
        path_store = revert_stack( path_store );
         LOG( "TEST",2,"Launching AgentMoveThread..." );        

        // TODO:  "Memory leak here"
        AgentMoveThread* amt = new AgentMoveThread( socket, path_store, w, h, dx, dy );
        /*boost::thread*   t   = */new boost::thread  ( *amt );
    }   
    else {
         LOG( "TEST",0,"Path not found!" );
    }
}

//----------------------------------------------------------------------------------------------------------------
void  gaze_test( ServerSocket*  socket, float  sphi) //, float  ephi )
{
    Obj3D*            body   = SimServer::Get()->GetDemonByPort( socket->GetParent()->GetPort() )->getRepresentation();
    double x, y, z = 0.0;
    //double phi, nphi = 0.0;
    //body->getOrientation( x, y, z, phi);
    body->getPosition   ( x, y, z);

    set<string> objs;

//  float next_phi = sphi;
    float dphi = 3.14/2; //3.14*2
    for (float next_phi = sphi; next_phi < dphi+sphi; next_phi+=0.01)
    {
        map<int, map<int, meshData> > o = iEngine::Instance().VisibleObjects(csVector3(x,y,z), next_phi);

//      if (o.empty()) { printf(".\n"); /*continue;*/ }

        for (map<int, map<int, meshData> >::iterator xi = o.begin(); xi != o.end(); xi++)
            for (map<int, meshData>::iterator y = xi->second.begin(); y != xi->second.end(); y++)
                objs.insert(y->second.wrapper->QueryObject()->GetName());
//              printf("(%d,%d): %s\n", xi->first, y->first, y->second->QueryObject()->GetName());
    }

    for (set<string>::iterator n = objs.begin(); n != objs.end(); n++)
        puts(n->c_str());
}


//----------------------------------------------------------------------------------------------------------------
//--- The valid form is "action (action name, action paramater) )" -------------------------------------------
ParseResult CommandTest::parse(std::string arguments) const
{
     LOG( "CommandTest", 2, arguments );
    StringTokenizer  tok( arguments, "\t\n\r ,();" );
    vector<string>   a = tok.WithoutEmpty();

    if ( a[0] == "0" )  {
        if ( a.size() < 3 )  { 
             LOG( "CommandTest", 2, "insufficient nr of arguments" ); 
            return PARSE_FAILED; 
        }

        goto_test( socket, atof( a[1].c_str() ), atof( a[2].c_str() ) );
    }
    else if ( a[0] == "1" )  {
        if ( a.size() < 3 )  { 
             LOG( "Direct CommandTest", 2, "insufficient nr of arguments" ); 
            return PARSE_FAILED; 
        }
        goto_test( NULL, atof( a[1].c_str() ), atof( a[2].c_str() ) );
    } 
    else if ( a[0] == "2" )  {
        if ( a.size() < 2 )  { 
             LOG( "Direct CommandTest", 2, "insufficient nr of arguments" ); 
            return PARSE_FAILED; 
        }
        gaze_test( socket, atof( a[1].c_str() ) );
    } 

    
    return PARSE_OK;
}
