/*
 * opencog/embodiment/AGISimSim/server/src/CSMeshTool.cpp
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

Copyright Teemu Keinonen / GenMind Ltd.
Not part of AGI-Sim.
Do not distribute.

*/


#include "simcommon.h"
#include "space.h"

#if CRYSTAL

#include "cssysdef.h"
#include "CSMeshTool.h"
#include "CSproxy.h"
#include "CSops.h"
#include "CSworld.h"

//-------------------------------------------------------------------------------------------------------------
CSMeshTool::CSMeshTool ()
{
}

CSMeshTool::~CSMeshTool ()
{
}

//-------------------------------------------------------------------------------------------------------------
void CSMeshTool::GetFromNodes (const char * kind, iDocumentNode * node, std::stack< csRef<iDocumentNode> >& nodes)
{
	const  char* value = node->GetValue();
	if ( std::string(value) == std::string(kind) ) {
		// std::cout << "Found:" << value;
		nodes.push(node);
		return;
	}
	csRef<iDocumentNodeIterator>  node_iter = node->GetNodes();

	while (node_iter->HasNext())	{
		csRef<iDocumentNode>  subnode = node_iter->Next();
		GetFromNodes (kind, subnode, nodes);
	}
}

//-------------------------------------------------------------------------------------------------------------
csRef<iDocument>  CSMeshTool::CloneFrom (iDocumentNode* node)
{
	csRef<iDocument>      newdoc  = docsys->CreateDocument ();
	csRef<iDocumentNode>  newroot = newdoc->CreateRoot     ();
	csRef<iDocumentNode>  bufnode;

	//  const char * nodetypenames[] = {"none", "CS_NODE_DOCUMENT", "CS_NODE_ELEMENT", "CS_NODE_COMMENT",
	//	"CS_NODE_UNKNOWN","CS_NODE_TEXT","CS_NODE_DECLARATION"};
	bufnode = newroot->CreateNodeBefore (CS_NODE_UNKNOWN, 0);
	
	// For some reason CS_NODE_UNKNOWN accepts no attributes.
	std::string  baseval (node->GetValue());
	csRef<iDocumentAttributeIterator>  ait = node->GetAttributes();

	while ( ait->HasNext() ) {
		csRef<iDocumentAttribute>  dat = ait->Next();
		baseval += std::string(" ") + dat->GetName() + "=\"" + dat->GetValue() + "\"";
	} 
	bufnode->SetValue(baseval.c_str());

	if (!bufnode) {
		 LOG("CSMeshTool",1,"NULL bufnode! (try different type of node");
	}

	//csRef<iDocumentAttributeIterator>
	ait = node->GetAttributes();
	while ( ait->HasNext() ) {
		csRef<iDocumentAttribute>  attr = ait->Next();
		bufnode->SetAttribute (attr->GetName(), attr->GetValue());
	}
	//std::cout << "Clone node " << node->GetValue() << " to new root " << newroot<< std::endl;
	CloneNode (node, newroot);
	csRef<iDocumentNodeIterator>  nit = newroot->GetNodes();
	bufnode = newroot->CreateNodeBefore (CS_NODE_UNKNOWN, 0);
	bufnode->SetValue( (std::string("/") + node->GetValue()).c_str() );

	return newdoc;
}

//-------------------------------------------------------------------------------------------------------------
void CSMeshTool::RemoveNodes (iDocumentNode* parent, const char* value, const char* name)
{
	bool need_reset = true;

	while ( need_reset )  {
		need_reset = false;
		csRef<iDocumentNodeIterator>  nit = parent->GetNodes();

		while ( nit->HasNext() ) {
			csRef<iDocumentNode>  node = nit->Next();
			if ( std::string (node->GetValue()) == std::string (value) )  {
				if (name != NULL)  {
					csRef<iDocumentAttribute> node_name = node->GetAttribute(name);
					if (node_name) {
						if ( std::string (node_name->GetValue()) == std::string(name) )  {
							parent->RemoveNode (node);
							need_reset = true;
							break;
						}
					}
				}
				else  {
					parent->RemoveNode (node);
					// std::cout << "Found and removed: " << value << std::endl;
					need_reset = true;
					break;
				}
			}
			RemoveNodes(node, value, name);
		} //while
    } //while
}

//-------------------------------------------------------------------------------------------------------------
csRef<iMeshWrapper> CSMeshTool::ReadModels (const char* mapFilename, csVector3 pos, string mesh_type, string mesh_name)
{

    csRef<iEngine> engine = FROM_CS(iEngine);
    iMeshFactoryWrapper* mesh_factory = engine->FindMeshFactory(mesh_type.c_str());
    if (!mesh_factory) {
        // Tries to load meshfact from file
        csRef<iLoader>  loader = FROM_CS(iLoader);  
        if (strstr(mapFilename,"/cally.cal3d")) {
            csRef<iMeshFactoryWrapper> imeshfact ( loader->LoadMeshObjectFactory (mapFilename));
            mesh_factory = imeshfact;
        } else {
        loader->LoadLibraryFile(mapFilename, 0, true, true);
//        loader->LoadLibraryFile(mapFilename);
        mesh_factory = engine->FindMeshFactory(mesh_type.c_str());
        }
    }
    printf(">>>>>>>>>>>>>> mesh_factory = %p <<<<<<<<<<<<<<<<<\n", mesh_factory); 

    if (mesh_factory) {
        iSector* room = CSWorld::Get().GetRoom();
        csRef<iMeshWrapper>  newmeshwrap = engine->CreateMeshWrapper(mesh_factory, mesh_name.c_str(), room, pos);

        // TODO: Check other needed parameters (SEE ball and box examples in CSworld.cc)
            {
             csBox3                 bbox  = newmeshwrap->GetWorldBoundingBox();
             csVector3              delta = bbox.GetSize();
             char  mbuf[400];
             sprintf(mbuf, "Bounding box size x: %f y: %f z: %f", delta.x, delta.y, delta.z);
             LOG("CSMeshTool",1,mbuf);
            }

            // TODO: Remove this hacky after S&M demo
            if (IntConfig("RescaleObjects")) {
                // Rescale the loaded objects to the small world
                float size = 0.01;
            
                csMatrix3  hardtrans; hardtrans.Identity();
                hardtrans *= 1.0/size;
            
                csReversibleTransform  transu (hardtrans, csVector3(0, 0, 0));
                csRef<iMeshObject>     sprite_obj = newmeshwrap->GetMeshObject();   
                sprite_obj->HardTransform(transu);
                
                csBox3                 bbox  = newmeshwrap->GetWorldBoundingBox();
                csVector3              delta = bbox.GetSize();
                char  mbuf[400];
                sprintf(mbuf, "after scaling %f bbox size x: %f y: %f z: %f", size, delta.x, delta.y, delta.z);
                LOG("CSMeshTool",1,mbuf);
            }
            
            //gets(mbuf);                       
            newmeshwrap->GetMovable()->SetPosition(pos);
            newmeshwrap->GetMovable()->UpdateMove();

       return newmeshwrap;
    }

	return NULL;
}

csRef<iDocument> CSMeshTool::GetDocument(std::string doc, bool reload)
{
  std::map<std::string, csRef<iDocument> >::iterator it;
  it = m_Docs.find(doc);
  if (it != m_Docs.end() && !reload)
    {
      return m_Docs[doc];
    }
  csRef<iVFS>         vfs = FROM_CS(iVFS);
  csRef<iDataBuffer>  buf = vfs->ReadFile(doc.c_str());
  
  if ( !buf || !buf->GetSize () )	{
    LOG("CSMeshTool",1,"File '" + doc + "' does not exist!");
    return false;
  }
	
  if (!docsys)
    {
      csRef<iPluginManager>  plugin_mgr = FROM_CS(iPluginManager);
      //  csRef<iDocumentSystem> 
      docsys = CS_LOAD_PLUGIN (plugin_mgr, "crystalspace.documentsystem.tinyxml", iDocumentSystem);
    }

  csRef<iDocument> xml = docsys->CreateDocument();
  const  char* error = xml->Parse(buf, true);
  if (error != 0) 
    {
      LOG("CSMeshTool",1,"Error while parsing! " + string(error));
      return csRef<iDocument>(0);
    }
  m_Docs.insert(std::pair<std::string, csRef<iDocument> >(doc, xml));
  return m_Docs[doc];
}

csRef<iDocumentNode> CSMeshTool::FindNode(iDocumentNode * node, 
			      const char * nodevalue,
			      const char * attributename,
			      const char * attributevalue)
{

  //  cout << "Node " << node << " with value " << node->GetValue() << std::endl;
  if (std::string(node->GetValue()) == std::string(nodevalue))	
    {
      //    std::cout << "Node value: " << node->GetValue() << std::endl;
      csRef<iDocumentAttributeIterator> ait = node->GetAttributes ();
      while (ait->HasNext())
	{
	  csRef<iDocumentAttribute> attr = ait->Next();
	  if (std::string(attr->GetName()) == std::string(attributename))
	    {
	      //      std::cout << "Attr name: " << attr->GetName() << std::endl;
	      if (std::string(attr->GetValue()) == std::string(attributevalue))
		{
		  //  std::cout << "Attr value: " << attr->GetValue() << std::endl;
		  return node;
		}
	    }
	}
    }
  
  csRef<iDocumentNodeIterator>  nit = node->GetNodes();
  while ( nit->HasNext() ) 
    {
      csRef<iDocumentNode>  subnode = nit->Next();
      csRef<iDocumentNode> ret;
      ret = FindNode(subnode, nodevalue, attributename, attributevalue);
      if (ret)
	return ret;
    }
  return csRef<iDocumentNode> (0);
}

//-------------------------------------------------------------------------------------------------------------
bool CSMeshTool::LoadModelFrom (const char* str,  string mesh_name)
{
  bool virgin = false;
  if (m_Docs.find(str) == m_Docs.end())
    virgin = true;

  csRef<iDocument> doc = GetDocument(std::string(str));

  if (doc->GetRoot()->GetNode("world"))
    {
      // run settings and stuff
      if (virgin)
	{
	  csRef<iDocument> defdoc = docsys->CreateDocument();
	  csRef< iDocumentNode > defroot = defdoc->CreateRoot ();
	  CloneNode(doc->GetRoot(), defroot);
	  RemoveNodes(defroot, "sector");
	  RemoveNodes(defroot, "meshobj");
	  RemoveNodes(defroot, "meshfact");
	  RunDoc(defdoc);
	}

      csRef<iDocumentNode> modelnode = FindNode(doc->GetRoot(), "meshobj", 
						"name", mesh_name.c_str());
      if (!modelnode)
	modelnode = FindNode(doc->GetRoot(), "meshfact", 
			     "name", mesh_name.c_str());
      if (!modelnode)
	return false;

      csRef<iDocument> model_doc = CloneFrom(modelnode);
  
      return RunDoc(model_doc);
    }
  else if (doc->GetRoot()->GetNode("library"))
    {
      csRef<iEngine> engine = FROM_CS(iEngine);
      csRef<iLoader> loader = FROM_CS(iLoader);
      if (virgin)
	{
	  iBase * res; 
	  loader->Load(doc->GetRoot(), res);
	}
      iMeshFactoryWrapper* fact_w = engine->FindMeshFactory(mesh_name.c_str());
      std::cout<<"Found mesh factory wrapper: " << mesh_name << std::endl;
      if (!fact_w)
	{
	  LOG("CSMeshTool", 1, "Couldn't find mesh "+mesh_name+" in library "+str);
	  return false;
	}
      engine->CreateMeshWrapper(fact_w, mesh_name.c_str());
      std::cout<<"Created mesh wrapper from factory " << fact_w->GetMeshObjectFactory() << " with same name\n";
      return true;
    }
  return false;
}


//-------------------------------------------------------------------------------------------------------------
void CSMeshTool::CloneNode (iDocumentNode* from, iDocumentNode* to)
{
	to->SetValue (from->GetValue ());
	csRef<iDocumentNodeIterator>  it = from->GetNodes ();
	
	while ( it->HasNext () )	{
		csRef<iDocumentNode>  child       = it->Next ();
		csRef<iDocumentNode>  child_clone = to->CreateNodeBefore (child->GetType (), 0);
		CloneNode (child, child_clone);
	}
	csRef<iDocumentAttributeIterator> atit = from->GetAttributes ();
	
	while ( atit->HasNext () )  {
		csRef<iDocumentAttribute>  attr = atit->Next ();
		to->SetAttribute (attr->GetName (), attr->GetValue ());
	}
}

//-------------------------------------------------------------------------------------------------------------
void CSMeshTool::ReplaceNodeValues(iDocumentNode*node, const char * key, const char * value)
{
    if ( std::string(node->GetValue()) == std::string(key) )  {
		node->SetValue(value);
    }
    
    csRef<iDocumentNodeIterator>  nit = node->GetNodes();    
    while ( nit->HasNext() )  {
		csRef<iDocumentNode>  child = nit->Next();
		ReplaceNodeValues(child, key, value);
    }
}

//-------------------------------------------------------------------------------------------------------------
bool  CSMeshTool::RunDoc(iDocument*doc)
{
	iBase*          result;
	bool load_ok = false;
	csRef<iLoader> loader = FROM_CS(iLoader);
#define CSMESHTOOL_WITH_TEMPFILE
#ifdef CSMESHTOOL_WITH_TEMPFILE
	csRef<iVFS>     vfs = FROM_CS(iVFS);
	doc->Write(vfs, "/sim/temp_xml");
	LOG("CSMeshTool",3,"Copy written to /sim/temp_xml\n");
	load_ok = loader->Load("/sim/temp_xml", result);	
	LOG("CSMeshTool",3,"Loaded /sim/temp_xml.\n");
	
#else
	load_ok = loader->Load(doc->GetRoot(), result);
#endif
	if (!load_ok)
	  {
	    LOG("CSMeshTool", 1, "Loading wasn't sukkess!");
	    return false;
	  }
	csRef<iEngine> engine = FROM_CS(iEngine);
	csRef<iMeshWrapper> mesh_w;
	csRef<iMeshFactoryWrapper> fact_w= SCF_QUERY_INTERFACE(result, iMeshFactoryWrapper);
	if (!fact_w)
	  {
	    mesh_w = SCF_QUERY_INTERFACE(result, iMeshWrapper);
	  }
	else
	  {
	    mesh_w = engine->CreateMeshWrapper(fact_w, NULL);
	  }
	if (mesh_w)
	  {
	    mesh_w->IncRef();
	    engine->AddMeshAndChildren(mesh_w);
	    LOG("CSMeshTool", 1, "Added mesh and children...");
	  }
	else
	  {
	    LOG("CSMeshTool", 1, "Was world or library... or nothing!");
	  }
	return true;
}

csVector3 CSMeshTool::ReCenter(iMeshWrapper*mesh_w, bool y_floor)
{
  mesh_w->GetMovable()->UpdateMove();

  csMatrix3 ident;
  csBox3 bbox =  mesh_w->GetWorldBoundingBox();
  csVector3 displacement = bbox.GetCenter();
  if (y_floor)
    displacement[1] -= bbox.GetSize()[1]/2.0;
  csReversibleTransform trans(ident, -displacement);
  if (!mesh_w->GetMeshObject()->SupportsHardTransform())
    {
      LOG("CSMeshTool",0,"ReCenter obj doesn't support hardtrans. (maybe factory does?");
    }

  mesh_w->HardTransform(trans);
  mesh_w->GetMovable()->UpdateMove();

  return displacement;
}

void CSMeshTool::ScaleByY(iMeshWrapper * mesh_w, float y)
{
  mesh_w->GetMovable()->UpdateMove();

  csBox3 bbox = mesh_w->GetWorldBoundingBox();
  float divisor = y/bbox.GetSize()[1];
  csMatrix3 mat; mat.Identity();
  mat /= divisor;
  csReversibleTransform trans(mat, csVector3(6, 6, 6));
  
  if (!mesh_w->GetMeshObject()->SupportsHardTransform())
    {
      LOG("CSMeshTool",0,"ReCenter obj doesn't support hardtrans. (maybe factory does?");
    }

  mesh_w->HardTransform(trans);
  mesh_w->GetMovable()->UpdateMove();
}


csRef<iMeshWrapper> CSMeshTool::InsertInstance(const char *mesh_name)
{
  csRef<iEngine> engine = FROM_CS(iEngine);
  csRef<iMeshWrapper> mesh_w= engine->FindMeshObject(mesh_name);
  int i = 0;
  std::string newname;
  while (1)
    {
      char itos[10];
      sprintf(itos, "%i", i);
      newname = std::string(mesh_name)+itos;
      if (!engine->FindMeshObject(newname.c_str()))
	break;
      i++;
    }

  if (!mesh_w)
    {
      LOG("CSworld", 0, "COULDn'T find LOADED obJECT!");
    }
  else
    {
      std::cout << "Reposition old meshwrap to sector room\n";
      csRef<iMeshWrapper> new_mesh_w;
      new_mesh_w = engine->CreateMeshWrapper(mesh_w->GetMeshObject(), newname.c_str(), CSWorld::Get().GetRoom());
      engine->RemoveObject(mesh_w);
      return new_mesh_w;
    }

  csRef<iMeshFactoryWrapper> fact_w = engine->FindMeshFactory(mesh_name);
  if (fact_w)
    {
      std::cout<<"Create new mesh from factory\n";
      mesh_w = engine->CreateMeshWrapper(fact_w, newname.c_str(), CSWorld::Get().GetRoom());
      return mesh_w;
    }
  else
    {
      std::cout<<"No fucktory found!\n";
    }
  std::cout<<"Wrapper: " << mesh_w << " name " << mesh_name << std::endl;
  //  csRef<iMeshFactoryWrapper> fact_w = mesh_w->GetFactory();
  //std::cout << "Factory: " << fact_w << std::endl;
  //std::cout << "Found factory: " << found_fact_w << std::endl;
  return NULL;
}


void CSMeshTool::ReportGeom(iMeshWrapper*mesh_w)
{
  csBox3 bbox = mesh_w->GetWorldBoundingBox();
  csVector3 size = bbox.GetSize();
  std::cout<<"Box MIN x: "<<bbox.MinX()<<" y: "<<bbox.MinY()<<" z: "<<bbox.MinZ()<<std::endl;
  std::cout<<"Box MAX x: "<<bbox.MaxX()<<" y: "<<bbox.MaxY()<<" z: "<<bbox.MaxZ()<<std::endl;
  std::cout<<"Box size x: "<<size[0]<<" Box size y: "<<size[1]<<" Box size z: "<<size[2]<<std::endl;
  csVector3 pos = mesh_w->GetMovable()->GetFullPosition();
  std::cout << "Mesh full pos x: "<<pos[0]<<" y: "<<pos[1]<<" z: "<<pos[2]<<std::endl;
}


#endif
