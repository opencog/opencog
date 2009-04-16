/*
    Copyright (C) 2001 by Teemu Keinonen

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.

    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#ifndef CSMESHTOOL_H
#define CSMESHTOOL_H

#include <crystalspace.h>
#include <string>
#include <map>
#include <stack>
#include <ios>
#include <iostream>
#include <ostream>
#include <istream>
#include <iutil/document.h>

/** @class CSMeshTool
 The class implements loading of
*/

class CSMeshTool : public Singleton<CSMeshTool>
{
protected:
    void CloneNode (iDocumentNode* from, iDocumentNode* to);

    csRef<iDocumentSystem> docsys;
    std::map<std::string, csRef<iDocument> > m_Docs;

    void GetFromNodes(const char * kind, iDocumentNode * node, std::stack<csRef<iDocumentNode> >& nodes);
    void RemoveNodes(iDocumentNode*node, const char* value, const char*name = NULL);
    csRef<iDocument> CloneFrom(iDocumentNode * node);

    void ReplaceNodeValues(iDocumentNode*node, const char * key, const char * value);
    bool RunDoc(iDocument*doc);

    csRef<iDocument> GetDocument(std::string doc, bool reload = false);

public:

    friend class Singleton<CSMeshTool>;
    csRef<iDocumentNode> FindNode(iDocumentNode * node,
                                  const char * nodevalue,
                                  const char * attributename,
                                  const char * attributevalue);
    /// Reads in a specific mesh from the world file
    bool LoadModelFrom (const char * world_name, string mesh_name);
    csRef<iMeshWrapper> ReadModels (const char * world_name, csVector3 pos, string mesh_type, string mesh_name);
    csRef<iMeshWrapper> InsertInstance(const char * mesh_name);

    // returns used displacement
    csVector3 ReCenter(iMeshWrapper*mesh_w, bool y_floor = true);

    // scale by Y maintaining aspect
    void ScaleByY(iMeshWrapper * mesh_w, float y);

    void ReportGeom(iMeshWrapper*mesh_w);

    CSMeshTool ();

    ~CSMeshTool ();
};

#endif // __CSMeshTool2_H__
