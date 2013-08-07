/*
 * opencog/visualizer/src/Vertices.cpp
 *
 * Copyright (C) 2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Erwin Joosten <eni247@gmail.com>
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

#include "../include/Vertices.h"

Vertices::Vertices()
{
    //ctor
}

Vertices::~Vertices()
{
    for (int i = 0; i < vertices.size(); i++)
        delete vertices[i];
}

Vertex* Vertices::FindVertexByUUID(UUID uuid)
{
    for (int i = 0; i < vertices.size(); i++)
        if(vertices[i]->uuid==uuid)
            return vertices[i];
    return NULL;
}

void Vertices::Remove(Vertex* vertex)
{
    for (int i = 0; i < vertices.size(); i++)
        if(vertices[i]==vertex)
        {
            vertices.erase(vertices.begin()+i);
            delete vertex;
            return;
        }
}
