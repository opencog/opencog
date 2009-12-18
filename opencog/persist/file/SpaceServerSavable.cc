/*
 * opencog/persist/file/SpaceServerSavable.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi
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
#include "SpaceServerSavable.h"

#include <opencog/util/Logger.h>
#include <opencog/util/StringTokenizer.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/SpaceServer.h>
#include <opencog/atomspace/TLB.h>

#include <string>
#include <vector>
#include <cstdlib>

using namespace opencog;

SpaceServerSavable::SpaceServerSavable()
{
    server = NULL;
}

SpaceServerSavable::~SpaceServerSavable()
{
    server = NULL;
}

const char* SpaceServerSavable::getId() const
{
    static const char* id = "SpaceServerSavable";
    return id;
}

void SpaceServerSavable::saveRepository(FILE * fp) const
{
    logger().debug("Saving %s (%ld)\n", getId(), ftell(fp));
    unsigned int mapSize = server->spaceMaps.size();
    fwrite(&mapSize, sizeof(unsigned int), 1, fp);
    for (std::vector<Handle>::const_iterator itr = server->sortedMapHandles.begin(); itr != server->sortedMapHandles.end(); itr++) {
        Handle mapHandle = *itr;
        fwrite(&mapHandle, sizeof(Handle), 1, fp);
        SpaceServer::SpaceMap* map = server->spaceMaps.find(mapHandle)->second;
        float xMin, xMax, yMin, yMax, radius;
        unsigned int xDim, yDim;
        xMin = map->xMin();
        xMax = map->xMax();
        yMin = map->yMin();
        yMax = map->yMax();
        radius = map->radius();
        xDim = map->xDim();
        yDim = map->yDim();
        fwrite(&xMin, sizeof(float), 1, fp);
        fwrite(&xMax, sizeof(float), 1, fp);
        fwrite(&yMin, sizeof(float), 1, fp);
        fwrite(&yMax, sizeof(float), 1, fp);
        fwrite(&radius, sizeof(float), 1, fp);
        fwrite(&xDim, sizeof(unsigned int), 1, fp);
        fwrite(&yDim, sizeof(unsigned int), 1, fp);
        map->save(fp);
    }
    unsigned int persistentHandlesSize = server->persistentMapHandles.size();
    fwrite(&persistentHandlesSize, sizeof(unsigned int), 1, fp);
    for (std::set<Handle>::const_iterator itr = server->persistentMapHandles.begin(); itr != server->persistentMapHandles.end(); itr++) {
        Handle mapHandle = *itr;
        fwrite(&mapHandle, sizeof(Handle), 1, fp);
    }
}

void SpaceServerSavable::loadRepository(FILE *fp, opencog::HandleMap<opencog::Atom*> *conv)
{
    logger().debug("Loading %s (%ld)\n", getId(), ftell(fp));

    unsigned int mapSize;
    fread(&mapSize, sizeof(unsigned int), 1, fp);
    for (unsigned int i = 0; i < mapSize; i++) {
        Handle mapHandle;
        fread(&mapHandle, sizeof(Handle), 1, fp);
        float xMin, xMax, yMin, yMax, radius;
        unsigned int xDim, yDim;
        fread(&xMin, sizeof(float), 1, fp);
        fread(&xMax, sizeof(float), 1, fp);
        fread(&yMin, sizeof(float), 1, fp);
        fread(&yMax, sizeof(float), 1, fp);
        fread(&radius, sizeof(float), 1, fp);
        fread(&xDim, sizeof(unsigned int), 1, fp);
        fread(&yDim, sizeof(unsigned int), 1, fp);
        SpaceServer::SpaceMap *map = new SpaceServer::SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, radius);
        map->load(fp);

        OC_ASSERT(conv->contains(mapHandle),
                "SpaceServerSavable - HandleMap conv does not contain mapHandle.");
        Handle newMapHandle = TLB::getHandle(conv->get(mapHandle));
        server->spaceMaps[newMapHandle] = map;
        server->sortedMapHandles.push_back(newMapHandle);
    }
    unsigned int persistentHandlesSize;
    fread(&persistentHandlesSize, sizeof(unsigned int), 1, fp);
    for (unsigned int i = 0; i < persistentHandlesSize; i++) {
        Handle mapHandle;
        fread(&mapHandle, sizeof(Handle), 1, fp);
        OC_ASSERT(conv->contains(mapHandle),
                "SpaceServerSavable - HandleMap conv does not contain mapHandle.");
        Handle newMapHandle = TLB::getHandle(conv->get(mapHandle));
        server->persistentMapHandles.insert(newMapHandle);
    }
}

void SpaceServerSavable::clear()
{
    server->clear();
}

