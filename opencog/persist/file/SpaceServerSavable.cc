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
#include <opencog/util/macros.h>

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/spacetime/SpaceServer.h>

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
    std::map<Handle, SpaceServer::SpaceMap*>::iterator itr;
    for (itr = server->spaceMaps.begin(); itr != server->spaceMaps.end(); ++itr) {
        Handle mapHandle = (Handle)(itr->first);
        fwrite(&mapHandle, sizeof(Handle), 1, fp);
        SpaceServer::SpaceMap* map = itr->second;
        int xMin,yMin, zMin,floorHeight;
        unsigned int xDim, yDim, zDim;

        std::string mapName;
        mapName = map->getMapName();

        // the map name should be shorter than 128
        assert(strlen(mapName.c_str()) < 128);

        char charMapName[128];

        memset(charMapName, '\0',strlen(mapName.c_str()));
        strcpy(charMapName,mapName.c_str());

        xMin = map->xMin();
        yMin = map->yMin();
        zMin = map->zMin();

        xDim = map->xDim();
        yDim = map->yDim();
        zDim = map->zDim();

        floorHeight = map->getFloorHeight();

        fwrite(&charMapName, 128,1,fp);
        fwrite(&xMin, sizeof(int), 1, fp);
        fwrite(&yMin, sizeof(int), 1, fp);
        fwrite(&zMin, sizeof(int), 1, fp);

        fwrite(&xDim, sizeof(unsigned int), 1, fp);
        fwrite(&yDim, sizeof(unsigned int), 1, fp);
        fwrite(&zDim, sizeof(unsigned int), 1, fp);

        fwrite(&floorHeight, sizeof(int), 1, fp);

        map->save(fp);
    }
    /*
    unsigned int persistentHandlesSize = server->persistentMapHandles.size();
    fwrite(&persistentHandlesSize, sizeof(unsigned int), 1, fp);
    for (std::set<Handle>::const_iterator itr = server->persistentMapHandles.begin(); itr != server->persistentMapHandles.end(); itr++) {
        Handle mapHandle = *itr;
        fwrite(&mapHandle, sizeof(Handle), 1, fp);
    }
    */
}

void SpaceServerSavable::loadRepository(FILE* fp, HandMapPtr conv)
{
    logger().debug("Loading %s (%ld)\n", getId(), ftell(fp));

    unsigned int mapSize;
    bool b_read = true;
    int rc = fread(&mapSize, sizeof(unsigned int), 1, fp);
    OC_ASSERT(0 < rc, "SpaceServerSavable - Failed read of mapSize.");
    for (unsigned int i = 0; i < mapSize; i++) {
        Handle mapHandle;
        FREAD_CK(&mapHandle, sizeof(Handle), 1, fp);
        int xMin,yMin, zMin,floorHeight;
        unsigned int xDim, yDim, zDim;

        char charMapName[128];

        FREAD_CK(&charMapName, 128, 1, fp);
        std::string mapName(charMapName);

        FREAD_CK(&xMin, sizeof(int), 1, fp);
        FREAD_CK(&yMin, sizeof(int), 1, fp);
        FREAD_CK(&zMin, sizeof(int), 1, fp);

        FREAD_CK(&xDim, sizeof(unsigned int), 1, fp);
        FREAD_CK(&yDim, sizeof(unsigned int), 1, fp);
        FREAD_CK(&zDim, sizeof(unsigned int), 1, fp);

        FREAD_CK(&floorHeight, sizeof( int), 1, fp);

        SpaceServer::SpaceMap *map = new SpaceServer::SpaceMap(mapName,xMin,yMin,zMin,xDim,yDim,zDim,floorHeight);
        map->load(fp);

        OC_ASSERT(conv->contains(mapHandle),
                "SpaceServerSavable - HandleMap conv does not contain mapHandle.");
        Handle newMapHandle = conv->get(mapHandle)->getHandle();
        server->spaceMaps[newMapHandle] = map;
    }
    CHECK_FREAD;
    /*
    unsigned int persistentHandlesSize;
    FREAD_CK(&persistentHandlesSize, sizeof(unsigned int), 1, fp);
    for (unsigned int i = 0; i < persistentHandlesSize; i++) {
        Handle mapHandle;
        FREAD_CK(&mapHandle, sizeof(Handle), 1, fp);
        OC_ASSERT(conv->contains(mapHandle),
                "SpaceServerSavable - HandleMap conv does not contain mapHandle.");
        Handle newMapHandle = conv->get(mapHandle)->getHandle();
        server->persistentMapHandles.insert(newMapHandle);
    }*/
}

void SpaceServerSavable::clear()
{
    server->clear();
}

