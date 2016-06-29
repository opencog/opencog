/*
 * PGStorageDelegate.cc
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#include "PGStorageDelegate.h"

PGStorageDelegate::PGStorageDelegate(const char *dbName, const char *userName, const char *password)
{
    dbInterface = new PGAtomStorage(dbName, userName, password);
    if (! dbInterface->connected()) {
        logger().info("PGStoreDelegate() could not connect to the database");
    } else {
        logger().info("Connected to database \"%s\" as \"%s\"", dbName, dbUser);
    }
}

PGStorageDelegate::~PGStorageDelegate()
{
    delete dbInterface;
}

void PGStorageDelegate::loadSCMFile(const char *fileName, int dbCommitThreshold)
{

    // TODO : consistency check of parameters

    std::vector<AtomPtr> allAtoms;
    parseSCMFile(fileName, allAtoms);
    bool dirty = false;
    for (int i = 0; i < allAtoms.size(); i++) {
        dbInterface->storeAtom(allAtoms[i], false);
        dirty = true;
        if (((i % dbCommitThreashold) == 0) && (i > 0)) {
            dbInterface->flushStoreQueue();
            dirty = false;
        }
    }
    if (dirty) {
        dbInterface->flushStoreQueue();
    }
}
