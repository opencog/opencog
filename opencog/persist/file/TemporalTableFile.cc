/*
 * opencog/persist/file/TemporalTableFile.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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


#include "TemporalTableFile.h"

#include <opencog/atomspace/Atom.h>
#include <opencog/spacetime/TemporalTable.h>

#include <set>

#include <opencog/util/Logger.h>
#include <opencog/util/macros.h>

using namespace opencog;

TemporalTableFile::TemporalTableFile()
{
}

TemporalTableFile::~TemporalTableFile()
{
}

void TemporalTableFile::load(FILE* fp, TemporalTable* tbl, HandMapPtr conv)
{
    int size;
    bool b_read = true;
    // reads the table size (number of temporal entries)
    FREAD_CK(&size, sizeof(int), 1, fp);
    for (int i = 0; i < size; i++) {
        // reads the current Temporal entry
        bool isNormal;
        unsigned long a, b;
        FREAD_CK(&isNormal, sizeof(bool), 1 , fp);
        FREAD_CK(&a, sizeof(unsigned long), 1 , fp);
        FREAD_CK(&b, sizeof(unsigned long), 1 , fp);
        Temporal t(a, b, isNormal);
        // reads the number of associated handles to the current Temporal entry
        int setSize;
        FREAD_CK(&setSize, sizeof(int), 1 , fp);
        for (int j = 0; j < setSize; j++) {
            // reads each associated handle
            UUID uuid;
            FREAD_CK(&uuid, sizeof(UUID), 1, fp);
            Handle oldHandle(uuid);
            if ((!conv->contains(oldHandle))) {
                throw InconsistenceException(TRACE_INFO,
                     "Temporal TableFile - Couldn't load TemporalRepository, "
                     "address incosistency.");
            }
            AtomPtr conv_atom(conv->get(oldHandle));
            tbl->add(conv_atom->getHandle(), t);
        }
    }
    CHECK_FREAD;
}

void TemporalTableFile::save(FILE *fp, TemporalTable *tbl)
{
    // writes table size (number of temporal entries)
    int size = tbl->temporalMap->getCount();
    fwrite(&size, sizeof(int), 1, fp);
    TemporalEntry* currentEntry = tbl->sortedTemporalList;
    while (currentEntry != NULL) {
        // writes current temporal entry
        Temporal* t = currentEntry->time;
        bool isNormal = t->isNormal();
        fwrite(&isNormal, sizeof(bool), 1, fp);
        unsigned long a = t->getA();
        fwrite(&a, sizeof(unsigned long), 1, fp);
        unsigned long b = t->getB();
        fwrite(&b, sizeof(unsigned long), 1, fp);
        // writes the number of associated handles to the current temporal entry
        UnorderedHandleSet* hs = tbl->temporalMap->get(t);
        int setSize = hs->size();
        fwrite(&setSize, sizeof(int), 1, fp);
        UnorderedHandleSet::iterator itr = hs->begin();
        while (itr != hs->end()) {
            // writes each associated handle
            Handle handle = *itr;
            ++itr;
            UUID uuid = handle.value();
            fwrite(&uuid, sizeof(UUID), 1, fp);
        }
        currentEntry = currentEntry->next;
    }
}
