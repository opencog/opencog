/*
 * opencog/persist/file/TimeServerSavable.cc
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

#include "TimeServerSavable.h"
#include "TemporalTableFile.h"

#include <opencog/util/Logger.h>

using namespace opencog;

TimeServerSavable::TimeServerSavable()
{
    timeserver = NULL;
}

TimeServerSavable::~TimeServerSavable()
{
    timeserver = NULL;
}

const char* TimeServerSavable::getId() const
{
    static const char* id = "TimeServer";
    return id;
}

void TimeServerSavable::saveRepository(FILE* fp) const
{
    logger().debug("Saving %s (%ld)\n", getId(), ftell(fp));
    // Saves TemporalTable
    TemporalTableFile ttf;
    ttf.save(fp, timeserver->table);
}

void TimeServerSavable::loadRepository(FILE* fp, HandMapPtr conv)
{
    logger().debug("Loading %s (%ld)\n", getId(), ftell(fp));
    // Loads the TemporalTable
    TemporalTableFile ttf;
    ttf.load(fp, timeserver->table, conv);
}

void TimeServerSavable::clear()
{
    timeserver->clear();
}

