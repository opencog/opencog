/*
 * opencog/persist/file/TemporalTableFile.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Carlos Lopes <dlopes@vettalabs.com>
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

#ifndef _OPENCOG_TEMPORAL_TABLE_FILE_H
#define _OPENCOG_TEMPORAL_TABLE_FILE_H

#include <opencog/spacetime/TemporalTable.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class TemporalTableFile
{

public:

    TemporalTableFile();
    virtual ~TemporalTableFile();

    /**
     * This method saves this table in the file specified.
     * @param the file pointer where the TemporalTable must be saved.
     */
    void save(FILE*, TemporalTable*);

    /**
     * This method loads a TemporalTable stored in the file specified.
     * @param the file pointer where the TemporalTable is stored.
     * @param a map of old Handles (stored in the file) to new Handles (in the current memory).
     */
    typedef std::shared_ptr<HandleMap<AtomPtr>> HandMapPtr;
    void load(FILE*, TemporalTable*, HandMapPtr);

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TEMPORAL_TABLE_H
