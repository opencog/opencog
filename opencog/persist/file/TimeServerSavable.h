/*
 * opencog/persistfile/TimeServerSavable.h
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

#ifndef _OPENCOG_TIME_SERVER_SAVABLE_H
#define _OPENCOG_TIME_SERVER_SAVABLE_H

#include <set>

#include <opencog/persist/file/SavableRepository.h>
#include <opencog/spacetime/TimeServer.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

/**
 * This class implements SavableRepository so that it can be saved and loaded by
 * SavingLoading class.
 */
class TimeServerSavable : public SavableRepository
{
private:
    TimeServer *timeserver;

public:
    TimeServerSavable();
    virtual ~TimeServerSavable();

    void setServer(TimeServer *ts) { timeserver = ts; }

    /**
     * Returns an identifier for the Repository.
     */
    const char* getId() const;

    /**
     * This method stores this repository in the file specified.
     * @param the file pointer where the TimeServer must be saved.
     */
    void saveRepository(FILE *) const;

    /**
     * This method loads a repository stored in the file specified.
     * @param the file pointer where the TimeServer is stored.
     * @param a map of old Handles (stored in the file) to new Handles (in the current memory).
     */
    void loadRepository(FILE *, HandleMap<Atom *> *);

    /**
     * This method is used to clear the whole TimeServer repository.
     */
    void clear();
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TIME_SERVER_SAVABLE_H
