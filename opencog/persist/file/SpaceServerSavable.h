/*
 * opencog/persist/file/SpaceServerSavable.h
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
#ifndef _OPENCOG_SPACESERVER_SAVABLE_H
#define _OPENCOG_SPACESERVER_SAVABLE_H

/**
 * SpaceServerSavable.h
 * This class implements SavableRepository so that it can be saved and loaded
 * by SavingLoading class.
 *
 * @author Welter Luigi
 */
#include <map>

#include <opencog/spacetime/SpaceServer.h>
#include <opencog/persist/file/SavableRepository.h>
#include <opencog/atomspace/types.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class SpaceServerSavable : public SavableRepository
{
private:
    SpaceServer *server;

public:
    explicit SpaceServerSavable();
    virtual ~SpaceServerSavable();

    void setServer(SpaceServer *s) { server = s; }

    const char* getId() const;
    void saveRepository(FILE*) const;
    void loadRepository(FILE*, HandMapPtr);
    void clear();

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_SPACESERVER_SAVABLE_H
