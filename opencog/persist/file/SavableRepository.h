/*
 * opencog/persist/file/SavableRepository.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Rodrigo Barra
 *            Carlos Lopes <dlopes@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_SAVABLE_REPOSITORY_H
#define _OPENCOG_SAVABLE_REPOSITORY_H

#include <stdio.h>

#include <opencog/atomspace/HandleMap.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class Atom;

/**
 * This interface should be implemented by any
 * Repositories that want to be called by the
 * SavingLoading. These repositories should
 * register in the SavingLoading using the
 * method SavingLoading::addSavableRepository.
 */
class SavableRepository
{

protected:

    /**
     * Protected constructor so that this class is abstract.
     */
    SavableRepository() {}

public:

    virtual ~SavableRepository() {}

    /**
     * Returns an identifier for the Repository.
     */
    virtual const char* getId() const = 0;

    /**
     * Saves the repository to a file.
     *
     * @param The file where the repository should be saved.
     */
    virtual void saveRepository(FILE *) const = 0;

    /**
     * Loads the repository from a file.
     *
     * @param The file from where the repository should be loaded.
     */
    virtual void loadRepository(FILE *, HandleMap<Atom*>*) = 0;


    /**
     * This method is used to clear the Repository.
     */
    virtual void clear() = 0;

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_SAVABLE_REPOSITORY_H
