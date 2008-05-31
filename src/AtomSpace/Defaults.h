/*
 * src/AtomSpace/Defaults.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef DEFAULTS_H
#define DEFAULTS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ClassServer.h"
#include "types.h"

/**
 * This class is a temporary solution for dynamics values until a more
 * thorough parameter system is defined.
 */
class Defaults
{

private:

    /**
     * Private default constructor for this class to make it abstract.
     */
    Defaults() {}

public:

    /**
     * Returns the default importance value for a given atom type.
     *
     * @param Atom type.
     * @return Default importance value for a given atom type.
     */
    static float getDefaultImportance(Type type) {
        return 1;
    }

    /**
     * Returns the default heat value for a given atom type.
     *
     * @param Atom type.
     * @return Default heat value for a given atom type.
     */
    static float getDefaultHeat(Type type) {
        return 0;
    }
};

#endif
