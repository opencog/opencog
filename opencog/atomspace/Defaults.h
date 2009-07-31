/*
 * opencog/atomspace/Defaults.h
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

#ifndef _OPENCOG_DEFAULTS_H
#define _OPENCOG_DEFAULTS_H

#include <opencog/atomspace/types.h>

namespace opencog
{

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

};

} // namespace opencog

#endif // _OPENCOG_DEFAULTS_H
