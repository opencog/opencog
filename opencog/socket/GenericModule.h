/*
 * opencog/socket/GenericModule.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_GENERIC_MODULE_H
#define _OPENCOG_GENERIC_MODULE_H

#include <opencog/server/GenericShell.h>
#include <opencog/server/Module.h>

namespace opencog
{

class AtomStorage;

class GenericModule : public Module
{

private:

    static const unsigned int DEFAULT_PORT = 17002;

    GenericShell _shell;   
    unsigned short _port;

public:

    static inline const char* id() {
        static const char* _id = "opencog::GenericModule";
        return _id;
    }

    GenericModule();
    virtual ~GenericModule();

    virtual void         init  (void);
    virtual GenericShell* shell (void);

}; // class

}  // namespace

#endif // _OPENCOG_GENERIC_MODULE_H
