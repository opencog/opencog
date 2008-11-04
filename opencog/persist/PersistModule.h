/*
 * opencog/persist/PersistModule.h
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

#ifndef _OPENCOG_PERSIST_MODULE_H
#define _OPENCOG_PERSIST_MODULE_H

#include <opencog/persist/SQLOpenRequest.h>
#include <opencog/persist/SQLCloseRequest.h>
#include <opencog/persist/SQLLoadRequest.h>
#include <opencog/persist/SQLStoreRequest.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

namespace opencog
{

class AtomStorage;

class PersistModule : public Module
{

private:

    AtomStorage* store;

    Factory<SQLOpenRequest,  Request> sqlopenFactory;
    Factory<SQLCloseRequest, Request> sqlcloseFactory;
    Factory<SQLLoadRequest,  Request> sqlloadFactory;
    Factory<SQLStoreRequest, Request> sqlstoreFactory;

public:

    const char* id(void);
#if 0
    static inline const char* id() {
        static const char* _id = "opencog::PersistModule";
        return _id;
    }
#endif

    PersistModule();
    virtual ~PersistModule();

    virtual void         init    (void);
    virtual void         setStore(AtomStorage *);
    virtual AtomStorage* getStore(void);

}; // class

}  // namespace

#endif // _OPENCOG_PERSIST_MODULE_H
