/*
 * opencog/persist/PersistModule.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
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

#include "PersistModule.h"
#include "AtomStorage.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/BackingStore.h>
#include <opencog/guile/SchemePrimitive.h>

using namespace opencog;

namespace opencog {
class SQLBackingStore : public BackingStore
{
	private:
		AtomStorage *store;
	public:
		SQLBackingStore();
		void set_store(AtomStorage *);

		virtual NodePtr getNode(Type, const char *) const;
		virtual LinkPtr getLink(Type, const HandleSeq&) const;
		virtual AtomPtr getAtom(Handle) const;
		virtual HandleSeq getIncomingSet(Handle) const;
		virtual void storeAtom(Handle);
};
};

SQLBackingStore::SQLBackingStore()
{
	store = NULL;
}

void SQLBackingStore::set_store(AtomStorage *as)
{
	store = as;
}

NodePtr SQLBackingStore::getNode(Type t, const char *name) const
{
	return store->getNode(t, name);
}

LinkPtr SQLBackingStore::getLink(Type t, const std::vector<Handle>& oset) const
{
	return store->getLink(t, oset);
}

AtomPtr SQLBackingStore::getAtom(Handle h) const
{
	return store->getAtom(h);
}

HandleSeq SQLBackingStore::getIncomingSet(Handle h) const
{
	return store->getIncomingSet(h);
}

void SQLBackingStore::storeAtom(Handle h)
{
	store->storeAtom(h);
}

DECLARE_MODULE(PersistModule);

PersistModule::PersistModule(CogServer& cs) : Module(cs), store(NULL)
{
	backing = new SQLBackingStore();
	do_close_register();
	do_load_register();
	do_open_register();
	do_store_register();

#ifdef HAVE_GUILE
	// XXX These probably should be declared by the atom-space directly,
	// instead of being declared here ... but I guess this is an OK
	// home for now.
	define_scheme_primitive("fetch-atom", &PersistModule::fetch_atom, this);
	define_scheme_primitive("fetch-incoming-set", &PersistModule::fetch_incoming_set, this);
	define_scheme_primitive("store-atom", &PersistModule::store_atom, this);
#endif
}

PersistModule::~PersistModule()
{
	do_close_unregister();
	do_load_unregister();
	do_open_unregister();
	do_store_unregister();
	delete backing;
}

void PersistModule::init(void)
{
}

std::string PersistModule::do_close(Request *dummy, std::list<std::string> args)
{
	if (!args.empty()) 
		return "sql-close: Error: Unexpected argument";

	if (store == NULL)
		return "sql-close: Error: Database not open";

	_cogserver.getAtomSpace().getImpl().unregisterBackingStore(backing);

	backing->set_store(NULL);
	delete store;
	store = NULL;
	return "Database closed";
}

std::string PersistModule::do_load(Request *dummy, std::list<std::string> args)
{
	if (!args.empty()) 
		return "sql-load: Error: Unexpected argument";

	if (store == NULL)
		return "sql-load: Error: Database not open";

	// XXX TODO: this should probably be done in a separate thread.
	store->load(const_cast<AtomTable&>(_cogserver.getAtomSpace().getAtomTable()));

	return "Database load completed";
}


std::string PersistModule::do_open(Request *dummy, std::list<std::string> args)
{
	if (args.size() != 3)
		return "sql-open: Error: invalid command syntax\n"
		       "Usage: sql-open <dbname> <username> <auth>";

	std::string dbname   = args.front(); args.pop_front();
	std::string username = args.front(); args.pop_front();
	std::string auth	   = args.front(); args.pop_front();

	store = new AtomStorage(dbname, username, auth);
	if (!store)
		return "sql-open: Error: Unable to open the database";

	if (!store->connected())
	{
		delete store;
		store = NULL;
		return "sql-open: Error: Unable to connect to the database";
	}

	// reserve() is critical here, to reserve UUID range.
	store->reserve();
	backing->set_store(store);
	_cogserver.getAtomSpace().getImpl().registerBackingStore(backing);

	std::string rc = "Opened \"" + dbname + "\" as user \"" + username + "\""; 
	return rc;
}

std::string PersistModule::do_store(Request *dummy, std::list<std::string> args)
{
	if (!args.empty()) 
		return "sql-store: Error: Unexpected argument";

	if (store == NULL)
		return "sql-store: Error: Database not open";

	// XXX TODO This should really be started in a new thread ...
	store->store(const_cast<AtomTable&>(_cogserver.getAtomSpace().getAtomTable()));

	return "Database store completed";
}

Handle PersistModule::fetch_atom(Handle h)
{
	AtomSpace *as = &_cogserver.getAtomSpace();
	h = as->getImpl().fetchAtom(h);
	return h;
}

Handle PersistModule::fetch_incoming_set(Handle h)
{
	AtomSpace *as = &_cogserver.getAtomSpace();
	// The "true" flag here means "fetch recursive".
	h = as->getImpl().fetchIncomingSet(h, true);
	return h;
}

/**
 * Store the single atom to the backing store hanging off the atom-space
 */
Handle PersistModule::store_atom(Handle h)
{
	AtomSpace *as = &_cogserver.getAtomSpace();
	as->getImpl().storeAtom(h);
	return h;
}

