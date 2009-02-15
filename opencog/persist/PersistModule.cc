/*
 * opencog/persist/PersistModule.cc
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

#include "PersistModule.h"
#include "AtomStorage.h"

#include <opencog/server/CogServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/BackingStore.h>

using namespace opencog;

namespace opencog {
class SQLBackingStore : public BackingStore
{
	private:
		AtomStorage *store;
	public:
		SQLBackingStore(void);
		void set_store(AtomStorage *);

		virtual Handle getHandle(Type, const char *) const;
		virtual Handle getHandle(Type, const std::vector<Handle>&) const;
		virtual void storeAtom(Handle);
};
};

SQLBackingStore::SQLBackingStore(void)
{
	store = NULL;
}

void SQLBackingStore::set_store(AtomStorage *as)
{
	store = as;
}

Handle SQLBackingStore::getHandle(Type t, const char *name) const
{
	Node *n = store->getNode(t, name);
	if (!n) return Handle::UNDEFINED;
	return TLB::getHandle(n);
}

Handle SQLBackingStore::getHandle(Type t, const std::vector<Handle>& oset) const
{
	Link *l = store->getLink(t, oset);
	if (!l) return Handle::UNDEFINED;
	return TLB::getHandle(l);
}

void SQLBackingStore::storeAtom(Handle h)
{
	store->storeAtom(h);
}

DECLARE_MODULE(PersistModule);

PersistModule::PersistModule(void) : store(NULL)
{
	backing = new SQLBackingStore();
	do_close_register();
	do_load_register();
	do_open_register();
	do_store_register();
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
		return "sql-close: Wrong num args";

	if (store == NULL)
		return "sql-close: database not open";

	atomspace().unregisterBackingStore(backing);

	backing->set_store(NULL);
	delete store;
	store = NULL;
	return "database closed";
}

std::string PersistModule::do_load(Request *dummy, std::list<std::string> args)
{
	if (!args.empty()) 
		return "sql-load: Wrong num args";

	if (store == NULL)
		return "sql-load: database not open";

	store->load(atomtable());

	return "database load started";
}


std::string PersistModule::do_open(Request *dummy, std::list<std::string> args)
{
	if (args.size() != 3)
		return "sql-load: Wrong num args";

	std::string dbname   = args.front(); args.pop_front();
	std::string username = args.front(); args.pop_front();
	std::string auth	   = args.front(); args.pop_front();

	store = new AtomStorage(dbname, username, auth);
	if (!store)
		return "sql-open: Unable to open the database";

	if (!store->connected())
	{
		delete store;
		store = NULL;
		return "sql-open: Unable to connect to the database";
	}

	// reserve() is critical here, to reserve UUID range.
	store->reserve();
	backing->set_store(store);
	atomspace().registerBackingStore(backing);

	return "database opened";
}

std::string PersistModule::do_store(Request *dummy, std::list<std::string> args)
{
	if (!args.empty()) 
		return "sql-store: Wrong num args";

	if (store == NULL)
		return "sql-store: database not open";

	store->store(atomtable());

	return "database store started";
}
