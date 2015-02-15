/*
 * opencog/persist/sql/PersistSCM.cc
 *
 * Copyright (c) 2008 by OpenCog Foundation
 * Copyright (c) 2008, 2009, 2013, 2015 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/BackingStore.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/types/atom_types.h>

#include "PersistModule.h"
#include "AtomStorage.h"

using namespace opencog;

namespace opencog {
class SQLBackingStore : public BackingStore
{
	private:
		AtomStorage *_store;
	public:
		SQLBackingStore();
		void set_store(AtomStorage *);

		virtual NodePtr getNode(Type, const char *) const;
		virtual LinkPtr getLink(Type, const HandleSeq&) const;
		virtual AtomPtr getAtom(Handle) const;
		virtual HandleSeq getIncomingSet(Handle) const;
		virtual void storeAtom(Handle);
		virtual void loadType(AtomTable&, Type);
		virtual void barrier();
};
};

SQLBackingStore::SQLBackingStore()
{
	_store = NULL;
}

void SQLBackingStore::set_store(AtomStorage *as)
{
	_store = as;
}

NodePtr SQLBackingStore::getNode(Type t, const char *name) const
{
	return _store->getNode(t, name);
}

LinkPtr SQLBackingStore::getLink(Type t, const std::vector<Handle>& oset) const
{
	return _store->getLink(t, oset);
}

AtomPtr SQLBackingStore::getAtom(Handle h) const
{
	return _store->getAtom(h);
}

HandleSeq SQLBackingStore::getIncomingSet(Handle h) const
{
	return _store->getIncomingSet(h);
}

void SQLBackingStore::storeAtom(Handle h)
{
	_store->storeAtom(h);
}

void SQLBackingStore::loadType(AtomTable& at, Type t)
{
	_store->loadType(at, t);
}

void SQLBackingStore::barrier()
{
	_store->flushStoreQueue();
}

PersistSCM::PersistSCM(AtomSpace *as)
{
	_as = as;
	_store = NULL;
	_backing = new SQLBackingStore();

	// XXX FIXME Huge hack alert.
	// As of 2013, no one uses this thing, except for NLP processing.
	// Since I'm too lazy to find an elegant solution right now, I'm
	// just going to hack this in.  Fix this someday.
	//
	// Anyway, what the below does is to ignore these certain types,
	// when they are to be fetched from the backing store.  This can
	// speed up document processing, since we know that word instances
	// and documents and sentences will not be stored in the database.
	// Thus, we don't even try to fetch these.

#define NLP_HACK 1
#ifdef NLP_HACK
	_backing->_ignored_types.insert(VARIABLE_NODE);
	_backing->_ignored_types.insert(VARIABLE_TYPE_NODE);
	_backing->_ignored_types.insert(TYPED_VARIABLE_LINK);
	_backing->_ignored_types.insert(BIND_LINK);

	_backing->_ignored_types.insert(DOCUMENT_NODE);
	_backing->_ignored_types.insert(SENTENCE_NODE);
	_backing->_ignored_types.insert(PARSE_NODE);
	_backing->_ignored_types.insert(PARSE_LINK);
	_backing->_ignored_types.insert(WORD_INSTANCE_NODE);
	_backing->_ignored_types.insert(WORD_INSTANCE_LINK);
#endif // NLP_HACK

#ifdef HAVE_GUILE
	define_scheme_primitive("sql-open", &PersistSCM::do_open, this);
	define_scheme_primitive("sql-close", &PersistSCM::do_close, this);
	define_scheme_primitive("sql-load", &PersistSCM::do_load, this);
	define_scheme_primitive("sql-store", &PersistSCM::do_store, this);

	// XXX These should be declared in some generic persistance module,
	// as they are not specific to the SQL backend only.  But I guess
	// this is an OK home for now, because there are no other backends.
	define_scheme_primitive("fetch-atom", &PersistSCM::fetch_atom, this);
	define_scheme_primitive("fetch-incoming-set", &PersistSCM::fetch_incoming_set, this);
	define_scheme_primitive("store-atom", &PersistSCM::store_atom, this);
	define_scheme_primitive("load-atoms-of-type", &PersistSCM::load_type, this);
	define_scheme_primitive("barrier", &PersistSCM::barrier, this);
#endif
}

PersistSCM::~PersistSCM()
{
	delete _backing;
}

void PersistSCM::do_open(const std::string& dbname,
                         const std::string& username,
                         const std::string& auth)
{
	_store = new AtomStorage(dbname, username, auth);
	if (!_store)
		throw RuntimeException(TRACE_INFO,
			"sql-open: Error: Unable to open the database");

	if (!_store->connected())
	{
		delete _store;
		_store = NULL;
		throw RuntimeException(TRACE_INFO,
			"sql-open: Error: Unable to connect to the database");
	}

	// reserve() is critical here, to reserve UUID range.
	_store->reserve();
	_backing->set_store(_store);
	_as->getImpl().registerBackingStore(_backing);
}

void PersistSCM::do_close(void)
{
	if (_store == NULL)
		throw RuntimeException(TRACE_INFO,
			 "sql-close: Error: Database not open");

	_as->getImpl().unregisterBackingStore(_backing);

	_backing->set_store(NULL);
	delete _store;
	_store = NULL;
}

void PersistSCM::do_load(void)
{
	if (_store == NULL)
		throw RuntimeException(TRACE_INFO,
			"sql-load: Error: Database not open");

	// XXX TODO: this should probably be done in a separate thread.
	_store->load(const_cast<AtomTable&>(_as->getAtomTable()));
}


void PersistSCM::do_store(void)
{
	if (_store == NULL)
		throw RuntimeException(TRACE_INFO,
			"sql-store: Error: Database not open");

	// XXX TODO This should really be started in a new thread ...
	_store->store(const_cast<AtomTable&>(_as->getAtomTable()));
}

// =====================================================================
// XXX TODO: the methods  below really belong in their own module,
// independent of this SQL module; they would be applicable for
// any backend, not just the SQL backend.

Handle PersistSCM::fetch_atom(Handle h)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("fetch-atom");
	h = as->getImpl().fetchAtom(h);
	return h;
}

Handle PersistSCM::fetch_incoming_set(Handle h)
{
	// The "false" flag here means that the fetch is NOT recursive.
	AtomSpace *as = SchemeSmob::ss_get_env_as("fetch-incoming-set");
	h = as->getImpl().fetchIncomingSet(h, false);
	return h;
}

/**
 * Store the single atom to the backing store hanging off the
  atom-space
 */
Handle PersistSCM::store_atom(Handle h)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("store-atom");
	as->getImpl().storeAtom(h);
	return h;
}

void PersistSCM::load_type(Type t)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("load-atoms-of-type");
	as->getImpl().loadType(t);
}

void PersistSCM::barrier(void)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("barrier");
	as->getImpl().barrier();
}
