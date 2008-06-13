/**
 * AtomCache.h
 *
 * Persistant Atom storage, backed by memcachedb
 *
 * Atoms are saved to, and restored from one, or more, memcachdb 
 * servers.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_LIBMEMCACHED

#ifndef MEMCACHE_ATOM_STORAGE_H_
#define MEMCACHE_ATOM_STORAGE_H_

#include <string>
#include <memcached.h>

#include "Atom.h"
#include "AtomTable.h"

namespace opencog
{

class AtomCache
{
	private:
		memcached_st *mc;
		memcached_return connect_status;

	public:
		AtomCache(const std::string server, int portno);
		~AtomCache();


		void storeAtom(Atom *);
		Atom * getAtom(Handle);
		void load(AtomTable &);
		void store(const AtomTable &);
};

};
#endif /* MEMCACHE_ATOM_STORAGE_H_ */

#endif /* HAVE_LIBMEMCACHED */

