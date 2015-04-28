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

#ifndef _OPENCOG_ATOM_STORAGE_H
#define _OPENCOG_ATOM_STORAGE_H

#ifdef HAVE_LIBMEMCACHED

#include <string>
#include <memcached.h>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomTable.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class AtomCache
{
	private:
		memcached_st *mc;
		memcached_return connect_status;

		unsigned long store_count;
		bool store_cb(Atom *);

		int depth(Atom *);
		int maxdepth;
#define MAX_LATTICE_DEPTH 20
		std::string depth_list[MAX_LATTICE_DEPTH];

		unsigned long load_count;
		void load_list(AtomTable &, int);

	public:
		AtomCache(const std::string server, int portno);
		~AtomCache();


		void storeAtom(Atom *);
		Atom * getAtom(Handle);
		void load(AtomTable &);
		void store(const AtomTable &);
};

/** @}*/
}

#endif // HAVE_LIBMEMCACHED

#endif // _OPENCOG_ATOM_STORAGE_H
