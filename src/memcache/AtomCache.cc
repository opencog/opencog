/**
 * AtomCache.cc
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

#include <string>

#include <memcached.h>

#include "Atom.h"
#include "AtomTable.h"
#include "AtomCache.h"

using namespace opencog;

AtomCache::AtomCache(const std::string dbname, int portno)
{
}

#endif /* HAVE_LIBMEMCACHED */

