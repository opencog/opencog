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

AtomCache::AtomCache(const std::string server, int portno)
{
	memcached_return rc;
	mc = memcached_create(NULL);

	memcached_server_st *servers;
	const char *servername = server.c_str();
	servers = memcached_server_list_append(NULL, (char *) servername, portno, &rc);
	
	connect_status = memcached_server_push(mc, servers);

	memcached_server_list_free(servers);

}

AtomCache::~AtomCache()
{
	memcached_free(mc);
}

#endif /* HAVE_LIBMEMCACHED */

