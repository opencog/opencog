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
#include "Node.h"
#include "Link.h"
#include "TLB.h"

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

#define CHECK_RC(rc) \
	if(MEMCACHED_SUCCESS != rc) \
	{ \
		fprintf(stderr, "Error: memcachedb: %s\n", memcached_strerror(mc, rc)); \
	}

void AtomCache::storeAtom(Atom *atom)
{
	memcached_return rc;

	Handle h = TLB::getHandle(atom);

	// Set up the basic root of the key
#define KBSIZE 50
	char keybuff[KBSIZE];
	int rootlen = snprintf(keybuff, KBSIZE, "%lu/", h);
	char *p = &keybuff[rootlen];

	// The buffer for values.
#define VBSIZE 1050
	char valbuff[VBSIZE];

	// Get the atom type.
	Type t = atom->getType();
	strcpy(p, "type");
	int vlen = snprintf(valbuff, VBSIZE, "%d", t);

	rc = memcached_set (mc, keybuff, rootlen+4, valbuff, vlen, 0, 0);
	CHECK_RC(rc);

	// If a node, store the name
	Node *n = dynamic_cast<Node *>(atom);
	if (n)
	{
		strcpy(p, "name");
		const char *name = n->getName().c_str();
		vlen = n->getName().size();
		rc = memcached_set (mc, keybuff, rootlen+4, name, vlen, 0, 0);
		CHECK_RC(rc);
	}
	else
	{
		// Store the outgoing set
		Link *l = dynamic_cast<Link *>(atom);
		int arity = l->getArity();
		vlen = snprintf(valbuff, VBSIZE, "(%d", arity);

		std::vector<Handle> out = l->getOutgoingSet();
		for (int i=0; i<arity; i++)
		{
			vlen += snprintf(valbuff+vlen, VBSIZE-vlen, ", %lu", out[i]);
		}

		vlen += snprintf(valbuff+vlen, VBSIZE-vlen, ")");
		strcpy(p, "edges");
		rc = memcached_set (mc, keybuff, rootlen+5, valbuff, vlen, 0, 0);
		CHECK_RC(rc);
	}
}

Atom * AtomCache::getAtom(Handle h)
{
	return NULL;
}

void AtomCache::load(AtomTable &at)
{
}

void AtomCache::store(const AtomTable &at)
{
}

#endif /* HAVE_LIBMEMCACHED */

