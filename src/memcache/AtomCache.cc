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
#include "AtomCache.h"
#include "AtomTable.h"
#include "ClassServer.h"
#include "Node.h"
#include "Link.h"
#include "SimpleTruthValue.h"
#include "TLB.h"
#include "TruthValue.h"

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

	store_count = 0;
}

AtomCache::~AtomCache()
{
	memcached_free(mc);
}

/* ================================================================== */

#define CHECK_RC(rc) \
	if(MEMCACHED_SUCCESS != rc) \
	{ \
		fprintf(stderr, "Error: memcachedb: %s\n", memcached_strerror(mc, rc)); \
		return; \
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

	// Store the truth value
	const TruthValue &tv = atom->getTruthValue();
	const SimpleTruthValue *stv = dynamic_cast<const SimpleTruthValue *>(&tv);
	if (NULL == stv)
	{
		fprintf(stderr, "Error: non-simple truth values are not handled\n");
		return;
	}

	vlen = snprintf(valbuff, VBSIZE, "(%20.16g, %20.16g)", tv.getMean(), tv.getCount());
	strcpy(p, "stv");
	rc = memcached_set (mc, keybuff, rootlen+3, valbuff, vlen, 0, 0);
	CHECK_RC(rc);
}

/* ================================================================== */

#define NCHECK_RC(rc, val) \
	if(MEMCACHED_SUCCESS != rc) \
	{ \
		fprintf(stderr, "Error: memcachedb: %s\n", memcached_strerror(mc, rc)); \
		if (val) free(val); \
		return atom; \
	}

Atom * AtomCache::getAtom(Handle h)
{
	size_t vlen;
	uint32_t flags;
	memcached_return rc;
	char * val;

	char keybuff[KBSIZE];
	int rootlen = snprintf(keybuff, KBSIZE, "%lu/", h);
	char *p = &keybuff[rootlen];

	// Does the atom exist already ?
	Atom *atom = TLB::getAtom(h);

	if (NULL == atom)
	{
		// Get the atom type.
		strcpy(p, "type");
		val = memcached_get(mc, keybuff, rootlen+4, &vlen, &flags, &rc);
		NCHECK_RC(rc, val);
		int atype = atoi(val);
		free(val);
	
		if (ClassServer::isAssignableFrom(NODE, atype))
		{
			// Get the atom name
			strcpy(p, "name");
			val = memcached_get(mc, keybuff, rootlen+4, &vlen, &flags, &rc);
			NCHECK_RC(rc, val);
			atom = new Node(atype, val);
			free(val);
		}
		else
		{
			// Get the outvec.
			strcpy(p, "edges");
			val = memcached_get(mc, keybuff, rootlen+5, &vlen, &flags, &rc);
			NCHECK_RC(rc, val);

			int arity = atoi(val+1);
			std::vector<Handle> outvec;
			outvec.resize(arity);

			char *comma = strchr(val+2, ',');
			for (int i=0; i<arity; i++)
			{
				Handle ho = (Handle) strtoul(comma+1, &comma, 10);
				outvec.at(i) = ho;
			}
			atom = new Link(atype, outvec);
		}
	}

	// Fetch the truth value
	const TruthValue &tv = atom->getTruthValue();
	const SimpleTruthValue *stv = dynamic_cast<const SimpleTruthValue *>(&tv);
	if (NULL == stv)
	{
		fprintf(stderr, "Error: non-simple truth values are not handled\n");
		return atom;
	}

	strcpy(p, "stv");
	val = memcached_get(mc, keybuff, rootlen+3, &vlen, &flags, &rc);
	NCHECK_RC(rc, val);

	double mean = atof(val+1);
	char *comma = strchr(val+2, ',');
	double count = atof(comma+1);
	SimpleTruthValue nstv(mean,count);
	atom->setTruthValue(nstv);

	return atom;
}

/* ================================================================== */

void AtomCache::load(AtomTable &at)
{
}

bool AtomCache::store_cb(Atom *atom)
{
	storeAtom(atom);
	store_count ++;
	if (store_count%1000 == 0)
	{
		fprintf(stderr, "\tStored %lu atoms.\n", store_count);
	}
	return false;
}

void AtomCache::store(const AtomTable &table)
{
	table.foreach_atom(&AtomCache::store_cb, this);
}

#endif /* HAVE_LIBMEMCACHED */

/* ======================= END OF FILE ============================== */
