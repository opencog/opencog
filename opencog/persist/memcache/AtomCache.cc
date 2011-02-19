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

#include "AtomCache.h"

#include <string>

#include <memcached.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/TruthValue.h>

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

	Handle h = atom->getHandle();

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
	
		if (classserver().isAssignableFrom(NODE, atype))
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

void AtomCache::load_list(AtomTable &table, int depth)
{
	size_t vlen;
	uint32_t flags;
	memcached_return rc;
	char * val;

	char keybuff[KBSIZE];
	size_t klen = snprintf(keybuff, KBSIZE, "depth-list-%d", depth);
	val = memcached_get(mc, keybuff, klen, &vlen, &flags, &rc);

printf("duude depth=%d nodelist len=%d\n", depth, vlen);
	if ((vlen == 0) || (NULL == val)) return;

	unsigned long ilc = load_count;
	char *comma = val;
	while (comma)
	{
		Handle h = (Handle) strtoul(comma+1, &comma, 10);
		getAtom(h);
		load_count ++;
		if (load_count%1000 == 0)
		{
			fprintf(stderr, "\tLoaded %lu atoms.\n", load_count);
		}
	}
	free(val);

	fprintf(stderr, "\tLoaded %lu atoms of depth %d.\n", load_count- ilc, depth);
}

void AtomCache::load(AtomTable &table)
{
	load_count = 0;
	for (int i=0; i<MAX_LATTICE_DEPTH; i++)
	{
		load_list(table, i);
	}
}

/* ================================================================== */

int AtomCache::depth(Atom *atom)
{
	Link *l = dynamic_cast<Link *>(atom);
	if (NULL == l) return 0;

	int maxd = 0;
	int arity = l->getArity();

	std::vector<Handle> out = l->getOutgoingSet();
	for (int i=0; i<arity; i++)
	{
		Handle h = out[i];
		int d = depth(TLB::getAtom(h));
		if (maxd < d) maxd = d;
	}
	return maxd +1;
}

/* ================================================================== */

bool AtomCache::store_cb(Atom *atom)
{
	Handle h = atom->getHandle();

	// Build an index of atoms of a given depth
	char hbuff[KBSIZE];
	snprintf(hbuff, KBSIZE, "%lu, ", h);

	int d = depth(atom);
	depth_list[d] += hbuff;
	if (maxdepth < d) maxdepth = d;

	// store the acctual atom.
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
	memcached_return rc;
	int i;
	maxdepth = 0;

	for (i=0; i<MAX_LATTICE_DEPTH; i++)
	{
		depth_list[i] = "(";
	}

	table.foreach_atom(&AtomCache::store_cb, this);

	// store the index lists too
	for (i=0; i<maxdepth; i++)
	{
		depth_list[i] += ")";

		char keybuff[KBSIZE];
		size_t klen = snprintf(keybuff, KBSIZE, "depth-list-%d", i);
		rc = memcached_set (mc, keybuff, klen, depth_list[i].c_str(), depth_list[i].size(), 0, 0);
		CHECK_RC(rc);
	}
}

#endif /* HAVE_LIBMEMCACHED */

/* ======================= END OF FILE ============================== */
