/*
 * FUNCTION:
 * Persistent Atom storage, SQL-backed.
 *
 * Atoms are saved to, and restored from, an SQL DB.
 * Atoms are identified by means of unique ID's, which are taken to
 * be the atom Handles, as maintained by the TLB. In particular, the
 * system here depends on the handles in the TLB and in the SQL DB
 * to be consistent (i.e. kept in sync).
 *
 * HISTORY:
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 *
 * LICENSE:
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
#ifdef HAVE_SQL_STORAGE

#include <stdlib.h>

#include "AtomStorage.h"
#include "odbcxx.h"

#include <opencog/util/platform.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/TruthValue.h>

using namespace opencog;

#define USE_INLINE_EDGES

/* ================================================================ */

/**
 * Utility class, hangs on to a single response to an SQL query,
 * and provides routines to parse it, i.e. walk the rows and columns,
 * converting each row into an Atom, or Edge.
 *
 * Intended to be allocated on stack, to avoid malloc overhead.
 * Methods are intended to be inlined, so as to avoid subroutine
 * call overhead.  It really *is* supposed to be a convenience wrapper. :-)
 */
class AtomStorage::Response
{
	public:
		ODBCRecordSet *rs;

		// Temporary cache of info about atom being assembled.
		Handle handle;
		int itype;
		const char * name;
		int tv_type;
		double mean;
		double confidence;
		double count;
		const char *outlist;
		int height;

		Response()
		{
			tname = "";
			itype = 0;
			intval = 0;
		}

		bool create_atom_column_cb(const char *colname, const char * colvalue)
		{
			// printf ("%s = %s\n", colname, colvalue);
			if (!strcmp(colname, "type"))
			{
				itype = atoi(colvalue);
			}
			else if (!strcmp(colname, "name"))
			{
				name = colvalue;
			}
			else if (!strcmp(colname, "outgoing"))
			{
				outlist = colvalue;
			}
			if (!strcmp(colname, "tv_type"))
			{
				tv_type = atoi(colvalue);
			}
			else if (!strcmp(colname, "stv_mean"))
			{
				mean = atof(colvalue);
			}
			else if (!strcmp(colname, "stv_confidence"))
			{
				confidence = atof(colvalue);
			}
			else if (!strcmp(colname, "stv_count"))
			{
				count = atof(colvalue);
			}
			else if (!strcmp(colname, "uuid"))
			{
				UUID uuid = strtoul(colvalue, NULL, 10);
				handle = Handle(uuid);
			}
			return false;
		}
		bool create_atom_cb(void)
		{
			// printf ("---- New atom found ----\n");
			rs->foreach_column(&Response::create_atom_column_cb, this);

			return false;
		}

		AtomTable *table;
		AtomStorage *store;
		bool load_all_atoms_cb(void)
		{
			// printf ("---- New atom found ----\n");
			rs->foreach_column(&Response::create_atom_column_cb, this);

			AtomPtr atom(store->makeAtom(*this, handle));
			table->add(atom);
			return false;
		}

		std::vector<Handle> *hvec;
		bool load_incoming_set_cb(void)
		{
			// printf ("---- New atom found ----\n");
			rs->foreach_column(&Response::create_atom_column_cb, this);

			store->makeAtom(*this, handle);
			hvec->push_back(handle);
			return false;
		}

		bool row_exists;
		bool row_exists_cb(void)
		{
			row_exists = true;
			return false;
		}

#ifndef USE_INLINE_EDGES
		// Temporary cache of info about the outgoing set.
		std::vector<Handle> *outvec;
		Handle dst;
		int pos;

		bool create_edge_cb(void)
		{
			// printf ("---- New edge found ----\n");
			rs->foreach_column(&Response::create_edge_column_cb, this);
			int sz = outvec->size();
			if (sz <= pos) outvec->resize(pos+1);
			outvec->at(pos) = dst;
			return false;
		}
		bool create_edge_column_cb(const char *colname, const char * colvalue)
		{
			// printf ("%s = %s\n", colname, colvalue);
			if (!strcmp(colname, "dst_uuid"))
			{
				dst = Handle(strtoul(colvalue, (char **) NULL, 10));
			}
			else if (!strcmp(colname, "pos"))
			{
				pos = atoi(colvalue);
			}
			return false;
		}
#endif /* USE_INLINE_EDGES */

		// deal twith the type-to-id map
		bool type_cb(void)
		{
			rs->foreach_column(&Response::type_column_cb, this);
			store->set_typemap(itype, tname);
			return false;
		}
		const char * tname;
		bool type_column_cb(const char *colname, const char * colvalue)
		{
			if (!strcmp(colname, "type"))
			{
				itype = atoi(colvalue);
			}
			else if (!strcmp(colname, "typename"))
			{
				tname = colvalue;
			}
			return false;
		}
#ifdef OUT_OF_LINE_TVS
		// Callbacks for SimpleTruthValues
		int tvid;
		bool create_tv_cb(void)
		{
			// printf ("---- New SimpleTV found ----\n");
			rs->foreach_column(&Response::create_tv_column_cb, this);
			return false;
		}
		bool create_tv_column_cb(const char *colname, const char * colvalue)
		{
			printf ("%s = %s\n", colname, colvalue);
			if (!strcmp(colname, "mean"))
			{
				mean = atof(colvalue);
			}
			else if (!strcmp(colname, "count"))
			{
				count = atof(colvalue);
			}
			return false;
		}

#endif /* OUT_OF_LINE_TVS */

		// get generic positive integer values
		unsigned long intval;
		bool intval_cb(void)
		{
			rs->foreach_column(&Response::intval_column_cb, this);
			return false;
		}
		bool intval_column_cb(const char *colname, const char * colvalue)
		{
			// we're not going to bother to check the column name ...
			intval = strtoul(colvalue, NULL, 10);
			return false;
		}

		// Get all handles in the database.
		std::set<Handle> *id_set;
		bool note_id_cb(void)
		{
			rs->foreach_column(&Response::note_id_column_cb, this);
			return false;
		}
		bool note_id_column_cb(const char *colname, const char * colvalue)
		{
			// we're not going to bother to check the column name ...
			UUID id = strtoul(colvalue, NULL, 10);
			Handle h(id);
			id_set->insert(h);
			return false;
		}

};

bool AtomStorage::idExists(const char * buff)
{
	Response rp;
	rp.row_exists = false;
	rp.rs = db_conn->exec(buff);
	rp.rs->foreach_row(&Response::row_exists_cb, &rp);
	rp.rs->release();
	return rp.row_exists;
}

/* ================================================================ */
#define BUFSZ 250

#ifndef USE_INLINE_EDGES
/**
 * Callback class, whose method is invoked on each outgoing edge.
 * The callback constructs an SQL query to store the edge.
 */
class AtomStorage::Outgoing
{
	private:
		ODBCConnection *db_conn;
		unsigned int pos;
		Handle src_handle;
	public:
		Outgoing (ODBCConnection *c, Handle h)
		{
			db_conn = c;
			src_handle = h;
			pos = 0;
		}
		bool each_handle (Handle h)
		{
			char buff[BUFSZ];
			UUID src_uuid = src_handle.value();
			UUID dst_uuid = h.value();
			snprintf(buff, BUFSZ, "INSERT  INTO Edges "
			        "(src_uuid, dst_uuid, pos) VALUES (%lu, %lu, %u);",
			        src_uuid, dst_uuid, pos);

			Response rp;
			rp.rs = db_conn->exec(buff);
			rp.rs->release();
			pos ++;
			return false;
		}
};

/**
 * Store the outgoing set of the atom.
 * Handle h must be the handle for the atom; its passed as an arg to
 * avoid having to look it up.
 */
void AtomStorage::storeOutgoing(AtomPtr atom, Handle h)
{
	Outgoing out(db_conn, h);

	foreach_outgoing_handle(h, &Outgoing::each_handle, &out);
}

#endif /* USE_INLINE_EDGES */

/* ================================================================ */
// Constructors

void AtomStorage::init(const char * dbname,
                       const char * username,
                       const char * authentication)
{
	db_conn = new ODBCConnection(dbname, username, authentication);
	type_map_was_loaded = false;
	max_height = 0;

	for (int i=0; i< TYPEMAP_SZ; i++)
	{
		db_typename[i] = NULL;
	}

	local_id_cache_is_inited = false;
	if (!connected()) return;

	reserve();
}

AtomStorage::AtomStorage(const char * dbname,
                         const char * username,
                         const char * authentication)
{
	init(dbname, username, authentication);
}

AtomStorage::AtomStorage(const std::string& dbname,
                         const std::string& username,
                         const std::string& authentication)
{
	init(dbname.c_str(), username.c_str(), authentication.c_str());
}

AtomStorage::~AtomStorage()
{
	if (!connected())
	{
		delete db_conn;
		db_conn = NULL;
		return;
	}

	setMaxUUID(getMaxObservedUUID());
	setMaxHeight(getMaxObservedHeight());
	delete db_conn;
	db_conn = NULL;

	for (int i=0; i< TYPEMAP_SZ; i++)
	{
		if (db_typename[i]) free(db_typename[i]);
	}
}

/**
 * connected -- return true if a successful connection to the 
 * database exists; else return false.
 */
bool AtomStorage::connected(void)
{
	return db_conn->connected();
}

/* ================================================================ */

#define STMT(colname,val) { \
	if(update) { \
		if (notfirst) { cols += ", "; } else notfirst = 1; \
		cols += colname; \
		cols += " = "; \
		cols += val; \
	} else { \
		if (notfirst) { cols += ", "; vals += ", "; } else notfirst = 1; \
		cols += colname; \
		vals += val; \
	} \
}

#define STMTI(colname,ival) { \
	char buff[BUFSZ]; \
	snprintf(buff, BUFSZ, "%u", ival); \
	STMT(colname, buff); \
}

#define STMTF(colname,fval) { \
	char buff[BUFSZ]; \
	snprintf(buff, BUFSZ, "%12.8g", fval); \
	STMT(colname, buff); \
}

/* ================================================================ */

#ifdef OUT_OF_LINE_TVS
/**
 * Return true if the indicated handle exists in the storage.
 */
bool AtomStorage::tvExists(int tvid)
{
	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "SELECT tvid FROM SimpleTVs WHERE tvid = %u;", tvid);
	return idExists(buff);
}

/**
 * Store the truthvalue of the atom.
 * Handle h must be the handle for the atom; its passed as an arg to
 * avoid having to look it up.
 */
int AtomStorage::storeTruthValue(AtomPtr atom, Handle h)
{
	int notfirst = 0;
	std::string cols;
	std::string vals;
	std::string coda;

	const TruthValue &tv = atom->getTruthValue();

	const SimpleTruthValue *stv = dynamic_cast<const SimpleTruthValue *>(&tv);
	if (NULL == stv)
	{
		fprintf(stderr, "Error: non-simple truth values are not handled\n");
		return 0;
	}

	int tvid = TVID(tv);

	// If its a stock truth value, there is nothing to do.
	if (tvid <= 4) return tvid;

	// Use the TLB Handle as the UUID.
	char tvidbuff[BUFSZ];
	snprintf(tvidbuff, BUFSZ, "%u", tvid);

	bool update = tvExists(tvid);
	if (update)
	{
		cols = "UPDATE SimpleTVs SET ";
		vals = "";
		coda = " WHERE tvid = ";
		coda += tvidbuff;
		coda += ";";
	}
	else
	{
		cols = "INSERT INTO SimpleTVs (";
		vals = ") VALUES (";
		coda = ");";
		STMT("tvid", tvidbuff);
	}

	STMTF("mean", tv.getMean());
	STMTF("count", tv.getCount());

	std::string qry = cols + vals + coda;
	Response rp;
	rp.rs = db_conn->exec(qry.c_str());
	rp.rs->release();

	return tvid;
}

/**
 * Return a new, unique ID for every truth value
 */
int AtomStorage::TVID(const TruthValue &tv)
{
	if (tv == TruthValue::NULL_TV()) return 0;
	if (tv == TruthValue::DEFAULT_TV()) return 1;
	if (tv == TruthValue::FALSE_TV()) return 2;
	if (tv == TruthValue::TRUE_TV()) return 3;
	if (tv == TruthValue::TRIVIAL_TV()) return 4;

	Response rp;
	rp.rs = db_conn->exec("SELECT NEXTVAL('tvid_seq');");
	rp.rs->foreach_row(&Response::tvid_seq_cb, &rp);
	rp.rs->release();
	return rp.tvid;
}

TruthValue* AtomStorage::getTV(int tvid)
{
	if (0 == tvid) return (TruthValue *) & TruthValue::NULL_TV();
	if (1 == tvid) return (TruthValue *) & TruthValue::DEFAULT_TV();
	if (2 == tvid) return (TruthValue *) & TruthValue::FALSE_TV();
	if (3 == tvid) return (TruthValue *) & TruthValue::TRUE_TV();
	if (4 == tvid) return (TruthValue *) & TruthValue::TRIVIAL_TV();

	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "SELECT * FROM SimpleTVs WHERE tvid = %u;", tvid);

	Response rp;
	rp.rs = db_conn->exec(buff);
	rp.rs->foreach_row(&Response::create_tv_cb, &rp);
	rp.rs->release();

	SimpleTruthValue *stv = new SimpleTruthValue(rp.mean,rp.count);
	return stv;
}

#endif /* OUT_OF_LINE_TVS */

/* ================================================================== */

/**
 * Return largest distance from this atom to any node under it.
 * Nodes have a height of 0, by definition.  Links that contain only
 * nodes in their outgoing set have a height of 1, by definition. 
 * The height of a link is, by definition, one more than the height
 * of the tallest atom in its outgoing set.
 * @note This can conversely be viewed as the depth of a tree.
 */
int AtomStorage::get_height(AtomPtr atom)
{
	LinkPtr l(LinkCast(atom));
	if (NULL == l) return 0;

	int maxd = 0;
	int arity = l->getArity();

	const HandleSeq& out = l->getOutgoingSet();
	for (int i=0; i<arity; i++)
	{
		Handle h = out[i];
		AtomPtr a = TLB::getAtom(h);
		int d = get_height(a);
		if (maxd < d) maxd = d;
	}
	return maxd +1;
}

/* ================================================================ */

std::string AtomStorage::oset_to_string(const std::vector<Handle>& out,
                                        int arity)
{ 
	std::string str;
	str += "\'{";
	for (int i=0; i<arity; i++)
	{
		Handle h = out[i];
		if (i != 0) str += ", ";
		char buff[BUFSZ];
		UUID uuid = h.value();
		snprintf(buff, BUFSZ, "%lu", uuid);
		str += buff;
	}
	str += "}\'";
	return str;
}

/* ================================================================ */
/**
 * Recursively store the indicated atom, and all that it points to.
 * Store its truth values too. The recursive store is unconditional;
 * its assumed that all sorts of underlying truuth values have changed, 
 * so that the whole thing needs to be stored.
 */
void AtomStorage::storeAtom(AtomPtr atom)
{
	get_ids();
	do_store_atom(atom, atom->getHandle());
}

void AtomStorage::storeAtom(Handle h)
{
	get_ids();
	AtomPtr atom = TLB::getAtom(h);
	do_store_atom(atom, h);
}

/**
 * Returns the height of the atom.
 */
int AtomStorage::do_store_atom(AtomPtr atom, Handle h)
{
	LinkPtr l(LinkCast(atom));
	if (NULL == l)
	{
		do_store_single_atom(atom, h, 0);
		return 0;
	}

	int lheight = 0;
	int arity = l->getArity();
	const HandleSeq& out = l->getOutgoingSet();
	for (int i=0; i<arity; i++)
	{
		Handle ho = out[i];
		AtomPtr ao = TLB::getAtom(ho);

		// Recurse.
		int heig = do_store_atom(ao, ho);
		if (lheight < heig) lheight = heig;
	}

	// Height of this link is, by definition, one more than tallest
	// atom in outgoing set.
	lheight ++;
	do_store_single_atom(atom, h, lheight);
	return lheight;
}

/* ================================================================ */
/**
 * Store the single, indicated atom.
 * Store its truth values too.
 */
void AtomStorage::storeSingleAtom(AtomPtr atom)
{
	get_ids();
	Handle h = atom->getHandle();
	int height = get_height(atom);
	do_store_single_atom(atom, h, height);
}

void AtomStorage::do_store_single_atom(AtomPtr atom, Handle h, int aheight)
{
	setup_typemap();

	int notfirst = 0;
	std::string cols;
	std::string vals;
	std::string coda;

	// Use the TLB Handle as the UUID.
	char uuidbuff[BUFSZ];
	UUID uuid = h.value();
	snprintf(uuidbuff, BUFSZ, "%lu", uuid);

	bool update = atomExists(h);
	if (update)
	{
		cols = "UPDATE Atoms SET ";
		vals = "";
		coda = " WHERE uuid = ";
		coda += uuidbuff;
		coda += ";";
	}
	else
	{
		cols = "INSERT INTO Atoms (";
		vals = ") VALUES (";
		coda = ");";

		STMT("uuid", uuidbuff);
	}

	// Store the atom type and node name only if storing for the
	// first time ever. Once an atom is in an atom table, it's
	// name can type cannot be changed. Only its truth value can
	// change.
	if (false == update)
	{
		// Store the atom UUID
		Type t = atom->getType();
		int dbtype = storing_typemap[t];
		STMTI("type", dbtype);
	
		// Store the node name, if its a node
		NodePtr n(NodeCast(atom));
		if (n)
		{
#if 0
			std::string qname = n->getName();
			escape_single_quotes(qname);
			qname.insert(0U,1U,'\'');
			qname += "'";
#else
			// Use postgres $-quoting to make unicode strings
			// easier to deal with. 
			std::string qname = " $ocp$";
			qname += n->getName();
			qname += "$ocp$ ";
#endif
			STMT("name", qname);

			// Nodes have a height of zero by definition.
			STMTI("height", 0);
		}
		else
		{
			if (max_height < aheight) max_height = aheight;
			STMTI("height", aheight);

#ifdef USE_INLINE_EDGES
			LinkPtr l(LinkCast(atom));
			if (l)
			{
				int arity = l->getArity();
				if (arity)
				{
					cols += ", outgoing";
					vals += ", ";
					vals += oset_to_string(l->getOutgoingSet(), arity);
				}
			}
#endif /* USE_INLINE_EDGES */
		}
	}

	// Store the truth value
	const TruthValue &tv = atom->getTruthValue();
	TruthValueType tvt = tv.getType();
	STMTI("tv_type", tvt);

	switch (tvt)
	{
		case SIMPLE_TRUTH_VALUE:
		case COUNT_TRUTH_VALUE:
			STMTF("stv_mean", tv.getMean());
			STMTF("stv_confidence", tv.getConfidence());
			STMTF("stv_count", tv.getCount());
			break;
		case INDEFINITE_TRUTH_VALUE:
		{
			const IndefiniteTruthValue *itv = static_cast<const IndefiniteTruthValue *>(&tv);
			STMTF("stv_mean", itv->getL());
			STMTF("stv_count", itv->getU());
			STMTF("stv_confidence", itv->getConfidenceLevel());
			break;
		}
		case COMPOSITE_TRUTH_VALUE:
			fprintf(stderr, "Error: Composite truth values are not handled\n");
			break;
		default:
			fprintf(stderr, "Error: store_single: Unknown truth value type\n");
	}

	std::string qry = cols + vals + coda;
	Response rp;
	rp.rs = db_conn->exec(qry.c_str());
	rp.rs->release();

#ifndef USE_INLINE_EDGES
	// Store the outgoing handles only if we are storing for the first
	// time, otherwise do nothing. The semantics is that, once the
	// outgoing set has been determined, it cannot be changed.
	if (false == update)
	{
		storeOutgoing(atom, h);
	}
#endif /* USE_INLINE_EDGES */

	// Make note of the fact that this atom has been stored.
	local_id_cache.insert(h);
}

/* ================================================================ */
/**
 * Store the concordance of type names to type values.
 *
 * The concordance is used to match up the type id's stored in 
 * the SQL database, against those currently in use in the current
 * version of the opencog server. The basic problem is that types
 * can be dynamic in OpenCog -- different versions will have 
 * different types, and will assign different type numbers to some
 * given type name. To overcome this, the SQL database stores all
 * atoms according to the type *name* -- although, to save space, it
 * actually stored type ids; however, the SQL type-name-to-type-id
 * mapping can be completely different than the OpenCog type-name
 * to type-id mapping. Thus, tables to convert the one to the other 
 * id are needed.
 *
 * Given an opencog type t, the storing_typemap[t] will contain the
 * sqlid for the named type. The storing_typemap[t] will *always*
 * contain a valid value.
 *
 * Given an SQL type sq, the loading_typemap[sq] will contain the 
 * opencog type t for the named type, or NOTYPE if this version of
 * opencog does not have this kind of atom.
 *
 * The typemaps must be constructed before any saving or loading of
 * atoms can happen. The typemaps will be a superset (union) of the
 * types used by OpenCog, and stored in the SQL table.
 */
void AtomStorage::setup_typemap(void)
{
	/* Only need to set up the typemap once. */
	if (type_map_was_loaded) return;
	type_map_was_loaded = true;

	// If we are here, we need to reconcile the types currently in
	// use, with a possibly pre-existing typemap. New types must be
	// stored.  So we start by loading a map from SQL (if its there).
	//
	// Be careful to initialize the typemap with invalid types,
	// in case there are unexpected holes in the map!
	for (int i=0; i< TYPEMAP_SZ; i++)
	{
		loading_typemap[i] = NOTYPE;
		storing_typemap[i] = -1;
		db_typename[i] = NULL;
	}

	Response rp;
	rp.rs = db_conn->exec("SELECT * FROM TypeCodes;");
	rp.store = this;
	rp.rs->foreach_row(&Response::type_cb, &rp);
	rp.rs->release();

	unsigned int numberOfTypes = classserver().getNumberOfClasses();
	for (Type t=0; t<numberOfTypes; t++)
	{
		int sqid = storing_typemap[t];
		/* If this typename is not yet known, record it */
		if (-1 == sqid)
		{
			const char * tname = classserver().getTypeName(t).c_str();

			// Let the sql id be the same as the current type number,
			// unless this sql number is already in use, in which case
			// we need to find another, unused one.  Its in use if we
			// have a string name associated to it.
			sqid = t;

			if ((db_typename[sqid] != NULL) &&
			    (loading_typemap[sqid] != t))
			{
				// Find some (any) unused type index to use in the
				// sql table. Use the lowest unused value that we
				// can find.
				for (sqid = 0; sqid<TYPEMAP_SZ; sqid++)
				{
					if (NULL == db_typename[sqid]) break;
				}

				if (TYPEMAP_SZ <= sqid)
				{
					fprintf(stderr, "Fatal Error: type table overflow!\n");
					abort();
				}
			}

			char buff[BUFSZ];
			snprintf(buff, BUFSZ,
			         "INSERT INTO TypeCodes (type, typename) "
			         "VALUES (%d, \'%s\');",
			         sqid, tname);
			rp.rs = db_conn->exec(buff);
			rp.rs->release();
			set_typemap(sqid, tname);
		}
	}
}

void AtomStorage::set_typemap(int dbval, const char * tname)
{
	Type realtype = classserver().getType(tname);
	loading_typemap[dbval] = realtype;
	storing_typemap[realtype] = dbval;
	if (db_typename[dbval] != NULL) free (db_typename[dbval]);
	db_typename[dbval] = strdup(tname);
}

/* ================================================================ */
/**
 * Return true if the indicated handle exists in the storage.
 */
bool AtomStorage::atomExists(Handle h)
{
#ifdef ASK_SQL_SERVER
	char buff[BUFSZ];
	UUID uuid = h.value();
	snprintf(buff, BUFSZ, "SELECT uuid FROM Atoms WHERE uuid = %lu;", uuid);
	return idExists(buff);
#else
	// look at the local cache of id's to see if the atom is in storage or not.
	return local_id_cache.count(h);
#endif
}

/**
 * Build up a client-side cache of all atom id's in storage
 */
void AtomStorage::get_ids(void)
{
	if (local_id_cache_is_inited) return;
	local_id_cache_is_inited = true;

	local_id_cache.clear();

	// It appears that, when the select statment returns more than
	// about a 100K to a million atoms or so, some sort of heap
	// corruption occurs in the odbc code, causing future mallocs
	// to fail. So limit the number of records processed in one go.
	// It also appears that asking for lots of records increases
	// the memory fragmentation (and/or there's a memory leak in odbc??)
#define USTEP 12003
	unsigned long rec;
	unsigned long max_nrec = getMaxUUID();
	for (rec = 0; rec <= max_nrec; rec += USTEP)
	{
		char buff[BUFSZ];
		snprintf(buff, BUFSZ, "SELECT uuid FROM Atoms WHERE "
		        "uuid > %lu AND uuid <= %lu;",
		         rec, rec+USTEP);

		Response rp;
		rp.id_set = &local_id_cache;
		rp.rs = db_conn->exec(buff);
		rp.rs->foreach_row(&Response::note_id_cb, &rp);
		rp.rs->release();
	}
}

/* ================================================================ */

#ifndef USE_INLINE_EDGES
void AtomStorage::getOutgoing(std::vector<Handle> &outv, Handle h)
{
	char buff[BUFSZ];
	UUID uuid = h.value();
	snprintf(buff, BUFSZ, "SELECT * FROM Edges WHERE src_uuid = %lu;", uuid);

	Response rp;
	rp.rs = db_conn->exec(buff);
	rp.outvec = &outv;
	rp.rs->foreach_row(&Response::create_edge_cb, &rp);
	rp.rs->release();
}
#endif /* USE_INLINE_EDGES */

/* ================================================================ */

/* One-size-fits-all atom fetcher */
AtomPtr  AtomStorage::getAtom(const char * query, int height)
{
	Response rp;
	rp.handle = Handle::UNDEFINED;
	rp.rs = db_conn->exec(query);
	rp.rs->foreach_row(&Response::create_atom_cb, &rp);

	// Did we actually find anything?
	// DO NOT USE TLB::IsInvalidHandle() HERE! It won't work, duhh!
	if (rp.handle.value() == Handle::UNDEFINED.value())
	{
		rp.rs->release();
		return NULL;
	}

	rp.height = height;
	AtomPtr atom = makeAtom(rp, rp.handle);
	rp.rs->release();
	return atom;
}

/**
 * Create a new atom, retreived from storage
 *
 * This method does *not* register the atom with any atomtable/atomspace
 * However, it does register with the TLB, as the SQL uuids and the
 * TLB Handles must be kept in sync, or all hell breaks loose.
 */
AtomPtr  AtomStorage::getAtom(Handle h)
{
	setup_typemap();
	char buff[BUFSZ];
	UUID uuid = h.value();
	snprintf(buff, BUFSZ, "SELECT * FROM Atoms WHERE uuid = %lu;", uuid);

	return getAtom(buff, -1);
}

/**
 * Retreive the entire incoming set of the indicated atom.
 */
std::vector<Handle> AtomStorage::getIncomingSet(Handle h)
{
	std::vector<Handle> iset;

	setup_typemap();
	char buff[BUFSZ];
	UUID uuid = h.value();
	snprintf(buff, BUFSZ, "SELECT * FROM Atoms WHERE outgoing @> ARRAY[%lu];", uuid);

	// Note: "select * from atoms where outgoing@>array[556];" will return
	// all links with atom 556 in the outgoing set -- i.e. the incoming set of 556.

	Response rp;
	rp.store = this;
	rp.height = -1;
	rp.hvec = &iset;
	rp.rs = db_conn->exec(buff);
	rp.rs->foreach_row(&Response::load_incoming_set_cb, &rp);
	rp.rs->release();

	return iset;
}

/**
 * Fetch Node from database, with the indicated type and name.
 * If there is no such node, NULL is returned.
 * More properly speaking, the point of this routine is really
 * to fetch the associated TruthValue for this node.
 *
 * This method does *not* register the atom with any atomtable/atomspace
 * However, it does register with the TLB, as the SQL uuids and the
 * TLB Handles must be kept in sync, or all hell breaks loose.
 */
NodePtr AtomStorage::getNode(Type t, const char * str)
{
	setup_typemap();
	char buff[40*BUFSZ];

	// Use postgres $-quoting to make unicode strings easier to deal with. 
	int nc = snprintf(buff, 4*BUFSZ, "SELECT * FROM Atoms WHERE "
	    "type = %hu AND name = $ocp$%s$ocp$ ;", storing_typemap[t], str);

	if (40*BUFSZ-1 <= nc)
	{
		fprintf(stderr, "Error: AtomStorage::getNode: buffer overflow!\n");
		buff[40*BUFSZ-1] = 0x0;
		fprintf(stderr, "\tnc=%d buffer=>>%s<<\n", nc, buff);
		return NULL;
	}

	return NodeCast(getAtom(buff, 0));
}

/**
 * Fetch Link from database, with the indicated type and outgoing set.
 * If there is no such link, NULL is returned.
 * More properly speaking, the point of this routine is really
 * to fetch the associated TruthValue for this link.
 *
 * This method does *not* register the atom with any atomtable/atomspace
 * However, it does register with the TLB, as the SQL uuids and the
 * TLB Handles must be kept in sync, or all hell breaks loose.
 */
LinkPtr AtomStorage::getLink(Type t, const std::vector<Handle>&oset)
{
	setup_typemap();

	char buff[BUFSZ];
	snprintf(buff, BUFSZ, 
	    "SELECT * FROM Atoms WHERE type = %hu AND outgoing = ",
	    storing_typemap[t]);

	std::string ostr = buff;
	ostr += oset_to_string(oset, oset.size());
	ostr += ";";

	AtomPtr atom = getAtom(ostr.c_str(), 1);
	return LinkCast(atom);
}

/** 
 * Instantiate a new atom, from the response buffer contents
 */
AtomPtr AtomStorage::makeAtom(Response &rp, Handle h)
{
	// Now that we know everything about an atom, actually construct one.
	AtomPtr atom(TLB::getAtom(h));
	Type realtype = loading_typemap[rp.itype];

	if (NOTYPE == realtype)
	{
		fprintf(stderr,
			"Fatal Error: OpenCog does not have a type called %s\n",
			db_typename[rp.itype]);
		return NULL;
	}

	if (NULL == atom)
	{
		// All height zero atoms are nodes,
		// All positive height atoms are links.
		// A negative height is "unknown" and must be checked.
		if ((0 == rp.height) || 
		    ((-1 == rp.height) &&
		      classserver().isA(realtype, NODE)))
		{
			atom = AtomPtr(new Node(realtype, rp.name));
		}
		else
		{
			std::vector<Handle> outvec;
#ifndef USE_INLINE_EDGES
			getOutgoing(outvec, h);
#else
			char *p = (char *) rp.outlist;
			while(p)
			{
				if (*p == '}') break;
				Handle hout = (Handle) strtoul(p+1, &p, 10);
				outvec.push_back(hout);
			}
#endif /* USE_INLINE_EDGES */
			atom = AtomPtr(new Link(realtype, outvec));
		}

		// Make sure that the handle in the TLB is synced with
		// the handle we use in the database.
		TLB::addAtom(atom, h);
	}
	else
	{
		// Perform at least some basic sanity checking ...
		if (realtype != atom->getType())
		{
			UUID uuid = h.value();
			fprintf(stderr,
				"Error: mismatched atom type for existing atom! "
				"uuid=%lu real=%d atom=%d\n",
				uuid, realtype, atom->getType());
		}
	}

	// Now get the truth value
	switch (rp.tv_type)
	{
		case SIMPLE_TRUTH_VALUE:
		{
			SimpleTruthValue stv(rp.mean, rp.count);
			atom->setTruthValue(stv);
			break;
		}
		case COUNT_TRUTH_VALUE:
		{
			CountTruthValue ctv(rp.mean, rp.confidence, rp.count);
			atom->setTruthValue(ctv);
			break;
		}
		case INDEFINITE_TRUTH_VALUE:
		{
			IndefiniteTruthValue itv(rp.mean, rp.count, rp.confidence);
			atom->setTruthValue(itv);
			break;
		}
		case COMPOSITE_TRUTH_VALUE:
			fprintf(stderr, "Error: Composite truth values are not handled\n");
			break;
		default:
			fprintf(stderr, "Error: makeAtom: Unknown truth value type\n");
	}

	load_count ++;
	if (load_count%10000 == 0)
	{
		fprintf(stderr, "\tLoaded %lu atoms.\n", load_count);
	}

	local_id_cache.insert(h);
	return atom;
}

/* ================================================================ */

void AtomStorage::load(AtomTable &table)
{
	unsigned long max_nrec = getMaxUUID();
	TLB::reserve_range(0,max_nrec);
	fprintf(stderr, "Max UUID is %lu\n", max_nrec);
	load_count = 0;
	max_height = getMaxHeight();
	fprintf(stderr, "Max Height is %d\n", max_height);

	setup_typemap();

	Response rp;
	rp.table = &table;
	rp.store = this;

	for (int hei=0; hei<=max_height; hei++)
	{
		unsigned long cur = load_count;

#if GET_ONE_BIG_BLOB
		char buff[BUFSZ];
		snprintf(buff, BUFSZ, "SELECT * FROM Atoms WHERE height = %d;", hei);
		rp.height = hei;
		rp.rs = db_conn->exec(buff);
		rp.rs->foreach_row(&Response::load_all_atoms_cb, &rp);
		rp.rs->release();
#else
		// It appears that, when the select statment returns more than
		// about a 100K to a million atoms or so, some sort of heap
		// corruption occurs in the iodbc code, causing future mallocs
		// to fail. So limit the number of records processed in one go.
		// It also appears that asking for lots of records increases
		// the memory fragmentation (and/or there's a memory leak in iodbc??)
		// XXX Not clear is UnixODBC suffers from this same problem.
#define STEP 12003
		unsigned long rec;
		for (rec = 0; rec <= max_nrec; rec += STEP)
		{
			char buff[BUFSZ];
			snprintf(buff, BUFSZ, "SELECT * FROM Atoms WHERE "
			        "height = %d AND uuid > %lu AND uuid <= %lu;",
			         hei, rec, rec+STEP);
			rp.height = hei;
			rp.rs = db_conn->exec(buff);
			rp.rs->foreach_row(&Response::load_all_atoms_cb, &rp);
			rp.rs->release();
		}
#endif
		fprintf(stderr, "Loaded %lu atoms at height %d\n", load_count - cur, hei);
	}
	fprintf(stderr, "Finished loading %lu atoms in total\n", load_count);
}

bool AtomStorage::store_cb(AtomPtr atom)
{
	storeSingleAtom(atom);
	store_count ++;
	if (store_count%1000 == 0)
	{
		fprintf(stderr, "\tStored %lu atoms.\n", store_count);
	}
	return false;
}

void AtomStorage::store(const AtomTable &table)
{
	max_height = 0;
	store_count = 0;

#ifdef ALTER
	rename_tables();
	create_tables();
#endif

	get_ids();
	UUID max_uuid = TLB::getMaxUUID();
	setMaxUUID(max_uuid);
	fprintf(stderr, "Max UUID is %lu\n", max_uuid);

	setup_typemap();

	Response rp;

#ifndef USE_INLINE_EDGES
	// Drop indexes, for faster loading.
	// But this only matters for the non-inline eges...
	rp.rs = db_conn->exec("DROP INDEX uuid_idx;");
	rp.rs->release();
	rp.rs = db_conn->exec("DROP INDEX src_idx;");
	rp.rs->release();
#endif

	table.foreachHandleByType(
       [&](Handle h)->void { store_cb(table.getAtom(h)); }, ATOM, true);

#ifndef USE_INLINE_EDGES
	// Create indexes
	rp.rs = db_conn->exec("CREATE INDEX uuid_idx ON Atoms (uuid);");
	rp.rs->release();
	rp.rs = db_conn->exec("CREATE INDEX src_idx ON Edges (src_uuid);");
	rp.rs->release();
#endif /* USE_INLINE_EDGES */

	rp.rs = db_conn->exec("VACUUM ANALYZE;");
	rp.rs->release();

	setMaxHeight(getMaxObservedHeight());
	fprintf(stderr, "\tFinished storing %lu atoms total.\n", store_count);

	// Now that we're done storing, reserve a more conservative
	// UUID value, based on what's actually in the database.
	max_uuid = getMaxObservedUUID();
	setMaxUUID(max_uuid);
	fprintf(stderr, "Set Max observed UUID to %lu\n", max_uuid);
}

/* ================================================================ */

void AtomStorage::rename_tables(void)
{
	Response rp;

	rp.rs = db_conn->exec("ALTER TABLE Atoms RENAME TO Atoms_Backup;");
	rp.rs->release();
#ifndef USE_INLINE_EDGES
	rp.rs = db_conn->exec("ALTER TABLE Edges RENAME TO Edges_Backup;");
	rp.rs->release();
#endif /* USE_INLINE_EDGES */
	rp.rs = db_conn->exec("ALTER TABLE Global RENAME TO Global_Backup;");
	rp.rs->release();
	rp.rs = db_conn->exec("ALTER TABLE TypeCodes RENAME TO TypeCodes_Backup;");
	rp.rs->release();
}

void AtomStorage::create_tables(void)
{
	Response rp;

	// See the file "atom.sql" for detailed documentation as to the 
	// structure of teh SQL tables.
	rp.rs = db_conn->exec("CREATE TABLE Atoms ("
	                      "uuid	INT PRIMARY KEY,"
	                      "type  SMALLINT,"
	                      "stv_mean FLOAT,"
	                      "stv_count FLOAT,"
	                      "height INT,"
	                      "name    TEXT,"
	                      "outgoing INT[]);");
	rp.rs->release();

#ifndef USE_INLINE_EDGES
	rp.rs = db_conn->exec("CREATE TABLE Edges ("
	                      "src_uuid  INT,"
	                      "dst_uuid  INT,"
	                      "pos INT);");
	rp.rs->release();
#endif /* USE_INLINE_EDGES */

	rp.rs = db_conn->exec("CREATE TABLE TypeCodes ("
	                      "type SMALLINT UNIQUE,"
	                      "typename TEXT UNIQUE);");
	rp.rs->release();
	type_map_was_loaded = false;

	rp.rs = db_conn->exec("CREATE TABLE Global ("
	                      "max_uuid INT,"
	                      "max_height INT);");
	rp.rs->release();
}

/**
 * kill_data -- destroy data in the database!! Dangerous !!
 * This routine is meant to be used only for running test cases.
 * It is extremely dangerous, as it can lead to total data loss.
 */
void AtomStorage::kill_data(void)
{
	Response rp;

	// See the file "atom.sql" for detailed documentation as to the 
	// structure of teh SQL tables.
	rp.rs = db_conn->exec("DELETE from Atoms;");
	rp.rs->release();

	rp.rs = db_conn->exec("UPDATE Global SET max_uuid = 500;");
	rp.rs->release();
	rp.rs = db_conn->exec("UPDATE Global SET max_height = 0;");
	rp.rs->release();
}

/* ================================================================ */
/*
 * XXX the table Global is a cache of values that can be obtained more
 * directly from the "observed" getters. I suspect that this table is 
 * not really needed; it just adds complexity to the code, and should
 * probably be eliminated.
 */

UUID AtomStorage::getMaxUUID(void)
{
	Response rp;
	rp.rs = db_conn->exec("SELECT max_uuid FROM Global;");
	rp.rs->foreach_row(&Response::intval_cb, &rp);
	rp.rs->release();
	return rp.intval;
}

void AtomStorage::setMaxUUID(UUID uuid)
{
	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "UPDATE Global SET max_uuid = %lu;", uuid);

	Response rp;
	rp.rs = db_conn->exec(buff);
	rp.rs->release();
}

void AtomStorage::setMaxHeight(int sqmax)
{
	// Max height of db contents can only get larger! 
	if (max_height < sqmax) max_height = sqmax;

	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "UPDATE Global SET max_height = %d;", max_height);

	Response rp;
	rp.rs = db_conn->exec(buff);
	rp.rs->release();
}

int AtomStorage::getMaxHeight(void)
{
	Response rp;
	rp.rs = db_conn->exec("SELECT max_height FROM Global;");
	rp.rs->foreach_row(&Response::intval_cb, &rp);
	rp.rs->release();
	return rp.intval;
}

UUID AtomStorage::getMaxObservedUUID(void)
{
	Response rp;
	rp.intval = 0;
	rp.rs = db_conn->exec("SELECT uuid FROM Atoms ORDER BY uuid DESC LIMIT 1;");
	rp.rs->foreach_row(&Response::intval_cb, &rp);
	rp.rs->release();
	return rp.intval;
}

int AtomStorage::getMaxObservedHeight(void)
{
	Response rp;
	rp.intval = 0;
	rp.rs = db_conn->exec("SELECT height FROM Atoms ORDER BY height DESC LIMIT 1;");
	rp.rs->foreach_row(&Response::intval_cb, &rp);
	rp.rs->release();
	return rp.intval;
}

void AtomStorage::reserve(void)
{
	UUID max_observed_id = getMaxObservedUUID();
	fprintf(stderr, "Reserving UUID up to %lu\n", max_observed_id);
	TLB::reserve_range(0, max_observed_id);
}

#endif /* HAVE_SQL_STORAGE */
/* ============================= END OF FILE ================= */
