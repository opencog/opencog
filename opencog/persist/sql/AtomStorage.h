/*
 * FUNCTION:
 * Base class for SQL-backed persistent storage.
 *
 * HISTORY:
 * Copyright (c) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_PERSITENT_ATOM_STORAGE_H
#define _OPENCOG_PERSITENT_ATOM_STORAGE_H

#include <set>
#include <vector>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/types.h>
#include <opencog/persist/sql/odbcxx.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class AtomStorage
{
	private:
		ODBCConnection *db_conn;

		// Utility for handling responses on stack.
		class Response;
		class Outgoing;

		void init(const char *, const char *, const char *);
		AtomPtr makeAtom (Response &, Handle);
		AtomPtr getAtom (const char *, int);

		int get_height(AtomPtr);
		int max_height;
		void setMaxHeight(int);
		int getMaxHeight(void);

		int do_store_atom(AtomPtr);
		void do_store_single_atom(AtomPtr, int);

		std::string oset_to_string(const HandleSeq&, int);
		void storeOutgoing(AtomPtr, Handle);
		void getOutgoing(HandleSeq&, Handle);
		bool store_cb(AtomPtr);
		unsigned long load_count;
		unsigned long store_count;

		void rename_tables(void);
		void create_tables(void);

		bool local_id_cache_is_inited;
		std::set<Handle> local_id_cache;
		void get_ids(void);
		UUID getMaxObservedUUID(void);
		int getMaxObservedHeight(void);
		bool idExists(const char *);

		UUID getMaxUUID(void);
		void setMaxUUID(UUID);

		// TYPEMAP_SZ is defined as the maximum number of possible opencog Types
		// (65536 as Type is a short int)
		#define TYPEMAP_SZ (1 << (8 * sizeof(Type)))
		int  storing_typemap[TYPEMAP_SZ];
		Type loading_typemap[TYPEMAP_SZ];
		char * db_typename[TYPEMAP_SZ];

		bool type_map_was_loaded;
		void load_typemap(void);
		void setup_typemap(void);
		void set_typemap(int, const char *);

#ifdef OUT_OF_LINE_TVS
		bool tvExists(int);
		int storeTruthValue(AtomPtr, Handle);
		int  TVID(const TruthValue &);
		TruthValue * getTV(int);
#endif /* OUT_OF_LINE_TVS */

	public:
		AtomStorage(const std::string& dbname, 
		            const std::string& username,
		            const std::string& authentication);
		AtomStorage(const char * dbname, 
		            const char * username,
		            const char * authentication);
		~AtomStorage();
		bool connected(void); // connection to DB is alive

		void kill_data(void); // destroy DB contents

		// Store atoms to DB
		void storeSingleAtom(AtomPtr);
		void storeAtom(Handle);
		void storeAtom(AtomPtr);

		// Fetch atoms from DB
		bool atomExists(Handle);
		AtomPtr getAtom(Handle);
		std::vector<Handle> getIncomingSet(Handle);
		NodePtr getNode(Type, const char *);
		NodePtr getNode(const Node &n)
		{
			return getNode(n.getType(), n.getName().c_str());
		}
		LinkPtr getLink(Type, const std::vector<Handle>&);
		LinkPtr getLink(const Link &l)
		{
			return getLink(l.getType(), l.getOutgoingSet());
		}

		void load(AtomTable &); // Load entire contents of DB
		void store(const AtomTable &); // Store entire contents of AtomTable
		void reserve(void);     // reserve range of UUID's
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PERSITENT_ATOM_STORAGE_H
