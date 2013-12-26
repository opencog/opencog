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

#include <atomic>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include <opencog/util/concurrent_queue.h>
#include <opencog/util/concurrent_stack.h>
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
		// Pool of shared connections
		ODBCConnection* get_conn();
		void put_conn(ODBCConnection*);
		concurrent_stack<ODBCConnection*> conn_pool;
		std::mutex conn_mutex;

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
		std::atomic<unsigned long> load_count;
		std::atomic<unsigned long> store_count;

		void rename_tables(void);
		void create_tables(void);

		// Track UUID's that are in use.
		std::mutex id_cache_mutex;
		bool local_id_cache_is_inited;
		std::set<UUID> local_id_cache;
		void add_id_to_cache(UUID);
		void get_ids(void);

		std::mutex id_create_mutex;
		std::set<UUID> id_create_cache;
		std::unique_lock<std::mutex> maybe_create_id(UUID);

		UUID getMaxObservedUUID(void);
		int getMaxObservedHeight(void);
		bool idExists(const char *);

		// The typemap translates between opencog type numbers and
		// the database type numbers.  Initially, they match up, but
		// might get askew if new types are added or deleted.

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

		// Stuff to support asynchronous store of atoms.
		concurrent_queue<AtomPtr> store_queue;
		std::vector<std::thread> write_threads;
		std::mutex write_mutex;
		unsigned int thread_count;
		std::atomic<unsigned long> busy_writers;
		void startWriterThread();
		void stopWriterThreads();
		bool stopping_writers;
		void writeLoop();

	public:
		AtomStorage(const std::string& dbname, 
		            const std::string& username,
		            const std::string& authentication);
		AtomStorage(const char * dbname, 
		            const char * username,
		            const char * authentication);
		AtomStorage(const AtomStorage&) = delete; // disable copying
		AtomStorage& operator=(const AtomStorage&) = delete; // disable assignment
		~AtomStorage();
		bool connected(void); // connection to DB is alive

		void kill_data(void); // destroy DB contents

		// Store atoms to DB
		void storeSingleAtom(AtomPtr);
		void storeAtom(AtomPtr, bool synchronous = false);
		void flushStoreQueue();

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

		// Large-scale loads and saves
		void loadType(AtomTable &, Type); // Load *all* atoms of type
		void load(AtomTable &); // Load entire contents of DB
		void store(const AtomTable &); // Store entire contents of AtomTable
		void reserve(void);     // reserve range of UUID's
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PERSITENT_ATOM_STORAGE_H
