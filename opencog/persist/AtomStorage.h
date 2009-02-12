/*
 * FUNCTION:
 * Base class for SQL-backed persistent storage.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_PERSITENT_ATOM_STORAGE_H
#define _OPENCOG_PERSITENT_ATOM_STORAGE_H

#include <set>
#include <vector>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/persist/odbcxx.h>

namespace opencog
{

class AtomStorage
{
	private:
		ODBCConnection *db_conn;

		// Utility for handling responses on stack.
		class Response;
		class Outgoing;

		Atom * makeAtom (Response &, Handle);

		int height(const Atom *);
		int max_height;
		void setMaxHeight(void);
		int getMaxHeight(void);

		void storeOutgoing(Atom *, Handle);
		void getOutgoing(std::vector<Handle> &, Handle);
		bool store_cb(const Atom *);
		unsigned long load_count;
		unsigned long store_count;

		void rename_tables(void);
		void create_tables(void);

		std::set<Handle> local_id_cache;
		void get_ids(void);
		bool idExists(const char *);

		unsigned long getMaxUUID(void);
		void setMaxUUID(unsigned long);

		// Reserve extra space for future growth -- 1200 should be enough.
		#define TYPEMAP_SZ (NUMBER_OF_CLASSES + 1200)
		int  storing_typemap[TYPEMAP_SZ];
		Type loading_typemap[TYPEMAP_SZ];
		char * db_typename[TYPEMAP_SZ];

		bool type_map_was_loaded;
		void load_typemap(void);
		void setup_typemap(void);
		void set_typemap(int, const char *);

#ifdef OUT_OF_LINE_TVS
		bool tvExists(int);
		int storeTruthValue(Atom *, Handle);
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
		bool connected(void);

		void kill_data(void);
		void storeAtom(const Atom *);
		bool atomExists(Handle);
		Atom * getAtom(Handle);
		Node * getNode(Type t, const char * str);
		Node * getNode(const Node &n)
		{
			return getNode(n.getType(), n.getName().c_str());
		}
		Link * getLink(const Link &);

		void load(AtomTable &);
		void store(const AtomTable &);
};

} // namespace opencog

#endif // _OPENCOG_PERSITENT_ATOM_STORAGE_H
