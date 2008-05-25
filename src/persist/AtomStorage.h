/*
 * FUNCTION:
 * Base class for SQL-backed persistent storage.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef PERSITENT_ATOM_STORAGE_H_
#define PERSITENT_ATOM_STORAGE_H_

#include "odbcxx.h"

#include <set>
#include <vector>

#include "Atom.h"
#include "AtomTable.h"

class AtomStorage
{
	private:
		ODBCConnection *db_conn;

		// Utility for handling responses on stack.
		class Response;
		class Outgoing;

		Atom * makeAtom (Response &, Handle h);

		void storeOutgoing(Atom *, Handle);
		void getOutgoing(std::vector<Handle> &, Handle);
		bool store_cb(Atom *);
		unsigned long load_count;
		unsigned long store_count;

		std::set<unsigned long> local_id_cache;
		void get_ids(void);
		bool idExists(const char *);

		unsigned long getMaxUUID(void);
		void setMaxUUID(unsigned long);

		// XXX assume there are fewer than 500 atom types.
		int  storing_typemap[500];
		Type loading_typemap[500];
		bool type_map_was_loaded;
		void load_typemap(void);
		void store_typemap(void);
		void set_typemap(int, const char *);

#ifdef OUT_OF_LINE_TVS
		bool tvExists(int);
		int storeTruthValue(Atom *, Handle);
		int  TVID(const TruthValue &);
		TruthValue * getTV(int);
#endif /* OUT_OF_LINE_TVS */

	public:
		AtomStorage(const std::string dbname, 
		            const std::string username,
		            const std::string authentication);
		AtomStorage(const char * dbname, 
		            const char * username,
		            const char * authentication);
		~AtomStorage();

		void storeAtom(Atom *);
		bool atomExists(Handle);
		Atom * getAtom(Handle);

		void load(AtomTable &);
		void store(const AtomTable &);
};

#endif /* PERSITENT_ATOM_STORAGE_H_ */

