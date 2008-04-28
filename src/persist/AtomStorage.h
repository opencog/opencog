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

#include <map>
#include <vector>

#include "Atom.h"

class AtomStorage
{
	private:
		ODBCConnection *db_conn;

		// Utility for handling responses on stack.
		class Response;
		class Outgoing;

		// Set of atoms that we know about.
		std::map<Handle,Atom *> handle_map;

		void storeOutgoing(Atom *, Handle);
		void getOutgoing(std::vector<Handle> &, Handle);

		bool idExists(const char *);

		unsigned long getMaxUUID(void);
		void setMaxUUID(unsigned long);

#ifdef OUT_OF_LINE_TVS
		bool tvExists(int);
		int storeTruthValue(Atom *, Handle);
		int  TVID(const TruthValue &);
		TruthValue * getTV(int);
#endif /* OUT_OF_LINE_TVS */

	public:
		AtomStorage(const char * dbname, 
		            const char * username,
		            const char * authentication);
		~AtomStorage();

		void storeAtom(Atom *);
		bool atomExists(Handle);
		Atom * getAtom(Handle);
};

#endif /* PERSITENT_ATOM_STORAGE_H_ */

