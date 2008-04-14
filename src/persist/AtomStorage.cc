/*
 * FUNCTION:
 * Persistent Atom storage, SQL-backed.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "odbcxx.h"
#include "Atom.h"
#include "ClassServer.h"
#include "Node.h"
#include "TLB.h"
#include "type_codes.h"

#include "AtomStorage.h"

class AtomStorage::Response
{
	public:
		ODBCRecordSet *rs;

		// Temporary cache of info about atom being assembled.
		Type itype;
		const char * name;

		bool create_atom_column_cb(const char *colname, const char * colvalue)
		{
			printf ("%s = %s\n", colname, colvalue);
			if (!strcmp(colname, "type"))
			{
				itype = atoi(colvalue);
			}
			else if (!strcmp(colname, "name"))
			{
				name = colvalue;
			}
			return false;
		}
		bool create_atom_row_cb(void)
		{
			printf ("---- New row found ----\n");
			rs->foreach_column(&Response::create_atom_column_cb, this);

			return false;
		}

		bool row_exists;
		bool row_exists_cb(void)
		{
			row_exists = true;
			return false;
		}
};

AtomStorage::AtomStorage(void)
{
	db_conn = new ODBCConnection("opencog", "linas", NULL);
}

AtomStorage::~AtomStorage()
{
	delete db_conn;
}

#define BUFSZ 34

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

/**
 * Store the outgoing set of the atom.
 * Handle h must be the handle for the atom; its passed as an arg to 
 * avoid having to look it up.
 */
void AtomStorage::storeOutgoing(Atom *atom, Handle h)
{
	// foreach_outgoing_handle(h, cb, T*);
}

void AtomStorage::storeAtom(Atom *atom)
{
	int notfirst = 0;
	std::string cols;
	std::string vals;
	std::string coda;

	Handle h = TLB::getHandle(atom);

	// Use the TLB Handle as the UUID.
	char uuidbuff[BUFSZ];
	snprintf(uuidbuff, BUFSZ, "%lu", (unsigned long) h);

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
	}

	STMT("uuid", uuidbuff);

	char buff[BUFSZ];
	Type t = atom->getType();
	snprintf(buff, BUFSZ, "%u", t);
	STMT("type", buff);

	Node *n = dynamic_cast<Node *>(atom);
	if (n)
	{
		std::string qname = "'";
		qname += n->getName();
		qname += "'";
		STMT("name", qname);
	}

	std::string qry = cols + vals + coda;
printf ("duude its %s\n", qry.c_str());
	Response rp;
	rp.rs = db_conn->exec(qry.c_str());
	rp.rs->release();

	storeOutgoing(atom, h);
}

/**
 * Return true if the indicated handle exists in the storage.
 */
bool AtomStorage::atomExists(Handle h)
{
	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "%lu", (unsigned long) h);

	std::string select = "SELECT uuid FROM Atoms WHERE uuid = ";
	select += buff;
	select += ";"; 

	Response rp;
	rp.row_exists = false;
	rp.rs = db_conn->exec(select.c_str());
	rp.rs->foreach_row(&Response::row_exists_cb, &rp);
	rp.rs->release();
	return rp.row_exists;
}

/**
 * Create a new atom, retreived from storage
 *
 * This method does *not* register the atom with any atomtable/atomspace
 */
Atom * AtomStorage::getAtom(Handle h)
{
	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "%lu", (unsigned long) h);

	std::string select = "SELECT * FROM Atoms WHERE uuid = ";
	select += buff;
	select += ";"; 

	Response rp;
	rp.rs = db_conn->exec(select.c_str());
	rp.rs->foreach_row(&Response::create_atom_row_cb, &rp);

	// Now that we know everything about an atom, actually construct one.
	Atom *atom = NULL;
	if (ClassServer::isAssignableFrom(NODE, rp.itype))
	{
		atom = new Node (rp.itype, rp.name);
	}
	else
	{
		// atom = new Link(itype, xxx);
	}

	rp.rs->release();
	return atom;
}

/* ============================= END OF FILE ================= */
