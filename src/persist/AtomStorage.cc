/*
 * FUNCTION:
 * Persistent Atom storage, SQL-backed.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "odbcxx.h"
#include "Atom.h"
#include "AtomStorage.h"
#include "Node.h"
#include "TLB.h"
#include "type_codes.h"



class AtomStorage::Response
{
	public:
		ODBCRecordSet *rs;
		bool create_atom_column_cb(const char *colname, const char * colvalue)
		{
			printf ("%s = %s\n", colname, colvalue);
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

void AtomStorage::storeAtom(Atom *atom)
{
	int notfirst = 0;
	std::string cols;
	std::string vals;
	std::string coda;

	Handle h = TLB::getHandle(atom);

	// Currently using the TLB Handle as the UUID, ...XXX
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
	rp.rs->release();
	return NULL;
}

/* ============================= END OF FILE ================= */
