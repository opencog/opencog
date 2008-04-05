/*
 * FUNCTION:
 * sniff test.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "odbcxx.h"

#include "Atom.h"

class AtomStorage
{
	private:
		ODBCConnection *db_conn;
		ODBCRecordSet *rs;
		bool column_cb(const char *colname, const char * colvalue)
		{
			printf ("%s = %s\n", colname, colvalue);
			return false;
		}
		bool row_cb(void)
		{
			printf ("---- New row found ----\n");
			rs->foreach_column(&AtomStorage::column_cb, this);
			return false;
		}

		bool row_exists;
		bool row_exists_cb(void)
		{
			row_exists = true;
			return false;
		}

	public:
		AtomStorage(void);
		~AtomStorage();

		void storeAtom(Atom *);
		bool atomExists(Handle);
		Atom * getAtom(Handle);
};

#include "Node.h"
#include "TLB.h"
#include "type_codes.h"

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
printf ("duude atom exists\n");
		cols = "UPDATE Atoms SET ";
		coda = "WHERE uuid = ";
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
	db_conn->exec(qry.c_str());
}

/**
 * Return true if the indicated handle esists in the storage.
 */
bool AtomStorage::atomExists(Handle h)
{
	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "%lu", (unsigned long) h);

	std::string select = "SELECT uuid FROM Atoms WHERE uuid = ";
	select += buff;
	select += ";"; 
	rs = db_conn->exec(select.c_str());

	row_exists = false;
	rs->foreach_row(&AtomStorage::row_exists_cb, this);
	rs->release();
	return row_exists;
}

Atom * AtomStorage::getAtom(Handle h)
{
	char buff[BUFSZ];
	snprintf(buff, BUFSZ, "%lu", (unsigned long) h);

	std::string select = "SELECT * FROM Atoms WHERE uuid = ";
	select += buff;
	select += ";"; 
	rs = db_conn->exec(select.c_str());
	rs->foreach_row(&AtomStorage::row_cb, this);
	rs->release();
	return NULL;
}

int main ()
{
	AtomStorage *store = new AtomStorage();

	Atom *a = new Node(SCHEMA_NODE, "someNode");
printf ("hello\n");

	store->storeAtom(a);

	Handle h = TLB::getHandle(a);
	store->getAtom(h);


	return 0;
}

/* ============================= END OF FILE ================= */
