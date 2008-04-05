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
	public:
		AtomStorage(void);
		~AtomStorage();

		void storeAtom(Atom *);
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

void AtomStorage::storeAtom(Atom *atom)
{
	std::string cols = "INSERT INTO Atoms (";
	std::string vals = ") VALUES (";

	char buff[24];
	Handle h = TLB::getHandle(atom);
	snprintf(buff, 24, "%lu", (unsigned long) h);
	cols += "uuid"; 
	vals += buff;

	Type t = atom->getType();
	snprintf(buff, 24, "%u", t);
	cols += ", type"; 
	vals += ", "; 
	vals += buff;

	Node *n = dynamic_cast<Node *>(atom);
	if (n)
	{
		cols += ", name";
		vals += ", '";
		vals += n->getName();
		vals += "'";
	}

	std::string qry = cols + vals + ");";
printf ("duude its %s\n", qry.c_str());
	db_conn->exec(qry.c_str());
}

Atom * AtomStorage::getAtom(Handle h)
{
	rs = db_conn->exec("SELECT * FROM Atoms;");
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
	store->getAtom(a);


	return 0;
}

/* ============================= END OF FILE ================= */
