/*
 * FUNCTION:
 * sniff test.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "odbcxx.h"

class Ola
{
	public:
		ODBCRecordSet *rs;
		bool column_cb(const char *colname, const char * colvalue)
		{
			printf ("%s = %s\n", colname, colvalue);
			return false;
		}
		bool row_cb(void)
		{
			printf ("---- New row found ----\n");
			rs->foreach_column(&Ola::column_cb, this);
			return false;
		}
};

int main ()
{
	ODBCConnection *conn;
	conn = new ODBCConnection("opencog", "linas", NULL);

	ODBCRecordSet *rs;

	// A third way of doing things
	Ola *ola = new Ola();
	rs = conn->exec("SELECT * FROM Atoms;");
	ola->rs = rs;
	rs->foreach_row(&Ola::row_cb, ola);
	rs->release();

	delete conn;

	return 0;
}

/* ============================= END OF FILE ================= */
