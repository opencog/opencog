/*
 * FUNCTION:
 * ODBC driver -- developed/tested with iODBC http://www.iodbc.org
 *
 * ODBC is basically brain-damaged, and so is this driver.
 * The problem is that ODBC forces you to guess how many columns there
 * are in your reply, and etc. which we don't know a-priori.  Also
 * makes VARCHAR difficult (impossible ??) to support correctly!
 * Blame it on SQLBindCol(), which is a terrible idea.  @#$%^ Microsoft.
 *
 * Threading:
 * ----------
 * This class is thread-enabled but not thread-safe. Two threads should
 * not try to use one instance of this class at the same time. Each
 * thread should construct it's own instance of this class. This class
 * uses no globals.
 *
 * HISTORY:
 * Copyright (c) 2002,2008 Linas Vepstas <linas@linas.org>
 * created by Linas Vepstas  March 2002
 * ported to C++ March 2008
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

#ifdef HAVE_SQL_STORAGE

#include "odbcxx.h"

#include <stack>
#include <string>

#include <sql.h>
#include <sqlext.h>
#include <stdio.h>

#include <opencog/util/platform.h>

// cheesy hack for missing PERR
#define PERR printf

/* =========================================================== */

#define PRINT_SQLERR(HTYPE,HAN)                                \
{                                                              \
	char sql_stat[10];                                          \
	SQLSMALLINT msglen;                                         \
	SQLINTEGER err;                                             \
	char msg[200];                                              \
	                                                            \
	SQLGetDiagRec(HTYPE, HAN, 1, (SQLCHAR *) sql_stat,          \
	              &err, (SQLCHAR*) msg, sizeof(msg), &msglen);  \
	PERR("(%ld) %s\n", (long int) err, msg);                    \
}

/* =========================================================== */

ODBCConnection::ODBCConnection(const char * _dbname,
                               const char * _username,
                               const char * _authentication)
{
	SQLRETURN rc;

	is_connected = false;

	sql_hdbc = NULL;
	sql_henv = NULL;

	if (NULL == _dbname)
	{
		PERR("No DB specified");
		return;
	}

	/* Allocate environment handle */
	rc = SQLAllocEnv(&sql_henv);
	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR("Can't SQLAllocEnv, rc=%d", rc);
		return;
	}

	/* set the ODBC version */
	rc = SQLSetEnvAttr(sql_henv, SQL_ATTR_ODBC_VERSION,
	                   (void*)SQL_OV_ODBC3, 0);
	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR("Can't SQLSetEnv, rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_ENV, sql_henv);
		SQLFreeHandle(SQL_HANDLE_ENV, sql_henv);
		sql_henv = NULL;
		return;
	}

	/* allocate the connection handle */
	rc = SQLAllocConnect(sql_henv, &sql_hdbc);
	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't SQLAllocConnect handle rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_ENV, sql_henv);
		SQLFreeHandle(SQL_HANDLE_ENV, sql_henv);
		sql_henv = NULL;
		return;
	}

	/* set the timeout to 5 seconds ?? hack alert fixme */
	// SQLSetConnectAttr(sql_hdbc, SQL_LOGIN_TIMEOUT, (SQLPOINTER *)5, 0);

	if (NULL == _authentication) _authentication = "";
	rc = SQLConnect(sql_hdbc,
	                (SQLCHAR*) _dbname, SQL_NTS,
	                (SQLCHAR*) _username, SQL_NTS,
	                (SQLCHAR*) _authentication, SQL_NTS);

	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't perform SQLConnect rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_DBC, sql_hdbc);
		SQLFreeHandle(SQL_HANDLE_DBC, sql_hdbc);
		SQLFreeHandle(SQL_HANDLE_ENV, sql_henv);
		sql_henv = NULL;
		sql_hdbc = NULL;
		return;
	}

	dbname = _dbname;
	username = _username;
	is_connected = true;
}

/* =========================================================== */

ODBCConnection::~ODBCConnection()
{
	if (sql_hdbc)
	{
		SQLDisconnect(sql_hdbc);
		SQLFreeHandle(SQL_HANDLE_DBC, sql_hdbc);
		sql_hdbc = NULL;
	}

	if (sql_henv)
	{
		SQLFreeHandle(SQL_HANDLE_ENV, sql_henv);
		sql_henv = NULL;
	}

	while (!free_pool.empty())
	{
		ODBCRecordSet *rs = free_pool.top();
		delete rs;
		free_pool.pop();
	}
}

bool ODBCConnection::connected (void) const
{
	return is_connected;
}

/* =========================================================== */
#define DEFAULT_NUM_COLS 50

ODBCRecordSet * ODBCConnection::get_record_set(void)
{
	ODBCRecordSet *rs;
	if (!free_pool.empty())
	{
		rs = free_pool.top();
		free_pool.pop();
		rs->ncols = -1;
	}
	else
	{
		rs = new ODBCRecordSet(this);
	}

	rs->alloc_and_bind_cols(DEFAULT_NUM_COLS);

	return rs;
}

/* =========================================================== */

ODBCRecordSet *
ODBCConnection::exec(const char * buff)
{
	ODBCRecordSet *rs;
	SQLRETURN rc;

	if (!is_connected) return NULL;

	rs = get_record_set();
	if (!rs) return NULL;

	rc = SQLExecDirect(rs->sql_hstmt, (SQLCHAR *)buff, SQL_NTS);

	/* If query returned no data, its not necessarily an error:
	 * its simply "no data", that's all.
	 */
	if (SQL_NO_DATA == rc)
	{
		rs->release();
		return NULL;
	}

	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't perform query rc=%d ", rc);
		PRINT_SQLERR (SQL_HANDLE_STMT, rs->sql_hstmt);
		rs->release();
		PERR ("\tQuery was: %s\n", buff);
		return NULL;
	}

	/* Use numbr of columns to indicate that the query hasn't
	 * given results yet. */
	rs->ncols = -1;
	return rs;
}

/* =========================================================== */

#define DEFAULT_COLUMN_NAME_SIZE 121
#define DEFAULT_VARCHAR_SIZE 4040

void
ODBCRecordSet::alloc_and_bind_cols(int new_ncols)
{
	// IMPORTANT! MUST NOT BE ON STACK!! Else stack corruption will result.
	// The ODBC driver really wants to write a return value here!
	static SQLLEN bogus;

	SQLRETURN rc;
	int i;

	if (new_ncols > arrsize)
	{
		if (column_labels)
		{
			for (i=0; i<arrsize; i++)
			{
				if (column_labels[i])
				{
					delete[] column_labels[i];
				}
			}
			delete[] column_labels;
		}
		if (column_datatype) delete[] column_datatype;

		if (values)
		{
			for (i=0; i<arrsize; i++)
			{
				if (values[i])
				{
					delete[] values[i];
				}
			}
			delete[] values;
		}
		if (vsizes) delete[] vsizes;

		column_labels = new char*[new_ncols];
		column_datatype = new int[new_ncols];
		values = new char*[new_ncols];
		vsizes = new int[new_ncols];

		/* intialize */
		for (i = 0; i<new_ncols; i++)
		{
			column_labels[i] = NULL;
			column_datatype[i] = 0;
			values[i] = NULL;
			vsizes[i] = 0;
		}

		arrsize = new_ncols;
	}

	rc = SQLAllocStmt (conn->sql_hdbc, &sql_hstmt);
	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR("Can't allocate statement handle, rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_STMT, sql_hstmt);
		/* oops memory leak */
		return;
	}

	/* Initialize the newly realloc'ed entries */
	for (i=0; i<new_ncols; i++)
	{
		column_datatype[i] = 0;

		if (NULL == column_labels[i])
		{
			column_labels[i] = new char[DEFAULT_COLUMN_NAME_SIZE];
			column_labels[i][0] = 0;
		}
		if (NULL == values[i])
		{
			values[i] = new char[DEFAULT_VARCHAR_SIZE];
			vsizes[i] = DEFAULT_VARCHAR_SIZE;
			values[i][0] = 0;
		}
		rc = SQLBindCol(sql_hstmt, i+1, SQL_C_CHAR,
			values[i], vsizes[i], &bogus);
		if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
		{
			PERR ("Can't bind col=%d rc=%d", i, rc);
			PRINT_SQLERR (SQL_HANDLE_STMT, sql_hstmt);
			return;
		}
	}
}

/* =========================================================== */
/* pseudo-private routine */


ODBCRecordSet::ODBCRecordSet(ODBCConnection *_conn)
{
	// If _conn is null, then this is null, too.
	if (NULL == _conn) return;

	conn = _conn;
	ncols = -1;
	arrsize = 0;
	column_labels = NULL;
	column_datatype = NULL;
	values = NULL;
	vsizes = NULL;
	sql_hstmt = NULL;
}

/* =========================================================== */

void
ODBCRecordSet::release(void)
{
	// 'this' is typically null when an error occurred, or if no data
	// was returned. We don't want a call to 'release()' to crash for
	// these cases ... we want to just keep going like normal.
	if (!this) return;

	// Avoid accidental double-release
	if (NULL == sql_hstmt) return;

	// SQLFreeStmt(sql_hstmt, SQL_UNBIND);
	// SQLFreeStmt(sql_hstmt, SQL_CLOSE);
	SQLFreeHandle(SQL_HANDLE_STMT, sql_hstmt);
	sql_hstmt = NULL;

	conn->free_pool.push(this);
}

/* =========================================================== */

ODBCRecordSet::~ODBCRecordSet()
{
	release();  // shouldn't be needed ... but just in case.

	conn = NULL;

	for (int i=0; i<arrsize; i++)
	{
		delete[] column_labels[i];
		delete[] values[i];
	}
	delete[] column_labels;
	column_labels = NULL;

	delete[] column_datatype;
	column_datatype = NULL;

	delete[] values;
	values = NULL;

	delete[] vsizes;
	vsizes = NULL;
}

/* =========================================================== */

void
ODBCRecordSet::get_column_labels(void)
{
	SQLSMALLINT _ncols;
	SQLRETURN rc;
	int i;

	if (0 <= ncols) return;

	/* If number of columns is negative, then we haven't
	 * gotten any results back yet.  Start by getting the
	 * column labels.
	 */

	rc = SQLNumResultCols(sql_hstmt, &_ncols);
	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't get num columns rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_STMT, sql_hstmt);
		return;
	}

	if (_ncols > arrsize)
	{
		PERR( "screwed not enough columns !! ");
		_ncols = arrsize;
	}

	for (i=0; i<_ncols; i++)
	{
		char namebuff[300];
		SQLSMALLINT namelen;
		SQLULEN column_size;
		SQLSMALLINT datatype;
		SQLSMALLINT decimal_digits;
		SQLSMALLINT nullable;

		rc = SQLDescribeCol (sql_hstmt, i+1,
		          (SQLCHAR *) namebuff, 299, &namelen,
		          &datatype, &column_size, &decimal_digits, &nullable);
		if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
		{
			PERR ("Can't describe col rc=%d", rc);
			PRINT_SQLERR (SQL_HANDLE_STMT, sql_hstmt);
			return;
		}

		namebuff[namelen] = 0x0;
		// PINFO ("column %d has name\'%s\'", i, namebuff);

		strncpy(column_labels[i], namebuff, DEFAULT_COLUMN_NAME_SIZE);
		column_labels[i][DEFAULT_COLUMN_NAME_SIZE-1] = 0;
		column_datatype[i] = datatype;
	}

	ncols = _ncols;
}

/* =========================================================== */

void
ODBCRecordSet::rewind(void)
{
	if (!this) return;
	SQL_POSITION_TO(sql_hstmt, 0);
}


int
ODBCRecordSet::fetch_row(void)
{
	if (!this) return 0;

	SQLRETURN rc = SQLFetch(sql_hstmt);

	/* no more data */
	if (SQL_NO_DATA == rc) return 0;
	if (SQL_NULL_DATA == rc) return 0;

	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't fetch row rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_STMT, sql_hstmt);
		return 0;
	}

	return 1;
}

/* =========================================================== */

int
ODBCRecordSet::get_col_by_name (const char * fieldname)
{
	int i;
	const char * fp;

	/* lookup the column number based on the column name */
	for (i=0; i<ncols; i++)
	{
		if (!strcasecmp (fieldname, column_labels[i])) return i;
	}

	/* oops. Try removing the table name if possible */
	fp = strrchr (fieldname, '.');
	if (!fp) return -1;
	fp ++;

	for (i=0; i<ncols; i++)
	{
		if (!strcasecmp (fp, column_labels[i])) return i;
	}

	return -1;
}

const char *
ODBCRecordSet::get_value(const char * fieldname)
{
	if (!this) return NULL;
	int column;

	/* If number of columns is negative, then we haven't
	 * gotten any results back yet.  Start by getting the
	 * column labels.
	 */
	if (0 > ncols)
	{
		get_column_labels();
	}

	column = get_col_by_name (fieldname);
	if (0 > column) return NULL;

	// LEAVE ("(rs=%p, fieldname=%s) {val=\'%s\'}", rs, fieldname,  rs->values[column]);
	return values[column];
}

/* =========================================================== */

#ifdef UNIT_TEST_EXAMPLE

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

// #define MAKE_SOME_DATA
#ifdef MAKE_SOME_DATA
	rs = conn->exec(
		"INSERT INTO Atoms VALUES (3,1,0.5, 0.5, 'umm');");
#endif

	// One way of doing things
	rs = conn->exec("SELECT * FROM Atoms;");
	while (rs->fetch_row())
	{
		const char * n = rs->get_value("name");
		printf ("found column with value %s\n", n);
	}
	rs->release();

	// Another way of doing things
	Ola *ola = new Ola();
	rs = conn->exec("SELECT * FROM Atoms;");
	while (rs->fetch_row())
	{
		printf("--- method 2 has row:\n");
		rs->foreach_column(&Ola::column_cb, ola);
	}
	rs->release();

	// A third way of doing things
	ola->rs = rs;
	rs = conn->exec("SELECT * FROM Atoms;");
	rs->foreach_row(&Ola::row_cb, ola);
	rs->release();

	delete conn;

	return 0;
}

#endif /* UNIT_TEST_EXAMPLE */

/* =========================================================== */

#if OLD_UNPORTED_C_CODE

/* =========================================================== */

DuiDBRecordSet *
dui_odbc_connection_tables (DuiDBConnection *dbc)
{
	DuiODBCConnection *conn = (DuiODBCConnection *) dbc;
	DuiODBCRecordSet *rs;
	SQLRETURN rc;

	ENTER ("(conn=%p)", conn);
	if (!conn) return NULL;

	rs = dui_odbc_recordset_new (conn);
	if (!rs) return NULL;

	rc = SQLTables (rs->sql_hstmt,NULL, 0, NULL, 0, NULL,0, NULL, 0);

	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't perform query rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_STMT, rs->sql_hstmt);
		dui_odbc_recordset_release (&rs->recset);
		return NULL;
	}

	LEAVE ("(conn=%p)", conn);
	/* Use numbr of columns to indicate that the query hasn't
	 * given results yet. */
	rs->ncols = -1;
	return &rs->recset;
}

/* =========================================================== */

DuiDBRecordSet *
dui_odbc_connection_table_columns (DuiDBConnection *dbc,
                                   const char * tablename)
{
	DuiODBCConnection *conn = (DuiODBCConnection *) dbc;
	DuiODBCRecordSet *rs;
	SQLRETURN rc;

	ENTER ("(conn=%p, table=%s)", conn, tablename);
	if (!conn || !tablename) return NULL;

	rs = dui_odbc_recordset_new (conn);
	if (!rs) return NULL;

	rc = SQLColumns (rs->sql_hstmt,NULL, 0, NULL, 0,
	                 (char *) tablename, SQL_NTS, NULL, 0);

	if ((SQL_SUCCESS != rc) && (SQL_SUCCESS_WITH_INFO != rc))
	{
		PERR ("Can't perform query rc=%d", rc);
		PRINT_SQLERR (SQL_HANDLE_STMT, rs->sql_hstmt);
		dui_odbc_recordset_release (&rs->recset);
		return NULL;
	}

	LEAVE ("(conn=%p, table=%s)", conn, tablename);
	/* Use numbr of columns to indicate that the query hasn't
	 * given results yet. */
	rs->ncols = -1;
	return &rs->recset;
}

#endif /* OLD_UNPORTED_C_CODE */

#endif /* HAVE_SQL_STORAGE */
/* ============================= END OF FILE ================= */

