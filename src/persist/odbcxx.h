/*
 * FUNCTION:
 * ODBC driver -- developed/tested with both iODBC http://www.iodbc.org
 * and with unixODBC
 *
 * HISTORY:
 * Copyright (c) 2002,2008 Linas Vepstas <linas@linas.org>
 * created by Linas Vepstas  March 2002
 * ported to C++ March 2008
 */

#include <stack>
#include <string>

#include <sql.h>
#include <sqlext.h>

class ODBCRecordSet;

class ODBCConnection
{
	friend class ODBCRecordSet;
	private:
		std::string dbname;
		std::string username;
		SQLHENV sql_henv;
		SQLHDBC sql_hdbc;
		std::stack<ODBCRecordSet *> free_pool;

		ODBCRecordSet *get_record_set(void);

	public:
		ODBCConnection(const char * dbname,
		               const char * username,
		               const char * authentication);
		~ODBCConnection();

		ODBCRecordSet *exec(const char * buff);
};

class ODBCRecordSet
{
	friend class ODBCConnection;
	private:
		ODBCConnection *conn;
		SQLHSTMT sql_hstmt;

		int ncols;
		int arrsize;
		char **column_labels;
		int  *column_datatype;
		char **values;
		int  *vsizes;

		void alloc_and_bind_cols(int ncols);
		ODBCRecordSet(ODBCConnection *);
		~ODBCRecordSet();

		void get_column_labels(void);
		int get_col_by_name (const char *);

	public:
		// rewind the cursor to the start
		void rewind(void);

		int fetch_row(void); // return non-zero value if there's another row.
		const char * get_value(const char * fieldname);

		// call this, instead of the destructor, 
		// when done with this instance.
		void release(void);

		// Calls the callback once for each row.
		template<class T> bool 
			foreach_row(bool (T::*cb)(void), T *data);

		// Calls the callback once for each column.
		template<class T> bool 
			foreach_column(bool (T::*cb)(const char *, const char *), T *data);
};

