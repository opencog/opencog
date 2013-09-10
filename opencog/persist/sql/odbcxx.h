/*
 * FUNCTION:
 * ODBC driver -- developed/tested with both iODBC http://www.iodbc.org
 * and with unixODBC
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

#ifndef _OPENCOG_PERSISTENT_ODBC_DRIVER_H
#define _OPENCOG_PERSISTENT_ODBC_DRIVER_H

#include <stack>
#include <string>

#include <sql.h>
#include <sqlext.h>

/** \addtogroup grp_persist
 *  @{
 */

class ODBCRecordSet;

class ODBCConnection
{
	friend class ODBCRecordSet;
	private:
		std::string dbname;
		std::string username;
		bool is_connected;
		SQLHENV sql_henv;
		SQLHDBC sql_hdbc;
		std::stack<ODBCRecordSet *> free_pool;

		ODBCRecordSet *get_record_set(void);

	public:
		ODBCConnection(const char * dbname,
		               const char * username,
		               const char * authentication);
		~ODBCConnection();

		bool connected(void) const;

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
			foreach_row(bool (T::*cb)(void), T *data)
		{
			while (fetch_row())
			{
				bool rc = (data->*cb) ();
				if (rc) return rc;
			}
			return false;
		}

		// Calls the callback once for each column.
		template<class T> bool 
			foreach_column(bool (T::*cb)(const char *, const char *), T *data)
		{
			int i;
			if (0 > ncols)
			{
				get_column_labels();
			}

			for (i=0; i<ncols; i++)
			{
				bool rc = (data->*cb) (column_labels[i], values[i]);
				if (rc) return rc;
			}
			return false;
		}
};

/**
 * Handy-dandy utility function: since SQL uses single-quotes
 * for delimiting strings, the strings themselves need to have 
 * any single-quotes escaped, to avoid bad syntax.
 */
inline void escape_single_quotes(std::string &str)
{
	std::string::size_type pos = 0;
	pos = str.find ('\'', pos);
	while (pos != std::string::npos)
	{
		str.insert(pos, 1, '\'');
		pos += 2;
		pos = str.find('\'', pos);
	}
}

/** @}*/

#endif // _OPENCOG_PERSISTENT_ODBC_DRIVER_H
