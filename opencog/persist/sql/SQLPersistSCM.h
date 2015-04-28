/*
 * opencog/persist/sql/SQLPersistSCM.h
 *
 * Copyright (c) 2008 by OpenCog Foundation
 * Copyright (c) 2008, 2009, 2013, 2015 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
 *
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

#ifndef _OPENCOG_SQL_PERSIST_SCM_H
#define _OPENCOG_SQL_PERSIST_SCM_H

#include <vector>
#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/persist/sql/AtomStorage.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class SQLBackingStore;
class SQLPersistSCM
{
private:
	static void* init_in_guile(void*);
	static void init_in_module(void*);
	void init(void);

	SQLBackingStore *_backing;
	AtomStorage *_store;
	AtomSpace *_as;

public:
	SQLPersistSCM(AtomSpace*);
	~SQLPersistSCM();

	void do_open(const std::string&, const std::string&, const std::string&);
	void do_close(void);
	void do_load(void);
	void do_store(void);

}; // class

/** @}*/
}  // namespace

extern "C" {
void opencog_persist_sql_init(void);
};

#endif // _OPENCOG_SQL_PERSIST_SCM_H
