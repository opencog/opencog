/*
 * opencog/persist/sql/PersistModule.cc
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

#include <opencog/persist/guile/PersistSCM.h>

#include "PersistModule.h"

using namespace opencog;

namespace opencog {

DECLARE_MODULE(PersistModule);

PersistModule::PersistModule(CogServer& cs) : Module(cs)
{
	opencog_persist_init();

	_api = new SQLPersistSCM(&_cogserver.getAtomSpace());

	do_close_register();
	do_load_register();
	do_open_register();
	do_store_register();
}

PersistModule::~PersistModule()
{
	do_close_unregister();
	do_load_unregister();
	do_open_unregister();
	do_store_unregister();
	delete _api;
}

void PersistModule::init(void)
{
}

std::string PersistModule::do_close(Request *dummy, std::list<std::string> args)
{
	if (!args.empty())
		return "sql-close: Error: Unexpected argument\n";

	try
	{
		_api->do_close();
	}
	catch (const std::exception& ex)
	{
		return std::string(ex.what()) + "\n";
	}

	return "Database closed\n";
}

std::string PersistModule::do_load(Request *dummy, std::list<std::string> args)
{
	if (!args.empty())
		return "sql-load: Error: Unexpected argument\n";

	try
	{
		_api->do_load();
	}
	catch (const std::exception& ex)
	{
		return std::string(ex.what()) + "\n";
	}

	return "Database load completed\n";
}


std::string PersistModule::do_open(Request *dummy, std::list<std::string> args)
{
	if (args.size() != 1 and args.size() != 3)
		return "sql-open: Error: invalid command syntax\n"
		       "Usage: sql-open <uri>\n"
		       "Usage: sql-open <dbname> <username> <passwd>\n";

	std::string uri;
	if (1 == args.size())
	{
		uri = args.front();
	}
	else
	{
		std::string dbname   = args.front(); args.pop_front();
		std::string username = args.front(); args.pop_front();
		std::string auth     = args.front(); args.pop_front();
		uri = "postgres://" + dbname + "?user=" + username;
		uri += "&password=" + auth;
	}

	try
	{
		_api->do_open(uri);
	}
	catch (const std::exception& ex)
	{
		return std::string(ex.what()) + "\n";
	}

	std::string rc = "Opened \"" + uri + "\"\n";
	return rc;
}

std::string PersistModule::do_store(Request *dummy, std::list<std::string> args)
{
	if (!args.empty())
		return "sql-store: Error: Unexpected argument\n";

	try
	{
		_api->do_store();
	}
	catch (const std::exception& ex)
	{
		return std::string(ex.what()) + "\n";
	}

	return "Database store completed\n";
}

}
