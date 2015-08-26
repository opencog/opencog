/*
 * opencog/modules/PersistZmqModule.cc
 *
 * Copyright (c) 2008 by OpenCog Foundation
 * Copyright (c) 2008, 2009, 2013, 2015 Linas Vepstas <linasvepstas@gmail.com>
 * Copyright (c) 2015 Hendy Irawan <ceefour666@gmail.com>
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

#include "PersistZmqModule.h"

using namespace opencog;

namespace opencog {

DECLARE_MODULE(PersistZmqModule);

PersistZmqModule::PersistZmqModule(CogServer& cs) : Module(cs)
{
	_api = new ZMQPersistSCM(&_cogserver.getAtomSpace());

	do_close_register();
	do_load_register();
	do_open_register();
	do_store_register();
}

PersistZmqModule::~PersistZmqModule()
{
	do_close_unregister();
	do_load_unregister();
	do_open_unregister();
	do_store_unregister();
	delete _api;
}

void PersistZmqModule::init(void)
{
}

std::string PersistZmqModule::do_close(Request *dummy, std::list<std::string> args)
{
	if (!args.empty())
		return "zmq-close: Error: Unexpected argument\n";

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

std::string PersistZmqModule::do_load(Request *dummy, std::list<std::string> args)
{
	if (!args.empty())
		return "zmq-load: Error: Unexpected argument\n";

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


std::string PersistZmqModule::do_open(Request *dummy, std::list<std::string> args)
{
	if (args.size() != 3)
		return "zmq-open: Error: invalid command syntax\n"
		       "Usage: zmq-open <network_address>\n";

	std::string networkAddress = args.front(); args.pop_front();

	try
	{
		_api->do_open(networkAddress);
	}
	catch (const std::exception& ex)
	{
		return std::string(ex.what()) + "\n";
	}

	std::string rc = "Opened \"" + networkAddress + "\n";
	return rc;
}

std::string PersistZmqModule::do_store(Request *dummy, std::list<std::string> args)
{
	if (!args.empty())
		return "zmq-store: Error: Unexpected argument\n";

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
