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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/BackingStore.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/types/atom_types.h>

#include "PersistModule.h"
#include "AtomStorage.h"

using namespace opencog;

namespace opencog {

DECLARE_MODULE(PersistModule);

PersistModule::PersistModule(CogServer& cs) : Module(cs)
{
	_api = new PersistSCM(&_cogserver.getAtomSpace());

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
	return _api->do_close(args);
	// return "Database closed\n";
}

std::string PersistModule::do_load(Request *dummy, std::list<std::string> args)
{
	return _api->do_load(args);
	// return "Database load completed\n";
}


std::string PersistModule::do_open(Request *dummy, std::list<std::string> args)
{
	return _api->do_open(args);

	// std::string rc = "Opened \"" + dbname + "\" as user \"" + username + "\"\n"; 
	// return rc;
}

std::string PersistModule::do_store(Request *dummy, std::list<std::string> args)
{
	return _api->do_store(args);
	// return "Database store completed\n";
}
}
