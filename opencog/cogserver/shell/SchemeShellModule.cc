/*
 * SchemeShellModule.cc
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
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

#ifdef HAVE_GUILE

#include <opencog/util/Logger.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/cogserver/server/ConsoleSocket.h>

#include "SchemeShell.h"
#include "SchemeShellModule.h"

using namespace opencog;

DECLARE_MODULE(SchemeShellModule);

SchemeShellModule::SchemeShellModule(CogServer& cs) : Module(cs)
{
}

void SchemeShellModule::init(void)
{
	shellout_register();
}

SchemeShellModule::~SchemeShellModule()
{
	shellout_unregister();
}

/**
 * Register this shell with the console.
 */
std::string SchemeShellModule::shellout(Request *req, std::list<std::string> args)
{
	ConsoleSocket *con = req->get_console();
	OC_ASSERT(con, "Invalid Request object");

	SchemeShell *sh = new SchemeShell();
	sh->set_socket(con);

	if (!args.empty())
	{
		bool hush = false;
		std::string &arg = args.front();
		if (arg == "quiet" || arg == "hush") hush = true;
		sh->hush_prompt(hush);
		sh->hush_output(hush);

		if (hush) return "";
	}

	std::string rv =
		"Entering scheme shell; use ^D or a single . on a "
		"line by itself to exit.\n" + sh->get_prompt();
	return rv;
}

#endif
/* ===================== END OF FILE ============================ */
