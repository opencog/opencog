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
	_cogserver.registerRequest(shelloutRequest::info().id,
	                           &shelloutFactory);
}

SchemeShellModule::~SchemeShellModule()
{
	_cogserver.unregisterRequest(shelloutRequest::info().id);
}

const RequestClassInfo&
SchemeShellModule::shelloutRequest::info(void)
{
	static const RequestClassInfo _cci("scm",
		"Enter the scheme shell",
		"Usage: scm [hush|quiet|sync]\n\n"
		"Enter the scheme interpreter shell. This shell provides a rich\n"
		"and easy-to-use environment for creating, deleting and manipulating\n"
		"OpenCog atoms and truth values. It provides a full R5RS-compliant\n"
		"interactive scheme shell, based on the GNU Guile extension language.\n\n"
		"If 'hush' or 'quiet' is specified after the command, then the prompt\n"
		"will not be returned.  This is nice when catting large scripts using\n"
		"netcat, as it avoids printing garbage when the scripts work well.\n"
		"If 'sync' is specified after the command, then the output is sync,\n"
		"instead of async.\n",
		true, false);
	return _cci;
}

/**
 * Register this shell with the console.
 */
bool
SchemeShellModule::shelloutRequest::execute(void)
{
	ConsoleSocket *con = this->get_console();
	OC_ASSERT(con, "Invalid Request object");

	SchemeShell *sh = new SchemeShell();
	sh->set_socket(con);

	if (!_parameters.empty())
	{
		bool hush = false;
		std::string &arg = _parameters.front();
		if (arg == "quiet" || arg == "hush") hush = true;
		sh->hush_prompt(hush);
		sh->hush_output(hush);

		if (hush) { send(""); return true; }
	}

	std::string rv =
		"Entering scheme shell; use ^D or a single . on a "
		"line by itself to exit.\n" + sh->get_prompt();
	send(rv);
	return true;
}

#endif
/* ===================== END OF FILE ============================ */
