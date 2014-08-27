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

#include <opencog/guile/SchemeEval.h>
#include <opencog/server/ConsoleSocket.h>
#include <opencog/util/Logger.h>
#include <opencog/util/foreach.h>

#include "SchemeShellModule.h"

using namespace opencog;

DECLARE_MODULE(SchemeShellModule);

SchemeShellModule::SchemeShellModule(CogServer& cs) : Module(cs)
{
	evaluator = new SchemeEval(&cs.getAtomSpace());
}

void SchemeShellModule::init(void)
{
	shellout_register();
	do_eval_register();
}

SchemeShellModule::~SchemeShellModule()
{
	shellout_unregister();
	do_eval_unregister();
	delete evaluator;
}

/**
 * Register this shell with the console.
 */
std::string SchemeShellModule::shellout(Request *req, std::list<std::string> args)
{
	ConsoleSocket *s = dynamic_cast<ConsoleSocket*>(req->getRequestResult());
	if (!s)
		throw RuntimeException(TRACE_INFO, "Invalid RequestResult object"
		       " for SchemeShellModule: a ConsoleSocket object was expected.");

	SchemeShell *sh = new SchemeShell();
	sh->set_socket(s);

	bool hush = false;
	if (0 < args.size())
	{
		std::string &arg = args.front();
		if (arg.compare("quiet") || arg.compare("hush")) hush = true;
	}
	sh->hush_prompt(hush);
	sh->hush_output(hush);

	if (hush) return "";

	std::string rv =
		"Entering scheme shell; use ^D or a single . on a "
		"line by itself to exit.\n" + sh->get_prompt();
	return rv;
}

std::string SchemeShellModule::do_eval(Request *req, std::list<std::string> args)
{
	// Needs to join the args back up into one string.
	std::string expr;
	std::string out;

	// Adds an extra space on the end, but that doesn't matter.
	foreach(std::string arg, args)
	{
		expr += arg + " ";
	}

	evaluator->begin_eval();
	evaluator->eval_expr(expr);
	out = evaluator->poll_result();
	// May not be necessary since an error message and backtrace are provided.
	if (evaluator->eval_error()) {
		out += "An error occurred\n";
	}
	if (evaluator->input_pending()) {
		out += "Invalid Scheme expression: missing something";
	}
	evaluator->clear_pending();

	return out;
}

#endif
/* ===================== END OF FILE ============================ */
