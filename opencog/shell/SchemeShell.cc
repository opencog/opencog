/*
 * SchemeShell.cc
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

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/ConsoleSocket.h>
#include <opencog/server/CogServer.h>

#include "SchemeShell.h"

using namespace opencog;

SchemeShell::SchemeShell(void)
{
	normal_prompt = "guile> ";
	if (config().get_bool("ANSI_ENABLED"))
		normal_prompt = config()["ANSI_SCM_PROMPT"];
	else
		normal_prompt = config()["SCM_PROMPT"];

	abort_prompt += normal_prompt;

	pending_prompt = "... ";
	evaluator = NULL;

	do_async_output = true;
}

SchemeShell::~SchemeShell()
{
	if (evaluator) delete evaluator;
}

/**
 * Register this shell with the console.
 */
void SchemeShell::set_socket(ConsoleSocket *s)
{
	// Let the generic shell do the basic work.
	GenericShell::set_socket(s);

	if (!evaluator) evaluator = new SchemeEval(&cogserver().getAtomSpace());
	evaluator->begin_eval();
	evaluator->eval_expr("(setlocale LC_CTYPE \"\")");
	evaluator->poll_result();
}

#endif
/* ===================== END OF FILE ============================ */
