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
#include <opencog/util/platform.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/ConsoleSocket.h>

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
}

SchemeShell::~SchemeShell()
{
	// Don't delete, its currently set to a singleton instance.
	//	if (evaluator) delete evaluator;
}

/**
 * Register this shell with the console.
 */
void SchemeShell::set_socket(ConsoleSocket *s)
{
	// Let the generic shell do the basic work.
	GenericShell::set_socket(s);

	//	if (!evaluator) evaluator = new SchemeEval();
	//	Someone did this singleton instance crapola because
	//	some scheme threading somehow doesn't work somewhere.
	//	buncha crap. fix this shit.
	if (!evaluator) evaluator = &SchemeEval::instance(&cogserver().getAtomSpace());
}

#endif
/* ===================== END OF FILE ============================ */
