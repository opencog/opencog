/*
 * opencog/cython/PyScheme.cc
 *
 * Copyright (C) 2013 by OpenCog Foundation
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
#include "PyScheme.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/CogServer.h>

using std::string;

using namespace opencog;

#ifdef HAVE_GUILE

static void check_err(SchemeEval* evaluator, const std::string &s)
{
	if (evaluator->eval_error()) {
		throw RuntimeException(TRACE_INFO,
		       "Scheme: Failed to execute '%s'", s.c_str());
	}

	if (evaluator->input_pending()) {
		throw RuntimeException(TRACE_INFO,
		      "Scheme: Syntax error in input: '%s'", s.c_str());
	}
}
#endif // HAVE_GUILE

// Convenience wrapper, for stand-alone usage.
std::string opencog::eval_scheme(AtomSpace& as, const std::string &s)
{
#ifdef HAVE_GUILE
	SchemeEval* evaluator = SchemeEval::get_evaluator(&as);
	std::string scheme_return_value = evaluator->eval(s);
	check_err(evaluator, s);
	return scheme_return_value;
#else // HAVE_GUILE
	return "Error: Compiled without Guile support";
#endif // HAVE_GUILE
}

// Convenience wrapper, for stand-alone usage.
Handle opencog::eval_scheme_h(AtomSpace& as, const std::string &s)
{
#ifdef HAVE_GUILE
	SchemeEval* evaluator = SchemeEval::get_evaluator(&as);
	Handle scheme_return_value = evaluator->eval_h(s);
	check_err(evaluator, s);
	return scheme_return_value;
#else // HAVE_GUILE
	return "Error: Compiled without Guile support";
#endif // HAVE_GUILE
}
