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

// Convenience wrapper, for stand-alone usage.
std::string opencog::eval_scheme(AtomSpace& as, const std::string &s)
{
#ifdef HAVE_GUILE
   SchemeEval& evaluator = SchemeEval::instance(&as);
   std::string scheme_return_value = evaluator.eval(s);

   if (evaluator.eval_error()) {
      logger().error( "%s - Failed to execute '%s'",
                     __FUNCTION__, s.c_str());
   }

   if (evaluator.input_pending()) {
      logger().error( "%s - Scheme syntax error in input: '%s'",
                     __FUNCTION__, s.c_str());
   }
   evaluator.clear_pending();

   return scheme_return_value;
#else // HAVE_GUILE
   return "";
#endif // HAVE_GUILE
}
