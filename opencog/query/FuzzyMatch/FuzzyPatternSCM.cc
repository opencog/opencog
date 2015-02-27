/*
 * FuzzyPatternSCM.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: Leung Man Hin <https://github.com/leungmanhin>
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

#include <opencog/guile/SchemePrimitive.h>
#include "FuzzyPatternMatch.h"
#include "FuzzyPatternSCM.h"

using namespace opencog;

FuzzyPatternSCM::FuzzyPatternSCM()
{
#ifdef HAVE_GUILE
    define_scheme_primitive("cog-fuzzy-match",
                            &FuzzyPatternMatch::find_approximate_matches,
                            new FuzzyPatternMatch);
#endif
}
