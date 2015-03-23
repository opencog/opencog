/*
 * opencog/learning/moses/example-progs/headers.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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

/* A collection of assorted headers that the various example programs
 * will need. These define the various optimization algos that get used.
 */

#include <opencog/util/Logger.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/selection.h>

#include <opencog/learning/moses/eda/initialization.h>
#include <opencog/learning/moses/eda/local_structure.h>
#include <opencog/learning/moses/eda/logging.h>
#include <opencog/learning/moses/eda/optimize.h>
#include <opencog/learning/moses/eda/replacement.h>
#include <opencog/learning/moses/eda/termination.h>

#include "edaopt.h"
#include "scoring_functions.h"

using namespace opencog;
using namespace moses;

