/*
 * opencog/comboreduct/reduct/clean_reduction.cc
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
#include "reduct.h"
#include "meta_rules.h"
#include "general_rules.h"

namespace reduct {
  const rule& clean_reduction() {
    static downwards r=downwards(remove_null_vertices());
    return r;
  }
  /*const rule& clean_and_full_reduction() {
    static sequential r=sequential(clean_reduction(),full_reduction());
    return r;
    }*/
} //~namespace reduct
