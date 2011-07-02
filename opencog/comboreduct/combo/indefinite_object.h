/*
 * opencog/comboreduct/combo/indefinite_object.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _COMBO_INDEFINITE_OBJECT_H
#define _COMBO_INDEFINITE_OBJECT_H

#include <opencog/util/exceptions.h>

#include "type_tree_def.h"
#include "operator_base.h"

namespace opencog { namespace combo {
  
//indefinite_object inherits from operator_base
//without additional properties
class indefinite_object_base : public operator_base {
public:
    virtual ~indefinite_object_base() {}
};

typedef const indefinite_object_base* indefinite_object;

typedef std::set<indefinite_object> indefinite_object_set;
typedef indefinite_object_set::iterator indefinite_object_set_it;
typedef indefinite_object_set::const_iterator indefinite_object_set_const_it;

} // ~namespace combo

std::ostream& operator<<(std::ostream&, combo::indefinite_object);

} // ~namespace opencog

#endif

