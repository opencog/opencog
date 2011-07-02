/*
 * opencog/comboreduct/combo/action_symbol.h
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
#ifndef _COMBO_ACTION_SYMBOL_H
#define _COMBO_ACTION_SYMBOL_H

#include "operator_base.h"
#include "type_tree_def.h"

namespace opencog { namespace combo {

//action_symbol_base inherit operator_base
//without additional properties
class action_symbol_base : public operator_base {
public:
    virtual ~action_symbol_base() {}
};

typedef const action_symbol_base* action_symbol;
  
} // ~namespace combo

std::ostream& operator<<(std::ostream&, combo::action_symbol);

} // ~namespace opencog

#endif
