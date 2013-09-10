/*
 * opencog/comboreduct/reduct/meta_rules.cc
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
#include "meta_rules.h"
#include <opencog/util/dorepeat.h>
#include <opencog/comboreduct/combo/assumption.h>

// uncomment if you want to have a trace of the rules
// #define META_RULE_DEBUG

namespace opencog { namespace reduct {

#ifdef META_RULE_DEBUG

static int tab = 0;
void printTab() { dorepeat(tab) { std::cerr << "   "; } }

#define INC_TAB tab++;
#define DEC_TAB tab--;
#define PRINT_DEBUG_WHEN printTab(); \
    std::cerr << this->get_name() << " " << (cond?"true ":"false ") \
              << r->get_name() << " " << combo_tree(it) << std::endl;
#define PRINT_DEBUG_STANDARD printTab(); \
    std::cerr << this->get_name() << " " << r->get_name() \
              << " " << combo_tree(it) << std::endl;
#define PRINT_DEBUG_STANDARD_REF printTab(); \
    std::cerr << this->get_name() << " " << r.get_name() \
              << " " << combo_tree(it) << std::endl;

#else // META_RULE_DEBUG

#define INC_TAB ;
#define DEC_TAB ;
#define PRINT_DEBUG_WHEN ;
#define PRINT_DEBUG_STANDARD ;
#define PRINT_DEBUG_STANDARD_REF ;

#endif // META_RULE_DEBUG

void when::operator()(combo_tree& tr, combo_tree::iterator it) const {
    INC_TAB
    PRINT_DEBUG_WHEN
    if(cond)
        (*r)(tr,it);
    DEC_TAB
}

void ignore_size_increase::operator()(combo_tree& tr, combo_tree::iterator it) const {
    INC_TAB
    PRINT_DEBUG_STANDARD
    combo_tree tmp(it);
    (*r)(tr,it);
    if(tmp.size() <= combo_tree(it).size()) { // ignore size increase or equal
        replace_without_changing_it(tr, it, tmp.begin());
    }
    DEC_TAB
}

void downwards::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    INC_TAB
    combo_tree::iterator end = it;
    end.skip_children();
    ++end;

    static const type_tree unknown_type_tree =
        type_tree(opencog::combo::id::unknown_type);

    if (input == unknown_type_tree)
        for( ; it != end; ++it) {
            PRINT_DEBUG_STANDARD
            (*r)(tr, it);
        }
    else
        for( ; it != end; ++it) {
            PRINT_DEBUG_STANDARD
            if(// combo::get_argument_type_tree(*it, tr.sibling_index(it))==input
               // &&
               // @todo: checking that it inherits would be better
               // but has to be sure of it (Nil)
               opencog::combo::get_output_type_tree(*it) == type_tree(output))
                (*r)(tr, it);
        }
    DEC_TAB
}

//apply rule from the leaves of the subtree rooted by it to it
void upwards::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    INC_TAB
    combo_tree::post_order_iterator at=it,end=it;
    ++end;
    at.descend_all();

    for (;at!=end;++at) {
        PRINT_DEBUG_STANDARD
        (*r)(tr,at);
    }
    DEC_TAB
}

void sequential::operator()(combo_tree& tr,combo_tree::iterator it) const {
    INC_TAB
    for (const rule& r : rules) {
        PRINT_DEBUG_STANDARD_REF
        r(tr,it);
    }
    DEC_TAB
}

void iterative::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    INC_TAB
    combo_tree tmp;
    do {
        tmp = combo_tree(it);
        PRINT_DEBUG_STANDARD
        (*r)(tr, it);
    } while (!tr.equal_subtree(it, tmp.begin()));
    DEC_TAB
}

void assum_iterative::operator()(combo_tree& tr,combo_tree::iterator it) const {
    INC_TAB
    combo_tree tmp;
    do {
        tmp = tr;
        PRINT_DEBUG_STANDARD
        (*r)(tr,it);
    } while(tr!=tmp || !equal_assumptions(tmp, tr));
    DEC_TAB
}

} // ~namespace reduct
} // ~namespace opencog

