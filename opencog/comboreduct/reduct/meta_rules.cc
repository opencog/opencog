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
#include <opencog/util/foreach.h>
#include <opencog/util/dorepeat.h>
#include <opencog/comboreduct/combo/assumption.h>

// uncomment if you want to have a trace of the rules
// #define META_RULE_DEBUG

namespace reduct {

#ifdef META_RULE_DEBUG
static int tab = 0;
void printTab() { dorepeat(tab) { std::cout << "   "; } }
#endif

void when::operator()(combo_tree& tr, combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

#ifdef META_RULE_DEBUG
    printTab();
    std::cout << this->get_name() << " " << (cond?"true ":"false ")
              << r->get_name() << " " << combo_tree(it) << std::endl;
#endif
    if(cond)
        (*r)(tr,it);

#ifdef META_RULE_DEBUG
    tab--;
#endif
}

void ignore_size_increase::operator()(combo_tree& tr, combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

#ifdef META_RULE_DEBUG
    printTab();
    std::cout << this->get_name() << " " << r->get_name() 
              << " " << combo_tree(it) << std::endl;
#endif
    combo_tree tmp(it);
    (*r)(tr,it);
    if(tmp.size() < combo_tree(it).size()) { // ignore the size increase
        replace_without_changing_it(tr, it, tmp.begin());
    }

#ifdef META_RULE_DEBUG
    tab--;
#endif
}

void downwards::operator()(combo_tree& tr,combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

    combo_tree::iterator end=it;  
    end.skip_children();
    ++end;
    
    static const type_tree unknown_type_tree = 
        type_tree(combo::id::unknown_type);
    
    if (input==unknown_type_tree)
        for(;it!=end;++it) {
#ifdef META_RULE_DEBUG
            printTab();
            std::cout << this->get_name() << " " << r->get_name() 
                      << " " << combo_tree(it) << std::endl;
#endif
            (*r)(tr,it);
        }
    else
        for(;it!=end;++it) {
            // debug
#ifdef META_RULE_DEBUG
            printTab();
            std::cout << this->get_name() << " " << r->get_name() 
                      << " " << combo_tree(it) << std::endl;
#endif
            if(//combo::get_argument_type_tree(*it, tr.sibling_index(it))==input
               //&& 
               //@todo: checking that it inherits would be better
               //but has to be sure of it (Nil)
               combo::get_output_type_tree(*it)==type_tree(output))
                (*r)(tr,it);
        }

#ifdef META_RULE_DEBUG
    tab--;
#endif
}

//apply rule from the leaves of the subtree rooted by it to it
void upwards::operator()(combo_tree& tr,combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

    combo_tree::post_order_iterator at=it,end=it;
    ++end;
    at.descend_all();
    
    for (;at!=end;++at) {
#ifdef META_RULE_DEBUG
        printTab();
        std::cout << this->get_name() << " " << r->get_name() 
                  << " " << combo_tree(at) << std::endl;
#endif
        (*r)(tr,at);
    }

#ifdef META_RULE_DEBUG
    tab--;
#endif
}
    
void sequential::operator()(combo_tree& tr,combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

    foreach (const rule& r,rules) {
#ifdef META_RULE_DEBUG
        printTab();
        std::cout << this->get_name() << " " << r.get_name() 
                  << " " << combo_tree(it) << std::endl;
#endif
        r(tr,it);
    }

#ifdef META_RULE_DEBUG
    tab--;
#endif
}

void iterative::operator()(combo_tree& tr,combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

    combo_tree tmp;
    do {
        tmp=combo_tree(it);
#ifdef META_RULE_DEBUG
        printTab();
        std::cout << this->get_name() << " " << r->get_name() 
                  << " " << combo_tree(it) << std::endl;
#endif
        (*r)(tr,it);
    } while (!tr.equal_subtree(it,tmp.begin()));

#ifdef META_RULE_DEBUG
    tab--;
#endif
}
    
void assum_iterative::operator()(combo_tree& tr,combo_tree::iterator it) const {
#ifdef META_RULE_DEBUG
    tab++;
#endif

    combo_tree tmp;
    do {
        tmp = tr;
#ifdef META_RULE_DEBUG
        printTab();
        std::cout << this->get_name() << " " << r->get_name() 
                  << " " << combo_tree(it) << std::endl;
#endif
        (*r)(tr,it);
    } while(tr!=tmp || !equal_assumptions(tmp, tr));

#ifdef META_RULE_DEBUG
    tab--;
#endif
}

} //~namespace reduct

