/*
 * LGDictReader.cc
 *
 * Copyright (c) 2012, 2013 Linas Vepstas <linasvepstas@gmail.com>
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
 *         William Ma <https://github.com/williampma>
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

#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/nlp/types/atom_types.h>
#include "LGDictReader.h"

using namespace opencog;

/**
 * Helper function for storing LG expression trees in custom container.
 *
 * @param exp   the input expression trees
 * @return      the flatten container
 */
static LGDictExpContainer lg_exp_to_container(Exp* exp)
{
    if (CONNECTOR_type == exp->type)
        return LGDictExpContainer(CONNECTOR_type, exp);

    // Whenever a null appears in an OR-list, it means the
    // entire OR-list is optional.  A null can never appear
    // in an AND-list.
    E_list* el = exp->u.l;

    if (NULL == el)
        return LGDictExpContainer(CONNECTOR_type, NULL);

    // The C data structure that link-grammar uses for connector
    // expressions is totally insane, as witnessed by the loop below.
    // Anyway: operators are infixed, i.e. are always binary,
    // with exp->u.l->e being the left hand side, and
    //      exp->u.l->next->e being the right hand side.
    // This means that exp->u.l->next->next is always null.
    std::vector<LGDictExpContainer> subcontainers;
    subcontainers.push_back(lg_exp_to_container(el->e));
    el = el->next;

    while (el && exp->type == el->e->type)
    {
        el = el->e->u.l;
        subcontainers.push_back(lg_exp_to_container(el->e));
        el = el->next;
    }

    if (el)
        subcontainers.push_back(lg_exp_to_container(el->e));

    return LGDictExpContainer(exp->type, subcontainers);
}

/**
 * Function to return LG dictionary entries.
 *
 * Given a word string, construct the corresponding atom representing
 * the Link Grammar's dictionary entry.
 *
 * The returned expression is in the form of an opencog-style
 * prefix-notation boolean expression.  Note that it is not in any
 * particular kind of normal form.  In particular, some AND nodes
 * may have only one child: these can be removed.
 *
 * Note that the order of the connectors is important: while linking,
 * these must be satisfied in left-to-right, nested order.
 *
 * Optional clauses are indicated by OR-ing with null, where "null"
 * is a CONNECTOR Node with string-value "0".  Optional clauses are
 * not necessarily in any sort of normal form; the null connector can
 * appear anywhere.
*
* XXX FIXME -- this gives incorrect results if the word has non-trivial
* morphology e.g. if its Russian, and can be split into a stem and a
* suffix.  The problem is that in LG, both stem and suffix count as
* distinct words, and so word splitting must be done first.
 *
 * @param word   the input word string
 * @return       the handle to the newly created atom
 */
HandleSeq opencog::getDictEntry(Dictionary _dictionary,
                                const std::string& word)
{
    // See if we know about this word, or not.
    Dict_node* dn_head = dictionary_lookup_list(_dictionary, word.c_str());

    HandleSeq outgoing;

// XXX FIXME -- if dn_head is null, then we should check regexes.
// Currently, LG does not do this automatically, but it almost surely
// should. i.e. the LG public API needs to also handle regexes
// automatically.
    if (!dn_head) return outgoing;

    Handle hWord(createNode(WORD_NODE, word));

    for (Dict_node* dn = dn_head; dn; dn = dn->right)
    {
        Exp* exp = dn->exp;
        HandleSeq qLG = lg_exp_to_container(exp).to_handle(hWord);

        outgoing.insert(outgoing.end(), qLG.begin(), qLG.end());
    }

    free_lookup_list(_dictionary, dn_head);
    return outgoing;
}
