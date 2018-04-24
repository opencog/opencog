/*
 * LGDictExpContainer.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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
#include "LGDictExpContainer.h"

using namespace opencog;

/**
 * Constructor for CONNECTOR_type.
 *
 * @param t     must be CONNECTOR_type
 * @param exp   pointer to the LG Exp structure
 */
LGDictExpContainer::LGDictExpContainer(Exp_type t, Exp* exp)
    : m_type(t)
{
    if (t != CONNECTOR_type)
        throw InvalidParamException(TRACE_INFO,
            "Expected CONNECTOR_type for expression type.");

    // fill stuff with fixed junk for the OPTIONAL connector
    if (exp == NULL)
    {
        m_string = "OPTIONAL";
        m_direction = '+';
        m_multi = false;
        return;
    }

#if (LINK_MAJOR_VERSION == 5) && (LINK_MINOR_VERSION == 4) && (LINK_MICRO_VERSION < 4)
    m_string = exp->u.string;
    m_direction = exp->dir;
    m_multi = exp->multi;
#endif

#if (LINK_MAJOR_VERSION == 5) && (LINK_MINOR_VERSION == 4) && (LINK_MICRO_VERSION == 4)
    #error "This version of link-grammar has a broken API! Use an earlier or a later version!"
#endif

#if (LINK_MAJOR_VERSION == 5) && (LINK_MINOR_VERSION >= 5)
    m_string = lg_exp_get_string(exp);
    m_direction = lg_exp_get_dir(exp);
    m_multi = lg_exp_get_multi(exp);
#endif
}

/**
 * Constructor for AND_type and OR_type.  Always return flatten DNF.
 *
 * @param t    AND_type or OR_type
 * @param s    vector of next level's containers
 */
LGDictExpContainer::LGDictExpContainer(Exp_type t,
                                       std::vector<LGDictExpContainer> s)
    : m_type(t), m_subexps(s)
{
    if (t != AND_type && t != OR_type)
        throw InvalidParamException(TRACE_INFO,
            "Expected AND_type/OR_type for expression type.");

    // flatten & dnf on construction
    basic_flatten();
    basic_dnf();

    // need flattening again after changing to dnf
    basic_flatten();

    // convert to normal order (- before +)
    basic_normal_order();
}

/**
 * Flatten the container.
 *
 * Remove nested and-recursive trees. Thus, AND(A, AND(B, C)) becomes
 * AND(A, B, C), assuming the sub-levels are already flattened.
 */
void LGDictExpContainer::basic_flatten()
{
    std::vector<LGDictExpContainer> new_subexps;

    // do not need this level if only one sub-expression
    if (m_subexps.size() == 1)
    {
        m_type = m_subexps[0].m_type;

        if (m_type == CONNECTOR_type)
        {
            m_string = m_subexps[0].m_string;
            m_direction = m_subexps[0].m_direction;
            m_multi = m_subexps[0].m_multi;
        }
        else
        {
            m_subexps = m_subexps[0].m_subexps;
        }

        return;
    }

    for (LGDictExpContainer& exp : m_subexps)
    {
        // cannot merge diffenent type
        if (m_type != exp.m_type)
        {
            new_subexps.push_back(exp);
            continue;
        }

        // bring things up a level, assuming the sub-level is already flat
        for (LGDictExpContainer& subexp : exp.m_subexps)
            new_subexps.push_back(subexp);
    }

    m_subexps = new_subexps;
}

/**
 * Construct disjunctive normal form.
 *
 * Convert the expression into DNF form (ie. a disjuction of
 * conjunctions of connectors).
 *
 * Connectors are OR-distributive but not AND-distributive. Thus, while
 * (A & (B or C)) = ((A & B) or (A & C)), it is NOT the case that
 * (A or (B & C)) = ((A or B) & (A or C)).
 */
void LGDictExpContainer::basic_dnf()
{
    std::vector<LGDictExpContainer> new_subexps(m_subexps);

    // assuming sub-level already in dnf, then OR can just be returned
    if (m_type == OR_type)
        return;

    // find the first OR to distribute
    auto or_exp_it = std::find_if(new_subexps.begin(),
                                  new_subexps.end(),
           [](LGDictExpContainer& exp) { return exp.m_type == OR_type; });

    // no OR, the end
    if (or_exp_it == new_subexps.end())
        return;

    // copy the OR exp out, then remove it from the list
    LGDictExpContainer or_exp = *or_exp_it;
    new_subexps.erase(or_exp_it);

    // Change the type of this container to OR, and distribute
    // the stuff in or_exp
    m_type = OR_type;
    m_subexps.clear();

    for (auto& e : or_exp.m_subexps)
    {
        // don't bother distributing the optional connector, just clone
        if (e.m_type == CONNECTOR_type && e.m_string == "OPTIONAL")
        {
            m_subexps.push_back(LGDictExpContainer(AND_type, new_subexps));
            continue;
        }

        new_subexps.push_back(e);
        m_subexps.push_back(LGDictExpContainer(AND_type, new_subexps));
        new_subexps.pop_back();
    }
}

/**
 * Convert to normal order.
 *
 * Putting connectors with - direction before those with + direction.
 * Assume everything already in DNF.
 */
void LGDictExpContainer::basic_normal_order()
{
    // do nothing for OR
    if (m_type == OR_type)
        return;

    std::vector<LGDictExpContainer> new_leftexps;
    std::vector<LGDictExpContainer> new_rightexps;

    for (auto& exp : m_subexps)
    {
        if (exp.m_direction == '-')
            new_leftexps.push_back(exp);
        else
            new_rightexps.push_back(exp);
    }

    m_subexps = new_leftexps;
    m_subexps.insert(m_subexps.end(),
                     new_rightexps.begin(),
                     new_rightexps.end());
}

/**
 * Create the OpenCog atom for the LG dictionary expression.
 *
 * @param as   pointer to the AtomSpace
 * @return     handle to the atom
 */
HandleSeq LGDictExpContainer::to_handle(const Handle& hWordNode)
{
    static Handle multi(createNode(LG_CONN_MULTI_NODE, "@"));
    static Handle optnl(createLink(LG_CONNECTOR,
                           Handle(createNode(LG_CONNECTOR_NODE, "0"))));

    if (m_type == CONNECTOR_type)
    {
        // XXX FIXME this does not smell right; optionals should get
        // blown up into pairs of disjuncts, one with and one without.
        if (m_string == "OPTIONAL") return { optnl };

        Handle connector(createNode(LG_CONNECTOR_NODE, m_string));
        Handle direction(createNode(LG_CONN_DIR_NODE,
                          std::string(1, m_direction)));

        if (m_multi)
            return { Handle(createLink(LG_CONNECTOR, connector, direction, multi)) };
        else
            return { Handle(createLink(LG_CONNECTOR, connector, direction)) };
    }

    HandleSeq outgoing;

    for (auto& exp: m_subexps)
    {
        HandleSeq q = exp.to_handle(hWordNode);
        outgoing.insert(outgoing.end(), q.begin(), q.end());
    }

    if (m_type == AND_type)
        return { Handle(createLink(outgoing, LG_AND)) };

    // remove repeated atoms from OR
    if (m_type == OR_type)
    {
        // XXX FIXME ... using an std::map would be more efficient.
        std::sort(outgoing.begin(), outgoing.end());
        outgoing.erase(std::unique(outgoing.begin(),
                                   outgoing.end()),
                                   outgoing.end());
        HandleSeq qDisjuncts;
        for (const Handle& h : outgoing)
            qDisjuncts.push_back(Handle(createLink(LG_DISJUNCT, hWordNode, h)));

        return qDisjuncts;
    }

    // Should never get here
    OC_ASSERT(false, "Unknown Link Grammar Expression type %d", m_type);
    return HandleSeq();
}
