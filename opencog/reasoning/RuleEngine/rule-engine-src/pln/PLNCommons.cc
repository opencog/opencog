/*
 * PLNCommons.h
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Oct 2014
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
#include "PLNCommons.h"

#include <opencog/util/macros.h>
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/guile/SchemeSmob.h>

PLNCommons::PLNCommons(AtomSpace * as) :
        as_(as)
{

}

PLNCommons::~PLNCommons()
{
}

Handle PLNCommons::create_quoted(Handle himplicant)
{
    Handle hquoted;
    if (LinkCast(himplicant)) {
        HandleSeq hs = as_->getOutgoing(himplicant);
        HandleSeq hs_new;
        for (Handle h : hs)
            hs_new.push_back(create_quoted(h));
        hquoted = as_->addLink(as_->getType(himplicant), hs_new);
    } else if (as_->getType(himplicant) == VARIABLE_NODE)
        hquoted = as_->addLink(QUOTE_LINK, himplicant);
    else
        hquoted = himplicant;
    return hquoted;
}

Handle PLNCommons::create_bindLink(Handle himplicant, bool vnode_is_typedv)
    throw (opencog::InvalidParamException)
{
    if (!LinkCast(himplicant)) {
        throw InvalidParamException(TRACE_INFO, "Input must be a link type ");
    } //xxx why?

    //if(vnode_is_typedv)
    himplicant = replace_nodes_with_varnode(himplicant);

    UnorderedHandleSet variable_nodes = get_outgoing_nodes(himplicant,
                                                           {VARIABLE_NODE});
    HandleSeq list_link_elem;

    // For searching ImplicationLinks with variables.
    if (vnode_is_typedv) {
        Handle h = as_->addNode(TYPE_NODE, "VariableNode");
        for (Handle hvn : variable_nodes) {
            Handle hi = as_->addLink(TYPED_VARIABLE_LINK, hvn, h);
            list_link_elem.push_back(hi);
        }
    } else
        list_link_elem.insert(list_link_elem.end(),
                              variable_nodes.begin(), variable_nodes.end());

    Handle var_listLink = as_->addLink(VARIABLE_LIST, list_link_elem);

    Handle implicationLink = as_->addLink(IMPLICATION_LINK, himplicant,
                                          himplicant);

    return as_->addLink(BIND_LINK, var_listLink, implicationLink);
}

bool PLNCommons::exists_in(Handle& hlink, Handle& h)
{
    if (not LinkCast(hlink))
        throw InvalidParamException(TRACE_INFO, "Need a LINK type to look in");
    auto outg = as_->getOutgoing(hlink);
    if (find(outg.begin(), outg.end(), h) != outg.end())
        return true;
    else {
        for (Handle hi : outg) {
            if (LinkCast(hi) and exists_in(hi, h))
                return true;
        }
    }
    return false;
}

void PLNCommons::clean_up_bind_link(Handle& hbind_link)
        throw (opencog::InvalidParamException)
{
    if (as_->getType(hbind_link) != BIND_LINK) {
        throw InvalidParamException(TRACE_INFO,
                                    "Input must be a BIND_LINK type");
    }
    remove_vnode_containing_links(hbind_link);
}

void PLNCommons::remove_vnode_containing_links(Handle& h)
{
    if (LinkCast(h)) {
        auto vnodes = get_outgoing_nodes(h, {VARIABLE_NODE});
        if (not vnodes.empty()) {
            HandleSeq outgoings = as_->getOutgoing(h);
            as_->removeAtom(h);
            for (Handle h : outgoings)
                remove_vnode_containing_links(h);
        }
    } else if (NodeCast(h))
        if (as_->getType(h) == VARIABLE_NODE)
            as_->removeAtom(h);

}

void PLNCommons::clean_up_implication_link(Handle& himplication_link)
        throw (opencog::InvalidParamException)
{
    if (as_->getType(himplication_link) != IMPLICATION_LINK)
        throw InvalidParamException(TRACE_INFO,
                                    "Input must be an ImplicationLink type ");
    remove_vnode_containing_links(himplication_link);
}

string PLNCommons::get_unique_name(Handle& h)
{
//xxx temporary implementation. need to be replaced by uuid generation for making sure name is always unique
    string name = as_->getName(h);
    HandleSeq hs = as_->getIncoming(h);
    if (!hs.empty())
        name.append(to_string(hs[0].value()));
    name.append("-bcgen");
    return name;
}

Handle PLNCommons::replace_nodes_with_varnode(Handle& handle,
                                              Type t /*=VARIABLE_NODE*/)
{
    UnorderedHandleSet hvars;
    if (t == NODE)
        hvars = get_outgoing_nodes(handle); // Get every node
    else
        hvars = get_outgoing_nodes(handle, {t});
    map<Handle, Handle> node_unique_var_map;
    for (Handle h : hvars)
        node_unique_var_map[h] = as_->addNode(VARIABLE_NODE,
                                              get_unique_name(h)); //TODO get_uuid is not implemented
    return change_node_types(handle, node_unique_var_map);
}

Handle PLNCommons::change_node_types(Handle& h,
                                     map<Handle, Handle>& replacement_map)
{
    Handle hcpy;
    if (LinkCast(h)) {
        HandleSeq hs_cpy;
        HandleSeq hs = as_->getOutgoing(h);
        for (Handle hi : hs) {
            if (NodeCast(hi)) {
                if (replacement_map.find(hi) != replacement_map.end())
                    hs_cpy.push_back(replacement_map[hi]);
                else
                    hs_cpy.push_back(hi);
            } else if (LinkCast(hi)) {
                hs_cpy.push_back(change_node_types(hi, replacement_map));
            }
        }
        hcpy = as_->addLink(as_->getType(h), hs_cpy);
        hcpy->setTruthValue(as_->getTV(h));
    } else if (NodeCast(h)) {
        if (replacement_map.find(h) != replacement_map.end())
            hcpy = replacement_map[h];
        else
            hcpy = h;
    }

    return hcpy;
}

void PLNCommons::get_root_links(Handle h, HandleSeq& parents)
{
    auto incoming = as_->getIncoming(h);
    if (incoming.empty())
        return;
    else {
        for (Handle hi : incoming) {
            auto i = as_->getIncoming(hi);
            if (i.empty()) {
                if (find(parents.begin(), parents.end(), hi) == parents.end())
                    parents.push_back(hi);
            } else {
                get_root_links(hi, parents);
            }
        }
    }
}

float PLNCommons::tv_fitness(Handle h)
{
    TruthValuePtr ptv = as_->getTV(h);
    confidence_t c = ptv->getConfidence();
    strength_t s = ptv->getMean();
    return (pow((1 - s), FITNESS_PARAM) * (pow(c, (2 - FITNESS_PARAM))));
}
