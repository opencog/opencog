/*
 * Commons.cc
 *
 *  Created on: 9 Oct, 2014
 *      Author: misgana
 */

#include "PLNCommons.h"

#include <opencog/util/macros.h>
#include <opencog/guile/SchemeSmob.h>

PLNCommons::PLNCommons(AtomSpace * as) :
		as_(as) {

}

PLNCommons::~PLNCommons() {

}
Handle PLNCommons::create_quoted(Handle himplicant) {
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
		throw (opencog::InvalidParamException) {
	if (!LinkCast(himplicant)) {
		throw InvalidParamException(TRACE_INFO, "Input must be a link type ");
	} //xxx why?

	//if(vnode_is_typedv)
	himplicant = create_with_unique_var(himplicant);

	HandleSeq variable_nodes = get_nodes(himplicant, vector<Type> {
			VARIABLE_NODE });
	HandleSeq list_link_elem;

	//for searching implicationLinks with variables
	if (vnode_is_typedv) {
		Handle h = as_->addNode(VARIABLE_TYPE_NODE, "VariableNode");
		for (Handle hvn : variable_nodes) {
			Handle hi = as_->addLink(TYPED_VARIABLE_LINK, HandleSeq { hvn, h });
			list_link_elem.push_back(hi);
		}
	} else
		list_link_elem = variable_nodes;

	Handle var_listLink = as_->addLink(LIST_LINK, list_link_elem,
			TruthValue::TRUE_TV());

	Handle implicant = himplicant;
	Handle implicand = himplicant; // the output should be the query result.
	assert(implicand == himplicant);
	HandleSeq implicationLink_elem { implicant, implicand };
	Handle implicatoinLink = as_->addLink(IMPLICATION_LINK,
			implicationLink_elem, TruthValue::TRUE_TV());

	HandleSeq binkLink_elements { var_listLink, implicatoinLink };
	Handle bindLink = as_->addLink(BIND_LINK, binkLink_elements,
			TruthValue::TRUE_TV());

	return bindLink;
}

HandleSeq PLNCommons::get_nodes(const Handle& hinput,
		vector<Type> required_nodes) {
	HandleSeq found_nodes;
	if (LinkCast(hinput)) {
		HandleSeq hsoutgoing = as_->getOutgoing(hinput);

		for (auto it = hsoutgoing.begin(); it != hsoutgoing.end(); ++it) {
			HandleSeq tmp = get_nodes(*it, required_nodes);
			for (Handle h : tmp) {
				if (!exists(found_nodes, h))
					found_nodes.push_back(h);
			}
		}
		return found_nodes;
	} else {
		if (NodeCast(hinput)) {
			Type t = NodeCast(hinput)->getType();
			if (required_nodes.empty()) { //empty means all kinds of nodes
				if (!exists(found_nodes, hinput))
					found_nodes.push_back(hinput);
			} else {
				auto it = find(required_nodes.begin(), required_nodes.end(), t); //check if this node is in our wish list
				if (it != required_nodes.end()) {
					if (!exists(found_nodes, hinput))
						found_nodes.push_back(hinput);
				}
			}
			return found_nodes;
		}
	}
	return found_nodes;
}

bool PLNCommons::exists(HandleSeq hseq, Handle h) {
	for (Handle hi : hseq) {
		if (hi.value() == h.value())
			return true;
	}
	return false;
}

void PLNCommons::clean_up_bind_link(Handle& hbind_link)
		throw (opencog::InvalidParamException) {
	if (as_->getType(hbind_link) != BIND_LINK) {
		throw InvalidParamException(TRACE_INFO,
				"Input must be a BIND_LINK type");
	}
	remove_vnode_containing_links(hbind_link);
}

void PLNCommons::remove_vnode_containing_links(Handle& h) {
	if (LinkCast(h)) {
		auto vnodes = get_nodes(h, vector<Type> { VARIABLE_NODE });
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
		throw (opencog::InvalidParamException) {
	if (as_->getType(himplication_link) != IMPLICATION_LINK)
		throw InvalidParamException(TRACE_INFO,
				"Input must be an ImplicationLink type ");
	remove_vnode_containing_links(himplication_link);
}

string PLNCommons::get_unique_name(Handle& h) {
//xxx temporary implementation. need to be replaced by uuid generation for making sure name is always unique
	string name = as_->getName(h);
	HandleSeq hs = as_->getIncoming(h);
	if (!hs.empty())
		name.append(to_string(hs[0].value()));
	name.append("-bcgen");
	return name;
}

Handle PLNCommons::create_with_unique_var(Handle& handle) {
	HandleSeq hvars = get_nodes(handle, vector<Type> { VARIABLE_NODE });
	map<Handle, Handle> var_unique_var_map;
	for (Handle h : hvars)
		var_unique_var_map[h] = as_->addNode(VARIABLE_NODE, get_unique_name(h)); //TODO get_uuid is not implemented
	return replace_vars(handle, var_unique_var_map);
}

Handle PLNCommons::replace_vars(Handle& h,
		map<Handle, Handle>& var_uniq_var_map) {
	Handle hcpy;
	if (LinkCast(h)) {
		HandleSeq hs = as_->getOutgoing(h);
		HandleSeq hs_cpy;
		for (Handle hi : hs) {
			if (NodeCast(hi)) {
				if (as_->getType(hi) == VARIABLE_NODE)
					hs_cpy.push_back(var_uniq_var_map[hi]);
				else
					hs_cpy.push_back(hi);
			} else if (LinkCast(hi)) {
				hs_cpy.push_back(
						replace_vars(hi, var_uniq_var_map));
			}

		}
		hcpy = as_->addLink(as_->getType(h), hs_cpy, as_->getTV(h));

	} else if (NodeCast(h)) {
		if (as_->getType(h) == VARIABLE_NODE)
			hcpy = var_uniq_var_map[h];
		else
			hcpy = h;
	}
	return hcpy;
}
