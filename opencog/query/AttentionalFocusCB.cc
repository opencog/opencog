#include "AttentionalFocusCB.h"
#include "DefaultPatternMatchCB.h"

#include <iostream>
using namespace opencog;
bool AttentionalFocusCB::node_match(Handle& node1, Handle& node2) {
	HandleSeq af_handle_seq;
	_atom_space->getHandleSetInAttentionalFocus(back_inserter(af_handle_seq));
	bool not_match = true;
	if (af_handle_seq.empty()) {
		std::cout << "[INFO]:No atom in AF looking for more node..."
				<< std::endl;
		return true; //not a match
	} else {
		for (std::vector<Handle>::const_iterator j = af_handle_seq.begin();
				j != af_handle_seq.end(); ++j) {
			Handle h(*j);
			//select atom not in AF [what other criteria can we have here ?]
			if (node2 == h) {
				not_match = false;
				break;
			}
		}

	}
	return not_match;
}

bool AttentionalFocusCB::link_match(LinkPtr& lpat, LinkPtr& lsoln) {
	HandleSeq af_handle_seq;
	_atom_space->getHandleSetInAttentionalFocus(back_inserter(af_handle_seq));
	bool not_match = true;
	if (af_handle_seq.empty()) {
		std::cout << "[INFO]:No atom in AF looking for more link..."
				<< std::endl;
		return true; //not a match
	} else {
		Handle hlsoln(lsoln);
		for (std::vector<Handle>::const_iterator j = af_handle_seq.begin();
				j != af_handle_seq.end(); ++j) {
			Handle h(*j);
			//select atom not in AF [what other criteria can we have here ?]
			if (hlsoln == h) {
				not_match = false;
				break;
			}
		}

	}
	return not_match;
}

IncomingSet AttentionalFocusCB::get_incoming_set(Handle h) {
	const IncomingSet &incoming_set = h->getIncomingSet();
	IncomingSet filtered_set;
	IncomingSet::iterator it = filtered_set.begin();
	HandleSeq af_handle_seq;
	_atom_space->getHandleSetInAttentionalFocus(back_inserter(af_handle_seq));
	if (af_handle_seq.empty()) {
		std::cout << "[INFO]:No atom in AF,sending back default incoming set"
				<< std::endl;
		//std::vector<Handle> empty {Handle::UNDEFINED};
		//return empty; //this is causing seg fault.I don't know how to send empty set without causing seg fault
		return incoming_set;
	} else {
		bool exists = false;
		for (IncomingSet::const_iterator i = incoming_set.begin();
				i != incoming_set.end(); ++i) {
			Handle candidate_handle(*i);
			for (HandleSeq::iterator j = af_handle_seq.begin();
					j != af_handle_seq.end(); ++j) {
				Handle h(*j);
				//select atom not in AF [what other criteria can we have here ?]
				if (candidate_handle == h) {
					exists = true;
					break;
				}
			}
			if (exists) {
				filtered_set.insert(it, *i);
				exists = false;
				it++;
			}
		}
	}
	// if none is in AF
	if (filtered_set.empty()) {
		//do sth here. else search will behave absurdly [My thought]
		return incoming_set;
	}
	//std::sort(filtered_set.begin(), filtered_set.end(), compare_sti); //sort by STI

	return filtered_set;
}
