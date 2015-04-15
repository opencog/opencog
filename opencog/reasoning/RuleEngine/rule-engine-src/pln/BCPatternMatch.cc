/*
 * BCPatternMatch.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  October 2014
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

#include "BCPatternMatch.h"

using namespace opencog;

BCPatternMatch::BCPatternMatch(AtomSpace * as)
    : DefaultPatternMatchCB(as), as_(as)
 //       : Implicator(as), DefaultPatternMatchCB(as), AttentionalFocusCB(as), PLNImplicator(as), as_(as)
{
}

BCPatternMatch::~BCPatternMatch()
{
}

bool BCPatternMatch::node_match(Handle& node1, Handle& node2)
{
	logger().debug("[BackwardChainer] In node_match looking at %s and %s", node1->toShortString().c_str(), node2->toShortString().c_str());
	return DefaultPatternMatchCB::node_match(node1, node2);

	//return AttentionalFocusCB::node_match(node1, node2);
}

bool BCPatternMatch::link_match(LinkPtr& lpat, LinkPtr& lsoln)
{
	logger().debug("[BackwardChainer] In link_match looking at %s and %s", lpat->toShortString().c_str(), lsoln->toShortString().c_str());
	return DefaultPatternMatchCB::link_match(lpat, lsoln);

	//return AttentionalFocusCB::link_match(lpat, lsoln);
}

bool BCPatternMatch::grounding(const std::map<Handle, Handle> &var_soln,
                               const std::map<Handle, Handle> &pred_soln)
{
	for (auto& p : pred_soln)
		logger().debug("PM pred: " + p.first->toShortString() + " map to " + p.second->toShortString());

	std::map<Handle, Handle> true_var_soln;

	// get rid of non-var mapping
	for (auto& p : var_soln)
	{
		if (p.first->getType() == VARIABLE_NODE)
		{
			true_var_soln[p.first] = p.second;
			logger().debug("PM var: " + p.first->toShortString() + " map to " + p.second->toShortString());
		}
	}


	// XXX TODO if a variable match to itself, reject?

	// store the variable solution
	var_solns_.push_back(true_var_soln);
	pred_solns_.push_back(pred_soln);

	return false;
}

std::vector<std::map<Handle, Handle>> BCPatternMatch::get_var_list()
{
	return var_solns_;
}
std::vector<std::map<Handle, Handle>> BCPatternMatch::get_pred_list()
{
	return pred_solns_;
}
