/**
 * ensemble.cc ---
 *
 * Copyright (C) 2014 Aidyia Limited
 *
 * Authors: Linas Vepstas
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

#include <algorithm>
#include <iostream>

#include "ensemble.h"

namespace opencog {
namespace moses {

using namespace combo;

ensemble::ensemble() {}

/**
 * Implement a boosted ensemble. Candidate combo trees are added to
 * the ensemble one at a time, weights are adjusted, and etc.
 */
void ensemble::add_candidates(scored_combo_tree_set& cands)
{
	while (true) {
		// Find the element with the least error
		scored_combo_tree_set::iterator best_p = 
			std::min_element(cands.begin(), cands.end(),
				[](const scored_combo_tree& a, const scored_combo_tree& b) {
					return a.get_score() > b.get_score(); });

std::cout << "===================================== "<<std::endl;
std::cout << "duuude best " << *best_p << std::endl;
std::cout << "===================================== "<<std::endl;
		for (const scored_combo_tree& sct : cands)
		{
std::cout << "duuude its " << sct << std::endl;
		}
		break;
	}
}


}}; // namespace opencog::moses

