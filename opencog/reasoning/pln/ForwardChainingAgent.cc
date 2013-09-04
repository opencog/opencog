/*
 * opencog/reasoning/pln/ForwardChainingAgent.cc
 *
 * Copyright (C) 2010 by OpenCog Foundation
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * All Rights Reserved
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

#include "ForwardChainingAgent.h"

#include "ForwardChainer.h"

#include "PLNUtils.h"
#include "utils/NMPrinter.h"

using std::vector;
using std::cout;
using std::endl;

namespace opencog
{
ForwardChainingAgent::ForwardChainingAgent(CogServer& cs) : Agent(cs) { }

ForwardChainingAgent::~ForwardChainingAgent() { }

void ForwardChainingAgent::run()
{
	// More of this should be / is done elsewhere

	cout << "FWBETA Starting chaining:" << endl;
	pHandleSeq results = fc.fwdChain(FWD_CHAIN_MAX_APPS);
	cout << "FWBETA Chaining finished, results:" << endl;
	NMPrinter np;
	foreach (pHandle h, results) {
	  np(h);
	}
	cout << results.size() << " results" << endl;
}

}
