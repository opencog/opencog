/*
 * Valuations.cc
 *
 * Copyright (C) 2018 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include "Valuations.h"

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <opencog/atoms/base/Atom.h>

namespace opencog
{

Valuations::Valuations(const Variables& vars, const Handle& satset)
	: variables(vars)
{
	if (satset)
	{
		OC_ASSERT(satset->get_type() == SET_LINK);
		for (const Handle& vals : satset->getOutgoingSet())
		{
			if (vars.size() == 1)
				values.push_back({vals});
			else
				values.push_back(vals->getOutgoingSet());
		}
	}
}

bool Valuations::novar() const
{
	return variables.empty();
}

Handle Valuations::front_variable() const
{
	return variables.varseq[0];
}

Handle Valuations::variable(unsigned i) const
{
	return variables.varseq[i];
}
	
Valuations Valuations::erase_front() const
{
	Variables nvars(variables);
	nvars.erase(front_variable());
	Valuations nvals(nvars);
	if (not nvals.novar())
		for (const HandleSeq& vals : values)
			nvals.values.emplace_back(std::next(vals.begin()), vals.end());
	return nvals;
}
	
std::string oc_to_string(const Valuations& valuations)
{
	std::stringstream ss;
	ss << "variables:" << std::endl << oc_to_string(valuations.variables);
	ss << "values:" << std::endl << oc_to_string(valuations.values);
	return ss.str();
}

std::string oc_to_string(const HandleValuationsMap& h2vals)
{
	std::stringstream ss;
	int i = 0;
	for (const auto& hv : h2vals)
	{
		ss << "atom [" << i << "]:" << std::endl << oc_to_string(hv.first);
		ss << "valuations [" << i << "]:" << std::endl << oc_to_string(hv.second);
		++i;
	}
	return ss.str();
}

} // namespace opencog
