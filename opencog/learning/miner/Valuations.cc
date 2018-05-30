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

#include "Miner.h"

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <opencog/atoms/base/Atom.h>
#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/atoms/core/LambdaLink.h>
#include <opencog/atomutils/TypeUtils.h>

#include <boost/range/algorithm/find.hpp>

namespace opencog
{

////////////////////
// ValuationsBase //
////////////////////

ValuationsBase::ValuationsBase(const Variables& vars) : variables(vars) {}

ValuationsBase::ValuationsBase() {}

bool ValuationsBase::novar() const
{
	return variables.empty();
}

Handle ValuationsBase::front_variable() const
{
	return variables.varseq[0];
}

Handle ValuationsBase::variable(unsigned i) const
{
	return variables.varseq[i];
}

unsigned ValuationsBase::index(const Handle& var) const
{
	return variables.index.at(var);
}

unsigned ValuationsBase::size() const
{
	return 0;
}

//////////////////
// SCValuations //
//////////////////

SCValuations::SCValuations(const Variables& vars, const Handle& satset)
	: ValuationsBase(vars)
{
	if (satset)
	{
		OC_ASSERT(satset->get_type() == SET_LINK);
		for (const Handle& vals : satset->getOutgoingSet())
		{
			if (vars.size() == 1)
				valuations.push_back({vals});
			else
				valuations.push_back(vals->getOutgoingSet());
		}
	}
}

SCValuations SCValuations::erase_front() const
{
	return erase(front_variable());
}

SCValuations SCValuations::erase(const Handle& var) const
{
	// No variable, just return itself
	if (not variables.is_in_varset(var))
		return *this;

	// Remove variable
	Variables nvars(variables);
	nvars.erase(var);
	auto it = boost::find(variables.varseq, var);
	int dst = std::distance(variables.varseq.begin(), it);
	SCValuations nvals(nvars);
	if (not nvals.novar())
	{
		for (HandleSeq vals : valuations)
		{
			vals.erase(std::next(vals.begin(), dst));
			nvals.valuations.push_back(vals);
		}
	}
	return nvals;
}

HandleUCounter SCValuations::values(const Handle& var) const
{
	return values(index(var));
}

HandleUCounter SCValuations::values(unsigned var_idx) const
{
	HandleUCounter vals;
	for (const HandleSeq& valuation : valuations)
		vals[valuation[var_idx]]++;
	return vals;
}

bool SCValuations::operator==(const SCValuations& other) const
{
	return variables == other.variables;
}

bool SCValuations::operator<(const SCValuations& other) const
{
	return variables < other.variables;
}

unsigned SCValuations::size() const
{
	return valuations.size();
}

////////////////
// Valuations //
////////////////

Valuations::Valuations(const Handle& pattern, const HandleSet& texts)
	: ValuationsBase(Miner::get_variables(pattern))
{
	for (const Handle& cp : Miner::get_component_patterns(pattern))
	{
		Handle satset = Miner::restricted_satisfying_set(cp, texts);
		scvs.insert(SCValuations(Miner::get_variables(cp), satset));
	}
	setup_size();
}

Valuations::Valuations(const Variables& vars, const SCValuationsSet& sc)
	: ValuationsBase(vars), scvs(sc)
{
	setup_size();
}

Valuations::Valuations(const Variables& vars)
	: ValuationsBase(vars), _size(0) {}

Valuations Valuations::erase_front() const
{
	// Take remaining variables
	Variables nvars(variables);
	Handle var = front_variable();
	nvars.erase(var);
	Valuations nvals(nvars);

	// Move values from remaining variables to new Valuations
	for (const SCValuations& scv : scvs)
	{
		SCValuations nscvals(scv.erase(var));
		if (not nscvals.novar())
			nvals.scvs.insert(nscvals);
	}

	// Keep the previous size as we still need to consider those
	// combinations
	nvals._size = _size;

	// Return new Valuations
	return nvals;
}

const SCValuations& Valuations::get_scvaluations(const Handle& var) const
{
	for (const SCValuations& scv : scvs)
		if (scv.variables.is_in_varset(var))
			return scv;
	throw RuntimeException(TRACE_INFO, "There's likely a bug");
}

unsigned Valuations::size() const
{
	return _size;
}

void Valuations::setup_size()
{
	_size = scvs.empty() ? 0 : 1;
	for (const SCValuations& scv : scvs)
		_size *= scv.size();
}

std::string oc_to_string(const SCValuations& scv, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "variables:" << std::endl
	   << oc_to_string(scv.variables, indent + OC_TO_STRING_INDENT);
	ss << indent << "valuations:" << std::endl
	   << oc_to_string(scv.valuations, indent + OC_TO_STRING_INDENT);
	return ss.str();
}

std::string oc_to_string(const SCValuations& scv)
{
	return oc_to_string(scv, "");
}

std::string oc_to_string(const SCValuationsSet& scvs, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << scvs.size() << std::endl;
	int i = 0;
	for (const auto& scv : scvs)
	{
		ss << indent << "scvaluations [" << i << "]:" << std::endl
		   << oc_to_string(scv, indent + OC_TO_STRING_INDENT);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const SCValuationsSet& scvs)
{
	return oc_to_string(scvs, "");
}

std::string oc_to_string(const Valuations& valuations, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << valuations.size() << std::endl;
	ss << indent << "variables:" << std::endl
	   << oc_to_string(valuations.variables, indent + OC_TO_STRING_INDENT);
	ss << indent << "scvaluations set:" << std::endl
	   << oc_to_string(valuations.scvs, indent + OC_TO_STRING_INDENT);
	return ss.str();
}

std::string oc_to_string(const Valuations& valuations)
{
	return oc_to_string(valuations, "");
}

std::string oc_to_string(const HandleValuationsMap& h2vals, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << h2vals.size() << std::endl;
	int i = 0;
	for (const auto& hv : h2vals)
	{
		ss << indent << "atom [" << i << "]:" << std::endl
		   << oc_to_string(hv.first, indent + OC_TO_STRING_INDENT);
		ss << indent << "valuations [" << i << "]:" << std::endl
		   << oc_to_string(hv.second, indent + OC_TO_STRING_INDENT);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const HandleValuationsMap& h2vals)
{
	return oc_to_string(h2vals, "");
}

} // namespace opencog
