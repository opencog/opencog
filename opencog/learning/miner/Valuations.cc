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

#include "MinerUtils.h"

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

ValuationsBase::ValuationsBase(const Variables& vars)
	: variables(vars), _var_idx(0) {}

ValuationsBase::ValuationsBase() {}

bool ValuationsBase::no_focus() const
{
	return variables.varset.size() <= _var_idx;
}

const Handle& ValuationsBase::focus_variable() const
{
	return variables.varseq[_var_idx];
}

unsigned ValuationsBase::focus_index() const
{
	return _var_idx;
}

HandleSeq ValuationsBase::remaining_variables() const
{
	return HandleSeq(std::next(variables.varseq.begin(), _var_idx + 1),
	                 variables.varseq.end());
}

void ValuationsBase::inc_focus_variable() const
{
	_var_idx++;
}

void ValuationsBase::dec_focus_variable() const
{
	OC_ASSERT(0 < _var_idx);
	_var_idx--;
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

const Handle& SCValuations::focus_value(const HandleSeq& values) const
{
	return values[_var_idx];
}

bool SCValuations::operator<(const SCValuations& other) const
{
	return variables < other.variables;
}

unsigned SCValuations::size() const
{
	return valuations.size();
}

std::string SCValuations::to_string(const std::string& indent) const
{
	std::stringstream ss;
	ss << indent << "variables:" << std::endl
	   << oc_to_string(variables, indent + OC_TO_STRING_INDENT)
		<< indent << "valuations:" << std::endl
	   << oc_to_string(valuations, indent + OC_TO_STRING_INDENT)
	   << indent << "_var_idx = " << _var_idx;
	return ss.str();
}

////////////////
// Valuations //
////////////////

Valuations::Valuations(const Handle& pattern, const HandleSet& texts)
	: ValuationsBase(MinerUtils::get_variables(pattern))
{
	// Useless clauses (like redundant, constants, and more) are
	// removed in order to simplify subsequent processing, and avoid
	// warnings from the pattern matcher
	Handle reduced_pattern = MinerUtils::remove_useless_clauses(pattern);
	for (const Handle& cp : MinerUtils::get_component_patterns(reduced_pattern))
	{
		Handle satset = MinerUtils::restricted_satisfying_set(cp, texts, texts.size());
		scvs.insert(SCValuations(MinerUtils::get_variables(cp), satset));
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

// TODO: maybe speed this up by sorting the SCValuation in order of variables
const SCValuations& Valuations::get_scvaluations(const Handle& var) const
{
	for (const SCValuations& scv : scvs)
		if (scv.variables.is_in_varset(var))
			return scv;
	throw RuntimeException(TRACE_INFO, "There's likely a bug");
}

const SCValuations& Valuations::focus_scvaluations() const
{
	return get_scvaluations(focus_variable());
}

void Valuations::inc_focus_variable() const
{
	focus_scvaluations().inc_focus_variable();
	ValuationsBase::inc_focus_variable();
}

void Valuations::dec_focus_variable() const
{
	ValuationsBase::dec_focus_variable();
	focus_scvaluations().dec_focus_variable();
}

unsigned Valuations::size() const
{
	return _size;
}

std::string Valuations::to_string(const std::string& indent) const
{
	std::stringstream ss;
	ss << indent << "size = " << size() << std::endl
		<< indent << "variables:" << std::endl
	   << oc_to_string(variables, indent + OC_TO_STRING_INDENT)
	   << indent << "scvaluations set:" << std::endl
	   << oc_to_string(scvs, indent + OC_TO_STRING_INDENT) << std::endl
	   << indent << "_var_idx = " << _var_idx;
	return ss.str();
}

void Valuations::setup_size()
{
	_size = scvs.empty() ? 0 : 1;
	for (const SCValuations& scv : scvs)
		_size *= scv.size();
}

std::string oc_to_string(const SCValuations& scv, const std::string& indent)
{
	return scv.to_string(indent);
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

std::string oc_to_string(const Valuations& valuations, const std::string& indent)
{
	return valuations.to_string(indent);
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

} // namespace opencog
