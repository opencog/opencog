/*
 * Valuations.h
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
#ifndef OPENCOG_VALUATIONS_H_
#define OPENCOG_VALUATIONS_H_

#include <opencog/util/empty_string.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/core/Variables.h>

namespace opencog
{

class ValuationsBase
{
public:
	ValuationsBase(const Variables& variables);
	ValuationsBase();               // TODO: just till be use dummy valuations

	/**
	 * Return true iff the valuation contains no variable to focus on
	 * (empty or _var_idx is out of range).
	 */
	bool no_focus() const;

	/**
	 * Return the variable under focus (at _var_idx)
	 */
	const Handle& focus_variable() const;

	/**
	 * Return the index of the variable under focus, _var_idx
	 */
	unsigned focus_index() const;

	/**
	 * Return all variables following (and not including) the variable under focus.
	 */
	HandleSeq remaining_variables() const;

	/**
	 * Move focus to the next (or previous) variable. That is increment
	 * (or decrement) _var_idx.
	 */
	void inc_focus_variable() const;
	void dec_focus_variable() const;

	/**
	 * Return the variable at index i.
	 */
	Handle variable(unsigned i) const;

	/**
	 * Return the index corresponding to var
	 */
	unsigned index(const Handle& var) const;

	/**
	 * Return the number of valuations
	 */
	unsigned size() const;

	Variables variables;

protected:
	// Index pointing to the current variable of focus. Useful for
	// MinerUtils::shallow_abstract recursive calls.
	mutable unsigned _var_idx;
};

/**
 * Valuations for a single strongly connected component.
 */
class SCValuations : public ValuationsBase
{
public:
	/**
	 * Given variables and a satisfying set obtained by running a
	 * satisfying_set pattern matcher query, resulting in
	 *
	 * (Set (List v11 ... v1m) ... (List vn1 ... vnm))
	 *
	 * construct the corresponding Valuations.
	 */
	SCValuations(const Variables& variables, const Handle& satset=Handle::UNDEFINED);

	/**
	 * Return all counted values corresponding to var.
	 */
	HandleUCounter values(const Handle& var) const;
	HandleUCounter values(unsigned var_idx) const;

	/**
	 * Return the value under focus (at var_idx) of a give row of
	 * values.
	 */
	const Handle& focus_value(const HandleSeq& values) const;

	/**
	 * Less than relationship according to Variables, because it's
	 * cheap and no 2 SCValuations with same variables will be in the
	 * same set at once.
	 */
	bool operator<(const SCValuations& other) const;

	/**
	 * Return the size of the SCValuations, that is its number of
	 * values.
	 */
	unsigned size() const;

	std::string to_string(const std::string& indent=empty_string) const;

	// Actual valuations, sequence of tuples of values associated to
	// each variable.
	HandleSeqSeq valuations;
};

typedef std::set<SCValuations> SCValuationsSet;

/**
 * Class representing valuations of a pattern against a text, that is
 * mappings from variables to values. Valuations is composed of
 * strongly connected valuations, as to not perform their expensive
 * Cartesian products.
 */
class Valuations : public ValuationsBase
{
public:
	/**
	 * Given a pattern and texts (ground terms), calculate its
	 * valuations.
	 */
	Valuations(const Handle& pattern, const HandleSet& texts);
	Valuations(const Variables& variables, const SCValuationsSet& scvs);
	Valuations(const Variables& variables);

	/**
	 * Get the SCValuations containing the given variable.
	 */
	const SCValuations& get_scvaluations(const Handle& var) const;

	/**
	 * Get the SCValuations containing the variable under focus
	 */
	const SCValuations& focus_scvaluations() const;

	/**
	 * Move focus to the next (or previous) variable. That is increment
	 * (or decrement) _var_idx, as well as move focus of corresponding
	 * variable of the corresponding strongly connected component.
	 */
	void inc_focus_variable() const;
	void dec_focus_variable() const;

	/**
	 * Return the size of the Valuations, that is its totally number
	 * of values accounting for the potential combinations of values
	 * between the strongly connected valuations.
	 */
	unsigned size() const;

	std::string to_string(const std::string& indent) const;

	SCValuationsSet scvs;

private:
	/**
	 * Calculate and set _size
	 */
	void setup_size();

	unsigned _size;
};

typedef std::map<Handle, Valuations> HandleValuationsMap;

std::string oc_to_string(const SCValuations& scvaluations,
                         const std::string& indent=empty_string);
std::string oc_to_string(const SCValuationsSet& scvs,
                         const std::string& indent=empty_string);
std::string oc_to_string(const Valuations& valuations,
                         const std::string& indent=empty_string);
std::string oc_to_string(const HandleValuationsMap& h2vals,
                         const std::string& indent=empty_string);

} // ~namespace opencog

#endif /* OPENCOG_VALUATIONS_H_ */
