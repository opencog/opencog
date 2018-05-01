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
	 * Return true iff the valuation contains no variable (thus is
	 * empty).
	 */
	bool novar() const;

	/**
	 * Return the front variable.
	 */
	Handle front_variable() const;

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
	 * Erase the front variable with all its corresponding values on a
	 * copy of this valuations.
	 *
	 * TODO: instead of erasing the front it would be better to move
	 * some pointer to the next variable.
	 */
	SCValuations erase_front() const;

	/**
	 * Erase the given variable, if exists, with all its corresponding
	 * values on a copy of this valuations.
	 */
	SCValuations erase(const Handle& var) const;

	/**
	 * Return all counted values corresponding to var.
	 */
	HandleUCounter values(const Handle& var) const;
	HandleUCounter values(unsigned var_idx) const;

	/**
	 * Compare if 2 SCValuations are equal, in fact only looking at
	 * their variables, as it is enough it the context in which they
	 * will be used.
	 */
	bool operator==(const SCValuations& other) const;

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
	 * Erase the front variable with all its corresponding associated
	 * values on a copy of this valuations.
	 */
	Valuations erase_front() const;

	/**
	 * Get the SCValuations containing the given variable.
	 */
	const SCValuations& get_scvaluations(const Handle& var) const;

	/**
	 * Return the size of the Valuations, that is its totally number
	 * of values accounting for the potential combinations of values
	 * between the strongly connected valuations.
	 */
	unsigned size() const;

	SCValuationsSet scvs;

private:
	/**
	 * Calculate and set _size
	 */
	void setup_size();

	unsigned _size;
};

typedef std::map<Handle, Valuations> HandleValuationsMap;

std::string oc_to_string(const SCValuations& scvaluations, const std::string& indent);
std::string oc_to_string(const SCValuations& scvaluations);
std::string oc_to_string(const SCValuationsSet& scvs, const std::string& indent);
std::string oc_to_string(const SCValuationsSet& scvs);
std::string oc_to_string(const Valuations& valuations, const std::string& indent);
std::string oc_to_string(const Valuations& valuations);
std::string oc_to_string(const HandleValuationsMap& h2vals, const std::string& indent);
std::string oc_to_string(const HandleValuationsMap& h2vals);

} // ~namespace opencog

#endif /* OPENCOG_VALUATIONS_H_ */
