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

/**
 * Class representing valuations, that is mappings from variables to
 * values.
 *
 * TODO: maybe this could be moved to Unify.h and being used in
 * Unify.cc.
 */
class Valuations
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
	Valuations(const Variables& variables, const Handle& satset=Handle::UNDEFINED);

	/**
	 * Return true iff the valuation contains no variable (thus is
	 * empty).
	 */
	bool novar() const;

	/**
	 * Return the front variable, the first one is the list.
	 */
	Handle front_variable() const;

	/**
	 * Return the variable at index i.
	 */
	Handle variable(unsigned i) const;
	
	/**
	 * Erase the front variable with all its corresponding associated
	 * values on a copy of this valuations.
	 */
	Valuations erase_front() const;

	Variables variables;
	HandleSeqSeq values;
};

typedef std::map<Handle, Valuations> HandleValuationsMap;
// typedef std::map<HandleSet, Valuations> HandleSetValuationsMap;

std::string oc_to_string(const Valuations& valuations);
std::string oc_to_string(const HandleValuationsMap& h2vals);

} // ~namespace opencog

#endif /* OPENCOG_VALUATIONS_H_ */
