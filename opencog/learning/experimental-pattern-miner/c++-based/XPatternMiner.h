/*
 * XPatternMiner.h
 *
 * Copyright (C) 2017 OpenCog Foundation
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
#ifndef OPENCOG_XPATTERNMINER_H_
#define OPENCOG_XPATTERNMINER_H_

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/core/Variables.h>
#include <opencog/atomspace/AtomSpace.h>

class XPatternMinerUTest;

namespace opencog
{

typedef std::map<Handle, HandleUCounter> HandleUCounterMap;

/**
 * Parameters for XPatternMiner. The terminology is taken from
 * Frequent Subtree Mining -- An Overview, from Yun Chi et al, when
 * possible.
 */
struct XPMParameters {

	/**
	 * CTor
	 */
	XPMParameters(int minsup=3,
	              const Handle& init_pattern=Handle::UNDEFINED,
	              int maxpats=-1);

	// Minimum support. Mined patterns must have a frequency equal or
	// above this value.
	unsigned minsup;

	// Initial pattern. All found patterns are specialization of
	// it. If UNDEFINED, then the initial pattern is the most abstract
	// one. i.e.
	//
	// Lambda
	//   X
	//   X
	//
	// That is the pattern matching the entire atomspace.
	Handle initpat;

	// Maximum number of patterns to output. If negative, then output
	// them all.
	int maxpats;
};

/**
 * Experimental pattern miner. Mined patterns should be compatible
 * with the pattern matcher, that is if feed to the pattern matcher,
 * the latter should return as many candidates as the pattern's
 * frequency.
 */
class XPatternMiner
{
    friend class ::XPatternMinerUTest;
public:

	/**
	 * CTor
	 */
	XPatternMiner(AtomSpace& as, const XPMParameters& param=XPMParameters());

	/**
	 * Mine and return maxpats (or all if negative) patterns with
	 * frequency equal to or above minsup, starting from the initial
	 * pattern, excluded.
	 */
	HandleSet operator()();

	/**
	 * Specialization. Given a pattern and a collection to text atoms,
	 * generate all specialized patterns of the given pattern.
	 *
	 * For now all limits, expect minsup, are ignored, all patterns
	 * are considered.
	 */
	HandleSet specialize(const Handle& pattern, const HandleSet& texts);

	/**
	 * Like above but maps each text to a count. That is useful to
	 * keep track of the frequence of sub-patterns.
	 */
	HandleSet specialize(const Handle& pattern, const HandleUCounter& texts);

	/**
	 * Calculate if pattern has enough support, that is whether its
	 * frequency is greater than or equal to minsup.
	 */
	bool enough_support(const Handle& pattern, const HandleUCounter& texts) const;

	/**
	 * Calculate the frequency of a given pattern.
	 */
	unsigned freq(const Handle& pattern, const HandleUCounter& texts) const;

	// AtomSpace containing the text trees to mine.
	AtomSpace& text_as;

	// Parameters
	XPMParameters param;

	// Working atomspace containing the patterns, and other junk.
	AtomSpace pattern_as;

	// Set of pattern discovered so far
	HandleSet patterns;
private:
	/**
	 * Given a pattern, return its vardecl and body. If the pattern is
	 * not a scope link (i.e. a constant/text), then get_variables and
	 * get_vardecl return the empty Variables and vardecl
	 * respectively, and get_body returns the pattern itself.
	 */
	const Variables& get_variables(const Handle& pattern) const;
	const Handle& get_vardecl(const Handle& pattern) const;
	const Handle& get_body(const Handle& pattern) const;

	/**
	 * Insert a set of new patterns to the set of discovered patterns
	 */
	void insert(const HandleSet& npats);

	/**
	 * Check whether a pattern matches a text.
	 *
	 * TODO: optimize
	 */
	bool match(const Handle& pattern, const Handle& text) const;

	/**
	 * Given a collection of text atoms, return all shallow patterns,
	 * associated to their matching texts. For instance
	 *
	 * texts = { (Concept "a"),
	 *           (Concept "b"),
	 *           (Inheritance (Concept "a") (Concept "b")) }
	 *
	 * shallow_patterns(texts) =
	 *     { (Concept "a") -> { (Concept "a") },
	 *       (Concept "b") -> { (Concept "b") },
	 *       (Lambda       -> { (Inheritance (Concept "a") (Concept "b")) }
	 *         (VariableList
	 *           (Variable "$X")
	 *           (Variable "$Y"))
	 *         (Inheritance
	 *           (Variable "$X")
	 *           (Variable "$Y")))
	 *     }
	 *
	 * TODO: we may want to support types in variable declaration.
	 */
	HandleUCounterMap shallow_patterns(const HandleUCounter& texts);

	/**
	 * Given a pattern, a collection of text atoms to match, build a
	 * set of mappings from the variables in that pattern to their
	 * values.
	 */
	HandleMapSet gen_var2vals(const Handle& pattern, const HandleSet& texts) const;

	/**
	 * Given a pattern and a list of values, map each variable in that
	 * pattern to its value.
	 */
	HandleMap gen_var2val(const Variables& variables, const Handle& values) const;

	/**
	 * Generate a list of hopefully unique random variables
	 */
	HandleSeq gen_rand_variables(size_t n) const;
	Handle gen_rand_variable() const;

	/**
	 * Given a collection of mappings from variable to values, and a
	 * variable, return the set of associated values.
	 */
	HandleSet select_values(const HandleMapSet& var2vals, const Handle& var) const;

	/**
	 * Given a pattern and a mapping from variables in that pattern to
	 * sub-patterns, return as many patterns as possible
	 * compositions. For instance
	 *
	 * pattern = (Inheritance X Y)
	 *
	 * var2patterns =
	 *   { X-> {(Concept "a"), (Set (Concept "a") (Concept "c"))},
	 *     Y-> {(Concept "b"), (Concept "d")} }
	 *
	 * then product_compose(pattern, var2patterns) =
	 *   { (Inheritance X (Concept "b")),
	 *     (Inheritance X (Concept "d")),
	 *     (Inheritance (Concept "a") Y),
	 *     (Inheritance (Concept "a") (Concept "b")),
	 *     (Inheritance (Concept "a") (Concept "d")),
	 *     (Inheritance (Set (Concept "a") (Concept "c")) Y),
	 *     (Inheritance (Set (Concept "a") (Concept "c")) (Concept "b")),
	 *     (Inheritance (Set (Concept "a") (Concept "c")) (Concept "d")) }
	 *
	 * TODO: add this as unit test.
	 */
	HandleSet product_compose(const Handle& pattern,
	                          const HandleMultimap& var2patterns) const;

	/**
	 * Given a pattern, and mapping from variables to sub-patterns,
	 * compose (as in function composition) the pattern with the
	 * sub-patterns. That is replace variables in the pattern by their
	 * associated sub-patterns, properly updating the variable
	 * declaration.
	 *
	 * TODO: add unit test
	 *
	 * TODO: add examples
	 *
	 * TODO Move this to a ComposeLink factory.
	 */
	Handle compose(const Handle& pattern, const HandleMap& var2pat) const;

	/**
	 * Given a variable declaration, and a mapping from variables to
	 * variable declaration, produce a new variable declaration, as
	 * obtained by compositing the pattern with the sub-patterns.
	 *
	 * If a variable in vardecl is missing in var2vardecl, then
	 * vardecl is untouched. But if a variable maps to the undefined
	 * Handle, then it is removed from the resulting variable
	 * declaration. That happens in cases where the variable maps to a
	 * constant pattern, i.e. a value. In such case composition
	 * amounts to application.
	 *
	 * TODO: add unit test
	 *
	 * TODO: add examples
	 *
	 * TODO: can probably be made static
	 */
	static Handle vardecl_compose(const Handle& vardecl,
	                              const HandleMap& var2subdecl);
};

} // namespace opencog

#endif /* OPENCOG_XPATTERNMINER_H_ */
