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

namespace opencog
{

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
	 * Generate all patterns specializing the given pattern from the
	 * given distance, put the results in XPatternMiner::patterns. If
	 * the distance is -1, then it will generate all patterns up to
	 * maxpats above or equal to minsup.
	 */
	void specialize(const Handle& pattern, int dst=-1);

	/**
	 * Experimental specialization. Given a pattern and a collection
	 * to text atoms, generate all specialized patterns of the given
	 * pattern.
	 *
	 * For now all limits, expect minsup, are ignored, all patterns
	 * are considered.
	 */
	HandleSet xspecialize(const Handle& pattern, const HandleSet& texts) const;

	/**
	 * Calculate the frequency of a given pattern
	 */
	unsigned int freq(const Handle& pattern) const;

	// AtomSpace containing the text trees to mine.
	AtomSpace& text_as;

	// Parameters
	const XPMParameters& param;

	// Working atomspace containing the patterns, and other junk.
	AtomSpace pattern_as;

	// Set of pattern discovered so far
	HandleSet patterns;
private:
	/**
	 * Given a pattern, return its vardecl and body
	 */
	const Variables& get_variables(const Handle& pattern) const;
	const Handle& get_vardecl(const Handle& pattern) const;
	const Handle& get_body(const Handle& pattern) const;

	/**
	 * Insert a set of new patterns to the set of discovered patterns
	 */
	void insert(const HandleSet& npats);

	/**
	 * Generate all specializations of the given pattern at distance 1.
	 */
	HandleSet next_specialize(const Handle& pattern) const;

	/**
	 * Given a collection of text atoms, return all shallow
	 * patterns. For instance
	 *
	 * texts = { (Concept "a"),
	 *           (Concept "b"),
	 *           (Inheritance (Concept "a") (Concept "b")) }
	 *
	 * shallow_patterns(texts) =
	 *     { (Concept "a"),
	 *       (Concept "b"),
	 *       (Lambda
	 *         (VariableList
	 *           (Variable "$X")
	 *           (Variable "$Y"))
	 *         (Inheritance
	 *           (Variable "$X")
	 *           (Variable "$Y"))) }
	 *
	 * TODO: we may want to support types in variable declaration.
	 */
	HandleSet shallow_patterns(const HandleSet& texts) const;

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
	 * sub-patterns, return as many patterns as variable substitutions
	 * by the associated patterns. For instance
	 *
	 * pattern = (Inheritance X Y)
	 *
	 * var2patterns =
	 *   { X->(Concept "a"), Y->(Concept "b"),
	 *     X->(And (Concept "a") (Concept "c")), Y->(Concept "d") }
	 *
	 * then substitution_product(pattern, var2patterns) =
	 *   { (Inheritance X (Concept "b")),
	 *     (Inheritance X (Concept "d")),
	 *     (Inheritance (Concept "a") Y),
	 *     (Inheritance (Concept "a") (Concept "b")),
	 *     (Inheritance (Concept "a") (Concept "d")),
	 *     (Inheritance (And (Concept "a") (Concept "c")) Y),
	 *     (Inheritance (And (Concept "a") (Concept "c")) (Concept "b")),
	 *     (Inheritance (And (Concept "a") (Concept "c")) (Concept "d")) }
	 *
	 * TODO: add this as unit test.
	 */
	HandleSet substitution_product(const Handle& pattern,
	                               const HandleMultimap& var2patterns) const;
};

} // namespace opencog

#endif /* OPENCOG_XPATTERNMINER_H_ */
