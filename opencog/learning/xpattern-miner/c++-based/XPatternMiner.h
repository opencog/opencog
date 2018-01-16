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
#include <opencog/atoms/core/RewriteLink.h>
#include <opencog/atomspace/AtomSpace.h>

#include "HandleTree.h"
#include "Valuations.h"

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
	 * CTor. Note that conjuncts will be overwritten by initpat, if
	 * provided.
	 */
	XPMParameters(unsigned minsup=1,
	              unsigned conjuncts=1,
	              const Handle& initpat=Handle::UNDEFINED,
	              int maxdepth=-1,
	              int maxpats=-1);

	// Minimum support. Mined patterns must have a frequency equal or
	// above this value.
	unsigned minsup;

	// Initial number of conjuncts. This value is overwritten by the
	// actual number of conjuncts of initpat, if provided.
	unsigned initconjuncts;

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

	// Maximum depth from pattern to output. If negative, then no
	// depth limit. Depth is the number of specializations between the
	// initial pattern and the produced patterns.
	int maxdepth;

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
	HandleTree operator()();

	/**
	 * Specialization. Given a pattern and a collection to text atoms,
	 * generate all specialized patterns of the given pattern.
	 *
	 * For now all limits, expect minsup, are ignored, all patterns
	 * are considered.
	 */
	HandleTree specialize(const Handle& pattern, const HandleSet& texts,
	                      int maxdepth=-1);

	/**
	 * Alternate implementation using valuations
	 */
	HandleTree specialize(const Handle& pattern,
	                      // TODO: can probably replace
	                      // HandleUCounter by HandleSet
	                      const HandleUCounter& texts,
	                      const Valuations& valuations,
	                      int maxdepth);

	/**
	 * Helper, automatically turns texts into valuations
	 */
	HandleTree specialize(const Handle& pattern,
	                      const HandleUCounter& texts,
	                      int maxdepth);

	// AtomSpace containing the text trees to mine.
	AtomSpace& text_as;

	// Parameters
	XPMParameters param;

	// Working atomspace containing the patterns, and other junk.
	AtomSpace pattern_as;

private:

	mutable AtomSpace tmp_as;

	/**
	 * Return true iff maxdepth is null or pattern is not a lambda or
	 * doesn't have enough support. Additionally the second one check
	 * whether the valuation has any variable left to specialize from.
	 */
	bool terminate(const Handle& pattern,
	               const HandleUCounter& texts,
	               int maxdepth) const;
	bool terminate(const Handle& pattern,
	               const HandleUCounter& texts,
	               const Valuations& valuations,
	               int maxdepth) const;
	
	/**
	 * Given a pattern, a variable of that pattern, create another
	 * pattern that is just that variable. For instance
	 *
	 * pattern = (Lambda
	 *             (VariableList
	 *               (TypedVariable
	 *                 (Variable "$X")
	 *                 (Type "ConceptNode"))
	 *               (TypedVariable
	 *                 (Variable "$Y")
	 *                 (Type "AndLink"))
	 *             (Inheritance
	 *               (Variable "$X")
	 *               (Variable "$Y")))
	 *
	 * var = (Variable "$X")
	 *
	 * mk_varpattern(pattern, var) = (Lambda
	 *                                 (TypeVariable
	 *                                   (Variable "$X")
	 *                                   (Type "ConceptNode"))
	 *                                 (Variable "$X"))
	 */
	Handle mk_varpattern(const Handle& pattern, const Handle& var) const;

	/**
	 * Specialize according to shallow abstractions.
	 * 
	 * TODO: improve comment.
	 */
	HandleTree specialize_shabs(const Handle& pattern,
	                            const HandleUCounter& texts,
	                            const Valuations& valuations,
	                            int maxdepth);

	/**
	 * Specialize by reducing variables, that is mapping non-front
	 * variables to the front one, when possible
	 *
	 * TODO: improve comment.
	 */
	HandleTree specialize_vared(const Handle& pattern,
	                            const HandleUCounter& texts,
	                            const Valuations& valuations,
	                            int maxdepth);

	/**
	 * Calculate if the pattern has enough support w.r.t. to the given
	 * texts, that is whether its frequency is greater than or equal
	 * to minsup.
	 */
	bool enough_support(const Handle& pattern,
	                    const HandleUCounter& texts) const;

	/**
	 * Like above but only look at the text total count. This works
	 * when the text has been filtered from a single conjunct pattern.
	 */
	bool enough_support(const HandleUCounter& texts) const;

	// TODO move all static methods down

	/**
	 * Given a pattern and a text corpus, calculate the pattern
	 * frequency, that is the number of matches. Note that this number
	 * may be greater than the total count of the text corpus if the
	 * pattern has more than one conjunct.
	 *
	 * maxf is used to halt the frequency calculation if it reaches a
	 * certain maximum, for saving resources.
	 */
	unsigned freq(const Handle& pattern,
	              const HandleUCounter& texts,
	              int maxf=-1) const;

	/**
	 * Calculate the total count of texts.
	 *
	 * maxf is used to halt the frequency calculation if it reaches a
	 * certain maximum. Purely for saving resources when possible.
	 */
	unsigned freq(const HandleUCounter& texts, int maxf=-1) const;

	/**
	 * Filter in only texts matching the pattern
	 */
	HandleUCounter filter_texts(const Handle& pattern,
	                            const HandleUCounter& texts) const;

	/**
	 * Check whether a pattern matches a text.
	 */
	bool match(const Handle& pattern, const Handle& text) const;

	/**
	 * Like above but returns the Set of Lists of values associated to
	 * the variables of the pattern. Assumes that pattern is always a
	 * LambdaLink, and not a constant.
	 *
	 * TODO: optimize
	 */
	Handle matched_results(const Handle& pattern, const Handle& text) const;

	/**
	 * TODO: add comment
	 */
	HandleValuationsMap variable_reduce(const Valuations& valuations) const;
	
	/**
	 * TODO: fix comment
	 *
	 * Given a collection of text atoms, return all shallow abstractions,
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
	Handle shallow_abstract(const Handle& text);

	/**
	 * Like above but take valuations instead of texts,
	 * produce shallow patterns based on the first variable, and
	 * associate the remaining valuations to each pattern.
	 */
	HandleValuationsMap shallow_abstract(const Valuations& valuations);
	
	/**
	 * Wrap a VariableList around a variable list, if more than one
	 * variable, otherwise return that one variable.
	 */
	Handle variable_list(const HandleSeq& vars);

	/**
	 * Wrap a LambdaLink around a vardecl and body.
	 */
	Handle lambda(const Handle& vardecl, const Handle& body);

	/**
	 * Wrap a QuoteLink around h
	 */
	Handle quote(const Handle& h);

	/**
	 * Wrap a UnquoteLink around h
	 */
	Handle unquote(const Handle& h);

	/**
	 * Wrap a LocalQuote link around h (typically used if it is a link
	 * of type AndLink. That is in order not to produce
	 * multi-conjuncts patterns when in fact we want to match an
	 * AndLink text.)
	 */
	Handle local_quote(const Handle& h);

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
	static Handle compose(const Handle& pattern, const HandleMap& var2pat);

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
	 */
	static Handle vardecl_compose(const Handle& vardecl,
	                              const HandleMap& var2subdecl);

	/**
	 * If a body is an AndLink with one argument, remove it. For instance
	 *
	 * remove_uniry_and(And (Concept "A")) = (Concept "A")
	 */
	static Handle remove_unary_and(const Handle& h);

	/**
	 * Call alpha_conversion if pattern is a scope, return itself
	 * otherwise.
	 */
	static Handle alpha_conversion(const Handle& pattern);

public:
	/**
	 * Given a pattern and texts, return the satisfying set of the
	 * pattern over the text. Please note that the texts count are
	 * ignored. But this is still useful for multi-conjuncts patterns
	 * where the counts are all 1 anyway.
	 *
	 * TODO: ignore permutations for unordered links.
	 */
	static Handle restricted_satisfying_set(const Handle& pattern,
	                                        const HandleUCounter& texts,
	                                        int maxf=-1);

	/**
	 * Return true iff the pattern is totally abstract like
	 *
	 * (Lambda
	 *   (Variable "$X")
	 *   (Variable "$X"))
	 *
	 * for a single conjunct. Or
	 *
	 * (Lambda
	 *   (List
	 *     (Variable "$X")
	 *     (Variable "$Y"))
	 *   (And
	 *     (Variable "$X")
	 *     (Variable "$Y"))
	 *
	 * for 2 conjuncts, etc.
	 */
	static bool totally_abstract(const Handle& pattern);

	/**
	 * Generate a list of hopefully unique random variables
	 */
	static HandleSeq gen_rand_variables(size_t n);
	static Handle gen_rand_variable();

	/**
	 * Given a pattern, return its vardecl and body. If the pattern is
	 * not a scope link (i.e. a constant/text), then get_variables and
	 * get_vardecl return the empty Variables and vardecl
	 * respectively, and get_body returns the pattern itself.
	 */
	static const Variables& get_variables(const Handle& pattern);
	static const Handle& get_vardecl(const Handle& pattern);
	static const Handle& get_body(const Handle& pattern);

	/**
	 * Return the number of conjuncts in a pattern. That is, if the
	 * pattern body is an AndLink, then returns its arity, otherwise
	 * if the body is not an AndLink, then return 1, and if it's not a
	 * pattern at all (i.e. not a LambdaLink), then return 0.
	 */
	static unsigned conjuncts(const Handle& pattern);

	/**
	 * Remove useless clauses from a body pattern. Useless clauses are
	 * constant clauses, as well as variables that already occur
	 * within an existing clause.
	 */
	static Handle remove_useless_clauses(const Handle& vardecl,
	                                     const Handle& body);

	/**
	 * Remove any closes clause (regardless of whether they are
	 * evaluatable or not).
	 */
	static Handle remove_constant_clauses(const Handle& vardecl,
	                                      const Handle& body);
};

std::string oc_to_string(const HandleUCounterMap& hucp);

} // ~namespace opencog

#endif /* OPENCOG_XPATTERNMINER_H_ */
