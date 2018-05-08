/*
 * Miner.h
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

class MinerUTest;

namespace opencog
{

/**
 * Parameters for Miner. The terminology is taken from
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
	              double info=1.0);

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

	// Modify how the frequency of strongly connected components is
	// calculated from the frequencies of its components. Specifically
	// it will go from f1*...*fn to min(f1,...,fn), where f1 to fn are
	// the frequencies of each component. This allows to dismiss
	// abstractions that are likely not to lead to specializations
	// with enough support.
	//
	// If the parameter equals to 0 the frequency of the whole pattern
	// is calculated as the product of f1 to fn, if it equals to 1 the
	// frequency of the whole pattern is calculated as the min of f1
	// to fn. And if the value is between, it is calculated as a
	// linear combination of both.
	//
	// It is called info for mutual information or its n-ary
	// generalizations (like interaction information). What it means
	// is that when info is low subsequent specializations are likely
	// independant, while when info is high subsequent specializations
	// are likely dependant and thus we can afford to estimate the
	// frequency of the specializations by the lowest frequency of its
	// component.
	double info;
};

/**
 * Experimental pattern miner. Mined patterns should be compatible
 * with the pattern matcher, that is if feed to the pattern matcher,
 * the latter should return as many candidates as the pattern's
 * frequency.
 */
class Miner
{
    friend class ::MinerUTest;
public:

	/**
	 * CTor
	 */
	Miner(AtomSpace& as, const XPMParameters& param=XPMParameters());

	/**
	 * Mine and return a tree of patterns linked by specialization
	 * relationship (children are specializations of parent) with
	 * frequency equal to or above minsup, starting from the initial
	 * pattern, excluded.
	 */
	HandleTree operator()();

	/**
	 * Specialization. Given a pattern and a collection to text atoms,
	 * generate all specialized patterns of the given pattern.
	 */
	HandleTree specialize(const Handle& pattern, const HandleSet& texts,
	                      int maxdepth=-1);

	/**
	 * Like above, where all valid texts have been converted into
	 * valuations.
	 */
	HandleTree specialize(const Handle& pattern,
	                      const HandleSet& texts,
	                      const Valuations& valuations,
	                      int maxdepth);

	/**
	 * Alternate specialization that reflects how the URE would work.
	 */
	HandleTree specialize_alt(const Handle& pattern,
	                          const HandleSet& texts,
	                          const Valuations& valuations,
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
	               const HandleSet& texts,
	               int maxdepth) const;
	bool terminate(const Handle& pattern,
	               const HandleSet& texts,
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
	 * Specialize the given pattern according to shallow abstractions
	 * obtained by looking at the valuations of the front variable of
	 * valuations, then recursively call Miner::specialize on these
	 * obtained specializations.
	 */
	HandleTree specialize_shabs(const Handle& pattern,
	                            const HandleSet& texts,
	                            const Valuations& valuations,
	                            int maxdepth);

	/**
	 * Specialize the given pattern with the given shallow abstraction
	 * at the given variable, then call Miner::specialize on the
	 * obtained specialization.
	 */
	HandleTree specialize_shapat(const Handle& pattern, const HandleSet texts,
	                             const Handle& var, const Handle& shapat,
	                             int maxdepth);

	/**
	 * Calculate if the pattern has enough support w.r.t. to the given
	 * texts, that is whether its frequency is greater than or equal
	 * to minsup.
	 */
	bool enough_support(const Handle& pattern,
	                    const HandleSet& texts) const;

	// TODO move all static methods down

	/**
	 * Given a pattern and a text corpus, calculate the pattern
	 * frequency, that is the number of matches if pattern is strongly
	 * connected.
	 *
	 * If pattern is not strongly connected AND some heuristic is in
	 * place TODO, then the definition of frequency deviates from the
	 * usual one and corresponds to the minimum frequency over all
	 * strongly connected components of that pattern.
	 *
	 * maxf is used to halt the frequency calculation if it reaches a
	 * certain maximum, for saving resources.
	 */
	unsigned freq(const Handle& pattern,
	              const HandleSet& texts,
	              int maxf=-1) const;

	/**
	 * Like above but assumes the pattern is a single strongly
	 * connected component, as opposed to a conjuction of strongly
	 * connected components.
	 */
	unsigned freq_component(const Handle& component,
	                        const HandleSet& texts,
	                        int maxf=-1) const;

	/**
	 * Calculate the frequency of the whole pattern, given the
	 * frequency of it's components.
	 */
	unsigned freq(const std::vector<unsigned>& freqs) const;

	/**
	 * Filter in only texts matching the pattern
	 */
	HandleSet filter_texts(const Handle& pattern,
	                       const HandleSet& texts) const;

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
	 * Given valuations produce all shallow abstractions reachind
	 * minimum support, over all variables. It basically applies
	 * front_shallow_abstract recursively. See the specification of
	 * front_shallow_abstract for more information.
	 *
	 * For instance, given
	 *
	 * valuations =
	 *   { { X->(Inheritance (Concept "A") (Concept "B")), Y->(Concept "C") },
	 *     { X->(Inheritance (Concept "B") (Concept "C")), Y->(Concept "D") },
	 *     { X->(Concept "E"), Y->(Concept "D") } }
	 * ms = 2
	 *
	 * shallow_abstract(valuations) =
	 *  {
	 *    ;; Shallow abstractions of X
	 *    { (Lambda
	 *        (VariableList
	 *          (Variable "$X1")
	 *          (Variable "$X2"))
	 *        (Inheritance
	 *          (Variable "$X1")
	 *          (Variable "$X2"))) },
	 *    ;; Shallow abstractions of Y
	 *    { (Concept "D") }
	 *  }
	 */
	static HandleSetSeq shallow_abstract(const Valuations& valuations, unsigned ms);

	/**
	 * Given valuations produce all shallow abstractions reaching
	 * minimum support based on the values associated to the front
	 * variable first variable. This shallow abstractions include
	 *
	 * 1. Single operator patterns, like (Lambda X Y (Inheritance X Y))
	 * 2. Constant nodes, like (Concept "A")
	 * 3. Remain variable after the front one, x2, ..., xn
	 *
	 * Composing these 3 sorts of abstractions are enough to generate
	 * all possible patterns.
	 *
	 * For instance, given
	 *
	 * valuations =
	 *   { { X->(Inheritance (Concept "A") (Concept "B")), Y->(Concept "C") },
	 *     { X->(Inheritance (Concept "B") (Concept "C")), Y->(Concept "D") },
	 *     { X->(Concept "E"), Y->(Concept "D") } }
	 * ms = 2
	 *
	 * front_shallow_abstract(valuations) = { (Lambda
	 *                                          (VariableList
	 *                                            (Variable "$X1")
	 *                                            (Variable "$X2"))
	 *                                          (Inheritance
	 *                                            (Variable "$X1")
	 *                                            (Variable "$X2"))) }
	 */
	static HandleSet front_shallow_abstract(const Valuations& valuations, unsigned ms);

	/**
	 * Given an atom, a value, return its corresponding shallow
	 * abstraction. A shallow abstraction of an atom is
	 *
	 * 1. itself if it is a node
	 *
	 * 2. (Lambda (VariableList X1 ... Xn) (L X1 ... Xn) if it is a
	 *    link of arity n.
	 *
	 * For instance, with
	 *
	 * text = (Inheritance (Concept "a") (Concept "b"))
	 *
	 * shallow_patterns(text) = (Lambda
	 *                            (VariableList
	 *                              (Variable "$X1")
	 *                              (Variable "$X2"))
	 *                            (Inheritance
	 *                              (Variable "$X1")
	 *                              (Variable "$X2")))
	 *
	 * TODO: we may want to support types in variable declaration.
	 */
	static Handle val_shallow_abstract(const Handle& value);

	/**
	 * Wrap a VariableList around a variable list, if more than one
	 * variable, otherwise return that one variable.
	 */
	static Handle variable_list(const HandleSeq& vars);

	/**
	 * Wrap a LambdaLink around a vardecl and body.
	 */
	static Handle lambda(const Handle& vardecl, const Handle& body);

	/**
	 * Wrap a QuoteLink around h
	 */
	static Handle quote(const Handle& h);

	/**
	 * Wrap a UnquoteLink around h
	 */
	static Handle unquote(const Handle& h);

	/**
	 * Wrap a LocalQuote link around h (typically used if it is a link
	 * of type AndLink. That is in order not to produce
	 * multi-conjuncts patterns when in fact we want to match an
	 * AndLink text.)
	 */
	static Handle local_quote(const Handle& h);

	/**
	 * TODO replace by RewriteLink::beta_reduce
	 *
	 * Given a pattern, and mapping from variables to sub-patterns,
	 * compose (as in function composition) the pattern with the
	 * sub-patterns. That is replace variables in the pattern by their
	 * associated sub-patterns, properly updating the variable
	 * declaration.
	 */
	static Handle compose(const Handle& pattern, const HandleMap& var2pat);

	/**
	 * TODO replace by RewriteLink::beta_reduce
	 *
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
	 * Like shallow_abstract(const Valuations&, unsigned) but takes a pattern
	 * and a texts instead, and generate the valuations of the pattern
	 * prior to calling shallow_abstract on its valuations.
	 *
	 * See comment on shallow_abstract(const Valuations&, unsigned) for more
	 * details.
	 */
	static HandleSetSeq shallow_abstract(const Handle& pattern,
	                                     const HandleSet& texts,
	                                     unsigned ms);

	/**
	 * Given a vardecl and a body, filter the vardecl to contain only
	 * variable of the body, and create a Lambda with them.
	 */
	static Handle mk_pattern(const Handle& vardecl, const HandleSeq& clauses);

	/**
	 * Given a pattern, split it into smaller patterns of strongly
	 * connected components.
	 */
	static HandleSeq get_component_patterns(const Handle& pattern);

	/**
	 * Given a pattern, split it into its disjuncts.
	 */
	static HandleSeq get_conjuncts(const Handle& pattern);

	/**
	 * Given a pattern and texts, return the satisfying set of the
	 * pattern over the text. Please note that the texts count are
	 * ignored. But this is still useful for multi-conjuncts patterns
	 * where the counts are all 1 anyway.
	 *
	 * TODO: ignore permutations for unordered links.
	 */
	static Handle restricted_satisfying_set(const Handle& pattern,
	                                        const HandleSet& texts,
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
	static Handle get_vardecl(const Handle& pattern);
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

} // ~namespace opencog

#endif /* OPENCOG_XPATTERNMINER_H_ */
