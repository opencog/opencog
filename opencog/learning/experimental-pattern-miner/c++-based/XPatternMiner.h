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
	 * CTor. Note that ngram will be overwritten by initpat, if
	 * provided.
	 */
	XPMParameters(unsigned minsup=1,
	              unsigned ngram=1,
	              const Handle& initpat=Handle::UNDEFINED,
	              int maxpats=-1);

	// Minimum support. Mined patterns must have a frequency equal or
	// above this value.
	unsigned minsup;

	// Initial gram. This value is overwritten by the actual gram
	// value of initpat, if provided.
	unsigned initgram;

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
	HandleSet specialize(const Handle& pattern, const HandleSet& texts,
	                     unsigned mingram=0);

	/**
	 * Like above but maps each text to a count. That is useful to
	 * keep track of the frequence of sub-patterns.
	 */
	HandleSet specialize(const Handle& pattern, const HandleUCounter& texts,
	                     unsigned mingram=0);

	/**
	 * Like above on assume the pattern is trivial with a mere
	 * variable. It assumes is has enough support.
	 */
	HandleSet specialize_varpat(const Handle& varpat,
	                            const HandleUCounter& texts);

	// AtomSpace containing the text trees to mine.
	AtomSpace& text_as;

	// Parameters
	XPMParameters param;

	// Working atomspace containing the patterns, and other junk.
	AtomSpace pattern_as;

	// Set of pattern discovered so far
	HandleSet patterns;
private:

	mutable AtomSpace tmp_as;

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
	 * Calculate if the pattern has enough support w.r.t. to the given
	 * texts, that is whether its frequency is greater than or equal
	 * to minsup.
	 */
	bool enough_support(const Handle& pattern,
	                    const HandleUCounter& texts) const;

	/**
	 * Like above but only look at the text total count. This works
	 * when the text has been filtered from a 1-gram pattern.
	 */
	bool enough_support(const HandleUCounter& texts) const;

	// TODO move all static methods down

	/**
	 * Given a pattern and a text corpus, calculate the pattern
	 * frequency, that is the number of matches. Note that is number
	 * may be greater than the total count of the text corpus if the
	 * pattern is an n-gram for any n > 1.
	 */
	unsigned freq(const Handle& pattern, const HandleUCounter& texts) const;

	/**
	 * Calculate the total count of texts.
	 */
	unsigned freq(const HandleUCounter& texts) const;

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
	 * Given a pattern and texts, return the satisfying set of the
	 * pattern over the text. Please note that the texts count are
	 * ignored. But this is still useful for n-gram patterns where the
	 * counts are all in 1 anyway.
	 */
	Handle restrict_satisfying_set(const Handle& pattern,
	                               const HandleUCounter& texts) const;

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
	 * Wrap a LocalQuote link around h, if it is a link of type
	 * AndLink. That is in order not to produce n-gram patterns when
	 * in fact we want to match an AndLink text.
	 */
	Handle local_quote_if_and(const Handle& h);

	/**
	 * Given a pattern, a collection of text atoms to match, build a
	 * set of mappings from the variables in that pattern to their
	 * values (subtexts + counts).
	 */
	HandleUCounterMap gen_var2subtexts(const Handle& pattern,
	                                   const HandleUCounter& texts) const;

	/**
	 * Given variables and a sequence of list of values, map each
	 * variable to counted values (see gen_var2val).
	 */
	HandleUCounterMap gen_var2vals(const Variables& vars,
	                               const HandleSeq& values_seq) const;

	/**
	 * Given variables and a list of values, as (List v1 ... vn), map
	 * each variable to its value. If there is only one variable, then
	 * `values` contain a single value not wrapped in a List.
	 */
	HandleMap gen_var2val(const Variables& variables, const Handle& values) const;

	/**
	 * Return true iff the pattern is totally abstract like
	 *
	 * (Lambda
	 *   (Variable "$X")
	 *   (Variable "$X"))
	 *
	 * for a 1-gram. Or
	 *
	 * (Lambda
	 *   (List
	 *     (Variable "$X")
	 *     (Variable "$Y"))
	 *   (And
	 *     (Variable "$X")
	 *     (Variable "$Y"))
	 *
	 * for a 2-gram, etc.
	 */
	bool totally_abstract(const Handle& pattern) const;

	/**
	 * Given a pattern, a variable and a subpattern, build subpatterns
	 * such that the variables in the subpatterns partially overlap,
	 * in all possible ways, with the variables in the pattern
	 * different than the given variable. For instance
	 *
	 * pattern = Lambda
	 *             VariableList
	 *               Variable "$X"
	 *               Variable "$Y"
	 *               Variable "$Z"
	 *             And
	 *               Variable "$X"
	 *               Inheritance
	 *                 Variable "$Y"
	 *                 Variable "$Z"
	 *
	 * variable = Variable "$X"
	 *
	 * subpattern = Lambda
	 *                VariableList
	 *                  Variable "$U"
	 *                  Variable "$V"
	 *                Inheritance
	 *                  Variable "$U"
	 *                  Variable "$V"
	 *
	 * all possible variable overlaps are
	 *
	 * 1. no overlap   # TODO: compare with test (-> *compilation*)
	 * 2. $V = $Y
	 * 3. $V = $Z
	 * 4. $U = $Y
	 * 5. $U = $Y, $V = $Y
	 * 6. $U = $Y, $V = $Z
	 * 7. $U = $Z
	 * 8. $U = $Z, $V = $Y
	 * 9. $U = $Z, $V = $Z
	 *
	 * thus
	 *
	 * gen_var_overlap_subpatterns(pattern, variable, subpattern)
	 * =
	 * { Lambda [no overlap]
	 *     VariableList
	 *       Variable "$U"
	 *       Variable "$V"
	 *     Inheritance
	 *       Variable "$U"
	 *       Variable "$V"
	 * , Lambda [$V = $Y]
	 *     VariableList
	 *       Variable "$U"
	 *       Variable "$Y"
	 *     Inheritance
	 *       Variable "$U"
	 *       Variable "$Y"
	 * , Lambda [$V = $Z]
	 *     VariableList
	 *       Variable "$U"
	 *       Variable "$Z"
	 *     Inheritance
	 *       Variable "$U"
	 *       Variable "$Z"
	 * , Lambda [$U = $Y]
	 *     VariableList
	 *       Variable "$Y"
	 *       Variable "$V"
	 *     Inheritance
	 *       Variable "$Y"
	 *       Variable "$V"
	 * , Lambda [$U = $Y, $V = $Y]
	 *     Variable "$Y"
	 *     Inheritance
	 *       Variable "$Y"
	 *       Variable "$Y"
	 * , Lambda [$U = $Y, $V = $Z]
	 *     VariableList
	 *       Variable "$Y"
	 *       Variable "$Z"
	 *     Inheritance
	 *       Variable "$Y"
	 *       Variable "$Z"
	 * , Lambda [$U = $Z]
	 *     VariableList
	 *       Variable "$Z"
	 *       Variable "$V"
	 *     Inheritance
	 *       Variable "$Z"
	 *       Variable "$V"
	 * , Lambda [$U = $Z, $V = $Y]
	 *     VariableList
	 *       Variable "$Z"
	 *       Variable "$Y"
	 *     Inheritance
	 *       Variable "$Z"
	 *       Variable "$Y"
	 * , Lambda [$U = $Z, $V = $Z]
	 *     Variable "$Z"
	 *     Inheritance
	 *       Variable "$Z"
	 *       Variable "$Z" }
	 *
	 * The output container is a HandleSeq to be able to contain
	 * alpha-equivalent subpatterns.
	 */
	HandleSeq gen_var_overlap_subpatterns(const Handle& pattern,
	                                      const Handle& variable,
	                                      const Handle& subpattern) const;

	/**
	 * Given 2 list of disjoint variables return all mappings between
	 * the first list to the second. For instance
	 *
	 * vs1 = {$U, $V}, vs2 = {$X, $Y}
	 *
	 * gen_var_overlaps(vs1, vs2) = { {},
	 *                                {$V->$X},
	 *                                {$V->$Y},
	 *                                {$U->$X},
	 *                                {$U->$X, $V->$X},
	 *                                {$U->$X, $V->$Y},
	 *                                {$U->$Y},
	 *                                {$U->$Y, $V->$X},
	 *                                {$U->$Y, $V->$Y},
	 */
	HandleMapSet gen_var_overlaps(const HandleSet& vs1,
	                              const HandleSet& vs2) const;

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
	 * TODO: I probably need to pass a HandleUCounterMap to keep the
	 * texts associated to each subpattern associated to each variable.
	 */
	HandleSet product_compose(const Handle& pattern,
	                          const HandleUCounter& texts,
	                          const HandleMultimap& var2patterns,
	                          unsigned mingram=0) const;

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
	 * Return the number of grams in a pattern. That is, if the
	 * pattern body is an AndLink, then returns its arity, otherwise
	 * if the body is not an AndLink, then return 1, and if it's not a
	 * pattern at all (i.e. not a LambdaLink), then return 0.
	 */
	static unsigned gram(const Handle& pattern);

	/**
	 * Remove useless clauses from a body pattern. Useless clauses are
	 * constant clauses, as well as variables that already occur
	 * within an existing clause.
	 */
	static Handle remove_useless_clauses(const Handle& vardecl,
	                                     const Handle& body);
};

std::string oc_to_string(const HandleUCounterMap& hucp);

} // namespace opencog

#endif /* OPENCOG_XPATTERNMINER_H_ */
