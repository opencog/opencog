/*
 * DefaultPatternMatchCB.h
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
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
 *
 * Created by Linas Vepstas January 2009
 */

#ifndef _OPENCOG_DEFAULT_PATTERN_MATCH_H
#define _OPENCOG_DEFAULT_PATTERN_MATCH_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/execution/EvaluationLink.h>
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/query/PatternMatchEngine.h>

namespace opencog {

/**
 * Callback mixin class, used to provide a default node and link
 * matching behaviour. This class is still a pure virtual class,
 * since it does not implement the solution method.
 *
 * The *only* thing it provides is node and link matching; it does
 * not consider any truth values in establishing a match.
 */
class DefaultPatternMatchCB : public virtual PatternMatchCallback
{
	public:
		DefaultPatternMatchCB(AtomSpace* as) :
			_type_restrictions(NULL),
			_dynamic(NULL),
			_as(as)
		{}

		/**
		 * Called when a node in the template pattern needs to
		 * be compared to a possibly matching node in the atomspace.
		 * The first argument is a node from the pattern, and the
		 * second is a possible solution (grounding) node from the
		 * atomspace.
		 *
		 * Return true if the nodes match, else return false.
		 * By default, the nodes must be identical.
		 */
		virtual bool node_match(const Handle& npat_h, const Handle& nsoln_h)
		{
			// If equality, then a match.
			return npat_h == nsoln_h;
		}

		/**
		 * Called when a variable in the template pattern
		 * needs to be compared to a possible grounding
		 * node in the atomspace. The first argument
		 * is a variable from the pattern, and the second
		 * is a possible grounding node from the atomspace.
		 * Return true if the nodes match, else return false.
		 */
		virtual bool variable_match(const Handle& npat_h, const Handle& nsoln_h)
		{
			Type pattype = npat_h->getType();

			// If the ungrounded term is not of type VariableNode, then just
			// accept the match. This allows any kind of node types to be
			// explicitly bound as variables.  However, the type VariableNode
			// gets special handling, below.
			if (pattype != VARIABLE_NODE) return true;

			// If the ungrounded term is a variable, then see if there
			// are any restrictions on the variable type.
			// If no restrictions, we are good to go.
			if (NULL == _type_restrictions) return true;

			// If we are here, there's a restriction on the grounding type.
			// Validate the node type, if needed.
			VariableTypeMap::const_iterator it = _type_restrictions->find(npat_h);
			if (it == _type_restrictions->end()) return true;

			// Is the ground-atom type in our list of allowed types?
			Type soltype = nsoln_h->getType();
			const std::set<Type> &tset = it->second;
			std::set<Type>::const_iterator allow = tset.find(soltype);
			return allow != tset.end();
		}

		/**
		 * Called when a link in the template pattern
		 * needs to be compared to a possibly matching
		 * link in the atomspace. The first argument
		 * is a link from the pattern, and the second
		 * is a possible solution link from the atomspace.
		 * Return true if the links should be compared,
		 * else return false.
		 *
		 * By default, the search continues if the link
		 * arity and the link types match.
		 */
		virtual bool link_match(const LinkPtr& lpat, const LinkPtr& lsoln)
		{
			// If the pattern is exactly the same link as the proposed
			// grounding, then its a perfect match. 
			if (lpat == lsoln) return true;

			// Accept all OrLink's by default! We will get another shot
			// at it when the contents of the OrLink are examined.
			Type pattype = lpat->getType();
			if (OR_LINK == pattype) return true;

			if (lpat->getArity() != lsoln->getArity()) return false;
			Type soltype = lsoln->getType();

			// If types differ, no match
			return pattype == soltype;
		}

		/**
		 * Called when a virtual link is encountered. Returns false
		 * to reject the match.
		 */
		virtual bool virtual_link_match(const Handle& pat, const Handle& args);

		/**
		 * Called to perform the actual search. This makes some default
		 * assumptions about the kind of things that might be matched,
		 * in order to drive a reasonably-fast search.
		 */
		virtual bool initiate_search(PatternMatchEngine *,
		                             const std::set<Handle>& vars,
		                             const HandleSeq& clauses);

		/**
		 * Indicate a set of restrictions on the types of the ground atoms.
		 * The typemap contains a map from variables to a set of types
		 * that the groundings for the variable are allowed to have.
		 */
		virtual void set_type_restrictions(const VariableTypeMap &tm)
		{
			_type_restrictions = &tm;
		}

		/**
		 * Indicate the dynamically-evaluatable terms. Searchs cannot
		 * be started with these, as groundings probably won't exist
		 * in the  atomspace.
		 */
		virtual void set_evaluatable_terms(const std::set<Handle>& terms)
		{
			_dynamic = &terms;
		}
	protected:
		Handle _root;
		Handle _starter_term;
		const VariableTypeMap* _type_restrictions;
		const std::set<Handle>* _dynamic;

		virtual Handle find_starter(const Handle&, size_t&, Handle&, size_t&);
		virtual Handle find_thinnest(const HandleSeq&, Handle&, size_t&);
		virtual void find_rarest(const Handle&, Handle&, size_t&);

		bool _search_fail;
		virtual bool neighbor_search(PatternMatchEngine *,
		                             const std::set<Handle>& vars,
		                             const HandleSeq& clauses);

		virtual bool disjunct_search(PatternMatchEngine *,
		                             const std::set<Handle>& vars,
		                             const HandleSeq& clauses);

		virtual bool link_type_search(PatternMatchEngine *,
		                             const std::set<Handle>& vars,
		                             const HandleSeq& clauses);

		virtual bool variable_search(PatternMatchEngine *,
		                             const std::set<Handle>& vars,
		                             const HandleSeq& clauses);

		AtomSpace *_as;
};

} // namespace opencog

#endif // _OPENCOG_DEFAULT_PATTERN_MATCH_H
