/*
 * PatternMatch.h
 *
 * Linas Vepstas February 2008
 */

#include "types.h"

namespace opencog {

class PatternMatch
{
	private:
		AtomSpace *atom_space;

		bool prt(Atom *);

		// Apply Filter rules, to create a normalized predicate.
		std::vector<Handle> normed_predicate;
		std::vector<Handle> bound_vars;
		bool apply_rule(Atom *);
		bool is_ling_rel(Atom *);

		// Examine evaluation links.
		bool do_eval_link(Atom *);

	public:
		PatternMatch(AtomSpace *);

		void filter(Handle, const std::vector<Handle> &);
		void match(void);

};
};

