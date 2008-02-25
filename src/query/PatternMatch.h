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
		bool prt(Atom *);

		// apply filter rules
		std::vector<Handle> norm_outgoing;
		bool apply_rule(Atom *);

	public:
		Handle filter(Handle, const std::vector<Handle> &);
		void match(Handle, const std::vector<Handle> &);

};
};

