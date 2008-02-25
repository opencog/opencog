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

	public:
		void match(Handle, const std::vector<Handle> &);

};
};

