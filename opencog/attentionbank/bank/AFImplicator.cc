/*
 * AFImplicator.cc
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

#include <opencog/atoms/pattern/BindLink.h>
#include "AFImplicator.h"

using namespace opencog;

// XXX FIXME -- do we need this function, at all?  Why isn't it
// sufficient to just do a normal pattern search, and weed out
// the atttention focus after the fact? I find it very hard to
// beleive that this provides any significant performance kick
// over a simpler, more modular design.

namespace opencog
{

/**
 * Attentional Focus specific PatternMatchCallback implementation
 */
Handle af_bindlink(AtomSpace* as, const Handle& hbindlink)
{
	BindLinkPtr bl(BindLinkCast(hbindlink));

	// Now perform the search.
	AFImplicator impl(as);
	impl.implicand = bl->get_implicand();
	bl->imply(impl, false);

	// The result_list contains a list of the grounded expressions.
	// (The order of the list has no significance, so it's really a set.)
	// Put the set into a SetLink, cache it, and return that.
	Handle rewr(createLink(impl.get_result_list(), SET_LINK));
	return as->add_atom(rewr);
}

}

/* ===================== END OF FILE ===================== */
