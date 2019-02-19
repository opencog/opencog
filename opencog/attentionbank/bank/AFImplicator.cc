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

#include "AFImplicator.h"
#include <opencog/query/DefaultImplicator.h>

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
	// Now perform the search.
	AFImplicator impl(as);
	return do_imply(as, hbindlink, impl, false);
}

}

/* ===================== END OF FILE ===================== */
