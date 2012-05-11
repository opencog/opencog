/*
 * opencog/util/copyif.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef OPENCOG_COPYIF_H
#define OPENCOG_COPYIF_H

// The newer gcc's implement std::copy_if which seems to do the same
// thing ...  I'm not sure if this starts with gcc-4.6 or earlier,
// though, so this may need tweaking.  Certainly, gcc-4.6.3 has this.
#if !((__GNUC__ >= 4) && (__GNUC_MINOR__ >= 6))

namespace opencog
{

template < typename ForwardIter,
typename OutputIter,
typename UnaryPred >
OutputIter copy_if(ForwardIter begin, ForwardIter end, OutputIter dest, UnaryPred f)
{
    while (begin != end) {
        if (f(*begin))
            *dest++ = *begin;
        ++begin;
    }
    return dest;
}

} // namespace opencog

#endif

#endif /* OPENCOG_COPYIF_H */
