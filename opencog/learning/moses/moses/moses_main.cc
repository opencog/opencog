/** moses_main.cc ---
 *
 * Copyright (C) 2012 Pouling Holdings
 *
 * Author: Linas Vepstas <linasvepstas@gmailcom>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "moses_main.h"

namespace opencog { namespace moses {

#define str(x) #x
#define stringify(x) str(x)

#ifdef MOSES_BZR_REVNO
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH) " (revno "
    stringify(MOSES_BZR_REVNO) ")";

#else
const char * version_string =
    stringify(#MOSES_VERSION_MAJOR) "."
    stringify(#MOSES_VERSION_MINOR) "."
    stringify(#MOSES_VERSION_PATCH);

#endif

} // ~namespace moses
} // ~namespace opencog

