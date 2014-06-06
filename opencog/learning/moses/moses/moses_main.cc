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

#include <opencog/util/Logger.h>

#include "../metapopulation/metapopulation.h"
#include "distributed_moses.h"
#include "local_moses.h"
#include "mpi_moses.h"

#include "moses_main.h"

namespace opencog { namespace moses {

#define strform(x) #x
#define stringify(x) strform(x)

#ifdef MOSES_GIT_DESCRIBE
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH) " (git-describe "
    stringify(MOSES_GIT_DESCRIBE) ")";

#else
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH);

#endif

void run_moses(metapopulation& metapop,
               deme_expander& dex,
               const moses_parameters& moses_params,
               moses_statistics& stats)
{
    // Run moses, either on localhost, or distributed.
    if (moses_params.local)
        local_moses(metapop, dex, moses_params, stats);
    else if (moses_params.mpi)
        mpi_moses(metapop, dex, moses_params, stats);
    else
        distributed_moses(metapop, dex, moses_params, stats);
}


} // ~namespace moses
} // ~namespace opencog

