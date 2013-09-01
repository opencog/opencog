/*
 * load-file.h
 *
 * Utility helper function -- load scheme code from a file
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef OPENCOG_SERVER_LOAD_FILE_H_
#define OPENCOG_SERVER_LOAD_FILE_H_

#include <opencog/atomspace/AtomSpace.h>

namespace opencog {
/** \addtogroup grp_smob
 *  @{
 */

#ifdef HAVE_GUILE
int load_scm_file (AtomSpace& as, const char * filename);
#else 
// If there is no guile, then load_scm_file() must always return 
// an error (i.e. a non-zero return value).
static inline int load_scm_file (AtomSpace& as, const char *) { return 1; }
#endif /* HAVE_GUILE */

/** @}*/
}

#endif /* OPENCOG_SERVER_LOAD_FILE_H */
