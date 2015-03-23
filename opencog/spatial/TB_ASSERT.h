/*
 * opencog/spatial/TB_ASSERT.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef _SPATIAL_TBASSERT_H_
#define _SPATIAL_TBASSERT_H_

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
/** \addtogroup grp_spatial
 *  @{
 */

#define TB_ASSERT(x) if (!(x)) OC_ASSERT("TB_ASSERT - %s",  #x)

/** @}*/
#endif
