/*
 * opencog/spatial/TB_ASSERT.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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

// DEPRECATED
// TB_ASSERT - do not throw exceptions and have trace info
//#define TB_ASSERT(x) if (!(x)) MAIN_LOGGER.log(Util::Logger::ERROR, "TB_ASSERT - %s",  #x)

#define TB_ASSERT(x) if (!(x)) opencog::cassert(TRACE_INFO, "TB_ASSERT - %s",  #x)

#endif
