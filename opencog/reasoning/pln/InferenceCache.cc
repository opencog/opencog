/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include "InferenceCache.h"

#include "PLN.h"

#ifdef WIN32
#pragma warning(disable : 4311)
#endif

#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include "rules/RuleProvider.h"
#include "rules/Rules.h"
#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/foreach.hpp>

InferenceCache* InferenceCache::standardInferenceCache()
{
    static InferenceCache* standard = new InferenceCache();;

    return standard;
}

InferenceCache::~InferenceCache()
{
    foreach(BITNode* b, nodes) delete b;
}
