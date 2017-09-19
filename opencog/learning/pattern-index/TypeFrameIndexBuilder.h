/*
 * TypeFrameIndexBuilder.h
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#ifndef _OPENCOG_TYPEFRAMEINDEXBUILDER_H
#define _OPENCOG_TYPEFRAMEINDEXBUILDER_H

#include "SCMLoaderCallback.h"
#include "TypeFrameIndex.h"

namespace opencog
{

class TypeFrameIndexBuilder : public SCMLoaderCallback
{

public:

    TypeFrameIndexBuilder(TypeFrameIndex *index);
    ~TypeFrameIndexBuilder();

    // SCMLoaderCallback interface
    void beforeInserting(const std::string &schemeStr);
    void afterInserting(const Handle &toplevelAtom);

private:

    TypeFrameIndex *index;
    int lastOffset;
};

}

#endif // _OPENCOG_TYPEFRAMEINDEXBUILDER_H
