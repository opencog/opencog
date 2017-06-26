/*
 * PatternHeap.h
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

#ifndef _OPENCOG_PATTERNHEAP_H
#define _OPENCOG_PATTERNHEAP_H

#include <vector>
#include "TypeFrame.h"

namespace opencog
{

/*
 * A custom implementation of a mining result heap.
 *
 * This is here because standard heap implementations (using priority_queue)
 * have a couple of annoyances like the lack of an iterator and an elegant way
 * to avoid equivalent answers to be kept.
 */
class PatternHeap: public std::vector<std::pair<float, TypeFrame>>
{

public:

    PatternHeap(unsigned int maxSize = 10);

    unsigned int maxSize;
    void push(float v, const TypeFrame &frame);

private:

    bool contains(float v, const TypeFrame &frame) const;

};

}

#endif // _OPENCOG_PATTERNHEAP_H
