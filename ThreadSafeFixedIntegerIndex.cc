/*
 * opencog/attentionbank/ThreadSafeFixedIntegerIndex.cc
 *
 * Copyright (C) 2016 Roman Treutlein <roman.treutlein@gmail.com>
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

#include <opencog/attentionbank/ThreadSafeFixedIntegerIndex.h>

using namespace opencog;

size_t ThreadSafeFixedIntegerIndex::size() const
{
    size_t cnt = 0;
    for (unsigned int i = 0; i < idx.size(); i++)
        cnt += size(i);
    return cnt;
}

// ================================================================
