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
#include <chrono>

#include <opencog/attentionbank/ThreadSafeFixedIntegerIndex.h>
#include <opencog/util/mt19937ar.h>

using namespace opencog;
using namespace std::chrono;

size_t ThreadSafeFixedIntegerIndex::size() const
{
    std::lock_guard<std::mutex> lck(_mtx);
    size_t cnt = 0;
    for (unsigned int i = 0; i < _idx.size(); i++)
    {
        const AtomSet& s(_idx.at(i));
        cnt += s.size();
    }

    return cnt;
}

Handle ThreadSafeFixedIntegerIndex::getRandomAtom(void)
{
    auto seed = duration_cast< microseconds >(
            system_clock::now().time_since_epoch());
    MT19937RandGen rng(seed.count());

    size_t  bins = bin_size();
    bool empty = true;
    for (size_t i=0; i<bins; i++)
    {
        if (size(i) > 0 )
        {
            empty = false;
            break;
        }
    }

    if (empty)
        return Handle::UNDEFINED;

    // `attempts` prevents an infinite loop.
    size_t bin = 0, attempts = 0;
    do
    {
        bin = rng.randint(bins-1);  
        attempts++;
    } while (size(bin) <= 0 or attempts < bins);

    std::vector<Atom*> bin_content;
    std::lock_guard<std::mutex> lck(_mtx);
    const AtomSet &s(_idx.at(bin));
    if (not s.size()) return Handle::UNDEFINED;
    size_t idx = rng.randint(bin_content.size()-1); 
    auto it = s.begin();
    std::advance(it, idx);
    Atom* atom = *it;
    return atom->getHandle();
}

// ================================================================
