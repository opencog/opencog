/*
 * opencog/attentionbank/bank/AtomBins.cc
 *
 * Copyright (C) 2016 Roman Treutlein <roman.treutlein@gmail.com>
 * Copyright (C) 2017 Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/util/mt19937ar.h>
#include <opencog/attentionbank/bank/AtomBins.h>

using namespace opencog;
using namespace std::chrono;

size_t AtomBins::size() const
{
    std::lock_guard<std::mutex> lck(_mtx);
    size_t cnt = 0;
    for (unsigned int i = 0; i < _idx.size(); i++)
    {
        cnt += _idx.at(i).size();
    }

    return cnt;
}

Handle AtomBins::getRandomAtom(void) const
{
    auto seed = duration_cast< microseconds >(
            system_clock::now().time_since_epoch());
    MT19937RandGen rng(seed.count());

    std::lock_guard<std::mutex> lck(_mtx);

    size_t bins = _idx.size();
    bool empty = true;
    for (size_t i=0; i<bins; i++)
    {
        if (0 < _idx.at(i).size())
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
    } while (_idx.at(bin).size() <= 0 or attempts < bins);

    const HandleSet& s(_idx.at(bin));
    if (0 == s.size()) return Handle::UNDEFINED;

    size_t idx = rng.randint(s.size()-1);
    auto it = s.begin();
    std::advance(it, idx);
    return *it;
}

// ================================================================
