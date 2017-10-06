/*
 * opencog/attentinbank/ThreadSafeFixedIntegerIndex.h
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

#ifndef _OPENCOG_THREADSAFEFIXEDINTEGERINDEX_H
#define _OPENCOG_THREADSAFEFIXEDINTEGERINDEX_H

#include <vector>
#include <mutex>
#include <atomic>
#include <memory>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class Atom;
/**
 * Implements a vector of atom sets.
 */
class ThreadSafeFixedIntegerIndex
{
    private:
        mutable std::mutex _mtx;
        std::vector<AtomSet> _idx;

    public:
        ThreadSafeFixedIntegerIndex(size_t sz)
        {
            _idx.resize(sz);
        }

        void insert(size_t i, Atom* a)
        {
            std::lock_guard<std::mutex> lck(_mtx);
            _idx.at(i).insert(a);
        }

        void remove(size_t i, Atom* a)
        {
            std::lock_guard<std::mutex> lck(_mtx);
            _idx.at(i).erase(a);
        }

        size_t size(size_t i) const
        {
            std::lock_guard<std::mutex> lck(_mtx);
            return _idx.at(i).size();
        }

        Handle getRandomAtom(void);

        size_t size() const;

        template <typename OutputIterator> OutputIterator
        getContent(size_t i, OutputIterator out) const
        {
            std::lock_guard<std::mutex> lck(_mtx);
            const AtomSet& s(_idx.at(i));
            return std::copy(s.begin(), s.end(), out);
        }

        template <typename OutputIterator> OutputIterator
        getContentIf(size_t i,
                    OutputIterator out,
                    std::function<bool(Atom*)> pred) const
        {
            std::lock_guard<std::mutex> lck(_mtx);
            const AtomSet& s(idx.at(i));
            return std::copy_if(s.begin(), s.end(), out, pred);
        }
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_FIXEDINTEGERINDEX_H
