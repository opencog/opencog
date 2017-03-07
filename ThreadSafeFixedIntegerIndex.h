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

#include <opencog/atomspace/FixedIntegerIndex.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class Atom;
/**
 * Implements a vector of atom sets; each set can be found via an
 * integer index.
 */
class ThreadSafeFixedIntegerIndex : public FixedIntegerIndex
{
    typedef std::unordered_set<Atom*> AtomSet;

    private:
        using FixedIntegerIndex::idx;
        mutable std::vector<std::unique_ptr<std::mutex>> _locks;
        //mutable std::vector<std::mutex> _locks;

        void resize(size_t sz)
        {
            FixedIntegerIndex::resize(sz);
            //std::vector<std::mutex> new_locks(sz);
            _locks.resize(sz);
            for (auto iter = _locks.begin(); iter != _locks.end(); ++iter)
                (*iter) = std::unique_ptr<std::mutex>(new std::mutex());
        }

    public:
        ThreadSafeFixedIntegerIndex(size_t size)
        {
            resize(size);
        }
        ~ThreadSafeFixedIntegerIndex () {}

        void insert(size_t i, Atom* a)
        {
            std::lock_guard<std::mutex> lck(*_locks[i]);
            FixedIntegerIndex::insert(i,a);
        }

        void remove(size_t i, Atom* a)
        {
            std::lock_guard<std::mutex> lck(*_locks[i]);
            FixedIntegerIndex::remove(i,a);
        }

        size_t size(size_t i) const
        {
            std::lock_guard<std::mutex> lck(*_locks[i]);
            return FixedIntegerIndex::size(i);
        }

        Handle getRandomAtom(void);
        
        size_t size() const;

        template <typename OutputIterator> OutputIterator
        getContent(size_t i,OutputIterator out) const
        {
            std::lock_guard<std::mutex> lck(*_locks[i]);
            const AtomSet &s(idx.at(i));
            return std::copy(s.begin(), s.end(), out);
        }

        template <typename OutputIterator> OutputIterator
        getContentIf(size_t i
                    ,OutputIterator out
                    ,std::function<bool(Atom *)> pred) const
        {
            std::lock_guard<std::mutex> lck(*_locks[i]);
            const AtomSet &s(idx.at(i));
            return std::copy_if(s.begin(), s.end(), out,pred);
        }
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_FIXEDINTEGERINDEX_H
