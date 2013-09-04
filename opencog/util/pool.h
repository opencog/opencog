/*
 * util/pool.h --- 
 *
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Author: Linas Vepstas
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


#ifndef _OPENCOG_UTIL_POOL_H
#define _OPENCOG_UTIL_POOL_H

#include <queue>
#include <mutex>
#include <condition_variable>

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

//! Thread-safe blocking resource allocator.
/// If there are no resources to borrow, then the borrow() method will
/// block until one is given back.
//
// Sadly, there is nothing in boost that does this.  The boost::pool
// author clearly didn't understand the problem.  Oh well.
//
// This is implemented on top of std::queue, for no particular reason;
// pretty much any other container would do.
//
template<typename Resource>
class pool
{
    public:
        /// Fetch a resource from the pool. Block if the pool is empty.
        /// If blocked, this will unblock when a resource is put into
        /// the pool.
        Resource& borrow()
        {
            std::unique_lock<std::mutex> lock(mu);
            while (objs.size() == 0) {
                cond.wait(lock);
            }
            Resource& rv = objs.front();
            objs.pop();
            return rv;
        }

        /// Put a resource into the pool.  If the pool is empty, and
        /// other threads are blocked and waiting, this will release
        /// some other blocked thread.
        void give_back(Resource& obj)
        {
            std::lock_guard<std::mutex> lock(mu);
            objs.push(obj);
            cond.notify_one();
        }

        size_t available()
        {
            return objs.size();
        }

    private:
        std::mutex mu;
        std::condition_variable cond;
        std::queue<Resource> objs;
};


/** @}*/
} // ~namespace opencog

#endif // _OPENCOG_UTIL_POOL_H
