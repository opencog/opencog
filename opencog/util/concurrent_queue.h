/*
 * opencog/util/concurrent_queue.h
 *
 * Based off of http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
 * Original version by Anthony Williams
 * Modifications by Michael Anderson
 * Modified by Linas Vepstas
 * Updated API to more closely resemble the proposed
 * ISO/IEC JTC1 SC22 WG21 N3533 C++ Concurrent Queues
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

#ifndef _OC_CONCURRENT_QUEUE_H
#define _OC_CONCURRENT_QUEUE_H

#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>

/** \addtogroup grp_cogutil
 *  @{
 */

//! Represents a thread-safe first in-first out list.
template<typename Element>
class concurrent_queue
{
private:
    std::deque<Element> the_queue;
    mutable std::mutex the_mutex;
    std::condition_variable the_cond;
    bool is_canceled;

public:
    concurrent_queue()
        : the_queue(), the_mutex(), the_cond(), is_canceled(false)
    {}
    concurrent_queue(const concurrent_queue&) = delete;  // disable copying
    concurrent_queue& operator=(const concurrent_queue&) = delete; // no assign

    struct Canceled : public std::exception
    {
        const char * what() { return "Cancellation of wait on concurrent_queue"; }
    };

    /// Push the Element onto the queue.
    void push(const Element& item)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        if (is_canceled) throw Canceled();
        the_queue.push_back(item);
        lock.unlock();
        the_cond.notify_one();
    }

    /// Push the Element onto the queue, by moving it.
    void push(Element&& item)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        if (is_canceled) throw Canceled();
        the_queue.push_back(std::move(item));
        lock.unlock();
        the_cond.notify_one();
    }

    /// Return true if the queue is empty.
    bool is_empty() const
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        if (is_canceled) throw Canceled();
        return the_queue.empty();
    }

    /// Return the size of the queue.
    /// Since the queue is time-varying, the size may become incorrect
    /// shortly after this method returns.
    unsigned int size() const
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        return the_queue.size();
    }

    bool try_get(Element& value)
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        if (is_canceled) throw Canceled();
        if (the_queue.empty())
        {
            return false;
        }

        value = the_queue.front();
        return true;
    }

    /// Pop an item off the queue. Block if the queue is empty.
    void pop(Element& value)
    {
        std::unique_lock<std::mutex> lock(the_mutex);

        // Use two nested loops here.  It can happen that the cond
        // wakes up, and yet the queue is empty.  And calling front()
        // on an empty dequeue is undefined and/or throws ick.
        do
        {
            while (the_queue.empty() and not is_canceled)
            {
                the_cond.wait(lock);
            }
            if (is_canceled) throw Canceled();
        }
        while (the_queue.empty());

        value = the_queue.front();
        the_queue.pop_front();
    }

    Element pop()
    {
        Element value;
        pop(value);
        return value;
    }

    std::deque<Element> wait_and_take_all()
    {
        std::unique_lock<std::mutex> lock(the_mutex);

        // Use two nested loops here.  It can happen that the cond
        // wakes up, and yet the queue is empty.
        do
        {
            while (the_queue.empty() and not is_canceled)
            {
                the_cond.wait(lock);
            }
            if (is_canceled) throw Canceled();
        }
        while (the_queue.empty());

        std::deque<Element> retval;
        std::swap(retval, the_queue);
        return retval;
    }

    /// A weak barrier. Its racy and thus unreliable across multiple
    /// threads, but should work just fine to serialize a single
    /// thread.
    void barrier()
    {
        std::unique_lock<std::mutex> lock(the_mutex);

        while (the_queue.empty() && !is_canceled)
        {
            the_cond.wait(lock);
        }
        if (is_canceled) throw Canceled();
    }

    void cancel_reset()
    {
       // This doesn't lose data, but it instead allows new calls
       // to not throw Canceled exceptions
       std::lock_guard<std::mutex> lock(the_mutex);
       is_canceled = false;
    }

    void cancel()
    {
       std::unique_lock<std::mutex> lock(the_mutex);
       if (is_canceled) throw Canceled();
       is_canceled = true;
       lock.unlock();
       the_cond.notify_all();
    }

};
/** @}*/

#endif // __CONCURRENT_QUEUE__
