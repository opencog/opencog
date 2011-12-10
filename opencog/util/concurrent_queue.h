/*
 * opencog/util/concurrent_queue.h
 *
 * Based off of http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
 * Original version by Anthony Williams
 * Modifications by Michael Anderson
 * Modified by Linas Vepstas; the original code had race-conditions in it.
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

#include "boost/thread.hpp"
#include <deque>
#include <stdio.h>

template<typename Data>
class concurrent_queue
{
private:
    std::deque<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
    bool is_canceled;

public:
    concurrent_queue() : the_queue(), the_mutex(), the_condition_variable(), is_canceled(false) {}
    struct Canceled{};
    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if (is_canceled) throw Canceled();
        the_queue.push_back(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if (is_canceled) throw Canceled();
        return the_queue.empty();
    }

    unsigned int approx_size() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.size();
    }

    bool try_get(Data& value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if (is_canceled) throw Canceled();
        if (the_queue.empty())
        {
            return false;
        }

        value = the_queue.front();
        return true;
    }

    // The code originally had a wait_and_pop() primitive.
    // Unfortunately, this will race against the empty() primitive.
    // The correct solution is to wait_and_get the value, do things
    // with the value, and then, only when done working with the value,
    // should one pop. Doing things this way will allow the empty()
    // function to correctly report the state of the work queue.
    void wait_and_get(Data& value)
    {
        boost::mutex::scoped_lock lock(the_mutex);

        while(the_queue.empty() && !is_canceled)
        {
            the_condition_variable.wait(lock);
        }
        if (is_canceled) {
            throw Canceled();
        }
        value = the_queue.front();
    }

    void pop()
    {
        the_queue.pop_front();
    }

    std::deque<Data> wait_and_take_all()
    {
        boost::mutex::scoped_lock lock(the_mutex);

        while(the_queue.empty() && !is_canceled)
        {
            the_condition_variable.wait(lock);
        }
        if (is_canceled) throw Canceled();

        std::deque<Data> retval;
        std::swap(retval, the_queue);
        return retval;
    }

    void cancel_reset()
    {
       // this doesn't lose data, but it instead allows new calls
       // to not throw Canceled exceptions
       boost::mutex::scoped_lock lock(the_mutex);
       is_canceled = false;
    }

    void cancel()
    {
       boost::mutex::scoped_lock lock(the_mutex);
       if (is_canceled) throw Canceled();
       is_canceled = true;
       lock.unlock();
       the_condition_variable.notify_all();
    }

};

#endif // __CONCURRENT_QUEUE__
