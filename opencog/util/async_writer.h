/*
 * FUNCTION:
 * Multi-threaded asynchronous write queue.
 *
 * HISTORY:
 * Copyright (c) 2013, 2015 Linas Vepstas <linasvepstas@gmail.com>
 *
 * LICENSE:
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

#ifndef _OC_ASYNC_WRITER_H
#define _OC_ASYNC_WRITER_H

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <opencog/util/concurrent_queue.h>
#include <opencog/util/concurrent_stack.h>
#include <opencog/util/Logger.h>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

template<typename Writer, typename Element>
class async_writer
{
	private:
		// Stuff to support asynchronous writes
		concurrent_queue<Element> _store_queue;
		std::vector<std::thread> _write_threads;
		std::mutex _write_mutex;
		unsigned int _thread_count;
		std::atomic<unsigned long> _busy_writers;
		bool _stopping_writers;

		Writer* _writer;
		void (Writer::*_do_write)(Element&);

		void start_writer_thread();
		void stop_writer_threads();
		void write_loop();

	public:
		async_writer(Writer*, void (Writer::*)(Element&), int nthreads=4);
		~async_writer();
		void enqueue(Element&);
		void flush_store_queue();
};



/* ================================================================ */
// Constructors

template<typename Writer, typename Element>
async_writer<Writer, Element>::async_writer(Writer* wr,
                                            void (Writer::*cb)(Element&),
                                            int nthreads)
{
	_writer = wr;
	_do_write = cb;
	_stopping_writers = false;
	_thread_count = 0;
	_busy_writers = 0;

	for (int i=0; i<nthreads; i++)
	{
		start_writer_thread();
	}
}

template<typename Writer, typename Element>
async_writer<Writer, Element>::~async_writer()
{
	stop_writer_threads();
}

/* ================================================================ */

/// Start a single writer thread.
/// May be called multiple times.
template<typename Writer, typename Element>
void async_writer<Writer, Element>::start_writer_thread()
{
	logger().info("async_writer: starting a writer thread");
	std::unique_lock<std::mutex> lock(_write_mutex);
	if (_stopping_writers)
		throw RuntimeException(TRACE_INFO,
			"Cannot start; async_writer writer threads are being stopped!");

	_write_threads.push_back(std::thread(&async_writer::write_loop, this));
	_thread_count ++;
}

/// Stop all writer threads, but only after they are done wroting.
template<typename Writer, typename Element>
void async_writer<Writer, Element>::stop_writer_threads()
{
	logger().info("async_writer: stopping all writer threads");
	std::unique_lock<std::mutex> lock(_write_mutex);
	_stopping_writers = true;

	// Spin a while, until the writeer threads are (mostly) done.
	while (not _store_queue.is_empty())
	{
		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
		usleep(1000);
	}

	// Now tell all the threads that they are done.
	// I.e. cancel all the threads.
	_store_queue.cancel();
	while (0 < _write_threads.size())
	{
		_write_threads.back().join();
		_write_threads.pop_back();
		_thread_count --;
	}

	// OK, so we've joined all the threads, but the queue
	// might not be totally empty; some dregs might remain.
	// Drain it now, single-threadedly.
	_store_queue.cancel_reset();
	while (not _store_queue.is_empty())
	{
		Element elt = _store_queue.pop();
		(_writer->*_do_write)(elt);
	}
	
	// Its now OK to start new threads, if desired ...(!)
	_stopping_writers = false;
}


/// Drain the pending store queue.
/// Caution: this is slightly racy; a writer could still be busy
/// even though this returns. (There's a window in writeLoop, between
/// the dequeue, and the busy_writer increment. I guess we should fix
/// this...
template<typename Writer, typename Element>
void async_writer<Writer, Element>::flush_store_queue()
{
	// std::this_thread::sleep_for(std::chrono::microseconds(10));
	usleep(10);
	while (0 < _store_queue.size() or 0 < _busy_writers);
	{
		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
		usleep(1000);
	}
}

/// A Single write thread. Reds atoms from queue, and stores them.
template<typename Writer, typename Element>
void async_writer<Writer, Element>::write_loop()
{
	try
	{
		while (true)
		{
			Element elt = _store_queue.pop();
			_busy_writers ++; // Bad -- window after pop returns, before increment!
			(_writer->*_do_write)(elt);
			_busy_writers --;
		}
	}
	catch (typename concurrent_queue<Element>::Canceled& e)
	{
		// We are so out of here. Nothing to do, just exit this thread.
		return;
	}
}


/* ================================================================ */
/**
 * Recursively store the indicated atom, and all that it points to.
 * Store its truth values too. The recursive store is unconditional;
 * its assumed that all sorts of underlying truuth values have changed, 
 * so that the whole thing needs to be stored.
 *
 * By default, the actual store is done asynchronously (in a different
 * thread); this routine merely queues up the atom. If the synchronous
 * flag is set, then the store is done in this thread.
 */
template<typename Writer, typename Element>
void async_writer<Writer, Element>::enqueue(Element& elt)
{
	// Sanity checks.
	if (_stopping_writers)
		throw RuntimeException(TRACE_INFO,
			"Cannot store; async_writer writer threads are being stopped!");
	if (0 == _thread_count)
		throw RuntimeException(TRACE_INFO,
			"Cannot store; No writer threads are running!");

	_store_queue.push(elt);

	// If the writer threads are falling behind, mitigate.
	// Right now, this will be real simple: just spin and wait
	// for things to catch up.  Maybe we should launch more threads!?
#define HIGH_WATER_MARK 100
#define LOW_WATER_MARK 10

	if (HIGH_WATER_MARK < _store_queue.size())
	{
		unsigned long cnt = 0;
		do
		{
			// std::this_thread::sleep_for(std::chrono::milliseconds(1));
			usleep(1000);
			cnt++;
		}
		while (LOW_WATER_MARK < _store_queue.size());
		logger().debug("async_writer overfull queue; had to sleep %d millisecs to drain!", cnt);
	}
}

/** @}*/
} // namespace opencog

#endif // _OC_ASYNC_WRITER_H
