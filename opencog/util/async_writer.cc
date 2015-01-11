/*
 * FUNCTION:
 * Multi-threaded asynchronous write queue
 *
 * Copyright (c) 2013,2015 Linas Vepstas <linas@linas.org>
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

#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <thread>

#include <opencog/util/async_writer.h>
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

using namespace opencog;


/* ================================================================ */
// Constructors

async_writer::async_writer(int nthreads)
{
	_stopping_writers = false;
	_thread_count = 0;
	_busy_writers = 0;

	for (int i=0; i<nthreads; i++)
	{
		start_writer_thread();
	}
}

async_writer::~async_writer()
{
	stop_writer_threads();
}

/* ================================================================ */

/// Start a single writer thread.
/// May be called multiple times.
void async_writer::start_writer_thread()
{
	logger().info("async_writer: starting a writer thread");
	std::unique_lock<std::mutex> lock(write_mutex);
	if (_stopping_writers)
		throw RuntimeException(TRACE_INFO,
			"Cannot start; async_writer writer threads are being stopped!");

	_write_threads.push_back(std::thread(&async_writer::write_oop, this));
	_thread_count ++;
}

/// Stop all writer threads, but only after they are done wroting.
void async_writer::stop_writer_threads()
{
	logger().info("async_writer: stopping all writer threads");
	std::unique_lock<std::mutex> lock(write_mutex);
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
		// AtomPtr atom = _store_queue.pop();
		// do_store_atom(atom);
	}
	
	// Its now OK to start new threads, if desired ...(!)
	_stopping_writers = false;
}

/// A Single write thread. Reds atoms from queue, and stores them.
void async_writer::write_loop()
{
	try
	{
		while (true)
		{
			// AtomPtr atom = store_queue.pop();
			_busy_writers ++; // Bad -- window after pop returns, before increment!
			// do_store_atom(atom);
			_busy_writers --;
		}
	}
	catch (concurrent_queue<AtomPtr>::Canceled& e)
	{
		// We are so out of here. Nothing to do, just exit this thread.
		return;
	}
}

/// Drain the pending store queue.
/// Caution: this is slightly racy; a writer could still be busy
/// even though this returns. (There's a window in writeLoop, between
/// the dequeue, and the busy_writer increment. I guess we should fix
// this...
void async_writer::flush_store_queue()
{
	// std::this_thread::sleep_for(std::chrono::microseconds(10));
	usleep(10);
	while (0 < _store_queue.size() or 0 < _busy_writers);
	{
		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
		usleep(1000);
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
void async_writer::enqueue(AtomPtr& atom, bool synchronous)
{
	// If a synchronous store, avoid the queues entirely.
	if (synchronous)
	{
		// do_store_atom(atom);
		return;
	}

	// Sanity checks.
	if (stopping_writers)
		throw RuntimeException(TRACE_INFO,
			"Cannot store; AtomStorage writer threads are being stopped!");
	if (0 == thread_count)
		throw RuntimeException(TRACE_INFO,
			"Cannot store; No writer threads are running!");

	_store_queue.push(atom);

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
		logger().debug("AtomStorage overfull queue; had to sleep %d millisecs to drain!", cnt);
	}
}

/* ============================= END OF FILE ================= */
