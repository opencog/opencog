/*
 * FUNCTION:
 * Base class for asynchrnouous index maintenance.
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

#ifndef _OPENCOG_ASYNC_UPDATE_H
#define _OPENCOG_ASYNC_UPDATE_H

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <opencog/util/concurrent_queue.h>
#include <opencog/util/concurrent_queue.h>
#include <opencog/util/concurrent_stack.h>

#include "Handle.h"

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class AsyncUpdate
{
	private:
		// Stuff to support asynchronous update of the indexes
		concurrent_queue<AtomPtr> store_queue;
		std::vector<std::thread> write_threads;
		std::mutex write_mutex;
		unsigned int thread_count;
		std::atomic<unsigned long> busy_writers;
		void startWriterThread();
		void stopWriterThreads();
		bool stopping_writers;
		void writeLoop();

	public:
		AsyncUpdate(int nthreads);
		~AsyncUpdate();
		void enqueue(AtomPtr&, bool);
		void flushStoreQueue();
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ASYNC_UPDATE_H
