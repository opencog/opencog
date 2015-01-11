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

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

template<typename Element>
class async_writer
{
	private:
		// Stuff to support asynchronous writes
		concurrent_queue<Element> _store_queue;
		std::vector<std::thread> _write_threads;
		std::mutex _write_mutex;
		unsigned int _thread_count;
		std::atomic<unsigned long> _busy_writers;
		void start_writer_thread();
		void stop_writer_threads();
		bool stopping_writers;
		void write_loop();

	public:
		async_update(int nthreads);
		~async_update();
		void enqueue(Element&, bool);
		void flush_store_queue();
};

/** @}*/
} // namespace opencog

#endif // _OC_ASYNC_WRITER_H
