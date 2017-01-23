/*
 * GenericShell.h
 *
 * Template for a generic shell
 * Copyright (c) 2008, 2013, 2014 Linas Vepstas <linas@linas.org>
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

#ifndef _OPENCOG_GENERIC_SHELL_H
#define _OPENCOG_GENERIC_SHELL_H

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <opencog/util/concurrent_queue.h>

/**
 * The GenericShell class implements an "escape" from the default cogserver
 * command processor. It is useful when a module has a large number of
 * commands, or a peculiar and complex syntax that must be handled. It
 * works by by-passing all of the command-processing apparatus in the
 * cogserver, and instead passing all socket I/O to the "eval" method
 * in this class. The eval method is then free to parse the input in
 * any way desired.
 *
 * If instead a module has only a small number of simple commands that
 * need to be implemented, then the DECLARE_CMD_REQUEST function, defined
 * in Request.h, provides a simpler and easier way implementing commands.
 */

namespace opencog {
/** \addtogroup grp_server
 *  @{
 */

class ConsoleSocket;
class GenericEval;

class GenericShell
{
	private:
		std::mutex _pending_mtx;
		std::string _pending_output;

		ConsoleSocket* socket;
		std::thread* evalthr;
		std::thread* pollthr;
		concurrent_queue<std::string> evalque;
		volatile bool _init_done;

	protected:
		std::string abort_prompt;
		std::string normal_prompt;
		std::string pending_prompt;
		bool show_output;
		bool show_prompt;
		volatile bool self_destruct;

		virtual GenericEval* get_evaluator(void) = 0;
		virtual void thread_init(void);
		virtual void line_discipline(const std::string &expr);

		// Concurrency handling
		void eval_loop();
		void poll_loop();
		bool _eval_done;
		std::condition_variable _cv;
		std::mutex _mtx;
		GenericEval* _evaluator;
		void start_eval();
		void finish_eval();
		void while_not_done();

		// Output handling.
		virtual void put_output(const std::string&);
		virtual std::string get_output();
		virtual std::string poll_output();

	public:
		GenericShell(void);
		virtual ~GenericShell();

		virtual void set_socket(ConsoleSocket *);
		virtual void eval(const std::string &);

		virtual const std::string& get_prompt(void);
		virtual void hush_output(bool);
		virtual void hush_prompt(bool);
};

/** @}*/
}

#endif // _OPENCOG_GENERIC_SHELL_H
