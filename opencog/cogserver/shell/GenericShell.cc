/*
 * GenericShell.cc
 *
 * Generic interactive shell
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

#include <mutex>
#include <thread>

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

#include <opencog/cogserver/server/ConsoleSocket.h>
#include <opencog/eval/GenericEval.h>
#include "GenericShell.h"

using namespace opencog;

// Some random RFC 854 characters
#define IAC  0xff // Telnet Interpret As Command
#define TEOF 0xec // Telnet EOF
#define SUSP 0xed // Telnet suspend
#define ABRT 0xee // Telnet abort
#define NOP  0xf1 // Telnet NOP no-op
#define BRK  0xf3 // Telnet break
#define IP   0xf4 // Telnet IP Interrupt Process
#define AO   0xf5 // Telnet AO Abort Output
#define AYT  0xf6 // Telnet AYT Are You There
#define EC   0xf7 // Telnet EC Erase Character
#define EL   0xf8 // Telnet EL Erase Line
#define GA   0xf9 // Telnet GA Go ahead
#define WILL 0xfb // Telnet WILL
#define WONT 0xfc // Telnet WONT
#define DO   0xfd // Telnet DO
#define DONT 0xfe // Telnet DONT

#define RFC_ECHO          1  // Telnet RFC 857 ECHO option
#define SUPPRESS_GO_AHEAD 3  // Telnet RFC 858 supporess go ahead
#define TIMING_MARK       6  // Telnet RFC 860 timing mark
#define LINEMODE          34 // Telnet RFC 1116 linemode

// Some random ASCII control characters (unix semantics)
#define EOT 0x4   // end    or ^D at keyboard.
#define SYN 0x16  // quit   or ^C at keyboard.
#define CAN 0x18  // cancel or ^X at keyboard.
#define ESC 0x1b  // ecsape or ^[ at keyboard.

GenericShell::GenericShell(void)
{
	show_output = true;
	show_prompt = true;
	normal_prompt = "> ";
	abort_prompt = normal_prompt;
	pending_prompt = "... ";

	socket = nullptr;
	evalthr = nullptr;
	pollthr = nullptr;
	self_destruct = false;
	_eval_done = true;
	_evaluator = nullptr;
}

GenericShell::~GenericShell()
{
	self_destruct = true;

	// It can happen that we already cancelled (e.g. control-D)
	try { evalque.cancel(); }
	catch (const std::exception& ex) {}

	if (evalthr)
	{
		logger().debug("[GenericShell] dtor, wait for eval thread 0x%x.",
		               evalthr->native_handle());
		evalthr->join();
		logger().debug("[GenericShell] dtor, joined eval thread");

		delete evalthr;
		evalthr = nullptr;
	}
	logger().debug("[GenericShell] dtor finished.");
}

/* ============================================================== */

void GenericShell::hush_output(bool hush)
{
	show_output = !hush;
}

void GenericShell::hush_prompt(bool hush)
{
	show_prompt = !hush;
}

const std::string& GenericShell::get_prompt(void)
{
	static const std::string empty_prompt = "";
	if (!show_prompt) return empty_prompt;

	// Use different prompts, depending on whether there is pending
	// input or not.
	if (_evaluator and _evaluator->input_pending())
	{
		return pending_prompt;
	}
	else
	{
		return normal_prompt;
	}
}

/* ============================================================== */
/**
 * Register this shell with the console.
 */
void GenericShell::set_socket(ConsoleSocket *s)
{
	OC_ASSERT(socket==nullptr, "Shell already associated with socket!");

	socket = s;
	socket->SetShell(this);
}

/* ============================================================== */

static std::mutex _stdout_redirect_mutex;

#ifdef PERFORM_STDOUT_DUPLICATION
static GenericShell* _redirector = nullptr;
#endif

// Implementation requirements:
//
// 1) We want all evaluations to be carried out in serial order,
//    so that the previous expression is fully evaluated before the
//    next one is started.
// 2) We want all evaluations to be interruptible, so that if an
//    expression is an infinite loop (or simply is taking too long)
//    the user can send a control-C and interrupt the execution.
// 3) Due to the client-server socket model, this method should
//    return as soon as possible, so that the caller can resume
//    waiting on the socket, in case the user is trying to send
//    a control-C to us.
// 4) Long-running evaluations should send output back to the user
//    synchronously: i.e. send output back to the socket as the
//    output is generated, instead of waiting for the evaluation
//    to terminate first, before relaying output.
//
// The above requirements force us to create not just one, but two
// threads for each evaluation: one thread for the evaluation, and
// another thread to listen for results, and pass them on.
//
// Side-note: the constructor for this class runs in a different thead
// than the caller for this method. That's because the socket listen
// and socket accept runs in a different thread, than the socket
// receive.  The receiver thread calls us.
//
void GenericShell::eval(const std::string &expr)
{
	assert (not self_destruct);
	// First time through, initialize the evaluator.  We can't do this
	// in the ctor, since we can't get the evaluator until after the
	// derived-class ctor has run, and thus informed us as to whether
	// the evaluator will be guile (scheme) or python.
	if (nullptr == _evaluator)
	{
		_init_done = false;
		// Run the evaluation loop in a distinct thread.
		auto eval_wrapper = [&](void) { eval_loop(); };
		evalthr = new std::thread(eval_wrapper);
		while (not _init_done) { sched_yield(); }
	}

	// Work-around some printing madness. See issue
	// https://github.com/opencog/atomspace/issues/629
	// This is kind of complicated to explain, so pay attention:
	// When runnning scheme, or python code, from the cogserver
	// shell, that code might cause all sorts of things to happen,
	// including possibly printing to the stdout file descriptor.
	// The stdout descriptor goes to the terminal in which the
	// cogserver was started. Which is nice, and all that, but
	// is usually not quite what the user expected --- and so we
	// actually want to redirect stdout to the shell, where the user
	// can see it.
	//
	// The code below, ifdefed PERFORM_STDOUT_DUPLICATION, does this.
	// It's a bit of a trick: make a backup copy of stdout,
	// then attach stdout to a pipe, perform the evaluation, then
	// restore stdout from the backup. Finally, drain the pipe,
	// printing both to stdout and to the shell socket.
// #define PERFORM_STDOUT_DUPLICATION 1
#ifdef PERFORM_STDOUT_DUPLICATION
	// What used to be stdout will now go to the pipe.
	int pipefd[2];
	int stdout_backup = -1;
	if (show_output and show_prompt)
	{
		std::lock_guard<std::mutex> lock(_stdout_redirect_mutex);
		if (nullptr == _redirector)
		{
			_redirector = this;
			int rc = pipe2(pipefd, 0);  // O_NONBLOCK);
			OC_ASSERT(0 == rc, "GenericShell pipe creation failure");
			stdout_backup = dup(fileno(stdout));
			OC_ASSERT(0 < stdout_backup, "GenericShell stdout dup failure");
			rc = dup2(pipefd[1], fileno(stdout));
			OC_ASSERT(0 < rc, "GenericShell pipe splice failure");
		}
	}
#endif // PERFORM_STDOUT_DUPLICATION

	// Queue up the expr, where it will be evaluated in another thread.
	line_discipline(expr);

#ifdef PERFORM_STDOUT_DUPLICATION
	if (show_output and show_prompt)
	{
		std::lock_guard<std::mutex> lock(_stdout_redirect_mutex);
		if (this == _redirector)
		{
			_redirector = nullptr;
			// Restore stdout
			fflush(stdout);
			int rc = write(pipefd[1], "", 1); // null-terminated string!
			OC_ASSERT(0 < rc, "GenericShell pipe termination failure");
			rc = close(pipefd[1]);
			OC_ASSERT(0 == rc, "GenericShell pipe close failure");
			rc = dup2(stdout_backup, fileno(stdout)); // restore stdout
			OC_ASSERT(0 < rc, "GenericShell restore stdout failure");

			// Drain the pipe
			auto drain_wrapper = [&](void)
			{
				char buf[4097];
				int nr = read(pipefd[0], buf, sizeof(buf)-1);
				OC_ASSERT(0 < rc, "GenericShell pipe read failure");
				while (0 < nr)
				{
					buf[nr] = 0;
					if (1 < nr or 0 != buf[0])
					{
						printf("hey hye hey %s", buf); // print to the cogservers stdout.
						socket->Send(buf);
					}
					nr = read(pipefd[0], buf, sizeof(buf)-1);
					OC_ASSERT(0 < rc, "GenericShell pipe read failure");
				}

				// Cleanup.
				close(pipefd[0]);
				close(stdout_backup);
			};

			drain_wrapper();
			// stdout_thr = new std::thread(drain_wrapper);
		}
	}
#endif // PERFORM_STDOUT_DUPLICATION

	// The user is exiting the shell. No one will ever call a method on
	// this instance ever again. So stop hogging space, and self-destruct.
	// We have to do this here; there is no other opportunity to call dtor.
	if (self_destruct)
	{
		socket->sendPrompt();
		socket->SetShell(nullptr);
		delete this;
	}
}

/**
 * Handle user-generated interrupt (ctrl-C, etc)
 */
void GenericShell::user_interrupt()
{
	// Discard all pending, unevaluated junk in the queue.
	// Failure to do so will typically result in confusing
	// the shell user.
	while (not evalque.is_empty()) evalque.pop();

	// Must send TIMING-MARK first, as otherwise telnet silently
	// ignores any bytes that come before it.
	unsigned char ok[] = {IAC, WILL, TIMING_MARK, '\n', 0};
	put_output((const char *) ok);
	_evaluator->interrupt();
	_evaluator->clear_pending();
	put_output(abort_prompt);
	finish_eval();
}

/* ============================================================== */
/**
 * Handle special characters, evaluate the expression
 */
void GenericShell::line_discipline(const std::string &expr)
{
	size_t len = expr.length();

	logger().debug("[GenericShell] line disc: expr, len of %zd ='%s'",
		len, expr.c_str());

	if (0 == len)
	{
		evalque.push("\n");
		return;
	}

	// Handle Telnet RFC 854 IAC format
	// Basically, we're looking for telnet-encoded abort or interrupt
	// characters, starting at the end of the input string. If they
	// are there, then don't process input, and clear out the evaluator.
	// Also, be sure to send telnet IAC WILL TIMING-MARK so that telnet
	// doesn't sit there flushing output forever.
	//
	// Search for IAC to at most 20 chars from the end of the string.
	int i = len-2;
	int m = len - 20;
	if (m < 0) m = 0;
	while (m <= i)
	{
		unsigned char c = expr[i];
		if (IAC == c)
		{
			c = expr[i+1];
			if (IP == c or AO == c or SUSP == c)
			{
				logger().debug("[GenericShell] got user-interrupt %d", c);
				user_interrupt();
				return;
			}

			// Erase line -- just ignore this line.
			// Also other things we want to ignore, like break and abort.
			if (EL == c or EC == c or BRK == c or ABRT == c or
			    AYT == c or GA == c or NOP == c)
			{
				logger().debug("[GenericShell] ignoring telnet IAC %d", c);
				put_output(get_prompt());
				return;
			}

			// End-of-file just like ctrl-D
			if (TEOF == c)
			{
				logger().debug("[GenericShell] got end-of-file; exiting shell");
				self_destruct = true;
				evalque.cancel();
				if (show_prompt)
					put_output("Exiting the shell\n");
				return;
			}

			if (DO == c)
			{
				c = expr[i+2];
				// Some telnets, including on Debian Stable, send us
				// IAC DO TIMING-MARK instead of a IAC IP or IAC AO
				// when the user hits ctrl-C.  This seems broken to me,
				// but whatever. Pretend its a normal interrupt.
				if (TIMING_MARK == c)
				{
					logger().debug("[GenericShell] timing mark (user-interrupt?)");
					user_interrupt();
					return;
				}

				// If telnet ever tries to go into character mode,
				// it will send us SUPPRESS-GO-AHEAD and ECHO. Try to
				// stop that, we don't want to effing fiddle with that.
				if (SUPPRESS_GO_AHEAD == c)
				{
					unsigned char ok[] = {IAC, WILL, SUPPRESS_GO_AHEAD, 0};
					put_output((const char *) ok);
					return;
				}
				if (RFC_ECHO == c)
				{
					unsigned char ok[] = {IAC, WONT, RFC_ECHO, '\n', 0};
					put_output((const char *) ok);
					return;
				}
				logger().debug("[GenericShell] IAC WONT %d", c);
				unsigned char ok[] = {IAC, WONT, c, '\n', 0};
				put_output((const char *) ok);
				return;
			}

			if (WILL == c)
			{
				// Just ignore these.  Typically, will be IAC WILL LINEMODE
				// sent by the telnet client.
				c = expr[i+2];
				logger().debug("[GenericShell] ignoring telnet IAC WILL %d", c);
				return;
			}
		}
		i--;
	}

	// Don't evaluate if the line is terminated by
	// escape (^[), cancel (^X) or quit (^C)
	// These would typically be sent by netcat, and not telnet.
	unsigned char c = expr[len-1];
	if ((SYN == c) || (CAN == c) || (ESC == c))
	{
		// Discard all pending, unevaluated junk in the queue.
		while (not evalque.is_empty()) evalque.pop();

		_evaluator->interrupt();
		_evaluator->clear_pending();

		put_output("\n");
		finish_eval();
		put_output(normal_prompt);
		return;
	}

	// Look for either an isolated control-D, or a single period on a line
	// by itself. This means "leave the shell". We leave the shell by
	// unsetting the shell pointer in the ConsoleSocket.
	// 0x4 is ASCII EOT, which is what ctrl-D at keybd becomes.
	if ((false == _evaluator->input_pending()) and
	    ((EOT == expr[len-1]) or ((1 == len) and ('.' == expr[0]))))
	{
		logger().debug("[GenericShell] got control-D; exiting shell");
		self_destruct = true;
		evalque.cancel();
		if (show_prompt)
			put_output("Exiting the shell\n");
		return;
	}

	/*
	 * The newline was cut by the request subsystem. Re-insert it;
	 * otherwise, comments within procedures will have the effect of
	 * commenting out the rest of the procedure, leading to garbage.
	 */
	evalque.push(expr + "\n");
}

/* ============================================================== */
// The problem being adressed here is that the shell destructor
// can start running (because the socket was closed) before the
// evaluator has even started running. This is not really a
// problem for this class; however, if a derived class
// (specifically, the SchemeShell) is destroyed before it has a
// chance to run thread_init() during evaluation, then crashes
// will result (in this case, because the atomspace was not set).

void GenericShell::start_eval()
{
	OC_ASSERT(_eval_done, "Bad evaluator flag state!");
	std::unique_lock<std::mutex> lck(_mtx);
	_eval_done = false;
}

void GenericShell::finish_eval()
{
	// Repeated control-C will send us here with _eval_done already set..
	std::unique_lock<std::mutex> lck(_mtx);
	_eval_done = true;
	_cv.notify_all();
}

void GenericShell::while_not_done()
{
	std::unique_lock<std::mutex> lck(_mtx);
	while (not _eval_done) _cv.wait(lck);
}

/* ============================================================== */

/// eval_loop. Dequeue and run each queued evaluation request.
/// Assumes line discipline has already been done (as it must be:
/// it is impossible to queue OOB interrupts.)
void GenericShell::eval_loop(void)
{
	logger().debug("[GenericShell] enter eval loop");
	OC_ASSERT(nullptr == _evaluator, "Bad evaluator state!");

	// Per-shell evaluator.  We do this here, not in the ctor, because
	// we want a unique, private evaluator for this thread. By contrast,
	// the ctor might be called many times within one thread, and thus,
	// each invocation would end up with the same evaluator.
	_evaluator = get_evaluator();
	_evaluator->clear_pending();

	// Poll for output from the evaluator, and send back results.
	auto poll_wrapper = [&](void) { poll_loop(); };
	pollthr = new std::thread(poll_wrapper);

	// Derived-class initializer. (The scheme shell uses this to set
	// the atomspace).
	thread_init();

	// Go through the body of the loop at least once.
	std::string in;
	do
	{
		try
		{
			// Do not begin the next queued expr until the last
			// has finished. Failure to do this can result in
			// weird crashes in the SchemeEval class.
			while_not_done();

			// Note that this pop will stall until the queue
			// becomes non-empty.
			evalque.pop(in);
			logger().debug("[GenericShell] start eval of '%s'", in.c_str());
			start_eval();
			_evaluator->begin_eval();
			_evaluator->eval_expr(in);
		}
		catch (const concurrent_queue<std::string>::Canceled& ex)
		{
			break;
		}
	} while (not self_destruct);

	// If we are here, then we can safely assume that the socket has
	// been closed, that the dtor for this instance has been called.
	// However, there may still be some remaining, unfinished work
	// in the command queue; drain the queue, before shutting down.
	assert(self_destruct);
	evalque.cancel_reset();

	// Let the polling thread die first. If we don't do this, it will
	// interfer with the manual polling below.
	pollthr->join();
	delete pollthr;
	pollthr = nullptr;

	// Nothing more will be queued, so we can safely loop over remainder
	// of the queue, without any additional need for locking/waiting.
	while (0 < evalque.size())
	{
		// As mentioned before, do not begin the next queued expr until
		// the last has finished. Failure to do this results in crashes.
		poll_and_send();
		while (not _eval_done)
		{
			usleep(10000);
			poll_and_send();
		}

		try
		{
			evalque.pop(in);
		}
		catch (const concurrent_queue<std::string>::Canceled& ex)
		{
			evalque.cancel_reset();
			continue;
		}

		logger().debug("[GenericShell] finishing; eval of '%s'", in.c_str());
		start_eval();
		_evaluator->begin_eval();
		_evaluator->eval_expr(in);
	}

	// Continue polling until the evaluation really is done.
	poll_and_send();
	while (not _eval_done)
	{
		usleep(10000);
		poll_and_send();
	}

	// After we exit, the _evaluator will be reclaimed by the
	// thread dtor running in the evaluator pool.
	_evaluator = nullptr;
	logger().debug("[GenericShell] exit eval loop");
}

void GenericShell::poll_and_send(void)
{
	std::string retstr(poll_output());
	if (0 < retstr.size())
		socket->Send(retstr);
}

void GenericShell::poll_loop(void)
{
	_init_done = true;

	// Poll for output from the evaluator, and send back results.
	while (not self_destruct)
	{
		poll_and_send();

		// Continue polling, about 100 times per second, even if
		// evaluation of the the previous expr is completed. It
		// might have started some long-running thread/agent that
		// is continuing to print, and we want to forward those
		// prints to the user. (Its pointless to poll faster or
		// slower than this...)
		if (_eval_done) usleep(10000);
	}

	// It's also possible that another thread reaches the dtor
	// (setting self_destruct == true) shortly after an evaluation
	// has just finished. It may then exit the loop above without
	// polling the output, eventually causing the evalthr to stay
	// in while_not_done() forever. So let's poll again, one more
	// time, here.
	do
	{
		usleep(10000);
		poll_and_send();
	}
	while (not _eval_done);
}

void GenericShell::thread_init(void)
{
	/* No-op. The Scheme shell sets the current atomspace here */
}

/* ============================================================== */

void GenericShell::put_output(const std::string& s)
{
	std::lock_guard<std::mutex> lock(_pending_mtx);
	_pending_output += s;
}

std::string GenericShell::get_output()
{
	std::lock_guard<std::mutex> lock(_pending_mtx);
	std::string result = _pending_output;
	_pending_output.clear();
	return result;
}

std::string GenericShell::poll_output()
{
	// If there's pending output, return that.
	std::string pend(get_output());
	if (0 < pend.size()) return pend;

	// If we are here, there's no pending output. Does the evaluator
	// have anything for us?  Note that the ->poll_result() method
	// will block, if the evaluator is not done. Note that we must
	// do the get_output() again, else ctrl-C's will not be returned
	// in proper order to a telnet connection.
	std::string result(_evaluator->poll_result());
	if (0 < result.size())
		return get_output() + result;

	// If we are here, the evaluator is done. Return shell prompts.
	if (_eval_done) return "";
	finish_eval();

	if (_evaluator->input_pending())
	{
		if (show_output and show_prompt)
			return pending_prompt;
		else
			return "";
	}

	if (show_output or _evaluator->eval_error())
	{
		if (show_prompt) return normal_prompt;
	}
	return "";
}

/* ===================== END OF FILE ============================ */
