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
#define IAC 0xff  // Telnet Interpret As Command
#define IP 0xf4   // Telnet IP Interrupt Process
#define AO 0xf5   // Telnet AO Abort Output
#define EL 0xf8   // Telnet EL Erase Line
#define WILL 0xfb // Telnet WILL
#define DO 0xfd   // Telnet DO
#define TIMING_MARK 0x6 // Telnet RFC 860 timing mark

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
	pending_prompt = "... ";
	abort_prompt = "asdf"; // simply reserve 4 chars
	abort_prompt[0] = IAC;
	abort_prompt[1] = WILL;
	abort_prompt[2] = TIMING_MARK;
	abort_prompt[3] = '\n';

	socket = nullptr;
	evalthr = nullptr;
	pollthr = nullptr;
	self_destruct = false;
	_eval_done = true;
	_evaluator = nullptr;
}

GenericShell::~GenericShell()
{
	if (evalthr)
	{
		logger().debug("[GenericShell] dtor, wait for eval thread 0x%x.",
		               evalthr->native_handle());
		evalthr->join();
		logger().debug("[GenericShell] dtor, joined eval thread");
		delete evalthr;
		evalthr = nullptr;
	}

	if (pollthr)
	{
		logger().debug("[GenericShell] dtor, wait for writer thread 0x%x.",
		               pollthr->native_handle());
		pollthr->join();
		delete pollthr;
		pollthr = nullptr;
	}

	if (_evaluator) _evaluator->clear_pending();
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



// Implementation requirements:
//
// 1) We want all evaluations to be carried out in serial order,
//    so that the previous expression is full evaluated before the
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
	// The first time through, there might not be an evaluator yet.
	// Create one now. We cannot do this in the constructor, for a
	// rather subtle reason: multiple calls to the constructor may
	// run in the same thread, resulting in multiple shells sharing
	// the same evaluator, which leads to a badness. We really want
	// each unique instance of the shell to have it's own evaluator,
	// and so we defer allocation until this point.
	//
	// The ctor runs in the main cogserver request-processor thread.
	// Calling get_evaluator() there would always return the same
	// single instance of the evaluator (because there can be at most
	// just one evaluator per thread).
	if (nullptr == _evaluator)
	{
		_evaluator = get_evaluator();
		_evaluator->clear_pending();
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


	// Run the evaluator (in a different thread)
	poll_needed = false;
	line_discipline(expr);

	// Avoid polling, if an evaluation thread was not created. This is
	// used to handle interrupts (control-c's).
	if (not poll_needed)
	{
		std::string retstr = poll_output();
		socket->Send(retstr);
	}
	else
	{
		// Poll for output from the evaluator, and send back results.
		auto poll_wrapper = [&](void)
		{
			std::string retstr = poll_output();
			while (0 < retstr.size())
			{
				socket->Send(retstr);
				retstr = poll_output();
			}
		};

		// Always wait for the previous poll of results to complete, before
		// starting the next one.  The goal here is to keep results
		// serialized on the socket, so that chronologically-earlier
		// results are written to the socket in order, before the newer
		// results.
		if (pollthr)
		{
			pollthr->join();
			delete pollthr;
		}
		sched_yield();
		pollthr = new std::thread(poll_wrapper);
	}


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
		do_eval("\n");
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
			if ((IP == c) || (AO == c))
			{
				_evaluator->interrupt();
				_evaluator->clear_pending();
				put_output(abort_prompt);
				return;
			}

			// Erase line -- just ignore this line.
			if (EL == c)
			{
				put_output(get_prompt());
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
		_evaluator->interrupt();
		_evaluator->clear_pending();
		put_output("\n");
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
		self_destruct = true;
		put_output("");
		if (show_prompt)
			put_output("Exiting the shell\n");
		return;
	}

	/*
	 * The newline is always cut. Re-insert it; otherwise, comments
	 * within procedures will have the effect of commenting out the
	 * rest of the procedure, leading to garbage.
	 * (This is a pointless string copy, it should be eliminated.)
	 */
	std::string input = expr + "\n";
	do_eval(input);
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
	OC_ASSERT(not _eval_done, "Bad evaluator flag state!");
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
/**
 * Evaluate the expression. Assumes line discipline was already done.
 */
void GenericShell::do_eval(const std::string &input)
{
	// Always wait for the previous evaluation to complete, before
	// starting the next one.  That is, evaluations are always
	// explicitly serialized.
	//
	// ... and even if they were not, we cannot use the same evaluator
	// in two different threads at the same time; a single evaluator is
	// not thread-safe against itself. So always wait for the previous
	// evaluation thread to finish, before we go at it again.
	if (evalthr)
	{
		evalthr->join();
		delete evalthr;
		evalthr = nullptr;
	}

	// Wait for the polling thread to finish also, as otherwise a new
	// evaluation might be started before polling for the last one has
	// finished.  The new evaluation might clobber previous results.
	if (pollthr)
	{
		pollthr->join();
		delete pollthr;
		pollthr = nullptr;
	}

	start_eval();
	poll_needed = true;

	auto eval_wrapper = [&](const std::string& in)
	{
		thread_init();
		_evaluator->begin_eval();
		_evaluator->eval_expr(in);
	};

	evalthr = new std::thread(eval_wrapper, input);
}

void GenericShell::thread_init(void)
{
	/* No-op. The Scheme shell sets the current atomspace here */
}

/* ============================================================== */

void GenericShell::put_output(const std::string& s)
{
	pending_output += s;	
}

std::string GenericShell::poll_output()
{
	// If there's pending output, return that.
	if (0 < pending_output.size())
	{
		std::string result = pending_output;
		pending_output.clear();
		return result;
	}

	// If we are here, there's no pending output. Does the
	// evaluator have anything for us?
	std::string result = _evaluator->poll_result();
	if (0 < result.size())
		return result;

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
