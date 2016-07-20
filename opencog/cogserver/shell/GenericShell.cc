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

	evaluator = NULL;
	socket = NULL;
	evalthr = NULL;
	self_destruct = false;
	do_async_output = false;
}

GenericShell::~GenericShell()
{
	if (evalthr)
	{
		evalthr->join();
		delete evalthr;
		evalthr = NULL;
	}
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

void GenericShell::sync_output(bool sync)
{
	do_async_output = !sync;
}

const std::string& GenericShell::get_prompt(void)
{
	static const std::string empty_prompt = "";
	if (!show_prompt) return empty_prompt;

	// Use different prompts, depending on whether there is pending
	// input or not.
	if (evaluator->input_pending())
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
	if (socket) socket->SetShell(NULL);

	socket = s;
	socket->SetShell(this);
}

/* ============================================================== */

static std::mutex _stdout_redirect_mutex;
static GenericShell* _redirector = nullptr;

void GenericShell::eval(const std::string &expr, ConsoleSocket *s)
{
	// XXX A subtle but important point: the way that socket handling
	// works in OpenCog is that socket-listen/accept happens in one
	// thread, while socket receive is in another. In particular, the
	// constructor for this class runs in a *different* thread than
	// this method does.
	if (NULL == socket)
	{
		socket = s;
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
#define PERFORM_STDOUT_DUPLICATION 1
#ifdef PERFORM_STDOUT_DUPLICATION
	// What used to be stdout will now go to the pipe.
	int pipefd[2];
	int stdout_backup = -1;
#endif // PERFORM_STDOUT_DUPLICATION

	// Launch the evaluator, possibly in a different thread,
	// and then send out whatever is reported back.
	line_discipline(expr);
	std::string retstr = poll_output();
	while (0 < retstr.size())
	{
		socket->Send(retstr);
		retstr = poll_output();
	}

#ifdef PERFORM_STDOUT_DUPLICATION
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
			char buf[4097];
			int nr = read(pipefd[0], buf, sizeof(buf)-1);
			OC_ASSERT(0 < rc, "GenericShell pipe read failure");
			while (0 < nr)
			{
				buf[nr] = 0;
				if (1 < nr or 0 != buf[0])
				{
					printf("%s", buf); // print to the cogservers stdout.
					socket->Send(buf);
				}
				nr = read(pipefd[0], buf, sizeof(buf)-1);
				OC_ASSERT(0 < rc, "GenericShell pipe read failure");
			}

			// Cleanup.
			close(pipefd[0]);
			close(stdout_backup);
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

/* ============================================================== */
/**
 * Handle special characters, evaluate the expression
 */
void GenericShell::line_discipline(const std::string &expr)
{
	size_t len = expr.length();

	logger().debug("[GenericShell] line disc: expr, len of %zd ='%s'",
		len, expr.c_str());

	// Make sure there is at least one character if we are checking
	// for abort, interrrupt, escape, etc.
	if (0 != len)
	{
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
					evaluator->clear_pending();
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
			evaluator->clear_pending();
			put_output("\n");
			put_output(normal_prompt);
			return;
		}

		// Look for either an isolated control-D, or a single period on a line
		// by itself. This means "leave the shell". We leave the shell by
		// unsetting the shell pointer in the ConsoleSocket.
		// 0x4 is ASCII EOT, which is what ctrl-D at keybd becomes.
		if ((false == evaluator->input_pending()) and
		    ((EOT == expr[len-1]) or ((1 == len) and ('.' == expr[0]))))
		{
			self_destruct = true;
			put_output("");
			if (show_prompt)
				put_output("Exiting the shell\n");
			return;
		}
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
/**
 * Evaluate the expression. Assumes line discipline was already done.
 */
void GenericShell::do_eval(const std::string &input)
{
	eval_done = false;
	evaluator->begin_eval(); // must be called in same thread as result_poll
	if (do_async_output)
	{
		auto async_wrapper = [&](GenericShell* p, const std::string& in)
		{
			thread_init();
			p->evaluator->eval_expr(in.c_str());
		};

		// We cannot use the same evaluator in two different threads
		// at the same time; a single evaluator is not thread-safe
		// against itself. So always wait for the previous thread to
		// finish, before we go at it again.
		if (evalthr)
		{
			evalthr->join();
			delete evalthr;
		}
		evalthr = new std::thread(async_wrapper, this, input);
	}
	else
	{
		evaluator->eval_expr(input.c_str());
	}
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
	std::string result = evaluator->poll_result();
	if (0 < result.size())
		return result;

	// If we are here, the evaluator is done. Return shell prompts.
	if (eval_done) return "";
	eval_done = true;

	if (evaluator->input_pending())
	{
		if (show_output && show_prompt)
			return pending_prompt;
		else
			return "";
	}

	if (show_output || evaluator->eval_error())
	{
		if (show_prompt) return normal_prompt;
	}
	return "";
}

/* ===================== END OF FILE ============================ */
