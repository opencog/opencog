/*
 * SchemeShell.c
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
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

#ifdef HAVE_GUILE

#include <opencog/server/ConsoleSocket.h>
#include <opencog/util/Logger.h>
#include <opencog/util/platform.h>

#include "SchemeShell.h"

using namespace opencog;

// Some random RFC 854 characters
#define IAC 0xff  // Telnet Interpret As Command
#define IP 0xf4   // Telnet IP Interrupt Process
#define AO 0xf5   // Telnet AO Abort Output
#define EL 0xf8   // Telnet EL Erase Line
#define WILL 0xfb // Telnet WILL
#define DO 0xfd   // Telnet DO
#define TIMING_MARK 0x6 // Telnet RFC 860 timing mark

DECLARE_MODULE(SchemeShell);

SchemeShell::SchemeShell(void)
{
	show_output = true;
	normal_prompt = "guile> ";
	pending_prompt = "... ";
	abort_prompt = "asdf";
	abort_prompt[0] = IAC;
	abort_prompt[1] = WILL;
	abort_prompt[2] = TIMING_MARK;
	abort_prompt[3] = '\n';
	abort_prompt += normal_prompt;
	evaluator = NULL;
	holder = NULL;
}

void SchemeShell::init(void)
{
	if (!evaluator) evaluator = new SchemeEval();
	shellout_register();
}

SchemeShell::~SchemeShell()
{
	if (holder)
	{
		holder->SetShell(NULL);
		holder->AtomicInc(-1);
		holder = NULL;
	}
	shellout_unregister();
	if (evaluator) delete evaluator;
}

/**
 * Register this shell with the console.
 */
std::string SchemeShell::shellout(Request *req, std::list<std::string> args)
{
	SocketHolder *h = req->getSocketHolder();
	if (holder)
	{
		holder->SetShell(NULL);
		holder->AtomicInc(-1);
	}

	holder = h;
	holder->AtomicInc(+1);
	holder->SetShell(this);

	if (evaluator) evaluator->thread_init();
	return "Entering scheme shell; use ^D or a single . on a "
	       "line by itself to exit.";
}

/* ============================================================== */

void SchemeShell::hush_output(bool hush)
{
	show_output = !hush;
}

const std::string& SchemeShell::get_prompt(void)
{
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

void SchemeShell::eval(const std::string &expr, SocketHolder *h)
{
	// XXX A subtle but important point: the way that socket handling
	// works in OpenCog is that socket-listen/accept happens in one
	// thread, while socket receive is in another. In particular, the
	// constructor for this class, the init() method, and the shellout()
	// method run in a *different* thread than this method does.  Now, 
	// guile is thread-aware, but it has to be inited in each thread,
	// otherwise it crashes. Thus, we *must* init the evaluator here,
	// rather than earlier, because this is the first place where we
	// are gaurenteed to be in the right thread. (Its OK to re-init
	// multiple times).
	if (evaluator)	evaluator->thread_init();

	if (NULL == holder)
	{
		holder = h;
		holder->AtomicInc(+1);
	}
	std::string retstr = do_eval(expr);
	// logger().debug("[SchemeShell] response: [%s]", retstr.c_str());
	//
	holder->send(retstr);
}

/**
 * Evaluate the expression
 */
std::string SchemeShell::do_eval(const std::string &expr)
{
	size_t len = expr.length();
	if (0 == len) 
	{
		return get_prompt();
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
				evaluator->clear_pending();
				return abort_prompt;
			}

			// Erase line -- just ignore this line.
			if (EL == c)
			{
				return get_prompt();
			}
		}
		i--;
	}

	// Don't evaluate if the line is terminated by 
	// escape (^[), cancel (^X) or quit (^C)
	// These would typically be sent by netcat, and not telnet.
	unsigned char c = expr[len-1];
	if ((0x16 == c) || (0x18 == c) || (0x1b == c))
	{
		evaluator->clear_pending();
		return "\n" + normal_prompt;
	}

	// Look for either an isolated control-D, or a single period on a line
	// by itself. This means "leave the shell". We leave the shell by
	// unsetting the shell pointer in the ConsoleSocket.
	if ((false == evaluator->input_pending()) &&
	    ((0x4 == expr[len-1]) || ((1 == len) && ('.' == expr[0]))))
	{
		holder->SetShell(NULL);
		return "Exiting the scheme shell\n";
	}

	/* The #$%^& Alhem CSockets code cuts off the newline character.
	 * (It also leaks memory like a seive, 1/2 Gig in 20 seconds under
	 * the right conditions. Grrr)
	 *
	 * Re-insert it; otherwise, comments within procedures will
	 * have the effect of commenting out the rest of the procedure,
	 * leading to garbage.
	 *
	 * (This is a pointless string copy, it should be eliminated)
	 */
	std::string input = expr + "\n";

	std::string result = evaluator->eval(input.c_str());

	if (evaluator->input_pending())
	{
		if (show_output)
			return pending_prompt;
		else
			return "";
	}

	if (show_output || evaluator->eval_error())
	{
		result += normal_prompt;
		return result;
	}
	else
	{
		return "";
	}

}

#endif
/* ===================== END OF FILE ============================ */
