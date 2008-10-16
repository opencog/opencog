/*
 * SchemeShell.c
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include "SchemeShell.h"

#include <libguile.h>
#include <libguile/backtrace.h>
#include <libguile/debug.h>
#include <libguile/lang.h>

#include "platform.h"
#include "SchemeSmob.h"

using namespace opencog;

SchemeShell::SchemeShell(void)
{
	pending_input = false;
	show_output = true;
	input_line = "";
	normal_prompt = "guile> ";
	pending_prompt = "... ";

	outport = scm_open_output_string();
	scm_set_current_output_port(outport);
}

void SchemeShell::hush_output(bool hush)
{
	show_output = !hush;
}

/* ============================================================== */

SCM SchemeShell::preunwind_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeShell *ss = (SchemeShell *)data;
	return ss->preunwind_handler(tag, throw_args);
	return SCM_EOL;
}

SCM SchemeShell::catch_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeShell *ss = (SchemeShell *)data;
	return ss->catch_handler(tag, throw_args);
}

SCM SchemeShell::preunwind_handler (SCM tag, SCM throw_args)
{
	// We can only record the stack before it is unwound. 
	// The normal catch handler body runs only *after* the stack
	// has been unwound.
	captured_stack = scm_make_stack (SCM_BOOL_T, SCM_EOL);
	return SCM_EOL;
}

SCM SchemeShell::catch_handler (SCM tag, SCM throw_args)
{
	// Check for read error. If a read error, then wait for user to correct it.
	SCM re = scm_symbol_to_string(tag);
	char * restr = scm_to_locale_string(re);
	pending_input = false;
	if (0 == strcmp(restr, "read-error"))
	{
		pending_input = true;
		free(restr);

		// quit accumulating text on escape (^[), cancel (^X) or ^C
		char c = input_line[input_line.length()-2];
		if ((0x6 == c) || (0x16 == c) || (0x18 == c) || (0x1b == c))
		{
			pending_input = false;
			input_line = "";
		}
		return SCM_EOL;
	}

	// If its not a read error, then its a regular error; report it.
	caught_error = true;

	/* get string port into which we write the error message and stack. */
	error_string_port = scm_open_output_string();
	SCM port = error_string_port;

	if (scm_is_true(scm_list_p(throw_args)) && (scm_ilength(throw_args) >= 1))
	{
		long nargs = scm_ilength(throw_args);
		SCM subr    = SCM_CAR (throw_args);
		SCM message = SCM_EOL;
		if (nargs >= 2)
			message = SCM_CADR (throw_args);
		SCM parts   = SCM_EOL;
		if (nargs >= 3)
			parts   = SCM_CADDR (throw_args);
		SCM rest    = SCM_EOL;
		if (nargs >= 4)
			rest    = SCM_CADDDR (throw_args);

		if (scm_is_true (captured_stack))
		{
			SCM highlights;

			if (scm_is_eq (tag, scm_arg_type_key) ||
			    scm_is_eq (tag, scm_out_of_range_key))
				highlights = rest;
			else
				highlights = SCM_EOL;

			scm_puts ("Backtrace:\n", port);
			scm_display_backtrace_with_highlights (captured_stack, port,
			      SCM_BOOL_F, SCM_BOOL_F, highlights);
			scm_newline (port);
		}
		scm_display_error (captured_stack, port, subr, message, parts, rest);
	}
	else
	{
		scm_puts ("ERROR: thow args are unexpectedly short!\n", port);
	}
	scm_puts("ABORT: ", port);
	scm_puts(restr, port);
	free(restr);

	return SCM_BOOL_F;
}

/* ============================================================== */

/**
 * Evaluate the expression
 */
std::string SchemeShell::eval(const std::string &expr)
{
	input_line += expr;

	/* The #$%^& opecong command shell processor cuts off the 
	 * newline character. Re-insert it; otherwise, comments within
	 * procedures will have the effect of commenting out the rest
	 * of the procedure, leading to garbage.
	 */
	input_line += "\n";

	caught_error = false;
	pending_input = false;
#if 0
	SCM rc = scm_internal_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) input_line.c_str(),
	            SchemeShell::catch_handler_wrapper, this);
#endif
	captured_stack = SCM_BOOL_F;
	SCM rc = scm_c_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) input_line.c_str(),
	            SchemeShell::catch_handler_wrapper, this,
	            SchemeShell::preunwind_handler_wrapper, this);

	if (pending_input)
	{
		if (show_output)
			return pending_prompt;
		else
			return "";
	}
	pending_input = false;
	input_line = "";

	if (caught_error)
	{
		std::string rv;
		rc = scm_get_output_string(error_string_port);
		char * str = scm_to_locale_string(rc);
		rv = str;
		free(str);
		scm_close_port(error_string_port);

		scm_truncate_file(outport, scm_from_uint16(0));

		rv += "\n";
		rv += normal_prompt;
		return rv;
	}
	else
	{
		if (show_output)
		{
			std::string rv;
			// First, we get the contents of the output port,
			// and pass that on.
			SCM out = scm_get_output_string(outport);
			char * str = scm_to_locale_string(out);
			rv = str;
			free(str);
			scm_truncate_file(outport, scm_from_uint16(0));

			// Next, we append the "interpreter" output
			rv += evaluator.prt(rc);
			rv += "\n";

			rv += normal_prompt;
			return rv;
		}
		else
			return "";
	}
	return "#<Error: Unreachable statement reached>";
}

#endif
/* ===================== END OF FILE ============================ */
