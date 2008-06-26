/*
 * SchemeShell.c
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>
#include <libguile/backtrace.h>

#include "SchemeShell.h"
#include "SchemeSmob.h"

using namespace opencog;

bool SchemeShell::is_inited = false;

SchemeShell::SchemeShell(void)
{
	if (!is_inited)
	{
		is_inited = true;
		scm_init_guile();
		scm_init_debug();
		scm_init_backtrace();
	}

	funcs = new SchemeSmob();
	pending_input = false;
	input_line = "";
	normal_prompt = "guile> ";
	pending_prompt = "... ";
}

/* ============================================================== */

std::string SchemeShell::prt(SCM node)
{
	if (scm_is_pair(node))
	{
		std::string str = "(";
      SCM node_list = node;
		const char * sp = "";
      do
      {
			str += sp;
			sp = " ";
         node = SCM_CAR (node_list);
         str += prt (node);
         node_list = SCM_CDR (node_list);
      }
      while (scm_is_pair(node_list));
		str += ")";
		return str;
   }
	else if (scm_is_true(scm_symbol_p(node))) 
	{
		node = scm_symbol_to_string(node);
		char * str = scm_to_locale_string(node);
		std::string rv = "'";
		rv += str;
		free(str);
		return rv;
	}
	else if (scm_is_true(scm_string_p(node))) 
	{
		char * str = scm_to_locale_string(node);
		std::string rv = "\"";
		rv += str;
		rv += "\"";
		free(str);
		return rv;
	}

	else if (SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, node))
	{
		return SchemeSmob::to_string(node);
	}

	else if (scm_is_true(scm_integer_p(node))) 
	{
		char buff[20];
		snprintf (buff, 20, "%ld", scm_to_long(node));
		return buff;
	}
	else if (scm_is_true(scm_char_p(node))) 
	{
		std::string rv;
		rv = (char) scm_to_char(node);
		return rv;
	}
	else if (scm_is_true(scm_boolean_p(node))) 
	{
		if (scm_to_bool(node)) return "#t";
		return "#f";
	}
	else if (scm_is_true(scm_null_p(node))) 
	{
		return "nil";
	}
#if 0
	else if (scm_is_true(scm_procedure_p(node))) 
	{
		return "procedure";
	}
	else if (scm_subr_p(node)) 
	{
		return "subr";
	}
	else if (scm_is_true(scm_operator_p(node))) 
	{
		return "operator";
	}
	else if (scm_is_true(scm_entity_p(node))) 
	{
		return "entity";
	}
	else if (scm_is_true(scm_variable_p(node))) 
	{
		return "variable";
	}
#endif

	return "";
}

/* ============================================================== */

SCM SchemeShell::catch_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeShell *ss = (SchemeShell *)data;
	return ss->catch_handler(tag, throw_args);
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
		char c = input_line[input_line.length()-1];
		if ((0x6 == c) || (0x16 == c) || (0x18 == c) || (0x1b == c))
		{
			pending_input = false;
			input_line = "";
		}
		return SCM_EOL;
	}
	free(restr);

	// If its not a read error, then its a regular error; report it.
	caught_error = true;

	/* get string port into which we write the error message and stack. */
	error_string_port = scm_open_output_string();
	SCM port = error_string_port;

	if (scm_is_true(scm_list_p(throw_args)) && (scm_ilength(throw_args) >= 4))
	{
		SCM stack   = scm_make_stack (SCM_BOOL_T, SCM_EOL);
		SCM subr    = SCM_CAR (throw_args);
		SCM message = SCM_CADR (throw_args);
		SCM parts   = SCM_CADDR (throw_args);
		SCM rest    = SCM_CADDDR (throw_args);
		if (scm_is_true (stack))
		{
			SCM highlights;

			if (scm_is_eq (tag, scm_arg_type_key) ||
			    scm_is_eq (tag, scm_out_of_range_key))
				highlights = rest;
			else
				highlights = SCM_EOL;

			scm_puts ("Backtrace:\n", port);
			scm_display_backtrace_with_highlights (stack, port,
			      SCM_BOOL_F, SCM_BOOL_F, highlights);
			scm_newline (port);
		}
		scm_i_display_error (stack, port, subr, message, parts, rest);
	}
	else
	{
		printf("ERROR: thow args are unexpectedly short!\n");
	}

#if 0
	/* find the stack, and conditionally display it */
	SCM the_stack = scm_fluid_ref(SCM_CDR(scm_the_last_stack_fluid_var));
	if (the_stack != SCM_BOOL_F)
	{
		scm_display_backtrace(the_stack, port, SCM_UNDEFINED, SCM_UNDEFINED);
	}
#endif

	return SCM_BOOL_F;
}

/* ============================================================== */

/**
 * Evaluate the expression
 */
std::string SchemeShell::eval(const std::string &expr)
{
	input_line += expr;

	caught_error = false;
	pending_input = false;
	SCM rc = scm_internal_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) input_line.c_str(),
	            SchemeShell::catch_handler_wrapper, this);

	if (pending_input)
	{
		return pending_prompt;
	}
	pending_input = false;
	input_line = "";

	std::string rv;
	if (caught_error)
	{
		rc = scm_get_output_string(error_string_port);
		char * str = scm_to_locale_string(rc);
		rv = str;
		free(str);
		scm_close_port(error_string_port);
	}
	else
	{
		rv = prt(rc);
	}
	rv += "\n";
	rv += normal_prompt;
	return rv;
}

#endif
/* ===================== END OF FILE ============================ */
