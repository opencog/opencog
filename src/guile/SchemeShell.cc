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

		scm_init_strports();
		string_outport = scm_open_output_string();
		register_procs();
	}
}

static SCM ss_hello (void)
{
	printf("hello world\n");
	return SCM_EOL;
}

void SchemeShell::register_procs(void)
{
	scm_c_define_gsubr("cog-hello",               0, 0, 0, ss_hello);
}

/* ============================================================== */

std::string SchemeShell::prt(SCM node)
{
  if (scm_is_pair(node))
   {
std::string str = "";
printf("duude pair\n");
      SCM node_list = node;
      do
      {
         node = SCM_CAR (node_list);
         str += prt (node);
         node_list = SCM_CDR (node_list);
      }
      while (scm_is_pair(node_list));
      str += prt (node_list);
		return str;
   }
	else if (scm_is_true(scm_list_p(node))) 
	{
		std::string str = "";
		int len = scm_to_long(scm_length(node));
		if (0 == len) return "()";
printf("duuude len=%d\n", len);
		for (int k=0; k<len; k++)
      {
printf("duuude k=%d\n", k);
         SCM n = scm_list_ref(node, scm_from_int(k));
         str += prt (n);
      }
		return str;
	}
	else if (scm_is_true(scm_symbol_p(node))) 
	{
		node = scm_symbol_to_string(node);
		char * str = scm_to_locale_string(node);
		std::string rv = "YYY>";
		rv += str;
		free(str);
		return rv;
	}
	else if (scm_is_true(scm_string_p(node))) 
	{
		char * str = scm_to_locale_string(node);
		std::string rv = "SSS>";
		rv += str;
		free(str);
		return rv;
	}
	else if (scm_is_true(scm_integer_p(node))) 
	{
		char buff[20];
		snprintf (buff, 20, "NNN>%d", scm_to_long(node));
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
		return "(xxxnull)";
	}

	return "Error: unknown type";
}

/* ============================================================== */

SCM SchemeShell::catch_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeShell *ss = (SchemeShell *)data;
	return ss->catch_handler(tag, throw_args);
}

SCM SchemeShell::catch_handler (SCM tag, SCM throw_args)
{
	caught_error = true;

	/* get string port into which we write the error message and stack. */
	SCM port = string_outport;

	if (scm_is_true(scm_list_p(throw_args)) && (scm_ilength(throw_args) == 4))
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
printf("duuude unexpected\n");
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
	SCM captured_stack = SCM_BOOL_F;

	caught_error = false;
	SCM rc = scm_internal_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) expr.c_str(),
	            SchemeShell::catch_handler_wrapper, this);

	if (captured_stack != SCM_BOOL_F)
	{
		printf("duude got stack too\n");
	}

	if (caught_error)
	{
		rc = scm_get_output_string(string_outport);
	}

	return prt(rc);
}

#endif
/* ===================== END OF FILE ============================ */
