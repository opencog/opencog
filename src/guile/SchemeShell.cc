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
	if (scm_is_true(scm_list_p(node))) 
	{
return "its a list";
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
	SCM the_stack;
	/* create string port into which we write the error message and
	   stack. */
	SCM port = scm_current_output_port();
port = string_outport;
	/* throw args seem to be: (FN FORMAT ARGS #f). split the pieces into
	   local vars. */
	if (scm_is_true(scm_list_p(throw_args)) && (scm_ilength(throw_args) >= 4))
	{
		SCM fn = scm_car(throw_args);
		SCM format = scm_cadr(throw_args);
		SCM args = scm_caddr(throw_args);
		SCM other_data = scm_car(scm_cdddr(throw_args));
		
		if (fn != SCM_BOOL_F)
		{ /* display the function name and tag */
			scm_puts("Function: ", port);
			scm_display(fn, port);
			scm_puts(", ", port);
			scm_display(tag, port);
			scm_newline(port);
		}

		if (scm_is_true(scm_string_p(format)))
		{ /* conditionally display the error message using format */
			scm_puts("Error: ", port);
			scm_display_error_message(format, args, port);
		}
		if (other_data != SCM_BOOL_F)
		{
			scm_puts("Other Data: ", port);
			scm_display(other_data, port);
			scm_newline(port);
			scm_newline(port);
		}
	}

	/* find the stack, and conditionally display it */
	the_stack = scm_fluid_ref(SCM_CDR(scm_the_last_stack_fluid_var));
	if (the_stack != SCM_BOOL_F)
	{
		scm_display_backtrace(the_stack, port, SCM_UNDEFINED, SCM_UNDEFINED);
	}

	return SCM_EOL;
}

/* ============================================================== */

static SCM my_preunwind_proc (void *handler_data,
                              SCM key,
                              SCM parameters)
{
	/* Capture the stack here: */
	*(SCM *)handler_data = scm_make_stack (SCM_BOOL_T, SCM_EOL);
	return SCM_EOL;
}
     
/**
 * Evaluate the expression
 */
std::string SchemeShell::eval(const std::string &expr)
{
	SCM captured_stack = SCM_BOOL_F;

	SCM rc = scm_c_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) expr.c_str(),
	            SchemeShell::catch_handler_wrapper, this,
	            my_preunwind_proc, &captured_stack);

	if (captured_stack != SCM_BOOL_F)
	{
		printf("duude got stack too\n");
	}

	return prt(rc);

///XXXX

	SCM outstr = scm_get_output_string(string_outport);
	if (scm_is_true(scm_string_p(outstr))) 
	{
		char * str =  scm_to_locale_string(outstr);
printf ("out duude=>>%s<<\n", str);
		std::string rv = str;
		free(str);
		return rv;
	}

	return "Error: bad scheme output";
}

#endif
