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


static SCM my_catch_handler (void *data, SCM tag, SCM throw_args)
{
	printf ("duude catchy this=%p\n", data);
	return SCM_EOL;
}

static SCM my_preunwind_proc (void *handler_data,
                              SCM key,
                              SCM parameters)
{
	/* Capture the stack here: */
	*(SCM *)handler_data = scm_make_stack (SCM_BOOL_T, SCM_EOL);
}
     
/**
 * Evaluate the expression
 */
char * SchemeShell::eval(const char * expr)
{
	SCM captured_stack = SCM_BOOL_F;

	scm_c_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) expr,
	            my_catch_handler, this,
	            my_preunwind_proc, &captured_stack);

	if (captured_stack != SCM_BOOL_F)
	{
		printf("duude got stack too\n");
	}
	SCM outstr = scm_get_output_string(string_outport);
	if (scm_string_p(outstr)) 
	{
		return scm_to_locale_string(outstr);
	}

	return strdup("Error: bad scheme output");
}

#endif
