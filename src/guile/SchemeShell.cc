/*
 * SchemeShell.c
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <guile/gh.h>
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

/**
 * Evaluate the expression
 */
std::string SchemeShell::eval(const std::string &expr)
{
	scm_c_eval_string(expr.c_str());

	// alternately, to catch any throws...
	// scm_internal_stack_catch (SCM_BOOL_T, (scm_t_catch_body) scm_c_eval_string,
	//            expr, (scm_t_catch_handler) my_catch_handler, expr);

	return "ola";
}

#endif
