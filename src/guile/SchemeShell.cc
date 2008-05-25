/*
 * SchemeShell.c
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <guile/gh.h>
#include <libguile.h>
#include <libguile/backtrace.h>

#include "SchemeShell.h"

using namespace opencog;

SchemeShell:SchemeShell(void)
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

void SchemeShell:register_procs(void)
{
	scm_c_define_gsubr("cog-hello",               0, 0, 0, ss_hello);
}
