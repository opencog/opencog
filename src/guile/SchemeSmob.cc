/*
 * SchemeSmob.c
 *
 * Scheme small objects.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include "Atom.h"
#include "SchemeShell.h"

using namespace opencog;

/* ============================================================== */

struct SchemeShell::cog
{
	Handle h;
	SCM name;
};

scm_t_bits SchemeShell::cog_tag;

SCM SchemeShell::mark_cog(SCM node)
{
	printf("duuude! mark\n");
	return SCM_EOL;
}

size_t SchemeShell::free_cog(SCM node)
{
	printf("duuude! free\n");
	return sizeof(struct SchemeShell::cog);
}

int SchemeShell::print_cog(SCM node, SCM um, scm_print_state * ps)
{
	printf("duuude! print\n");
	return 0;
}

SCM SchemeShell::equalp_cog(SCM a, SCM b)
{
	printf("duuude! equalp\n");
	return SCM_BOOL_F;
}

void SchemeShell::init_smob_type(void)
{
	cog_tag = scm_make_smob_type ("opencog_atom", sizeof (struct cog));
	scm_set_smob_mark (cog_tag, mark_cog);
	scm_set_smob_free (cog_tag, free_cog);
	scm_set_smob_print (cog_tag, print_cog);
	scm_set_smob_equalp (cog_tag, equalp_cog);
}



/* ============================================================== */

static SCM ss_hello (void)
{
	return scm_from_locale_string("Hello, world!");
}

void SchemeShell::register_procs(void)
{
	scm_c_define_gsubr("cog-hello",               0, 0, 0, ss_hello);
}

#endif
/* ===================== END OF FILE ============================ */
