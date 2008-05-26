/*
 * SchemeSmob.c
 *
 * Scheme small objects.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include "Atom.h"
#include "SchemeSmob.h"

using namespace opencog;

/* ============================================================== */

bool SchemeSmob::is_inited = false;
scm_t_bits SchemeSmob::cog_tag;

SchemeSmob::SchemeSmob(void)
{
	if (!is_inited)
	{
		is_inited = true;
		init_smob_type();
		register_procs();
	}
}

/* ============================================================== */

SCM SchemeSmob::mark_cog(SCM node)
{
	printf("duuude! mark\n");
	return SCM_EOL;
}

size_t SchemeSmob::free_cog(SCM node)
{
	printf("duuude! free\n");
	return 0;
}

int SchemeSmob::print_cog(SCM node, SCM port, scm_print_state * ps)
{
	scm_puts ("#<atom>", port);
	printf("duuude! print\n");
	return 1; //non-zero meanss success
}

SCM SchemeSmob::equalp_cog(SCM a, SCM b)
{
	printf("duuude! equalp\n");
	return SCM_BOOL_F;
}

void SchemeSmob::init_smob_type(void)
{
	cog_tag = scm_make_smob_type ("opencog_atom", sizeof (scm_t_bits));
	scm_set_smob_mark (cog_tag, mark_cog);
	scm_set_smob_free (cog_tag, free_cog);
	scm_set_smob_print (cog_tag, print_cog);
	scm_set_smob_equalp (cog_tag, equalp_cog);
}



/* ============================================================== */

SCM SchemeSmob::ss_atom (SCM shandle)
{
	SCM smob;
printf ("duude handle=%d\n", scm_to_int(shandle));
	SCM_NEWSMOB (smob, cog_tag, shandle);
	return smob;
}

static SCM ss_hello (void)
{
	return scm_from_locale_string("Hello, world!");
}

#define C(X) ((SCM (*) ()) X)

void SchemeSmob::register_procs(void)
{
	scm_c_define_gsubr("cog-atom",                1, 0, 0, C(ss_atom));
	scm_c_define_gsubr("cog-hello",               0, 0, 0, ss_hello);
}

#endif
/* ===================== END OF FILE ============================ */
