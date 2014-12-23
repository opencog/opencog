/*
 * SchemePrimitive.cc
 *
 * Allow C++ code to be invoked from scheme -- 
 * by defining a scheme primitive function.
 *
 * Copyright (C) 2009 Linas Vepstas
 */

#include <exception>

#include "SchemeEval.h"
#include "SchemePrimitive.h"
#include "SchemeSmob.h"

using namespace opencog;

bool PrimitiveEnviron::is_inited = false;

#ifdef HAVE_GUILE2
 #define C(X) ((scm_t_subr) X)
#else
 #define C(X) ((SCM (*) ()) X)
#endif


/**
 * initialization code -- This is currently called under a 
 * lock, from SchemeEval::init()
 */
void PrimitiveEnviron::init(void)
{
	if (is_inited) return;
	is_inited = true;
	scm_c_define_gsubr("opencog-extension", 2,0,0, C(do_call));
}

PrimitiveEnviron::~PrimitiveEnviron() {}

void PrimitiveEnviron::do_register(const char *name, int nargs)
{
	// Now enter guile mode, and do the actual work there.
	tmp_name = name;
	tmp_nargs = nargs;
	scm_with_guile(c_wrap_register, this);
}

void *PrimitiveEnviron::c_wrap_register(void *p)
{
	PrimitiveEnviron *self = (PrimitiveEnviron *) p;
	self->really_do_register(self->tmp_name, self->tmp_nargs);
	return NULL;
}

/**
 * Create a new smob that will store a pointer to "this", which, in 
 * turn, holds a pointer to the C++ instance and the C++ method to be
 * invoked, when its called from scheme.  The evaluation of the scheme
 * function will actually end up calling "opencog-extension", which
 * passes this smob to do_call(). It will be do_call that then calls
 * the actual C++ function.
 *
 * Note that this method must be called in "guile mode".
 */
void PrimitiveEnviron::really_do_register(const char *name, int nargs)
{
	// Scheme garbage collection will be managing the lifecycle 
	scm_gc_register_collectable_memory (this, get_size(),
	                                    "opencog primitive environ");

	// The smob will hold a pointer to "this" -- the PrimitiveEnviron
	SCM smob;
	SCM_NEWSMOB (smob, SchemeSmob::cog_misc_tag, this);
	SCM_SET_SMOB_FLAGS(smob, SchemeSmob::COG_EXTEND);

	// We need to give the smob a unique name. Using addr of this is 
	// sufficient for this purpose.
#define BUFLEN 40
	char buff[BUFLEN];
	snprintf(buff, BUFLEN, "cog-prim-%p", this);
	scm_c_define (buff, smob);

	std::string wrapper = "(define (";
	wrapper += name;
	for (int i=0; i<nargs; i++)
	{
		wrapper += " ";
		char arg = 'a' + i;
		wrapper += arg;
	}
	wrapper += ") (opencog-extension ";
	wrapper += buff;
	wrapper += " (list";
	for (int i=0; i<nargs; i++)
	{
		wrapper += " ";
		char arg = 'a' + i;
		wrapper += arg;
	}
	wrapper += ")))";
	scm_c_eval_string(wrapper.c_str());
	// printf("Debug: do_regsiter %s\n", wrapper.c_str());
}

SCM PrimitiveEnviron::do_call(SCM sfe, SCM arglist)
{
	// First, get the environ.
	PrimitiveEnviron *fe = verify_pe(sfe, "opencog-extension");

	SCM rc = SCM_EOL;

	// If the C++ code throws any exceptions, and no one else
	// has caught them, then we have to catch them, and print 
	// an error message to the shell. Actually, we'll be
	// quasi-nice about this, and convert the C++ exception
	// into a scheme exception.
	try
	{
		rc = fe->invoke(arglist);
	}
	catch (const std::exception& ex)
	{
		const char *msg = ex.what();

		// Should we even bother to log this?
		logger().info("Guile caught C++ exception: %s", msg);

		// scm_misc_error(fe->get_name(), msg, SCM_EOL);
		scm_throw(
			scm_from_locale_symbol("C++-EXCEPTION"),
			scm_cons(
				scm_from_locale_string(fe->get_name()),
				scm_cons(
					scm_from_locale_string(msg),
					SCM_EOL)));
		// Hmm. scm_throw never returns.
	}
	catch (...)
	{
		// scm_misc_error(fe->get_name(), "unknown C++ exception", SCM_EOL);
		scm_error_scm(
			scm_from_locale_symbol("C++ exception"),
			scm_from_locale_string(fe->get_name()),
			scm_from_locale_string("unknown C++ exception"),
			SCM_EOL,
			SCM_EOL);
		logger().error("Guile caught unknown C++ exception");
	}
	scm_remember_upto_here_1(sfe);
	return rc;
}

PrimitiveEnviron * PrimitiveEnviron::verify_pe(SCM spe, const char *subrname)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, spe))
		scm_wrong_type_arg_msg(subrname, 1, spe, "opencog primitive function");

	scm_t_bits misctype = SCM_SMOB_FLAGS(spe);
	if (SchemeSmob::COG_EXTEND != misctype)
		scm_wrong_type_arg_msg(subrname, 1, spe, "opencog primitive function");

	PrimitiveEnviron * pe = (PrimitiveEnviron *) SCM_SMOB_DATA(spe);
	return pe;
}

