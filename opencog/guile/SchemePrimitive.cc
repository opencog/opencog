/*
 * SchemePrimitive.cc
 *
 * Allow C++ code to be invoked from scheme -- 
 * by defining a scheme primitive function.
 *
 * Copyright (C) 2009 Linas Vepstas
 */

#include "SchemeEval.h"
#include "SchemePrimitive.h"
#include "SchemeSmob.h"

using namespace opencog;

bool PrimitiveEnviron::is_inited = false;

#define C(X) ((SCM (*) ()) X)

/**
 * initialization code -- XXX not thread-safe
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
	// Force initialization of the guile subsystem.
	SchemeEval::instance();

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
	snprintf(buff, BUFLEN, "cog-ext-%p", this);
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
	SCM rc = fe->invoke(arglist);
	return rc;
}

PrimitiveEnviron * PrimitiveEnviron::verify_pe(SCM spe, const char *subrname)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, spe))
		scm_wrong_type_arg_msg(subrname, 2, spe, "opencog primitive function");

	scm_t_bits misctype = SCM_SMOB_FLAGS(spe);
	if (SchemeSmob::COG_EXTEND != misctype)
		scm_wrong_type_arg_msg(subrname, 2, spe, "opencog primitive function");

	PrimitiveEnviron * pe = (PrimitiveEnviron *) SCM_SMOB_DATA(spe);
	return pe;
}

