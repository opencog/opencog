/*
 * AttentionSCM.cc
 *
 * Guile Scheme bindings for the attentionbank
 * Copyright (c) 2008, 2014, 2015, 2018 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include <opencog/guile/SchemeModule.h>

namespace opencog {

class AttentionSCM : public ModuleWrap
{
	protected:
		virtual void init(void);
		static std::vector<FunctionWrap*> _binders;
	public:
		AttentionSCM(void);
		~AttentionSCM();
};

}

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeSmob.h>

#include "AttentionBank.h"

using namespace opencog;

// ========================================================

// XXX HACK ALERT This needs to be static, in order for python to
// work correctly.  The problem is that python keeps creating and
// destroying this class, but it expects things to stick around.
// Oh well. I guess that's OK, since the definition is meant to be
// for the lifetime of the process, anyway.
std::vector<FunctionWrap*> AttentionSCM::_binders;

AttentionSCM::AttentionSCM(void) :
	ModuleWrap("opencog attention")
{}

/// This is called while (opencog attention) is the current module.
/// Thus, all the definitions below happen in that module.
void AttentionSCM::init(void)
{
	// Run implication, assuming that the first argument is a handle to a
	// BindLink containing variables, a pattern and a rewrite rules.
	// Returns the first N matches, assuming that N is the second argument.
	_binders.push_back(new FunctionWrap(bindlink,
	                   "cog-bind-first-n", "query"));

	// Attentional Focus function
	_binders.push_back(new FunctionWrap(af_bindlink,
	                   "cog-bind-af", "query"));

	// A bindlink that returns a TV
	_binders.push_back(new FunctionWrap(do_satlink,
	                   "cog-satisfy", "query"));

	// Finds set of all variable groundings, assuming that the first
	// argument is a handle to pattern. Returns the first N matches,
	// assuming that N is the second argument.
	_binders.push_back(new FunctionWrap(satisfying_set,
	                   "cog-satisfying-set-first-n", "query"));

	// Rule recognition.
	_binders.push_back(new FunctionWrap(recognize,
	                   "cog-recognize", "query"));
}

AttentionSCM::~AttentionSCM()
{
#if PYTHON_BUG_IS_FIXED
	for (FunctionWrap* pw : _binders)
		delete pw;
#endif
}


extern "C" {
void opencog_attention_init(void);
};

void opencog_attention_init(void)
{
	static AttentionSCM atty;
	atty.module_init();
}
#endif // HAVE_GUILE
