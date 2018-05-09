/*
 * AttentionSCM.cc
 *
 * Guile Scheme bindings for the attentionbank
 * Copyright (C) 2014 Cosmo Harrigan
 * Copyright (c) 2008, 2014, 2015, 2018 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/guile/SchemePrimitive.h>

namespace opencog {

class AttentionSCM
{
	protected:
		static void* init_in_guile(void*);
		static void init_in_module(void*);
		void init(void);
	public:
		AttentionSCM(void);
		~AttentionSCM();

		int af_size(void);
		int set_af_size(int);
};

}

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

// ========================================================

AttentionSCM::AttentionSCM(void)
{
	static bool is_init = false;
	if (is_init) return;
	is_init = true;
	scm_with_guile(init_in_guile, this);
}


void* AttentionSCM::init_in_guile(void* self)
{
	scm_c_define_module("opencog attention-bank", init_in_module, self);
	scm_c_use_module("opencog attention-bank");
	return NULL;
}

void AttentionSCM::init_in_module(void* data)
{
	AttentionSCM* self = (AttentionSCM*) data;
	self->init();
}

/// This is called while (opencog attention-bank) is the current module.
/// Thus, all the definitions below happen in that module.
void AttentionSCM::init(void)
{
	define_scheme_primitive("cog-af-size", &AttentionSCM::af_size, this, "attention-bank");
	define_scheme_primitive("cog-set-af-size!", &AttentionSCM::set_af_size, this, "attention-bank");
}

AttentionSCM::~AttentionSCM()
{
}

/**
 *   Return AttentionalFocus Size
 **/
int AttentionSCM::af_size(void)
{
    AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-af-size");
    return attentionbank(atomspace).get_af_size();
}

/**
 * Set AttentionalFocus Size
 */
int AttentionSCM::set_af_size (int ssize)
{
    AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-set-af-size!");

    attentionbank(atomspace).set_af_size(ssize);
    return attentionbank(atomspace).get_af_size();
}

#ifdef FOO
xxxxxxxxxxxxxxxxxxxxxxx
/**
 * Return the list of top n atoms in the AttentionalFocus or
 * return all atoms in the AF if n is unspecified or is larger
 * than the AF size.
 */
SCM SchemeSmob::ss_af (SCM n)
{
	AtomSpace* atomspace = ss_get_env_as("cog-af");
	HandleSeq attentionalFocus;
	attentionbank(atomspace).get_handle_set_in_attentional_focus(back_inserter(attentionalFocus));
	size_t isz = attentionalFocus.size();
	if (0 == isz) return SCM_EOL;

	SCM head = SCM_EOL;
	size_t N = isz;
	if( SCM_UNDEFINED != n) N = scm_to_uint(n);
	if( N > isz)  N = isz;
	for (size_t i = isz - N; i < isz; i++) {
		Handle hi = attentionalFocus[i];
		SCM smob = handle_to_scm(hi);
		head = scm_cons(smob, head);
	}

	return head;
}

/**
 *  Stimulate an atom with given stimulus amount.
 */
SCM SchemeSmob::ss_stimulate (SCM satom, SCM sstimulus)
{
	Handle h(scm_to_handle(satom));
	double stimulus = scm_to_double(sstimulus);
	AtomSpace* atomspace = ss_get_env_as("cog-stimulate");
	attentionbank(atomspace).stimulate(h, stimulus);
	return satom;
}
#endif


extern "C" {
void opencog_attention_init(void);
};

void opencog_attention_init(void)
{
	static AttentionSCM atty;
}
#endif // HAVE_GUILE
