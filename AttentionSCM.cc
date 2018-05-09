/*
 * AttentionSCM.cc
 *
 * Guile Scheme bindings for the attentionbank
 * Copyright (C) 2014 Cosmo Harrigan
 * Copyright (c) 2008, 2014, 2015, 2018 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include <opencog/atoms/base/Link.h>
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

		Handle update_af(int);
		int af_size(void);
		int set_af_size(int);
		Handle stimulate (const Handle&, double);
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
	define_scheme_primitive("cog-update-af", &AttentionSCM::update_af, this, "attention-bank");
	define_scheme_primitive("cog-af-size", &AttentionSCM::af_size, this, "attention-bank");
	define_scheme_primitive("cog-set-af-size!", &AttentionSCM::set_af_size, this, "attention-bank");
	define_scheme_primitive("cog-stimulate", &AttentionSCM::stimulate, this, "attention-bank");
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

/**
 * Return the list of top n atoms in the AttentionalFocus or
 * return all atoms in the AF if n is unspecified or is larger
 * than the AF size.
 */
Handle AttentionSCM::update_af(int n)
{
	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-af");

	Handle af_anchor = atomspace->add_node(ANCHOR_NODE,
	                                "*-attentional-focus-boundary-*");

	// Get the atoms that were previously in attention focus
	IncomingSet paf(af_anchor->getIncomingSetByType(MEMBER_LINK));
	HandleSet prev_af;
	for (const LinkPtr& lp: paf)
	{
		Handle h(lp->getOutgoingAtom(0));
		if (h != af_anchor)
			prev_af.insert(h);
	}

	HandleSeq attentionalFocus;
	attentionbank(atomspace).get_handle_set_in_attentional_focus(back_inserter(attentionalFocus));
	size_t isz = attentionalFocus.size();

	size_t N = isz;
	if (0 < n) N = n;
	if( N > isz)  N = isz;

	/* Add all the atoms in the current attentional focus that are not
	 * already there. */
	HandleSet curr_af;
	for (size_t i = isz - N; i < isz; i++)
	{
		Handle hi = attentionalFocus[i];
		auto gone = prev_af.find(hi);
		if (prev_af.end() == gone)
		{
			atomspace->add_link(MEMBER_LINK, hi, af_anchor);
		}
		else
		{
			curr_af.insert(hi);
		}
	}

	/* Remove the atoms no longer in attentional focus */
	for (const Handle& h: prev_af)
	{
		auto gone = curr_af.find(h);
		if (curr_af.end() == gone)
		{
			atomspace->extract_atom(h);
		}
	}

	return af_anchor;
}

/**
 *  Stimulate an atom with given stimulus amount.
 */
Handle AttentionSCM::stimulate (const Handle& h, double stimulus)
{
	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-stimulate");
	attentionbank(atomspace).stimulate(h, stimulus);
	return h;
}

extern "C" {
void opencog_attention_init(void);
};

void opencog_attention_init(void)
{
	static AttentionSCM atty;
}
#endif // HAVE_GUILE
