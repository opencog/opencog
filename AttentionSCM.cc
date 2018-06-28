/*
 * AttentionSCM.cc
 *
 * Guile Scheme bindings for the attentionbank
 * Copyright (C) 2014 Cosmo Harrigan
 * Copyright (c) 2008, 2014, 2015, 2018 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include "AttentionSCM.h"

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
	define_scheme_primitive("cog-av", &AttentionSCM::get_av, this, "attention-bank");
	define_scheme_primitive("cog-set-av!", &AttentionSCM::set_av, this, "attention-bank");
	define_scheme_primitive("cog-inc-vlti!", &AttentionSCM::inc_vlti, this, "attention-bank");
	define_scheme_primitive("cog-dec-vlti!", &AttentionSCM::dec_vlti, this, "attention-bank");

	// Attentional-focus-related stuff
	define_scheme_primitive("cog-update-af", &AttentionSCM::update_af, this, "attention-bank");
	define_scheme_primitive("cog-af-size", &AttentionSCM::af_size, this, "attention-bank");
	define_scheme_primitive("cog-set-af-size!", &AttentionSCM::set_af_size, this, "attention-bank");
	define_scheme_primitive("cog-stimulate", &AttentionSCM::stimulate, this, "attention-bank");

	// Pattern matching with filtering.
	define_scheme_primitive("cog-bind-af", &AttentionSCM::af_bindlink, this, "attention-bank");
}

AttentionSCM::~AttentionSCM()
{
}

AttentionValuePtr AttentionSCM::get_av(const Handle& h)
{
	return opencog::get_av(h);
}

Handle AttentionSCM::set_av(const Handle& h, const AttentionValuePtr& av)
{
	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-set-av!");
	attentionbank(atomspace).change_av(h, av);
	return h;
}

Handle AttentionSCM::inc_vlti(const Handle& h)
{
	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-inc-vlti!");
	attentionbank(atomspace).inc_vlti(h);
	return h;
}

Handle AttentionSCM::dec_vlti(const Handle& h)
{
	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-dec-vlti!");
	attentionbank(atomspace).dec_vlti(h);
	return h;
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
	HandleSeq attentionalFocus;

	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-af");

	Handle af_anchor = atomspace->add_node(ANCHOR_NODE, "*-attentional-focus-boundary-*");
	Handle af_key = atomspace->add_node(PREDICATE_NODE, "AttentionalFocus");

	attentionbank(atomspace).get_handle_set_in_attentional_focus(back_inserter(attentionalFocus));
	size_t isz = attentionalFocus.size();

	size_t N = isz;
	if (0 < n) N = n;
	if( N > isz)  N = isz;
	std::reverse(attentionalFocus.begin(), attentionalFocus.end());
	attentionalFocus.resize(N);

	std::vector<ProtoAtomPtr> af;
	for (Handle& h : attentionalFocus) {
		af.push_back(h);
	}

	af_anchor->setValue(af_key, createLinkValue(af));

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

Handle AttentionSCM::af_bindlink(const Handle& h)
{
	AtomSpace* atomspace = SchemeSmob::ss_get_env_as("cog-bind-af");
	return opencog::af_bindlink(atomspace, h);
}

extern "C" {
void opencog_attention_init(void);
};

void opencog_attention_init(void)
{
	static AttentionSCM atty;
}
#endif // HAVE_GUILE
