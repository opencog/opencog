/*
 * SchemeSmobAV.c
 *
 * Scheme small objects (SMOBS) for attention values.
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */
/**
 * Search for an attention value in a list of values.
 * Return the attention value if found, else return null.
 * Throw errors if the list is not stictly just key-value pairs
 */
AttentionValue * SchemeSmob::get_av_from_list(SCM slist)
{
	while (scm_is_pair(slist))
	{
		SCM sval = SCM_CAR(slist);
		if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sval))
		{
			scm_t_bits misctype = SCM_SMOB_FLAGS(sval);
			switch (misctype)
			{
				case COG_AV:
					return (AttentionValue *) SCM_SMOB_DATA(sval);
				default:
					break;
			}
		}

		slist = SCM_CDR(slist);
	}

	return NULL;
}

/* ============================================================== */

std::string SchemeSmob::av_to_string(const AttentionValue *av)
{
#define BUFLEN 120
	char buff[BUFLEN];

	snprintf(buff, BUFLEN, "(av %d %d %u)",
	         av->getSTI(), av->getLTI(), av->getVLTI());

	return buff;
}

/* ============================================================== */
/**
 * Take over memory management of an attention value
 */
SCM SchemeSmob::take_av (AttentionValue *av)
{
	scm_gc_register_collectable_memory (av,
	                 sizeof(*av), "opencog av");

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, av);
	SCM_SET_SMOB_FLAGS(smob, COG_AV);
	return smob;
}

/* ============================================================== */
/**
 * Create a new attention value, with indicated sti/lti/vlti
 */
SCM SchemeSmob::ss_new_av (SCM ssti, SCM slti, SCM svlti)
{
	if (!scm_is_integer(ssti)) {
		scm_wrong_type_arg_msg("cog-new-av", 1, ssti, "signed short");
	}
	if (!scm_is_integer(slti)) {
		scm_wrong_type_arg_msg("cog-new-av", 2, slti, "signed short");
	}
	if (!scm_is_integer(svlti)) {
		scm_wrong_type_arg_msg("cog-new-av", 3, svlti, "unsigned short");
	}
	AttentionValue::sti_t sti = scm_to_short(ssti);
	AttentionValue::lti_t lti = scm_to_short(slti);
	AttentionValue::vlti_t vlti = scm_to_ushort(svlti);
	AttentionValue *av = new AttentionValue(sti, lti, vlti);
	return take_av(av);
}

/* ============================================================== */
/**
 * Return true if the scm is an attention value
 */
SCM SchemeSmob::ss_av_p (SCM s)
{
	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
	{
		scm_t_bits misctype = SCM_SMOB_FLAGS(s);
		switch (misctype)
		{
			case COG_AV:
				return SCM_BOOL_T;

			default:
				return SCM_BOOL_F;
		}
	}
	return SCM_BOOL_F;
}

/* ============================================================== */

AttentionValue * SchemeSmob::verify_av(SCM sav, const char *subrname, int pos)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sav))
		scm_wrong_type_arg_msg(subrname, pos, sav, "opencog attention value");

	scm_t_bits misctype = SCM_SMOB_FLAGS(sav);
	if (COG_AV != misctype)
		scm_wrong_type_arg_msg(subrname, pos, sav, "opencog attention value");

	AttentionValue *av = (AttentionValue *) SCM_SMOB_DATA(sav);
	return av;
}

/**
 * Return association list holding contents of an attention value
 */
SCM SchemeSmob::ss_av_get_value (SCM s)
{
	AttentionValue *av = verify_av(s, "cog-av->alist");

	SCM sti = scm_from_short(av->getSTI());
	SCM lti = scm_from_short(av->getLTI());
	SCM vlti = scm_from_ushort(av->getVLTI());

	SCM ssti = scm_from_locale_symbol("sti");
	SCM slti = scm_from_locale_symbol("lti");
	SCM svlti = scm_from_locale_symbol("vlti");

	SCM rc = SCM_EOL;
	rc = scm_acons(svlti, vlti, rc);
	rc = scm_acons(slti, lti, rc);
	rc = scm_acons(ssti, sti, rc);
	return rc;
}

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
