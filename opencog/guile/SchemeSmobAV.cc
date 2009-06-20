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

	snprintf(buff, BUFLEN, "(av %d %d %d)", 
		av->getSTI(), av->getLTI(), av->getVLTI());
	std::string ret = buff;
	return ret;
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
	sti_t sti = scm_to_short(ssti);
	lti_t lti = scm_to_short(slti);
	vlti_t vlti = scm_to_short(svlti);

	AttentionValue *av = new AttentionValue(sti, lti, vlti);
	return take_av(av);
}

/* ============================================================== */
/**
 * Return true if the scm is a truth value
 */
SCM SchemeSmob::ss_tv_p (SCM s)
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

#if 0
/**
 * Return true if the scm is a truth value
 */
inline SCM SchemeSmob::tv_p (SCM s, AttentionValueType wanted)
{
	if (SCM_BOOL_F == ss_tv_p(s)) return SCM_BOOL_F;

	AttentionValue *tv = (AttentionValue *) SCM_SMOB_DATA(s);
	AttentionValueType tvt = tv->getType();
	if (wanted == tvt) return SCM_BOOL_T;
	return SCM_BOOL_F;
}

SCM SchemeSmob::ss_stv_p (SCM s)
{
	return tv_p(s, SIMPLE_TRUTH_VALUE);
}

SCM SchemeSmob::ss_ctv_p (SCM s)
{
	return tv_p(s, COUNT_TRUTH_VALUE);
}

SCM SchemeSmob::ss_itv_p (SCM s)
{
	return tv_p(s, INDEFINITE_TRUTH_VALUE);
}

/* ============================================================== */

AttentionValue * SchemeSmob::verify_tv(SCM stv, const char *subrname)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, stv))
		scm_wrong_type_arg_msg(subrname, 2, stv, "opencog truth value");

	scm_t_bits misctype = SCM_SMOB_FLAGS(stv);
	if (COG_AV != misctype)
		scm_wrong_type_arg_msg(subrname, 2, stv, "opencog truth value");

	AttentionValue *tv = (AttentionValue *) SCM_SMOB_DATA(stv);
	return tv;
}

/**
 * Return association list holding contents of a truth value
 */
SCM SchemeSmob::ss_tv_get_value (SCM s)
{
	AttentionValue *tv = verify_tv(s, "cog-tv->alist");
	AttentionValueType tvt = tv->getType();
	switch(tvt)
	{
		case SIMPLE_TRUTH_VALUE:
		{
			SimpleAttentionValue *stv = static_cast<SimpleAttentionValue *>(tv);
			SCM mean = scm_from_double(stv->getMean());
			SCM conf = scm_from_double(stv->getConfidence());
			SCM smean = scm_from_locale_symbol("mean");
			SCM sconf = scm_from_locale_symbol("confidence");
	
			SCM rc = SCM_EOL;
			rc = scm_acons(sconf, conf, rc);
			rc = scm_acons(smean, mean, rc);
			return rc;
		}
		case COUNT_TRUTH_VALUE:
		{
			CountAttentionValue *ctv = static_cast<CountAttentionValue *>(tv);
			SCM mean = scm_from_double(ctv->getMean());
			SCM conf = scm_from_double(ctv->getConfidence());
			SCM cont = scm_from_double(ctv->getCount());
			SCM smean = scm_from_locale_symbol("mean");
			SCM sconf = scm_from_locale_symbol("confidence");
			SCM scont = scm_from_locale_symbol("count");
	
			SCM rc = SCM_EOL;
			rc = scm_acons(scont, cont, rc), 
			rc = scm_acons(sconf, conf, rc);
			rc = scm_acons(smean, mean, rc);
			return rc;
		}
		case INDEFINITE_TRUTH_VALUE:
		{
			IndefiniteAttentionValue *itv = static_cast<IndefiniteAttentionValue *>(tv);
			SCM lower = scm_from_double(itv->getL());
			SCM upper = scm_from_double(itv->getU());
			SCM conf = scm_from_double(itv->getConfidence());
			SCM slower = scm_from_locale_symbol("lower");
			SCM supper = scm_from_locale_symbol("upper");
			SCM sconf = scm_from_locale_symbol("confidence");
	
			SCM rc = SCM_EOL;
			rc = scm_acons(sconf, conf, rc);
			rc = scm_acons(supper, upper, rc), 
			rc = scm_acons(slower, lower, rc);
			return rc;
		}
		case COMPOSITE_TRUTH_VALUE:
		{
			CompositeAttentionValue *mtv = static_cast<CompositeAttentionValue *>(tv);
			const AttentionValue &ptv = mtv->getVersionedAV(NULL_VERSION_HANDLE);
			AttentionValue *nptv = ptv.clone();
			SCM sptv = take_tv(nptv);

			SCM sprimary = scm_from_locale_symbol("primary");
			SCM sversion = scm_from_locale_symbol("versions");

			// Loop over all the version handles.
			SCM vers = SCM_EOL;
			int nvh = mtv->getNumberOfVersionedAVs();
			for (int i=0; i<nvh; i++)
			{
				VersionHandle vh = mtv->getVersionHandle(i);
				VersionHandle *nvh = new VersionHandle(vh);
				SCM svh = take_vh(nvh);

				const AttentionValue& vtv = mtv->getVersionedAV(vh);
				AttentionValue *nvtv = vtv.clone();
				SCM svtv = take_tv(nvtv);

				vers = scm_acons(svh, svtv, vers);
			}

			SCM rc = SCM_EOL;
			rc = scm_acons(sversion, vers, rc);
			rc = scm_acons(sprimary, sptv, rc);
			return rc;
		}
		default:
			return SCM_EOL;
	}
	return SCM_EOL;
}

#endif
#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
