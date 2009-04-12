/*
 * SchemeSmobVH.c
 *
 * Scheme small objects (SMOBS) for version handles.
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/VersionHandle.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */

#if 0
/**
 * Search for a truth value in a list of values.
 * Return the truth value if found, else return null.
 * Throw errors if the list is not stictly just key-value pairs
 */
TruthValue * SchemeSmob::get_tv_from_list(SCM slist)
{
	while (scm_is_pair(slist))
	{
		SCM sval = SCM_CAR(slist);
		if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sval))
		{
			scm_t_bits misctype = SCM_SMOB_FLAGS(sval);
			switch (misctype)
			{
				case COG_TV:
					return (TruthValue *) SCM_SMOB_DATA(sval);
				default:
					break;
			}
		}

		slist = SCM_CDR(slist);
	}

	return NULL;
}

#endif

/* ============================================================== */

std::string SchemeSmob::vh_to_string(const VersionHandle *vh)
{
#define BUFLEN 120
	char buff[BUFLEN];

	std::string ret = "(vh ";
	ret += VersionHandle::indicatorToStr(vh->indicator);
	snprintf(buff, BUFLEN, " %lu)", vh->substantive.value());
	ret += buff;
	return ret;
}

#if 0
/* ============================================================== */
/**
 * Create a new simple truth value, with indicated mean and confidence.
 */
SCM SchemeSmob::take_tv (TruthValue *tv)
{
	scm_gc_register_collectable_memory (tv,
	                 sizeof(*tv), "opencog tv");

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, tv);
	SCM_SET_SMOB_FLAGS(smob, COG_TV);
	return smob;
}

/* ============================================================== */
/**
 * Create a new simple truth value, with indicated mean and confidence.
 */
SCM SchemeSmob::ss_new_stv (SCM smean, SCM sconfidence)
{
	double mean = scm_to_double(smean);
	double confidence = scm_to_double(sconfidence);

	float cnt = SimpleTruthValue::confidenceToCount(confidence);
	TruthValue *tv = new SimpleTruthValue(mean, cnt);
	return take_tv(tv);
}


#endif 
/* ============================================================== */
/**
 * Return true if the scm is a version handle
 */
SCM SchemeSmob::ss_vh_p (SCM s)
{
	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
	{
		scm_t_bits misctype = SCM_SMOB_FLAGS(s);
		switch (misctype)
		{
			case COG_VH:
				return SCM_BOOL_T;

			default:
				return SCM_BOOL_F;
		}
	}
	return SCM_BOOL_F;
}

#if 0
/* ============================================================== */
/**
 * Return scheme-accessible numerical value of a truth value
 */
SCM SchemeSmob::ss_tv_get_value (SCM s)
{
	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
	{
		scm_t_bits misctype = SCM_SMOB_FLAGS(s);
		switch (misctype)
		{
			// Return association list
			case COG_TV:
			{
				TruthValue *tv;
				tv = (TruthValue *) SCM_SMOB_DATA(s);
				TruthValueType tvt = tv->getType();
				switch(tvt)
				{
					case SIMPLE_TRUTH_VALUE:
					{
						SimpleTruthValue *stv = static_cast<SimpleTruthValue *>(tv);
						SCM mean = scm_from_double(stv->getMean());
						SCM conf = scm_from_double(stv->getConfidence());
						SCM smean = scm_from_locale_symbol("mean");
						SCM sconf = scm_from_locale_symbol("confidence");
				
						return scm_cons2(
							scm_cons(smean, mean),
							scm_cons(sconf, conf), 
							SCM_EOL);
					}
					default:
						return SCM_EOL;
				}
			}
			default:
				return SCM_EOL;
		}
	}
	return SCM_EOL;
}
#endif

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
