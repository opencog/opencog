/*
 * SchemeSmobTV.c
 *
 * Scheme small objects (SMOBS) for truth values.
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */

#ifdef USE_KEYWORD_LIST_NOT_USED
/**
 * Search for a truth value (demarked by #:tv) in a list of key-value
 * pairs.  Return the truth value if found, else return null.
 * Throw errors if the list is not stictly just key-value pairs
 *
 * XXX This code is not currently used, since it seems pointless
 * to have key-value pairs for this function. After all, an atom
 * can only have one truth value ever -- if we find a truth value, we
 * use it. We don't really need a key to tell us that its a truth value.
 * So punt, and get truth values implicitly. Meanwhile, this code is
 * stubbed out, for a rainy tay, in case we need to resurrect key-value
 * pairs in the future.
 */
static TruthValue *get_tv_from_kvp(SCM kvp, const char * subrname, int pos)
{
	if (!scm_is_pair(kvp)) return NULL;

	do
	{
		SCM skey = SCM_CAR(kvp);

		// Verify that the first item is a keyword.
		if (!scm_is_keyword(skey))
			scm_wrong_type_arg_msg(subrname, pos, skey, "keyword");

		skey = scm_keyword_to_symbol(skey);
		skey = scm_symbol_to_string(skey);
		char * key = scm_to_locale_string(skey);

		kvp = SCM_CDR(kvp);
		pos ++;
		if (!scm_is_pair(kvp))
		{
			scm_wrong_type_arg_msg(subrname, pos, kvp, "value following keyword");
		}

		if (0 == strcmp(key, "tv"))
		{
			SCM sval = SCM_CAR(kvp);
			scm_t_bits misctype = SCM_SMOB_FLAGS(sval);
			if (misctype != COG_SIMPLE_TV)
				scm_wrong_type_arg_msg(subrname, pos, sval, "opencog truth value");
			TruthValue *tv;
			tv = (TruthValue *) SCM_SMOB_DATA(sval);
			return tv;
		}
		free(key);

		kvp = SCM_CDR(kvp);
		pos ++;
	}
	while (scm_is_pair(kvp));

	return NULL;
}
#endif /* USE_KEYWORD_LIST_NOT_USED */

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

/* ============================================================== */

std::string SchemeSmob::tv_to_string(const TruthValue *tv)
{
#define BUFLEN 120
	char buff[BUFLEN];
	TruthValueType tvt = tv->getType();

	// They're only floats, not doubles, so print with 8 digits
	std::string ret = "";
	switch (tvt)
	{
		case SIMPLE_TRUTH_VALUE:
		{
			const SimpleTruthValue *stv = static_cast<const SimpleTruthValue *>(tv);
			snprintf(buff, BUFLEN, "(stv %.8g ", stv->getMean());
			ret += buff;
			snprintf(buff, BUFLEN, "%.8g)", stv->getConfidence());
			ret += buff;
			return ret;
		}
		case COUNT_TRUTH_VALUE:
		{
			const CountTruthValue *ctv = static_cast<const CountTruthValue *>(tv);
			snprintf(buff, BUFLEN, "(ctv %.8g ", ctv->getMean());
			ret += buff;
			snprintf(buff, BUFLEN, "%.8g ", ctv->getConfidence());
			ret += buff;
			snprintf(buff, BUFLEN, "%.8g)", ctv->getCount());
			ret += buff;
			return ret;
		}
		case INDEFINITE_TRUTH_VALUE:
		{
			const IndefiniteTruthValue *itv = static_cast<const IndefiniteTruthValue *>(tv);
			snprintf(buff, BUFLEN, "(itv %.8g ", itv->getL());
			ret += buff;
			snprintf(buff, BUFLEN, "%.8g ", itv->getU());
			ret += buff;
			snprintf(buff, BUFLEN, "%.8g)", itv->getConfidenceLevel());
			ret += buff;
			return ret;
		}
		default:
			return ret;
	}
}

/* ============================================================== */
/**
 * Take over memory management of a truth value
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

SCM SchemeSmob::ss_new_ctv (SCM smean, SCM sconfidence, SCM scount)
{
	double mean = scm_to_double(smean);
	double confidence = scm_to_double(sconfidence);
	double count = scm_to_double(scount);

	TruthValue *tv = new CountTruthValue(mean, confidence, count);
	return take_tv(tv);
}

SCM SchemeSmob::ss_new_itv (SCM slower, SCM supper, SCM sconfidence)
{
	double lower = scm_to_double(slower);
	double upper = scm_to_double(supper);
	double confidence = scm_to_double(sconfidence);

	TruthValue *tv = new IndefiniteTruthValue(lower, upper, confidence);
	return take_tv(tv);
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
			case COG_TV:
				return SCM_BOOL_T;

			default:
				return SCM_BOOL_F;
		}
	}
	return SCM_BOOL_F;
}

/**
 * Return true if the scm is a truth value
 */
inline SCM SchemeSmob::tv_p (SCM s, TruthValueType wanted)
{
	if (SCM_BOOL_F == ss_tv_p(s)) return SCM_BOOL_F;

	TruthValue *tv = (TruthValue *) SCM_SMOB_DATA(s);
	TruthValueType tvt = tv->getType();
	scm_remember_upto_here_1(s);
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

TruthValue * SchemeSmob::verify_tv(SCM stv, const char *subrname, int pos)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, stv))
		scm_wrong_type_arg_msg(subrname, pos, stv, "opencog truth value");

	scm_t_bits misctype = SCM_SMOB_FLAGS(stv);
	if (COG_TV != misctype)
		scm_wrong_type_arg_msg(subrname, pos, stv, "opencog truth value");

	TruthValue *tv = (TruthValue *) SCM_SMOB_DATA(stv);
	return tv;
}

/**
 * Return association list holding contents of a truth value
 */
SCM SchemeSmob::ss_tv_get_value (SCM s)
{
	TruthValue *tv = verify_tv(s, "cog-tv->alist");
	TruthValueType tvt = tv->getType();
	switch (tvt)
	{
		case SIMPLE_TRUTH_VALUE:
		{
			SimpleTruthValue *stv = static_cast<SimpleTruthValue *>(tv);
			SCM mean = scm_from_double(stv->getMean());
			SCM conf = scm_from_double(stv->getConfidence());
			SCM count = scm_from_double(stv->getCount());
			SCM smean = scm_from_locale_symbol("mean");
			SCM sconf = scm_from_locale_symbol("confidence");
			SCM scount = scm_from_locale_symbol("count");
	
			SCM rc = SCM_EOL;
			rc = scm_acons(sconf, conf, rc);
			rc = scm_acons(smean, mean, rc);
			rc = scm_acons(scount, count, rc);
			scm_remember_upto_here_1(s);
			return rc;
		}
		case COUNT_TRUTH_VALUE:
		{
			CountTruthValue *ctv = static_cast<CountTruthValue *>(tv);
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
			scm_remember_upto_here_1(s);
			return rc;
		}
		case INDEFINITE_TRUTH_VALUE:
		{
			IndefiniteTruthValue *itv = static_cast<IndefiniteTruthValue *>(tv);
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
			scm_remember_upto_here_1(s);
			return rc;
		}
		default:
			scm_remember_upto_here_1(s);
			return SCM_EOL;
	}
	scm_remember_upto_here_1(s);
	return SCM_EOL;
}

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
