/*
 * SchemeSmobTV.c
 *
 * Scheme small objects (SMOBS) for truth values.
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

SCM SchemeSmob::mark_misc(SCM misc_smob)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(misc_smob);

	switch (misctype)
	{
		case COG_SIMPLE_TV: // Nothing to do here ...
		case COG_COUNT_TV:
		case COG_INDEFINITE_TV:
			return SCM_BOOL_F;
		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to mark this type: %d\n",
			        (int) misctype);
			break;
	}

	return SCM_BOOL_F;
}

/**
 * Free the memory associated with an opencog guile object.
 * This routine is called by the guile garbage collector, from time to
 * time. For testing purposes, you can force the garbage collector to
 * run by saying (gc), while, for stats, try (gc-stats) and
 * (gc-live-object-stats). The later should show both "opencog-handle"
 * and "opencog-misc" stats.
 */
size_t SchemeSmob::free_misc(SCM node)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(node);

	switch (misctype)
	{
		case COG_SIMPLE_TV:
			SimpleTruthValue *stv;
			stv = (SimpleTruthValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (stv,
			                  sizeof(SimpleTruthValue), "opencog simple tv");
			delete stv;
			return 0;

		case COG_COUNT_TV:
			CountTruthValue *ctv;
			ctv = (CountTruthValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (ctv,
			                  sizeof(CountTruthValue), "opencog count tv");
			delete ctv;
			return 0;

		case COG_INDEFINITE_TV:
			IndefiniteTruthValue *itv;
			itv = (IndefiniteTruthValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (itv,
			                  sizeof(IndefiniteTruthValue), "opencog indefinite tv");
			delete itv;
			return 0;

		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to free this type: %d\n",
			        (int) misctype);
			break;
	}
	return 0;
}

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
			free(key);
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
#endif

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
				case COG_SIMPLE_TV:
				{
					SimpleTruthValue *stv = (SimpleTruthValue *) SCM_SMOB_DATA(sval);
					return static_cast<TruthValue *>(stv);
				}
				case COG_COUNT_TV:
				{
					CountTruthValue *ctv = (CountTruthValue *) SCM_SMOB_DATA(sval);
					return static_cast<TruthValue *>(ctv);
				}
				case COG_INDEFINITE_TV:
				{
					IndefiniteTruthValue *itv = (IndefiniteTruthValue *) SCM_SMOB_DATA(sval);
					return static_cast<TruthValue *>(itv);
				}
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
			snprintf(buff, BUFLEN, "%.8g)", itv->getConfidence());
			ret += buff;
			return ret;
		}
		default:
			return ret;
	}
}

std::string SchemeSmob::misc_to_string(SCM node)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(node);
	switch (misctype)
	{
		case COG_SIMPLE_TV:
		case COG_COUNT_TV:
		case COG_INDEFINITE_TV:
			TruthValue *tv;
			tv = (TruthValue *) SCM_SMOB_DATA(node);
			return tv_to_string(tv);

		default:
			return "#<unknown opencog type>\n";
	}
	return "";
}

/* ============================================================== */
/**
 * Create a new simple truth value, with indicated mean and confidence.
 */
SCM SchemeSmob::take_stv (SimpleTruthValue *stv)
{
	scm_gc_register_collectable_memory (stv,
	                 sizeof(SimpleTruthValue), "opencog simple tv");

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, stv);
	SCM_SET_SMOB_FLAGS(smob, COG_SIMPLE_TV);
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
	SimpleTruthValue *stv = new SimpleTruthValue(mean, cnt);
	return take_stv(stv);
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
			case COG_SIMPLE_TV:
				return SCM_BOOL_T;

			default:
				return SCM_BOOL_F;
		}
	}
	return SCM_BOOL_F;
}

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
			case COG_SIMPLE_TV:
			{
				SimpleTruthValue *stv;
				stv = (SimpleTruthValue *) SCM_SMOB_DATA(s);
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
	return SCM_EOL;
}

#endif
/* ===================== END OF FILE ============================ */
