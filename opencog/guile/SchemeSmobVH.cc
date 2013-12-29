/*
 * SchemeSmobVH.c
 *
 * Scheme small objects (SMOBS) for version handles.
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

#include <opencog/atomspace/VersionHandle.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */

std::string SchemeSmob::vh_to_string(const VersionHandle *vh)
{
	std::string ret = "(vh \"";
	ret += VersionHandle::indicatorToStr(vh->indicator);
	ret += "\" " + vh->substantive;
	return ret;
}

/* ============================================================== */
/**
 * Take over memory management of a version handle.
 */
SCM SchemeSmob::take_vh (VersionHandle *vh)
{
	scm_gc_register_collectable_memory (vh,
	                 sizeof(*vh), "opencog vh");

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, vh);
	SCM_SET_SMOB_FLAGS(smob, COG_VH);
	return smob;
}

/* ============================================================== */
/**
 * Create a new version handle
 */
SCM SchemeSmob::ss_new_vh (SCM sind, SCM shandle)
{
	Handle h = verify_handle(shandle, "cog-new-vh", 2);
	std::string ind_name = verify_string (sind, "cog-new-vh", 3,
		"indicator for the version handle");

	IndicatorType ind;
	try
	{
		ind = VersionHandle::strToIndicator(ind_name.c_str());
	}
	catch (InvalidParamException e)
	{
		scm_wrong_type_arg_msg("cog-new-vh", 1, sind,
			"version handle indicator string name");
	}

	VersionHandle *vh = new VersionHandle(ind, h);
	return take_vh(vh);
}

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

/* ============================================================== */

VersionHandle * SchemeSmob::verify_vh(SCM svh, const char *subrname, int pos)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, svh))
		scm_wrong_type_arg_msg(subrname, pos, svh, "opencog version handle");

	scm_t_bits misctype = SCM_SMOB_FLAGS(svh);
	if (COG_VH != misctype)
		scm_wrong_type_arg_msg(subrname, pos, svh, "opencog version handle");

	VersionHandle *vh = (VersionHandle *) SCM_SMOB_DATA(svh);
	return vh;
}

/* ============================================================== */
/**
 * Return association list for the version handle.
 */
SCM SchemeSmob::ss_vh_get_value (SCM s)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
	{
		return SCM_EOL;
	}

	scm_t_bits misctype = SCM_SMOB_FLAGS(s);
	if (COG_VH != misctype) return SCM_EOL;

	// Return association list
	VersionHandle *vh;
	vh = (VersionHandle *) SCM_SMOB_DATA(s);

	const char * str = VersionHandle::indicatorToStr(vh->indicator);
	SCM ind = scm_from_locale_string(str);
	UUID uuid = vh->substantive.value();
	SCM suuid = scm_from_ulong(uuid);
	SCM h;
	SCM_NEWSMOB (h, cog_uuid_tag, suuid);
	SCM sind = scm_from_locale_symbol("indicator");
	SCM satom = scm_from_locale_symbol("atom");

	SCM rc = SCM_EOL;
	rc = scm_acons(satom, h, rc);
	rc = scm_acons(sind, ind, rc);
	return rc;
}

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
