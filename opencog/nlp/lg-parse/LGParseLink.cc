/*
 * LGParseLink.cc
 *
 * Copyright (C) 2017 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#include "LGParseLink.h"

using namespace opencog;

LGParseLink::LGParseLink(const HandleSeq& oset, Type t)
	: FunctionLink(oset, t)
{
}

LGParseLink::LGParseLink(const Link& l)
	: FunctionLink(l)
{
	// Type must be as expected
	Type tparse = l.getType();
	if (not classserver().isA(tparse, LG_PARSE_LINK))
	{
		const std::string& tname = classserver().getTypeName(tparse);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgDictNode, got %s", tname.c_str());
	}
}

Handle LGParseLink::execute(AtomSpace* as) const
{
}

DEFINE_LINK_FACTORY(LGParseLink, LG_PARSE_LINK)

/* ===================== END OF FILE ===================== */
