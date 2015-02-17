/*
 * LGDictModule.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>

#include <opencog/nlp/types/atom_types.h>

#include "LGDictModule.h"
#include "LGDictReader.h"
#include "LGDictUtils.h"

using namespace opencog;

DECLARE_MODULE(LGDictModule);

/**
 * The constructor for LGDictModule.
 *
 * @param cs   the OpenCog server
 */
LGDictModule::LGDictModule(CogServer& cs) : Module(cs)
{

}

/**
 * The destructor for LGDictModule.
 */
LGDictModule::~LGDictModule()
{
    dictionary_delete(m_pDictionary);
}

/**
 * The required implementation for the pure virtual init method.
 */
void LGDictModule::init(void)
{
    m_pDictionary = dictionary_create_default_lang();

#ifdef HAVE_GUILE
    define_scheme_primitive("lg-get-dict-entry", &LGDictModule::do_lg_get_dict_entry, this, "nlp lg-dict");
    define_scheme_primitive("lg-conn-type-match?", &LGDictModule::do_lg_conn_type_match, this, "nlp lg-dict");
    define_scheme_primitive("lg-conn-linkable?", &LGDictModule::do_lg_conn_linkable, this, "nlp lg-dict");
#endif
}

/**
 * Implementation of the "lg-get-dict-entry" scheme primitive.
 *
 * The corresponding implementation for the "lg-get-dict-entry" primitive,
 * which accepts a WordNode as input and output the LG dictionary atom.
 *
 * @param h   the input WordNode containing the word string
 * @return    the LG dictionary atom
 */
Handle LGDictModule::do_lg_get_dict_entry(Handle h)
{
#ifdef HAVE_GUILE
    AtomSpace* pAS = SchemeSmob::ss_get_env_as("lg-get-dict-entry");

    if (pAS->isNode(h) && pAS->getType(h) == WORD_NODE)
    {
		// check if the dictionary entry is already in the atomspace
		HandleSeq qExisting;
		pAS->getHandleSet(std::back_inserter(qExisting), h, LG_WORD_CSET, false);

		// avoid the disjuncts building if entries exist
		if (not qExisting.empty())
			return pAS->addLink(SET_LINK, qExisting);

        LGDictReader reader(m_pDictionary, pAS);

        return reader.getAtom(pAS->getName(h));
    }

    return Handle::UNDEFINED;

#else
    return Handle::UNDEFINED;
#endif
}

/**
 * Implementation of the "lg-conn-type-match?" scheme primitive.
 *
 * @param h1    the first LGConnector
 * @param h2    the second LGConnector
 * @return      true if the type matches
 */
bool LGDictModule::do_lg_conn_type_match(Handle h1, Handle h2)
{
#ifdef HAVE_GUILE
    return lg_conn_type_match(h1, h2);
#else
    return false;
#endif
}

/**
 * Implementation of the "lg-conn-linkable?" scheme primitive.
 *
 * @param h1    the first LGConnector
 * @param h2    the second LGConnector
 * @return      true if linkable
 */
bool LGDictModule::do_lg_conn_linkable(Handle h1, Handle h2)
{
#ifdef HAVE_GUILE
    return lg_conn_linkable(h1, h2);
#else
    return false;
#endif
}
