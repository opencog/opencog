/*
 * LGDictSCM.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
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
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/types/atom_types.h>

#include "LGDictSCM.h"
#include "LGDictReader.h"
#include "LGDictUtils.h"

using namespace opencog::nlp;
using namespace opencog;

/**
 * The constructor for LGDictSCM.
 */
LGDictSCM::LGDictSCM()
{
	static bool is_init = false;
	if (is_init) return;
	is_init = true;
	m_pDictionary = nullptr;
	scm_with_guile(init_in_guile, this);
}

/**
 * The destructor for LGDictSCM.
 */
LGDictSCM::~LGDictSCM()
{
	if (m_pDictionary)
		dictionary_delete(m_pDictionary);
}

/**
 * Init function for using with scm_with_guile.
 *
 * Creates the sureal scheme module and uses it by default.
 *
 * @param self   pointer to the LGDictSCM object
 * @return       null
 */
void* LGDictSCM::init_in_guile(void* self)
{
	scm_c_define_module("opencog nlp lg-dict", init_in_module, self);
	scm_c_use_module("opencog nlp lg-dict");
	return NULL;
}

/**
 * The main function for defining stuff in the sureal scheme module.
 *
 * @param data   pointer to the LGDictSCM object
 */
void LGDictSCM::init_in_module(void* data)
{
	LGDictSCM* self = (LGDictSCM*) data;
	self->init();
}

/**
 * The main init function for the SuRealSCM object.
 */
void LGDictSCM::init()
{
	define_scheme_primitive("lg-dict-open",
		 &LGDictSCM::do_lg_dictopen, this, "nlp lg-dict");

	define_scheme_primitive("lg-dict-close",
		 &LGDictSCM::do_lg_dictclose, this, "nlp lg-dict");

	define_scheme_primitive("lg-dict-entry",
		 &LGDictSCM::do_lg_dict_entry, this, "nlp lg-dict");

	// XXX this is deprecated.
	define_scheme_primitive("lg-get-dict-entry",
		 &LGDictSCM::do_lg_get_dict_entry, this, "nlp lg-dict");

	define_scheme_primitive("lg-conn-type-match?",
		 &LGDictSCM::do_lg_conn_type_match, this, "nlp lg-dict");
	define_scheme_primitive("lg-conn-linkable?",
		 &LGDictSCM::do_lg_conn_linkable, this, "nlp lg-dict");
}

/**
 * Implementation of the "lg-dict-open" scheme primitive.
 *
 * XXX FIXME the current API allows only one global dictionary
 * at a time.  The correct fix is to invent a new Link type that
 * inherits from FunctionLink:
 *
 *    LgDictEntry
 *        WordNode "foobar"
 *        LgDictNode "en"
 *
 * When the above is executed, the word would be looked up, and the
 * disjuncts placed into the atomspace.  See the implementation of
 * LgParse for an example of how to do this.
 */
void LGDictSCM::do_lg_dictopen(Handle h)
{
	if (not h->isNode()) return;

	if (m_pDictionary)
		dictionary_delete(m_pDictionary);

	const char * lang = h->getName().c_str();
	m_pDictionary = dictionary_create_lang(lang);
}

/**
 * Implementation of the "lg-dict-close" scheme primitive.
 *
 * XXX FIXME the current API allows only one global dictionary
 * at a time.  Some future version should fix this, to allow
 * multiple dictionaries at a time, right?  This is a low-priority
 * though, it seems.
 */
void LGDictSCM::do_lg_dictclose(void)
{
	if (m_pDictionary)
		dictionary_delete(m_pDictionary);
	m_pDictionary = nullptr;
}

/**
 * Implementation of the "lg-dict-entry" scheme primitive.
 *
 * The corresponding implementation for the "lg-dict-entry"
 * primitive, which accepts a WordNode as input and places
 * the LG dictionary entry into the atomspace.
 *
 * @param h   the input WordNode containing the word string
 */
void LGDictSCM::do_lg_dict_entry(Handle h)
{
	if (h->getType() != WORD_NODE) return;

	// Check if the dictionary entry is already in the atomspace.
	HandleSeq djset;
	h->getIncomingSetByType(std::back_inserter(djset), LG_DISJUNCT);

	// Avoid the disjuncts building if entries exist.
	if (not djset.empty()) return;

	if (nullptr == m_pDictionary)
		m_pDictionary = dictionary_create_default_lang();

	AtomSpace* pAS = SchemeSmob::ss_get_env_as("lg-dict-entry");
	LGDictReader reader(m_pDictionary, pAS);

	reader.getDictEntry(h->getName());
}

/**
 * Implementation of the "lg-get-dict-entry" scheme primitive.
 *
 * The corresponding implementation for the "lg-get-dict-entry"
 * primitive, which accepts a WordNode as input and output the LG
 * dictionary atom.
 *
 * XXX FIXME! This should NOT return a SetLink!  It should return
 * nothing at all; users can already get the needed data by calling
 * getIncomingSetByType() themselves.  SetLink just clogs the atomspace
 * with junk.
 *
 * @param h   the input WordNode containing the word string
 * @return    the LG dictionary atom
 */
Handle LGDictSCM::do_lg_get_dict_entry(Handle h)
{
	if (h->getType() != WORD_NODE)
		return Handle::UNDEFINED;

	// Check if the dictionary entry is already in the atomspace.
	HandleSeq djset;
	h->getIncomingSetByType(std::back_inserter(djset), LG_DISJUNCT);

	// Avoid the disjuncts building if entries exist.
	if (not djset.empty())
		return Handle(createLink(djset, SET_LINK));

	if (nullptr == m_pDictionary)
		m_pDictionary = dictionary_create_default_lang();

	AtomSpace* pAS = SchemeSmob::ss_get_env_as("lg-get-dict-entry");
	LGDictReader reader(m_pDictionary, pAS);

	djset = reader.getDictEntry(h->getName());
	return Handle(createLink(djset, SET_LINK));
}

/**
 * Implementation of the "lg-conn-type-match?" scheme primitive.
 *
 * @param h1    the first LGConnector
 * @param h2    the second LGConnector
 * @return      true if the type matches
 */
bool LGDictSCM::do_lg_conn_type_match(Handle h1, Handle h2)
{
	return lg_conn_type_match(h1, h2);
}

/**
 * Implementation of the "lg-conn-linkable?" scheme primitive.
 *
 * @param h1    the first LGConnector
 * @param h2    the second LGConnector
 * @return      true if linkable
 */
bool LGDictSCM::do_lg_conn_linkable(Handle h1, Handle h2)
{
	return lg_conn_linkable(h1, h2);
}

void opencog_nlp_lgdict_init(void)
{
	static LGDictSCM lgdict;
}
