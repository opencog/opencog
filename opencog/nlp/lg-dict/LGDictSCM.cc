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

#include <opencog/atoms/base/Handle.h>
#include <opencog/guile/SchemePrimitive.h>

#include "LGDictReader.h"
#include "LGDictUtils.h"

namespace opencog
{
class LGDictSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    bool do_lg_conn_type_match(Handle, Handle);
    bool do_lg_conn_linkable(Handle, Handle);

public:
    LGDictSCM();
};

}

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
	scm_with_guile(init_in_guile, this);
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
	define_scheme_primitive("lg-conn-type-match?",
		 &LGDictSCM::do_lg_conn_type_match, this, "nlp lg-dict");
	define_scheme_primitive("lg-conn-linkable?",
		 &LGDictSCM::do_lg_conn_linkable, this, "nlp lg-dict");
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

extern "C" {
void opencog_nlp_lgdict_init(void)
{
	static LGDictSCM lgdict;
}
};
