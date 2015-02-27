/*
 * LGDictSCM.h
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

#ifndef _OPENCOG_LG_DICT_SCM_H
#define _OPENCOG_LG_DICT_SCM_H


#include <link-grammar/dict-api.h>
#include <opencog/atomspace/Handle.h>


namespace opencog
{
namespace nlp
{


class LGDictSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    Handle do_lg_get_dict_entry(Handle);
    bool do_lg_conn_type_match(Handle, Handle);
    bool do_lg_conn_linkable(Handle, Handle);

    Dictionary m_pDictionary;

public:
    LGDictSCM();
    ~LGDictSCM();
};


}
}

extern "C" {
void opencog_nlp_lgdict_init(void);
};


#endif // _OPENCOG_LG_DICT_SCM_H
