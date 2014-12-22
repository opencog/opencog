/*
 * LGDictMoudle.h
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

#ifndef _OPENCOG_LG_DICT_MODULE_H
#define _OPENCOG_LG_DICT_MODULE_H

#include <link-grammar/dict-api.h>

#include <opencog/server/Module.h>
#include <opencog/atomspace/Handle.h>

namespace opencog {

/**
 * An OpenCog module for reading LG dictionary.
 *
 * This module creates the necessary scheme bindings for accessing the
 * Link Grammar dictionary.
 */
class LGDictModule : public Module
{
    private:
        Handle do_lg_get_dict_entry(Handle);

        Dictionary m_pDictionary;

    public:
        LGDictModule(CogServer&);
        virtual ~LGDictModule();
        const char * id(void);
        virtual void init(void);
};

}

#endif // _OPENCOG_LG_DICT_MODULE_H
