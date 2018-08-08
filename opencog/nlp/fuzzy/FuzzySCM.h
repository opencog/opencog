/*
 * FuzzySCM.h
 *
 * Copyright (C) 2015, 2016 OpenCog Foundation
 * All Rights Reserved
 *
 * Author: Leung Man Hin <https://github.com/leungmanhin>
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

#ifndef FUZZYSCM_H
#define FUZZYSCM_H

#include <opencog/atoms/base/Handle.h>

namespace opencog
{
namespace nlp
{

class FuzzySCM
{
    private:
        static void* init_in_guile(void*);
        static void init_in_module(void*);
        void init(void);

        Handle do_nlp_fuzzy_match(Handle, Type, const HandleSeq&,bool);
        Handle do_nlp_fuzzy_compare(Handle, Handle);

    public:
        FuzzySCM();
};

}
}

extern "C" {
void opencog_nlp_fuzzy_init(void);
};

#endif  // FUZZYSCM_H
