/*
 * SuRealSCM.h
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

#ifndef _OPENCOG_SUREAL_SCM_H
#define _OPENCOG_SUREAL_SCM_H


#include <map>
#include <opencog/atoms/base/Handle.h>


namespace opencog
{
namespace nlp
{

/**
 * A class for defining the SuReal scheme bindings.
 *
 * Creates the necessary scheme bindings doing pattern matching and getting the
 * corresponding mapping.
 */
class SuRealSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    HandleSeqSeq do_sureal_match(Handle, bool);
    HandleSeqSeq do_non_cached_sureal_match(Handle);
    HandleSeqSeq do_cached_sureal_match(Handle);
    void reset_cache(void);

    HandleSeqSeq sureal_get_mapping(Handle&, std::vector<HandleMap >&);

public:
    SuRealSCM();
};

}
}

extern "C" {
void opencog_nlp_sureal_init(void);
};

#endif // _OPENCOG_SUREAL_SCM_H
