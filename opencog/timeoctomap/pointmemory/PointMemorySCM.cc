/*
 * PointMemorySCM.cc
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Mandeep Singh Bhatia <https://github.com/yantrabuddhi>
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

#include <PointMemorySCM.h>
#include <opencog/guile/SchemePrimitive.h>

using namespace opencog::ato;
using namespace opencog;

/**
 * The constructor for PointMemorySCM.
 */
PointMemorySCM::PointMemorySCM()
{
//allocate point ato data structure object
    static bool is_init = false;
    if (is_init) return;
    space_res.push_back(SPACE_RES_M);
    tsa = new TimeSpaceAtom(MEM_SEC*FPS,space_res);
    is_init = true;
    scm_with_guile(init_in_guile, this);
}
PointMemorySCM::~PointMemorySCM()
{
    delete tsa;
}
/**
 * Init function for using with scm_with_guile.
 *
 * Creates the PointMemorySCM scheme module and uses it by default.
 *
 * @param self   pointer to the PointMemorySCM object
 * @return       null
 */
void* PointMemorySCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog ato pointmem", init_in_module, self);
    scm_c_use_module("opencog ato pointmem");
    return NULL;
}

/**
 * The main function for defining stuff in the PointMemorySCM scheme module.
 *
 * @param data   pointer to the PointMemorySCM object
 */
void PointMemorySCM::init_in_module(void* data)
{
    PointMemorySCM* self = (PointMemorySCM*) data;
    self->init();
}

/**
 * The main init function for the PointMemorySCM object.
 */
void PointMemorySCM::init()
{
#ifdef HAVE_GUILE
    define_scheme_primitive("map-ato-pt", &PointMemorySCM::map_ato, this, "ato pointmem");
    define_scheme_primitive("get-first-ato-pt", &PointMemorySCM::get_first_ato, this, "ato pointmem");
    define_scheme_primitive("get-last-ato-pt", &PointMemorySCM::get_last_ato, this, "ato pointmem");
    define_scheme_primitive("get-mem-elapse-pt", &PointMemorySCM::get_mem_elapse, this, "ato pointmem");
#endif
}

