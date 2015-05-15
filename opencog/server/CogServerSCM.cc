/*
 * CogServerSCM.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_COGSERVER_SCM_H
#define _OPENCOG_COGSERVER_SCM_H

#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

class CogServerSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    // void start_server(AtomSpace* = NULL);
    void start_server(void);
    void stop_server(void);


public:
    CogServerSCM();
};


extern "C" {
void opencog_cogserver_init(void);
};

}
#endif // _OPENCOG_COGSERVER_SCM_H

// --------------------------------------------------------------

#include <opencog/guile/SchemePrimitive.h>

#include "CogServer.h"

using namespace opencog;

CogServerSCM::CogServerSCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

void* CogServerSCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog cogserver", init_in_module, self);
    scm_c_use_module("opencog cogserver");
    return NULL;
}

void CogServerSCM::init_in_module(void* data)
{
    CogServerSCM* self = (CogServerSCM*) data;
    self->init();
}

/**
 * The main init function for the SuRealSCM object.
 */
void CogServerSCM::init()
{
    define_scheme_primitive("start-server", &CogServerSCM::start_server, this, "cogserver");
}

void opencog_cogserver_init(void)
{
    static CogServerSCM cogserver;
}

// void CogServerSCM::start_server(AtomSpace* as)
void CogServerSCM::start_server(void)
{
}

void CogServerSCM::stop_server(void)
{
}
