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

// The header file bits.

#ifndef _OPENCOG_COGSERVER_SCM_H
#define _OPENCOG_COGSERVER_SCM_H

#include <thread>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>

namespace opencog
{

class CogServerSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    void start_server(const std::string&);
    void stop_server(void);

    CogServer* srvr = NULL;
    std::thread * main_loop = NULL;

public:
    CogServerSCM();
};


extern "C" {
void opencog_cogserver_init(void);
};

}
#endif // _OPENCOG_COGSERVER_SCM_H

// --------------------------------------------------------------

#include <opencog/util/Config.h>
#include <opencog/guile/SchemePrimitive.h>

/**
 * Implement a dynamically-loadable cogserver guile module.
 * This is mostly to allow a networked, agent-based system to run.
 *
 * This is a bare-bones implementation. Things like the server
 * port number, and maybe even the server atomspace, should be
 * configurable from guile, instead of the conf file.
 *
 * If/when python agents are supplanted by something newer, then
 * maybe this goes away.
 */

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
 * The main init function for the CogServerSCM object.
 */
void CogServerSCM::init()
{
    define_scheme_primitive("start-cogserver", &CogServerSCM::start_server, this, "cogserver");
}

extern "C" {
void opencog_cogserver_init(void)
{
    static CogServerSCM cogserver;
}
};

// --------------------------------------------------------------

void CogServerSCM::start_server(const std::string& cfg)
{
    // singleton instance
    if (srvr) return;

    if (0 < cfg.size())
        config().load(cfg.c_str(), true);
    else
        config().load(NULL, true);

    srvr = &cogserver();

    // Open database *before* loading modules, since the modules
    // might create atoms, and we can't have that happen until
    // storage is open, as otherwise, there will be handle conflicts.
    srvr->openDatabase();

    // Load modules specified in config
    srvr->loadModules();
    srvr->loadSCMModules();

    // Enable the network server and run the server's main loop
    srvr->enableNetworkServer();
    main_loop = new std::thread(&CogServer::serverLoop, srvr);
}

void CogServerSCM::stop_server(void)
{
}
