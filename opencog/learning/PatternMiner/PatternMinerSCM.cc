/*
 * PatternMinerSCM.cc
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Shujing Ke
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


// The header file bits.

#ifndef _OPENCOG_PATTERNMINER_SCM_H
#define _OPENCOG_PATTERNMINER_SCM_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/learning/PatternMiner/PatternMiner.h>

using namespace std;
using namespace opencog::PatternMining;

namespace opencog
{

class PatternMinerSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);



    PatternMiner* patternMiner;


public:
    PatternMinerSCM();
    void run_patternminer();
};


extern "C" {
void opencog_patternminer_init(void);
};

}
#endif // _OPENCOG_PATTERNMINER_SCM_H

// --------------------------------------------------------------

#include <opencog/util/Config.h>
#include <opencog/guile/SchemePrimitive.h>

/**
 * Implement a dynamically-loadable Pattern Miner guile module.
 */

using namespace opencog;


PatternMinerSCM::PatternMinerSCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

void* PatternMinerSCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog patternminer", init_in_module, self);
    scm_c_use_module("opencog patternminer");
    return NULL;
}

void PatternMinerSCM::init_in_module(void* data)
{
    PatternMinerSCM* self = (PatternMinerSCM*) data;
    self->init();
}

/**
 * The main init function for the PatternMinerSCM object.
 */
void PatternMinerSCM::init()
{
    define_scheme_primitive("run-patternminer", &PatternMinerSCM::run_patternminer, this, "patternminer");

}

extern "C" {
void opencog_patternminer_init(void)
{
    static PatternMinerSCM patternminerscm;
}
};

// --------------------------------------------------------------

void PatternMinerSCM::run_patternminer()
{

    AtomSpace* as = SchemeSmob::ss_get_env_as("run_patternminer");
    patternMiner = new PatternMiner(as);
    patternMiner->runPatternMiner(1);


}
