/*
 * opencog/embodiment/Control/EmbodimentConfig.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna, Nil Geisweiller
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



#include <iostream>
#include "EmbodimentConfig.h"

using namespace opencog;
using namespace opencog::control;

Config* EmbodimentConfig::embodimentCreateInstance()
{
    return new EmbodimentConfig();
}

EmbodimentConfig::~EmbodimentConfig()
{
}

EmbodimentConfig::EmbodimentConfig() {
    reset();
}

void EmbodimentConfig::reset() {
    //Note that C++ calls Config::Config(), that itself calls Config::reset()
    //so there is no need to call it here in order to
    //inherit the default parameters.
    //Also, EmbodimentConfig::reset() overwrites existing default paramters
    //defined in Config::reset()
    
    // load embodiment default configuration
    for (unsigned int i = 0; EMBODIMENT_DEFAULT()[i] != ""; i += 2) {
        table[EMBODIMENT_DEFAULT()[i]] = EMBODIMENT_DEFAULT()[i + 1];
    }
}

