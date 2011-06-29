/*
 * opencog/embodiment/RuleValidation/Processor/ProcessorExec.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include "RuleProcessor.h"
#include <opencog/util/files.h>
#include <opencog/util/exceptions.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>

using namespace opencog;

int main(int argc, char * argv[])
{

    if (argc != 3) {
        fprintf(stdout, "processor <scenario-file> <type: pet or humanoid>\n");
        return (1);
    }

    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);

    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }


    if ((strcmp(argv[2], "pet") != 0) &&
            (strcmp(argv[2], "humanoid") != 0)) {
        fprintf(stdout, "processor <scenario-file> <type: pet or humanoid>. Got '%s'.\n", argv[2]);
        return (1);
    }

    //Nil: very odd, when replacing the following 2 non-commented
    //by the commented line just below, the compiler complains
    //Processor::RuleProcessor rp(std::string(argv[2]));
    std::string arg_2(argv[2]); 
    Processor::RuleProcessor rp(arg_2);

    try {
        rp.evaluateRules(std::string(argv[1]));
    } catch (...) {
        fprintf(stdout, "An error has occured while evaluating rules. Check log.\n");
        return (1);
    }
    return (0);
}
