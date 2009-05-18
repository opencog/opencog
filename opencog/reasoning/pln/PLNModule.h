/*
 * opencog/reasoning/pln/PLNModule.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
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

#ifndef _OPENCOG_PLN_MODULE_H
#define _OPENCOG_PLN_MODULE_H

#include <string>

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

#include "BackChainingAgent.h"

namespace opencog
{

class CogServer;

class PLNModule : public Module
{
private:

    DECLARE_CMD_REQUEST(PLNModule, "pln", do_pln, 
        "Run a PLN command", 
        "Usage: pln <command>\n\n"
        "Run the specified PLN command.\n"
        "( NOTE THE DIFFERENCE BETWEEN ARG TYPES:\n"
        "- some commands take PLN Handles, these are different from AtomSpace Handles!\n"
        "- some take BITNode pointers. )\n"
        "\n"
        "---\n"
        " log <[-4..4]> - Set log level (0 = normal log level).\n"
        " record-trails - Switch the recording of inference trails ON/OFF (default: ON)\n"
        " infer <s>     - Infer until result found with conf > 0.01 OR 's' inference steps\n"
        "                 have been taken.\n"
        " atom <h>      - print the atom with PLN Handle h\n"
        " plan <h>      - Show the plan ie. sequence of 'do' statements pertaining to inference\n"
        "                 result of PLN Handle h\n"
        " trail <h>     - print the inference trail for PLN Handle #h\n"
        "\n"
        "--- Pool\n"
        " pool          - Show the current BIT node expansion pool sorted by heuristic fitness\n"
        " pool-size     - Return current BIT node expansion pool size\n"
        " pool-fittest  - Show the current BIT node expansion pool sorted by heuristics\n"
        "                 fitness and expand the fittest BIT node\n"
        " pool-expand <n>- Execute the #n fittest BIT nodes\n"
        "\n"
        "--- BIT\n"
        " bit <n>            - Print the inference (BIT) tree under node n (0 = root)\n"
        " bit-expand <n>     - Expand BITNode with id n\n"
        " bit-results <n>    - Print out the results of BIT node n (0 = root)\n"
        " bit-parents <n>    - Show the parents of BITNode #n\n"
        " bit-rule-args <n>  - Print the Rule arguments of BIT node n\n"
        " bit-rule-target <n>- Print the Rule target of BIT node n\n"
        " bit-direct-results #i - (disabled) Show the direct results (by lookup or hypothesis)\n "
        "                 of BIT node #i\n"
        "\n"
        "--- Testing\n"
        " load-axioms <path> - Load XML axiom file in 'path'\n"
        " test-count         - count the number of pre-defined inference targets\n"
        " test-target <n>    - Load in a new pre-defined target #n (from TestTargets.h)\n"
        " = <n1> <n1>        - Check if BIT nodes n1 and n2 are equal.\n"
        "\n"
        "--- The following are not recommended unless you know what your doing:\n"
        " bit-next-level     - Expand the tree's whole next level (usually not recommended)\n"
        " bit-eval           - Manually evaluate the current tree (usually not recommended)\n"
        " bit-find-node  bT b1 b2  a1T a10 a11 [a2T a20 a21] <Rule ptr> - find a BITNode \n"
        "                 for the given rule with the given parameters. 3rd parameter depends \n"
        "                 on rule number, but must be specified (needs to be more friendly).\n"
        " loop-check         - check for loops\n");

    Factory<BackChainingAgent, Agent> backChainingFactory;

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::PLNModule");
        return _ci;
    }
    
    static inline const char* id();

    PLNModule();
    ~PLNModule();
    void init();

}; // class

//! Takes a real handle, and sets the backward-chaining target to that atom.
void setTarget(Handle h);
//! Does steps inference steps on target h. Does not set the BC target used in the PLN commands.
void infer(Handle h, int &steps);

} // namespace opencog


#endif // _OPENCOG_PLN_MODULE_H

