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

//#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

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
       "(NOTE THE DIFFERENCE BETWEEN ARG TYPES:\n\
Some commands take Handles, some take BITNodes.\n\
\n\
-3 - Minimal log level  \n\
0 - normal log level\n\
2 - recommended maximally informative log level\n\
r #n - Load in a new pre-defined target #n (from PLNShell.cc)\n\
x [path] - Load XML axiom file in 'path'\n\
s #s - Infer until result found with conf>0.01 OR 's' inference steps have been taken \n\
S #n - Execute the #n of the fittest BIT nodes\n\
i #n - Expand BITNode with id #n\n\
E #n - Print out the results of the BITnode #n  (0 = root)\n\
A #n - Print the Rule arguments of BITnode #n\n\
b #n - Print the Rule target of BITnode #n\n\
P #n - print the inference (BIT) tree under node #n (0 = root)\n\
a #h - Show the plan ie. sequence of 'do' statements pertaining to inference result Handle #h\n\
F - Show the current BIT node expansion pool sorted by heuristic fitness\n\
O #n - show the parent of BITNode #n\n\
t #h - print the inference trail for Handle #h\n\
\n\
c - switch the recording of inference trails ON/OFF (default: ON)\n\
B #i - Show the direct results (by lookup or hypothesis) of BIT node #i\n\
b #i - Show the target atom and pre-bindings of BIT node #i\n\
f - Show the current BIT node expansion pool sorted by heuristics fitness and Execute the fittest BIT node\n\
\n\
n - expand the tree's whole next level (usually not recommended)\n\
e - manually evaluate the current tree (usually not recommended)\n\
\n\
These should be bug-free, but there's no type checking of parameters, so providing eg. BIT node number instead of Handle number will SegFault.")

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

} // namespace opencog


#endif // _OPENCOG_PLN_MODULE_H

