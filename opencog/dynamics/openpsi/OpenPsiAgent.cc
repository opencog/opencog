/*
 * @file opencog/dynamics/openpsi/OpenPsiAgent.cc
 * @author Amen Belayneh <amenbelayneh@gmail.com> August 2015
 *
 * Copyright (C) 2015 OpenCog Foundation
 * All Rights Reserved
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

#include "OpenPsiAgent.h"

#include <opencog/server/CogServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/rule-engine/UREConfigReader.h>
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

/**
 * The constructor for OpenPsiAgent.
 *
 * @param cs   A cogserver
 */
OpenPsiAgent::OpenPsiAgent(CogServer& cs) : Agent(cs)
{
    logger().info("[OpenPsiAgent] Entering constructor");
}

OpenPsiAgent::~OpenPsiAgent()
{
    logger().info("[OpenPsiAgent] Entering destructor");
}


void OpenPsiAgent::run()
{

    AtomSpace& as = _cogserver.getAtomSpace();
    Handle asp = as.get_node(CONCEPT_NODE, "OpenPsi: active-schema-pool");

     // If the the openpsi active-schema-pool is not defined do nothing.
     if (asp == Handle::UNDEFINED){
         if ( !(_cogserver.getCycleCount() % 100)){
             logger().info("[OpenPsiAgent] OpenPsi's active-schema-pool "
                           "definition hasn't been loaded into atomspace.");
         }
         return;
     }

     // Making sure the chainer isn't run before the whole asp definition is
     // loaded into the atomspace.
     try {
         UREConfigReader cr(as, asp);
     }
     catch (std::exception& e) {
         logger().info("[OpenPsiAgent] OpenPsi's active-schema-pool definition"
                       " hasn't been FULLY loaded into atomspace.\n"
                       "Exception Detail: %s", e.what());
         return;
     }

#if HAVE_GUILE

    SchemeEval evaluator(&as);
    std::string scheme_expression, scheme_returned_value;
    scheme_expression = "(psi-run)";

    scheme_returned_value = evaluator.eval(scheme_expression);

    if (evaluator.eval_error()) {
        logger().info("[OpenPsiAgent] %s - Failed to execute '%s'",
                         __FUNCTION__, scheme_expression.c_str());
    }

#else // HAVE_GUILE
    logger().info("[OpenPsiAgent] %s - guile is required", __FUNCTION__);
    return;

#endif // HAVE_GUILE

}
