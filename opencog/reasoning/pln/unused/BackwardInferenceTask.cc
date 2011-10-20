/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include "PLN.h"

#ifndef AUTO_H_PLNEVALUATOR_H
  #define AUTO_H_PLNEVALUATOR_H
  #include "PLNEvaluator.h"
#endif

// Welter's comment: This seem not used anymore
//#ifndef AUTO_H_INFERENCEMINDAGENT_H
//  #define AUTO_H_INFERENCEMINDAGENT_H
//  #include "InferenceMindAgent.h"
//#endif

#ifndef AUTO_H_RULES_H
  #define AUTO_H_RULES_H
  #include "PLNRules/Rules.h"
#endif

#ifndef AUTO_H_BACKWARDINFERENCETASK_H
  #define AUTO_H_BACKWARDINFERENCETASK_H
  #include "BackwardInferenceTask.h"
#endif


namespace opencog { namespace pln
{

BackwardInferenceTask::BackwardInferenceTask(const InferenceTaskParameters& _pars)
: pars(_pars)
{
    puts("BackwardInferenceTask created.");
}

BackwardInferenceTask::~BackwardInferenceTask()
{
}

void BackwardInferenceTask::cycle()
{
    puts("BackwardInferenceTask cycle()");  

/*
    HandleSeq ret = PLNEvaluator::evaluate(pars);
    
    if (ret && ret)
        printTree( ret );*/
    finished = true;
}

void BackwardInferenceTask::finish()
{
    puts("BackwardInferenceTask finish()");
}

bool BackwardInferenceTask::ok() { return finished; }

}} //opencog::pln

