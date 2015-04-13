/*
 * opencog/embodiment/Learning/behavior/EvaluationLinkSimilarityEvaluator.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef EVALUATIONLINKSIMILARITYEVALUATOR_H
#define EVALUATIONLINKSIMILARITYEVALUATOR_H

#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

namespace behavior
{

class EvaluationLinkSimilarityEvaluator
{

private:

public:

    // ***********************************************/
    // Constructors/destructors

    ~EvaluationLinkSimilarityEvaluator();
    EvaluationLinkSimilarityEvaluator();

    static float computeHandleSimilarity(AtomSpace &atomSpace, Handle h1, Handle h2);
    static float similarity(AtomSpace &atomSpace, Handle h1, Handle h2);

}; // class
}  // namespace

#endif
