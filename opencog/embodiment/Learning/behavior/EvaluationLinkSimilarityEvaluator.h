/**
 * EvaluationLinkSimilarityEvaluator.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Sat Sep 15 14:55:19 BRT 2007
 */

#ifndef EVALUATIONLINKSIMILARITYEVALUATOR_H
#define EVALUATIONLINKSIMILARITYEVALUATOR_H

#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

namespace behavior {

class EvaluationLinkSimilarityEvaluator {

    private:

    public:

        // ***********************************************/
        // Constructors/destructors

        ~EvaluationLinkSimilarityEvaluator();
        EvaluationLinkSimilarityEvaluator();

        static float computeHandleSimilarity(Handle h1, Handle h2);
        static float similarity(AtomSpace *atomSpace, Handle h1, Handle h2);

}; // class
}  // namespace

#endif
