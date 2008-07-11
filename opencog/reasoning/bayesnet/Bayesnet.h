#ifndef _BAYESNET_H
#define _BAYESNET_H

#include <IndefiniteTruthValue.h>
#include <IndefinitePLNFormulas.h>
#include <vector>

using namespace reasoning;

namespace bayes {

class InferenceEngine {
        public:
                virtual IndefiniteRule* CreateEngine() = 0;
};

template <typename R>
class InferenceEngineGenerator: public InferenceEngine {
        public:
                virtual InferenceRule* CreateEngine();
};

template <typename R>
InferenceRule* InferenceEngineGenerator<R>::CreateEngine() {
        return new R;
}

//  

}; // namespace bayes
