#include <string>
#include <vector>
#include <map>

#include <opencog/atomspace/types.h>
#include "PetInterface.h"

using namespace std;
using namespace opencog;


typedef map<Handle, double> scoreCandidatesMap;

class NominalReferenceResolver {
    private:
        Control::PetInterface& petInterface;

        bool createSetOfCandidates(const string& name, unsigned long timestamp, set<Handle> & candidates) const;
        void scoreCandidates(const string& name, const string& speakerId, unsigned long timestamp, const set<Handle> & candidates, scoreCandidatesMap& scoredCandidates) const; 
        Handle selectCandidate(const scoreCandidatesMap& scoredCandidates) const;
    public:
        NominalReferenceResolver(Control::PetInterface& _petInterface);
        string solve(const string& name, const string& speakerId, unsigned long timestamp) const;
};
