#include "PatternHeap.h"

using namespace opencog;

PatternHeap::PatternHeap(unsigned int max) 
{
    maxSize = max;
}

bool PatternHeap::contains(float v, const TypeFrame &frame) const
{
    bool answer = false;

    for (unsigned int i = 0; i < size(); i++) {
        // This float comparison is not supposed to be harmful because
        // equivalent frames will result in exactly equals quality measure
        if (v == at(i).first) {
            if (at(i).second.isEquivalent(frame)) {
                answer = true;
                break;
            }
        }
    }

    return answer;
}

void PatternHeap::push(float v, const TypeFrame &frame)
{
    if (((size() < maxSize) || (back().first < v)) && (! contains(v, frame))) {
        if (size() == maxSize) {
            pop_back();
        }
        iterator it;
        for (it = begin(); it != end(); ++it) {
            if (v >= (*it).first) {
                break;
            }
        }
        insert(it, std::make_pair(v, frame));
    }
}
