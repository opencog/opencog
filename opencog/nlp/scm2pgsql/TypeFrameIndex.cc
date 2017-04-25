#include "TypeFrameIndex.h"
#include "TypeFrame.h"

using namespace opencog;

TypeFrameIndex::TypeFrameIndex() 
{
}

TypeFrameIndex::~TypeFrameIndex() 
{
}

bool TypeFrameIndex::addFromScheme(const std::string &txt)
{

    bool returnValue = true;
    TypeFrame frame(txt);
    if (frame.isValid()) {
        // frame.printForDebug();
        TypeFrame::TypePair key = frame.at(0);
        TypePairMap::iterator it = patterns.find(key);
        TypeFramePattern *pattern = NULL;
        if (it == patterns.end()) {
            pattern = new TypeFramePattern();
            patterns.insert(TypePairMap::value_type(key, pattern));
        } else {
            pattern = (*it).second;
        }
        pattern->add(frame);
        returnValue = false;
    } else {
        printf("INVALID FRAME <%s>\n", txt.c_str());
    }

    return returnValue;
}

void TypeFrameIndex::printForDebug(bool showNames)
{
    TypePairMap::iterator it = patterns.begin();
    int count = 1;
    while (it != patterns.end()) {
        std::string prefix = std::to_string(count);
        if (showNames) {
            printf("%s: (%s,%d) {", prefix.c_str(), classserver().getTypeName((*it).first.first).c_str(), (*it).first.second);
        } else {
            printf("%s: (%d,%d) {", prefix.c_str(), (*it).first.first, (*it).first.second);
        }
        (*it).second->printForDebug("", prefix, showNames);
        printf("}\n");
        it++;
        count++;
    }
}
