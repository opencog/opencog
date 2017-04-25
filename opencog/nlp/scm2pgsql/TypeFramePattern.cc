#include "TypeFramePattern.h"

using namespace opencog;

TypeFramePattern::TypeFramePattern() 
{
}

TypeFramePattern::~TypeFramePattern() 
{
}

void TypeFramePattern::add(TypeFrame &frame)
{
    if (DEBUG) printf("START REC\n");
    recursiveAdd(frame, 0);
    if (DEBUG) printf("END REC\n");
}

void TypeFramePattern::recursiveAdd(TypeFrame &frame, int cursor)
{

    if (DEBUG) printf("cursor: %d\n", cursor);
    //frame.printForDebug();

    TypeFrame::TypeVector key = frame.buildSignatureVector(cursor);
    if (DEBUG) printf("numargs: %d\n", (int) key.size());
    if (key.size() > 0) {
        TypeVectorMap::iterator it = branches.find(key);
        if (it == branches.end()) {
            if (DEBUG) printf("NEW BRANCH\n");
            BranchVector target;
            int offset = 1;
            for (unsigned int i = 0; i < key.size(); i++) {
                if (DEBUG) printf("NB: i = %u offset = %d\n", i, offset);
                TypeFramePattern *newPattern = new TypeFramePattern();    
                newPattern->recursiveAdd(frame, cursor + offset);
                offset += key.at(i).second + 1;
                target.push_back(newPattern);
            }
            branches.insert(TypeVectorMap::value_type(key, target));
        } else {
            if (DEBUG) printf("REUSING BRANCH\n");
            int offset = 1;
            for (unsigned int i = 0; i < (*it).second.size(); i++) {
                if (DEBUG) printf("RB: i = %u offset = %d\n", i, offset);
                (*it).second.at(i)->recursiveAdd(frame, cursor + offset);
                offset += key.at(i).second + 1;
            }
        }
    }
    if (DEBUG) printf("RETURNING\n");
}

void TypeFramePattern::printForDebug(std::string upperIndent, std::string upperPrefix, bool showNames)
{
    TypeVectorMap::iterator it = branches.begin();
    if (it == branches.end()) {
        printf(" ");
    } else {
        printf("\n");
    }
    int count = 1;
    while (it != branches.end()) {
        std::string indent = upperIndent + "    ";
        std::string prefix = upperPrefix + "." + std::to_string(count);
        for (unsigned int i = 0; i < (*it).first.size(); i++) {
            if (showNames) {
                printf("%s%s: (%s,%d) {", indent.c_str(), prefix.c_str(), classserver().getTypeName((*it).first.at(i).first).c_str(), (*it).first.at(i).second); 
            } else {
                printf("%s%s: (%d,%d) {", indent.c_str(), prefix.c_str(), (*it).first.at(i).first, (*it).first.at(i).second); 
            }
            (*it).second.at(i)->printForDebug(indent, prefix, showNames);
            printf("%s}\n", indent.c_str());
        }
        it++;
        count++;
    }
}
