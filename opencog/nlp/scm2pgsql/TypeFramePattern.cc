#include "TypeFramePattern.h"

using namespace opencog;

TypeFramePattern::TypeFramePattern() 
{
}

TypeFramePattern::~TypeFramePattern() 
{
}

void TypeFramePattern::add(TypeFrame &frame, int hitPos)
{
    if (DEBUG) printf("START REC\n");
    recursiveAdd(frame, hitPos, 0);
    if (DEBUG) printf("END REC\n");
}

void TypeFramePattern::recursiveAdd(TypeFrame &frame, int hitPos, int cursor)
{

    if (DEBUG) printf("cursor: %d\n", cursor);
    //frame.printForDebug();

    occurrences.push_back(hitPos);
    TypeFrame key = frame.buildSignature(cursor);
    if (DEBUG) printf("numargs: %d\n", (int) key.size());
    if (key.size() > 0) {
        // sub-frame is a link
        TypeFrameMap::iterator it = linkBranches.find(key);
        if (it == linkBranches.end()) {
            if (DEBUG) printf("NEW BRANCH\n");
            BranchVector target;
            int offset = 1;
            for (unsigned int i = 0; i < key.size(); i++) {
                if (DEBUG) printf("NB: i = %u offset = %d\n", i, offset);
                TypeFramePattern *newPattern = new TypeFramePattern();    
                newPattern->recursiveAdd(frame, hitPos, cursor + offset);
                offset += key.at(i).second + 1;
                target.push_back(newPattern);
            }
            linkBranches.insert(TypeFrameMap::value_type(key, target));
        } else {
            if (DEBUG) printf("REUSING BRANCH\n");
            int offset = 1;
            for (unsigned int i = 0; i < (*it).second.size(); i++) {
                if (DEBUG) printf("RB: i = %u offset = %d\n", i, offset);
                (*it).second.at(i)->recursiveAdd(frame, hitPos, cursor + offset);
                offset += key.at(i).second + 1;
            }
        }
    } else {
        // sub-frame is a node
        NodeNameMap::iterator it = nodeBranches.find(frame.nodeNameAt(cursor));
        if (it == nodeBranches.end()) {
            if (DEBUG) printf("NEW NODE TYPE\n");
            TypeFramePattern *newPattern = new TypeFramePattern();
            newPattern->occurrences.push_back(hitPos);
            nodeBranches.insert(NodeNameMap::value_type(frame.nodeNameAt(cursor), newPattern));
        } else {
            if (DEBUG) printf("REUSING NODE TYPE\n");
            (*it).second->occurrences.push_back(hitPos);
        }
    }
    if (DEBUG) printf("RETURNING\n");
}

// XXX
int COUNT = 0;
void TypeFramePattern::buildSubPatternsIndex(TypeFrameIndex *index, TypeFrame &pattern)
{
    TypeFrame subPattern;
    std::vector<int> subPatternOccurrences;

    if (DEBUG) {
        printf("buildSubPatternsIndex: ");
        pattern.printForDebug(true);
        printf("\n");
    }

    NodeNameMap::iterator itn = nodeBranches.begin();
    if (itn == nodeBranches.end()) {
        TypeFrameMap::iterator it = linkBranches.begin();
        int arity = 0;
        while (it != linkBranches.end()) {
            arity = (*it).first.size();
            subPattern = pattern;
            subPatternOccurrences.clear();
            for (int i = 0; i < arity; i++) {
                subPattern.push_back((*it).first.at(i));
                //if ((*it).first.nodeNameDefined(i)) {
                //    subPattern.setNodeNameAt(subPattern.size() - 1, (*it).first.nodeNameAt(i));
                //}
                for (unsigned int j = 0; j < (*it).second.at(i)->occurrences.size(); j++) {
                    subPatternOccurrences.push_back((*it).second.at(i)->occurrences.at(j));
                }
            }
            if (DEBUG) printf("Adding current pattern\n");
            index->addSubPattern(subPattern, subPatternOccurrences);
            for (int i = 0; i < arity; i++) {
                subPattern = pattern;
                for (int k = 0; k < i; k++) {

                }
                if (DEBUG) printf("%d Entering recursion\n", COUNT++);
                (*it).second.at(i)->buildSubPatternsIndex(index, subPattern);
                if (DEBUG) printf("%d Backing from recursion recursion\n", --COUNT);
            }
            if (arity == 2) {
                subPattern = pattern;
                subPatternOccurrences.clear();
                subPattern.push_back(TypeFrame::STAR_PATTERN);
                subPattern.push_back((*it).first.at(1));
                if ((*it).first.nodeNameDefined(1)) {
                    subPattern.setNodeNameAt(subPattern.size() - 1, (*it).first.nodeNameAt(1));
                }
                for (unsigned int j = 0; j < (*it).second.at(1)->occurrences.size(); j++) {
                    subPatternOccurrences.push_back((*it).second.at(1)->occurrences.at(j));
                }
                if (DEBUG) printf("Adding */X\n");
                index->addSubPattern(subPattern, subPatternOccurrences);

                subPattern = pattern;
                subPatternOccurrences.clear();
                subPattern.push_back((*it).first.at(0));
                if ((*it).first.nodeNameDefined(0)) {
                    subPattern.setNodeNameAt(subPattern.size() - 1, (*it).first.nodeNameAt(0));
                }
                subPattern.push_back(TypeFrame::STAR_PATTERN);
                for (unsigned int j = 0; j < (*it).second.at(0)->occurrences.size(); j++) {
                    subPatternOccurrences.push_back((*it).second.at(0)->occurrences.at(j));
                }
                if (DEBUG) printf("Adding X/*\n");
                index->addSubPattern(subPattern, subPatternOccurrences);
            }
            it++;
        }
        subPattern = pattern;
        for (int i = 0; i < arity; i++) {
            subPattern.push_back(TypeFrame::STAR_PATTERN);
        }
        if (DEBUG) printf("Adding */*\n");
        index->addSubPattern(subPattern, occurrences);
    } else {
        while (itn != nodeBranches.end()) {
            subPattern = pattern;
            std::string name = (*itn).first;
            subPattern.setNodeNameAt(subPattern.size() - 1, name);
            if (DEBUG) printf("Adding terminal\n");
            index->addSubPattern(subPattern, (*itn).second->occurrences);
            itn++;
        }
        //subPattern = pattern;
        //subPattern.setNodeNameAt(subPattern.size() - 1, TypeFrame::STAR_NODE_NAME);
        //if (DEBUG) printf("Adding terminal *\n");
        //index->addSubPattern(subPattern, occurrences);
    }
}

void TypeFramePattern::printForDebug(std::string upperIndent, std::string upperPrefix, bool showNames)
{
    NodeNameMap::iterator itn = nodeBranches.begin();

    std::string indent = upperIndent + "    ";
    if (itn == nodeBranches.end()) {
        for (unsigned int j = 0; j < occurrences.size(); j++) {
            printf(" %u", occurrences.at(j));
        }
        printf("\n");
        TypeFrameMap::iterator it = linkBranches.begin();
        int count = 1;
        while (it != linkBranches.end()) {
            std::string prefix = upperPrefix + "." + std::to_string(count);
            for (unsigned int i = 0; i < (*it).first.size(); i++) {
                if (showNames) {
                    printf("%s%s: (%s,%d) { ", indent.c_str(), prefix.c_str(), classserver().getTypeName((*it).first.at(i).first).c_str(), (*it).first.at(i).second); 
                } else {
                    printf("%s%s: (%d,%d) { ", indent.c_str(), prefix.c_str(), (*it).first.at(i).first, (*it).first.at(i).second); 
                }
                (*it).second.at(i)->printForDebug(indent, prefix, showNames);
                printf("%s}\n", indent.c_str());
            }
            it++;
            count++;
        }
    } else {
        printf("\n");
        while (itn != nodeBranches.end()) {
            printf("%s%s: ", indent.c_str(), (*itn).first.c_str());
            for (unsigned int i = 0; i < (*itn).second->occurrences.size(); i++) {
                printf("%u ", (*itn).second->occurrences.at(i));
            }
            printf("\n");
            itn++;
        }
    }
}
