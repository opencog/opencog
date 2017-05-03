#include "TypeFrameIndex.h"
#include "TypeFrame.h"

using namespace opencog;

TypeFrameIndex::TypeFrameIndex() 
{
}

TypeFrameIndex::~TypeFrameIndex() 
{
}

bool TypeFrameIndex::addFrame(TypeFrame &frame, int offset)
{
    bool exitStatus = true;
    if (frame.isValid()) {
        frames.push_back(frame);
        printf("%d: ", offset);
        frame.printForDebug("", "\n", true);
        exitStatus = false;
    } else {
        printf("DISCARDING INVALID FRAME (offset = %d)\n", offset);
    }

    return exitStatus;
}

bool TypeFrameIndex::addFromScheme(const std::string &txt, int offset)
{
    bool exitStatus = true;
    TypeFrame frame(txt);
    if (frame.isValid()) {
        exitStatus = addFrame(frame, offset);
    } else {
        printf("INVALID FRAME <%s>\n", txt.c_str());
    }
    return exitStatus;
}

// TODO: This method should allocate answers in the heap instead of the stack to
// avoid copying data all the away in the recursive calls
// TODO: Break this method into smaller pieces
std::vector<TypeFrame> TypeFrameIndex::computeSubPatterns(TypeFrame &baseFrame, int cursor)
{
    std::vector<TypeFrame> answer;
    unsigned int headArity = baseFrame.at(cursor).second;
    if (headArity == 0) {

        // Node

        TypeFrame pattern1, pattern2;
        pattern1.push_back(baseFrame.at(cursor));    // discards node name
        pattern2.pickAndPushBack(baseFrame, cursor); // uses node name

        if (DEBUG) {
            printf("Adding node pattern\n");
            printf("Name: %s\n", baseFrame.nodeNameAt(cursor).c_str());
            pattern1.printForDebug("pattern1: ", "\n", true);
            pattern2.printForDebug("pattern2: ", "\n", true);
        }

        answer.push_back(pattern1); 
        answer.push_back(pattern2);

    } else if (headArity == 2) {

        // Arity 2 Link, compute all possible patterns

        std::vector<TypeFrame> recurseResult1, recurseResult2;
        recurseResult1 = computeSubPatterns(baseFrame, cursor + 1);
        recurseResult2 = computeSubPatterns(baseFrame, cursor + 2 + baseFrame.at(cursor + 1).second);
        if (DEBUG) {
            for (unsigned int i = 0; i < recurseResult1.size(); i++) {
                recurseResult1.at(i).printForDebug(">>>>> recurseResult1: ", "\n", true);
            }
            for (unsigned int i = 0; i < recurseResult2.size(); i++) {
                recurseResult1.at(i).printForDebug(">>>>> recurseResult2: ", "\n", true);
            }
        }

        TypeFrame pattern;

        // Link * *
        pattern = TypeFrame::EMPTY_PATTERN;
        pattern.pickAndPushBack(baseFrame, cursor); 
        pattern.push_back(TypeFrame::STAR_PATTERN); 
        pattern.push_back(TypeFrame::STAR_PATTERN); 

        if (DEBUG) {
            printf("Adding * * 2-link pattern\n");
            pattern.printForDebug("", "\n", true);
        }

        answer.push_back(pattern);

        // Link [recurse1] *
        for (unsigned int i = 0; i < recurseResult1.size(); i++) {
            pattern = TypeFrame::EMPTY_PATTERN;
            pattern.pickAndPushBack(baseFrame, cursor); 
            pattern.append(recurseResult1.at(i));
            pattern.push_back(TypeFrame::STAR_PATTERN);
            if (DEBUG) {
                printf("Adding [%u] * 2-link pattern\n", i);
                pattern.printForDebug("", "\n", true);
            }
            answer.push_back(pattern);
        }

        // Link * [recurse2]
        for (unsigned int j = 0; j < recurseResult2.size(); j++) {
            pattern = TypeFrame::EMPTY_PATTERN;
            pattern.pickAndPushBack(baseFrame, cursor);
            pattern.push_back(TypeFrame::STAR_PATTERN); 
            pattern.append(recurseResult2.at(j));
            if (DEBUG) {
                printf("Adding * [%u] 2-link pattern\n", j);
                pattern.printForDebug("", "\n", true);
            }
            answer.push_back(pattern);
        }

        // Link [recurse1] [recurse2]
        for (unsigned int i = 0; i < recurseResult1.size(); i++) {
            for (unsigned int j = 0; j < recurseResult2.size(); j++) {
                pattern = TypeFrame::EMPTY_PATTERN;
                pattern.pickAndPushBack(baseFrame, cursor);
                pattern.append(recurseResult1.at(i));
                pattern.append(recurseResult2.at(j));
                if (DEBUG) {
                    printf("Adding [%u] [%u] 2-link pattern\n", i, j);
                    pattern.printForDebug("", "\n", true);
                }
                answer.push_back(pattern);
            }
        }
    } else {

        // Arity != 2 Link, doesn't compute all possible patterns (to avoid
        // combinatorial explosion). For an N-Arity Link, computes: 
        //
        // Link  *  *  * ...  *
        // Link T1  *  * ...  *
        // Link  * T2  * ...  *
        // Link  *  * T3 ...  *
        // Link  *  *  * ... TN
          
        // TODO: It would probably be OK to compute all patterns for 3-arity
        // links. So this should be done here and let this "generic" computation
        // only for links with arity > 3.

        std::vector<TypeFrame> recurseResult[headArity];
        int offset = 1;
        for (unsigned int k = 0; k < headArity; k++) {
            recurseResult[k] = computeSubPatterns(baseFrame, cursor + offset);
            offset += baseFrame.at(cursor + offset).second + 1;
        }

        TypeFrame pattern;

        // Link * * * ... *
        pattern = TypeFrame::EMPTY_PATTERN;
        pattern.pickAndPushBack(baseFrame, cursor);
        for (unsigned int k = 0; k < headArity; k++) {
            pattern.push_back(TypeFrame::STAR_PATTERN);
        }
        if (DEBUG) {
            printf("Adding * link pattern\n");
            pattern.printForDebug("", "\n", true);
        }
        answer.push_back(pattern);

        for (unsigned int k = 0; k < headArity; k++) {
            for (unsigned int i = 0; i < recurseResult[k].size(); i++) {
                pattern = TypeFrame::EMPTY_PATTERN;
                pattern.pickAndPushBack(baseFrame, cursor);
                for (unsigned int m = 0; m < k; m++) {
                    pattern.push_back(TypeFrame::STAR_PATTERN);
                }
                pattern.append(recurseResult[k].at(i));
                for (unsigned int m = k + 1; m < headArity; m++) {
                    pattern.push_back(TypeFrame::STAR_PATTERN);
                }
                if (DEBUG) {
                    printf("Adding [] [] link pattern (%u %u)\n", k, i);
                    pattern.printForDebug("", "\n", true);
                }
                answer.push_back(pattern);
            }
        }
    }

    return answer;
}

    typedef std::set<int> IntegerSet;
    typedef std::map<TypeFrame, IntegerSet> PatternMap;


void TypeFrameIndex::addPatternOccurrence(TypeFrame &pattern, int pos)
{
    PatternMap::iterator it = occurrenceSet.find(pattern);
    if (it == occurrenceSet.end()) {
        if (DEBUG) {
            printf("%d: ", pos);
            pattern.printForDebug("ADD NEW PATTERN ", "\n", true);
        }
        IntegerSet newSet;
        newSet.insert(pos);
        occurrenceSet.insert(PatternMap::value_type(pattern, newSet));
    } else {
        if (DEBUG) {
            printf("%d: ", pos);
            pattern.printForDebug("ADD PATTERN ", "\n", true);
            (*it).first.printForDebug("Found: ", "\n", true);
        }
        (*it).second.insert(pos);
    }
}

void TypeFrameIndex::buildSubPatternsIndex()
{
    for (unsigned int i = 0; i < frames.size(); i++) {
        TypeFrame currentFrame = frames.at(i);
        std::vector<TypeFrame> patterns = computeSubPatterns(currentFrame, 0);
        //currentFrame.printForDebug("", "\n", true);
        addPatternOccurrence(currentFrame, i);
        for (unsigned int j = 0; j < patterns.size(); j++) {
            //patterns.at(j).printForDebug("    ", "\n", true);
            addPatternOccurrence(patterns.at(j), i);
        }
    }
    printForDebug(true);
}

void TypeFrameIndex::printForDebug(bool showNodeNames)
{
    PatternMap::iterator it1 = occurrenceSet.begin();
    while (it1 != occurrenceSet.end()) {
        (*it1).first.printForDebug("[", "]: ", showNodeNames);
        IntegerSet::iterator it2 = (*it1).second.begin();
        while (it2 != (*it1).second.end()) {
            printf("%d ", *it2);
            it2++;
        }
        printf("\n");
        it1++;
    }
}

