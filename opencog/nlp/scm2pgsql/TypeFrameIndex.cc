#include "TypeFrameIndex.h"
#include "TypeFrame.h"

using namespace opencog;

TypeFrameIndex::TypeFrameIndex() 
{
}

TypeFrameIndex::~TypeFrameIndex() 
{
}

TypeFrame TypeFrameIndex::getFrameAt(int index)
{
    return frames.at(index);
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
std::vector<TypeFrame> TypeFrameIndex::computeSubPatterns(TypeFrame &baseFrame, int cursor, int pos)
{
    std::vector<TypeFrame> answer;
    unsigned int headArity = baseFrame.at(cursor).second;
    std::vector<int> argPos = baseFrame.getArgumentsPosition(cursor);

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

        recurseResult1 = computeSubPatterns(baseFrame, argPos.at(0), pos);
        recurseResult2 = computeSubPatterns(baseFrame, argPos.at(1), pos);
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
        for (unsigned int k = 0; k < headArity; k++) {
            recurseResult[k] = computeSubPatterns(baseFrame, argPos.at(k), pos);
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

    if (! TOPLEVEL_ONLY) {
        for (unsigned int i = 0; i < answer.size(); i++) {
            addPatternOccurrence(answer.at(i), pos);
        }
    }
    return answer;
}

void TypeFrameIndex::addPatternOccurrence(TypeFrame &pattern, int pos)
{
    occurrenceUniverse.insert(pos);
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
        std::vector<TypeFrame> patterns = computeSubPatterns(currentFrame, 0, i);
        if (TOPLEVEL_ONLY) {
            addPatternOccurrence(currentFrame, i);
            for (unsigned int j = 0; j < patterns.size(); j++) {
                addPatternOccurrence(patterns.at(j), i);
            }
        }
    }
    printForDebug(true);
}

void TypeFrameIndex::query(IntegerSet &result, const std::string &queryScm)
{
    TypeFrame queryFrame(queryScm);
    queryFrame.printForDebug("query: ", "\n", true);
    query(result, queryFrame);
}

void TypeFrameIndex::selectCurrentElement(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor)
{
    if (baseFrame.typeAtEqualsTo(cursor, "VariableNode")) {
        answer.push_back(TypeFrame::STAR_PATTERN);
        std::string key = baseFrame.nodeNameAt(cursor);
        StringMap::iterator it = variableOccurrences.find(key);
        if (it == variableOccurrences.end()) {
            IntegerSet newSet;
            newSet.insert(answer.size() - 1);
            variableOccurrences.insert(StringMap::value_type(key, newSet));
            if (DEBUG) printf("ADD NEW SET variable occurrence %s %lu\n", key.c_str(), answer.size() - 1);
        } else {
            if (DEBUG) printf("ADD variable occurrence %s %lu\n", key.c_str(), answer.size() - 1);
            (*it).second.insert(answer.size() - 1);
        }
    } else {
        answer.pickAndPushBack(baseFrame, cursor);
    }
}


void TypeFrameIndex::buildConstraints(IntPairVector &constraints, StringMap &variableOccurrences)
{
    constraints.clear();
    StringMap::iterator it = variableOccurrences.begin();
    while (it != variableOccurrences.end()) {
        std::vector<int> v((*it).second.begin(), (*it).second.end());
        for (unsigned int i = 0; i < v.size(); i++) {
            for (unsigned int j = i + 1; j < v.size(); j++) {
                constraints.push_back(std::make_pair(v.at(i), v.at(j)));
            }
        }
        it++;
    }
}

void TypeFrameIndex::buildJointConstraints(IntPairPairVector &constraints, std::vector<StringMap> &variableOccurrences)
{
    constraints.clear();
    for (unsigned int i = 0; i < variableOccurrences.size(); i++) {
        for (unsigned int j = i + 1; j < variableOccurrences.size(); j++) {
            for (StringMap::iterator it1 = variableOccurrences.at(i).begin(); it1 != variableOccurrences.at(i).end(); it1++) {
                StringMap::iterator it2 = variableOccurrences.at(j).find((*it1).first);
                if (it2 != variableOccurrences.at(j).end()) {
                    for (IntegerSet::iterator it3 = (*it1).second.begin(); it3 != (*it1).second.end(); it3++) {
                        for (IntegerSet::iterator it4 = (*it2).second.begin(); it4 != (*it2).second.end(); it4++) {
                            constraints.push_back(std::make_pair(std::make_pair(i, *it3), std::make_pair(j, *it4)));
                        }
                    }
                }
            }
        }
    }
}

void TypeFrameIndex::buildQueryTerm(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor)
{
    selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
    int nargs = baseFrame.at(cursor).second;
    while (nargs > 0) {
        cursor++;
        selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
        nargs--;
        if (nargs > 0) {
            int skip = baseFrame.at(cursor).second;
            while (skip > 0) {
                cursor++;
                selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
                skip += baseFrame.at(cursor).second;
                skip--;
            }
        }
    }
}

void TypeFrameIndex::query(IntegerSet &result, const TypeFrame &queryFrame)
{
    std::vector<ResultPair> recursionResult;
    TypeFrame keyExpression;
    StringMap variableOccurrences;

    query(recursionResult, keyExpression, variableOccurrences, queryFrame, 0);

    result.clear();
    for (unsigned int i = 0; i < recursionResult.size(); i++) {
        result.insert(recursionResult.at(i).first);
    }
}

void TypeFrameIndex::query(std::vector<ResultPair> &answer, TypeFrame &keyExpression, StringMap &variableOccurrences, const TypeFrame &queryFrame, int cursor)
{
    if (DEBUG) queryFrame.printForDebug("Query frame: ", "\n", true);
    if (DEBUG) printf("Cursor: %d\n", cursor);
    answer.clear();
    keyExpression.clear();
    variableOccurrences.clear();
    unsigned int arity = queryFrame.at(cursor).second;
    std::vector<int> argPos = queryFrame.getArgumentsPosition(cursor);

    if (queryFrame.typeAtEqualsTo(cursor, "AndLink")) {
        if (DEBUG) printf("Head is AND\n");
        std::vector<std::vector<ResultPair>> recursionQueryResult(arity);
        std::vector<TypeFrame> recursionKeyExpression(arity);
        std::vector<StringMap> recursionVariables(arity);
        keyExpression.pickAndPushBack(queryFrame, cursor);
        for (unsigned int i = 0; i < arity; i++) {
            query(recursionQueryResult.at(i), recursionKeyExpression.at(i), recursionVariables.at(i), queryFrame, argPos.at(i));
        }
        for (unsigned int i = 0; i < arity; i++) {
            if (DEBUG) {
                printf("recursion key[%d]: ", i);
                recursionKeyExpression.at(i).printForDebug("", "\n", true);
            }
            int offset = keyExpression.size();
            for (StringMap::iterator it1 = recursionVariables.at(i).begin(); it1 != recursionVariables.at(i).end(); it1++) {
                std::string key = (*it1).first;
                if (variableOccurrences.find(key) == variableOccurrences.end()) {
                    IntegerSet newSet;
                    variableOccurrences.insert(StringMap::value_type(key, newSet));
                }
                for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); it2++) {
                    variableOccurrences.find(key)->second.insert((*it2) + offset);
                }
            }
            keyExpression.append(recursionKeyExpression.at(i));
        }
        keyExpression.printForDebug("Resulting key: ", "\n", true);
        IntPairPairVector jointConstraints;
        buildJointConstraints(jointConstraints, recursionVariables);
        if (DEBUG) {
            printf("Joint constraints:\n");
        }
        IntegerSet invalidResults;
        for (unsigned int j = 0; j < jointConstraints.size(); j++) {
            int index1 = jointConstraints.at(j).first.first;
            int index2 = jointConstraints.at(j).second.first;
            int pos1 = jointConstraints.at(j).first.second;
            int pos2 = jointConstraints.at(j).second.second;
            if (DEBUG) {
                printf("(%d, %d) == (%d, %d)\n", index1, pos1, index2, pos2);
            }
            for (unsigned int m = 0; m < recursionQueryResult.at(index1).size(); m++) {
                unsigned int candidate1 = recursionQueryResult.at(index1).at(m).first;
                for (unsigned int n = 0; n < recursionQueryResult.at(index2).size(); n++) {
                    unsigned int candidate2 = recursionQueryResult.at(index2).at(n).first;
                    if ((candidate1 == candidate2) && (! frames.at(candidate1).subFramesEqual(recursionQueryResult.at(index1).at(m).second.at(pos1), recursionQueryResult.at(index2).at(n).second.at(pos2)))) {
                        invalidResults.insert(candidate1);
                        if (DEBUG) {
                            printf("Invalid candidate: %d\n", candidate1);
                        }
                    }
                }
            }
        }
        for (unsigned int i = 0; i < arity; i++) {
            if (DEBUG) {
                printf("Recursion result [%u]:\n", i);
                printRecursionResult(recursionQueryResult.at(i));
            }
            for (unsigned int j = 0; j < recursionQueryResult.at(i).size(); j++) {
                if (invalidResults.find(recursionQueryResult.at(i).at(j).first) == invalidResults.end()) {
                    std::vector<int> newMap;
                    for (unsigned int k = 0; k < recursionQueryResult.at(i).at(j).second.size(); k++) {
                        newMap.push_back(recursionQueryResult.at(i).at(j).second.at(k) + argPos.at(i));
                    }
                    answer.push_back(std::make_pair(recursionQueryResult.at(i).at(j).first, newMap));
                }
            }
        }
        if (DEBUG) {
            printf("Answer:\n");
            printRecursionResult(answer);
        }
    } else if (queryFrame.typeAtEqualsTo(cursor, "OrLink")) {
        if (DEBUG) printf("Head is OR\n");
    } else if (queryFrame.typeAtEqualsTo(cursor, "NotLink")) {
        if (DEBUG) printf("Head is NOT\n");
    } else {
        if (DEBUG) printf("Head is query term\n");
        IntPairVector constraints;
        buildQueryTerm(keyExpression, variableOccurrences, queryFrame, cursor);
        buildConstraints(constraints, variableOccurrences);
        if (DEBUG) {
            keyExpression.printForDebug("Query term: ", "\n", true);
            for (StringMap::iterator it1 = variableOccurrences.begin(); it1 != variableOccurrences.end(); it1++) {
                printf("%s: ", (*it1).first.c_str());
                for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); it2++) {
                    printf("%d ", (*it2));
                }
                printf("\n");
            }
            for (unsigned int i = 0; i < constraints.size(); i++) {
                printf("%d %d\n", constraints.at(i).first, constraints.at(i).second);
            }
        }
        PatternMap::iterator it1 = occurrenceSet.find(keyExpression);
        if (it1 != occurrenceSet.end()) {
            for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); it2++) {
                std::vector<int> mapping;
                if (frames.at(*it2).match(mapping, keyExpression, constraints)) {
                    answer.push_back(std::make_pair(*it2, mapping));
                }
            }
        }
    }
}

void TypeFrameIndex::printRecursionResult(std::vector<ResultPair> &v)
{
    for (unsigned int i = 0; i < v.size(); i++) {
        printf("%d [", v.at(i).first);
        for (unsigned int j = 0; j < v.at(i).second.size(); j++) {
            printf("%d", v.at(i).second.at(j));
            if (j != (v.at(i).second.size() - 1)) {
                printf(" ");
            }
        }
        printf("]\n");
    }
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

