#include "TypeFrameIndex.h"
#include "TypeFrame.h"

using namespace opencog;

const int TypeFrameIndex::OPERATOR_NOP = 0;
const int TypeFrameIndex::OPERATOR_AND = 1;
const int TypeFrameIndex::OPERATOR_OR = 2;
const int TypeFrameIndex::OPERATOR_NOT = 3;

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
        if (DEBUG) {
            printf("%d: ", offset);
            frame.printForDebug("", "\n", true);
        }
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
    if (DEBUG) printForDebug(true);
}

void TypeFrameIndex::query(std::vector<ResultPair> &result, const std::string &queryScm)
{
    TypeFrame queryFrame(queryScm);
    if (DEBUG) queryFrame.printForDebug("NEW QUERY: ", "\n", true);
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

void TypeFrameIndex::query(std::vector<ResultPair> &result, const TypeFrame &queryFrame)
{
    TypeFrame keyExpression;
    std::vector<VarMapping> forbiddenMappings;
    int headLogicOperator;

    query(result, keyExpression, forbiddenMappings, headLogicOperator, queryFrame, 0);
}

bool TypeFrameIndex::compatibleVarMappings(const VarMapping &map1, const VarMapping &map2)
{
    bool answer = true;

    /*
    if (DEBUG) {
        printf("TypeFrameIndex::compatibleVarMappings()\n");
        printVarMapping(map1);
        printf("\n");
        printVarMapping(map2);
    }
    */

    for (VarMapping::const_iterator it1 = map1.begin(); it1 != map1.end(); it1++) {
        VarMapping::const_iterator it2 = map2.find((*it1).first);
        if ((it2 != map2.end()) && (! (*it1).second.equals((*it2).second))) {
            answer = false;
            /*
            if (DEBUG) {
                printf("Failed at %s\n", (*it1).first.c_str());
            }
            */
            break;
        }
    }

    return answer;
}

void TypeFrameIndex::typeFrameSetUnion(TypeFrameSet &answer, const TypeFrameSet &set1, const TypeFrameSet &set2)
{
    answer.clear();
    for (TypeFrameSet::const_iterator it1 = set1.begin(); it1 != set1.end(); it1++) {
        answer.insert(*it1);
    }
    for (TypeFrameSet::const_iterator it2 = set2.begin(); it2 != set2.end(); it2++) {
        answer.insert(*it2);
    }
}

void TypeFrameIndex::varMappingUnion(VarMapping &answer, const VarMapping &map1, const VarMapping &map2)
{
    for (VarMapping::const_iterator it = map1.begin(); it != map1.end(); it++) {
        answer.insert(*it);
    }
    for (VarMapping::const_iterator it = map2.begin(); it != map2.end(); it++) {
        answer.insert(*it);
    }
}

bool TypeFrameIndex::isForbiddenMapping(const VarMapping &mapping, const std::vector<VarMapping> &forbiddenVector)
{
    bool answer = false;
    for (unsigned int i = 0; i < forbiddenVector.size(); i++) {
        bool match = true;
        for (VarMapping::const_iterator it1 = forbiddenVector.at(i).begin(); it1 != forbiddenVector.at(i).end(); it1++) {
            VarMapping::const_iterator it2 = mapping.find((*it1).first);
            if ((it2 != mapping.end()) && (! (*it1).second.equals((*it2).second))) {
                match = false;
                break;
            }
        }
        if (match) {
            answer = true;
            break;
        }
    }

    return answer;
}

void TypeFrameIndex::query(std::vector<ResultPair> &answer, TypeFrame &keyExpression, std::vector<VarMapping> &forbiddenMappings, int &logicOperator, const TypeFrame &queryFrame, int cursor)
{
    if (DEBUG) queryFrame.printForDebug("Query frame: ", "\n", true);
    if (DEBUG) printf("Cursor: %d\n", cursor);
    answer.clear();
    keyExpression.clear();
    forbiddenMappings.clear();
    unsigned int arity = queryFrame.at(cursor).second;
    std::vector<int> argPos = queryFrame.getArgumentsPosition(cursor);
    std::vector<std::vector<ResultPair>> recursionQueryResult(arity);
    std::vector<TypeFrame> recursionKeyExpression(arity);
    std::vector<std::vector<VarMapping>> recursionForbiddenMappings(arity);
    std::vector<int> recursionHeadLogicOperator(arity);
    bool AndFlag = queryFrame.typeAtEqualsTo(cursor, "AndLink");
    bool OrFlag = queryFrame.typeAtEqualsTo(cursor, "OrLink");
    bool NotFlag = queryFrame.typeAtEqualsTo(cursor, "NotLink");
    if (DEBUG) printf("Head is %s\n", (AndFlag ? "AND" : (OrFlag ? "OR" : (NotFlag ? "NOT" : "LEAF EXPRESSION"))));

    if (AndFlag || OrFlag || NotFlag) {
        // Recursive call on each clause
        keyExpression.pickAndPushBack(queryFrame, cursor);
        for (unsigned int i = 0; i < arity; i++) {
            query(recursionQueryResult.at(i), recursionKeyExpression.at(i), recursionForbiddenMappings.at(i), recursionHeadLogicOperator.at(i), queryFrame, argPos.at(i));
        }
        for (unsigned int i = 0; i < arity; i++) {
            if (DEBUG) {
                printf("recursion key[%d]: ", i);
                recursionKeyExpression.at(i).printForDebug("", "\n", true);
            }
            keyExpression.append(recursionKeyExpression.at(i));
        }
        if (DEBUG) {
            keyExpression.printForDebug("Resulting key: ", "\n", true);
            for (unsigned int i = 0; i < arity; i++) {
                printf("Recursion result [%u]:\n", i);
                printRecursionResult(recursionQueryResult.at(i));
            }
        }
    }

    if (AndFlag) {
        if (DEBUG) printf("Start processing AND\n");
        logicOperator = OPERATOR_AND;
        for (unsigned int i = 0; i < arity; i++) {
            for (unsigned int j = 0; j < recursionForbiddenMappings.at(i).size(); j++) {
                if (DEBUG) {
                    printf("(AND) Adding forbidden mapping:\n");
                    printVarMapping(recursionQueryResult.at(i).at(j).second);
                }
                forbiddenMappings.push_back(recursionForbiddenMappings.at(i).at(j));
            }
        }
        std::vector<std::vector<ResultPair>> cleanRecursionQueryResult(arity);
        for (unsigned int i = 0; i < arity; i++) {
            if (DEBUG) printf("Branch: %u\n", i);
            for (unsigned int j = 0; j < recursionQueryResult.at(i).size(); j++) {
                if (! isForbiddenMapping(recursionQueryResult.at(i).at(j).second, forbiddenMappings)) {
                    cleanRecursionQueryResult.at(i).push_back(recursionQueryResult.at(i).at(j));
                    if (DEBUG) {
                        printf("Pushing:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                    }
                } else {
                    if (DEBUG) {
                        printf("Forbidden:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                    }
                }
            }
        }
        std::vector<ResultPair> aux[2];
        int src = 0;
        int tgt = 1;

        unsigned int baseBranch = 0;
        bool selectedFlag = false;
        for (unsigned int i = 0; i < arity; i++) {
            if (recursionHeadLogicOperator.at(i) != OPERATOR_NOT) {
                baseBranch = i;
                selectedFlag = true;
                for (unsigned int j = 0; j < cleanRecursionQueryResult.at(baseBranch).size(); j++) {
                    aux[tgt].push_back(cleanRecursionQueryResult.at(baseBranch).at(j));
                    if (DEBUG) {
                        printf("(AND) Adding solution to result:\n");
                        printTypeFrameSet(cleanRecursionQueryResult.at(baseBranch).at(j).first);
                        printVarMapping(cleanRecursionQueryResult.at(baseBranch).at(j).second);
                    }
                }
                break;
            }
        }
        // TODO: raise an exception if selectedFlag == false
        int switchCount = 1;
        for (unsigned int i = 0; i < arity; i++) {
            if (selectedFlag && (i != baseBranch) && (recursionHeadLogicOperator.at(i) != OPERATOR_NOT)) {
                src = switchCount++ % 2;
                tgt = 1 - src;
                aux[tgt].clear();
                TypeFrameSet newSet;
                for (unsigned int b = 0; b < cleanRecursionQueryResult.at(i).size(); b++) {
                    for (unsigned int a = 0; a < aux[src].size(); a++) {
                        if (compatibleVarMappings(aux[src].at(a).second, cleanRecursionQueryResult.at(i).at(b).second)) {
                            TypeFrameSet newSet;
                            VarMapping newMapping;
                            typeFrameSetUnion(newSet, aux[src].at(a).first, cleanRecursionQueryResult.at(i).at(b).first);
                            varMappingUnion(newMapping, aux[src].at(a).second, cleanRecursionQueryResult.at(i).at(b).second);
                            if (DEBUG) {
                                printf("(AND) Adding solution to result:\n");
                                printf("Base:\n");
                                printTypeFrameSet(aux[src].at(a).first);
                                printVarMapping(aux[src].at(a).second);
                                printf("Adding:\n");
                                printTypeFrameSet(cleanRecursionQueryResult.at(i).at(b).first);
                                printVarMapping(cleanRecursionQueryResult.at(i).at(b).second);
                                printf("Union:\n");
                                printTypeFrameSet(newSet);
                                printVarMapping(newMapping);
                            }
                            aux[tgt].push_back(std::make_pair(newSet, newMapping));
                        } else {
                            if (DEBUG) {
                                printf("(AND) rejecting non-compatible var maps:\n");
                                printVarMapping(aux[src].at(a).second);
                                printf("-\n");
                                printVarMapping(cleanRecursionQueryResult.at(i).at(b).second);
                            }
                        }
                    }
                }
            }
        }
        for (unsigned int j = 0; j < aux[tgt].size(); j++) {
            answer.push_back(aux[tgt].at(j));
        }
    } else if (OrFlag) {
        if (DEBUG) printf("Start processing OR\n");
        logicOperator = OPERATOR_OR;
        for (unsigned int i = 0; i < arity; i++) {
            for (unsigned int j = 0; j < recursionQueryResult.at(i).size(); j++) {
                if (! isForbiddenMapping(recursionQueryResult.at(i).at(j).second, recursionForbiddenMappings.at(i))) {
                    if (DEBUG) {
                        printf("(OR) Adding solution to result:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                        printVarMapping(recursionQueryResult.at(i).at(j).second);
                    }
                    answer.push_back(recursionQueryResult.at(i).at(j));
                } else {
                    if (DEBUG) {
                        printf("(OR) Rejecting solution:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                        printVarMapping(recursionQueryResult.at(i).at(j).second);
                    }
                }
            }
        }
    } else if (NotFlag) {
        if (DEBUG) printf("Start processing NOT\n");
        logicOperator = OPERATOR_NOT;
        // arity is 1
        for (unsigned int j = 0; j < recursionQueryResult.at(0).size(); j++) {
            if (DEBUG) {
                printf("(NOT) Adding solution to result:\n");
                printTypeFrameSet(recursionQueryResult.at(0).at(j).first);
                printVarMapping(recursionQueryResult.at(0).at(j).second);
            }
            answer.push_back(recursionQueryResult.at(0).at(j));
            if (! isForbiddenMapping(recursionQueryResult.at(0).at(j).second, recursionForbiddenMappings.at(0))) {
                if (DEBUG) {
                    printf("(NOT) Adding forbidden mapping:\n");
                    printVarMapping(recursionQueryResult.at(0).at(j).second);
                }
                forbiddenMappings.push_back(recursionQueryResult.at(0).at(j).second);
            }
        }
    } else {
        logicOperator = OPERATOR_NOP;
        IntPairVector constraints;
        StringMap variableOccurrences;
        buildQueryTerm(keyExpression, variableOccurrences, queryFrame, cursor);
        buildConstraints(constraints, variableOccurrences);
        if (DEBUG) {
            keyExpression.printForDebug("Key: ", "\n", true);
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
                    VarMapping varMap;
                    TypeFrameSet frameSet;
                    for (StringMap::iterator it = variableOccurrences.begin(); it != variableOccurrences.end(); it++) {
                        varMap.insert(VarMapping::value_type((*it).first, frames.at(*it2).subFrameAt(*((*it).second.begin()))));
                    }
                    frameSet.insert(frames.at(*it2));
                    if (DEBUG) {
                        printf("(LEAF) Adding solution to result:\n");
                        printTypeFrameSet(frameSet);
                        printVarMapping(varMap);
                    }
                    answer.push_back(std::make_pair(frameSet, varMap));
                }
            }
        }
    }
    if (DEBUG) {
        printf("Answer:\n");
        printRecursionResult(answer);
    }
}

void TypeFrameIndex::printVarMapping(const VarMapping &map) const
{
    for (VarMapping::const_iterator it = map.begin(); it != map.end(); it++) {
        printf("%s = ", (*it).first.c_str());
        (*it).second.printForDebug("", "\n", true);
    }
}

void TypeFrameIndex::printTypeFrameSet(const TypeFrameSet &set) const
{
    for (TypeFrameSet::const_iterator it = set.begin(); it != set.end(); it++) {
        (*it).printForDebug("", "\n", true);
    }
}

void TypeFrameIndex::printRecursionResult(const std::vector<ResultPair> &v) const
{
    for (unsigned int i = 0; i < v.size(); i++) {
        printf("Solution %u\n" , i);
        printTypeFrameSet(v.at(i).first);
        printVarMapping(v.at(i).second);
    }
}

void TypeFrameIndex::printForDebug(bool showNodeNames) const
{
    PatternMap::const_iterator it1 = occurrenceSet.begin();
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

