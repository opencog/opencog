#include "TypeFrame.h"
#include <opencog/atoms/proto/NameServer.h>

using namespace opencog;
using namespace std;

// TODO: Change the constant value
TypePair TypeFrame::STAR_PATTERN = make_pair(1000, 1000);
// TODO: Change the constant value
string TypeFrame::STAR_NODE_NAME = "*";
TypeFrame TypeFrame::EMPTY_PATTERN;

TypeFrame::TypeFrame(const string &schemeRepresentation) 
{
    validInstance = ! buildFrameRepresentation(schemeRepresentation);
}

TypeFrame::TypeFrame(Handle handle)
{
    validInstance = true;
    recursiveHandleTraverse(handle);
}

TypeFrame::TypeFrame() 
{
    validInstance = true;
}

TypeFrame::~TypeFrame() 
{
}

bool TypeFrame::isValid() const
{
    return validInstance;
}

bool TypeFrame::nodeNameDefined(unsigned int pos) const
{
    return (nodeNameMap.find(pos) != nodeNameMap.end());
}

bool TypeFrame::typeAtEqualsTo(unsigned int pos, Type type) const
{
    return at(pos).first == type;
}

bool TypeFrame::typeAtIsSymmetricLink(unsigned int pos) const
{
    return (nameserver().isA(at(pos).first, UNORDERED_LINK));
}

string TypeFrame::nodeNameAt(unsigned int pos) const
{
    string answer = "";
    NodeNameMap::const_iterator it = nodeNameMap.find(pos);
    if (it == nodeNameMap.end()) {
        fprintf(stderr, "Attempting to get name at non-node position: %d\n", pos);
    } else {
        answer = it->second;
    }
    return answer;
}

void TypeFrame::setNodeNameAt(unsigned int pos, const string& name)
{
    NodeNameMap::iterator it = nodeNameMap.find(pos);
    if (it == nodeNameMap.end()) {
        nodeNameMap.emplace(pos, name);
    } else {
        it->second = name;
    }
}

void TypeFrame::append(const TypeFrame &other)
{
    for (unsigned int i = 0; i < other.size(); i++) {
        push_back(other.at(i));
        if (other.nodeNameDefined(i)) {
            setNodeNameAt(size() - 1, other.nodeNameAt(i));
        }
    }
}

void TypeFrame::pickAndPushBack(const TypeFrame &other, unsigned int pos)
{
    push_back(other.at(pos));
    if (other.nodeNameDefined(pos)) {
        setNodeNameAt(size() - 1, other.nodeNameAt(pos));
    }
}

vector<int> TypeFrame::getArgumentsPosition(unsigned int cursor) const
{
    vector<int> answer;

    int nargs = at(cursor).second;
    while (nargs > 0) {
        cursor++;
        answer.push_back(cursor);
        nargs--;
        if (nargs > 0) {
            int skip = at(cursor).second;
            while (skip > 0) {
                cursor++;
                skip += at(cursor).second;
                skip--;
            }
        }
    }

    return answer;
}

TypeFrame TypeFrame::buildSignature(unsigned int cursor)
{
    TypeFrame answer;

    for (int pos : getArgumentsPosition(cursor)) {
        answer.push_back(at(pos));
        if (nodeNameDefined(pos)) {
            answer.setNodeNameAt(answer.size() - 1, nodeNameAt(pos));
        }
    }

    return answer;
}

bool TypeFrame::isStarPattern(const TypePair &pair) const
{
    return ((pair.first == STAR_PATTERN.first) && 
            (pair.second == STAR_PATTERN.second));
}

bool TypeFrame::subFrameEqual(unsigned int cursor,
                              const TypeFrame &other,
                              unsigned int otherCursor)
{
    bool answer = true;

    int numArgs = 1;
    while (numArgs > 0) {
        if ((cursor == size()) || (otherCursor == other.size())) {
            answer = false;
            break;
        }
        if ((at(cursor).first != other.at(otherCursor).first) || 
            (at(cursor).second != other.at(otherCursor).second))
        {
            answer = false;
            break;
        }
        numArgs--;
        numArgs += at(cursor).second;
        cursor++;
        otherCursor++;
    }

    return answer;
}

bool TypeFrame::subFramesEqual(unsigned int cursorA,
                               unsigned int cursorB) const
{
    bool answer = true;

    int numArgs = 1;
    while (numArgs > 0) {
        if ((cursorA == size()) || (cursorB == size())) {
            answer = false;
            break;
        }
        if ((at(cursorA).first != at(cursorB).first) || 
            (at(cursorA).second != at(cursorB).second) ||
            (nodeNameDefined(cursorA) != nodeNameDefined(cursorB)) ||
            (nodeNameDefined(cursorA) && (nodeNameAt(cursorA).compare(nodeNameAt(cursorB)) != 0)))
        {
            answer = false;
            break;
        }
        numArgs--;
        numArgs += at(cursorA).second;
        cursorA++;
        cursorB++;
    }

    return answer;
}

unsigned int TypeFrame::getNextAtomPos(unsigned int cursor) const
{
    int nargs = 1;
    while (nargs > 0) {
        if (cursor >= size()) {
            cursor = -1;
            break;
        }
        nargs--;
        nargs += at(cursor).second;
        cursor++;
    }

    return cursor;
}

bool TypeFrame::match(vector<int> &mapping, const TypeFrame &pattern) const
{
    bool answer = true;
    unsigned int patternCursor = 0;
    unsigned int cursor = 0;

    mapping.clear();
    while (patternCursor < pattern.size()) {
        if (cursor >= size()) {
            answer = false;
            break;
        }
        if (isStarPattern(pattern.at(patternCursor))) {
            mapping.push_back(cursor);
            patternCursor++;
            cursor = getNextAtomPos(cursor);
        } else {
            if ((at(cursor).first != pattern.at(patternCursor).first) || 
                (at(cursor).second != pattern.at(patternCursor).second) || 
                (pattern.nodeNameDefined(patternCursor) && (pattern.nodeNameAt(patternCursor).compare(nodeNameAt(cursor)) != 0)))
            {
                answer = false;
                break;
            } else {
                mapping.push_back(cursor);
                patternCursor++;
                cursor++;
            }
        }
    }

    if (cursor < size()) answer = false;
    if (! answer) mapping.clear();

    return answer;
}

bool TypeFrame::match(vector<int> &mapping,
                      const TypeFrame &pattern,
                      const IntPairVector &constraints) const
{
    bool answer = true;

    if (match(mapping, pattern)) {
        for (const auto& constraint : constraints) {
            if (! subFramesEqual(mapping.at(constraint.first),
                                 mapping.at(constraint.second))) {
                answer = false;
                break;
            }
        }
    } else {
        answer = false;
    }

    return answer;
}

TypeFrame TypeFrame::subFrameAt(int pos) const
{
    TypeFrame answer;

    int numArgs = 1;
    while (numArgs > 0) {
        answer.push_back(at(pos));
        if (nodeNameDefined(pos)) {
            answer.setNodeNameAt(answer.size() - 1, nodeNameAt(pos));
        }
        numArgs--;
        numArgs += at(pos).second;
        pos++;
    }

    return answer;
}

bool TypeFrame::equals(const TypeFrame &other) const
{
    bool answer = true;

    if (other.size() == size()) {
        for (unsigned int i = 0; i < size(); i++) {
            if ((at(i).first != other.at(i).first) || 
                (at(i).second != other.at(i).second) ||
                (nodeNameDefined(i) != other.nodeNameDefined(i)) || 
                (nodeNameDefined(i) && (nodeNameAt(i).compare(other.nodeNameAt(i)) != 0)))
            {
                answer = false;
                break;
            }
        }
    } else {
        answer = false;
    }

    return answer;
}

int TypeFrame::lineComparisson(const vector<vector<int>> &matrix) const
{
    int n = (int) matrix.size();
    int answer = 0;

    bool outterFlag = true;
    for (int i = 0; i < n; i++) {
        bool flag = true;
        for (int j = 0; j < n; j++) {
            if (matrix[i][j] > 0) {
                flag = false;
                break;
            }
        }
        if (flag) {
            answer = -1;
            outterFlag = false;
            break;
        }
    }
    if (outterFlag) {
        for (int j = 0; j < n; j++) {
            bool flag = true;
            for (int i = 0; i < n; i++) {
                if (matrix[i][j] < 0) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                answer = 1;
                break;
            }
        }
    }

    return answer;
}

bool TypeFrame::isFeasible(const vector<vector<bool>> &matrix) const
{
    int n = (int) matrix.size();
    int mapping[n];
    int lineCursor[n];

    for (int i = 0; i < n; i++) {
        lineCursor[i] = 0;
    }

    int mappingCursor = 0;
    while (mappingCursor < n) {
        int candidate = lineCursor[mappingCursor];

        if (candidate == n) {
            return false;
        }
        if (matrix.at(mappingCursor).at(candidate)) {
            bool flag = true;
            for (int i = mappingCursor - 1; i >= 0; i--) {
                if (mapping[i] == candidate) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                mapping[mappingCursor] = candidate;
                mappingCursor++;
            } else {
                lineCursor[mappingCursor]++;
            }
        } else {
            lineCursor[mappingCursor]++;
        }
        while (lineCursor[mappingCursor] == n) {
            lineCursor[mappingCursor] = 0;
            mappingCursor--;
            if (mappingCursor < 0) {
                return false;
            }
            lineCursor[mappingCursor]++;
        }
    }

    return true;
}

bool TypeFrame::isEquivalent(const TypeFrame &other, int cursorThis, int cursorOther) const
{
    bool answer = true;

    if (DEBUG) {
        printf("isEquivalent() %d %d\n", cursorThis, cursorOther);
        printForDebug("this: ", "\n", true);
        other.printForDebug("other:", "\n", true);
    }

    if ((at(cursorThis).first != other.at(cursorOther).first) ||
        (at(cursorThis).second != other.at(cursorOther).second))
    {
        answer = false;
    } else {
        if (at(cursorThis).second == 0) {
            if (nodeNameAt(cursorThis).compare(other.nodeNameAt(cursorOther)) != 0) {
                answer = false;
            }
        } else {
            vector<int> argPosThis = getArgumentsPosition(cursorThis);
            vector<int> argPosOther = other.getArgumentsPosition(cursorOther);
            if (typeAtIsSymmetricLink(cursorThis)) {
                int n = argPosThis.size();
                vector<vector<bool>> equivalent;
                vector<bool> aux;
                for (int i = 0; i < n; i++) {
                    aux.clear();
                    equivalent.push_back(aux);
                    for (int j = 0; j < n; j++) {
                        bool f = isEquivalent(other, argPosThis.at(i), argPosOther.at(j));
                        equivalent.at(i).push_back(f);
                    }
                }
                // TODO XXX
                // Ensure this function gets called
                answer = isFeasible(equivalent);
            } else {
                for (unsigned int i = 0; i < argPosThis.size(); i++) {
                    if (! isEquivalent(other, argPosThis.at(i), argPosOther.at(i))) {
                        answer = false;
                        break;
                    }
                }
            }
        }
    }

    if (DEBUG) printf("EQUIVALENT: %s\n", (answer ? "TRUE" : "FALSE"));

    return answer;
}

int TypeFrame::compareUsingEquivalence(const TypeFrame &other) const
{
    return compareUsingEquivalence(other, 0, 0);
}

int TypeFrame::compareUsingEquivalence(const TypeFrame &other, int cursorThis, int cursorOther) const
{
    int answer = 0;

    if (DEBUG) {
        printf("lessThanUsingEquivalence() %d %d\n", cursorThis, cursorOther);
        printForDebug("this: ", "\n", true);
        other.printForDebug("other:", "\n", true);
    }

    if (at(cursorThis).first < other.at(cursorOther).first) {
        answer = -1;
    } else if (at(cursorThis).first > other.at(cursorOther).first) {
        answer = 1;
    } else if (at(cursorThis).second < other.at(cursorOther).second) {
        answer = -1;
    } else if (at(cursorThis).second > other.at(cursorOther).second) {
        answer = 1;
    } else {
        if (at(cursorThis).second == 0) {
            answer = nodeNameAt(cursorThis).compare(other.nodeNameAt(cursorOther));
        } else {
            vector<int> argPosThis = getArgumentsPosition(cursorThis);
            vector<int> argPosOther = other.getArgumentsPosition(cursorOther);
            if (typeAtIsSymmetricLink(cursorThis)) {
                int n = argPosThis.size();
                vector<vector<bool>> equivalent;
                vector<bool> aux1;
                vector<vector<int>> comparisson;
                vector<int> aux2;
                vector<int> zeroLine;
                vector<int> zeroColumn;
                for (int i = 0; i < n; i++) {
                    aux1.clear();
                    equivalent.push_back(aux1);
                    comparisson.push_back(aux2);
                    for (int j = 0; j < n; j++) {
                        int comp = compareUsingEquivalence(other, argPosThis.at(i), argPosOther.at(j));
                        //bool f = isEquivalent(other, argPosThis.at(i), argPosOther.at(j));
                        equivalent.at(i).push_back(comp == 0);
                        comparisson.at(i).push_back(comp);
                        if (comp == 0) {
                            zeroLine.push_back(i);
                            zeroColumn.push_back(j);
                        }
                    }
                }
                // TODO XXX
                //printf(" \n");
                if (! isFeasible(equivalent)) {
                    for (int i : zeroLine) {
                        for (int j = 0; j < n; j++) {
                            comparisson[i][j] = 0;
                        }
                    }
                    for (int j : zeroColumn) {
                        for (int i = 0; i < n; i++) {
                            comparisson[i][j] = 0;
                        }
                    }
                    answer = lineComparisson(comparisson);
                }
            } else {
                for (unsigned int i = 0; i < argPosThis.size(); i++) {
                    int comp = compareUsingEquivalence(other, argPosThis.at(i), argPosOther.at(i));
                    if (comp != 0) {
                        answer = comp;
                        break;
                    }
                }
            }
        }
    }

    if (DEBUG) printf("Compare: %d\n", answer);

    return answer;
}

bool TypeFrame::isEquivalent(const TypeFrame &other) const
{
    return isEquivalent(other, 0, 0);
}

bool TypeFrame::containsEquivalent(const TypeFrame &other, unsigned int cursor) const
{
    bool answer = false;

    if (DEBUG) {
        printf("containsEquivalent()\n");
        printForDebug("this: ", "\n", true);
        other.printForDebug("other:", "\n", true);
    }

    if (isEquivalent(other, cursor, 0)) {
        answer = true;
    } else {
        for (int pos : getArgumentsPosition(cursor)) {
            if (containsEquivalent(other, pos)) {
                answer = true;
                break;
            }
        }
    }

    if (DEBUG) printf("CONTAINS: %s\n", (answer ? "TRUE" : "FALSE"));

    return answer;
}

bool TypeFrame::topLevelIsLink() const
{
    return (at(0).second > 0);
}

bool TypeFrame::nonEmptyNodeIntersection(const TypeFrame &other) const
{
    for (unsigned int i = 0; i < size(); i++) {
        if (at(i).second == 0) {
            for (unsigned int j = 0; j < other.size(); j++) {
                if ((at(i).first == other.at(j).first) && (nodeNameAt(i).compare(other.nodeNameAt(j)) == 0)) {
                    return true;
                }
            }
        }
    }
    return false;
}

void TypeFrame::buildNodesSet(set<TypeFrame,
                              TypeFrame::LessThan> &answer,
                              const set<Type> &allowed,
                              bool happensTwiceOrMoreOnly) const
{
    answer.clear();
    set<TypeFrame, TypeFrame::LessThan> aux;

    TypeFrame frame;
    for (unsigned int i = 0; i < size(); i++) {
        if (at(i).second == 0) {
            if (allowed.find(at(i).first) != allowed.end()) {
                frame.clear();
                frame.pickAndPushBack(*this, i);
                if (happensTwiceOrMoreOnly) {
                    if (aux.find(frame) == aux.end()) {
                        aux.insert(frame);
                    } else {
                        answer.insert(frame);
                    }
                } else {
                    answer.insert(frame);
                }
            }
        }
    }
}

TypeFrame TypeFrame::copyReplacingFrame(const TypeFrame &key,
                                        const TypeFrame &frame) const
{
    TypeFrame answer;
    TypeFrame aux;

    if (at(0).second == 0) {
        if (equals(key)) {
            answer.append(frame);
        } else {
            answer.append(*this);
        }
    } else {
        answer.push_back(at(0));
        for (int pos : getArgumentsPosition(0)) {
            aux.clear();
            aux = subFrameAt(pos);
            // TODO XXX
            // perhaps using isEquivalent() instead of equals() makes more sense
            if (aux.equals(key)) {
                answer.append(frame);
            } else {
                answer.append(aux.copyReplacingFrame(key, frame));
            }
        }
    }

    return answer;
}

string TypeFrame::toSCMString(unsigned int cursor) const
{
    string answer = "(";
    answer += nameserver().getTypeName(at(cursor).first) + " ";
    if (at(cursor).second == 0) {
        answer += "\"" + nodeNameAt(cursor) + "\"";
    } else {
        for (int pos : getArgumentsPosition(cursor)) {
            answer += toSCMString(pos);
        }
    }
    answer += ") ";

    return answer;
}

void TypeFrame::printForDebug(string prefix, string suffix, bool showNames) const
{
    printf("%s", prefix.c_str());
    for (unsigned int i = 0; i < size(); i++) {
        if (showNames && (at(i).first < 1000)) {
            printf("(%s %lu) ", nameserver().getTypeName(at(i).first).c_str(), at(i).second);
        } else {
            if (at(i).first != 1000) {
                printf("(%u %lu) ", at(i).first, at(i).second);
            } else {
                printf("(*) ");
            }
        }
        if (at(i).second == 0) {
            NodeNameMap::const_iterator it = nodeNameMap.find(i);
            if (it == nodeNameMap.end()) {
                printf("\"*\" ");
            } else {
                printf("\"%s\" ", it->second.c_str());
            }
        }
    }
    printf("%s", suffix.c_str());
}

bool TypeFrame::buildFrameRepresentation(const string &schemeTxt)
{
    if (DEBUG) printf("0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\n");
    if (DEBUG) printf("%s\n", schemeTxt.c_str());
    if (recursiveParse(schemeTxt, 0) < 0) {
        if (DEBUG) fprintf(stderr, "Invalid scheme: %s\n", schemeTxt.c_str());
        return true;
    } else {
        //printForDebug();
    }
    return false;
}

void TypeFrame::recursiveHandleTraverse(Handle handle)
{
    bool isLink = handle->is_link();
    unsigned int n = (isLink ? handle->get_arity() : 0);
    emplace_back(handle->get_type(), n);
    if (isLink) {
        for (unsigned int i = 0; i < n; i++) {
            recursiveHandleTraverse(handle->getOutgoingAtom(i));
        }
    } else {
        setNodeNameAt(size() - 1, handle->get_name());
    }
}

int TypeFrame::countTargets(const string &txt, unsigned int begin)
{

    int answer = 0;
    unsigned int cursor = begin;
    int level = 0;
    bool inStr = false;
    while (true) {
        if (cursor >= txt.length()) return -1;
        char c = txt.at(cursor++);
        if (inStr) {
            if (c == '\"') inStr = false;
        } else {
            if (c == '\"') {
                inStr = true;
            } else {
                if (c == '(') {
                    if (level == 0) answer++;
                    level++;
                }
                if (c == ')') {
                    if (level == 0) return answer;
                    level--;
                }
            }
        }
    }
}

int TypeFrame::recursiveParse(const string &txt, unsigned int begin)
{
    if (txt.at(begin) != '(') {
        fprintf(stderr, "Expected \'(\' at index %d of string %s\n", begin, txt.c_str());
        return -1;
    }
    if (DEBUG) printf("recursiveParse() begin = %d\n", begin);
    int separatorPos = txt.find_first_of(" (", begin + 1);
    if (DEBUG) printf("separatorPos = %d\n", separatorPos);
    string typeName = txt.substr(begin + 1, separatorPos - begin - 1);
    if (DEBUG) printf("typeName = %s\n", typeName.c_str());
    Type type = nameserver().getType(typeName);
    if (type == NOTYPE) {
        fprintf(stderr, "Unknown type name: %s\n", typeName.c_str());
        return -1;
    }
    if (nameserver().isLink(type)) {
        if (DEBUG) printf("isLink\n");
        int tvBegin = txt.find_first_of("(", separatorPos);
        if (tvBegin == -1) {
            error("Could not parse link");
            return -1;
        }
        if (DEBUG) printf("tvBegin = %d\n", tvBegin);
        string checkTV = txt.substr(tvBegin + 1, 3);
        if (DEBUG) printf("checkTV = %s\n", checkTV.c_str());
        int targetBegin;
        if (checkTV == "stv") {
            int tvEnd = txt.find_first_of(")", tvBegin + 1);
            if (DEBUG) printf("tvEnd = %d\n", tvEnd);
            targetBegin = txt.find_first_of("(", tvEnd + 1);
        } else {
            targetBegin = tvBegin;
        }
        if (DEBUG) printf("targetBegin = %d\n", targetBegin);
        int targetEnd = 0;
        Arity targetCount = countTargets(txt, targetBegin);
        if (DEBUG) printf("targetCount = %lu\n", targetCount);
        if (targetCount < 0){
            error("Could not compute targetCount");
            return -1;
        }
        push_back(TypePair(type, targetCount));
        for (unsigned i = 0; i < targetCount; i++) {
            if (DEBUG) printf("Processing target #%u\n", i);
            targetEnd = recursiveParse(txt, targetBegin);
            if (DEBUG) printf("Done with target#%u\n", i);
            if (DEBUG) printf("targetEnd = %d\n", targetEnd);
            if (targetEnd < 0) return -1;
            if (i != (targetCount -1)) {
                targetBegin = txt.find_first_of("(", targetEnd + 1);
                if (DEBUG) printf("targetBegin = %d\n", targetBegin);
            }
        }
        int DUMMY = txt.find_first_of(")", targetEnd + 1);
        if (DEBUG) printf("Ready to return %d\n", DUMMY);
        return DUMMY;
    } else {
        if (DEBUG) printf("isNode\n");
        push_back(TypePair(type, 0));
        int cursor = separatorPos + 2;
        bool nameEndedFlag = false;
        int level = 1;
        string nodeName = "";
        while (true) {
            char c = txt.at(cursor++);
            if (c == '\"') {
                nameEndedFlag = true;
                nodeNameMap.emplace(size() - 1, nodeName);
            } else {
                if (nameEndedFlag) {
                    if (c == '(') level++;
                    if ((c == ')') && (--level == 0)) {
                        if (DEBUG) printf("Ready to return %d\n", (cursor - 1));
                        return cursor - 1;
                    }
                } else {
                    nodeName += c;
                }
            }
        }
    }
}

bool TypeFrame::check() const
{
    unsigned int i;
    int pending = 1;
    for (i = 0; i < size(); i++) {
        pending += at(i).second;
        pending--;
        if (pending <= 0) {
            break;
        }
    }
    return ((i == (size() - 1)) && (pending == 0));
}

void TypeFrame::computeHashCode()
{
    hashCode = 5;
    // TODO: probably this is not a good hash function.
    for (unsigned int i = 0; i < size(); i++) {
        boost::hash_combine(hashCode, at(i).first);
        boost::hash_combine(hashCode, at(i).second);
        if (nodeNameDefined(i)) {
            boost::hash_combine(hashCode, std::hash<std::string>()(nodeNameAt(i)));
        }
    }
}

void TypeFrame::clear()
{
    nodeNameMap.clear();
    vector<TypePair>::clear();
}

void TypeFrame::error(string message)
{
    fprintf(stderr, "Parse error. %s\n", message.c_str());
}

