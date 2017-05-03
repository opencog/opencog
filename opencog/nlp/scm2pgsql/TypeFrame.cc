#include "TypeFrame.h"
#include <opencog/atoms/base/ClassServer.h>

using namespace opencog;
using namespace std;

// TODO: Change the constant value
TypePair TypeFrame::STAR_PATTERN = std::make_pair(1000, 1000);
// TODO: Change the constant value
std::string TypeFrame::STAR_NODE_NAME = "*";
TypeFrame TypeFrame::EMPTY_PATTERN;

TypeFrame::TypeFrame(const string &schemeRepresentation) 
{
    validInstance = ! buildFrameRepresentation(schemeRepresentation);
}

TypeFrame::TypeFrame() 
{
    validInstance = true;
}

TypeFrame::~TypeFrame() 
{
}

bool TypeFrame::lessThan(TypeFrame &other) const
{
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
printf("OPERATOR LESSTHAN\n");
printForDebug("", "", true);
other.printForDebug("", "", true);
    TypeFrame::const_iterator it1 = this->begin();    
    TypeFrame::const_iterator it2 = other.begin();    
    int cursor = 0;
    while (it1 < this->end()) {
        if (it2 == other.end()) return false;
        if ((*it1).first < (*it2).first) {
            return true;
        } else if ((*it1).first > (*it2).first) {
            return false;
        } else {
            if ((*it1).second < (*it2).second) {
                return true;
            } else if ((*it1).second > (*it2).second) {
                return false;
            } else {
                bool check1 = nodeNameDefined(cursor);
                bool check2 = other.nodeNameDefined(cursor);
printf("INNER CHECK\n");
printf("cursor: %d\n", cursor);
printf("check1: %s\n", (check1 ? "true" : "false"));
printf("check2: %s\n", (check2 ? "true" : "false"));
                if (!check1 && check2) {
                    return true;
                } else if (check1 && !check2) {
                    return false;
                } else if (check1 && check2) {
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
                    int comp = nodeNameAt(cursor).compare(other.nodeNameAt(cursor));
                    if (comp < 0) {
                        return true;
                    } else if (comp > 0) {
                        return false;
                    }
                }
            }
        }
        it1++;
        it2++;
        cursor++;
    }
    return (it2 != other.end());
}

bool TypeFrame::isValid() const
{
    return validInstance;
}

bool TypeFrame::nodeNameDefined(int pos) const
{
    return (nodeNameMap.find(pos) != nodeNameMap.end());
}

std::string TypeFrame::nodeNameAt(int pos) const
{
    std::string answer = "";
    NodeNameMap::const_iterator it = nodeNameMap.find(pos);
    if (it == nodeNameMap.end()) {
        fprintf(stderr, "Attempting to get name at non-node position: %d\n", pos);
    } else {
        answer = (*it).second;
    }
    return answer;
}

void TypeFrame::setNodeNameAt(int pos, std::string name)
{
    nodeNameMap.insert(NodeNameMap::value_type(pos, name));
}

void TypeFrame::append(TypeFrame &other)
{
    for (unsigned int i = 0; i < other.size(); i++) {
        push_back(other.at(i));
        if (other.nodeNameDefined(i)) {
            setNodeNameAt(size() - 1, other.nodeNameAt(i));
        }
    }
}

void TypeFrame::pickAndPushBack(TypeFrame &other, int pos)
{
    push_back(other.at(pos));
    if (other.nodeNameDefined(pos)) {
        setNodeNameAt(size() - 1, other.nodeNameAt(pos));
    }
}

TypeFrame TypeFrame::buildSignature(int cursor)
{
    TypeFrame answer;

    int nargs = at(cursor).second;
    while (nargs > 0) {
        cursor++;
        answer.push_back(at(cursor));
        if (nodeNameDefined(cursor)) {
            answer.setNodeNameAt(answer.size() - 1, nodeNameAt(cursor));
        }
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

void TypeFrame::printForDebug(std::string prefix, std::string suffix, bool showNames) const
{
    printf("%s", prefix.c_str());
    for (unsigned int i = 0; i < size(); i++) {
        if (showNames && (at(i).first < 1000)) {
            printf("(%s %d) ", classserver().getTypeName(at(i).first).c_str(), at(i).second);
        } else {
            if (at(i).first != 1000) {
                printf("(%d %d) ", at(i).first, at(i).second);
            } else {
                printf("(*) ");
            }
        }
        if (at(i).second == 0) {
            NodeNameMap::const_iterator it = nodeNameMap.find(i);
            if (it == nodeNameMap.end()) {
                printf("\"*\" ");
            } else {
                printf("\"%s\" ", (*it).second.c_str());
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

int TypeFrame::countTargets(const string &txt, int begin)
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

int TypeFrame::recursiveParse(const string &txt, int begin)
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
    Type type = classserver().getType(typeName);
    if (type == NOTYPE) {
        fprintf(stderr, "Unknown type name: %s\n", typeName.c_str());
        return -1;
    }
    if (classserver().isLink(type)) {
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
        if (DEBUG) printf("targetCount = %d\n", targetCount);
        if (targetCount < 0){
            error("Could not compute targetCount");
            return -1;
        }
        push_back(TypePair(type, targetCount));
        for (int i = 0; i < targetCount; i++) {
            if (DEBUG) printf("Processing target #%d\n", i);
            targetEnd = recursiveParse(txt, targetBegin);
            if (DEBUG) printf("Done with target#%d\n", i);
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
        std::string nodeName = "";
        while (true) {
            char c = txt.at(cursor++);
            if (c == '\"') {
                nameEndedFlag = true;
                nodeNameMap.insert(NodeNameMap::value_type(size() - 1, nodeName));
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

void TypeFrame::check()
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
    validInstance = ((i == (size() - 1)) && (pending == 0));
}

void TypeFrame::error(string message)
{
    fprintf(stderr, "Parse error. %s\n", message.c_str());
}

