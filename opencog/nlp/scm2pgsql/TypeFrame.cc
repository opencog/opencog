#include "TypeFrame.h"
#include <opencog/atoms/base/ClassServer.h>

using namespace opencog;
using namespace std;

TypeFrame::TypeFrame(const string &schemeRepresentation) 
{
    validInstance = ! buildFrameRepresentation(schemeRepresentation);
}

TypeFrame::~TypeFrame() 
{
}

bool TypeFrame::isValid()
{
    return validInstance;
}

int TypeFrame::size()
{
    return parse.size();
}

TypeFrame::TypePair TypeFrame::at(int pos)
{
    return parse.at(pos);
}

TypeFrame::TypeVector TypeFrame::buildSignatureVector(int cursor)
{
    TypeVector answer;

    int nargs = at(cursor).second;
    while (nargs > 0) {
        cursor++;
        answer.push_back(at(cursor));
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

void TypeFrame::printForDebug()
{
    for (unsigned int i = 0; i < parse.size(); i++) {
        printf("(%d %d) ", parse.at(i).first, parse.at(i).second);
    }
    printf("\n");
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

bool TypeFrame::fake_isLink(string t) {
    return !fake_isNode(t);
}

bool TypeFrame::fake_isNode(string t) {
    return (t == "t3");
}

int TypeFrame::fake_getType(string typeName) {
    if (typeName == "t1") return 1;
    if (typeName == "t2") return 2;
    if (typeName == "t3") return 3;
    return -1;
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
        parse.push_back(TypePair(type, targetCount));
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
        parse.push_back(TypePair(type, 0));
        int cursor = separatorPos + 2;
        bool nameEndedFlag = false;
        int level = 1;
        while (true) {
            char c = txt.at(cursor++);
            if (c == '\"') {
                nameEndedFlag = true;
            } else {
                if (nameEndedFlag) {
                    if (c == '(') level++;
                    if ((c == ')') && (--level == 0)) {
                        if (DEBUG) printf("Ready to return %d\n", (cursor - 1));
                        return cursor - 1;
                    }
                }
            }
        }
    }
}

void TypeFrame::error(string message)
{
    if (DEBUG) fprintf(stderr, "Parse error. %s\n", message.c_str());
}

