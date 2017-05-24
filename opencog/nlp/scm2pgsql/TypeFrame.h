/*
 * TypeFrame.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_TYPEFRAME_H
#define _OPENCOG_TYPEFRAME_H

#include <opencog/atoms/base/Atom.h>
#include <string>
#include <vector>

namespace opencog
{

// XXX 
typedef std::pair<Type, Arity> TypePair;

/**
 *
 */
class TypeFrame: public std::vector<TypePair>
{

private:

    typedef std::map<int, std::string> NodeNameMap;
    typedef std::vector<std::pair<int, int>> IntPairVector;


    bool DEBUG = false;
    bool validInstance;
    NodeNameMap nodeNameMap;

    bool isStarPattern(const TypePair &pair);
    bool subFrameEqual(unsigned int cursor, const TypeFrame &other, unsigned int otherCursor);
    unsigned int getNextAtomPos(unsigned int cursor);
    bool buildFrameRepresentation(const std::string &schemeRepresentation);
    int countTargets(const std::string &txt, unsigned int begin);
    int recursiveParse(const std::string &txt, unsigned int begin);
    void error(std::string message);
    void check();

public:

    TypeFrame(const std::string &schemeRepresentation);
    TypeFrame();
    ~TypeFrame();

    static TypePair STAR_PATTERN;
    static std::string STAR_NODE_NAME;
    static TypeFrame EMPTY_PATTERN;
    struct LessThan {
        bool operator()(const TypeFrame &a, const TypeFrame &b) const {
            //printf("OPERATOR LESSTHAN\n");
            //a.printForDebug("", "", true);
            //b.printForDebug("", "", true);
            TypeFrame::const_iterator it1 = a.begin();    
            TypeFrame::const_iterator it2 = b.begin();    
            int cursor = 0;
            while (it1 < a.end()) {
                if (it2 == b.end()) return false;
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
                        bool check1 = a.nodeNameDefined(cursor);
                        bool check2 = b.nodeNameDefined(cursor);
                        //printf("INNER CHECK\n");
                        //printf("cursor: %d\n", cursor);
                        //printf("check1: %s\n", (check1 ? "true" : "false"));
                        //printf("check2: %s\n", (check2 ? "true" : "false"));
                        if (!check1 && check2) {
                            return true;
                        } else if (check1 && !check2) {
                            return false;
                        } else if (check1 && check2) {
                            int comp = a.nodeNameAt(cursor).compare(b.nodeNameAt(cursor));
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
            return (it2 != b.end());
        }
    };

    /*
    bool operator<(TypeFrame &other)
    {
    }
    */

    bool lessThan(TypeFrame &other) const;
    bool isValid() const;
    std::vector<int> getArgumentsPosition(unsigned int cursor) const;
    TypeFrame buildSignature(unsigned int cursor);
    bool equals(const TypeFrame &other) const;
    bool nodeNameDefined(unsigned int pos) const;
    std::string nodeNameAt(unsigned int pos) const;
    void setNodeNameAt(unsigned int pos, std::string name);
    bool typeAtIsSymmetricLink(unsigned int pos) const;
    bool typeAtEqualsTo(unsigned int pos, const std::string &typeName) const;
    void append(TypeFrame &other);
    void pickAndPushBack(const TypeFrame &other, unsigned int pos);
    bool match(std::vector<int> &mapping, const TypeFrame &pattern);
    bool match(std::vector<int> &mapping, const TypeFrame &pattern, const IntPairVector &constraints);
    bool subFramesEqual(unsigned int cursorA, unsigned int cursorB);
    TypeFrame subFrameAt(int pos) const;
    void printForDebug(std::string prefix = "", std::string suffix = "", bool showNames = false) const;

};
}

#endif // _OPENCOG_TYPEFRAME_H
