/*
 * TypeFrame.h
 *
 * Copyright (C) 2017 OpenCog Foundation
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
 * TypeFrame is a representation of an atom in a format that make it easier to
 * use to build an index.
 *
 * It is a vector of pairs (T, A) representing a subgraph 
 * (T = atom type, A = atom arity).
 *
 * Examples.
 *
 * (WordNode "name") 
 *
 * [(WORD_NODE, 0)]
 *
 * (SimilarityLink 
 *   (ConceptNode "man") 
 *   (ConceptNode "monkey")
 * )
 *
 * [(SIMILARITY_LINK, 2) (ConceptNode 0) (ConceptNode 0)]
 *
 * (EvaluationLink
 *   (PredicateNode "stateOf")
 *   (ListLink
 *     (ConceptNode "Florida")
 *     (COnceptNode "USA")
 *   )
 * )
 *
 * [(EVALUATION_LINK, 2) (PREDICATE_NODE 0) (LIST_LINK 2) (CONCEPT_NODE 0) (CONCEPT_NODE 0)]
 *
 * Node names are also stored but in a separate hash table (hash key is the
 * position of the node in the above array).
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

    bool isStarPattern(const TypePair &pair) const;
    bool subFrameEqual(unsigned int cursor,
                       const TypeFrame &other,
                       unsigned int otherCursor);
    unsigned int getNextAtomPos(unsigned int cursor) const;
    bool buildFrameRepresentation(const std::string &schemeRepresentation);
    void recursiveHandleTraverse(Handle handle);
    int countTargets(const std::string &txt, unsigned int begin);
    int recursiveParse(const std::string &txt, unsigned int begin);
    bool isEquivalent(const TypeFrame &other,
                      int cursorThis,
                      int cursorOther) const;
    int compareUsingEquivalence(const TypeFrame &other,
                                int cursorThis,
                                int cursorOther) const;
    int lineComparisson(const std::vector<std::vector<int>> &matrix) const;
    bool isFeasible(const std::vector<std::vector<bool>> &matrix) const;

    void error(std::string message);
    void check();

public:

    TypeFrame(const std::string &schemeRepresentation);
    TypeFrame(Handle handle);
    TypeFrame();
    ~TypeFrame();

    static TypePair STAR_PATTERN;
    static std::string STAR_NODE_NAME;
    static TypeFrame EMPTY_PATTERN;
    struct LessThan {
        bool operator()(const TypeFrame &a, const TypeFrame &b) const {
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
    struct LessThanUsingEquivalence {
        bool operator()(const TypeFrame &a, const TypeFrame &b) const {
            return a.compareUsingEquivalence(b, 0, 0);
        }
    };

    bool isValid() const;
    void clear();
    std::vector<int> getArgumentsPosition(unsigned int cursor) const;
    void buildNodesSet(std::set<TypeFrame,
                       TypeFrame::LessThan> &answer,
                       bool happensTwiceOrMoreOnly = false,
                       bool excludeVariableNodes = false) const;
    TypeFrame buildSignature(unsigned int cursor);
    bool equals(const TypeFrame &other) const;
    bool isEquivalent(const TypeFrame &other) const;
    int compareUsingEquivalence(const TypeFrame &other) const;
    bool containsEquivalent(const TypeFrame &other,
                            unsigned int cursor = 0) const;
    bool nodeNameDefined(unsigned int pos) const;
    std::string nodeNameAt(unsigned int pos) const;
    void setNodeNameAt(unsigned int pos, std::string name);
    bool typeAtIsSymmetricLink(unsigned int pos) const;
    bool typeAtEqualsTo(unsigned int pos, const std::string &typeName) const;
    void append(const TypeFrame &other);
    void pickAndPushBack(const TypeFrame &other, unsigned int pos);
    bool match(std::vector<int> &mapping, const TypeFrame &pattern) const;
    bool match(std::vector<int> &mapping,
               const TypeFrame &pattern,
               const IntPairVector &constraints) const;
    bool subFramesEqual(unsigned int cursorA, unsigned int cursorB) const;
    TypeFrame subFrameAt(int pos) const;
    bool topLevelIsLink() const;
    TypeFrame copyReplacingFrame(const TypeFrame &key,
                                 const TypeFrame &frame) const;
    bool nonEmptyNodeIntersection(const TypeFrame &other) const;
    std::string toSCMString(unsigned int cursor = 0) const;
    void printForDebug(std::string prefix = "",
                       std::string suffix = "",
                       bool showNames = true) const;

};
}

#endif // _OPENCOG_TYPEFRAME_H
