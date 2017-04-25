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

/**
 *
 */
class TypeFrame 
{

public:

    typedef std::pair<Type, Arity> TypePair;

    class TypeVector: public std::vector<TypeFrame::TypePair> {
        bool operator <(TypeVector &other)
        {
            TypeVector::iterator it1 = this->begin();    
            TypeVector::iterator it2 = other.begin();    
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
                    }
                }
                it1++;
                it2++;
            }
            return (it2 != other.end());
        }
    };

    TypeFrame(const std::string &schemeRepresentation);
    ~TypeFrame();

    bool isValid();
    TypeVector buildSignatureVector(int cursor);
    int size();
    TypePair at(int pos);
    void printForDebug();
    

private:

    bool DEBUG = false;
    bool validInstance;
    std::vector<TypePair> parse;

    bool buildFrameRepresentation(const std::string &schemeRepresentation);

    int countTargets(const std::string &txt, int begin);
    int recursiveParse(const std::string &txt, int begin);
    void error(std::string message);

};
}

#endif // _OPENCOG_TYPEFRAME_H
