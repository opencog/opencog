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

    TypeFrame(const std::string &schemeRepresentation);
    bool isValid();
    ~TypeFrame();

private:

    bool validInstance;
    std::vector<int> parseAnswer;

    bool buildFrameRepresentation(const std::string &schemeRepresentation);

    int countTargets(const std::string &txt, int begin);
    int recursiveParse(const std::string &txt, int begin);
    void error(std::string message);
    bool fake_isLink(std::string t);
    bool fake_isNode(std::string t);
    int fake_getType(std::string typeName);

};
}

#endif // _OPENCOG_TYPEFRAME_H
