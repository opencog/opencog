/*
 * opencog/learning/PatternMiner/Pattern.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
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

#ifndef _OPENCOG_PATTERNMINER_PATTERN_H
#define _OPENCOG_PATTERNMINER_PATTERN_H
#include <map>
#include <vector>
#include <opencog/atomspace/AtomSpace.h>


using namespace std;

namespace opencog 
{
 namespace PatternMining
{

// class Skeleton
// {
//public:
//	vector<Handle> skeletonLinks;
//	Skeleton(){}
//    Skeleton(vector<Handle>& _skeletonLinks){skeletonLinks = _skeletonLinks;}

//    // Make sure the input Links are connected before calling this function
//    static Skeleton* extractSkeleton(vector<Handle> inputLinks);

// };

// class Pattern
// {
//public:
//	Skeleton* skeleton;
//	map<Handle, Handle> bindings; // bindings of the variables map<varaibleName, varaibleValue>

//    Pattern(Skeleton* _skeleton, map<Handle, Handle>& _bindings)
//	{
//		skeleton = _skeleton;
//        bindings = _bindings;
//    }
        
//    Pattern(Skeleton* _skeleton)
//	{
//		skeleton = _skeleton;
//    }

//    bool addVariableBinding(Handle variableH, Handle valueH);

// };

}
}

#endif //_OPENCOG_PATTERNMINER_PATTERN_H
