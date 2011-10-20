/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
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

/*Time:
·	MomentNode
·	TimeIntervalNode

Functions of time intervals
·	Member
		(Member t T) means the moment t lies in the interval T
·	length
·	start
·	end
·	ImmediatelyAfter 
		(t2,t3) is immediately after (t0,t1) if t0<=t2<=t1 and t3>t1

Space:
·	ObservedDistanceTo
·	ObservedDirectionTo
·	ObservedVectorTo

Compound Procedures
·	Increasing
	A predicate F that takes time as an input is Increasing on interval T if F is bigger at the end of T than at the start
·	Decreasing
*/

#ifndef __SPACETIME_H__
#define __SPACETIME_H__

#include <types.h>

using namespace opencog;

struct Vector1D { float x1; };
struct Vector3D { float x1; float x2; float x3; };

#define exec_assert(__t) if (!(__t)) { cprintf(3, "Wrong Nr of args.\n"); return NULL; }

//! @todo Standardize this
//const int MAX_COUNT = 100000;

#define USE_TIME_SERVER 1

typedef unsigned long timeUnit;

Handle AddAsNumberNode(int N);
int intFromNumberNode(Handle h);

Handle AddCrispEvaluationLink(	const char* predName,
							Handle arg1,
							Handle arg2,
							bool tv);

struct bin_op_plus {
	int operator()(int a, int b)
	{ return a+b; }
};
struct bin_op_minus {
	int operator()(int a, int b)
	{ return a-b; }
};
struct un_op_plus {
	int operator()(int a)
	{ return a; }
};
struct un_op_minus {
	int operator()(int a)
	{ return -a; }
};
struct op_gt {
	bool operator()(int a, int b)
	{ return a>b; }
};
struct op_lt {
	bool operator()(int a, int b)
	{ return a<b; }
};
struct op_equal {
	bool operator()(int a, int b)
	{ return a==b; }
};

template<typename op, typename returnT>
struct BinaryOpSchema
{
	returnT operator()(Handle a, Handle b) {
		return (a && b)
				? op()(intFromNumberNode(a), intFromNumberNode(b))
				: ((returnT)0);
	}
			
/*	static Handle unary(Handle a, Handle b);
	static Handle plus(Handle a, Handle b);
	static Handle minus(Handle a, Handle b);
	static Handle greaterThan(Handle a, Handle b);
	static Handle lessThan(Handle a, Handle b);
	static Handle equal(Handle a, Handle b);		*/
};

template<typename op, typename returnT>
struct  UnaryOpSchema
{
	returnT operator()(Handle a) {
		return (a
				? op()(intFromNumberNode(a))
				: ((returnT)0));
	}
};

/** @class ExplitTimeApparatus
	Offers various time schemata, with explicit time arguments.
*/

class TimeApparatus
{
/**
	@param SchemaNode of type (NumberNode => Integer)
	@param ListLink of 2 NumberNodes or an interval stamp, dep. on policy.
	@param "direction of time"
	@return NumberNode denoting the length of the interval
*/

	static Handle timeDiff(Handle F, Handle interval,int sign);

public:
	
	/**
	@param An atom with an associated timestamp
	@return The NumberNode representation of the timestamp
	*/
	
	static Handle getMomentNode(Handle t);

	/**
	@param NumberNode (moment)
	@return The integer representation of the moment
	*/
	static timeUnit getMoment(Handle t);

/**
	@param ListLink of 2 NumberNodes
	@return NumberNode denoting the start moment
*/
	static Handle intervalStart(Handle interval);

/**
	@param ListLink of 2 NumberNodes
	@return NumberNode denoting the end moment
*/
	static Handle intervalEnd(Handle interval);

/**
	@param ListLink of 2 NumberNodes
	@return NumberNode denoting the length of the interval
*/
	static Handle intervalLength(Handle interval); 

/**
	@param NumberNode
	@param ListLink of 2 NumberNodes
	@return EvaluationLink denoting whether t is member of interval T.
*/
	static Handle intervalMember(Handle t, Handle T);
	
/**
	@param ListLink of 2 NumberNodes
	@param ListLink of 2 NumberNodes
	@return EvaluationLink denoting whether A is imm. after B
*/
	static Handle immediatelyAfter(Handle intervalA, Handle intervalB);
	/// Input: f:Schema-with-Moment-argument, T:interval
/**
	@param SchemaNode of type (NumberNode => Integer)
	@param ListLink of 2 NumberNodes
	@return EvaluationLink denoting whether F increases on the interval.
*/	
	static Handle increasing(Handle F, Handle interval);
/**
	@param SchemaNode of type (NumberNode => Integer)
	@param ListLink of 2 NumberNodes
	@return EvaluationLink denoting whether F decreases on the interval.
*/	
	static Handle decreasing(Handle F, Handle interval);
};

/** @class SpaceTimeApparatus
	Offers various space & time schemata.
*/

class SpaceTimeApparatus : public TimeApparatus
{
public:

#if USE_TIME_SERVER
	
/**
	@param PixelPerceptNode
	@param PixelPerceptNode
	@param Timestamp
	@return The 1-D distance between the pixels.
*/
// float
	static Handle observedDistanceBetween(Handle a, Handle b, Handle t);

/**
	@param PixelPerceptNode
	@param PixelPerceptNode
	@param Timestamp
	@return -1 if pos(a) < pos(b), 0 if pos(a)=pos(b), +1 otherwise.
*/
// int
	static Handle observedDirectionBetween(Handle a, Handle b, Handle t);

//	static float observedDistanceTo(Handle x, Handle t);
//	static float observedDirectionTo(Handle x, Handle t);
//	static Vector3D observedVectorTo3D(Handle xyz, Handle t);
//	static Vector1D observedVectorTo1D(Handle x, Handle t);

#endif
};

#endif //  __SPACETIME_H__
