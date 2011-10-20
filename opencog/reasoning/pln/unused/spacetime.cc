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

#include "spacetime.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <functional>

#include <CogServer.h>
#include <SimpleTruthValue.h>
#if USE_TIME_SERVER
//! @todo TimeServer has not yet been implemented for PseudoCore!!!
#include <TimeServer.h>
#endif
//#include "StaticNovamenteHandleArgumentType.h"
//#include "NovamenteIntegerArgumentType.h"

#include "PLN.h"
#include "PLNEvaluator.h"
#include "AtomSpaceWrapper.h"
#include "PLNatom.h"

#define INPUT_CHECK(msg)

#define MAX_TV_COUNT 1000

#define _abs(a) (((a)>0) ? (a) : -(a))

// this include was added by ricbit
#include <HandleTemporalPairEntry.h>

namespace haxx
{
using namespace opencog::pln;

extern map<Type, vector<Handle> > mindShadowMap;

Handle exec(vector<Handle>& hs);

Handle exec(const vector<BoundVertex>& hs)
{
vector<Handle> exec_args(hs.size());
transform(	hs.begin(), hs.end(), exec_args.begin(),
            Concat<DropVertexBindings, GetHandle,const weak_atom<Vertex>,Handle>());

return exec(exec_args);
}

Handle exec(Handle* hs, const int N)
{
vector<Handle> vhs;
for (int i = 0; i < N; i++)
    vhs.push_back(hs[i]);

return exec(vhs);
}


Handle exec(vector<Handle>& hs)
{
try
{
#if 1
if (hs.size() < 2)
{
    cprintf(2, "Execution with too few parameters!");
    return NULL;
}
#endif

AtomSpaceWrapper *as = GET_ASW;
nocase_string procedure_name = as->getName(hs[0]);

//	puts(("Launching " + procedure_name).c_str());

// hs[1] should be list link
vector<Handle> args = as->getOutgoing(hs[1]);
const int N = as->getArity(hs[1]);
//	printf("%d arguments\n", N);

if (procedure_name == "+" || procedure_name == "plus")
{
    exec_assert(N == 1 || N == 2);
    if (N == 1)
        return AddAsNumberNode(BinaryOpSchema<bin_op_plus, int>()(args[0], args[1]));
    else
        return AddAsNumberNode(UnaryOpSchema<un_op_plus, int>()(args[0]));
}

if (procedure_name == "-" || procedure_name == "minus")
{
    exec_assert(N == 1 || N == 2);
    if (N == 1)
        return AddAsNumberNode(BinaryOpSchema<bin_op_minus, int>()(args[0], args[1]));
    else
        return AddAsNumberNode(UnaryOpSchema<un_op_minus, int>()(args[0]));
}

if (procedure_name == "gt" || procedure_name == ">")
{
    exec_assert(N == 2);
    return AddAsNumberNode((int)BinaryOpSchema<op_gt,bool>()(args[0], args[1]));
}

if (procedure_name == "lt" || procedure_name == "<")
{
    exec_assert(N == 2);
    return AddAsNumberNode((int)BinaryOpSchema<op_lt,bool>()(args[0], args[1]));
}

if (procedure_name == "equal" || procedure_name == "=" || procedure_name == "==")
{
    exec_assert(N == 2);
    return AddAsNumberNode((int)BinaryOpSchema<op_equal,bool>()(args[0], args[1]));
}

if (procedure_name == "intervalMember")
{
    exec_assert(N == 2);
    return SpaceTimeApparatus::intervalMember(args[0], args[1]);
}

if (procedure_name == "getMomentNode")
{
    exec_assert(N == 1);
    return SpaceTimeApparatus::getMomentNode(args[0]);
}
if (procedure_name == "immediatelyAfter")
{
    exec_assert(N == 2);
    return SpaceTimeApparatus::immediatelyAfter(args[0], args[1]);
}
if (procedure_name == "increasing")
{
    exec_assert(N == 2);
    return SpaceTimeApparatus::increasing(args[0], args[1]);
}
if (procedure_name == "decreasing")
{
    exec_assert(N == 2);
    return SpaceTimeApparatus::decreasing(args[0], args[1]);
}
if (procedure_name == "intervalStart")
{
    exec_assert(N == 1);
    return SpaceTimeApparatus::intervalStart(args[0]);
}
if (procedure_name == "intervalEnd")
{
    exec_assert(N == 1);
    return SpaceTimeApparatus::intervalEnd(args[0]);
}
if (procedure_name == "intervalLength")
{
    exec_assert(N == 1);
    return SpaceTimeApparatus::intervalLength(args[0]);
}

#if USE_TIME_SERVER
if (procedure_name == "observedDistanceBetween")
{
    exec_assert(N == 3);
    return SpaceTimeApparatus::observedDistanceBetween(args[0], args[1], args[2]);
}
if (procedure_name == "observedDirectionBetween")
{
    exec_assert(N == 3);
    return SpaceTimeApparatus::observedDirectionBetween(args[0], args[1], args[2]);
}

/*	if (procedure_name == "observedVectorTo3D")
{
    exec_assert(N == 2);
    return SpaceTimeApparatus::observedVectorTo3D(args[0], args[1]);
}
if (procedure_name == "observedVectorTo1D")
{
    exec_assert(N == 2);
    return SpaceTimeApparatus::observedVectorTo1D(args[0], args[1]);
}*/
#endif
    else {
            // ???
    return NULL;
    }
} catch(...) {
    LOG(0, "Exception in schema exec()");
    #ifdef NMDEBUG
        getc(stdin);
    #endif

    return NULL;
}
}

} //namespace haxx

Handle AddAsNumberNode(int N)
{
char buff[128];
sprintf(buff, "%d", N);

//	return MindDBProxy::getInstance()->add(new Node(NUMBER_NODE, strdup(buff),
//		TruthValue::TRIVIAL_TV()));

//! @todo fresh bug
return GET_ASW->addNode(NUMBER_NODE, strdup(buff),
    TruthValue::TRIVIAL_TV());//,false,true);
}

timeUnit TimeApparatus::getMoment(Handle t)
{
string sname("getMoment");
cprintf(3, "%s\n", sname.c_str());
if (!t)
{ cprintf(2, ("NULL input at " + sname).c_str()); return 0; }

/*	HandleTemporalPairEntry* interval = TimeServer::getInstance()->getTimeLag(t);
return (interval
        ? (interval->handleTemporalPair->getTemporal()->getA())
        : NULL);
*/
// Alternatively: assumes that the name of the handle gives the timestamp.

if (inheritsType(GET_ASW->getType(t), NODE))
    return (timeUnit)atof(GET_ASW->getName(t).c_str());
    //return (timeUnit)atof(((Node*) TLB::getAtom(t))->getName().c_str());
else
    return 0;
}

Handle TimeApparatus::getMomentNode(Handle t)
{
    timeUnit m = getMoment(t);

    return (m ? AddAsNumberNode(m) : NULL);
}

Handle TimeApparatus::intervalStart(Handle interval)
{
string sname("intervalStart");
cprintf(3, "%s\n", sname.c_str());
if (!interval)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

//Assumes interval is in the form LinkType(Start, End)

//Atom* intervalAtom = TLB::getAtom(interval); 
vector<Handle> intervalOutgoingSet = GET_ASW->getOutgoing(interval);
int arity = GET_ASW->getArity(interval); 

exec_assert(arity == 2);

if (intervalOutgoingSet.empty())
    return NULL;

return intervalOutgoingSet[0];
}

Handle TimeApparatus::intervalEnd(Handle interval)
{
string sname("intervalEnd");
cprintf(3, "%s\n", sname.c_str());
if (!interval)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }
//Assumes interval is in the form LinkType(Start, End)


//        Atom* intervalAtom = TLB::getAtom(interval); 
vector<Handle> intervalOutgoingSet = GET_ASW->getOutgoing(interval);
int arity = GET_ASW->getArity(interval); 
exec_assert(arity == 2);

if (intervalOutgoingSet.empty())
    return NULL;
    
return intervalOutgoingSet[1];
}

Handle TimeApparatus::intervalLength(Handle interval)
{
string sname("intervalLength");
cprintf(3, "%s\n", sname.c_str());
if (!interval)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

timeUnit end = getMoment(intervalEnd(interval));
timeUnit start = getMoment(intervalStart(interval));
return AddAsNumberNode(end	- start);
}

Handle TimeApparatus::intervalMember(Handle t, Handle T)
{
string sname("intervalMember");
cprintf(3, "%s\n", sname.c_str());
if (!t || !T)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

timeUnit tMoment = getMoment(t);

if (!t)
    return NULL;

return AddCrispEvaluationLink(
    "intervalMember",
    t,
    T,
    (getMoment(intervalStart(T)) <= tMoment
     &&  tMoment <= getMoment(intervalEnd(T))));
}

Handle TimeApparatus::immediatelyAfter(Handle intervalB, Handle intervalA)
{
string sname("immediatelyAfter");
cprintf(3, "%s\n", sname.c_str());
if (!intervalA ||  !intervalB)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

timeUnit t0 = getMoment(intervalStart(intervalA));
timeUnit t1 = getMoment(intervalEnd(intervalA));
timeUnit t2 = getMoment(intervalStart(intervalB));
timeUnit t3 = getMoment(intervalEnd(intervalB));
    
//(t2,t3) is immediately after (t0,t1) if t0<=t2<=t1 and t3>t1

return AddCrispEvaluationLink(
    "immediatelyAfter",
    intervalA,
    intervalB,
    (t0 && t1 && t2 && t3 &&
     t0<=t2 && t2<=t1 && t3>t1));
}

bool executableToInt(Handle h)
{
string sname("executableToInt");
cprintf(3, "%s\n", sname.c_str());
if (!h)
{ cprintf(2, ("NULL input at " + sname).c_str()); return false; }

//    Atom* atom = TLB::getAtom(h);
return (GET_ASW->getType(h) == NUMBER_NODE);
}

int intFromNumberNode(Handle h)
{
string sname("intFromNumberNode");
cprintf(3, "%s\n", sname.c_str());
if (!h)
{ cprintf(2, ("NULL input at " + sname).c_str()); return 0; }

//       Node* numberNode = (Node*) TLB::getAtom(h);
return atoi(GET_ASW->inheritsType(GET_ASW->getType(h), NODE) ? GET_ASW->getName(h).c_str() : "0");
}

Handle TimeApparatus::timeDiff(Handle F, Handle interval,int sign)
{
//	if (!executableToInt(F))
//		return false;
string sname = "timeDiff";
cprintf(3, "%s\n", sname.c_str());
if (!interval  || !F)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

std::vector<Handle> args_start;
args_start.push_back(intervalStart(interval));
std::vector<Handle> args_end;
args_end.push_back(intervalEnd(interval));

if (!args_start[0] || !args_end[0])
    return NULL;

//! @todo fresh bug
Handle arglist1 = GET_ASW->addLink(LIST_LINK, args_start, TruthValue::NULL_TV()); //, false, true);
Handle arglist2 = GET_ASW->addLink(LIST_LINK, args_end, TruthValue::NULL_TV()); //, false, true);

Handle hs_start[] = { F, arglist1 };
Handle hs_end[] = { F, arglist2 };

int begin = intFromNumberNode(haxx::exec(hs_start,2));
int end = intFromNumberNode(haxx::exec(hs_end,2));

if (!begin || !end)
    return NULL;

return AddCrispEvaluationLink(
    ((sign>0) ? "increasing" : "decreasing"),
    F,
    interval,
    begin && end &&
        ((sign>0 && begin <= end )
        ||(sign<0 && begin >= end )));
}

Handle AddCrispEvaluationLink(	const char* predName,
                        Handle arg1,
                        Handle arg2,
                        bool tv)
{
if (!arg1 || !arg2)
    return NULL;

std::vector<Handle> argsList;
argsList.push_back(arg1); // (time->int)
argsList.push_back(arg2); // [time]

std::vector<Handle> argsEval;
//! @todo fresh bug
argsEval.push_back(GET_ASW->addNode(PREDICATE_NODE, predName, TruthValue::NULL_TV())); //, false,true));
argsEval.push_back(GET_ASW->addLink(LIST_LINK, argsList, TruthValue::NULL_TV())); //, false,true));

return GET_ASW->addLink(EVALUATION_LINK, argsEval,
        SimpleTruthValue((float)tv, MAX_TV_COUNT)); //, false,true);
}

Handle TimeApparatus::increasing(Handle F, Handle interval)
{
string sname = "increasing";
cprintf(3, "%s\n", sname.c_str());
if (!interval || !F)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

return timeDiff(F, interval, 1);
}

Handle TimeApparatus::decreasing(Handle F, Handle interval)
{
string sname = "decreasing";
cprintf(3, "%s\n", "%s\n", sname.c_str());
if (!interval || !F)
{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }

return timeDiff(F, interval, -1);
}

//! @todo TimeServer has not yet been implemented for PseudoCore!!!
#if USE_TIME_SERVER

struct moment_equal : public binary_function<reasoning::BoundVertex, timeUnit, bool>
{
	bool operator()(reasoning::BoundVertex h, timeUnit t) const
	{
        std::list<HandleTemporalPair> intervals;
        GET_ASW->getTimeServer().get(back_inserter(intervals),
                boost::get<Handle>(h.value));
		return (intervals.size() > 0
				? (intervals.front().getTemporal()->getA() == t)
				: false);
	}
};

float pos(Handle pixel, Handle t)
{
	string sname = "pos";
	cprintf(3, "%s\n", "%s\n", sname.c_str());
	if (!pixel || !t)
	{ cprintf(2, ("NULL input at " + sname).c_str()); return NULL; }
	
	/// Temporary hack until sufficient lookup & support code is written
	/// without using atom
	
	reasoning::atom* args = new reasoning::atom(LIST_LINK, 2,
		pixel,
		new reasoning::atom(FW_VARIABLE_NODE, 0, "$1")); //Should unify to a NumberNode.
	
	reasoning::atom eval(EVALUATION_LINK, 2,
		new reasoning::atom(PREDICATE_NODE, 0, "CW_position"),
		args);
	
	tree<Vertex> queryt(eval.makeHandletree(ASW()));

	//! @todo Take advantage of the fact that only 1 result needed.
	
	reasoning::TableGather all_res(queryt,ASW(),-1);
	reasoning::TableGather::iterator res =
			find_if(all_res.begin(), all_res.end(),
				bind2nd(moment_equal(), intFromNumberNode(t)));
	if (res != all_res.end())
		return (float)intFromNumberNode(boost::get<Handle>(res->value));
    
    return NULL;
}

Handle SpaceTimeApparatus::observedDistanceBetween(Handle a, Handle b, Handle t)
{
	string sname = "observedDistanceBetween";
	cprintf(3, "%s\n", "%s\n", sname.c_str());
	if (!a || !b || !t)
	{ cprintf(2, ("NULL input at " + sname).c_str()); return AddAsNumberNode(0.0f); }
	
	return AddAsNumberNode(pos(b,t) - pos(a,t));
}

Handle SpaceTimeApparatus::observedDirectionBetween(Handle a, Handle b, Handle t)
{
	string sname = "observedDirectionBetween";
	cprintf(3, "%s\n", "%s\n", sname.c_str());
	if (!a || !b || !t)
	{ cprintf(2, ("NULL input at " + sname).c_str()); return 0; }
	
	int pa = pos(a,t);	
	int pb = pos(b,t);
	
	if (pa < pb)
		return AddAsNumberNode(-1);
	else if (pb > pa)
		return AddAsNumberNode(1);
	else return AddAsNumberNode(0);
}

/*Handle SpaceTimeApparatus::observedDistanceTo(Handle x, Handle t)
{
	AddAsNumberNode();
}*/

#endif

//#define TIME_INTERVAL_NODE	1

// WELTER @todo StaticNovamenteHandleArgumentType is not defined in anywhere!!! SO ALL THE CODE BELLOW IS COMMENTED OUT 
//typedef StaticNovamenteHandleArgumentType<NUMBER_NODE,0> NumberNodeType;
//typedef StaticNovamenteHandleArgumentType<TIME_INTERVAL_NODE,0> IntervalNodeType;
//
//class IntervalMemberSchema
//	: public SchemaOutput<
//		SchemaInput2<NumberNodeType, IntervalNodeType>,
//		NumberNodeType>
//{
//public:
//	
//	/**
//	 * Constructor.
//	 */
//	IntervalMemberSchema() {}
//
//	/**
//	 * Destructor.
//	 */
//	virtual ~IntervalMemberSchema(){};
//
//	// BuiltInSchema interface
//	virtual void execute(Task *t, ProcedureArgument *args) const
//	{
//		int N = ((ProcedureArgumentList*)ProcedureArgument)->getNArguments();
//		assert(N == 2);
//
//		Handle hargs[2];
//		((ProcedureArgumentList*)ProcedureArgument)->getHandleArguments(hargs);
//	
//		ProcedureArgument *ret = new NovamenteTruthValueArgument(operate(tvs, ((ProcedureArgumentList *)args)->getNArguments()));
//	
//		HandleSet *trail = new HandleSet();
//		
//		for (int i = 0; i < ((ProcedureArgumentList *)args)->getNArguments(); i++){
//			delete(tvs[i]);
//			ProcedureArgument *arg = ((ProcedureArgumentList *)args)->getArgument(i);
//			if (typeid(*arg) == typeid(NovamenteHandleArgument)){
//				trail->add(((NovamenteHandleArgument *)arg)->getHandle());
//			}
//		}
//
///*		EvaluationTaskResult *result = new EvaluationTaskResult();
//		
//		result->addResult(ret, trail);
//		Lobe::getInstance()->getTaskBroker()->taskFinished(t, result);*/
//	}
//
//	//Procedure interface
//	std::string toString()
//	{
//		return "";
//	}
//};
