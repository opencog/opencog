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

#ifndef _PLNEVALUATOR_H_
#define _PLNEVALUATOR_H_

#include <queue>
#include <set>
#include <opencog/util/tree.h>
#include "rules/Rule.h"
#include "PLN.h"

#define LOCAL_ATW 0
enum MetaProperty { NONE, STRENGTH, CONFIDENCE, STRENGTH_CONFIDENCE, LTI, STI };

namespace opencog {
namespace pln {
typedef opencog::pln::BoundVertex atomT;
typedef std::set<atomT> atomSetT;
typedef std::vector<atomT> atomVectorT;
	
class Rule;
class InferenceNode2;
class BackwardInferenceTask;
class InferenceNode;
struct iAtomSpaceWrapper;

namespace simple_evaluator
{
struct RuleProvider
{
	virtual const std::vector<RULE>& get()=0;
    virtual ~RuleProvider() {};
};

class DefaultRuleProvider : public RuleProvider
{
protected:	
	std::vector<RULE> rs;
public:
	DefaultRuleProvider();
	virtual ~DefaultRuleProvider();
	const std::vector<RULE>& get();
};


/** Stores inference tree for Breadth-1st inference and last-attempted-depth 
 * for Depth-1st.
 */
typedef boost::variant<InferenceNode2, int> InferenceSession;

class SimplePLNEvaluator
{
	static bool exists(Handle top, Handle* args, const int N, Handle& ret);
	static bool exists(Handle top, const VertexVector& args, Handle& ret);
	static bool exists(Handle top, const std::vector<BoundVertex>& args, Handle& ret);
	static bool exists(Handle top, const std::vector<Handle>& args, Handle& ret);
	static bool supportedOperator(Handle h);
	
	static Handle unify_all(const std::set<BoundVertex>& v);
	
	static Btr<BV_Set> w_evaluate(const tree<Vertex>& target,
											tree<Vertex>::iterator top,
											MetaProperty policy);
	
	friend class PLNEvaluator;
};

/* Exported General PLN services */

struct InferenceTaskParameters
{
	InferenceTaskParameters(
					RuleProvider* _ruleProvider,
					Btr<tree<Vertex> > _target);
	
	RuleProvider* ruleProvider;
	Btr<vtree> target;
};

class PLNEvaluator
{
  public:
	  
  	/** Calls the simple Boolean bottom-up PLN bw chainer */
	static Vertex v_evaluate(	const tree<Vertex>& target,
								tree<Vertex>::iterator top,
								MetaProperty policy);
};

} //namespace simple_evaluator

}} //namespace opencog::pln

namespace haxx
{
	using namespace opencog::pln;
Handle exec(Handle* hs, const int N);
Handle exec(std::vector<Handle>& hs);
Handle exec(const std::vector<BoundVertex>& hs);
}


#endif //_PLNEVALUATOR_H_
