#ifndef RULEPROVIDER_H_
#define RULEPROVIDER_H_

#include "Rule.h"
//#pragma once

namespace reasoning
{
/**
	Takes ownership of the Rule objects given to it
*/

class RuleProvider : public vector<Rule*> {
protected:
	void AddRule(Rule* r, float priority);
public:
	RuleProvider(void);
public:
	virtual ~RuleProvider(void);
};

class VariableRuleProvider : public RuleProvider {
public:
	VariableRuleProvider(void);
	virtual ~VariableRuleProvider(void);

//	void CreateCustomCrispUnificationRules(); //Re-create these rules every time a new axiom set is loaded in!
//	const set<uint>& GetCustomCrispUnificationRules() const { return CustomCrispUnificationRules; }
};

class DefaultVariableRuleProvider : public VariableRuleProvider
{
public:
	DefaultVariableRuleProvider(void);
	virtual ~DefaultVariableRuleProvider(void);
};

class ForwardTestRuleProvider : public VariableRuleProvider
{
public:
	ForwardTestRuleProvider(void);
	virtual ~ForwardTestRuleProvider(void);
};

} //namespace reasoning
#endif
