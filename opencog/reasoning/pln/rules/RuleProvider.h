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

class ForwardChainerRuleProvider : public VariableRuleProvider
{
private:
    //! The seed handle
    Handle seed;
    unsigned int seedIndex;

    //! Current rule that's been checked out last by getRule
    Rule* current;

    Rule* findHighestPriorityRule();
public:
	ForwardChainerRuleProvider(void);
	virtual ~ForwardChainerRuleProvider(void);

    //! Rules that have already be attempted.
    std::vector<Rule*> invalidRules;

    //! Set the seed atom, when this changes the seedStack and invalidRules need to
    //! be reset.
    void setSeed(Handle s);
    
    //! Get the index for the seed in the last Rule provided by nextRule;
    unsigned int getSeedIndex();

    //! Reset stacks
    void reset();

    //! Retrieve the next rule that can have the seed atom bound.
    //! Note: the seed atom may be bindable to more than one spot in the
    //! returned rule.
    Rule* nextRule();
    
};

} //namespace reasoning
#endif
