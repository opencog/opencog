/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef RULEPROVIDER_H_
#define RULEPROVIDER_H_

#include "Rule.h"
//#pragma once

namespace opencog { namespace pln {

/**
 * Takes ownership of the Rule objects given to it
 */
class RuleProvider : public std::vector<Rule*> {
protected:
    void AddRule(Rule* r, float priority);
public:
    RuleProvider(void);
    virtual ~RuleProvider(void);

    /**
     * @param ruleName the name of the rule we are looking for
     *
     * @return a pointer to the rule with name ruleName.
     *         If no such rule exists then it return NULL
     */
    const Rule* findRule(const std::string& ruleName) const;
};

class VariableRuleProvider : public RuleProvider {
public:
    VariableRuleProvider(void);
    virtual ~VariableRuleProvider(void);

//  void CreateCustomCrispUnificationRules(); //Re-create these rules every time a new axiom set is loaded in!
//  const set<uint>& GetCustomCrispUnificationRules() const { return CustomCrispUnificationRules; }
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
    pHandle seed;
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
    void setSeed(pHandle s);
    
    //! Get the index for the seed in the last Rule provided by nextRule;
    unsigned int getSeedIndex();

    //! Reset stacks
    void reset();

    //! Retrieve the next rule that can have the seed atom bound.
    //! Note: the seed atom may be bindable to more than one spot in the
    //! returned rule.
    Rule* nextRule();
    
};

}} //namespace opencog { namespace pln {
#endif
