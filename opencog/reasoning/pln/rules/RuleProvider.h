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

//#include <boost/mutex.hpp>

namespace opencog { namespace pln {

class RuleProvider  {
protected:
public:
    // This map is used to record both the rules in this
    // provider and the priorities
    std::map<const std::string, float> rulePriorities; 

    RuleProvider(void);
    virtual ~RuleProvider(void);

    void addRule(const std::string& ruleName, float priority);
    virtual void removeRule(const std::string& name);

    /** Set the Rule's priority that is used by inference heuristics.
     *
     * @param r The Rule
     * @param priority The Rule's priority.
     */
    void setPriority(const std::string& ruleName, float priority);

    /** Get the priority of the rule for use by inference heuristics.
     *
     * @param r The Rule
     * @return The Rule's priority.
     */
    float getPriority(const std::string& ruleName);

    /**
     * @param ruleName the name of the rule we are looking for
     *
     * @return a smart pointer to the rule with name ruleName.
     *         If no such rule exists then it return NULL
     */
    virtual RulePtr findRule(const std::string& ruleName) const;

    std::vector<std::string> getRuleNames() const;

    bool empty() const {return rulePriorities.size() == 0 ? false : true; }
};

class ReferenceRuleProvider : public RuleProvider
{
    // For monitoring additions to the AtomSpace from outside of PLN
    bool handleAddSignal(Handle h); //!< Signal handler for atom adds.
    bool handleRemoveSignal(Handle h); //!< Signal handler for atom removals.

    boost::signals::connection c_add; //! Connection to add atom signals
    boost::signals::connection c_remove; //! Connection to remove atom signals

    std::map<std::string,RulePtr> rules; //! name to rule mapping

    /**
     * Takes ownership of the Rule objects given to it
     */
    void addRule(Rule* r, float priority);
public:
    /**
     * @param ruleName the name of the rule we are looking for
     *
     * @return a smart pointer to the rule with name ruleName.
     *         If no such rule exists then it return NULL
     */
    virtual RulePtr findRule(const std::string& ruleName) const;

    virtual void removeRule(const std::string& name);

    ReferenceRuleProvider(void);
    virtual ~ReferenceRuleProvider(void);
};

/**
 * A "reference" RuleProvider containing the official versions of all Rules.
 * Particular FC or BC processes are allowed to use their own RuleProviders,
 * which may have different combinations of Rules, but those RuleProviders must
 * contain pointers to the same Rule objects as contained here.
 */
RuleProvider& referenceRuleProvider();


class VariableRuleProvider : public RuleProvider {
public:
    VariableRuleProvider(void);
    virtual ~VariableRuleProvider(void);

};

class ForwardComposerRuleProvider : public VariableRuleProvider
{
public:
    ForwardComposerRuleProvider(void);
    virtual ~ForwardComposerRuleProvider(void);
};

class ForwardGeneratorRuleProvider : public VariableRuleProvider
{
public:
    ForwardGeneratorRuleProvider(void);
    virtual ~ForwardGeneratorRuleProvider(void);
};

class DeductionRuleProvider : public VariableRuleProvider
{
public:
    DeductionRuleProvider(void);
    virtual ~DeductionRuleProvider(void);
};

class EvaluationRuleProvider : public VariableRuleProvider
{
public:
    EvaluationRuleProvider(void);
    virtual ~EvaluationRuleProvider(void);
};

}} //namespace opencog { namespace pln {
#endif
