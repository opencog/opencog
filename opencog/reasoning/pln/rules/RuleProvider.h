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

#ifndef RULEPROVIDER_H_
#define RULEPROVIDER_H_

#include "Rule.h"

#include <boost/thread/mutex.hpp>

namespace opencog {

class AtomSpaceImpl;
    
namespace pln {

class RuleProvider  {
protected:
    //! Watching for instantiation atoms
    bool watchingForInstantiationAtoms;

    //! Guard is used when add/remove signals are sent from AtomSpace via the
    //! ReferenceRuleProvider.
    mutable boost::mutex guard;
public:
    /** This map is used to record both the rules in this
     * provider and the priorities.
     */
    std::map<const std::string, float> rulePriorities; 

    RuleProvider(void);
    virtual ~RuleProvider(void);

    //! Add rule given by rule name with priority
    void addRule(const std::string& ruleName, float priority);

    //! Remove rule with given name
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

    //! Get a list of names for rules in this provider.
    std::vector<std::string> getRuleNames() const;

    //! Does this rule provider contain any rules?
    bool empty() const;

    /** Tell the reference rule provider that it should update this rule
     * provider when atoms matching FORALL_LINK or AVERAGE_LINK are
     * added/removed.
     */
    void watchInstantiationAtoms();

    void printPriorities();
};

class ReferenceRuleProvider : public RuleProvider
{
    // For monitoring additions to the AtomSpace from outside of PLN
    bool handleAddSignal(AtomSpaceImpl *a, Handle h); //!< Signal handler for atom adds.
    bool handleRemoveSignal(AtomSpaceImpl *a, Handle h); //!< Signal handler for atom removals.

    boost::signals2::connection c_add; //! Connection to add atom signals
    boost::signals2::connection c_remove; //! Connection to remove atom signals

    std::map<std::string,RulePtr> rules; //! name to rule mapping

    //! notify these rule providers when instantiation atoms are added/removed.
    std::vector<RuleProvider*> watchers;
    //! lock for watchers
    mutable boost::mutex watchersLock;

    std::vector<RulePtr> instantiationRules;

    static const float instantiationRulePriority;

    /**
     * Takes ownership of the Rule objects given to it
     */
    RulePtr addRule(Rule* r, float priority);
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

    void registerWatcher(RuleProvider* rp);
    void unregisterWatcher(RuleProvider* rp);

    void printRules();
};


/**
 * A "reference" RuleProvider containing the official versions of all Rules.
 * Particular FC or BC processes are allowed to use their own RuleProviders,
 * which may have different combinations of Rules, but those RuleProviders must
 * contain pointers to the same Rule objects as contained here.
 */
ReferenceRuleProvider& referenceRuleProvider();


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
