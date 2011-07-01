/*
 * opencog/comboreduct/combo/action.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Carlos Eduardo Rodrigues Lopes
 *            Nil Geisweiller
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
#ifndef VARIABLE_UNIFIER_H_
#define VARIABLE_UNIFIER_H_

#include <map>
#include <string>
#include <vector>

namespace opencog { namespace combo {

enum UnifierOperation {
    UNIFY_NOT,
    UNIFY_AND,
    UNIFY_OR

};

// Map used to hold the variables and its states (true or false)
typedef std::map<std::string, bool> UnifierMap;

// Map iterator
typedef std::map<std::string, bool>::iterator UnifierIt;

// Map const_itertator
typedef std::map<std::string, bool>::const_iterator UnifierConstIt;

typedef std::vector<std::string> activeSet_t;

/**
 * variable_unifier is used to hold the variable condidates in a unifier
 * process. It also executes this process, based on three logical operations
 * NOT, AND, OR.
 * The *updated* field informs if the default values have been changed and
 * oneVariableActive informs if there is at least one candidates variable whose
 * state is set true.
 */
 class variable_unifier : public UnifierMap
{

public:

    //Nil : added that temporary function to check is there exists
    //at least one active variable (should be removed once
    //isOneVariableActive works properly
    inline bool isOneVariableActiveTMP() {
        for(UnifierConstIt it = begin(); it != end(); ++it) {
            if(it->second) return true;
        }
        return false;
    }

    /**
     * Special variable_unifier
     */
    static variable_unifier& DEFAULT_VU();

    /**
     * Constructors
     */
    variable_unifier();
    variable_unifier(const variable_unifier& unifier);

    /**
     * Destructor
     */
    virtual ~variable_unifier();

    /**
     * Unifiy this variable_unifier object with the parameter unifier informed
     *
     * @param operation the unifier operation
     * @param unifier The variable_unifier that should be unified with this object.
     */
    void unify(combo::UnifierOperation operation,
               const variable_unifier& unifier);

    /**
     * Get the variable state from UnifierMap.
     *
     * @param variable The variable name
     * @return The variable state for the given variable. If variable
     * not present in UnifierMap, false is returned.
     */
    bool getVariableState(std::string& variable);

    /**
     * Set the variable statee in the UnifiedMap
     *
     * @param variable The variable name.
     * @param state The variable state.
     */
    void setVariableState(std::string& variable, bool state);

    /**
     * Insert a variable into unifier map.
     * If the variable already exists it does nothing
     *
     * @param variable The variable's name
     * @param state The variable's state (true default)
     */
    void insert(const std::string& variable, const bool state = true);

    /**
     * Inform if a given variable is presente in the UnifierMap
     *
     * @param variable The variable name.
     * @return True if the variable is present in the UnifierMap,
     * false otherwise.
     */
    bool contains(const std::string& variable) const;

    /**
     * Inform if there is at least one variable in UnifierMap that has its
     * state set to true.
     */
    inline bool isOneVariableActive() const {
        return this->oneVariableActive;
    }

    /**
     * Set oneVariableActive field
     */
    inline void setOneVariableActive(bool state) {
        this->oneVariableActive = state;
    }

    /**
     * return the satifying set
     */
    inline activeSet_t getActiveSet() const {
        activeSet_t res;
        for(UnifierConstIt it = begin(); it != end(); ++it) {
            if(it->second) res.push_back(it->first);
        }
        return res;
    }


    /**
     * Inform if the VariableUnifier was updated in last evaluation
     */
    inline bool isUpdated() const {
        return this->updated;
    }

    /**
     * Set updated state
     */
    inline void setUpdated(bool state) {
        this->updated = state;
    }

    std::string toString() const;

private:

    /**
     * Inform if there is at least one variable in UnifierMap that has its
     * state set to true.
     */
    bool oneVariableActive;

    /**
     * Inform if the unifier has been updated between unifications. If not
     * updated, no unifications needs to take place.
     */
    bool updated;

}; // class

}} // ~namespaces combo opencog

#endif /*VARIABLE_UNIFIER_H_*/
