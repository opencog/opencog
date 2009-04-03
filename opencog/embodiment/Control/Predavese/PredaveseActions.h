/*
 * opencog/embodiment/Control/Predavese/PredaveseActions.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#ifndef PREDAVESE_ACTIONS_H
#define PREDAVESE_ACTIONS_H

//#include "PredaveseStdafx.h"
#include "Predavese.h"
#include "util/exceptions.h"
#include "PetInterface.h"
#include "SystemParameters.h"
#include "NominalReferenceResolver.h"

/// This file contains all the action objects that are associated with
/// specific expressions in the parser

namespace predavese
{

class action
{
protected:
    NominalReferenceResolver *nameResolver;
    Control::PetInterface& petInterface;
    Control::SystemParameters parameters; // TODO: Get the parameters dynamically (by passing its reference as constructor argument, for instance)

    void boostPayAttentionSchema() const;
public:
    action(Control::PetInterface& _petInterface) : petInterface(_petInterface) {
        nameResolver = new NominalReferenceResolver(_petInterface);
    }

    virtual ~action() {
        delete nameResolver;
    }
    virtual bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const = 0;

};

/// start_index = How many words we omit from the beginning.
/// Eg. "start learning to" is ommitted, because that's covered

template<typename OutIterT>
void FormatArgs(vector<c>& i_args, int start_index, OutIterT next_arg, int end_index = -1)
{
    for (int i = start_index; i < (int)i_args.size() && (end_index == -1 || i <= end_index); i++) {
        c& _c = i_args[i];
//  pAtom* a = get<pAtom>(&_c);

        _c = promote()(_c);
        string* s = get<string>(&_c);
        //printf("String %s\n", s->c_str());

#ifdef USE_ASSERT_OR_EXIT
        cassert(TRACE_INFO, s, "predavese::FormatArgs: Got no string from the argument %d\n", i);
#else
        if (!s) {
            throw opencog::RuntimeException(TRACE_INFO, "predavese::FormatArgs: Got no string from the argument %d\n", i);
        }
#endif
        //if(*s != "DO" && *s != "TRY"){
        *(next_arg++) = *s;
        //}
    }
}

class reward_action : public action
{
public:
    reward_action(Control::PetInterface& petInterface);
    virtual ~reward_action() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class punish_action : public action
{
public:
    punish_action(Control::PetInterface& petInterface);
    virtual ~punish_action() {}
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class predicate_action : public action
{
private:
    int arity;
public:
    predicate_action(Control::PetInterface& petInterface, int _arity);
    virtual ~predicate_action() {}
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_command : public action
{
private:
    int arity;
public:
    virtual ~pet_command() {};
    pet_command(Control::PetInterface& petInterface, int _arity);
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_dev_meta_command : public action
{
private:
    int arity;
public:
    virtual ~pet_dev_meta_command() {};
    pet_dev_meta_command(Control::PetInterface& petInterface, int _arity);
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_learn_command : public action
{
private:
    int arity;
public:
    virtual ~pet_learn_command() {};
    pet_learn_command(Control::PetInterface& petInterface, int _arity);
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_stop_learn_command : public action
{
private:
    int arity;
public:
    pet_stop_learn_command(Control::PetInterface& petInterface, int _arity);
    virtual ~pet_stop_learn_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_stop_command : public action
{
private:
    int arity;
public:
    pet_stop_command(Control::PetInterface& petInterface, int _arity);
    virtual ~pet_stop_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class exemplar_start_command : public action
{
private:
    int arity;
public:
    exemplar_start_command(Control::PetInterface& petInterface, int _arity);
    virtual ~exemplar_start_command() {};
    bool operator()(const pAtom& arg, unsigned long timestampavatarId, const std::string& agentIdWhichSentTheCommand) const;
};

class exemplar_end_command : public action
{
private:
    int arity;
public:
    exemplar_end_command(Control::PetInterface& petInterface, int _arity);
    virtual ~exemplar_end_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class try_schema_command : public action
{
private:
    int arity;
public:
    try_schema_command(Control::PetInterface& petInterface, int _arity);
    virtual ~try_schema_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_meta_command : public action
{
private:
    int arity;
public:
    pet_meta_command(Control::PetInterface& petInterface, int _arity);
    virtual ~pet_meta_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_interrogative_command : public action
{
private:
    int arity;
public:
    pet_interrogative_command(Control::PetInterface& petInterface, int _arity);
    virtual ~pet_interrogative_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class pet_declarative_command : public action
{
private:
    int arity;
public:
    pet_declarative_command(Control::PetInterface& petInterface, int _arity);
    virtual ~pet_declarative_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

class any_command : public action
{
public:
    any_command(Control::PetInterface& petInterface);
    virtual ~any_command() {};
    bool operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const;
};

typedef any_command Relation_obj_action;
typedef any_command Relation_subj_action;
typedef any_command Relation_obj2_action;
typedef any_command Relation_poss_action;
typedef any_command Relation_subj_r_action;
typedef any_command Relation_to_do_action;

} //namespace predavese

#endif
