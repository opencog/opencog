/*
 * PatternMinerSCM.cc
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Shujing Ke
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


// The header file bits.

#ifndef _OPENCOG_PATTERNMINER_SCM_H
#define _OPENCOG_PATTERNMINER_SCM_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/learning/PatternMiner/PatternMiner.h>

using namespace std;
using namespace opencog::PatternMining;

namespace opencog
{

class PatternMinerSCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    PatternMiner* patternMiner;

public:
    PatternMinerSCM();
    void run_patternminer();

    string get_current_settings();

    string get_Pattern_Max_Gram(){return "max_gram: " + patternMiner->get_Pattern_Max_Gram();}
    string set_Pattern_Max_Gram(unsigned int _max_gram)
    {
        patternMiner->set_Pattern_Max_Gram(_max_gram);
        return get_Pattern_Max_Gram();
    }

    string get_Enable_Interesting_Pattern(){return "enable_Interesting_Pattern: " + patternMiner->get_Enable_Interesting_Pattern();}
    string set_Enable_Interesting_Pattern(bool _enable)
    {
        patternMiner->set_Enable_Interesting_Pattern(_enable) ;
        return get_Enable_Interesting_Pattern();
    }

    string get_Frequency_threshold() {return "Frequency_threshold: " + patternMiner->get_Frequency_threshold();}
    string set_Frequency_threshold(unsigned int _Frequency_threshold)
    {
        patternMiner->set_Frequency_threshold(_Frequency_threshold);
        return get_Frequency_threshold();
    }


    string get_use_keyword_black_list(){return "use_keyword_black_list: " + patternMiner->get_use_keyword_black_list();}
    string set_use_keyword_black_list(bool _use)
    {
        patternMiner->set_use_keyword_black_list(_use);
        return get_use_keyword_black_list();
    }

    string get_use_keyword_white_list(){return "use_keyword_white_list: " + patternMiner->get_use_keyword_white_list();}
    string set_use_keyword_white_list(bool _use)
    {
        patternMiner->set_use_keyword_white_list(_use);
        return get_use_keyword_white_list();
    }

    string get_Ignore_Link_Types()
    {
        string result =  "Ignore_Link_Types:";
        vector<Type> ignore_list = patternMiner->get_Ignore_Link_Types();
        for (Type type : ignore_list)
            result +=  " " + classserver().getTypeName(type);

        return result;
    }

    string add_Ignore_Link_Type(string _typeStr)
    {
        Type atomType = classserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Input type is wrong!";

        string result = "";

        if (patternMiner->add_Ignore_Link_Type(atomType))
            result += "Added!\n";
        else
            result += "Input type already exists in the ingnore Link list!\n";

        return result + get_Ignore_Link_Types();

    }

    string remove_Ignore_Link_Type(string _typeStr)
    {
        Type atomType = classserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Input type is wrong!";

        string result = "";
        if (patternMiner->remove_Ignore_Link_Type(atomType))
            result += "Removed!\n";
        else
            result += "Input type does not exist in the ingnore Link list!\n";

        return result + get_Ignore_Link_Types();

    }

    string get_keyword_black_list()
    {
        string result =  "keyword_black_list:";
        vector<string> black_list = patternMiner->get_keyword_black_list();
        for (string word : black_list)
            result +=  " " + word;

        return result;
    }

    string add_keyword_to_black_list(string _keyword)
    {
        string result = "";
        if (patternMiner->add_keyword_to_black_list(_keyword))
            result += "Added!\n";
        else
            result +=  "Input keyword already exists in the black list!\n";
        return result + get_keyword_black_list();
    }

    string remove_keyword_from_black_list(string _keyword)
    {
        string result = "";
        if (patternMiner->remove_keyword_from_black_list(_keyword))
            result += "Removed!\n";
        else
            result +=  "Input keyword does not exist in the black list!\n";
        return result + get_keyword_black_list();
    }


    string get_keyword_white_list()
    {
        string result =  "keyword_white_list:";
        vector<string> white_list = patternMiner->get_keyword_white_list();
        for (string word : white_list)
            result +=  " " + word;

        return result;
    }

    string add_keyword_to_white_list(string _keyword)
    {
        string result = "";
        if (patternMiner->add_keyword_to_white_list(_keyword))
            result += "Added!\n";
        else
            result +=  "Input keyword already exists in the white list!\n";
        return result + get_keyword_white_list();
    }

    string remove_keyword_from_white_list(string _keyword)
    {
        string result = "";
        if (patternMiner->remove_keyword_from_white_list(_keyword))
            result += "Removed!\n";
        else
            result +=  "Input keyword does not exist in the white list!\n";
        return result + get_keyword_white_list();
    }
};


extern "C" {
void opencog_patternminer_init(void);
};

}
#endif // _OPENCOG_PATTERNMINER_SCM_H

// --------------------------------------------------------------

#include <opencog/util/Config.h>
#include <opencog/guile/SchemePrimitive.h>

/**
 * Implement a dynamically-loadable Pattern Miner guile module.
 */

using namespace opencog;


PatternMinerSCM::PatternMinerSCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

void* PatternMinerSCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog patternminer", init_in_module, self);
    scm_c_use_module("opencog patternminer");
    return NULL;
}

void PatternMinerSCM::init_in_module(void* data)
{
    PatternMinerSCM* self = (PatternMinerSCM*) data;
    self->init();
}

/**
 * The main init function for the PatternMinerSCM object.
 */
void PatternMinerSCM::init()
{
    define_scheme_primitive("pm-run-patternminer", &PatternMinerSCM::run_patternminer, this, "patternminer");
    define_scheme_primitive("pm-get-current-settings", &PatternMinerSCM::get_current_settings, this, "patternminer");

}

extern "C" {
void opencog_patternminer_init(void)
{
    static PatternMinerSCM patternminerscm;
}
};

// --------------------------------------------------------------

void PatternMinerSCM::run_patternminer()
{

    AtomSpace* as = SchemeSmob::ss_get_env_as("run_patternminer");
    patternMiner = new PatternMiner(as);
    patternMiner->runPatternMiner(1);

}

string PatternMinerSCM::get_current_settings()
{

}
