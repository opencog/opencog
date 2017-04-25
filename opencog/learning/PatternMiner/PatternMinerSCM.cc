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

#include <boost/algorithm/string.hpp>

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

    void run_patternminer()
    {
        patternMiner->runPatternMiner(false);
    }

    string get_Pattern_Max_Gram()
    {
        return  "max_gram: " + std::to_string( (int)(patternMiner->get_Pattern_Max_Gram()));
    }
    string set_Pattern_Max_Gram(int _max_gram)
    {
        patternMiner->set_Pattern_Max_Gram(_max_gram);
        return get_Pattern_Max_Gram();
    }

    string get_Enable_Interesting_Pattern()
    {
        bool enable = patternMiner->get_Enable_Interesting_Pattern();

        if (enable)
            return "enable_Interesting_Pattern: true";
        else
            return "enable_Interesting_Pattern: false";
    }

    string set_Enable_Interesting_Pattern(bool _enable)
    {
        patternMiner->set_Enable_Interesting_Pattern(_enable) ;
        return get_Enable_Interesting_Pattern();
    }

    string get_Frequency_threshold() {return "Frequency_threshold: " + std::to_string(patternMiner->get_Frequency_threshold());}
    string set_Frequency_threshold(int _Frequency_threshold)
    {
        patternMiner->set_Frequency_threshold(_Frequency_threshold);
        return get_Frequency_threshold();
    }


    string get_use_keyword_black_list()
    {
        bool enable = patternMiner->get_use_keyword_black_list();

        if (enable)
            return "use_keyword_black_list: true";
        else
            return "use_keyword_black_list: false";
    }

    string set_use_keyword_black_list(bool _use)
    {
        patternMiner->set_use_keyword_black_list(_use);
        return get_use_keyword_black_list();
    }

    string get_use_keyword_white_list()
    {
        bool enable = patternMiner->get_use_keyword_white_list();

        if (enable)
            return "use_keyword_white_list: true";
        else
            return "use_keyword_white_list: false";
    }

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

    string add_Ignore_Link_Type(const string& _typeStr)
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

    string remove_Ignore_Link_Type(const string& _typeStr)
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

    string add_keyword_to_black_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->add_keyword_to_black_list(_keyword))
            result += "Added!\n";
        else
            result +=  "Input keyword already exists in the black list!\n";
        return result + get_keyword_black_list();
    }

    string add_keywords_to_black_list(const string& _keywordlist)
    {
        vector<string> keyword_list;
        string keywordstr = _keywordlist;
        keywordstr .erase(std::remove(keywordstr .begin(), keywordstr .end(), ' '), keywordstr .end());
        boost::split(keyword_list, keywordstr , boost::is_any_of(","));

        for (string keyword : keyword_list)
            add_keyword_to_black_list(keyword);

        return "Added!" + get_keyword_black_list();
    }

    string remove_keyword_from_black_list(const string& _keyword)
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

    string add_keyword_to_white_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->add_keyword_to_white_list(_keyword))
            result += "Added!\n";
        else
            result +=  "Input keyword already exists in the white list!\n";
        return result + get_keyword_white_list();
    }

    string add_keywords_to_white_list(const string& _keywordlist)
    {
        vector<string> keyword_list;
        string keywordstr = _keywordlist;
        keywordstr .erase(std::remove(keywordstr .begin(), keywordstr .end(), ' '), keywordstr .end());
        boost::split(keyword_list, keywordstr , boost::is_any_of(","));

        for (string keyword : keyword_list)
            add_keyword_to_white_list(keyword);

        return "Added!" + get_keyword_white_list();
    }

    string remove_keyword_from_white_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->remove_keyword_from_white_list(_keyword))
            result += "Removed!\n";
        else
            result +=  "Input keyword does not exist in the white list!\n";
        return result + get_keyword_white_list();
    }


    void clear_keyword_black_list(){patternMiner->clear_keyword_white_list();}
    void clear_keyword_white_list(){patternMiner->clear_keyword_white_list();}

    string get_current_settings()
    {
        string result = "Current all settings:\n";
        result += get_Pattern_Max_Gram() + "\n";
        result += get_Enable_Interesting_Pattern() + "\n";
        result += get_Frequency_threshold() + "\n";
        result += get_Ignore_Link_Types() + "\n";
        result += get_use_keyword_black_list() + "\n";
        result += get_use_keyword_white_list() + "\n";
        result += get_keyword_black_list() + "\n";
        result += get_keyword_white_list() + "\n";

        return result;

    }

    // select a subset from the current AtomSpace with a list of keywords,splitted by ','.
    // the subset will contain max_distance connected atoms of these keywords
    // e.g.: "dog,cat,bike,swimming pool,computer"
    // when the max_distance, Pattern_Max_Gram will be used
    void select_subset_from_atomspace(const string& _keywordlist, int max_distance = 0)
    {
        vector<string> keyword_list;
        string keywordstr = _keywordlist;
        keywordstr .erase(std::remove(keywordstr .begin(), keywordstr .end(), ' '), keywordstr .end());
        boost::split(keyword_list, keywordstr , boost::is_any_of(","));

        if (keyword_list.size() == 0)
        {
            cout << "\nError: Keyword list is empty!" << std::endl;
            return;
        }

        if (max_distance <= 0)
        {
            max_distance = patternMiner->get_Pattern_Max_Gram();
            cout <<"\n Becasue max_distance <= 0, Pattern_Max_Gram = " << max_distance << " will be used!" << std::endl;
        }

        patternMiner->selectSubsetFromCorpus(keyword_list, max_distance);
    }

    void select_whitelist_subset_from_atomspace(int max_distance = 0)
    {
        vector<string> keyword_list = patternMiner->get_keyword_white_list();
        if (keyword_list.size() == 0)
        {
            cout << "\nError: white keyword list is empty!" << std::endl;
            return;
        }

        if (max_distance <= 0)
        {
            max_distance = patternMiner->get_Pattern_Max_Gram();
            cout <<"\n Becasue max_distance <= 0, Pattern_Max_Gram = " << max_distance << " will be used!" << std::endl;
        }

        patternMiner->selectSubsetFromCorpus(keyword_list, max_distance);
    }

    void apply_whitelist_keyword_filter_after_mining()
    {
        patternMiner->applyWhiteListKeywordfilterAfterMining();
    }


    // Note: this will release all the previous pattern mining results
    void reset_patternminer(bool resetAllSettingsFromConfig)
    {
        patternMiner->resetPatternMiner(resetAllSettingsFromConfig);
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
    AtomSpace* as = SchemeSmob::ss_get_env_as("patten miner");
    patternMiner = new PatternMiner(as);

    //---------------Note-----------------
    //
    // If you get compile error here, please pull the AtomSpace and make intall it.
    // git pull https://github.com/opencog/atomspace.git master
    //------------------------------------
    define_scheme_primitive("pm-run-patternminer", &PatternMinerSCM::run_patternminer, this, "patternminer");
    define_scheme_primitive("pm-get-current-settings", &PatternMinerSCM::get_current_settings, this, "patternminer");
    define_scheme_primitive("pm-get-pattern-max-gram", &PatternMinerSCM::get_Pattern_Max_Gram, this, "patternminer");
    define_scheme_primitive("pm-set-pattern-max-gram", &PatternMinerSCM::set_Pattern_Max_Gram, this, "patternminer");
    define_scheme_primitive("pm-get-enable-interesting-pattern", &PatternMinerSCM::get_Enable_Interesting_Pattern, this, "patternminer");
    define_scheme_primitive("pm-set-enable-interesting-pattern", &PatternMinerSCM::set_Enable_Interesting_Pattern, this, "patternminer");
    define_scheme_primitive("pm-get-frequency-threshold", &PatternMinerSCM::get_Frequency_threshold, this, "patternminer");
    define_scheme_primitive("pm-set-frequency-threshold", &PatternMinerSCM::set_Frequency_threshold, this, "patternminer");
    define_scheme_primitive("pm-get-ignore-link-types", &PatternMinerSCM::get_Ignore_Link_Types, this, "patternminer");
    define_scheme_primitive("pm-add-ignore-link-type", &PatternMinerSCM::add_Ignore_Link_Type, this, "patternminer");
    define_scheme_primitive("pm-remove-ignore-link-type", &PatternMinerSCM::remove_Ignore_Link_Type, this, "patternminer");
    define_scheme_primitive("pm-get-use-keyword-black-list", &PatternMinerSCM::get_use_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-get-use-keyword-white-list", &PatternMinerSCM::get_use_keyword_white_list, this, "patternminer");
    define_scheme_primitive("pm-get-keyword-black-list", &PatternMinerSCM::get_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-get-keyword-white-list", &PatternMinerSCM::get_keyword_white_list, this, "patternminer");
    define_scheme_primitive("pm-add-keyword-to-black-list", &PatternMinerSCM::add_keyword_to_black_list, this, "patternminer");
    define_scheme_primitive("pm-add-keywords-to-black-list", &PatternMinerSCM::add_keywords_to_black_list, this, "patternminer");
    define_scheme_primitive("pm-remove-keyword-from-black-list", &PatternMinerSCM::remove_keyword_from_black_list, this, "patternminer");
    define_scheme_primitive("pm-add-keyword-to-white-list", &PatternMinerSCM::add_keyword_to_white_list, this, "patternminer");
    define_scheme_primitive("pm-add-keywords-to-white-list", &PatternMinerSCM::add_keywords_to_white_list, this, "patternminer");
    define_scheme_primitive("pm-remove-keyword-from-white-list", &PatternMinerSCM::remove_keyword_from_white_list, this, "patternminer");
    define_scheme_primitive("pm-clear-keyword-black-list", &PatternMinerSCM::clear_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-clear-keyword-white-list", &PatternMinerSCM::clear_keyword_white_list, this, "patternminer");
    define_scheme_primitive("pm-select-subset-from-atomspace", &PatternMinerSCM::select_subset_from_atomspace, this, "patternminer");
    define_scheme_primitive("pm-select-whitelist-subset-from-atomspace", &PatternMinerSCM::select_whitelist_subset_from_atomspace, this, "patternminer");
    define_scheme_primitive("pm-apply-whitelist-keyword-filter-after-mining", &PatternMinerSCM::apply_whitelist_keyword_filter_after_mining, this, "patternminer");
    define_scheme_primitive("pm-reset-patternminer", &PatternMinerSCM::reset_patternminer, this, "patternminer");


}

extern "C" {
void opencog_patternminer_init(void)
{
    static PatternMinerSCM patternminerscm;
}
};

// --------------------------------------------------------------

