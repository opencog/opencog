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

#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/learning/PatternMiner/PatternMiner.h>

#include <boost/range/algorithm/remove.hpp>
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
        return  "max_gram: " + std::to_string( (int)(patternMiner->param.get_pattern_max_gram()));
    }
    string set_Pattern_Max_Gram(int _max_gram)
    {
        patternMiner->param.set_pattern_max_gram(_max_gram);
        return get_Pattern_Max_Gram();
    }

    string get_Enable_Interesting_Pattern()
    {
        bool enable = patternMiner->param.get_enable_interesting_pattern();

        if (enable)
            return "enable_Interesting_Pattern: true";
        else
            return "enable_Interesting_Pattern: false";
    }

    string set_Enable_Interesting_Pattern(bool _enable)
    {
        patternMiner->param.set_enable_interesting_pattern(_enable) ;
        return get_Enable_Interesting_Pattern();
    }

    string get_Frequency_threshold() {return "Frequency_threshold: " + std::to_string(patternMiner->param.get_frequency_threshold());}
    string set_Frequency_threshold(int _Frequency_threshold)
    {
        patternMiner->param.set_frequency_threshold(_Frequency_threshold);
        return get_Frequency_threshold();
    }


    string get_use_keyword_black_list()
    {
        bool enable = patternMiner->param.get_use_keyword_black_list();

        if (enable)
            return "use_keyword_black_list: true";
        else
            return "use_keyword_black_list: false";
    }

    string set_use_keyword_black_list(bool _use)
    {
        patternMiner->param.set_use_keyword_black_list(_use);
        return get_use_keyword_black_list();
    }

    string get_use_keyword_white_list()
    {
        bool enable = patternMiner->param.get_use_keyword_white_list();

        if (enable)
            return "use_keyword_white_list: true";
        else
            return "use_keyword_white_list: false";
    }

    string set_use_keyword_white_list(bool _use)
    {
        patternMiner->param.set_use_keyword_white_list(_use);
        return get_use_keyword_white_list();
    }

    string get_ignore_link_types()
    {
        string result =  "Ignore_Link_Types:";
        for (Type type : patternMiner->param.get_ignore_link_types())
            result +=  " " + nameserver().getTypeName(type);

        return result;
    }

    string get_linktype_white_list()
    {
        string result =  "linktype_white_list:";
        for (Type type : patternMiner->param.get_linktype_white_list())
            result +=  " " + nameserver().getTypeName(type);

        return result;
    }


    string get_use_linktype_white_list()
    {
        bool enable = patternMiner->param.get_use_linktype_white_list();

        if (enable)
            return "use_linktype_white_list: true";
        else
            return "use_linktype_white_list: false";
    }

    string set_use_linktype_white_list(bool _use)
    {
        patternMiner->param.set_use_linktype_white_list(_use);
        return get_use_linktype_white_list();
    }

    string get_use_linktype_black_list()
    {
        bool enable = patternMiner->param.get_use_linktype_black_list();

        if (enable)
            return "use_linktype_black_list: true";
        else
            return "use_linktype_black_list: false";
    }

    string set_use_linktype_black_list(bool _use)
    {
        patternMiner->param.set_use_linktype_black_list(_use);
        return get_use_linktype_black_list();
    }

    string add_Link_Type_to_white_list(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";

        if (patternMiner->param.add_linktype_to_white_list(atomType))
            result += "Added!\n";
        else
            result += "Input type already exists in the linktype white list!\n";

        return result + get_linktype_white_list();

    }

    string remove_Link_Type_from_white_list(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";
        if (patternMiner->param.remove_linktype_from_white_list(atomType))
            result += "Removed!\n";
        else
            result += "Input type does not exist in the linktype white list!\n";

        return result + get_linktype_white_list();

    }

    string add_ignore_link_type(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";

        if (patternMiner->param.add_ignore_link_type(atomType))
            result += "Added!\n";
        else
            result += "Input type already exists in the ingnore Link list!\n";

        return result + get_ignore_link_types();

    }

    string remove_Ignore_Link_Type(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";
        if (patternMiner->param.remove_ignore_link_type(atomType))
            result += "Removed!\n";
        else
            result += "Input type does not exist in the ingnore Link list!\n";

        return result + get_ignore_link_types();

    }

    string get_keyword_black_list()
    {
        string result =  "keyword_black_list:";
        for (const string& word : patternMiner->param.get_keyword_black_list())
            result +=  " " + word;

        return result;
    }

    string add_keyword_to_black_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->param.add_keyword_to_black_list(_keyword))
            result += "Added!\n";
        else
            result +=  "Input keyword already exists in the black list!\n";
        return result + get_keyword_black_list();
    }

    string add_keywords_to_black_list(const string& _keywordlist)
    {
        vector<string> keyword_list = Parameters::parse_comma_separated_list(_keywordlist);

        for (const string& keyword : keyword_list)
            add_keyword_to_black_list(keyword);

        return "Added!" + get_keyword_black_list();
    }

    string remove_keyword_from_black_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->param.remove_keyword_from_black_list(_keyword))
            result += "Removed!\n";
        else
            result +=  "Input keyword does not exist in the black list!\n";
        return result + get_keyword_black_list();
    }


    string get_keyword_white_list()
    {
        string result =  "keyword_white_list:";
        for (const string& word : patternMiner->param.get_keyword_white_list())
            result +=  " " + word;

        return result;
    }


    string add_keyword_to_white_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->param.add_keyword_to_white_list(_keyword))
            result += "Added!\n";
        else
            result +=  "Input keyword already exists in the white list!\n";
        return result + get_keyword_white_list();
    }

    string add_keywords_to_white_list(const string& _keywordlist)
    {
        vector<string> keyword_list = Parameters::parse_comma_separated_list(_keywordlist);

        for (const string& keyword : keyword_list)
            add_keyword_to_white_list(keyword);

        return "Added!" + get_keyword_white_list();
    }

    string remove_keyword_from_white_list(const string& _keyword)
    {
        string result = "";
        if (patternMiner->param.remove_keyword_from_white_list(_keyword))
            result += "Removed!\n";
        else
            result +=  "Input keyword does not exist in the white list!\n";
        return result + get_keyword_white_list();
    }

    string get_keyword_white_list_logic()
    {
        string result = "keyword_white_list_logic:";

        QUERY_LOGIC logic = patternMiner->param.get_keyword_white_list_logic();
        if (logic == QUERY_LOGIC::AND)
            result += "AND";
        else if (logic == QUERY_LOGIC::OR)
            result += "OR";
        else
            result += "EXCEPTION";

        return result;
    }

    string set_keyword_white_list_logic(const string& logic)
    {

        if ((logic == "AND") || (logic == "And") || (logic == "and"))
        {
            patternMiner->param.set_keyword_white_list_logic(QUERY_LOGIC::AND);
            return get_keyword_white_list_logic();
        }
        else if ((logic == "OR") || (logic == "Or") || (logic == "or"))
        {
            patternMiner->param.set_keyword_white_list_logic(QUERY_LOGIC::OR);
            return get_keyword_white_list_logic();
        }
        else
            return "Exception: keyword_white_list_logic only can be AND or OR.";
    }


    void clear_keyword_black_list(){patternMiner->param.clear_keyword_white_list();}
	void clear_keyword_white_list(){patternMiner->param.clear_keyword_white_list();}

    string set_enable_filter_node_types_should_not_be_vars(bool _enable)
    {
        patternMiner->param.set_enable_filter_node_types_should_not_be_vars(_enable);
        return get_enable_filter_node_types_should_not_be_vars();
    }

    string get_enable_filter_node_types_should_not_be_vars()
    {
        bool enable = patternMiner->param.get_enable_filter_node_types_should_not_be_vars();

        if (enable)
            return "enable_filter_node_types_should_not_be_vars: true";
        else
            return "enable_filter_node_types_should_not_be_vars: false";
    }

    string get_node_types_should_not_be_vars()
    {
        string result =  "node_types_should_not_be_vars:";
        for (Type type : patternMiner->param.get_node_types_should_not_be_vars())
            result +=  " " + nameserver().getTypeName(type);

        return result;
    }

    string add_node_type_to_node_types_should_not_be_vars(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";

        if (patternMiner->param.add_node_type_to_node_types_should_not_be_vars(atomType))
            result += "Added!\n";
        else
            result += "Input type already exists in the node_types_should_not_be_vars list!\n";

        return result + get_node_types_should_not_be_vars();

    }

    string remove_node_type_from_node_types_should_not_be_vars(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";
        if (patternMiner->param.remove_node_type_from_node_types_should_not_be_vars(atomType))
            result += "Removed!\n";
        else
            result += "Input type does not exist in the node_types_should_not_be_vars list!\n";

        return result + get_node_types_should_not_be_vars();

    }

    void clear_node_types_should_not_be_vars(){patternMiner->param.clear_node_types_should_not_be_vars();}


    string set_enable_filter_node_types_should_be_vars(bool _enable)
    {
        patternMiner->param.set_enable_filter_node_types_should_be_vars(_enable);
        return get_enable_filter_node_types_should_be_vars();
    }

    string get_enable_filter_node_types_should_be_vars()
    {
        bool enable = patternMiner->param.get_enable_filter_node_types_should_be_vars();

        if (enable)
            return "enable_filter_node_types_should_be_vars: true";
        else
            return "enable_filter_node_types_should_be_vars: false";
    }

    string get_node_types_should_be_vars()
    {
        string result =  "node_types_should_be_vars:";
        for (Type type : patternMiner->param.get_node_types_should_be_vars())
            result +=  " " + nameserver().getTypeName(type);

        return result;
    }

    string add_node_type_to_node_types_should_be_vars(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";

        if (patternMiner->param.add_node_type_to_node_types_should_be_vars(atomType))
            result += "Added!\n";
        else
            result += "Input type already exists in the node_types_should_be_vars list!\n";

        return result + get_node_types_should_be_vars();
    }

    string remove_node_type_from_node_types_should_be_vars(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";
        if (patternMiner->param.remove_node_type_from_node_types_should_be_vars(atomType))
            result += "Removed!\n";
        else
            result += "Input type does not exist in the node_types_should_be_vars list!\n";

        return result + get_node_types_should_be_vars();

    }

    void clear_node_types_should_be_vars(){patternMiner->param.clear_node_types_should_not_be_vars();}


    string set_enable_filter_links_of_same_type_not_share_second_outgoing(bool _enable)
    {
        patternMiner->param.set_enable_filter_links_of_same_type_not_share_second_outgoing(_enable);
        return get_enable_filter_links_of_same_type_not_share_second_outgoing();
    }

    string get_enable_filter_links_of_same_type_not_share_second_outgoing()
    {
        bool enable = patternMiner->param.get_enable_filter_links_of_same_type_not_share_second_outgoing();

        if (enable)
            return "enable_filter_links_of_same_type_not_share_second_outgoing: true";
        else
            return "enable_filter_links_of_same_type_not_share_second_outgoing: false";
    }

    string get_same_link_types_not_share_second_outgoing()
    {
        string result =  "same_link_types_not_share_second_outgoing Link_Types:";
        for (Type type : patternMiner->param.get_same_link_types_not_share_second_outgoing())
            result +=  " " + nameserver().getTypeName(type);

        return result;
    }

    string add_link_type_to_same_link_types_not_share_second_outgoing(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";

        if (patternMiner->param.add_link_type_to_same_link_types_not_share_second_outgoing(atomType))
            result += "Added!\n";
        else
            result += "Input type already exists in the same_link_types_not_share_second_outgoing list!\n";

        return result + get_same_link_types_not_share_second_outgoing();

    }

    string remove_link_type_from_same_link_types_not_share_second_outgoing(const string& _typeStr)
    {
        Type atomType = nameserver().getType(_typeStr);
        if (atomType == NOTYPE)
            return "Error: Input type doesn't exist!";

        string result = "";
        if (patternMiner->param.remove_link_type_from_same_link_types_not_share_second_outgoing(atomType))
            result += "Removed!\n";
        else
            result += "Input type does not exist in the same_link_types_not_share_second_outgoing list!\n";

        return result + get_same_link_types_not_share_second_outgoing();

    }

	void clear_same_link_types_not_share_second_outgoing(){patternMiner->param.clear_same_link_types_not_share_second_outgoing();}


    string get_current_settings()
    {
        string result = "Current all settings:\n";
        result += get_Pattern_Max_Gram() + "\n";
        result += get_Enable_Interesting_Pattern() + "\n";
        result += get_Frequency_threshold() + "\n";
        result += get_use_linktype_black_list() + "\n";
        result += get_use_linktype_white_list() + "\n";
        result += get_linktype_white_list() + "\n";
        result += get_ignore_link_types() + "\n";
        result += get_use_keyword_black_list() + "\n";
        result += get_use_keyword_white_list() + "\n";
        result += get_keyword_black_list() + "\n";
        result += get_keyword_white_list() + "\n";
        result += get_keyword_white_list_logic() + "\n";
        result += get_enable_filter_links_of_same_type_not_share_second_outgoing() + "\n";
        result += get_same_link_types_not_share_second_outgoing()  + "\n";
        result += get_enable_filter_node_types_should_not_be_vars()  + "\n";
        result += get_node_types_should_not_be_vars() + "\n";
        result += get_enable_filter_node_types_should_be_vars()  + "\n";
        result += get_node_types_should_be_vars() + "\n";

        return result;

    }

    // select a subset from the current AtomSpace with a list of keywords,splitted by ','.
    // the subset will contain max_distance connected atoms of these keywords
    // e.g.: "dog,cat,bike,swimming pool,computer"
    void select_subset_from_atomspace(const string& _keywordlist, int max_distance, bool if_contain_logic)
    {
        set<string> keyword_list = Parameters::parse_comma_separated_set(_keywordlist);

        if (keyword_list.empty())
        {
            cout << "\nError: Keyword list is empty!" << std::endl;
            return;
        }

        if (max_distance <= 0)
        {
            max_distance = patternMiner->param.get_pattern_max_gram();
            cout <<"\n Becasue max_distance <= 0, Pattern_Max_Gram = " << max_distance << " will be used!" << std::endl;
        }

        patternMiner->selectSubsetFromCorpus(keyword_list, max_distance, if_contain_logic);
    }

    // select a subset from the current AtomSpace with a list of keywords,splitted by ','.
    // the subset will contain max_distance connected atoms which contains these keywords in their labels
    // e.g.: "book,computer", Nodes with labels : "book", "handbook", "computer", "super computer" will all be selected.
    void select_subset_from_atomspace_nodes_contain_keywords(const string& _keywordlist, int max_distance)
    {
        select_subset_from_atomspace(_keywordlist, max_distance, true);
    }


    // select a subset from the current AtomSpace with a list of keywords,splitted by ','.
    // the subset will contain max_distance connected atoms which contains these keywords in their labels
    // e.g.: "book,computer", only Nodes with labels : "book", "computer" will all be selected;
    // "handbook", "super computer"  will not be selected
    void select_subset_from_atomspace_nodes_equalto_keywords(const string& _keywordlist, int max_distance)
    {
        select_subset_from_atomspace(_keywordlist, max_distance, false);
    }

    void select_whitelist_subset_from_atomspace(int max_distance, bool if_contain_logic)
    {
	    set<string> keyword_list = patternMiner->param.get_keyword_white_list();
        if (keyword_list.empty())
        {
            cout << "\nError: white keyword list is empty!" << std::endl;
            return;
        }

        if (max_distance <= 0)
        {
            max_distance = patternMiner->param.get_pattern_max_gram();
            cout <<"\n Becasue max_distance <= 0, Pattern_Max_Gram = " << max_distance << " will be used!" << std::endl;
        }

        patternMiner->selectSubsetFromCorpus(keyword_list, max_distance, if_contain_logic);
    }

    // select a subset from the current AtomSpace with the keywords defined in whitelist.
    // the subset will contain max_distance connected atoms which contains these keywords in their labels
    // e.g.: "book,computer", Nodes with labels : "book", "handbook", "computer", "super computer" will all be selected.
    void select_whitelist_subset_from_atomspace_contain_keywords(int max_distance)
    {
        select_whitelist_subset_from_atomspace(max_distance, true);
    }

    // select a subset from the current AtomSpace with the keywords defined in whitelist.
    // the subset will contain max_distance connected atoms which contains these keywords in their labels
    // e.g.: "book,computer", only Nodes with labels : "book", "computer" will all be selected;
    // "handbook", "super computer"  will not be selected
    void select_whitelist_subset_from_atomspace_equalto_keywords(int max_distance)
    {
        select_whitelist_subset_from_atomspace(max_distance, false);
    }

    void apply_whitelist_keyword_filter_after_mining()
    {
        patternMiner->applyWhiteListKeywordfilterAfterMining();
    }

    void select_whitelist_entity_links_subset_equalto_keywords()
    {
        set<string> whiteKeywords = patternMiner->param.get_keyword_white_list();
        patternMiner->selectSubsetAllEntityLinksContainsKeywords(whiteKeywords);
    }

    // Note: this will release all the previous pattern mining results
    void reset_patternminer(bool resetAllSettingsFromConfig)
    {
        patternMiner->resetPatternMiner(resetAllSettingsFromConfig);
    }

    void run_interestingness_evaluation()
    {
        patternMiner->runInterestingnessEvaluation();
    }

    void load_patterns_from_result_file(const string& fileName)
    {
        patternMiner->loadPatternsFromResultFile(fileName);
    }

    void query_patterns_with_frequency_surprisingnessI_ranges(const string& ranges, int gram)
    {
        vector<string> range_vector = Parameters::parse_comma_separated_list(ranges);
        unsigned int min_frequency = std::stoi(range_vector[0]);
        unsigned int max_frequency = std::stoi(range_vector[1]);
        float min_surprisingness_I = std::stof(range_vector[2]);
        float max_surprisingness_I = std::stof(range_vector[3]);
        patternMiner->queryPatternsWithFrequencySurprisingnessIRanges(min_frequency,  max_frequency, min_surprisingness_I, max_surprisingness_I, gram);
    }

    void query_patterns_with_surprisingnessI_and_surprisingnessII_ranges(const string& ranges, int gram)
    {
        vector<string> range_vector = Parameters::parse_comma_separated_list(ranges);
        unsigned int min_frequency = std::stoi(range_vector[0]);
        unsigned int max_frequency = std::stoi(range_vector[1]);
        float min_surprisingness_I = std::stof(range_vector[2]);
        float max_surprisingness_I = std::stof(range_vector[3]);
        float min_surprisingness_II = std::stof(range_vector[4]);
        float max_surprisingness_II = std::stof(range_vector[5]);
        patternMiner->queryPatternsWithSurprisingnessIAndIIRanges(min_frequency,  max_frequency, min_surprisingness_I, max_surprisingness_I,
                                                                  min_surprisingness_II,max_surprisingness_II, gram);
    }

    void query_patterns_with_frequency_surprisingnessB_and_subpatternnum_ranges(const string& ranges, int gram)
    {
	    vector<string> range_vector = Parameters::parse_comma_separated_list(ranges);
        unsigned int min_frequency = std::stoi(range_vector[0]);
        unsigned int max_frequency = std::stoi(range_vector[1]);
        double min_surprisingness_B = std::stof(range_vector[2]);
        double max_surprisingness_B = std::stof(range_vector[3]);
        unsigned int min_subpattern_num = std::stof(range_vector[4]);
        unsigned int max_subpattern_num = std::stof(range_vector[5]);
        patternMiner->queryPatternsWithFrequencySurprisingnessBRanges(min_frequency, max_frequency,
                                                                      min_surprisingness_B, max_surprisingness_B,
                                                                      min_subpattern_num, max_subpattern_num, gram);
    }

    void query_patterns_with_frequency_interactioninformation_ranges(const string& ranges, int gram)
    {
        vector<string> range_vector = Parameters::parse_comma_separated_list(ranges);
        unsigned int min_frequency = std::stoi(range_vector[0]);
        unsigned int max_frequency = std::stoi(range_vector[1]);
        float min_ii = std::stof(range_vector[2]);
        float max_ii = std::stof(range_vector[3]);
        patternMiner->queryPatternsWithFrequencyAndInteractionInformationRanges(min_frequency,  max_frequency, min_ii, max_ii, gram);
    }

    void OutPutStaticsToCsvFile(int n_gram)
    {
        patternMiner->OutPutStaticsToCsvFile(n_gram);
    }

    AtomSpace* get_result_AtomSpace()
    {
        return patternMiner->getResultAtomSpace();
    }

    void select_subset_for_DBpedia()
    {
        patternMiner->selectSubsetForDBpedia();
    }

    void load_all_DBpediaKeyNodes()
    {
        patternMiner->loandAllDBpediaKeyNodes();
    }

    void test_pattern_matcher()
    {
        patternMiner->testPatternMatcher();
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
    return nullptr;
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
    patternMiner = new PatternMiner(*as);

    //---------------Note-----------------
    //
    // If you get compile error here, please pull the AtomSpace and make intall it.
    // git pull https://github.com/opencog/atomspace.git master
    //------------------------------------
    define_scheme_primitive("pm-run-patternminer", &PatternMinerSCM::run_patternminer, this, "patternminer");

    define_scheme_primitive("pm-get-result-AtomSpace", &PatternMinerSCM::get_result_AtomSpace, this, "patternminer");

    define_scheme_primitive("pm-get-current-settings", &PatternMinerSCM::get_current_settings, this, "patternminer");
    define_scheme_primitive("pm-get-pattern-max-gram", &PatternMinerSCM::get_Pattern_Max_Gram, this, "patternminer");
    define_scheme_primitive("pm-set-pattern-max-gram", &PatternMinerSCM::set_Pattern_Max_Gram, this, "patternminer");
    define_scheme_primitive("pm-get-enable-interesting-pattern", &PatternMinerSCM::get_Enable_Interesting_Pattern, this, "patternminer");
    define_scheme_primitive("pm-set-enable-interesting-pattern", &PatternMinerSCM::set_Enable_Interesting_Pattern, this, "patternminer");
    define_scheme_primitive("pm-get-frequency-threshold", &PatternMinerSCM::get_Frequency_threshold, this, "patternminer");
    define_scheme_primitive("pm-set-frequency-threshold", &PatternMinerSCM::set_Frequency_threshold, this, "patternminer");

    define_scheme_primitive("pm-get-use-linktype-black-list", &PatternMinerSCM::get_use_linktype_black_list, this, "patternminer");
    define_scheme_primitive("pm-set-use-linktype-black-list", &PatternMinerSCM::get_use_linktype_black_list, this, "patternminer");
    define_scheme_primitive("pm-get-ignore-link-types", &PatternMinerSCM::get_ignore_link_types, this, "patternminer");
    define_scheme_primitive("pm-add-ignore-link-type", &PatternMinerSCM::add_ignore_link_type, this, "patternminer");


    define_scheme_primitive("pm-get-use-linktype-white-list", &PatternMinerSCM::get_use_linktype_white_list, this, "patternminer");
    define_scheme_primitive("pm-set-use-linktype-white-list", &PatternMinerSCM::get_use_linktype_white_list, this, "patternminer");
    define_scheme_primitive("pm-get-linktype-white-list", &PatternMinerSCM::get_linktype_white_list, this, "patternminer");
    define_scheme_primitive("pm-add-Link-Type-to-white-list", &PatternMinerSCM::add_Link_Type_to_white_list, this, "patternminer");


    define_scheme_primitive("pm-get-enable-filter-node-types-should-not-be-vars",
                            &PatternMinerSCM::get_enable_filter_node_types_should_not_be_vars, this, "patternminer");
    define_scheme_primitive("pm-set-enable-filter-node-types-should-not-be-vars",
                            &PatternMinerSCM::set_enable_filter_node_types_should_not_be_vars, this, "patternminer");
    define_scheme_primitive("pm-get-node-types-should-not-be-vars",
                            &PatternMinerSCM::get_node_types_should_not_be_vars, this, "patternminer");
    define_scheme_primitive("pm-add-node-type-to-node-types-should-not-be-vars",
                            &PatternMinerSCM::add_node_type_to_node_types_should_not_be_vars, this, "patternminer");
    define_scheme_primitive("pm-remove-node-type-from-node-types-should-not-be-vars",
                            &PatternMinerSCM::remove_node_type_from_node_types_should_not_be_vars, this, "patternminer");
    define_scheme_primitive("pm-clear-node-types-should-not-be-vars",
                            &PatternMinerSCM::clear_node_types_should_not_be_vars, this, "patternminer");

    define_scheme_primitive("pm-get-enable-filter-node-types-should-not-be-vars",
                            &PatternMinerSCM::get_enable_filter_node_types_should_be_vars, this, "patternminer");
    define_scheme_primitive("pm-set-enable-filter-node-types-should-not-be-vars",
                            &PatternMinerSCM::set_enable_filter_node_types_should_be_vars, this, "patternminer");
    define_scheme_primitive("pm-get-node-types-should-not-be-vars",
                            &PatternMinerSCM::get_node_types_should_be_vars, this, "patternminer");
    define_scheme_primitive("pm-add-node-type-to-node-types-should-not-be-vars",
                            &PatternMinerSCM::add_node_type_to_node_types_should_be_vars, this, "patternminer");
    define_scheme_primitive("pm-remove-node-type-from-node-types-should-not-be-vars",
                            &PatternMinerSCM::remove_node_type_from_node_types_should_be_vars, this, "patternminer");
    define_scheme_primitive("pm-clear-node-types-should-not-be-vars",
                            &PatternMinerSCM::clear_node_types_should_be_vars, this, "patternminer");


    define_scheme_primitive("pm-get-enable-filter-links-of-same-type-not-share-second-outgoing",
                            &PatternMinerSCM::get_enable_filter_links_of_same_type_not_share_second_outgoing, this, "patternminer");
    define_scheme_primitive("pm-set-enable-filter-links-of-same-type-not-share-second-outgoing",
                            &PatternMinerSCM::set_enable_filter_links_of_same_type_not_share_second_outgoing, this, "patternminer");
    define_scheme_primitive("pm-get-same-link-types-not-share-second-outgoing",
                            &PatternMinerSCM::get_same_link_types_not_share_second_outgoing, this, "patternminer");
    define_scheme_primitive("pm-add-link-type-to-same-link-types-not-share-second-outgoing",
                            &PatternMinerSCM::add_link_type_to_same_link_types_not_share_second_outgoing, this, "patternminer");
    define_scheme_primitive("pm-remove-link-type-from-same-link-types-not-share-second-outgoing",
                            &PatternMinerSCM::remove_link_type_from_same_link_types_not_share_second_outgoing, this, "patternminer");
    define_scheme_primitive("pm-clear-same-link-types-not-share-second-outgoing",
                            &PatternMinerSCM::clear_same_link_types_not_share_second_outgoing, this, "patternminer");

    define_scheme_primitive("pm-remove-ignore-link-type", &PatternMinerSCM::remove_Ignore_Link_Type, this, "patternminer");
    define_scheme_primitive("pm-get-use-keyword-black-list", &PatternMinerSCM::get_use_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-set-use-keyword-black-list", &PatternMinerSCM::set_use_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-get-use-keyword-white-list", &PatternMinerSCM::get_use_keyword_white_list, this, "patternminer");
    define_scheme_primitive("pm-set-use-keyword-white-list", &PatternMinerSCM::set_use_keyword_white_list, this, "patternminer");
    define_scheme_primitive("pm-get-keyword-black-list", &PatternMinerSCM::get_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-get-keyword-white-list", &PatternMinerSCM::get_keyword_white_list, this, "patternminer");
    define_scheme_primitive("pm-get-keyword-white-list-logic", &PatternMinerSCM::get_keyword_white_list_logic, this, "patternminer");
    define_scheme_primitive("pm-set-keyword-white-list-logic", &PatternMinerSCM::set_keyword_white_list_logic, this, "patternminer");
    define_scheme_primitive("pm-add-keyword-to-black-list", &PatternMinerSCM::add_keyword_to_black_list, this, "patternminer");
    define_scheme_primitive("pm-add-keywords-to-black-list", &PatternMinerSCM::add_keywords_to_black_list, this, "patternminer");
    define_scheme_primitive("pm-remove-keyword-from-black-list", &PatternMinerSCM::remove_keyword_from_black_list, this, "patternminer");
    define_scheme_primitive("pm-add-keyword-to-white-list", &PatternMinerSCM::add_keyword_to_white_list, this, "patternminer");
    define_scheme_primitive("pm-add-keywords-to-white-list", &PatternMinerSCM::add_keywords_to_white_list, this, "patternminer");
    define_scheme_primitive("pm-remove-keyword-from-white-list", &PatternMinerSCM::remove_keyword_from_white_list, this, "patternminer");
    define_scheme_primitive("pm-clear-keyword-black-list", &PatternMinerSCM::clear_keyword_black_list, this, "patternminer");
    define_scheme_primitive("pm-clear-keyword-white-list", &PatternMinerSCM::clear_keyword_white_list, this, "patternminer");

    define_scheme_primitive("pm-select-subset-from-atomspace-nodes-contain-keywords", &PatternMinerSCM::select_subset_from_atomspace_nodes_contain_keywords, this, "patternminer");
    define_scheme_primitive("pm-select-subset-from-atomspace-nodes-equalto-keywords", &PatternMinerSCM::select_subset_from_atomspace_nodes_equalto_keywords, this, "patternminer");
    define_scheme_primitive("pm-select-whitelist-subset-from-atomspace-contain-keywords", &PatternMinerSCM::select_whitelist_subset_from_atomspace_contain_keywords, this, "patternminer");
    define_scheme_primitive("pm-select-whitelist-subset-from-atomspace-equalto-keywords", &PatternMinerSCM::select_whitelist_subset_from_atomspace_equalto_keywords, this, "patternminer");
    define_scheme_primitive("pm-apply-whitelist-keyword-filter-after-mining", &PatternMinerSCM::apply_whitelist_keyword_filter_after_mining, this, "patternminer");
    define_scheme_primitive("pm-query-patterns-with-frequency-surprisingnessI-ranges", &PatternMinerSCM::query_patterns_with_frequency_surprisingnessI_ranges, this, "patternminer");
    define_scheme_primitive("pm-query-patterns-with-frequency-surprisingnessI-surprisingnessII-ranges", &PatternMinerSCM::query_patterns_with_surprisingnessI_and_surprisingnessII_ranges, this, "patternminer");
    define_scheme_primitive("pm-query-patterns-with-frequency-surprisingnessB-subpatternnum-ranges", &PatternMinerSCM::query_patterns_with_frequency_surprisingnessB_and_subpatternnum_ranges, this, "patternminer");
    define_scheme_primitive("pm-query-patterns-with-frequency-interactioninformation-ranges", &PatternMinerSCM::query_patterns_with_frequency_interactioninformation_ranges, this, "patternminer");

    define_scheme_primitive("pm-reset-patternminer", &PatternMinerSCM::reset_patternminer, this, "patternminer");
    define_scheme_primitive("pm-run-interestingness-evaluation", &PatternMinerSCM::run_interestingness_evaluation, this, "patternminer");
    define_scheme_primitive("pm-load-patterns-from-result-file", &PatternMinerSCM::load_patterns_from_result_file, this, "patternminer");


    define_scheme_primitive("pm-select-whitelist-entity-links-subset-equalto-keywords", &PatternMinerSCM::select_whitelist_entity_links_subset_equalto_keywords, this, "patternminer");

    define_scheme_primitive("pm-select-subset-for-DBpedia", &PatternMinerSCM::select_subset_for_DBpedia, this, "patternminer");
    define_scheme_primitive("pm-load-all-DBpediaKeyNodes", &PatternMinerSCM::load_all_DBpediaKeyNodes, this, "patternminer");

    define_scheme_primitive("pm-out-put-statics-to-csvfile", &PatternMinerSCM::OutPutStaticsToCsvFile, this, "patternminer");

    define_scheme_primitive("pm-test-pattern-matcher", &PatternMinerSCM::test_pattern_matcher, this, "patternminer");


}

extern "C" {
void opencog_patternminer_init(void)
{
    static PatternMinerSCM patternminerscm;
}
};

// --------------------------------------------------------------

