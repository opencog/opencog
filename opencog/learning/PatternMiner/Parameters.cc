/*
 * opencog/learning/PatternMiner/Parameters.cc
 *
 * Copyright (C) 2017 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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

#include <thread>

#include <boost/range/algorithm/remove.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <opencog/util/Config.h>
#include <opencog/util/oc_assert.h>
#include <opencog/atoms/base/ClassServer.h>

#include "Parameters.h"

namespace opencog { namespace PatternMining {

using namespace std;

void Parameters::reSetAllSettingsFromConfig()
{
	pattern_mining_mode = config().get("Pattern_mining_mode");

	if (0 == pattern_mining_mode.size())
	{
		config().load("opencog_patternminer.conf", true);
		pattern_mining_mode = config().get("Pattern_mining_mode");
	}

	OC_ASSERT(pattern_mining_mode == "Breadth_First" || pattern_mining_mode == "Depth_First");

	int max_gram = config().get_int("Pattern_Max_Gram");
	MAX_GRAM = (unsigned int)max_gram;

	enable_interesting_pattern = config().get_bool("Enable_Interesting_Pattern");
	enable_interaction_information = config().get_bool("Enable_Interaction_Information");
	enable_surprisingness = config().get_bool("Enable_surprisingness");

	THREAD_NUM = config().get_int("Max_thread_num");
	unsigned int system_thread_num = std::thread::hardware_concurrency();
	if (THREAD_NUM > system_thread_num - 1)
	{
		THREAD_NUM = std::max(1U, system_thread_num - 1);

		cout << "\nThere is only " << system_thread_num
		     << " cores in this machine, so the Max_thread_num = "
		     << config().get_int("Max_thread_num")
		     << " will not be used. " << THREAD_NUM
		     << " threads will be used instead." << std::endl;
	}

	threshold_frequency = config().get_int("Frequency_threshold");

	max_var_num_percent = config().get_double("max_var_num_percent");

	if_quote_output_pattern = config().get_bool("if_quote_output_pattern");
	string quotedTypeStr = config().get("output_pattern_quoted_linktype");
//    cout << "quotedTypeStr = " << quotedTypeStr << std::endl;
	output_pattern_quoted_linktype = nameserver().getType(quotedTypeStr);
	if (output_pattern_quoted_linktype == NOTYPE)
	{
		cout << "\nError: output_pattern_quoted_linktype : "<< quotedTypeStr << " in config file doesn't exist!" << std::endl;
	}

	calculate_type_b_surprisingness = config().get_bool("calculate_type_b_surprisingness");

	use_keyword_black_list = config().get_bool("use_keyword_black_list");
	use_keyword_white_list = config().get_bool("use_keyword_white_list");

	keyword_black_logic_is_contain = config().get_bool("keyword_black_logic_is_contain");

	string keyword_black_list_str  = config().get("keyword_black_list");
	keyword_black_list = parse_comma_separated_set(keyword_black_list_str);

	string keyword_white_list_str  = config().get("keyword_white_list");
	keyword_white_list = parse_comma_separated_set(keyword_white_list_str);

	string keyword_white_list_logic_str = config().get("keyword_white_list_logic");
	keyword_white_list_logic =
		boost::algorithm::to_lower_copy(keyword_white_list_logic_str) == "and" ?
		QUERY_LOGIC::AND : QUERY_LOGIC::OR;

	use_linktype_black_list = config().get_bool("use_linktype_black_list");
	use_linktype_white_list = config().get_bool("use_linktype_white_list");

	// use_linktype_black_list and use_linktype_white_list should not both be true
	assert((! use_linktype_black_list) || (! use_linktype_white_list));

	linktype_black_list.clear();
	string linktype_black_list_str = config().get("linktype_black_list");
	addAtomTypesFromString(linktype_black_list_str, linktype_black_list);

	linktype_white_list.clear();
	string linktype_white_list_str = config().get("linktype_white_list");
	addAtomTypesFromString(linktype_white_list_str, linktype_white_list);

	enable_filter_leaves_should_not_be_vars = config().get_bool("enable_filter_leaves_should_not_be_vars");
	enable_filter_links_should_connect_by_vars = config().get_bool("enable_filter_links_should_connect_by_vars");
	enable_filter_node_types_should_not_be_vars =  config().get_bool("enable_filter_node_types_should_not_be_vars");
	enable_filter_node_types_should_be_vars =  config().get_bool("enable_filter_node_types_should_be_vars");
	enable_filter_links_of_same_type_not_share_second_outgoing = config().get_bool("enable_filter_links_of_same_type_not_share_second_outgoing");
	enable_filter_not_all_first_outgoing_const = config().get_bool("enable_filter_not_all_first_outgoing_const");
	enable_filter_not_same_var_from_same_predicate = config().get_bool("enable_filter_not_same_var_from_same_predicate");
	enable_filter_first_outgoing_evallink_should_be_var = config().get_bool("enable_filter_first_outgoing_evallink_should_be_var");

	node_types_should_not_be_vars.clear();
	string node_types_str = config().get("node_types_should_not_be_vars");
	addAtomTypesFromString(node_types_str, node_types_should_not_be_vars);

	node_types_should_be_vars.clear();
	node_types_str = config().get("node_types_should_be_vars");
	addAtomTypesFromString(node_types_str, node_types_should_be_vars);

	same_link_types_not_share_second_outgoing.clear();
	string link_types_str = config().get("same_link_types_not_share_second_outgoing");
	addAtomTypesFromString(link_types_str, same_link_types_not_share_second_outgoing);

	only_mine_patterns_start_from_white_list = config().get_bool("only_mine_patterns_start_from_white_list");
	only_mine_patterns_start_from_white_list_contain = config().get_bool("only_mine_patterns_start_from_white_list_contain");

	only_output_patterns_contains_white_keywords = config().get_bool("only_output_patterns_contains_white_keywords");

	enable_unify_unordered_links = config().get_bool("enable_unify_unordered_links");
}

bool Parameters::add_linktype_to_white_list(Type _type)
{
    return linktype_white_list.insert(_type).second;
}

bool Parameters::remove_linktype_from_white_list(Type _type)
{
    return linktype_white_list.erase(_type);
}

bool Parameters::add_ignore_link_type(Type _type)
{
    return linktype_black_list.insert(_type).second;
}

bool Parameters::remove_ignore_link_type(Type _type)
{
    return linktype_black_list.erase(_type);
}

bool Parameters::add_keyword_to_black_list(const string& _keyword)
{
    if (_keyword == "")
        return false;
    return keyword_black_list.insert(_keyword).second;
}

bool Parameters::remove_keyword_from_black_list(const string& _keyword)
{
    return keyword_black_list.erase(_keyword);
}

bool Parameters::add_keyword_to_white_list(const string& _keyword)
{
    return keyword_white_list.insert(_keyword).second;
}

bool Parameters::remove_keyword_from_white_list(const string& _keyword)
{
    return keyword_white_list.erase(_keyword);
}

bool Parameters::add_link_type_to_same_link_types_not_share_second_outgoing(Type _type)
{
    return same_link_types_not_share_second_outgoing.insert(_type).second;
}

bool Parameters::remove_link_type_from_same_link_types_not_share_second_outgoing(Type _type)
{
    return same_link_types_not_share_second_outgoing.erase(_type);
}

bool Parameters::add_node_type_to_node_types_should_not_be_vars(Type _type)
{
    return node_types_should_not_be_vars.insert(_type).second;
}

bool Parameters::remove_node_type_from_node_types_should_not_be_vars(Type _type)
{
    return node_types_should_not_be_vars.erase(_type);
}

bool Parameters::add_node_type_to_node_types_should_be_vars(Type _type)
{
    return node_types_should_be_vars.insert(_type).second;
}

bool Parameters::remove_node_type_from_node_types_should_be_vars(Type _type)
{
    return node_types_should_be_vars.erase(_type);
}

void Parameters::addAtomTypesFromString(const string& node_types_str,
                                        set<Type>& types)
{
    for (const string& typestr : parse_comma_separated_list(node_types_str))
    {
        Type atomType = nameserver().getType(typestr);
        if (atomType == NOTYPE)
        {
            cout << "\nCannot find Node Type: " << typestr << " in config file.\n";
            continue;
        }
        types.insert(atomType);
    }
}

vector<string> Parameters::parse_comma_separated_list(string str)
{
    str.erase(boost::remove(str, ' '), str.end());
    vector<string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    return strs;
}

set<string> Parameters::parse_comma_separated_set(string str)
{
    vector<string> l = parse_comma_separated_list(str);
    return set<string>(l.begin(), l.end());
}

}}
