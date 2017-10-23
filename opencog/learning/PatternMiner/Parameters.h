/*
 * opencog/learning/PatternMiner/Parameters.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller in 2017
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

#ifndef _OPENCOG_PATTERNMINER_Parameters_H
#define _OPENCOG_PATTERNMINER_Parameters_H

#include <set>
#include <vector>
#include <string>

#include <opencog/learning/PatternMiner/types/atom_types.h>

namespace opencog { namespace PatternMining {

enum QUERY_LOGIC
{
    OR,
    AND
};

struct Parameters {
	unsigned int MAX_GRAM;

	unsigned int THREAD_NUM;

	std::string pattern_mining_mode;

	double max_var_num_percent;

	bool enable_filter_leaves_should_not_be_vars;
	bool enable_filter_links_should_connect_by_vars;
	bool enable_filter_links_of_same_type_not_share_second_outgoing;
	bool enable_filter_not_same_var_from_same_predicate;
	bool enable_filter_not_all_first_outgoing_const;
	bool enable_filter_first_outgoing_evallink_should_be_var;
	bool enable_filter_node_types_should_not_be_vars;

	std::set<Type> node_types_should_not_be_vars;

	bool enable_filter_node_types_should_be_vars;
	std::set<Type> node_types_should_be_vars;

	std::set<Type> same_link_types_not_share_second_outgoing;

	unsigned int threshold_frequency;

	bool use_keyword_white_list;
	bool use_keyword_black_list;

	std::set<std::string> keyword_black_list;
	std::set<std::string> keyword_white_list;

	// = true will filter out atoms with labels contain keyword, = fasle will only filter out atoms with labels equal to any keyword
	bool keyword_black_logic_is_contain;

	QUERY_LOGIC keyword_white_list_logic;

	bool use_linktype_black_list;
	bool use_linktype_white_list;

	std::set<Type> linktype_black_list;
	std::set<Type> linktype_white_list;

	bool if_quote_output_pattern;

	bool calculate_type_b_surprisingness;

	bool enable_interesting_pattern;

	// Only effective when Enable_Interesting_Pattern is true.
	bool enable_interaction_information;
	bool enable_surprisingness;

	bool enable_unify_unordered_links; // if the corpus contains unordered Links like AND_LINK

	Type output_pattern_quoted_linktype;

	bool only_mine_patterns_start_from_white_list;
	bool only_mine_patterns_start_from_white_list_contain;

	bool only_output_patterns_contains_white_keywords;

	void reSetAllSettingsFromConfig();

	// -------------------------------basic settings----------------------

	unsigned int get_pattern_max_gram() {return MAX_GRAM;}
	void set_pattern_max_gram(unsigned int _max_gram) { MAX_GRAM = _max_gram;}

	bool get_enable_interesting_pattern() {return enable_interesting_pattern;}
	void set_enable_interesting_pattern(bool _enable) {enable_interesting_pattern = _enable;}

	unsigned int get_frequency_threshold() {return threshold_frequency;}
	void set_frequency_threshold(unsigned int _Frequency_threshold) {threshold_frequency = _Frequency_threshold;}

	// -------------------------------end basic settings----------------------

	// -------------------------------filter settings----------------------
	bool get_use_keyword_black_list() {return use_keyword_black_list;}
	void set_use_keyword_black_list(bool _use) {use_keyword_black_list = _use;}

	bool get_use_keyword_white_list() {return use_keyword_white_list;}
	void set_use_keyword_white_list(bool _use) {use_keyword_white_list = _use;}

	bool get_use_linktype_black_list() {return use_linktype_black_list;}
	void set_use_linktype_black_list(bool _use) {use_linktype_black_list = _use;}

	bool get_use_linktype_white_list() {return use_linktype_white_list;}
	void set_use_linktype_white_list(bool _use) {use_linktype_white_list = _use;}

	std::set<Type> get_linktype_white_list() {return linktype_white_list;}
	bool add_linktype_to_white_list(Type _type);
	bool remove_linktype_from_white_list(Type _type);

	std::set<Type> get_ignore_link_types() {return linktype_black_list;}
	bool add_ignore_link_type(Type _type);
	bool remove_ignore_link_type(Type _type);

	std::set<std::string> get_keyword_black_list() {return keyword_black_list;}
	bool add_keyword_to_black_list(const std::string& _keyword);
	bool remove_keyword_from_black_list(const std::string& _keyword);
	void clear_keyword_black_list() {keyword_black_list.clear();}

	std::set<std::string> get_keyword_white_list() {return keyword_white_list;}
	bool add_keyword_to_white_list(const std::string& _keyword);
	bool remove_keyword_from_white_list(const std::string& _keyword);
	void clear_keyword_white_list() {keyword_white_list.clear();}

	QUERY_LOGIC get_keyword_white_list_logic() {return keyword_white_list_logic;}
	void set_keyword_white_list_logic(QUERY_LOGIC logic) {keyword_white_list_logic = logic;}

	void set_enable_filter_links_of_same_type_not_share_second_outgoing(bool _enable){enable_filter_links_of_same_type_not_share_second_outgoing = _enable;}
	bool get_enable_filter_links_of_same_type_not_share_second_outgoing(){return enable_filter_links_of_same_type_not_share_second_outgoing;}
	std::set<Type> get_same_link_types_not_share_second_outgoing(){return same_link_types_not_share_second_outgoing;}
	bool add_link_type_to_same_link_types_not_share_second_outgoing(Type _type);
	bool remove_link_type_from_same_link_types_not_share_second_outgoing(Type _type);
	void clear_same_link_types_not_share_second_outgoing() {same_link_types_not_share_second_outgoing.clear();}

	void set_enable_filter_node_types_should_not_be_vars(bool _enable) {enable_filter_node_types_should_not_be_vars=_enable;}
	bool get_enable_filter_node_types_should_not_be_vars() {return enable_filter_node_types_should_not_be_vars;}
	std::set<Type> get_node_types_should_not_be_vars() {return node_types_should_not_be_vars;}
	bool add_node_type_to_node_types_should_not_be_vars(Type _type);
	bool remove_node_type_from_node_types_should_not_be_vars(Type _type);
	void clear_node_types_should_not_be_vars() {node_types_should_not_be_vars.clear();}

	void set_enable_filter_node_types_should_be_vars(bool _enable) {enable_filter_node_types_should_be_vars = _enable;}
	bool get_enable_filter_node_types_should_be_vars() {return enable_filter_node_types_should_be_vars;}
	std::set<Type> get_node_types_should_be_vars() {return node_types_should_be_vars;}
	bool add_node_type_to_node_types_should_be_vars(Type _type);
	bool remove_node_type_from_node_types_should_be_vars(Type _type);
	void clear_node_types_should_be_vars() {node_types_should_be_vars.clear();}

	// -------------------------------end filter settings----------------------

	// ------------------------------------- Helpers --------------------------

	void addAtomTypesFromString(const std::string& node_types_str,
	                            std::set<Type>& types);

	// Turn a string like "qewr, wert, erty" into a vector of 3
	// strings {"qwer", wert", erty"}
	static std::vector<std::string> parse_comma_separated_list(std::string str);
	static std::set<std::string> parse_comma_separated_set(std::string str);
};
		
}}
	
#endif
