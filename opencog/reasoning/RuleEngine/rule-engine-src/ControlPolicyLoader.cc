/*
 * ControlPolicy.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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
#include "ControlPolicyLoader.h"

#include <opencog/guile/load-file.h>
#include <opencog/util/misc.h>
#include <opencog/util/Config.h>
#include <opencog/guile/SchemeEval.h>

ControlPolicyLoader::ControlPolicyLoader(AtomSpace * as, string conf_path) :
		as_(as){
	_conf_path = conf_path;
	scm_eval_ = new SchemeEval(as_);
}

ControlPolicyLoader::~ControlPolicyLoader() {
	delete scm_eval_;
	for (Rule *r : rules_)
		delete r;
}

void ControlPolicyLoader::load_chaining_rules() {
	vector<string> str_tokens;
	//FCHAIN_RULES= "[blink-var1,blink-var1,...]:rule_path1","[blink-var2]:rule_path2",...
	tokenize(config()["FCHAIN_RULES"], back_inserter(str_tokens), ", ");

	if (!str_tokens.empty())
		throw std::invalid_argument("no rules specified"); //xxx what type of exception?
	for (string rule : str_tokens) {
		auto it = remove_if(rule.begin(), rule.end(),
				[](char c) {return (c==']' or c=='[' or c=='"');});
		rule.erase(it, rule.end());
		vector<string> rule_names;
		tokenize(rule, back_inserter(rule_names), ":");
		assert(rule_names.size() == 2);
		load_scm_file_relative(*as_, rule_names[1], vector<string>(0)); // load rules to the chaining processor atomspace (i.e target_list_atom_space)
		istringstream is(rule_names[0]);
		string var_name;
		while (getline(is, var_name, ',')) {
			Rule *r = new Rule(scm_eval_->eval_h(var_name));
			rules_.push_back(r);
			strname_rule_map_[var_name] = r;
		}

	}
}

void ControlPolicyLoader::load_mutexes() {
	vector<string> str_tokens;

	//MUTEX = "nameA,nameB,...","namex,namey,..."
	tokenize(config()["MUTEX"], back_inserter(str_tokens), ", ");
	for (string r : str_tokens) {
		auto it = remove_if(r.begin(), r.end(), [](char c) {return (c=='"');});
		r.erase(it, r.end());
		string var_name;
		vector<Rule*> mutexes;
		istringstream is(r); //make sure the mutexes are already declared in FCHAIN_RULES param
		while (getline(is, var_name, ','))
			if (strname_rule_map_.find(var_name) == strname_rule_map_.end())
				throw std::invalid_argument(
						"No rule by name" + var_name + " is declared");
			else
				mutexes.push_back(strname_rule_map_[var_name]);
		mutex_sets_.push_back(mutexes);
	}

}
void ControlPolicyLoader::load_single_val_params() {
	max_iter_ = config().get_int("ITERATION_SIZE");
	attention_alloc_ = config().get_bool("ATTENTION_ALLOCATION_ENABLED"); //informs the callbacks to look for atoms only on the attentional focus
}

void ControlPolicyLoader::load_config() {
	try {
		config().load(_conf_path.c_str());
	} catch (RuntimeException &e) {
		std::cerr << e.getMessage() << std::endl;
	}
	load_chaining_rules();
	load_mutexes();
	load_single_val_params();
}

int ControlPolicyLoader::get_max_iter() {
	return max_iter_;
}

bool ControlPolicyLoader::get_attention_alloc() {
	return attention_alloc_;
}

vector<Rule*>& ControlPolicyLoader::get_rules() {
	return rules_;
}

vector<vector<Rule*>> ControlPolicyLoader::get_mutex_sets() {
	return mutex_sets_;
}
