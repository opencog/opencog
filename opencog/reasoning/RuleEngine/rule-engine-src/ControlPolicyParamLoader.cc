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
#include "ControlPolicyParamLoader.h"

#include <opencog/guile/load-file.h>
#include <opencog/util/misc.h>
#include <opencog/util/files.h>
#include <opencog/util/Config.h>
#include <opencog/guile/SchemeEval.h>

ControlPolicyParamLoader::ControlPolicyParamLoader(AtomSpace * as, string conf_path) :
		as_(as)
{
	conf_path_ = conf_path;
	scm_eval_ = get_evaluator(as_);
}

ControlPolicyParamLoader::~ControlPolicyParamLoader()
{
	for (Rule *r : rules_)
		delete r;
}

/**
 * Helper function for loading chaining rules.
 *
 * The rules are in .conf file specified in the format:
 *
 * FCHAIN_RULES = "[blink-var1,blink-var1,...]:rule/path/1.scm",
 *                "[blink-var2]:rule/path/2.scm",
 *                ...
 *
 * where each blink-var# is a scheme variable of the same name in the scm file,
 * linked to a BindLink.
 */
void ControlPolicyParamLoader::load_chaining_rules()
{
	vector<string> str_tokens;
	tokenize(config()["FCHAIN_RULES"], back_inserter(str_tokens), ", ");

	if (!str_tokens.empty())
		throw std::invalid_argument("no rules specified"); //xxx what type of exception?

	for (string rule : str_tokens) {
		auto it = remove_if(rule.begin(), rule.end(),
		                    [](char c) { return (c==']' or c=='[' or c=='"'); });
		rule.erase(it, rule.end());

		vector<string> rule_names;
		tokenize(rule, back_inserter(rule_names), ":");
		assert(rule_names.size() == 2);

		// load rules to the chaining processor atomspace (i.e target_list_atom_space)
		load_scm_file_relative(*as_, rule_names[1], DEFAULT_MODULE_PATHS);

		istringstream is(rule_names[0]);
		string var_name;
		while (getline(is, var_name, ',')) {
			// resolve the scheme variable to get the BindLink
			Rule *r = new Rule(scm_eval_->eval_h(var_name));
			rules_.push_back(r);
			strname_rule_map_[var_name] = r;
		}

	}
}

void ControlPolicyParamLoader::load_mutexes()
{
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

void ControlPolicyParamLoader::load_single_val_params()
{
	max_iter_ = config().get_int("ITERATION_SIZE");
	attention_alloc_ = config().get_bool("ATTENTION_ALLOCATION_ENABLED"); //informs the callbacks to look for atoms only on the attentional focus
}

void ControlPolicyParamLoader::load_config()
{
	try {
		config().load(conf_path_.c_str());
	} catch (RuntimeException &e) {
		std::cerr << e.getMessage() << std::endl;
	}
	load_chaining_rules();
	load_mutexes();
	load_single_val_params();
}

int ControlPolicyParamLoader::get_max_iter()
{
	return max_iter_;
}

bool ControlPolicyParamLoader::get_attention_alloc()
{
	return attention_alloc_;
}

vector<Rule*>& ControlPolicyParamLoader::get_rules()
{
	return rules_;
}

vector<vector<Rule*>> ControlPolicyParamLoader::get_mutex_sets()
{
	return mutex_sets_;
}
