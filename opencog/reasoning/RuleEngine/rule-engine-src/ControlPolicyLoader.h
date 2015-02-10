/*
 * ControlPolicy.h
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
#ifndef CONTROL_POLICY_
#define CONTROL_POLICY_
/*xxx what do we need? load stuffs. what kind of stuffs.
 -(load-pln with this config file)
 -then what rules to use
 -what rules are mutually exclusive
 */
#include "Rule.h"

#include <opencog/guile/load-file.h>
#include <opencog/util/misc.h>
#include <opencog/util/Config.h>
#include <opencog/guile/SchemeEval.h>

using namespace std;
/**
 * A default control policy loader that loads from an opencog config file
 */
class ControlPolicyLoader {
private:
	void load_chaining_rules();
	void load_mutexes();
	void load_single_val_params();
protected:
	//list of control policy parameters
	AtomSpace* as_;
	SchemeEval* scm_eval_;
	vector<Rule*> rules_;
	vector<vector<Rule*>> mutex_sets_; //mutually exclusive rules
	map<string, Rule*> strname_rule_map_; //a map of name of the rule as represented in the scheme file and associated c++ rule object
	int max_iter_;
	bool attention_alloc_;
	string _conf_path;
	string log_level_;
	/**
	 * @return a set of mutually exclusive rules defined in the control policy file
	 */
	vector<vector<Rule*>> get_mutex_sets(void);
public:
	ControlPolicyLoader(AtomSpace* as, string conf_path);
	virtual ~ControlPolicyLoader();
	/**
	 * loads the configuration file that contains control policy and other params
	 */
	virtual void load_config(void);
	/**
	 * @return the maximum iteration size
	 */
	int get_max_iter(void);
	/**
	 * @return a boolean flag that tells whether to look only for atoms in the attentional focus or an entire atomspace
	 */
	bool get_attention_alloc(void);
	/**
	 * @return get all rules defined in the control policy config
	 */
	vector<Rule*>& get_rules(void);
};

#endif

