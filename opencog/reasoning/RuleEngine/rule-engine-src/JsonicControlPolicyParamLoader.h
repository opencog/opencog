/*
 * JsonicControlPolicyParamLoader.h
 *
 * Copyright (C) 2015 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>   2015
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

#ifndef JSONICCONTROLPOLICYLOADER_H_
#define JSONICCONTROLPOLICYLOADER_H_

#include <lib/json_spirit/json_spirit.h>
#include <opencog/guile/SchemeEval.h>

#include "Rule.h"

namespace opencog {

using namespace json_spirit;

/**
 * The JsonicControlPolicyParamLoader class for loading rules.
 *
 * Loading the control policy from a .json file for setting what rules are
 * available and how they should be applied.
 */
class JsonicControlPolicyParamLoader
{
public:
    JsonicControlPolicyParamLoader(AtomSpace* as, string conf_path);
    virtual ~JsonicControlPolicyParamLoader();
    virtual void load_config();

    int get_max_iter();
    bool get_attention_alloc();
    vector<Rule*>& get_rules();

private:
    Rule* cur_read_rule_;

    AtomSpace* as_;
    SchemeEval* scm_eval_;

    vector<Rule*> rules_;

    // XXX what is these two variables?
    // Writing rule exclusion this way will fall in the same trap as RelEx2Logic...
    // As it gives the illusion that rules are exclusive to each other no matter the input, which is incorrect.
    map<Rule*, vector<string>> rule_mutex_map_;
    vector<vector<Rule*>> mutex_sets_; //mutually exclusive rules

    // XXX FIXME unused member variable
    map<string, Rule*> strname_rule_map_; //a map of name of the rule as represented in the scheme file and associated c++ rule object

    int max_iter_;
    bool attention_alloc_;
    string conf_path_;
    string log_level_;

    void read_json(const Value &v, int level = -1);
    void read_array(const Value &v, int level);
    void read_obj(const Value &v, int level);
    void read_null(const Value &v, int level);
    template<typename T> void read_primitive(const Value &v, int level);

    void set_disjunct_rules(void);

    Rule* get_rule(const string& name);
    const string get_working_path(const string& filename,
                                   vector<string> search_paths = { });
    vector<vector<Rule*>> get_mutex_sets(void);
};

} // ~namespace opencog

#endif /* JSONICCONTROLPOLICYLOADER_H_ */
