/*
 * JsonicControlPolicyParamLoader.cc
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

#include "JsonicControlPolicyParamLoader.h"
#include "PolicyParams.h"

#include <fstream>
#include <lib/json_spirit/json_spirit.h>
#include <lib/json_spirit/json_spirit_stream_reader.h>
#include <boost/filesystem.hpp>

#include <opencog/guile/load-file.h>
#include <opencog/util/files.h>
#include <opencog/util/misc.h>
#include <opencog/util/Config.h>

using namespace opencog;

/**
 * Constructor for JsonicControlPolicyParamLoader
 *
 * @param as          the atomspace where the rules' BindLink are loaded into
 * @param conf_path   path to the .json file
 */
JsonicControlPolicyParamLoader::JsonicControlPolicyParamLoader(AtomSpace* as,
                                                               string conf_path)
    : as_(as), conf_path_(conf_path)
{
    cur_read_rule_ = NULL;
    attention_alloc_ = false;
    scm_eval_ = SchemeEval::get_evaluator(as_);
}

/**
 * Destructor for JsonicControlPolicyParamLoader
 */
JsonicControlPolicyParamLoader::~JsonicControlPolicyParamLoader()
{
    // delete all dynamically allocated Rule object
    for (Rule* r : rules_)
        delete r;
}

/**
 * The main method for actually loading the config file.
 *
 * The .json file is not loaded until this method is called.
 */
void JsonicControlPolicyParamLoader::load_config()
{
    try {
        ifstream is(get_working_path(conf_path_));
        Stream_reader<ifstream, Value> reader(is);

        Value value;
        while (reader.read_next(value))
            read_json(value);

        set_disjunct_rules();
    } catch (std::ios_base::failure& e) {
        std::cerr << e.what() << '\n';
    }
}

/**
 * Get the max iteration value set in the .json
 *
 * @return the maximum iteration size
 */
int JsonicControlPolicyParamLoader::get_max_iter()
{
    return max_iter_;
}

/**
 * Get all rules defined in the control policy config.
 *
 * @return a vector of Rule*
 */
vector<Rule*> &JsonicControlPolicyParamLoader::get_rules()
{
    return rules_;
}

/**
 * Get whether to use attentional focus.
 *
 * For controlling whether to look only for atoms in the attentional focus or
 * the entire atomspace.
 *
 * @return true to use attentional focus, false otherwise
 */
bool JsonicControlPolicyParamLoader::get_attention_alloc()
{
    return attention_alloc_;
}

/**
 * Helper class for reading json value of type Array.
 *
 * @param v     a json Value
 * @param lev   the depth level of the json Value
 */
void JsonicControlPolicyParamLoader::read_array(const Value &v, int lev)
{
    const Array& a = v.get_array();
    for (Array::size_type i = 0; i < a.size(); ++i)
        read_json(a[i], lev + 1);
}

/**
 * Helper class for reading json value of type Object.
 *
 * @param v     a json Value
 * @param lev   the depth level of the json Value
 */
void JsonicControlPolicyParamLoader::read_obj(const Value &v, int lev)
{
    const Object& o = v.get_obj();

    for (Object::size_type i = 0; i < o.size(); ++i) {
        const Pair& p = o[i];
        auto key = p.name_;
        Value value = p.value_;

        if (key == RULES) {
            read_json(value, lev + 1);

        } else if (key == RULE_NAME) {
            cur_read_rule_ = new Rule(Handle::UNDEFINED);

            // the rule name is actually a scheme variable on the same name
            cur_read_rule_->set_name(value.get_value<string>());
            rules_.push_back(cur_read_rule_); // XXX take care of pointers

        } else if (key == FILE_PATH) {
            load_scm_file_relative(*as_, value.get_value<string>(), DEFAULT_MODULE_PATHS);

            // resolve the scheme variable to get the BindLink
            Handle rule_handle = scm_eval_->eval_h(cur_read_rule_->get_name());
            cur_read_rule_->set_handle(rule_handle);

        } else if (key == PRIORITY) {
            cur_read_rule_->set_cost(value.get_value<int>());

        } else if (key == CATEGORY) {
            cur_read_rule_->set_category(value.get_value<string>());

        } else if (key == ATTENTION_ALLOC) {
            attention_alloc_ = value.get_value<bool>();

        } else if (key == LOG_LEVEL) {
            log_level_ = value.get_value<string>();

        } else if (key == MUTEX_RULES and value.type() != null_type) {
            const Array& a = value.get_array();
            for (Array::size_type i = 0; i < a.size(); ++i) {
                rule_mutex_map_[cur_read_rule_].push_back(a[i].get_value<string>());
            }

        } else if (key == MAX_ITER) {
            max_iter_ = value.get_value<int>();

        } else if (key == LOG_LEVEL) {
            log_level_ = value.get_value<string>();

        } else {
            read_json(value, lev + 1);
        }

    }
}

/**
 * Helper class for reading a json Value of any type.
 *
 * Mostly for calling helper method for each type.
 *
 * @param v       a json Value
 * @param level   the depth level of the json Value
 */
void JsonicControlPolicyParamLoader::read_json(const Value &v,
                                               int level /* = -1*/)
{
    switch (v.type())
    {
    case obj_type:
        read_obj(v, level + 1);
        break;
    case array_type:
        read_array(v, level + 1);
        break;
    case str_type:
        break;
    case bool_type:
        break;
    case int_type:
        break;
    case real_type:
        break;
    case null_type:
        read_null(v, level + 1);
        break;
    default:
        break;
    }
}

/**
 * Helper class for reading json value of type null.
 *
 * @param v     a json Value
 * @param lev   the depth level of the json Value
 */
void JsonicControlPolicyParamLoader::read_null(const Value &v, int lev)
{
}

/**
 * XXX FIXME What is this method for?
 */
template<typename> void JsonicControlPolicyParamLoader::read_primitive(
    const Value &v, int lev)
{
}

/**
 * sets the disjunct rules
 *
 * XXX FIXME What is this method for?
 */
void JsonicControlPolicyParamLoader::set_disjunct_rules(void)
{

}

/**
 * Get a single Rule of a specific name.
 *
 * @param name   the name of the Rule
 * @return       pointer to a Rule object if found, nullptr otherwise
 */
Rule* JsonicControlPolicyParamLoader::get_rule(const string& name)
{
    for (Rule* r : rules_) {
        if (r->get_name() == name)
            return r;
    }
    return nullptr;
}

/**
 * Resolve which search path actually contain the file.
 *
 * Look throught each path in search_paths and check if file of filename
 * exists.
 *
 * @param filename       the name or sub-path to a file
 * @param search_paths   a vector of paths to look
 * @return               a successful path
 */
const string JsonicControlPolicyParamLoader::get_working_path(
        const string& filename, vector<string> search_paths)
{
    if (search_paths.empty())
        search_paths = DEFAULT_MODULE_PATHS;

    for (auto search_path : search_paths) {
        boost::filesystem::path modulePath(search_path);
        modulePath /= filename;

        logger().debug("Searching path %s", modulePath.string().c_str());

        if (boost::filesystem::exists(modulePath))
            return modulePath.string();
    }

    throw RuntimeException(TRACE_INFO, "%s could not be found",
                           filename.c_str());
}

/**
 * Get the set of mutually exclusive rules defined in the control policy file
 *
 * XXX What is this?
 *
 * @return a vector of a vector of Rule*
 */
vector<vector<Rule*> > JsonicControlPolicyParamLoader::get_mutex_sets()
{
    return mutex_sets_;
}

