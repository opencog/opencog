/*
 * JsonicControlPolicyLoader.cc
 *
 *  Created on: 29 Dec, 2014
 *      Author: misgana
 */

#include "JsonicControlPolicyParamLoader.h"
#include "PolicyParams.h"

#include <fstream>
#include <lib/json_spirit/json_spirit.h>
#include <lib/json_spirit/json_spirit_stream_reader.h>

#include <opencog/guile/load-file.h>
#include <opencog/util/files.h>
#include <opencog/util/misc.h>
#include <opencog/util/Config.h>

JsonicControlPolicyParamLoader::JsonicControlPolicyParamLoader(AtomSpace * as,
		string conf_path) :
		ControlPolicyParamLoader(as, conf_path) {
	cur_read_rule_ = NULL;
}

JsonicControlPolicyParamLoader::~JsonicControlPolicyParamLoader() {

}

void JsonicControlPolicyParamLoader::set_disjunct_rules(void) {
	for (auto i = rule_mutex_map_.begin(); i != rule_mutex_map_.end(); ++i) {
		auto mset = i->second;
		auto cur_rule = i->first;
		for (string name : mset) {
			Rule* r = get_rule(name);
			if (!r)
				throw invalid_argument(
						"A rule by name " + name + " doesn't exist"); //TODO throw appropriate exception
			cur_rule->add_disjunct_rule(r);
		}
	}
}

Rule* JsonicControlPolicyParamLoader::get_rule(string& name) {
	for (Rule* r : rules_) {
		if (r->get_name() == name)
			return r;
	}
	return NULL;
}

void JsonicControlPolicyParamLoader::load_config() {
	try {
		ifstream is(conf_path_);
		Stream_reader<ifstream, Value> reader(is);
		Value value;
		while (reader.read_next(value))
			read_json(value);
		set_disjunct_rules();
	} catch (std::ios_base::failure& e) {
		std::cerr << e.what() << '\n';
	}
}

void JsonicControlPolicyParamLoader::read_array(const Value &v, int lev) {
	const Array& a = v.get_array();
	for (Array::size_type i = 0; i < a.size(); ++i)
		read_json(a[i], lev + 1);
}

void JsonicControlPolicyParamLoader::read_obj(const Value &v, int lev) {
	const Object& o = v.get_obj();
	for (Object::size_type i = 0; i < o.size(); ++i) {
		const Pair& p = o[i];
		auto key = p.name_;
		Value value = p.value_;
		if (key == RULES) {
			read_json(value, lev + 1);

		} else if (key == RULE_NAME) {
			cur_read_rule_ = new Rule(Handle::UNDEFINED);
			cur_read_rule_->set_name(value.get_value<string>());
			rules_.push_back(cur_read_rule_); //xxx take care of pointers
		} else if (key == FILE_PATH) {
			load_scm_file_relative(*as_, value.get_value<string>(),
					DEFAULT_MODULE_PATHS);
			Handle rule_handle = scm_eval_->eval_h(cur_read_rule_->get_name());
			cur_read_rule_->set_rule_handle(rule_handle);

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
				rule_mutex_map_[cur_read_rule_].push_back(
						a[i].get_value<string>());
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

void JsonicControlPolicyParamLoader::read_json(const Value &v,
		int level /* = -1*/) {
	switch (v.type()) {
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
		break;
	default:
		break;
	}
}

void JsonicControlPolicyParamLoader::read_null(const Value &v, int lev) {

}

template<typename > void JsonicControlPolicyParamLoader::read_primitive(
		const Value &v, int lev) {

}
