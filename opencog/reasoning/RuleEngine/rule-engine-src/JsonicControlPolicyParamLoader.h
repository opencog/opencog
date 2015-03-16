/*
 * JsonicControlPolicyLoader.h
 *
 *  Created on: 29 Dec, 2014
 *      Author: misgana
 */

#ifndef JSONICCONTROLPOLICYLOADER_H_
#define JSONICCONTROLPOLICYLOADER_H_

#include "ControlPolicyParamLoader.h"
#include "Rule.h"

#include <lib/json_spirit/json_spirit.h>

using namespace opencog;
using namespace json_spirit;

class JsonicControlPolicyParamLoader:public virtual ControlPolicyParamLoader {
private:
	Rule* cur_read_rule_ = NULL;
	map<Rule*,vector<string>> rule_mutex_map_;
	void read_json(const Value &v, int level = -1);
	void read_array(const Value &v, int level);
	void read_obj(const Value &v, int level);
	void read_null(const Value &v, int level);
	template <typename T> void read_primitive(const Value &v,int level);
	/**
	 * sets the disjunct rules
	 */
	void set_disjunct_rules(void);
	Rule* get_rule(string& name);
	const string load_json_file_relative(const string& filename,
	            vector<string> search_paths = { });
public:
	JsonicControlPolicyParamLoader(AtomSpace* as, string conf_path);
	virtual ~JsonicControlPolicyParamLoader();
	virtual void load_config();
};

#endif /* JSONICCONTROLPOLICYLOADER_H_ */
