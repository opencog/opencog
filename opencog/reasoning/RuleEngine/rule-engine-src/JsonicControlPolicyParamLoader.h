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

#include "ControlPolicyParamLoader.h"
#include "Rule.h"

#include <lib/json_spirit/json_spirit.h>

using namespace opencog;
using namespace json_spirit;

class JsonicControlPolicyParamLoader: public virtual ControlPolicyParamLoader {
private:
    Rule* cur_read_rule_ = NULL;
    map<Rule*, vector<string>> rule_mutex_map_;
    void read_json(const Value &v, int level = -1);
    void read_array(const Value &v, int level);
    void read_obj(const Value &v, int level);
    void read_null(const Value &v, int level);
    template<typename T> void read_primitive(const Value &v, int level);
    /**
     * sets the disjunct rules
     */
    void set_disjunct_rules(void);
    Rule* get_rule(string& name);
    const string get_absolute_path(const string& filename,
                                   vector<string> search_paths = { });
public:
    JsonicControlPolicyParamLoader(AtomSpace* as, string conf_path);
    virtual ~JsonicControlPolicyParamLoader();
    virtual void load_config();
};

#endif /* JSONICCONTROLPOLICYLOADER_H_ */
