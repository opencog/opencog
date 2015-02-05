/*
 * Rule.h
 *
 * Copyright (C) 2015 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  2015
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

#ifndef RULE_H_
#define RULE_H_

/*#include <string>
 #include <vector>*/
#include <opencog/atomspace/Handle.h>

using namespace opencog;
using namespace std;
/**
 * Mainly wraps a BindLink/ImplicationLink but with other important attributes
 */
class Rule {
private:
	Handle rule_handle_;
	string name_;
	string category_;
	int priority_ = -1;
	vector<Rule*> mutex_rules_;
public:
	Handle get_handle();
	Rule(Handle rule);
	inline bool operator ==(const Rule& r) {
		return r.rule_handle_ == rule_handle_;
	}
	inline bool operator !=(const Rule& r) {
		return !(r.rule_handle_ == rule_handle_);
	}
	inline bool operator >(const Rule& r) {
		return priority_ > r.priority_;
	}
	inline bool operator <(const Rule& r) {
		return priority_ < r.priority_;
	}
	void set_rule_handle(Handle h) throw (exception);
	void set_name(string name);
	string get_name();
	void set_category(string name);
	string& get_category();
	void set_priority(int p);
	int get_priority();
	void add_mutex_rule(Rule* r);
	vector<Rule*> get_mutex_rules(void);
	virtual ~Rule();
};
#endif /* RULE_H_ */
