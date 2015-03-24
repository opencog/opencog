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

#include <opencog/atomspace/Handle.h>

using namespace opencog;
using namespace std;

/**
 * Mainly wraps a BindLink/ImplicationLink but with other important attributes
 */
class Rule
{
private:
	Handle rule_handle_;
	string name_;
	string category_;
	int cost_ = -1;
	vector<Rule*> disjunct_rules_;

public:
	Rule(Handle rule);
	virtual ~Rule();

	inline bool operator ==(const Rule& r) {
		return r.rule_handle_ == rule_handle_;
	}
	inline bool operator !=(const Rule& r) {
		return !(r.rule_handle_ == rule_handle_);
	}
	inline bool operator >(const Rule& r) {
		return cost_ > r.cost_;
	}
	inline bool operator <(const Rule& r) {
		return cost_ < r.cost_;
	}

	void set_rule_handle(Handle h) throw (exception);
	void set_name(string name);
	string get_name();
	void set_category(string name);
	void set_cost(int p);

	Handle get_handle();
	string& get_category();
	int get_cost();
	vector<Rule*> get_disjunct_rules(void);

	void add_disjunct_rule(Rule* r);
};
#endif /* RULE_H_ */
