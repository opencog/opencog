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

#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

using namespace std;

/**
 * Mainly wraps a BindLink/ImplicationLink but with other important attributes
 */
class Rule
{
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

	void set_handle(Handle h) throw (InvalidParamException);
	void set_name(string name);
	string get_name();
	void set_category(string name);
	void set_cost(int p);

	Handle get_handle();
	Handle get_implicant();
	Handle get_implicand();
	string& get_category();
	int get_cost();

	Rule standardize_apart();

private:
	Handle rule_handle_;
	string name_;
	string category_;
	int cost_;

	Handle standardize_helper(Handle, std::map<Handle, Handle>&);
};

} // ~namespace opencog

#endif /* RULE_H_ */
