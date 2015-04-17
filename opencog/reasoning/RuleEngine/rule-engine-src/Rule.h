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

#include <opencog/atoms/bind/VariableList.h>
#include <opencog/atomspace/AtomSpace.h>

#include <boost/operators.hpp>

namespace opencog {

using namespace std;

/**
 * Mainly wraps a BindLink/ImplicationLink but with other important attributes
 */
class Rule : public boost::less_than_comparable<Rule>,
             public boost::equality_comparable<Rule>
{
public:
	Rule(Handle rule);
	virtual ~Rule();

	// Comparison
	bool operator==(const Rule& r) const {
		return r.rule_handle_ == rule_handle_;
	}
	bool operator<(const Rule& r) const {
		return cost_ < r.cost_;
	}

	// Modifiers
	void set_handle(Handle h) throw (InvalidParamException);
	void set_name(const string& name);
	void set_category(const string& name);
	void set_cost(int p);	

	// Access
	string& get_name();
	const string& get_name() const;
	string& get_category();
	const string& get_category() const;
	Handle get_handle();
	Handle get_vardecl();
	Handle get_implicant();
	HandleSeq get_implicand();
	int get_cost();

	Rule gen_standardize_apart(AtomSpace* as);

private:
	Handle rule_handle_;
	string name_;
	string category_;

	// TODO weird here it is called cost, but it seems it's actually a
	// priority
	int cost_;

	Handle standardize_helper(AtomSpace* as, Handle, std::map<Handle, Handle>&);
};

} // ~namespace opencog

#endif /* RULE_H_ */
