/*
 * Rule.cc
 *
 * Copyright (C) 2015 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com> 2015
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

#include "Rule.h"

Rule::Rule(Handle rule)
{
	rule_handle_ = rule;
}


Rule::~Rule()
{

}

int Rule::get_cost()
{
	return cost_;
}

void Rule::set_category(string name)
{
	category_ = name;
}

string& Rule::get_category()
{
	return category_;
}

void Rule::set_name(string name)
{
	name_ = name;
}

string Rule::get_name()
{
	return name_;
}

void Rule::set_rule_handle(Handle h) throw (exception)
{
	rule_handle_ = h;
}

Handle Rule::get_handle()
{
	return rule_handle_;
}

void Rule::set_cost(int p)
{
	cost_ = p;
}

void Rule::add_disjunct_rule(Rule* r)
{
	disjunct_rules_.push_back(r);
}

vector<Rule*> Rule::get_disjunct_rules(void)
{
	return disjunct_rules_;
}
