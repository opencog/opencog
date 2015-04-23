/*
 * FCMemory.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
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
#include "FCMemory.h"

#include <boost/range/algorithm/find.hpp>

using namespace opencog;

FCMemory::FCMemory(AtomSpace* as)
{
    _as = as;
}

FCMemory::~FCMemory()
{
}

void FCMemory::update_premise_list(HandleSeq input)
{
    for (Handle i : input) {
        if (boost::find(_premise_list, i) == _premise_list.end())
            _premise_list.push_back(i);
    }
}

vector<Rule*>& FCMemory::get_rules()
{
    return _rules;
}
const vector<Rule*>& FCMemory::get_rules() const
{
    return _rules;
}
void FCMemory::set_rules(vector<Rule*> rules)
{
    _rules = rules;
}
void FCMemory::set_source(Handle source)
{
    _cur_source = source;
    _source_list.push_back(_cur_source);
}
HandleSeq FCMemory::get_source_list()
{
    return _source_list;
}
HandleSeq FCMemory::get_premise_list()
{
    return _premise_list;
}
void FCMemory::set_search_in_af(bool val)
{
    _search_in_af = val;
}
bool FCMemory::is_search_in_af()
{
    return _search_in_af;
}
Rule* FCMemory::get_cur_rule()
{
    return _cur_rule;
}
void FCMemory::set_cur_rule(Rule* r)
{
    _cur_rule = r;
}
void FCMemory::add_rules_product(int iteration, HandleSeq product)
{
    for (Handle p : product) {
        Inference inf;
        inf.iter_step = iteration;
        inf.applied_rule = _cur_rule;
        inf.inf_product.push_back(p);
        _inf_history.push_back(inf);
    }
}
void FCMemory::add_inference(int iter_step, HandleSeq product,
                             HandleSeq matched_nodes)
{
    Inference inf;
    inf.applied_rule = _cur_rule;
    inf.iter_step = iter_step;
    for (Handle p : product)
        inf.inf_product.push_back(p);
    for (Handle mn : matched_nodes)
        inf.matched_nodes.push_back(mn);
    _inf_history.push_back(inf);
}
Handle FCMemory::get_cur_source()
{
    return _cur_source;
}
bool FCMemory::isin_source_list(Handle h)
{
    return (boost::find(_source_list, h) != _source_list.end());
}
bool FCMemory::isin_premise_list(Handle h)
{
    for (Handle hi : _premise_list) {
        if (hi.value() == h.value())
            return true;
        //recursive lookup
        else if (LinkCast(hi)) {
            HandleSeqSeq hseqs;
            hseqs.push_back(_as->getOutgoing(hi));
            do {
                HandleSeq iset = hseqs[hseqs.size() - 1];
                hseqs.pop_back();
                for (Handle i : iset) {
                    if (i.value() == h.value())
                        return true;
                    else if (LinkCast(i))
                        hseqs.push_back(_as->getOutgoing(i));
                }
            } while (not hseqs.empty());
        }
    }
    return false;
}

HandleSeq FCMemory::get_result()
{
    HandleSeq result;
    for (Inference i : _inf_history)
        result.insert(result.end(), i.inf_product.begin(), i.inf_product.end());
    return result;
}

vector<Inference>& FCMemory::get_inf_history()
{
    return _inf_history;
}

vector<Rule*> FCMemory::get_applied_rules()
{
    vector<Rule*> applied_rules;
    for (Inference i : _inf_history) {
        if (boost::find(applied_rules, i.applied_rule) == applied_rules.end())
            applied_rules.push_back(i.applied_rule);
    }
    return applied_rules;
}
