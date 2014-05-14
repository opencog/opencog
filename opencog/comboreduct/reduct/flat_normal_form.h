/*
 * opencog/comboreduct/reduct/flat_normal_form.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _REDUCT_FLAT_NORMAL_FORM_H
#define _REDUCT_FLAT_NORMAL_FORM_H

#include <set>
#include <list>
#include <numeric>
#include <map>
#include <unordered_map>
#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/numeric.h>
#include <opencog/util/tree.h>

#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace reduct {
typedef std::set<int, opencog::absolute_value_order<int> > clause;
typedef std::list<clause> nf;
using namespace opencog::combo;

//does c contain p and !p?
bool tautology(const clause& c);
//is c1 a subset of (or equal to) c2?
bool subset_eq(const clause& c1,const clause& c2);
bool subset(const clause& c1,const clause& c2);
int number_of_literals(const nf& f);

//(p X)(!p Y) implies (X Y); this function finds all such matches
template<typename Out>
void implications(const clause& c1,const clause& c2,Out out)
{
    for (clause::const_iterator it1=c1.begin();it1!=c1.end();++it1) {
        for (clause::const_iterator it2=c2.begin();it2!=c2.end();++it2) {
            if (*it1==-*it2) {
                clause c;
                std::merge(c1.begin(),c1.end(),c2.begin(),c2.end(),
                           inserter(c,c.begin()),c1.key_comp());
                c.erase(*it1);
                c.erase(*it2);
                if (!tautology(c))
                    (*out++)=c;
            }
        }
    }
}

template<typename BinaryPredicate>
void pairwise_erase_if(nf& f,const BinaryPredicate& p)
{
    nf::iterator c1=f.begin();
    while (c1!=f.end()) {
        nf::iterator next1=c1,c2=f.begin();
        ++next1;
        while (c2!=c1) {
            nf::iterator next2=c2;
            ++next2;
            if (c1!=c2) {
                if (p(*c1,*c2)) {
                    f.erase(c2);
                } else if (p(*c2,*c1)) {
                    f.erase(c1);
                    break;
                }
            }
            c2=next2;
        }
        c1=next1;
    }
}

template<typename T>
class nf_mapper
{
public:
    typedef opencog::tree<T> tree;
    typedef typename tree::sibling_iterator sib_it;
    typedef std::map<sib_it,int,
                     opencog::lexicographic_subtree_order<T> > Item2Int;
    typedef std::unordered_map<int,tree> Int2Item;

    nf add_cnf(sib_it, sib_it);
    nf add_dnf(sib_it, sib_it);

    template<typename It>
    void extract_cnf(It, It, tree&, sib_it) const;
    template<typename It>
    void extract_dnf(It, It, tree&, sib_it) const;

    void extract_conjunction(const clause&, tree&, sib_it) const;
protected:
    Item2Int _item2int;
    Int2Item _int2item;

    int add_item(sib_it);

    void create(tree&, sib_it, int) const;
};

template<typename T>
nf nf_mapper<T>::add_cnf(sib_it from,sib_it to)
{
    nf res(std::distance(from, to));
    for (nf::iterator out = res.begin(); out != res.end(); ++from, ++out)
        if (*from == id::logical_or)
            for (sib_it item = from.begin(); item != from.end(); ++item)
                out->insert(add_item(item));
        else
            out->insert(add_item(from));
    return res;
}

template<typename T>
nf nf_mapper<T>::add_dnf(sib_it from, sib_it to)
{
    nf res(std::distance(from, to));
    for (nf::iterator out = res.begin(); out != res.end(); ++from, ++out)
        if (*from == id::logical_and)
            for (sib_it item = from.begin(); item != from.end(); ++item)
                out->insert(add_item(item));
        else 
            out->insert(add_item(from));
    return res;
}

template<typename T>
template<typename It>
void nf_mapper<T>::extract_cnf(It from,It to,tree& t,sib_it sib) const
{
    t.erase_children(sib);
    for (nf::const_iterator c=from;c!=to;++c)
        if (c->size()==1) {
            if (*c->begin()!=0)
                create(t,sib,*c->begin());
        } else if (!c->empty()) {
            sib_it clause=t.append_child(sib,id::logical_or);
            for (clause::const_iterator item=c->begin();item!=c->end();++item) {
                create(t,clause,*item);
            }
        }
    if (sib.has_one_child()) {
        *sib=*sib.begin();
        t.erase(t.flatten(sib.begin()));
    } else {
        *sib=sib.is_childless() ? id::logical_false : id::logical_and;
    }
}


template<typename T>
template<typename It>
void nf_mapper<T>::extract_dnf(It from, It to, tree& t, sib_it sib) const
{
    t.erase_children(sib);
    for (nf::const_iterator c = from; c != to; ++c) {
        if (c->empty()) {
            t.erase_children(sib);
            *sib = id::logical_true;
            return;
        } else if (c->size() == 1) {
            if (*c->begin() != 0)
                create(t, sib, *c->begin());
        } else {
            sib_it clause = t.append_child(sib, id::logical_and);
            for (clause::const_iterator item = c->begin(); item != c->end(); ++item) {
                create(t, clause, *item);
            }
        }
    }
    if (sib.has_one_child()) {
        *sib = *sib.begin();
        t.erase(t.flatten(sib.begin()));
    } else {
        *sib = sib.is_childless() ? id::logical_false : id::logical_or;
    }
}

template<typename T>
void nf_mapper<T>::extract_conjunction(const clause& c,
                                       tree& t,sib_it sib) const
{
    if (c.size()==1) {
        if (*c.begin()!=0) {
            if (*c.begin()<0) {
                typename tree::pre_order_iterator tmp=
                    _int2item.find(-*c.begin())->second.begin();
                OC_ASSERT(is_argument(*tmp), "Tree node referenced by iterator isn't an argument.");
                *sib=*tmp;
                get_argument(*sib).negate();
            } else {
                typename tree::pre_order_iterator tmp=
                    _int2item.find(*c.begin())->second.begin();
                *sib=*tmp;
                t.append_children(sib,tmp.begin(),tmp.end());
            }
        }
    } else {
        *sib=id::logical_and;
        for (clause::const_iterator item=c.begin();item!=c.end();++item) {
            create(t,sib,*item);
        }
    }
}

template<typename T>
int nf_mapper<T>::add_item(sib_it item)
{
    typename Item2Int::const_iterator item_loc = _item2int.find(item);

    // If found, return the index.
    if (item_loc != _item2int.end())
        return item_loc->second;


    // Perhaps its under minus-the-index?
    if (argument* arg = boost::get<argument>(&*item)) {
        arg->negate();
        item_loc = _item2int.find(item);
        arg->negate();
        if (item_loc != _item2int.end()) {
            return -item_loc->second;
        }
    }

    // Add it directly
    int next_id = _int2item.size() + 1;

    std::pair<int, tree> pr = std::make_pair(next_id, tree(item));

    int actual_id = _int2item.insert(pr).first->first;

    return _item2int.insert(std::make_pair(item, actual_id)).first->second;
}

template<typename T>
void nf_mapper<T>::create(tree& t, sib_it at, int idx) const
{
    if (idx < 0) {
        get_argument(*t.append_child
                     (typename tree::pre_order_iterator(at),
                      _int2item.find(-idx)->second.begin())).negate();

        /*t.append_child(typename tree::pre_order_iterator
          (t.append_child(at,id::logical_not)),
          _int2item.find(-idx)->second.begin());*/
    } else {
        t.append_child(typename tree::pre_order_iterator(at),
                       _int2item.find(idx)->second.begin());
    }
}

} // ~namespace reduct
} // ~namespace opencog

std::ostream& operator<<(std::ostream& out,const opencog::reduct::clause& c);
std::ostream& operator<<(std::ostream& out,const opencog::reduct::nf& d);

#endif
