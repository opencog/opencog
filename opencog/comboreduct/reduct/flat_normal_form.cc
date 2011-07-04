/*
 * opencog/comboreduct/reduct/flat_normal_form.cc
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
#include "flat_normal_form.h"

namespace opencog { namespace reduct {

  //does c contain p and !p?
  bool tautology(const clause& c) {
    return (std::adjacent_find(c.begin(),c.end(),
			       bind(std::equal_to<int>(),_1,
				    bind(std::negate<int>(),_2)))!=c.end());
  }
  //is c1 a subset of (or equal to) c2?
  bool subset_eq(const clause& c1,const clause& c2) {
    return (c2.size()>=c1.size() && 
	    std::includes(c2.begin(),c2.end(),
			  c1.begin(),c1.end(),c2.key_comp()));
  }
  bool subset(const clause& c1,const clause& c2) {
    return (c2.size()>c1.size() && 
	    std::includes(c2.begin(),c2.end(),
			  c1.begin(),c1.end(),c2.key_comp()));
  }
  int number_of_literals(const nf& f) {
    return std::accumulate(f.begin(),f.end(),0,
			   bind(std::plus<int>(),_1,bind(&clause::size,_2)));
  }

} // ~namespace reduct
} // ~namespace opencog

std::ostream& operator<<(std::ostream& out,const opencog::reduct::clause& c) {
    out << "(";
    if (!c.empty()) {
        out << *c.begin();
        for (opencog::reduct::clause::iterator i1=++c.begin();
             i1!=c.end();++i1)
            out << " " << *i1;
    }
    out << ")";
    return out;
}

std::ostream& operator<<(std::ostream& out,const opencog::reduct::nf& d) {
    for (opencog::reduct::nf::const_iterator c=d.begin();c!=d.end();++c)
        out << *c;
    return out;
}

