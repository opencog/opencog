#include "ComboReduct/reduct/flat_normal_form.h"

namespace reduct {

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

} //~namespace reduct

std::ostream& operator<<(std::ostream& out,const reduct::clause& c) {
  out << "(";
  if (!c.empty()) {
    out << *c.begin();
    for (reduct::clause::iterator i1=++c.begin();
	 i1!=c.end();++i1)
      out << " " << *i1;
  }
  out << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out,const reduct::nf& d) {
  for (reduct::nf::const_iterator c=d.begin();c!=d.end();++c)
    out << *c;
  return out;
}

