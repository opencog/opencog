#ifndef _UTIL_DIGRAPH_H
#define _UTIL_DIGRAPH_H

#include <queue>
#include <vector>
#include <set>
#include "foreach.h"
#include "algorithm.h"
#include "exceptions.h"
#include <boost/iterator/counting_iterator.hpp>

namespace Util {

  struct digraph {
    typedef unsigned int size_type;
    typedef size_type value_type;
    typedef std::set<value_type> value_set;

    digraph(size_type n) : _incoming(n),_outgoing(n) { }

    void insert(value_type src,value_type dst) {
      _incoming[dst].insert(src);
      _outgoing[src].insert(dst);
    }
    void erase(value_type src,value_type dst) {
      _incoming[dst].erase(src);
      _outgoing[src].erase(dst);
    }
    size_type n_nodes() const { return _incoming.size(); }
    size_type n_edges() const { 
      return accumulate2d(_incoming.begin(),_incoming.end(),size_type(0));
    }
    bool empty() const { return (n_edges()==0); }
    
    const value_set& incoming(value_type x) const { 
      return _incoming[x];
    }
    const value_set& outgoing(value_type x) const { 
      return _outgoing[x];
    }
  protected:
    std::vector<value_set> _incoming;
    std::vector<value_set> _outgoing;
  };

  //digraph must be a dag
  template<typename Out>
  Out randomized_topological_sort(digraph g,Out out) {
    typedef digraph::value_type value_t;
    std::vector<value_t> nodes
      (boost::make_counting_iterator(digraph::size_type(0)),
       boost::make_counting_iterator(g.n_nodes()));
    std::random_shuffle(nodes.begin(),nodes.end());
    std::queue<value_t> q;

    foreach (value_t node,nodes)
      if (g.incoming(node).empty())
	q.push(node);

    while (!q.empty()) {
      value_t src=q.front();
      q.pop();

      *out++=src;

      std::vector<value_t> outgoing(g.outgoing(src).begin(),
				    g.outgoing(src).end());
      foreach (value_t dst,outgoing) {
	g.erase(src,dst);
	if (g.incoming(dst).empty())
	  q.push(dst);
      }
    }
    cassert(TRACE_INFO, g.empty(), "digraph - g must be a DAG."); //must be a dag
    return out;
  }

} //~namespace Util

#if 0
    const std::vector<value_seq >& incoming() const {
      return _incoming;
    }
    const std::vector<value_seq >& outgoing() const {
      return _outgoing
    }

  dag::value_seq::iterator toremove=
	std::remove_if(g.outgoing(src).begin(),g.outgoing(src).end(),
		       bind(&dag::value_seq::size,
			    ref(bind(&dag::value_seq::incoming,_1))));
            q.insert(q.begin(),toremove,g.outgoing(n).end());
      for_each(q.begin(),q.begin()+distance(toremove,g.outgoing(n).end()),
	       bind(&dag::erase,src,_1));
#endif

#endif
