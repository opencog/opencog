#ifndef _MOSES_REPRESENTATION_H
#define _MOSES_REPRESENTATION_H

#include "comboreduct/reduct/reduct.h"
#include "comboreduct/combo/type_tree.h"

#include "moses/using.h"
#include "moses/knob_mapper.h"
#include <boost/utility.hpp>


namespace moses {

  struct representation : public knob_mapper,boost::noncopyable {
    typedef eda::instance instance;
    
    typedef std::set<combo::vertex> operator_set;
    typedef std::set<combo::combo_tree, opencog::size_tree_order<combo::vertex> > 
     	combo_tree_ns_set;


    // Optional arguments are used only for actions/Petbrain
    representation(const reduct::rule& simplify,
                   const combo_tree& exemplar_,
		   const combo::type_tree& t, 
                   opencog::RandGen& rng,
                   const operator_set* os=NULL,
                   const combo_tree_ns_set* perceptions=NULL,
                   const combo_tree_ns_set* actions=NULL);


    void transform(const instance&);
    void clear_exemplar();
    void get_clean_exemplar(combo_tree& result); // PJ & Moshe Looks

    const field_set& fields() const { return _fields; }

    const combo_tree& exemplar() const { return _exemplar; } 
    
  protected:
    combo_tree _exemplar;
    field_set _fields;
    opencog::RandGen& rng;
    const reduct::rule* _simplify;
  };

} //~namespace moses

#endif
