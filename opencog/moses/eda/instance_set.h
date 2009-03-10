#ifndef _EDA_INSTANCE_SET_H
#define _EDA_INSTANCE_SET_H

#include "eda/field_set.h"
#include "eda/scoring.h"

namespace eda {

  template<typename ScoreT>
  struct instance_set : public vector<scored_instance<ScoreT> > {
    typedef vector<scored_instance<ScoreT> > super;
    typedef boost::transform_iterator<opencog::select_tag,
				      typename super::iterator,
				      ScoreT&,
				      ScoreT&> score_iterator;
    typedef boost::transform_iterator<opencog::select_tag,
				      typename super::const_iterator,
				      const ScoreT&,
				      const ScoreT&> const_score_iterator;
    typedef boost::transform_iterator<opencog::select_item,
				      typename super::iterator,
				      instance&,
				      instance&> instance_iterator;
    typedef boost::transform_iterator<opencog::select_item,
				      typename super::const_iterator,
				      const instance&,
				      const instance&> const_instance_iterator;

    instance_set(int n,const field_set& fs)
      : super(n,instance(fs.packed_width())),_fields(fs) { }
    instance_set(const field_set& fs) : _fields(fs) { }

    void resize(int n) { super::resize(n,instance(_fields.packed_width())); }
    
    const field_set& fields() const { return _fields; }

    score_iterator begin_scores() { return score_iterator(this->begin()); }
    score_iterator end_scores() { return score_iterator(this->end()); }
    /*const_score_iterator begin_scores() const { 
      return const_score_iterator(begin()); 
      }*/

    //instance_iterator begin_instances() { return instance_iterator(begin()); }
    //instance_iterator end_instances() { return instance_iterator(end()); }
    const_instance_iterator begin_instances() const { 
      return const_instance_iterator(this->begin()); 
    }
    const_instance_iterator end_instances() const { 
      return const_instance_iterator(this->end()); 
    }
  protected:
    field_set _fields;
  };

} //~namespace eda

#endif
