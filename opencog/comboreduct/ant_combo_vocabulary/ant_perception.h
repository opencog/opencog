#ifndef _ANT_PERCEPTION_H
#define _ANT_PERCEPTION_H

#include <LADSUtil/numeric.h>

#include "ComboReduct/combo/perception.h"
#include "ComboReduct/ant_combo_vocabulary/ant_operator.h"

namespace combo {
  
  //later to be replaced by id
  namespace id {
    enum ant_perception_enum {
      is_food_ahead,

      ant_perception_count
    };
  }

  typedef id::ant_perception_enum ant_perception_enum;

  /*********************************************************************
   *       Arrays containing perception name type and properties       *
   *                 to be edited by the developer                     *
   *********************************************************************/

  namespace ant_perception_properties {

    //struct for description of name and type
    typedef combo::ant_operator<ant_perception_enum, id::ant_perception_count>::basic_description perception_basic_description;

    //struct for decription of perception properties
    struct perception_property_description {
      ant_perception_enum perception;
      bool ultrametric;
      bool transitive;
      bool irreflexive;
      bool reflexive;
      bool symmetric;
      bool identity_of_indiscernibles;
    };
    
    static const perception_basic_description pbd[] = {
      //perception             name                 type
      { id::is_food_ahead,     "is_food_ahead",     "boolean" }
    };
    
    static const perception_property_description ppd[] = {
      //perception             ultrametric transitive irreflexive reflexive symmetric identity_of_indiscernibles
      { id::is_food_ahead,     false,      false,     false,      false,    false,    false }
    };
    
    
  }//~namespace ant_perception_properties

  class ant_perception : public ant_operator<ant_perception_enum, id::ant_perception_count>, public perception_base {

  private:

    //standard properties
    bool _ultrametric;
    bool _transitive;
    bool _irreflexive;
    bool _reflexive;
    bool _symmetric;
    bool _identity_of_indiscernibles;

    //private methods

    //ctor
    ant_perception();

    const basic_description * get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const ant_perception* init_perceptions();
    void set_perception(ant_perception_enum);

  public:
    //return a pointer of the static ant_perception corresponding
    //to a given name string
    //if no such ant_perception exists then return NULL pointer
    static const ant_perception* instance(const std::string& name);

    //return a pointer of the static ant_perception corresponding
    //to a given ant_perception_enum
    static const ant_perception* instance(ant_perception_enum);

    //basic access methods
    const std::string& get_name() const;
    const type_tree& get_type_tree() const;
    arity_t arity() const;
    type_tree get_output_type_tree() const;

    //return the type tree of the argument of index i
    //if the operator has arg_list(T) as last input argument
    //then it returns always T past that index
    const type_tree& get_input_type_tree(arity_t i) const;

    //action property methods
    bool is_ultrametric() const;
    bool is_transitive() const;
    bool is_irreflexive() const;
    bool is_reflexive() const;
    bool is_symmetric() const;
    bool is_identity_of_indiscernibles() const;
  };
}

#endif
