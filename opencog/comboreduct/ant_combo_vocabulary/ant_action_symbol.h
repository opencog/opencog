#ifndef _ANT_ACTION_SYMBOL_H
#define _ANT_ACTION_SYMBOL_H

#include <LADSUtil/numeric.h>

#include "ComboReduct/combo/action_symbol.h"
#include "ComboReduct/ant_combo_vocabulary/ant_operator.h"

namespace combo {

  //this set is empty but the code is given as example
  namespace id {
    enum ant_action_symbol_enum { 
      ant_action_symbol_count
    };
  }

  typedef id::ant_action_symbol_enum ant_action_symbol_enum;

  /*********************************************************************
   *      Arrays containing action_symbol name type and properties     *
   *                 to be edited by the developer                     *
   *********************************************************************/

  namespace ant_action_symbol_properties {

    //struct for description of name and type
    typedef combo::ant_operator<ant_action_symbol_enum, id::ant_action_symbol_count>::basic_description action_symbol_basic_description;

    static const action_symbol_basic_description asbd[] = {
      //action_symbol          name                 type
    };
        
  }//~namespace pet_perception_properties

  //pet_action_symbol both derive from action_symbol_base and pet_operator
  class ant_action_symbol : public ant_operator<ant_action_symbol_enum, id::ant_action_symbol_count>, public action_symbol_base {

  private:

    //private methods

    //ctor
    ant_action_symbol();

    const basic_description * get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const ant_action_symbol* init_action_symbol();
    void set_action_symbol(ant_action_symbol_enum);

  public:
    //name
    const std::string& get_name() const;

    //type_tree
    const type_tree& get_type_tree() const;

    //helper methods for fast access type properties
    //number of arguments that takes the operator
    arity_t arity() const;
    //return the type node of the operator
    type_tree get_output_type_tree() const;

    //return the type tree of the input argument of index i
    //if the operator has arg_list(T) as last input argument
    //then it returns always T past that index
    const type_tree& get_input_type_tree(arity_t index) const;

    //return a pointer of the static action_symbol corresponding
    //to a given name string
    //if no such action_symbol exists then return NULL pointer
    static action_symbol instance(const std::string& name);

    //return a pointer of the static pet_perception_action corresponding
    //to a given pet_perception_enum
    static action_symbol instance(ant_action_symbol_enum);

  };
}//~namespace combo

#endif
