#ifndef _MOSES_KNOB_MAPPER_H
#define _MOSES_KNOB_MAPPER_H

#include <map>
#include "eda/field_set.h"
#include "moses/knobs.h"

namespace moses {
  struct knob_mapper {
    typedef eda::field_set field_set;

    //important: knobs are kept sorted in an order consistant with that of the
    //field_set _fields that is constructed according to their corresponding specs
    typedef std::multimap<field_set::disc_spec,disc_knob> disc_map;
    typedef disc_map::value_type disc_v;
    typedef std::multimap<field_set::contin_spec,contin_knob> contin_map;
    typedef contin_map::value_type contin_v;

    disc_map disc;
    contin_map contin;
  };

} //~namespace moses

#endif
