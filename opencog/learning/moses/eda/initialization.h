#ifndef _EDA_INITIALIZATION_H
#define _EDA_INITIALIZATION_H

#include "eda/field_set.h"
#include "util/RandGen.h"

//various routines for initializing instances
namespace eda {
  
  using namespace opencog;

  //occam randomize a particular contin or onto field
  void occam_randomize(const field_set&,instance&,
		       field_set::const_contin_iterator, 
		       RandGen& rng);
  void occam_randomize(const field_set&,instance&,
		       field_set::const_onto_iterator,
		       RandGen& rng);

  //occam randomize all contin or onto fields
  void occam_randomize_onto(const field_set&,instance&,
		       RandGen& rng);
  void occam_randomize_contin(const field_set&,instance&,
		       RandGen& rng);

  //uniformly randomize all bit or disc fields
  void uniform_randomize_bits(const field_set&,instance&,
		       RandGen& rng);
  void uniform_randomize_disc(const field_set&,instance&,
		       RandGen& rng);

  //occam randomize all contin and onto fields, and uniformly randomize all bit
  //and disc fields
  void randomize(const field_set&,instance&,
		       RandGen& rng);

} //~namespace eda

#endif
