/*************************************************************************/
/* Copyright (c) 2012 Linas Vepstas <linasvepstas@gmail.com>             */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_VITERBI_DISJOIN_H
#define _LG_VITERBI_DISJOIN_H

#include "atom.h"

namespace link_grammar {
namespace viterbi {

/**
 * Convert dictionary-normal form into disjunctive normal form.
 * That is, convert the mixed-form dictionary entries into a disjunction
 * of a list of conjoined connectors.  The goal of this conversion is to
 * simplify the parsing algorithm.
 */
Atom* disjoin(Atom* mixed_form);


} // namespace viterbi
} // namespace link-grammar

#endif /* _LG_VITERBI_DISJOIN_H */
