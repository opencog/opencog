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

#ifndef _LG_CONNECTOR_UTILS_H
#define _LG_CONNECTOR_UTILS_H

#include "atom.h"
#include "compile.h"

namespace link_grammar {
namespace viterbi {

bool conn_match(const NameString&, const NameString&);
NameString conn_merge(const NameString&, const NameString&);
bool is_optional(Atom *);

WordCset* cset_trim_left_pointers(WordCset*);

} // namespace viterbi
} // namespace link-grammar

#endif // _LG_CONNECTOR_UTILS_H
