/*************************************************************************/
/* Copyright (c) 2013 Linas Vepstas <linasvepstas@gmail.com>             */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_VITERBI_COMPRESS_H
#define _LG_VITERBI_COMPRESS_H

#include "atom.h"
#include "compile.h"

namespace link_grammar {
namespace viterbi {

Set* compress_alternatives(Set*);

} // namespace viterbi
} // namespace link-grammar

#endif // _LG_VITERBI_COMPRESS_H
