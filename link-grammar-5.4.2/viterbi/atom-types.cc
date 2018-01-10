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

#include "atom-types.h"

namespace atombase {

const std::string type_name(AtomType t)
{
	switch(t)
	{
		// Generic node types
		case NODE:       return "NODE";
		case INDEX:      return "INDEX";
		case LABEL:      return "LABEL";
		case NUMBER:     return "NUMBER";

		// Viterbi-specific node types
		case WORD:       return "WORD";
		case LING_TYPE:  return "LING_TYPE";
		case CONNECTOR:  return "CONNECTOR";

		// Generic link types
		case LINK:       return "LINK";
		case RELATION:   return "RELATION";
		case SEQ:        return "SEQ";
		case SET:        return "SET";
		case UNIQ:       return "UNIQ";
		case OR:         return "OR";
		case AND:        return "AND";

		// Viterbi-specific link types
		case WORD_CSET:  return "WORD_CSET";
		case WORD_DISJ:  return "WORD_DISJ";
		case LING:       return "LING";
		case STATE_TRIPLE: return "STATE_TRIPLE";
		case RULE:       return "RULE";
	}

	return "UNHANDLED_TYPE_NAME";
}

std::ostream& operator<<(std::ostream& out, AtomType t)
{
	out << type_name(t);
	return out;
}

} // namespace atombase

