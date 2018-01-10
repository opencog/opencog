/*************************************************************************/
/* Copyright (c) 2012, 2013 Linas Vepstas <linasvepstas@gmail.com>       */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _LG_VITERBI_CONNECT_H
#define _LG_VITERBI_CONNECT_H

#include "atom.h"
#include "compile.h"
#include "garbage.h"

namespace link_grammar {
namespace viterbi {

class Connect : public gc
{
	public:
		Connect(WordCset*, WordCset*);
		StateTriple* try_alternative(Atom*, Atom*);

	protected:

		StateTriple* alternative(Connector*, Connector*);
		StateTriple* alternative(Connector*, And*);
		StateTriple* alternative(And*, Connector*);
		StateTriple* alternative(And*, And*);

		Ling* conn_connect_nn(Connector*, Connector*);
		Ling* reassemble(Ling*, WordCset*, WordCset*);

		static const OutList& flatten(OutList&);

	private:
		WordCset* _left_cset;
		WordCset* _right_cset;
};


} // namespace viterbi
} // namespace link-grammar

#endif // _LG_VITERBI_CONNECT_H
