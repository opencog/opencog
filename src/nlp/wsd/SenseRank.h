/*
 * SenseRank.h
 *
 * Implements page-rank centrality algorithm for choosing word-sense 
 * liklihood.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SENSE_RANK_H
#define OPENCOG_SENSE_RANK_H

namespace opencog {

class SenseRank
{
	private:
		double damping_factor;
		bool rank_parse(Handle);

	public:
		SenseRank(void);
		~SenseRank();
		void iterate(Handle);

};
}

#endif /* OPENCOG_SENSE_RANK_H */
