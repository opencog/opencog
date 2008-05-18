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
		double edge_sum;
		double rank_sum;

		bool rank_parse(Handle);
		bool start_word(Handle);
		bool start_sense(Handle, Handle);
		void rank_sense(Handle);
		bool outer_sum(Handle, Handle);
		bool inner_sum(Handle, Handle);
		void rand_walk(Handle);

	public:
		SenseRank(void);
		~SenseRank();
		void iterate(Handle);

};
}

#endif /* OPENCOG_SENSE_RANK_H */
