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
		bool init_word(Handle);
		bool init_senses(Handle, Handle);

		bool rank_parse_f(Handle);
		bool start_word(Handle);
		bool start_sense(Handle, Handle);

		double damping_factor;
		double rank_sum;
		void rank_sense(Handle);
		bool outer_sum(Handle, Handle);

		double edge_sum;
		bool inner_sum(Handle, Handle);

		double randy;
		Handle pick_random_edge(Handle);
		bool random_sum(Handle, Handle);
		Handle next_sense;

		double converge;
		double convergence_damper;
		double convergence_limit;

	public:
		SenseRank(void);
		~SenseRank();
		void rank_sentence(Handle);
		void rank_parse(Handle);

};
}

#endif /* OPENCOG_SENSE_RANK_H */
