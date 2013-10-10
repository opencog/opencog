/*
 * SenseRank.h
 *
 * Implements page-rank centrality algorithm for choosing word-sense 
 * liklihood.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_SENSE_RANK_H
#define _OPENCOG_SENSE_RANK_H

#include <opencog/atomspace/Handle.h>

#include <deque>

namespace opencog {

class AtomSpace;

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
        AtomSpace *as;

        void log_bad_sense(Handle, const std::string&, bool);

	public:
		SenseRank();
		~SenseRank();
        void set_atom_space(AtomSpace *_as) {as = _as;}
		void init_parse(Handle);
		void rank_parse(Handle);
		void rank_sentence(Handle);
		void rank_document(const std::deque<Handle> &);

};

} // namespace opencog

#endif // _OPENCOG_SENSE_RANK_H
