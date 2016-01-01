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

#include <deque>

#include <opencog/atoms/base/Handle.h>

namespace opencog {

class AtomSpace;

class SenseRank
{
	private:
		bool init_word(const Handle&);
		bool init_senses(const Handle&, const Handle&);

		bool rank_parse_f(const Handle&);
		bool start_word(const Handle&);
		bool start_sense(const Handle&, const Handle&);

		double damping_factor;
		double rank_sum;
		void rank_sense(const Handle&);
		bool outer_sum(const Handle&, const Handle&);

		double edge_sum;
		bool inner_sum(const Handle&, const Handle&);

		double randy;
		Handle pick_random_edge(const Handle&);
		bool random_sum(const Handle&, const Handle&);
		Handle next_sense;

		double converge;
		double convergence_damper;
		double convergence_limit;

		void log_bad_sense(const Handle&, const std::string&, bool);

	public:
		SenseRank();
		~SenseRank();
		void init_parse(const Handle&);
		void rank_parse(const Handle&);
		void rank_sentence(const Handle&);
		void rank_document(const std::deque<Handle> &);

};

} // namespace opencog

#endif // _OPENCOG_SENSE_RANK_H
