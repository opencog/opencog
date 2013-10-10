/*
 * ReportRank.h
 *
 * Implements page-rank centrality algorithm for choosing word-sense 
 * liklihood.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_REPORT_RANK_H
#define _OPENCOG_REPORT_RANK_H

#include <deque>

#include <opencog/atomspace/Handle.h>

namespace opencog {

class AtomSpace;

class ReportRank
{
	private:
		int parse_cnt;
		bool report_parse_f(Handle);

		bool count_word(Handle);
		bool count_sense(Handle, Handle);
		int word_count;
		double normalization;
		double sense_count;
		double choosen_sense_count;
		bool renorm_parse(Handle);
		bool renorm_word(Handle);
		bool renorm_sense(Handle, Handle);

		double hi_score;
		const char *hi_sense;

        AtomSpace *as;

	public:
		ReportRank(void);
		~ReportRank();
        void set_atom_space(AtomSpace *_as) { as=_as; };
		void report_sentence(Handle);
		void report_parse(Handle);
		void report_document(const std::deque<Handle> &);

};

} // namespace opencog

#endif // _OPENCOG_REPORT_RANK_H
