/*
 * ReportRank.h
 *
 * Implements page-rank centrality algorithm for choosing word-sense 
 * liklihood.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_REPORT_RANK_H
#define OPENCOG_REPORT_RANK_H

namespace opencog {

class ReportRank
{
	private:
		int parse_cnt;
		bool report_parse_f(Handle);
		bool report_word(Handle);
		double hi_score;
		Handle hi_scorer;
		bool choose_sense(Handle, Handle);

	public:
		ReportRank(void);
		~ReportRank();
		void report_sentence(Handle);
		void report_parse(Handle);

};
}

#endif /* OPENCOG_REPORT_RANK_H */
