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

		bool count_word(Handle);
		bool count_sense(Handle, Handle);
		double normalization;
		double sense_count;
		bool renorm_parse(Handle);
		bool renorm_word(Handle);
		bool renorm_sense(Handle, Handle);

	public:
		ReportRank(void);
		~ReportRank();
		void report_sentence(Handle);
		void report_parse(Handle);

};
}

#endif /* OPENCOG_REPORT_RANK_H */
