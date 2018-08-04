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

#include <opencog/atoms/base/Handle.h>

namespace opencog {

class ReportRank
{
	private:
		int parse_cnt;
		bool report_parse_f(const Handle&);

		bool count_word(const Handle&);
		bool count_sense(const Handle&, const Handle&);
		int word_count;
		double normalization;
		double sense_count;
		double choosen_sense_count;
		bool renorm_parse(const Handle&);
		bool renorm_word(const Handle&);
		bool renorm_sense(const Handle&, const Handle&);

		double hi_score;
		const char *hi_sense;

	public:
		ReportRank(void);
		~ReportRank();
		void report_sentence(const Handle&);
		void report_parse(const Handle&);
		void report_document(const std::deque<Handle> &);
};

} // namespace opencog

#endif // _OPENCOG_REPORT_RANK_H
