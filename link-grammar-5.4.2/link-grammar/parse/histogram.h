/*************************************************************************/
/* Copyright (c) 2015 Linas Vepstas                                      */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _HISTOGRAM_H_
#define _HISTOGRAM_H_

#ifndef _MSC_VER
typedef long long s64; /* signed 64-bit integer, even on 32-bit cpus */
#define PARSE_NUM_OVERFLOW (1LL<<24)
#else
/* Microsoft Visual C Version 6 doesn't support long long. */
typedef signed __int64 s64; /* signed 64-bit integer, even on 32-bit cpus */
#define PARSE_NUM_OVERFLOW (((s64)1)<<24)
#endif


/*
 * Count Histogramming is currently not required for anything, and the
 * code runs about 6% faster when it is disabled.
 *
#define PERFORM_COUNT_HISTOGRAMMING 1
 */
#ifdef PERFORM_COUNT_HISTOGRAMMING

/**
 * A histogram distribution of the parse counts.
 * The histogram is with respect to the cost of the parse.  Thus, each
 * bin of the histogram contains a count of the number of parses
 * achievable with that cost.  Rather than setting the baseline cost
 * at zero, it is dynamically scaled, so that 'base' is the number of
 * the first bin with a non-zero count in it.  If there are counts that
 * don't fit into the available bins, then they are accumulated into
 * the overrun bin.  It is always the case that
 *     total == sum_i bin[i] + overrun
 */
#define NUM_BINS 12
struct Count_bin_s
{
	short base;
	s64 total;
	s64 bin[NUM_BINS];
	s64 overrun;
};

typedef struct Count_bin_s Count_bin;

Count_bin hist_zero(void);
Count_bin hist_one(void);

void hist_accum(Count_bin* sum, double, const Count_bin*);
void hist_accumv(Count_bin* sum, double, const Count_bin);
void hist_prod(Count_bin* prod, const Count_bin*, const Count_bin*);
void hist_muladd(Count_bin* prod, const Count_bin*, double, const Count_bin*);
void hist_muladdv(Count_bin* prod, const Count_bin*, double, const Count_bin);

static inline s64 hist_total(Count_bin* tot) { return tot->total; }
s64 hist_cut_total(Count_bin* tot, int min_total);

double hist_cost_cutoff(Count_bin*, int count);

#else

typedef s64 Count_bin;

static inline Count_bin hist_zero(void) { return 0; }
static inline Count_bin hist_one(void) { return 1; }

static inline void hist_accum(Count_bin* sum, double cost, Count_bin* a)
	{ *sum += *a; }
static inline void hist_accumv(Count_bin* sum, double cost, Count_bin a)
	{ *sum += a; }
static inline void hist_prod(Count_bin* prod, Count_bin* a, Count_bin* b)
	{ *prod = (*a) * (*b); }
static inline void hist_muladd(Count_bin* prod, Count_bin* a, double cost, Count_bin* b)
	{ *prod += (*a) * (*b); }
static inline void hist_muladdv(Count_bin* prod, Count_bin* a, double cost, Count_bin b)
	{ *prod += (*a) * b; }

static inline s64 hist_total(Count_bin* tot) { return *tot; }
static inline s64 hist_cut_total(Count_bin* tot, int min_total) { return *tot; }

static inline double hist_cost_cutoff(Count_bin* tot, int count) { return 1.0e38; }

#endif /* PERFORM_COUNT_HISTOGRAMMING */

#endif /* _HISTOGRAM_H_ */
