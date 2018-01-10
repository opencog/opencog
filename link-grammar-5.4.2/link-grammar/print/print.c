/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2013, 2014 Linas Vepstas                                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include "api-structures.h"
#include "connectors.h"
#include "corpus/corpus.h"
#include "dict-common/dict-utils.h"   // For size_of_expression()
#include "disjunct-utils.h"
#include "linkage/linkage.h"
#include "print.h"
#include "tokenize/word-structures.h" // For Word_struct
#include "wcwidth.h"

#define LEFT_WALL_SUPPRESS ("Wd") /* If this connector is used on the wall, */
                                  /* then suppress the display of the wall. */
#define RIGHT_WALL_SUPPRESS ("RW")/* If this connector is used on the wall, */
                                  /* then suppress the display of the wall. */

#define HEAD_CHR ('h') /* Single char marking head-word. */
#define DEPT_CHR ('d') /* Single char marking dependent word */

/**
 * Find the position of the center of each word.
 * Also find the offset of each word, relative to the previous one,
 * needed to fully fit the names of the links between them.
 * FIXME Long link names between more distant words may still not
 * fit the space between these words.
 *
 * Return the number of characters needed for the all the words,
 * including the space needed for the link names as described above.
 */
static size_t
set_centers(const Linkage linkage, int center[], int word_offset[],
            bool print_word_0, int N_words_to_print)
{
	int i, tot;
	size_t n;
	int start_word = print_word_0 ? 0 : 1;
	int *link_len = alloca(linkage->num_words * sizeof(*link_len));
	size_t max_line_len = 0; /* Needed picture array line length */

	memset(link_len, 0, linkage->num_words * sizeof(*link_len));

	for (n = 0; n < linkage->num_links; n++)
	{
		Link *l = &linkage->link_array[n];

		if ((l->lw + 1 == l->rw) && (NULL != l->link_name))
		{
			link_len[l->rw] = strlen(l->link_name) +
				(DEPT_CHR == l->rc->string[0]) +
				(HEAD_CHR == l->rc->string[0]) +
				(DEPT_CHR == l->lc->string[0]) +
				(HEAD_CHR == l->lc->string[0]);
		}
	}

	tot = 0;
	for (i = start_word; i < N_words_to_print; i++)
	{
		int len, center_t;

		/* Centers obtained by counting the characters column widths,
		 * not the bytes in the string. */
		len = utf8_strwidth(linkage->word[i]);
		center_t = tot + (len/2);
#if 1 /* Long labels - disable in order to compare output with old versions. */
		if (i > start_word)
			center[i] = MAX(center_t, center[i-1] + link_len[i] + 1);
		else
#endif
			center[i] = center_t;
		word_offset[i] = center[i] - center_t;
		tot += len+1 + word_offset[i];
		max_line_len += word_offset[i] + strlen(linkage->word[i]) + 1;
	}

	return max_line_len;
}

/* The following are all for generating postscript */
typedef struct
{
	int N_rows; /* N_rows -- the number of rows */
	/* tells the height of the links above the sentence */
	int * link_heights;
	/* the word beginning each row of the display */
	int * row_starts;
} ps_ctxt_t;

/**
 * Prints s then prints the last |t|-|s| characters of t.
 * if s is longer than t, it truncates s.
 * Handles utf8 strings correctly.
 */
static void left_append_string(dyn_str * string, const char * s, const char * t)
{
	size_t i;
	size_t slen = utf8_strwidth(s);
	size_t tlen = utf8_strwidth(t);

	for (i = 0; i < tlen; i++)
	{
		if (i < slen)
			append_utf8_char(string, s);
		else
			append_utf8_char(string, t);

		s += utf8_next(s);
		t += utf8_next(t);
	}
}

static void print_a_link(dyn_str * s, const Linkage linkage, LinkIdx link)
{
	WordIdx l, r;
	const char *label, *llabel, *rlabel;

	l      = linkage_get_link_lword(linkage, link);
	r      = linkage_get_link_rword(linkage, link);
	label  = linkage_get_link_label(linkage, link);
	llabel = linkage_get_link_llabel(linkage, link);
	rlabel = linkage_get_link_rlabel(linkage, link);

	if (l == 0)
	{
		left_append_string(s, LEFT_WALL_DISPLAY, "               ");
	}
	else if (l == (linkage_get_num_words(linkage) - 1))
	{
		left_append_string(s, RIGHT_WALL_DISPLAY, "               ");
	}
	else
	{
		left_append_string(s, linkage_get_word(linkage, l), "               ");
	}

	/* XXX FIXME -- the space allocated to a link name should depend
	 * on the longest link-name in the sentence! */
	left_append_string(s, llabel, "           ");
	if (DEPT_CHR == llabel[0])
		dyn_strcat(s, "   <---");
	else if (HEAD_CHR == llabel[0])
		dyn_strcat(s, "   >---");
	else
		dyn_strcat(s, "   ----");

	left_append_string(s, label, "-----");
	if (DEPT_CHR == rlabel[0])
		dyn_strcat(s, "->  ");
	else if (HEAD_CHR == rlabel[0])
		dyn_strcat(s, "-<  ");
	else
		dyn_strcat(s, "--  ");
	left_append_string(s, rlabel, "           ");
	append_string(s, "     %s\n", linkage_get_word(linkage, r));
}


/**
 * To the left of each link, print the sequence of domains it is in.
 * Printing a domain means printing its type.
 * Takes info from pp_link_array and pp and chosen_words.
 */
char * linkage_print_links_and_domains(const Linkage linkage)
{
	int link, longest, j;
	int N_links = linkage_get_num_links(linkage);
	dyn_str * s = dyn_str_new();
	const char ** dname;

	longest = 0;
	for (link=0; link<N_links; link++)
	{
		// if (linkage_get_link_lword(linkage, link) == SIZE_MAX) continue;
		assert (linkage_get_link_lword(linkage, link) < SIZE_MAX);
		if (linkage_get_link_num_domains(linkage, link) > longest)
			longest = linkage_get_link_num_domains(linkage, link);
	}
	for (link=0; link<N_links; link++)
	{
		// if (linkage_get_link_lword(linkage, link) == SIZE_MAX) continue;
		assert (linkage_get_link_lword(linkage, link) < SIZE_MAX);
		dname = linkage_get_link_domain_names(linkage, link);
		for (j=0; j<linkage_get_link_num_domains(linkage, link); ++j) {
			append_string(s, " (%s)", dname[j]);
		}
		for (; j<longest; j++) {
			dyn_strcat(s, "    ");
		}
		dyn_strcat(s, "   ");
		print_a_link(s, linkage, link);
	}
	dyn_strcat(s, "\n");
	if (linkage_get_violation_name(linkage) != NULL) {
		dyn_strcat(s, "P.P. violations:\n");
		append_string(s, "        %s\n\n", linkage_get_violation_name(linkage));
	}

	return dyn_str_take(s);
}

char * linkage_print_senses(Linkage linkage)
{
	dyn_str * s = dyn_str_new();
#ifdef USE_CORPUS
	Linkage_info *lifo = linkage->info;
	Sense *sns;
	size_t nwords;
	WordIdx w;

	lg_corpus_linkage_senses(linkage);

	nwords = linkage->num_words;
	for (w=0; w<nwords; w++)
	{
		sns = lg_get_word_sense(linkage, w);
		while (sns)
		{
			int idx = lg_sense_get_index(sns);
			const char * wd = lg_sense_get_subscripted_word(sns);
			const char * dj = lg_sense_get_disjunct(sns);
			const char * sense = lg_sense_get_sense(sns);
			double score = lg_sense_get_score(sns);
			append_string(s, "%d %s dj=%s sense=%s score=%f\n",
			              idx, wd, dj, sense, score);
			sns = lg_sense_next(sns);
		}
	}

#else
	dyn_strcat(s, "Corpus statistics is not enabled in this version\n");
#endif
	return dyn_str_take(s);
}

char * linkage_print_disjuncts(const Linkage linkage)
{
#ifdef USE_CORPUS
	double score;
#endif
	const char * dj;
	int w;
	dyn_str * s = dyn_str_new();
	int nwords = linkage->num_words;

	/* Loop over each word in the sentence */
	for (w = 0; w < nwords; w++)
	{
		int pad = 21;
		double cost;
		const char *infword;
		Disjunct *disj = linkage->chosen_disjuncts[w];
		if (NULL == disj) continue;

		/* Subscript mark will be cleaned up by append_string(). */
		infword = disj->word_string;

		/* Make sure the glyphs align during printing. */
		pad += strlen(infword) - utf8_strwidth(infword);

		dj = linkage_get_disjunct_str(linkage, w);
		if (NULL == dj) dj = "";
		cost = linkage_get_disjunct_cost(linkage, w);

#ifdef USE_CORPUS
		score = linkage_get_disjunct_corpus_score(linkage, w);
		append_string(s, "%*s    %5.3f %6.3f %s\n", pad, infword, cost, score, dj);
#else
		append_string(s, "%*s    %5.3f  %s\n", pad, infword, cost, dj);
#endif
	}
	return dyn_str_take(s);
}

/**
 * postscript printing ...
 */
static char *
build_linkage_postscript_string(const Linkage linkage,
                                bool display_walls, ps_ctxt_t *pctx)
{
	int link, i,j;
	int d;
	bool print_word_0, print_word_N;
	int N_links = linkage->num_links;
	Link *ppla = linkage->link_array;
	dyn_str * string;
	int N_words_to_print;

	string = dyn_str_new();

	if (!display_walls) {
		int N_wall_connectors = 0;
		bool suppressor_used = false;
		for (j=0; j<N_links; j++) {
			if (ppla[j].lw == 0) {
				if (ppla[j].rw == linkage->num_words-1) continue;
				N_wall_connectors ++;
				if (strcmp(ppla[j].lc->string, LEFT_WALL_SUPPRESS)==0) {
					suppressor_used = true;
				}
			}
		}
		print_word_0 = (((!suppressor_used) && (N_wall_connectors != 0))
						|| (N_wall_connectors != 1));
	}
	else print_word_0 = true;

	if (!display_walls) {
		int N_wall_connectors = 0;
		bool suppressor_used = false;
		for (j=0; j<N_links; j++) {
			if (ppla[j].rw == linkage->num_words-1) {
				N_wall_connectors ++;
				if (strcmp(ppla[j].lc->string, RIGHT_WALL_SUPPRESS)==0){
					suppressor_used = true;
				}
			}
		}
		print_word_N = (((!suppressor_used) && (N_wall_connectors != 0))
						|| (N_wall_connectors != 1));
	}
	else print_word_N = true;

	if (print_word_0) d=0; else d=1;

	i = 0;
	N_words_to_print = linkage->num_words;
	if (!print_word_N) N_words_to_print--;

	dyn_strcat(string, "[");
	for (j=d; j<N_words_to_print; j++) {
		if ((i%10 == 0) && (i>0)) dyn_strcat(string, "\n");
		i++;
		append_string(string, "(%s)", linkage->word[j]);
	}
	dyn_strcat(string,"]");
	dyn_strcat(string,"\n");

	dyn_strcat(string,"[");
	j = 0;
	for (link=0; link<N_links; link++) {
		if (!print_word_0 && (ppla[link].lw == 0)) continue;
		if (!print_word_N && (ppla[link].rw == linkage->num_words-1)) continue;
		// if (ppla[link]->lw == SIZE_MAX) continue;
		assert (ppla[link].lw != SIZE_MAX);
		if ((j%7 == 0) && (j>0)) dyn_strcat(string,"\n");
		j++;
		append_string(string,"[%zu %zu %d",
				ppla[link].lw - d, ppla[link].rw - d,
				pctx->link_heights[link]);
		append_string(string," (%s)]", ppla[link].link_name);
	}
	dyn_strcat(string,"]");
	dyn_strcat(string,"\n");
	dyn_strcat(string,"[");
	for (j=0; j < pctx->N_rows; j++ )
	{
		if (j>0) append_string(string, " %d", pctx->row_starts[j]);
		else append_string(string,"%d", pctx->row_starts[j]);
	}
	dyn_strcat(string,"]\n");

	return dyn_str_take(string);
}

#define HEIGHT_INC 10

static void diagram_alloc_tmpmem(size_t **start, char ***pic, char ***xpic,
                                 size_t *cur_height, size_t max_height,
                                 size_t max_line, size_t line_len)
{
	assert(line_len < max_line);
	assert(max_height > *cur_height);

	*start = realloc(*start, max_height * sizeof(size_t));
	*pic = realloc(*pic, max_height * sizeof(char *));
	*xpic = realloc(*xpic, max_height * sizeof(char *));


	for (size_t i = *cur_height; i < max_height; i++)
	{
		/* Allocate memory for both pic and xpic. */
		char *picmem = malloc(max_line * 2);

		(*pic)[i] = picmem;
		(*xpic)[i] = picmem + max_line;

		memset((*pic)[i], ' ', line_len);
		(*pic)[i][line_len] = '\0';
	}

	*cur_height = max_height;
}

static void diagram_free_tmpmem(size_t *start, char **pic, char **xpic,
                                size_t cur_height)
{

	for (size_t i = 0; i < cur_height; i++)
		free(pic[i]);

	free(start);
	free(pic);
	free(xpic);
}

/**
 * Print the indicated linkage into a utf8-diagram.
 * Works fine for general utf8 multi-byte sentences.
 * Links and connectors are still mostly assumed to be ASCII, though;
 * to fix this, grep for "strlen" in the code below, replace by utf8 len.
 *
 * Returned string is allocated with exalloc.
 * Needs to be freed with linkage_free_diagram()
 */
static char *
linkage_print_diagram_ctxt(const Linkage linkage,
                           bool display_walls,
                           size_t x_screen_width,
                           ps_ctxt_t *pctx)
{
	bool display_short = true;
	unsigned int i, j, k, cl, cr, inc, row, top_row, top_row_p1;
	const char *s;
	char *t;
	bool print_word_0 , print_word_N;
	int *center = alloca((linkage->num_words+1)*sizeof(int));
	int *word_offset = alloca((linkage->num_words+1) * sizeof(*word_offset));
	unsigned int link_length;
	unsigned int N_links = linkage->num_links;
	Link *ppla = linkage->link_array;
	dyn_str * string;
	unsigned int N_words_to_print;

	// Avoid pathological case and the resulting crash.
	if (0 == linkage->num_words) return strdup("");

	string = dyn_str_new();

	/* Do we want to print the left wall? */
	if (!display_walls)
	{
		int N_wall_connectors = 0;
		bool suppressor_used = false;
		for (j=0; j<N_links; j++)
		{
			if (0 == ppla[j].lw)
			{
				if (ppla[j].rw == linkage->num_words-1) continue;
				N_wall_connectors ++;
				if (0 == strcmp(ppla[j].lc->string, LEFT_WALL_SUPPRESS))
				{
					suppressor_used = true;
				}
			}
		}
		print_word_0 = (((!suppressor_used) && (N_wall_connectors != 0))
					|| (N_wall_connectors != 1));
	}
	else print_word_0 = true;

	/* Do we want to print the right wall? */
	if (!display_walls)
	{
		int N_wall_connectors = 0;
		bool suppressor_used = false;
		for (j=0; j<N_links; j++)
		{
			if (ppla[j].rw == linkage->num_words-1)
			{
				N_wall_connectors ++;
				if (0 == strcmp(ppla[j].lc->string, RIGHT_WALL_SUPPRESS))
				{
					suppressor_used = true;
				}
			}
		}
		print_word_N = (((!suppressor_used) && (N_wall_connectors != 0))
					|| (N_wall_connectors != 1));
	}
	else print_word_N = true;

	N_words_to_print = linkage->num_words;
	if (!print_word_N) N_words_to_print--;

	size_t *start = NULL;
	char **picture = NULL;
	char **xpicture = NULL;
	size_t max_height = 0;
	size_t max_line = set_centers(linkage, center, word_offset,
	                              print_word_0, N_words_to_print) +1;
	unsigned int line_len = center[N_words_to_print-1]+1;

	diagram_alloc_tmpmem(&start, &picture, &xpicture,
	                     &max_height, HEIGHT_INC,
	                     max_line, line_len);

	top_row = 0;

	// Longer links are printed above the lower links.
	for (link_length = 1; link_length < N_words_to_print; link_length++)
	{
		for (j=0; j<N_links; j++)
		{
			assert (ppla[j].lw != SIZE_MAX);
			if (NULL == ppla[j].link_name) continue;
			if (((unsigned int) (ppla[j].rw - ppla[j].lw)) != link_length)
			  continue;
			if (!print_word_0 && (ppla[j].lw == 0)) continue;
			/* Gets rid of the irrelevant link to the left wall */
			if (!print_word_N && (ppla[j].rw == linkage->num_words-1)) continue;

			/* Put it into the lowest position */
			cl = center[ppla[j].lw];
			cr = center[ppla[j].rw];
			for (row=0; row < max_height; row++)
			{
				for (k=cl+1; k<cr; k++)
				{
					if (picture[row][k] != ' ') break;
				}
				if (k == cr) break;
			}

			/* We know it fits, so put it in this row */
			pctx->link_heights[j] = row;

			if (2*row+2 > max_height-1) {
				lgdebug(+9, "Extending rows up to %d.\n", (2*row+2)+HEIGHT_INC);
				diagram_alloc_tmpmem(&start, &picture, &xpicture,
				                     &max_height, (2*row+2)+HEIGHT_INC,
				                     max_line, line_len);
			}
			if (row > top_row) top_row = row;

			picture[row][cl] = '+';
			picture[row][cr] = '+';
			for (k=cl+1; k<cr; k++) {
				picture[row][k] = '-';
			}

			s = ppla[j].link_name;
			k = strlen(s);
			inc = cl + cr + 2;
			if (inc < k) inc = 0;
			else inc = (inc-k)/2;
			if (inc <= cl) {
				t = picture[row] + cl + 1;
			} else {
				t = picture[row] + inc;
			}

			/* Add direction indicator */
			// if (DEPT_CHR == ppla[j]->lc->string[0]) { *(t-1) = '<'; }
			if (DEPT_CHR == ppla[j].lc->string[0] &&
			    (t > &picture[row][cl])) { picture[row][cl+1] = '<'; }
			if (HEAD_CHR == ppla[j].lc->string[0]) { *(t-1) = '>'; }

			/* Copy connector name; stop short if no room */
			while ((*s != '\0') && (*t == '-')) *t++ = *s++;

			/* Add direction indicator */
			// if (DEPT_CHR == ppla[j]->rc->string[0]) { *t = '>'; }
			if (DEPT_CHR == ppla[j].rc->string[0]) { picture[row][cr-1] = '>'; }
			if (HEAD_CHR == ppla[j].rc->string[0]) { *t = '<'; }

			/* The direction indicators may have clobbered these. */
			picture[row][cl] = '+';
			picture[row][cr] = '+';

			/* Now put in the | below this one, where needed */
			for (k=0; k<row; k++) {
				if (picture[k][cl] == ' ') {
					picture[k][cl] = '|';
				}
				if (picture[k][cr] == ' ') {
					picture[k][cr] = '|';
				}
			}
		}
	}

	/* We have the link picture, now put in the words and extra "|"s */
	t = xpicture[0];
	if (print_word_0) k = 0; else k = 1;
	for (; k<N_words_to_print; k++) {
		for (i = 0; i < (size_t)word_offset[k]; i++) *t++ = ' ';
		s = linkage->word[k];
		i = 0;
		while (*s != '\0') {
			*t++ = *s++;
			i++;
		}
		*t++ = ' ';
	}
	*t = '\0';

	/* If display_short is NOT true, then the linkage diagram is printed
	 * in the "tall" style, with an extra row of vertical descenders
	 * between each level. */
	if (display_short) {
		for (k=0; picture[0][k] != '\0'; k++) {
			if ((picture[0][k] == '+') || (picture[0][k] == '|')) {
				xpicture[1][k] = '|';
			} else {
				xpicture[1][k] = ' ';
			}
		}
		xpicture[1][k] = '\0';
		for (row=0; row < top_row+1; row++) {
			strcpy(xpicture[row+2], picture[row]);
		}
		top_row += 2;
	} else {
		for (row=0; row < top_row+1; row++) {
			strcpy(xpicture[2*row+2],picture[row]);
			for (k=0; picture[row][k] != '\0'; k++) {
				if ((picture[row][k] == '+') || (picture[row][k] == '|')) {
					xpicture[2*row+1][k] = '|';
				} else {
					xpicture[2*row+1][k] = ' ';
				}
			}
			xpicture[2*row+1][k] = '\0';
		}
		top_row = 2*top_row + 2;
	}

	/* We've built the picture, now print it out. */

	/* Start locations, for each row.  These may vary, due to different
	 * utf8 character widths. */
	top_row_p1 = top_row + 1;
	for (row = 0; row < top_row_p1; row++)
		start[row] = 0;
	pctx->N_rows = 0;
	pctx->row_starts[pctx->N_rows] = 0;
	pctx->N_rows++;

	if (print_word_0) i = 0; else i = 1;
	while (i < N_words_to_print)
	{
		unsigned int revrs;
		/* Count the column-widths of the words,
		 * up to the max screen width. */
		unsigned int uwidth = 0;
		unsigned int wwid;
		do {
			wwid = word_offset[i] + utf8_strwidth(linkage->word[i]) + 1;
			if (x_screen_width <= uwidth + wwid) break;
			uwidth += wwid;
			i++;
		} while (i < N_words_to_print);

		pctx->row_starts[pctx->N_rows] = i - (!print_word_0);    /* PS junk */
		if (i < N_words_to_print) pctx->N_rows++;     /* same */

		dyn_strcat(string, "\n");
		top_row_p1 = top_row + 1;
		for (revrs = 0; revrs < top_row_p1; revrs++)
		{
			/* print each row of the picture */
			/* `blank` is used solely to detect blank lines */
			bool blank = true;
			/*
			 * The `glyph_width` is the width, in columns, of the printable
			 * glyph. Chinese glyphs are almost always width two.
			 */
			unsigned int glyph_width = 0;

			wchar_t  wc;
			mbstate_t mbss;
			memset(&mbss, 0, sizeof(mbss));

			row = top_row - revrs;
			k = start[row];
			for (j = k; (glyph_width < uwidth) && (xpicture[row][j] != '\0'); )
			{
				blank = blank && (xpicture[row][j] == ' ');

				/* Update the printable column width */
				mbrtowc(&wc, &xpicture[row][j], MB_CUR_MAX, &mbss);
				glyph_width += mk_wcwidth(wc);

				j += utf8_next(&xpicture[row][j]);
			}
			start[row] = j;

			if (!blank)
			{
				memset(&mbss, 0, sizeof(mbss));
				glyph_width = 0;
				for (j = k; (glyph_width < uwidth) && (xpicture[row][j] != '\0'); )
				{
					mbrtowc(&wc, &xpicture[row][j], MB_CUR_MAX, &mbss);
					glyph_width += mk_wcwidth(wc);

					/* Copy exactly one multi-byte character to buf */
					j += append_utf8_char(string, &xpicture[row][j]);
				}
				dyn_strcat(string, "\n");
			}
		}
		dyn_strcat(string, "\n");
	}

	diagram_free_tmpmem(start, picture, xpicture, max_height);
	return dyn_str_take(string);
}

/**
 * Print the indicated linkage as utf8-art into the given string.
 * The width of the diagram is given by the terminal width, taken
 * from the parse options.
 *
 * The returned string is malloced, and needs to be freed with
 * linkage_free_diagram()
 */
char * linkage_print_diagram(const Linkage linkage, bool display_walls, size_t screen_width)
{
	ps_ctxt_t ctx;
	if (!linkage) return NULL;

	ctx.link_heights = (int *) alloca(linkage->num_links * sizeof(int));
	ctx.row_starts = (int *) alloca((linkage->num_words + 1) * sizeof(int));
	return linkage_print_diagram_ctxt(linkage, display_walls, screen_width, &ctx);
}

void linkage_free_diagram(char * s)
{
	exfree(s, strlen(s)+1);
}

void linkage_free_disjuncts(char * s)
{
	exfree(s, strlen(s)+1);
}

void linkage_free_links_and_domains(char * s)
{
	exfree(s, strlen(s)+1);
}

void linkage_free_senses(char * s)
{
	exfree(s, strlen(s)+1);
}

/* Forward declarations, the gunk is at the bottom. */
static const char * trailer(bool print_ps_header);
static const char * header(bool print_ps_header);

char * linkage_print_postscript(const Linkage linkage, bool display_walls, bool print_ps_header)
{
	char * ps, * qs, * ascii;
	int size;

	/* call the ascii printer to initialize the row size stuff. */
	ps_ctxt_t ctx;
	ctx.link_heights = (int *) alloca(linkage->num_links * sizeof(int));
	ctx.row_starts = (int *) alloca((linkage->num_words + 1) * sizeof(int));
	ascii = linkage_print_diagram_ctxt(linkage, display_walls, 8000, &ctx);
	linkage_free_diagram(ascii);

	ps = build_linkage_postscript_string(linkage, display_walls, &ctx);
	size = strlen(header(print_ps_header)) + strlen(ps) + strlen(trailer(print_ps_header)) + 1;

	qs = (char *) exalloc(sizeof(char)*size);
	snprintf(qs, size, "%s%s%s", header(print_ps_header), ps, trailer(print_ps_header));
	exfree(ps, strlen(ps)+1);

	return qs;
}

void linkage_free_postscript(char * s)
{
	exfree(s, strlen(s)+1);
}

char * linkage_print_pp_msgs(Linkage linkage)
{
	if (linkage && linkage->lifo.pp_violation_msg)
		return strdup(linkage->lifo.pp_violation_msg);
	return strdup("");
}

void linkage_free_pp_msgs(char * s)
{
	exfree(s, strlen(s)+1);
}

void print_disjunct_counts(Sentence sent)
{
	size_t i;
	int c;
	Disjunct *d;
	for (i=0; i<sent->length; i++)
	{
		c = 0;
		for (d=sent->word[i].d; d != NULL; d = d->next) c++;

		/* XXX alternatives[0] is not really correct, here .. */
		printf("%s(%d) ", sent->word[i].alternatives[0], c);
	}
	printf("\n\n");
}

static const char * trailer(bool print_ps_header)
{
	static const char * trailer_string=
		"diagram\n"
		"\n"
		"%%EndDocument\n"
		;

	if (print_ps_header) return trailer_string;
	else return "";
}

static const char * header(bool print_ps_header)
{
	static const char * header_string=
		"%!PS-Adobe-2.0 EPSF-1.2\n"
		"%%Pages: 1\n"
		"%%BoundingBox: 0 -20 500 200\n"
		"%%EndComments\n"
		"%%BeginDocument: \n"
		"\n"
		"% compute size of diagram by adding\n"
		"% #rows x 8.5\n"
		"% (#rows -1) x 10\n"
		"% \\sum maxheight x 10\n"
		"/nulllink () def                     % The symbol of a null link\n"
		"/wordfontsize 11 def      % the size of the word font\n"
		"/labelfontsize 9 def      % the size of the connector label font\n"
		"/ex 10 def  % the horizontal radius of all the links\n"
		"/ey 10 def  % the height of the level 0 links\n"
		"/ed 10 def  % amount to add to this height per level\n"
		"/radius 10 def % radius for rounded arcs\n"
		"/row-spacing 10 def % the space between successive rows of the diagram\n"
		"\n"
		"/gap wordfontsize .5 mul def  % the gap between words\n"
		"/top-of-words wordfontsize .85 mul def\n"
		"             % the delta y above where the text is written where\n"
		"             % the major axis of the ellipse is located\n"
		"/label-gap labelfontsize .1 mul def\n"
		"\n"
		"/xwordfontsize 10 def      % the size of the word font\n"
		"/xlabelfontsize 10 def      % the size of the connector label font\n"
		"/xex 10 def  % the horizontal radius of all the links\n"
		"/xey 10 def  % the height of the level 0 links\n"
		"/xed 10 def  % amount to add to this height per level\n"
		"/xradius 10 def % radius for rounded arcs\n"
		"/xrow-spacing 10 def % the space between successive rows of the diagram\n"
		"/xgap wordfontsize .5 mul def  % the gap between words\n"
		"\n"
		"/centerpage 6.5 72 mul 2 div def\n"
		"  % this number of points from the left margin is the center of page\n"
		"\n"
		"/rightpage 6.5 72 mul def\n"
		"  % number of points from the left margin is the right margin\n"
		"\n"
		"/show-string-centered-dict 5 dict def\n"
		"\n"
		"/show-string-centered {\n"
		"  show-string-centered-dict begin\n"
		"  /string exch def\n"
		"  /ycenter exch def\n"
		"  /xcenter exch def\n"
		"  xcenter string stringwidth pop 2 div sub\n"
		"  ycenter labelfontsize .3 mul sub\n"
		"  moveto\n"
		"  string show\n"
		"  end\n"
		"} def\n"
		"\n"
		"/clear-word-box {\n"
		"  show-string-centered-dict begin\n"
		"  /string exch def\n"
		"  /ycenter exch def\n"
		"  /xcenter exch def\n"
		"  newpath\n"
		"  /urx string stringwidth pop 2 div def\n"
		"  /ury labelfontsize .3 mul def\n"
		"  xcenter urx sub ycenter ury sub moveto\n"
		"  xcenter urx add ycenter ury sub lineto\n"
		"  xcenter urx add ycenter ury add lineto\n"
		"  xcenter urx sub ycenter ury add lineto\n"
		"  closepath\n"
		"  1 setgray fill\n"
		"  0 setgray\n"
		"  end\n"
		"} def\n"
		"\n"
		"/diagram-sentence-dict 20 dict def\n"
		"\n"
		"/diagram-sentence-circle\n"
		"{diagram-sentence-dict begin  \n"
		"   /links exch def\n"
		"   /words exch def\n"
		"   /n words length def\n"
		"   /Times-Roman findfont wordfontsize scalefont setfont\n"
		"   /x 0 def\n"
		"   /y 0 def\n"
		"\n"
		"   /left-ends [x dup words {stringwidth pop add gap add dup}\n"
		"	                     forall pop pop] def\n"
		"   /right-ends [x words {stringwidth pop add dup gap add} forall pop] def\n"
		"   /centers [0 1 n 1 sub {/i exch def\n"
		"		      left-ends i get\n"
		"		      right-ends i get\n"
		"		      add 2 div\n"
		"		    } for ] def\n"
		"\n"
		"   x y moveto\n"
		"   words {show gap 0 rmoveto} forall\n"
		"\n"
		"   .5 setlinewidth \n"
		"\n"
		"   links {dup 0 get /leftword exch def\n"
		"          dup 1 get /rightword exch def\n"
		"          dup 2 get /level exch def\n"
		"          3 get /string exch def\n"
		"          newpath\n"
		"          string nulllink eq {[2] 1 setdash}{[] 0 setdash} ifelse\n"
		"%          string nulllink eq {.8 setgray}{0 setgray} ifelse\n"
		"          centers leftword get\n"
		"	  y top-of-words add\n"
		"          moveto\n"
		"      \n"
		"          centers rightword get\n"
		"          centers leftword get\n"
		"          sub 2  div dup\n"
		"          radius \n"
		"          lt {/radiusx exch def}{pop /radiusx radius def} ifelse\n"
		"  \n"
		"          \n"
		" \n"
		"          centers leftword get\n"
		"	  y top-of-words add ey ed level mul add add\n"
		"          centers rightword get\n"
		"	  y top-of-words add ey ed level mul add add\n"
		"	  radiusx\n"
		"          arcto\n"
		"          4 {pop} repeat\n"
		"	  centers rightword get\n"
		"          y top-of-words add ey ed level mul add add\n"
		"	  centers rightword get\n"
		"	  y top-of-words add\n"
		"	  radiusx\n"
		"	  arcto\n"
		"          4 {pop} repeat\n"
		"	  centers rightword get\n"
		"	  y top-of-words add\n"
		"	  lineto\n"
		"\n"
		"	  stroke\n"
		"\n"
		"          /radius-y    ey ed level mul add	  def\n"
		"\n"
		"	  /center-arc-x\n"
		"	     centers leftword get centers rightword get add 2 div\n"
		"	  def\n"
		"	  \n"
		"          /center-arc-y\n"
		"             y top-of-words radius-y add add\n"
		"	  def\n"
		"\n"
		"          /Courier-Bold findfont labelfontsize scalefont setfont \n"
		"	  center-arc-x center-arc-y string clear-word-box\n"
		"	  center-arc-x center-arc-y string show-string-centered\n"
		"          } forall\n"
		"	  end\n"
		"  } def\n"
		"\n"
		"/diagramdict 20 dict def\n"
		"\n"
		"/diagram\n"
		"{diagramdict begin\n"
		"   /break-words exch def\n"
		"   /links exch def\n"
		"   /words exch def\n"
		"   /n words length def\n"
		"   /n-rows break-words length def\n"
		"   /Times-Roman findfont wordfontsize scalefont setfont\n"
		"\n"
		"   /left-ends [0 dup words {stringwidth pop add gap add dup}\n"
		"	                     forall pop pop] def\n"
		"   /right-ends [0 words {stringwidth pop add dup gap add} forall pop] def\n"
		"\n"
		"   /lwindows [ break-words {left-ends exch get gap 2 div sub } forall ] def\n"
		"   /rwindows [1 1 n-rows 1 sub {/i exch def\n"
		"		      lwindows i get } for\n"
		"	              right-ends n 1 sub get gap 2 div add\n"
		"	      ] def\n"
		"\n"
		"\n"
		"    /max 0 def\n"
		"    0 1 links length 1 sub {\n"
		"	/i exch def\n"
		"	/t links i get 2 get def\n"
		"	t max gt {/max t def} if\n"
		"      } for\n"
		"\n"
		"    /max-height ed max mul ey add top-of-words add row-spacing add def\n"
		"    /total-height n-rows max-height mul row-spacing sub def\n"
		"\n"
		"    /max-width 0 def            % compute the widest window\n"
		"    0 1 n-rows 1 sub {\n"
		"        /i exch def\n"
		"        /t rwindows i get lwindows i get sub def\n"
		"        t max-width gt {/max-width t def} if\n"
		"      } for\n"
		"\n"
		"    centerpage max-width 2 div sub 0 translate  % centers it\n"
		"   % rightpage max-width sub 0 translate      % right justified\n"
		"                        % Delete both of these to make it left justified\n"
		"\n"
		"   n-rows 1 sub -1 0\n"
		"     {/i exch def\n"
		"	gsave\n"
		"	newpath\n"
		"        %/centering centerpage rwindows i get lwindows i get sub 2 div sub def\n"
		"               % this line causes each row to be centered\n"
		"        /centering 0 def\n"
		"               % set centering to 0 to prevent centering of each row \n"
		"\n"
		"	centering -100 moveto  % -100 because some letters go below zero\n"
		"        centering max-height n-rows mul lineto\n"
		"        rwindows i get lwindows i get sub centering add\n"
		"                       max-height n-rows mul lineto\n"
		"        rwindows i get lwindows i get sub centering add\n"
		"                       -100 lineto\n"
		"	closepath\n"
		"        clip\n"
		"	lwindows i get neg n-rows i sub 1 sub max-height mul translate\n"
		"        centerpage centering 0 translate\n"
		"        words links diagram-sentence-circle\n"
		"	grestore\n"
		"     } for\n"
		"     end\n"
		"} def \n"
		"\n"
		"/diagramx\n"
		"{diagramdict begin\n"
		"   /break-words exch def\n"
		"   /links exch def\n"
		"   /words exch def\n"
		"   /n words length def\n"
		"   /n-rows break-words length def\n"
		"   /Times-Roman findfont xwordfontsize scalefont setfont\n"
		"\n"
		"   /left-ends [0 dup words {stringwidth pop add gap add dup}\n"
		"	                     forall pop pop] def\n"
		"   /right-ends [0 words {stringwidth pop add dup gap add} forall pop] def\n"
		"\n"
		"   /lwindows [ break-words {left-ends exch get gap 2 div sub } forall ] def\n"
		"   /rwindows [1 1 n-rows 1 sub {/i exch def\n"
		"		      lwindows i get } for\n"
		"	              right-ends n 1 sub get xgap 2 div add\n"
		"	      ] def\n"
		"\n"
		"\n"
		"    /max 0 def\n"
		"    0 1 links length 1 sub {\n"
		"	/i exch def\n"
		"	/t links i get 2 get def\n"
		"	t max gt {/max t def} if\n"
		"      } for\n"
		"\n"
		"    /max-height xed max mul xey add top-of-words add xrow-spacing add def\n"
		"    /total-height n-rows max-height mul xrow-spacing sub def\n"
		"\n"
		"    /max-width 0 def            % compute the widest window\n"
		"    0 1 n-rows 1 sub {\n"
		"        /i exch def\n"
		"        /t rwindows i get lwindows i get sub def\n"
		"        t max-width gt {/max-width t def} if\n"
		"      } for\n"
		"\n"
		"    centerpage max-width 2 div sub 0 translate  % centers it\n"
		"   % rightpage max-width sub 0 translate      % right justified\n"
		"                        % Delete both of these to make it left justified\n"
		"\n"
		"   n-rows 1 sub -1 0\n"
		"     {/i exch def\n"
		"	gsave\n"
		"	newpath\n"
		"        %/centering centerpage rwindows i get lwindows i get sub 2 div sub def\n"
		"               % this line causes each row to be centered\n"
		"        /centering 0 def\n"
		"               % set centering to 0 to prevent centering of each row \n"
		"\n"
		"	centering -100 moveto  % -100 because some letters go below zero\n"
		"        centering max-height n-rows mul lineto\n"
		"        rwindows i get lwindows i get sub centering add\n"
		"                       max-height n-rows mul lineto\n"
		"        rwindows i get lwindows i get sub centering add\n"
		"                       -100 lineto\n"
		"	closepath\n"
		"        clip\n"
		"	lwindows i get neg n-rows i sub 1 sub max-height mul translate\n"
		"        centerpage centering 0 translate\n"
		"        words links diagram-sentence-circle\n"
		"	grestore\n"
		"     } for\n"
		"     end\n"
		"} def \n"
		"\n"
		"/ldiagram\n"
		"{diagramdict begin\n"
		"   /break-words exch def\n"
		"   /links exch def\n"
		"   /words exch def\n"
		"   /n words length def\n"
		"   /n-rows break-words length def\n"
		"   /Times-Roman findfont wordfontsize scalefont setfont\n"
		"\n"
		"   /left-ends [0 dup words {stringwidth pop add gap add dup}\n"
		"	                     forall pop pop] def\n"
		"   /right-ends [0 words {stringwidth pop add dup gap add} forall pop] def\n"
		"\n"
		"   /lwindows [ break-words {left-ends exch get gap 2 div sub } forall ] def\n"
		"   /rwindows [1 1 n-rows 1 sub {/i exch def\n"
		"		      lwindows i get } for\n"
		"	              right-ends n 1 sub get gap 2 div add\n"
		"	      ] def\n"
		"\n"
		"\n"
		"    /max 0 def\n"
		"    0 1 links length 1 sub {\n"
		"	/i exch def\n"
		"	/t links i get 2 get def\n"
		"	t max gt {/max t def} if\n"
		"      } for\n"
		"\n"
		"    /max-height ed max mul ey add top-of-words add row-spacing add def\n"
		"    /total-height n-rows max-height mul row-spacing sub def\n"
		"\n"
		"    /max-width 0 def            % compute the widest window\n"
		"    0 1 n-rows 1 sub {\n"
		"        /i exch def\n"
		"        /t rwindows i get lwindows i get sub def\n"
		"        t max-width gt {/max-width t def} if\n"
		"      } for\n"
		"\n"
		"   % centerpage max-width 2 div sub 0 translate  % centers it\n"
		"   % rightpage max-width sub 0 translate      % right justified\n"
		"                        % Delete both of these to make it left justified\n"
		"\n"
		"   n-rows 1 sub -1 0\n"
		"     {/i exch def\n"
		"	gsave\n"
		"	newpath\n"
		"        %/centering centerpage rwindows i get lwindows i get sub 2 div sub def\n"
		"               % this line causes each row to be centered\n"
		"        /centering 0 def\n"
		"               % set centering to 0 to prevent centering of each row \n"
		"\n"
		"	centering -100 moveto  % -100 because some letters go below zero\n"
		"        centering max-height n-rows mul lineto\n"
		"        rwindows i get lwindows i get sub centering add\n"
		"                       max-height n-rows mul lineto\n"
		"        rwindows i get lwindows i get sub centering add\n"
		"                       -100 lineto\n"
		"	closepath\n"
		"        clip\n"
		"	lwindows i get neg n-rows i sub 1 sub max-height mul translate\n"
		"        centerpage centering 0 translate\n"
		"        words links diagram-sentence-circle\n"
		"	grestore\n"
		"     } for\n"
		"     end\n"
		"} def \n"
		;
	if (print_ps_header) return header_string;
	else return "";
}

/**
 * Print elements of the 2D-word-array produced for the parsers.
 *
 * - print_sentence_word_alternatives(sent, false, NULL, tokenpos)
 * If a pointer to struct "tokenpos" is given, return through it the index of
 * the first occurrence in the sentence of the given token. This is used to
 * prevent duplicate information display for repeated morphemes (if there are
 * multiples splits, each of several morphemes, otherwise some of them may
 * repeat).
 *
 * - print_sentence_word_alternatives(sent, true, NULL, NULL)
 * If debugprint is "true", this is a debug printout of the sentence.  (The
 * debug printouts are with level 0 because this function is invoked for debug
 * on certain positive level.)
 *
 *
 * - print_sentence_word_alternatives(sent, false, display_func, NULL)
 * Iterate over the sentence words and their alternatives.  Handle each
 * alternative using the display_func function if it is supplied, or else (if it
 * is NULL) just print them. It is used to display disjunct information when
 * command !!word is used.
 * FIXME In the current version (using Wordgraph) the "alternatives" in the
 * word-array don't necessarily consist of real word alternatives.
 *
 */

struct tokenpos /* First position of the given token - to prevent duplicates */
{
	const char * token;
	size_t wi;
	size_t ai;
};

void print_sentence_word_alternatives(dyn_str *s, Sentence sent, bool debugprint,
     char * (*display)(Dictionary, const char *), struct tokenpos * tokenpos)
{
	size_t wi;   /* Internal sentence word index */
	size_t ai;   /* Index of a word alternative */
	size_t sentlen = sent->length;     /* Shortened if there is a right-wall */
	size_t first_sentence_word = 0;    /* Used for skipping a left-wall */
	bool word_split = false;           /* !!word got split */
	Dictionary dict = sent->dict;

	if (0 == sentlen)
	{
		/* It should not happen, but if it actually happens due to some
		 * strange conditions, it's better not to abort the program. */
		prt_error("Error: Sentence length is 0 (reason unknown)\n");
		return;
	}

	if (debugprint || (NULL != tokenpos))
		; /* Do nothing */
	else
	{
		/* For analyzing words we need to ignore the left/right walls */
		if (dict->left_wall_defined &&
		    (0 == strcmp(sent->word[0].unsplit_word, LEFT_WALL_WORD)))
			first_sentence_word = 1;
		if (dict->right_wall_defined &&
		    ((NULL != sent->word[sentlen-1].unsplit_word)) &&
		    (0 == strcmp(sent->word[sentlen-1].unsplit_word, RIGHT_WALL_WORD)))
			sentlen--;

		/* Find if a word got split. This is indicated by:
		 * 1. More than one word in the sentence
		 * (no check if it actually results from !!w1 w2 ...).
		 * 2. A word containing more than one alternative. */
		if (sentlen - first_sentence_word > 1)
		{
			word_split = true;
		}
		else
		{
			for (wi=first_sentence_word; wi<sentlen; wi++)
			{
				Word w = sent->word[wi];

				/* There should always be at least one alternative */
				assert((NULL != w.alternatives) && (NULL != w.alternatives[0]) &&
				 ('\0' != w.alternatives[0][0]), "Missing alt for word %zu", wi);

				if (NULL != w.alternatives[1])
				{
					word_split = true;
					break;
				}
			}
		}
		/* "String", because it can be a word, morpheme, or (TODO) idiom */
		if (word_split && (NULL == display)) dyn_strcat(s, "String splits to:\n");
		/* We used to print the alternatives of the word here, one per line.
		 * In the current (Wordgraph) version, the alternatives may look
		 * like nonsense combination of tokens - not as the strict split
		 * possibilities of words as in previous versions.
		 * E.g.: For Hebrew word "הכלב", we now get these "alternatives":
		 *   ה=  כלב  לב  ב=
		 *   ה=  כ=  ל=
		 *   ה=  כ=
		 * For "'50s,"
		 *   ' s s ,
		 *   '50 50 ,
		 *   '50s
		 * Clearly, this is not informative any more. Instead, one line with a
		 * list of tokens (without repetitions) is printed
		 * ה= כלב לב ב= כ= ל=
		 *
		 * FIXME Print the alternatives from the wordgraph.
		 */
	}

	/* Iterate over sentence input words */
	for (wi=first_sentence_word; wi<sentlen; wi++)
	{
		Word w = sent->word[wi];
		size_t w_start = wi;   /* input word index */
		size_t max_nalt = 0;

#if 0 /* In the Wordgraph version firstupper and post_quote don't exist. */
		if (debugprint) lgdebug(0, "  word%d %c%c: %s\n   ",
		 wi, w.firstupper ? 'C' : ' ', sent->post_quote[wi] ? 'Q' : ' ',
#endif
		if (debugprint) append_string(s, "  word%zu: %s\n", wi, w.unsplit_word);

		/* There should always be at least one alternative */
		assert((NULL != w.alternatives) && (NULL != w.alternatives[0]) &&
		 ('\0' != w.alternatives[0][0]), "Missing alt for word %zu", wi);

		//err_msg(lg_Debug, "word%zu '%s' nalts %zu\n",
		//	 wi, sent->word[wi].unsplit_word, altlen(sent->word[wi].alternatives));

		for (wi = w_start; (wi == w_start) ||
			((wi < sentlen) && (! sent->word[wi].unsplit_word)); wi++)
		{
			size_t nalt = altlen(sent->word[wi].alternatives);

			max_nalt = MAX(max_nalt, nalt);
		}

		/* Iterate over alternatives */
		for (ai=0; ai < max_nalt;  ai++)
		{
			if (debugprint)
			{
				if (0 < ai) dyn_strcat(s, "\n   ");
				append_string(s, "   alt%zu:", ai);
			}

			for (wi = w_start; (wi == w_start) ||
			    ((wi < sentlen) && (! sent->word[wi].unsplit_word)); wi++)
			{
				size_t nalts = altlen(sent->word[wi].alternatives);
				const char *wt;

				if (ai >= nalts) continue;
				wt = sent->word[wi].alternatives[ai];

				/* Don't display information again for the same word */
				if ((NULL != tokenpos) && (0 == strcmp(tokenpos->token, wt)))
				{
					tokenpos->wi = wi;
					tokenpos->ai = ai;
					return;
				}
				if (!debugprint)
				{
					struct tokenpos firstpos = { wt };

					print_sentence_word_alternatives(s, sent, false, NULL, &firstpos);
					if (((firstpos.wi != wi) || (firstpos.ai != ai)) &&
					  firstpos.wi >= first_sentence_word) // allow !!LEFT_WORD
					{
						/* We encountered this token earlier */
						if (NULL != display)
							lgdebug(6, "Skipping repeated %s\n", wt);
						continue;
					}
				}

				if (debugprint)
				{
					const char *opt_start = "", *opt_end = "";
					if (sent->word[wi].optional)
					{
						opt_start = "{";
						opt_end = "}";
					}
					append_string(s, " %s%s%s", opt_start, wt, opt_end);
				}

				/* Don't try to give info on the empty word. */
				if ('\0' != wt[0])
				{
					/* For now each word component is called "Token".
					 * TODO: Its type can be decoded and a more precise
					 * term (stem, prefix, etc.) can be used.
					 * Display the features of the token */
					if ((NULL == tokenpos) && (NULL != display))
					{
						char *info = display(sent->dict, wt);

						if (NULL == info) return;
						append_string(s, "Token \"%s\" %s", wt, info);
						free(info);
					}
					else if (word_split) append_string(s, " %s", wt);
				}
			}

			/* Commented out - no alternatives for now - print as one line. */
			//if (word_split && (NULL == display)) dyn_strcat(s, "\n");
		}
		wi--;
		if (debugprint) dyn_strcat(s, "\n");
	}
	if (debugprint) dyn_strcat(s, "\n");
}

// Use for debug and error printing.
void print_sentence_context(Sentence sent, dyn_str *outbuf)
{
	size_t i, j;
	const char **a, **b;

	dyn_strcat(outbuf,
		"\tFailing sentence contains the following words/morphemes:\n\t");
	for (i=0; i<sent->length; i++)
	{
		for (a = sent->word[i].alternatives; NULL != *a; a++)
		{
			bool next_word = false;

			for (j=0; j<sent->length; j++)
			{
				for (b = sent->word[j].alternatives; NULL != *b; b++)
				{
					/* print only the first occurrence. */
					if (0 == strcmp(*a, *b))
					{
						next_word = true;
						if (a != b) break;
						dyn_strcat(outbuf, *a);
						dyn_strcat(outbuf, " ");
						break;
					}
				}
				if (next_word) break;
			}
		}
	}
	dyn_strcat(outbuf, "\n");
}
