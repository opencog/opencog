/***************************************************************************/
/* Copyright (c) 2014 Linas Vepstas                                        */
/* All rights reserved                                                     */
/*                                                                         */
/* Use of the link grammar parsing system is subject to the terms of the   */
/* license set forth in the LICENSE file included with this software.      */
/* This license allows free redistribution and use in source and binary    */
/* forms, with or without modification, subject to certain conditions.     */
/*                                                                         */
/***************************************************************************/

// This implements a simple check, opening and closing the dictionary
// repeatedly.

#include <locale.h>
#include <stdio.h>
#include "link-grammar/link-includes.h"

int main()
{
	const char * input_string[] = {
		"под броню боевого робота устремились потоки энергии.",
		"через четверть часа здесь будет полно полицейских.",
		"He is the kind of person who would do that.",
		"The mystery of the Nixon tapes was never solved."
	};

	setlocale(LC_ALL, "en_US.UTF-8");

	dictionary_set_data_dir(DICTIONARY_DIR "/data");
	Parse_Options opts = parse_options_create();
	parse_options_set_spell_guess(opts, 0);

	for (int i=0; i<20; i++)
	{
		printf("Opening the dictionary for the %d'th time\n", i+1);
		Dictionary dict;
		if (i%2 == 0)
			dict = dictionary_create_lang("ru");
		else
			dict = dictionary_create_lang("en");

		if (!dict) {
			printf ("Fatal error: Unable to open the dictionary\n");
			return 1;
		}

		for (int j=0; j<2; j++)
		{
			int isnt = 2 * (i%2) + j;
			Sentence sent = sentence_create(input_string[isnt], dict);
			sentence_split(sent, opts);
			int num_linkages = sentence_parse(sent, opts);
			if (num_linkages > 0) {
				Linkage linkage = linkage_create(0, sent, opts);
				linkage_delete(linkage);
			}
			sentence_delete(sent);
		}
		dictionary_delete(dict);
	}
	parse_options_delete(opts);
	return 0;
}
