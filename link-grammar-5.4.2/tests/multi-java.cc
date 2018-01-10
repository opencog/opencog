/***************************************************************************/
/* Copyright (c) 2014,2017 Linas Vepstas                                        */
/* All rights reserved                                                     */
/*                                                                         */
/* Use of the link grammar parsing system is subject to the terms of the   */
/* license set forth in the LICENSE file included with this software.      */
/* This license allows free redistribution and use in source and binary    */
/* forms, with or without modification, subject to certain conditions.     */
/*                                                                         */
/***************************************************************************/

// This implements a very simple-minded multi-threaded unit test for
// the Java bindings.
// All it does is to make sure the system doesn't crash e.g. due to
// memory allocation conflicts.

#include <thread>
#include <vector>

#include <locale.h>
#include <stdio.h>
#include "link-grammar/link-includes.h"
#include "bindings/java-jni/jni-client.h"

extern "C" {
void unit_test_jparse(JNIEnv *env, const char* inputString);
};

static void parse_one_sent(const char *sent_str)
{
	// Java_org_linkgrammar_LinkGrammar_parse(NULL, 0, jstring
	unit_test_jparse(NULL, sent_str);

	int num_linkages = Java_org_linkgrammar_LinkGrammar_getNumLinkages(NULL, 0);

	if (0 < num_linkages)
	{
		if (10 < num_linkages) num_linkages = 10;

		for (int li = 0; li<num_linkages; li++)
		{
			Java_org_linkgrammar_LinkGrammar_makeLinkage(NULL, 0, li);

			Java_org_linkgrammar_LinkGrammar_getLinkageDisjunctCost(NULL, 0);
			Java_org_linkgrammar_LinkGrammar_getLinkageLinkCost (NULL, 0);
			Java_org_linkgrammar_LinkGrammar_getNumLinks(NULL, 0);
		}
	}
}

static void parse_sents(int thread_id, int niter)
{
	Java_org_linkgrammar_LinkGrammar_init(NULL, 0);

	const char *sents[] = {
		"Frank felt vindicated when his long time friend Bill revealed that he was the winner of the competition.",
		"Logorrhea, or excessive and often incoherent talkativeness or wordiness, is a social disease.",
		"It was covered with bites.",
		"I have no idea what that is.",
		"His shout had been involuntary, something anybody might have done.",
		"He obtained the lease of the manor of Great Burstead Grange (near East Horndon) from the Abbey of Stratford Langthorne, and purchased the manor of Bayhouse in West Thurrock.",
		"We ate popcorn and watched movies on TV for three days.",
		"Sweat stood on his brow, fury was bright in his one good eye.",
		"One of the things you do when you stop your bicycle is apply the brake.",
		"The line extends 10 miles offshore."
// "под броню боевого робота устремились потоки энергии.",
// "через четверть часа здесь будет полно полицейских."
	};

	int nsents = sizeof(sents) / sizeof(const char *);

	for (int j=0; j<niter; j += nsents)
	{
		for (int i=0; i < nsents; ++i)
		{
			parse_one_sent(sents[i]);
		}
	}
}

int main(int argc, char* argv[])
{
	setlocale(LC_ALL, "en_US.UTF-8");
	dictionary_set_data_dir(DICTIONARY_DIR "/data");
	Java_org_linkgrammar_LinkGrammar_init(NULL, 0);

	int n_threads = 10;
	int niter = 50;

	printf("Creating %d threads, each parsing %d sentences\n",
		 n_threads, niter);
	std::vector<std::thread> thread_pool;
	for (int i=0; i < n_threads; i++) {
		thread_pool.push_back(std::thread(parse_sents, i, niter));
	}

	// Wait for all threads to complete
	for (std::thread& t : thread_pool) t.join();

	Java_org_linkgrammar_LinkGrammar_doFinalize(NULL, 0);

	printf("Done with multi-threaded parsing\n");

	return 0;
}
