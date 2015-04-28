/**
 * wni.c
 * WordNet Import
 *
 * Import wordnet database into opencog.
 *
 * This version uses the native C progamming interfaces supplied by 
 * Princeton, as a part of the Wordnet project. 
 *
 * Copyright (C) 2008 Linas Vepstas
 */

#include <wn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFSZ 300

/**
 * If defined, generate output for the various relations (holo/hypernym,
 * etc.) Otherise, only the sense-keys are generated.
 */
#define GENERATE_NYMS 1

/**
 * Skip processing of colocations if this flag is set to 1
 * This makes the output smaller, but breaks the hierachy
 * tree, and thus breaks word-sense similarity measures.
 */
static int skip_colocations = 0;

static int do_export(const char * word)
{
	if (0 == skip_colocations) return 1;
	char * p = strchr(word, '_');
	return (p == NULL);
}

static int getsspos(Synset *synp)
{
	int pos = 0;
	switch (synp->pos[0])
	{
		case 'n': pos = 1; break;
		case 'v': pos = 2; break;
		case 'a': pos = 3; break;
		case 'r': pos = 4; break;
		case 's': pos = 5; break;
		default:
			fprintf(stderr, "Error: unexpected pos %x\n", synp->pos[0]);
			exit(1);
	}
	return pos;
}

/**
 * Create a sense-key string, given a synset.
 */
static void get_sense_key(char * buff, Synset *synp, int idx)
{
	// The synp->ppos field seems to frequently contain garbage or
	// erroneous data. Have to use getsspos instead, to get the proper
	// part-of-speech.  This is OK, because hypernyms/hyponyms, etc. 
	// will always have the same part of speech (right?).  This may not
	// be true for cause-by, pertains-to, entails.
	// 
	// if (getsspos(synp) != synp->ppos[idx]) 
	//    fprintf (stderr, "fail! %d %d\n", getsspos(synp), synp->ppos[idx]);
	//
	int pos = getsspos(synp);
	if (!synp->headword)
	{
		sprintf(buff, "%s%%%d:%02d:%02d::",
		              synp->words[idx], pos,
		              synp->fnum, synp->lexid[idx]);
	}
	else
	{
		sprintf(buff, "%s%%%d:%02d:%02d:%s:%02d",
		              synp->words[idx], pos,
		              synp->fnum, synp->lexid[idx],
		              synp->headword, synp->headsense);
	}
}

/**
 * Main loop for finding a relation, and printing it out.
 * See 'man 3 wnintro' for an overview, and 'man findtheinfo'
 * for a description of how synsets are to be navigated.
 */
#define SENSE(RELNAME, BLOCK) { \
	if ((1<<RELNAME) & bitmask) \
	{ \
		Synset *nymp = findtheinfo_ds(word, pos, RELNAME, sense_num); \
		Synset *sroot = nymp; \
		if (nymp) nymp = nymp->ptrlist; \
 \
		while(nymp) \
		{ \
			if (5 != getsspos(nymp)) \
			{ \
				/* printf("<!-- gloss=%s -->\n", nymp->defn); */ \
				int i; \
				for (i=0; i<nymp->wcount; i++) \
				{ \
					if (0 == do_export(nymp->words[i])) continue; \
					get_sense_key(buff, nymp, i); \
					(BLOCK); \
				} \
			} \
			nymp = nymp->nextss; \
		} \
		if (sroot) free_syns(sroot); \
	} \
}

/**
 * Print the relations between different synsets.
 */
static void print_nyms(char * sense_key, char * word, int sense_num, Synset *synp)
{
	char buff[BUFSZ];

	// Hmm .. GetSenseIndex() is buggy, it crashes due to bad access
	// SnsIndex *si = GetSenseIndex(sense_key);
	// printf ("nym sense=%d, %d\n", si->wnsense, sense_num);

	int pos = getsspos(synp);

	unsigned int bitmask = is_defined(word, pos);
	// printf ("word=%s sense=%d mask=%x\n", word, sense_num, bitmask);

	// Not needed, we'd printed this previously ... 
	// printf("<WordSenseNode name=\"%s\" />\n", sense_key);

	/* Hypernym */
	SENSE (HYPERPTR, ({
		printf("(InheritanceLink s (WordSenseNode \"%s\"))\n", buff);
	}))

	/* Hyponym */
	SENSE (HYPOPTR, ({
		printf("(InheritanceLink (WordSenseNode \"%s\") s)\n", buff);
	}))

	/* Similarity */
	SENSE (SIMPTR, ({
		printf("(SimilarityLink s (WordSenseNode \"%s\"))\n", buff);
	}))

	/* Member holonym */
	SENSE (ISMEMBERPTR, ({
		printf("(HolonymLink s (WordSenseNode \"%s\"))\n", buff);
	}))

	/* Substance holonym */
	SENSE (ISSTUFFPTR, ({
		printf("(HolonymLink s (WordSenseNode \"%s\"))\n", buff);
	}))

	/* Substance holonym */
	SENSE (ISPARTPTR, ({
		printf("(HolonymLink s (WordSenseNode \"%s\"))\n", buff);
	}))

	/* Member meronym */
	SENSE (HASMEMBERPTR, ({
		printf("(HolonymLink (WordSenseNode \"%s\") s)\n", buff);
	}))

	/* Substance meronym */
	SENSE (HASSTUFFPTR, ({
		printf("(HolonymLink (WordSenseNode \"%s\") s)\n", buff);
	}))

	/* Substance meronym */
	SENSE (HASPARTPTR, ({
		printf("(HolonymLink (WordSenseNode \"%s\") s)\n", buff);
	}))

#if LATER
	SENSE (DERIVATION, ({
		printf("duuude  name=\"%s\" />\n", sense_key);
		printf("yeah duu name=\"%s\" />\n", buff);
	}))
#endif

	/* Some unhandled cases */
	if ((1<<ENTAILPTR) & bitmask)
	{
		fprintf(stderr, "Warning: unhandled entail for %s\n", sense_key);
	}
	if ((1<<CAUSETO) & bitmask)
	{
		fprintf(stderr, "Warning: unhandled causeto for %s\n", sense_key);
	}
	if ((1<<PPLPTR) & bitmask)
	{
		fprintf(stderr, "Warning: unhandled participle of verb for %s\n", sense_key);
	}
	if ((1<<PERTPTR) & bitmask)
	{
		fprintf(stderr, "Warning: unhandled pertaining for %s\n", sense_key);
	}
}

/**
 * Print the synset.
 * First, print the word, and associate the word-sense index to it.
 * Next, associate the part-of-speech to the word-sense index.
 * Finally, traverse the set of synset relations, and print those.
 */
static int syn_count = 0;

static void print_synset(char * sense_key, int sense_num, Synset *synp)
{
	char * posstr = "";
	switch(synp->pos[0])
	{
		/* These string match the RelEx naming for parts-of-speech. */
		case 'n': posstr = "noun"; break;
		case 'v': posstr = "verb"; break;
		case 'a':  posstr = "adj"; break;
		case 'r':  posstr = "adv"; break;
		default:
			fprintf(stderr, "Error: unknown pos %x\n", synp->pos[0]);
			exit(1);
	}

	printf("(define s (WordSenseNode \"%s\")) ", sense_key);
	printf("(PartOfSpeechLink s %s)\n", posstr);

	// Don't print gloss - some glosses have double-dash, 
	// which drives XML parser nuts.
	// printf("<!-- gloss=%s -->\n", synp->defn);

	int i;
	for (i=0; i<synp->wcount; i++)
	{
		if (0 == do_export(synp->words[i])) continue;

		printf("(define x (WordNode \"%s\")) ", synp->words[i]);
		printf("(WordSenseLink x s)\n");

#ifdef GENERATE_NYMS
		print_nyms(sense_key, synp->words[i], synp->wnsns[i], synp);
#endif
		syn_count ++;
	}
}

/**
 * Parse a line out from /usr/share/wordnet/index.sense
 * The format of this line is documented in 'man index.sense'
 * Use this line to find the corresponding sysnset.
 * Print the synset, then delete the synset.
 */
static int show_index(char * index_entry)
{
	Synset *synp;

	if (!do_export(index_entry)) return -1;

	char * p = strchr(index_entry, '%') + 1;
	int ipos = atoi(p);

	char * sense_key = index_entry;
	p = strchr(index_entry, ' ');
	*p = 0;
	char * byte_offset = ++p;
	int offset = atoi(byte_offset);

	p = strchr(p, ' ') + 1;
	int sense_num = atoi(p); 

	// Read the synset corresponding to this line.
	synp = read_synset(ipos, offset, NULL);

	if (5 == ipos)
	{
		return -2;
	}

	if (!synp)
	{
		fprintf(stderr, "Error: failed to find sysnset!!\n");
		fprintf(stderr, "sense=%s pos=%d off=%d\n", sense_key, ipos, offset);
		return -3;
	}

	if (synp->hereiam != offset)
	{
		fprintf(stderr, "Error: bad offset!!\n");
		fprintf(stderr, "sense=%s pos=%d off=%d\n", sense_key, ipos, offset);
	}

	printf ("; ----------------------------- \n");
	print_synset(sense_key, sense_num, synp);

	// free_synset() only frees one sysnet, not the whole chain of them.
	free_syns(synp);

	return 0;
}

main (int argc, char * argv[])
{
	char buff[BUFSZ];
	wninit();

	/* Default sense index location over-ridden at the command line */
	char * sense_index = "/usr/share/wordnet/index.sense";
	if (2 == argc) sense_index = argv[1];

	// open /usr/share/wordnet/index.sense
	// The format of this file is described in 'man senseidx'
	FILE *fh = fopen(sense_index, "r");
	if (!fh)
	{
		fprintf(stderr, "Fatal error: cannot open file %s\n", sense_index);
		exit(1);
	}

#ifdef TEST_STRINGS
	// Some sample strings, typical of what is encountered in
	// the index.sense file.
	strcpy(buff, "shiny%3:00:04:: 01119421 2 0");
	strcpy(buff, "abandon%2:40:01:: 02227741 2 6");
	strcpy(buff, "fast%4:02:01:: 00086000 1 16");
	strcpy(buff, "abnormal%5:00:00:immoderate:00 01533535 3 0");
	strcpy(buff, "bark%1:20:00:: 13162297 1 4");
	strcpy(buff, "abnormally%4:02:00:: 00227171 1 1");
	strcpy(buff, "sign%1:10:03:: 06791372 3 4");
	strcpy(buff, "covering%1:17:00:: 09257949 1 0");
	strcpy(buff, "cut%2:35:10:: 01610834 4 2");
	show_index(buff);
	exit(0);
#endif

	// XXX ?? is there some reason these are not "DefinedLinguisticConceptNode" ??
	// I'd think they should be, right ... ? Is this a bug ??
	printf("scm\n");
	printf("(define noun (ConceptNode \"noun\"))\n");
	printf("(define verb (ConceptNode \"verb\"))\n");
	printf("(define adj (ConceptNode \"adj\"))\n");
	printf("(define adv (ConceptNode \"adv\"))\n");

	int cnt = 0;
	while (1)
	{
		char * p = fgets(buff, BUFSZ, fh);
		if (!p) break;

		int rc = show_index(buff);
		if (0 == rc) cnt ++;

		if (cnt % 1000 == 0) fprintf(stderr, "Info: done processing %d word senes\n", cnt);
	}

	printf(".\n");
	printf("exit\n");

	fprintf(stderr, "Info: finished loading %d word senses\n", cnt);
	double avg = ((double) syn_count) / ((double) cnt);
	fprintf(stderr, "Info: %d syns avg=%g\n", syn_count, avg);
}
