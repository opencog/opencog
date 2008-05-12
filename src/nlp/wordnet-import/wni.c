/**
 * wni.c
 * WordNet Import
 *
 * Import wordnet database into opencog.
 *
 * This version uses the native C progamming interfaces supplied by 
 * Princeton, as a part of the Wordnet project. 
 */

#include <wn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFSZ 300

static int getsspos(Synset *synp)
{
	// The value stored in *(synp->ppos) seems to be incorrect,
	// its alwaus 1, so construct the pos from string.
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

static void get_sense_key(char * buff, Synset *synp, int idx)
{
	if (!synp->headword)
	{
		sprintf(buff, "%s%%%d:%02d:%02d::",
		              synp->words[idx], getsspos(synp),
		              synp->fnum, synp->lexid[idx]);
	}
	else
	{
		sprintf(buff, "%s%%%d:%02d:%02d:%s:%02d",
		              synp->words[idx], getsspos(synp),
		              synp->fnum, synp->lexid[idx],
		              synp->headword, synp->headsense);
	}
}

#define SENSE(RELNAME, BLOCK) { \
	if ((1<<RELNAME) & bitmask) \
	{ \
		Synset *nymp = findtheinfo_ds(word, pos, RELNAME, sense_num); \
		if (nymp) nymp = nymp->ptrlist; \
 \
		while(nymp) \
		{ \
			if (5 != getsspos(nymp)) \
			{ \
				printf("<!-- gloss=%s -->\n", nymp->defn); \
				int i; \
				for (i=0; i<nymp->wcount; i++) \
				{ \
					get_sense_key(buff, nymp, i); \
					printf("<WordSenseNode name=\"%s\" />\n", buff); \
					(BLOCK); \
				} \
			} \
			nymp = nymp->nextss; \
		} \
	} \
}

static void print_nyms(char * sense_key, char * word, int sense_num, Synset *synp)
{
	char buff[BUFSZ];

	// Hmm .. GetSenseIndex() is buggy, it crashes due to bad access
	// SnsIndex *si = GetSenseIndex(sense_key);
	// printf ("nym sense=%d, %d\n", si->wnsense, sense_num);

	int pos = getsspos(synp);

	unsigned int bitmask = is_defined(word, pos);
	// printf ("mask=%x\n", bitmask);

	printf("<WordSenseNode name=\"%s\" />\n", sense_key);

	// Consult 'man 3 winintro' for details of these calls.
	//
	/* Hypernym */
	SENSE (HYPERPTR, ({
		printf("<InheritanceLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("</InheritanceLink>\n");
	}))

	/* Hyponym */
	SENSE (HYPOPTR, ({
		printf("<InheritanceLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("</InheritanceLink>\n");
	}))

	/* Similarity */
	SENSE (SIMPTR, ({
		printf("<SimilarityLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("</SimilarityLink>\n");
	}))

	/* Member holonym */
	SENSE (ISMEMBERPTR, ({
		printf("<HolonymLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("</HolonymLink>\n");
	}))

	/* Substance holonym */
	SENSE (ISSTUFFPTR, ({
		printf("<HolonymLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("</HolonymLink>\n");
	}))

	/* Substance holonym */
	SENSE (ISPARTPTR, ({
		printf("<HolonymLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("</HolonymLink>\n");
	}))

	/* Member meronym */
	SENSE (HASMEMBERPTR, ({
		printf("<HolonymLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("</HolonymLink>\n");
	}))

	/* Substance meronym */
	SENSE (HASSTUFFPTR, ({
		printf("<HolonymLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("</HolonymLink>\n");
	}))

	/* Substance meronym */
	SENSE (HASPARTPTR, ({
		printf("<HolonymLink>\n");
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", sense_key);
		printf("   <Element class=\"WordSenseNode\" name=\"%s\" />\n", buff);
		printf("</HolonymLink>\n");
	}))

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

void print_synset(char * sense_key, int sense_num, Synset *synp)
{
	char * posstr = "";
	switch(synp->pos[0])
	{
		case 'n': posstr = "noun"; break;
		case 'v': posstr = "verb"; break;
		case 'a':  posstr = "adjective"; break;
		case 'r':  posstr = "adverb"; break;
		default:
			fprintf(stderr, "Error: unknown pos %x\n", synp->pos[0]);
			exit(1);
	}

	printf("<ConceptNode name = \"%s\" />\n", sense_key);
	printf("<PartOfSpeechLink>\n");
	printf("   <Element class=\"ConceptNode\" name = \"%s\" />\n", sense_key);
	printf("   <Element class=\"ConceptNode\" name = \"%s\" />\n", posstr);
	printf("</PartOfSpeechLink>\n");

	printf("<!-- gloss=%s -->\n", synp->defn);

	int i;
	for (i=0; i<synp->wcount; i++)
	{
		printf("<WordNode name = \"%s\" />\n", synp->words[i]);
		printf("<WordSenseLink>\n");
		printf("   <Element class=\"WordNode\" name = \"%s\" />\n", synp->words[i]);
		printf("   <Element class=\"ConceptNode\" name = \"%s\" />\n", sense_key);
		printf("</WordSenseLink>\n");

		print_nyms(sense_key, synp->words[i], sense_num, synp);
	}
}

void show_index(char * index_entry)
{
	Synset *synp;

	// Parse a line out from /usr/share/wordnet/index.sense
	// The format of this line is documented in 'man index.sense'
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
		return;
	}

	if (!synp)
	{
		fprintf(stderr, "Error: failed to find sysnset!!\n");
		fprintf(stderr, "sense=%s pos=%d off=%d\n", sense_key, ipos, offset);
		return;
	}

	if (synp->hereiam != offset)
	{
		fprintf(stderr, "Error: bad offset!!\n");
		fprintf(stderr, "sense=%s pos=%d off=%d\n", sense_key, ipos, offset);
	}

	print_synset(sense_key, sense_num, synp);

	free_synset(synp);
}

main (int argc, char * argv[])
{
	char buff[BUFSZ];
	wninit();

#ifdef TEST_STRINGS
	strcpy(buff, "shiny%3:00:04:: 01119421 2 0");
	strcpy(buff, "abandon%2:40:01:: 02227741 2 6");
	strcpy(buff, "fast%4:02:01:: 00086000 1 16");
	strcpy(buff, "abnormal%5:00:00:immoderate:00 01533535 3 0");
	strcpy(buff, "bark%1:20:00:: 13162297 1 4");
#endif

	printf("data\n");
	printf("<list>\n");
	printf("<ConceptNode name = \"noun\" />\n");
	printf("<ConceptNode name = \"verb\" />\n");
	printf("<ConceptNode name = \"adjective\" />\n");
	printf("<ConceptNode name = \"adverb\" />\n");

	// open /usr/share/wordnet/index.sense
	// The format of this file is described in 'man senseidx'
	FILE *fh = fopen("/usr/share/wordnet/index.sense", "r");
	while (1)
	{
		char * rc = fgets(buff, BUFSZ, fh);
		if (!rc) break;
		show_index(buff);
	}
	printf("</list>\n");

}
