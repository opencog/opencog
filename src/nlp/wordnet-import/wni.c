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

void print_nyms(char * sense_key, char * word, Synset *synp)
{
	unsigned int bitmask = is_defined(word, *synp->ppos);
	printf ("duude mask=%x\n", bitmask);

	// if ((1<<HYPERPTR) & bitmask)
	int j;
	for (j=0; j<32; j++)
	{
		SnsIndex *si = GetSenseIndex(sense_key);
		printf ("duude hyper sense=%d pos=%d\n", si->wnsense, *synp->ppos);
		// Synset *nymp = findtheinfo_ds(word, *synp->ppos, HYPERPTR, si->wnsense);
		// yaSynset *nymp = findtheinfo_ds(word, 2, HYPERPTR, si->wnsense);
		Synset *nymp = findtheinfo_ds(word, 2, j, si->wnsense);
if (!nymp) continue;

		printf("duude gloss=%s\n", nymp->defn);
		printf("duude whichword=%d\n", nymp->whichword);
		int i;
		for (i=0; i<nymp->wcount; i++)
		{
			printf("duude its %s\n", nymp->words[i]);
		}
	}
}

void print_synset(char * sense_key, Synset *synp)
{
	char * posstr = "";
	switch(*(synp->ppos)) // XXX this is wrong somehow
	{
		case NOUN: posstr = "noun"; break;
		case VERB: posstr = "verb"; break;
		case ADJ:  posstr = "adjective"; break;
		case ADV:  posstr = "adverb"; break;
		default:
			fprintf(stderr, "Error: unknown pos %d\n", *synp->ppos);
			exit(1);
	}

	printf("<PartOfSpeechLink>\n");
	printf("   <ConceptNode name = \"%s\" />\n", sense_key);
	printf("   <ConceptNode name = \"%s\" />\n", posstr);
	printf("</PartOfSpeechLink>\n");

	printf("<!-- gloss=%s -->\n", synp->defn);

	int i;
	for (i=0; i<synp->wcount; i++)
	{
		printf("<WordSenseLink>\n");
		printf("   <WordNode name = \"%s\" />\n", synp->words[i]);
		printf("   <ConceptNode name = \"%s\" />\n", sense_key);
		printf("</WordSenseLink>\n");
print_nyms(sense_key, synp->words[i], synp);
	}
}

void show_index(char * index_entry)
{
	Synset *synp;

	// Parse a line out from /usr/share/wordnet/index.sense
	// The format of this line is documented in 'man index.sense'
	char * sense_key = index_entry;
	char * p = strchr(index_entry, ' ');
	*p = 0;
	char * byte_offset = ++p;
	int offset = atoi(byte_offset);
	p = strchr(index_entry, '%') + 1;
	int ipos = atoi(p);

	// Read the synset corresponding to this line.
	synp = read_synset(ipos, offset, NULL);

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

	print_synset(sense_key, synp);

	free_synset(synp);
}

main (int argc, char * argv[])
{
	wninit();

	// open /usr/share/wordnet/index.sense
	//
	char buff[120];
	strcpy(buff, "abandon%2:40:01:: 02227741 2 6");
	show_index(buff);

}
