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

static int getpos(Synset *synp)
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
		default:
			fprintf(stderr, "Error: unexpoected pos %x\n", synp->pos[0]);
			exit(1);
	}
	return pos;
}


void print_nyms(char * sense_key, char * word, Synset *synp)
{
	SnsIndex *si = GetSenseIndex(sense_key);
	printf ("duude nym sense=%d\n", si->wnsense);

	int pos = getpos(synp);

	unsigned int bitmask = is_defined(word, pos);
	printf ("duude mask=%x\n", bitmask);

	if ((1<<HYPERPTR) & bitmask)
	{
		Synset *nymp = findtheinfo_ds(word, pos, HYPERPTR, si->wnsense);
		if (nymp) nymp = nymp->ptrlist;

		while(nymp)
		{
			printf("<!-- gloss=%s -->\n", nymp->defn);
			printf("duude whichword=%d\n", nymp->whichword);
			int i;
			for (i=0; i<nymp->wcount; i++)
			{
				printf("duude its %s\n", nymp->words[i]);
			}
			nymp = nymp->nextss;
		}
	}

	if ((1<<HYPOPTR) & bitmask)
	{
		Synset *nymp = findtheinfo_ds(word, pos, HYPOPTR, si->wnsense);
		if (nymp) nymp = nymp->ptrlist;

		while(nymp)
		{
			printf("<!-- gloss=%s -->\n", nymp->defn);
			printf("duude whichword=%d\n", nymp->whichword);
			int i;
			for (i=0; i<nymp->wcount; i++)
			{
				printf("duude its %s\n", nymp->words[i]);
				printf("duude guess %s uhh %d\n", nymp->words[i], pos);
			}
			nymp = nymp->nextss;
		}
	}
}

void print_synset(char * sense_key, Synset *synp)
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
	strcpy(buff, "shiny%3:00:04:: 01119421 2 0");
	strcpy(buff, "abandon%2:40:01:: 02227741 2 6");
	strcpy(buff, "fast%4:02:01:: 00086000 1 16");
	strcpy(buff, "bark%1:20:00:: 13162297 1 4");


	show_index(buff);

}
