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
#include <string.h>

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

	printf("sense=%s pos=%d off=%d\n", sense_key, ipos, offset);
	
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
	printf("pos=%s\n", synp->pos);
	printf("gloss=%s\n", synp->defn);

	printf("wcount=%d\n", synp->wcount);

	int i;
	for (i=0; i<synp->wcount; i++)
	{
		printf ("its %s\n", synp->words[i]);
	}
	

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


	SnsIndex *sp;
	sp = GetSenseIndex("a_horizon%1:15:00::");

	printf ("duude %p %s\n", sp, sp->word);

}
