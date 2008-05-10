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

main (int argc, char * argv[])
{
	Synset *synp;

	wninit();
	printf ("hello world \n");
	synp = findtheinfo_ds("dog", 1, 0, ALLSENSES);
	printf ("duude %p\n", synp);

	printf("hereiam=%d\n", synp->hereiam);
	printf("key=%d\n", synp->key);
	printf("pos=%d\n", synp->pos);
	printf("gloss=%s\n", synp->defn);

	printf("wcount=%d\n", synp->wcount);

	int i;
	for (i=0; i<synp->wcount; i++)
	{
		printf ("its %s\n", synp->words[i]);
	}
	

	free_synset(synp);
}
