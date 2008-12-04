/*
 * EdgeThin.cc
 *
 * Thin out, remove edges between words, sentences.
 *
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>

#include "EdgeThin.h"
#include "ForeachWord.h"

#define DEBUG

using namespace opencog;
/**
 * Remove edges between senses of sets of words
 *
 * The routines in this file operate in a manner similr to those in
 * MihalceaEdge, but "in reverse" -- removing edges instead of adding
 * them.  Most of the routines will have a "count" paramter, indicating
 * the number of edges to keep. The "count" parameter works as follows:
 *
 * If zero, it will remove all edges between all word pairs.
 *
 * If one, it will remove all edges, except the one connecting the
 * top-ranked sense of each word.
 *
 * If two, it will remove all edges that do not connect the top-two
 * ranked senses for each word.
 *
 * If N, it will remove all edges that do not connect the top-N ranked
 * senses for each word.
 */

void EdgeThin::set_atom_space(AtomSpace *as)
{
   atom_space = as;
}

/**
 * Remove edges between senses in the indicated parse.
 *
 * Similar to, but opposite of MihalceaEdge::annotate_parse()
 * rather than adding edges, it removes them.
 */
void EdgeThin::thin_parse(Handle h, int keep)
{
	foreach_word_instance(h, &EdgeUtils::look_at_word, (EdgeUtils *) this);

	// At this point, "words" contains all of the words in the parse.
	// Loop over word-pairs, and thin edges.
	edge_count = 0;
	word_pair_count = 0;
	std::set<Handle>::const_iterator f;
	for (f = words.begin(); f != words.end(); f++)
	{
		std::set<Handle>::const_iterator s = f;
		for (s++; s != words.end(); s++)
		{
			thin_word_pair(*f, *s, keep);
		}
	}
}

/**
 * Remove edges between senses between a pair opf parses
 *
 * Similar to, but opposite of MihalceaEdge::annotate_parse_pair()
 * rather than adding edges, it removes them.
 */
void EdgeThin::thin_parse_pair(Handle e, Handle l, int keep)
{
printf ("duude remove all but %d edges from parse pair\n", keep);
}

/**
 * Compare two senses, returning true if the first is more likely than
 * the second.
 */
static bool sense_compare(Handle ha, Handle hb)
{
	Link *la = dynamic_cast<Link *>(TLB::getAtom(ha));
	Link *lb = dynamic_cast<Link *>(TLB::getAtom(hb));
	double sa = la->getTruthValue().getMean();
	double sb = lb->getTruthValue().getMean();
	if (sa > sb) return true;
	return false;
}

bool EdgeThin::make_sense_list(Handle sense_h, Handle sense_link_h)
{
	sense_list.push_back(sense_link_h);
	return false;
}

/**
 * Remove a similarity relation between a pair of word senses.
 * The arg should be a handle to a CosenseLink, previusly constructed
 * by MihalceaEdge::sense_of_second_inst().
 */
bool EdgeThin::delete_sim(Handle h)
{
	atom_space->removeAtom(h, false);
	edge_count ++;
	return false;
}

/**
 * Remove edges between senses of a pair of words.
 *
 * Similar to, but opposite to MihalceaEdge::annotate_word_pair()
 * rather than adding edges, it removes them.
 */
bool EdgeThin::thin_word_pair(Handle first, Handle second, int keep)
{
#ifdef DEBUG
	Node *f = dynamic_cast<Node *>(TLB::getAtom(first));
	Node *s = dynamic_cast<Node *>(TLB::getAtom(second));
	const std::string &fn = f->getName();
	const std::string &sn = s->getName();
	printf ("; Thin out wordPair (%s, %s) to %d\n", fn.c_str(), sn.c_str(), keep);
#endif

	sense_list.clear();
	foreach_word_sense_of_inst(first, &EdgeThin::make_sense_list, this);
	sense_list.sort(sense_compare);

	// unque the ones that we will keep
	int k = sense_list.size();
	if (keep < k) k = keep;
	for (int i=0; i<k; i++) sense_list.pop_front();

	// delete the rest.
	std::list<Handle>::iterator it;
	for (it=sense_list.begin(); it != sense_list.end(); it++)
	{
		Handle sense_h = *it;
#ifdef DEBUG
		Link *la = dynamic_cast<Link *>(TLB::getAtom(sense_h));
		double sa = la->getTruthValue().getMean();
		Handle hws = get_word_sense_of_sense_link(sense_h);
		Node *ws = dynamic_cast<Node *>(TLB::getAtom(hws));
		printf ("; deleteing sense %s with score %f\n", ws->getName(), sa);
#endif
		foreach_incoming_handle(sense_h, &EdgeThin::delete_sim, this);
		// atom_space->removeAtom(sense_h, false);
	}

printf ("duuude deleted %d edges so far\n", edge_count);

	// second_word_inst = second;
	// foreach_word_sense_of_inst(first, &EdgeThin::sense_of_first_inst, this);
	
	word_pair_count ++;
	return false;
}

