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

bool EdgeThin::prune_sense(Handle sense_h, Handle sense_link_h)
{
	// If incoming set of this sense link is empty, remove it entirely.
	Atom *a = TLB::getAtom(sense_link_h);
	if (NULL == a->getIncomingSet())
	{
		atom_space->removeAtom(sense_link_h, false);
		prune_count ++;
	}
	return false;
}

bool EdgeThin::prune_word(Handle h)
{
	foreach_word_sense_of_inst(h, &EdgeThin::prune_sense, this);
	return false;
}

/**
 * Remove all word senses that are not attached to anything.
 * Argument should be a sentence parse.
 */
void EdgeThin::prune_parse(Handle h)
{
	prune_count = 0;
	foreach_word_instance(h, &EdgeThin::prune_word, this);
#ifdef DEBUG
	printf("; EdgeThin::prune_senses pruned %d senses\n", prune_count);
#endif
}

/**
 * Remove edges between senses in the indicated parse.
 *
 * Similar to, but opposite of MihalceaEdge::annotate_parse()
 * rather than adding edges, it removes them.
 */
void EdgeThin::thin_parse(Handle h, int _keep)
{
	edge_count = 0;
	keep = _keep;
	foreach_word_instance(h, &EdgeThin::thin_word, this);
#ifdef DEBUG
	printf("; EdgeThin::thin_parse deleted %d edges\n", edge_count);
#endif
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
bool EdgeThin::thin_word(Handle word_h)
{
#ifdef DEBUG
	Node *w = dynamic_cast<Node *>(TLB::getAtom(word_h));
	const std::string &wn = w->getName();
	printf ("; EdgeThin::thin_word %s to %d\n", wn.c_str(), keep);
#endif

	sense_list.clear();
	foreach_word_sense_of_inst(word_h, &EdgeThin::make_sense_list, this);
	sense_list.sort(sense_compare);

	// unqueue the ones that we will keep
	int k = sense_list.size();
	if (keep < k) k = keep;
	for (int i=0; i<k; i++) sense_list.pop_front();

	// delete the rest.
	std::list<Handle>::iterator it;
	for (it=sense_list.begin(); it != sense_list.end(); it++)
	{
		Handle sense_h = *it;
#ifdef XDEBUG
		Link *la = dynamic_cast<Link *>(TLB::getAtom(sense_h));
		double sa = la->getTruthValue().getMean();
		Handle hws = get_word_sense_of_sense_link(sense_h);
		Node *ws = dynamic_cast<Node *>(TLB::getAtom(hws));
		printf ("; deleting sense %s with score %f\n", ws->getName().c_str(), sa);
#endif
		foreach_incoming_handle(sense_h, &EdgeThin::delete_sim, this);

		// XXX Hmm, should we, or should we not delete the now-disconnected
		// senses?
		// atom_space->removeAtom(sense_h, false);
	}

	return false;
}

