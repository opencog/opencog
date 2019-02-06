/*
 * FuzzyMatch.h
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: Leung Man Hin <https://github.com/leungmanhin>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef FUZZY_MATCH_H
#define FUZZY_MATCH_H

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/Link.h>

namespace opencog
{
/**
 * The fuzzy pattern matcher searches for trees which are similar but
 * not identical to the input target pattern. This is done by examining
 * all possible trees that have at least one leaf node in common with
 * the target pattern.  A similarity score is assigned to each such
 * tree, and the ones with the highest scores are returned.
 *
 * This is a virtual base class, providing three methods that must be
 * implemented.  The accept_starter() should return true, if the trees
 * attached to this leaf should be explored.  The try_match() method is
 * called to suggest a possible matching tree. It should return true to
 * continue searching.  The finished_serach() method is called when all
 * trees have been explored; it should return a list of the best
 * solutions.
 *
 * The `perform_search()` method performs the actual search for similar
 * trees.  It is a rather simple algorithm, and works like this: it
 * is given a Handle that specifies the target pattern to be matched.
 * This target is just a tree, if one considers the outgoing sets of
 * the links in it.  This tree has leaves, which are nodes.  Each leaf
 * may occur in other trees as well, i.e. may be shared by other trees.
 * These other trees can be found by exploring the incoming set to the
 * leaf. Any tree that shares this leaf must, by definition, occur in
 * it's incoming set.  Thus, to find potentially similar trees,  one
 * needs only to recursively explore the incoming set of each leaf.
 * So -- that is what the `perform_search()` method does: an exhaustive
 * recursive search of all possible trees sharing at least one node
 * leaf.
 *
 * Its possible that a similar tree will not have any leaf-nodes
 * in common with the target. In this case, this class wil fail to
 * consider such a tree; some other approach would be needed to find it.
 *
 * To limit the search, the return values of `accept_starter()` and
 * `try_match()` are used.  The `accept_starter()` is called for every
 * atom in the target tree. If it returns true, then other trees that
 * share that atom are proposed to `try_match()`.  Initially, the
 * smallest such trees are proposed; as long as `try_match() returns
 * true, then larger and larger trees holding the starter are proposed.
 * If it returns false, then the proposal of the ever-larger trees
 * halts.
 */

typedef std::vector<std::pair<Handle, double>> RankedHandleSeq;

class FuzzyMatch
{
public:
    RankedHandleSeq perform_search(const Handle&);
    virtual ~FuzzyMatch() {}

protected:
    virtual void start_search(const Handle&) = 0;
    virtual bool accept_starter(const Handle&) = 0;
    virtual bool try_match(const Handle&) = 0;
    virtual RankedHandleSeq finished_search(void) = 0;

private:
    void find_starters(const Handle&);
    void explore(const Handle&);
};

} // namespace opencog
#endif  // FUZZY_MATCH_H
