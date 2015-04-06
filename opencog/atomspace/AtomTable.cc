/*
 * opencog/atomspace/AtomTable.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2013-2015 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
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

#include "AtomTable.h"

#include <iterator>
#include <set>

#include <stdlib.h>
#include <boost/bind.hpp>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atoms/NumberNode.h>
#include <opencog/atoms/TypeNode.h>
#include <opencog/atoms/bind/BetaRedex.h>
#include <opencog/atoms/bind/BindLink.h>
#include <opencog/atoms/bind/DefineLink.h>
#include <opencog/atoms/bind/LambdaLink.h>
#include <opencog/atoms/bind/VariableList.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/functional.h>
#include <opencog/util/Logger.h>

//#define DPRINTF printf
//#define tableId (0) // Hack around some DPRINTF statements that want an old tableID member variable
#define DPRINTF(...)

using namespace opencog;

std::recursive_mutex AtomTable::_mtx;

AtomTable::AtomTable(AtomTable* parent)
    : _index_queue(this, &AtomTable::put_atom_into_index)
{
    _environ = parent;
    _uuid = TLB::reserve_extent(1);
    size = 0;

    // Set resolver before doing anything else, such as getting
    // the atom-added signals.  Just in case some other thread
    // is busy adding types while we are being created.
    Handle::set_resolver(this);

    // Connect signal to find out about type additions
    addedTypeConnection =
        classserver().addTypeSignal().connect(
            boost::bind(&AtomTable::typeAdded, this, _1));
}

AtomTable::~AtomTable()
{
    // Disconnect signals. Only then clear the resolver.
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    addedTypeConnection.disconnect();
    Handle::clear_resolver(this);

    // No one who shall look at these atoms ahall ever again
    // find a reference to this atomtable.
    UUID undef = Handle::UNDEFINED.value();
    for (Handle h : _atom_set) {
        h->_atomTable = NULL;
        h->_uuid = undef;
    }
}

bool AtomTable::isCleared(void) const
{
    if (size != 0) {
        DPRINTF("AtomTable::size is not 0\n");
        return false;
    }

    std::lock_guard<std::recursive_mutex> lck(_mtx);
    // if (nameIndex.size() != 0) return false;
    if (typeIndex.size() != 0) return false;
    if (importanceIndex.size() != 0) return false;
    return true;
}

AtomTable& AtomTable::operator=(const AtomTable& other)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomTable - Cannot copy an object of this class");
}

AtomTable::AtomTable(const AtomTable& other)
    :_index_queue(this, &AtomTable::put_atom_into_index)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomTable - Cannot copy an object of this class");
}

Handle AtomTable::getHandle(Type t, std::string name) const
{
    // Special types need validation
    try {
        if (NUMBER_NODE == t) {
            name = NumberNode::validate(name);
        } else if (TYPE_NODE == t) {
            TypeNode::validate(name);
        }
    }
    catch (...) { return Handle::UNDEFINED; }

    std::lock_guard<std::recursive_mutex> lck(_mtx);
    Atom* atom = nodeIndex.getAtom(t, name);
    if (atom) return atom->getHandle();
    if (_environ and NULL == atom)
        return _environ->getHandle(t, name);
    return Handle::UNDEFINED;
}

/// Find an equivalent atom that has exactly the same name and type.
/// That is, if there is an atom with this name and type already in
/// the table, then return that; else return undefined.
Handle AtomTable::getHandle(const NodePtr& n) const
{
    const AtomTable *env = this;
    do {
        if (n->_atomTable == env) return Handle(n);
        env = env->_environ;
    } while (env);

    return getHandle(n->getType(), n->getName());
}

Handle AtomTable::getHandle(Type t, const HandleSeq &seq) const
{
    // Make sure all the atoms in the outgoing set are resolved :-)
    HandleSeq resolved_seq;
    for (Handle ho : seq) {
        resolved_seq.push_back(getHandle(ho));
    }

    // Aiieee! unordered link!
    if (classserver().isA(t, UNORDERED_LINK)) {
        struct HandleComparison
        {
            bool operator()(const Handle& h1, const Handle& h2) const {
                return (Handle::compare(h1, h2) < 0);
            }
        };
        std::sort(resolved_seq.begin(), resolved_seq.end(), HandleComparison());
    }

    std::lock_guard<std::recursive_mutex> lck(_mtx);
    Handle h(linkIndex.getHandle(t, resolved_seq));
    if (_environ and Handle::UNDEFINED == h)
        return _environ->getHandle(t, resolved_seq);
    return h;
}

/// Find an equivalent atom that has exactly the same type and outgoing
/// set.  That is, if there is an atom with this ype and outset already
/// in the table, then return that; else return undefined.
Handle AtomTable::getHandle(const LinkPtr& l) const
{
    const AtomTable *env = this;
    do {
        if (l->_atomTable == env) return Handle(l);
        env = env->_environ;
    } while (env);

    return getHandle(l->getType(), l->getOutgoingSet());
}

/// Find an equivalent atom that is exactly the same as the arg. If
/// such an atom is in the table, it is returned, else the return
/// is the bad handle.
Handle AtomTable::getHandle(const AtomPtr& a) const
{
    NodePtr nnn(NodeCast(a));
    if (nnn)
         return getHandle(nnn);
    else {
        LinkPtr lll(LinkCast(a));
        if (lll)
            return getHandle(lll);
    }
    return Handle::UNDEFINED;
}

Handle AtomTable::getHandle(Handle& h) const
{
    // If we have an atom, but don't know the uuid, find uuid.
    if (Handle::UNDEFINED.value() == h.value())
        return getHandle(AtomPtr(h));

    // If we have both a uuid and pointer, AND the pointer is
    // pointing to an atom that is in this table (not some other
    // table), then there's nothing to do.  Otherwise, we have to
    // find the equivalent atom in this atomspace.
    // Note: we access the naked pointer itself; that's because
    // Handle itself calls this method to resolve null pointers.
    if (h._ptr) {
        if (this == h._ptr->_atomTable)
            return h;
        else if (_environ)
            return _environ->getHandle(h);
        else
            return getHandle(AtomPtr(h));
    }

    // Read-lock for the _atom_set.
    std::lock_guard<std::recursive_mutex> lck(_mtx);

    // If we have a uuid but no atom pointer, find the atom pointer.
    auto hit = _atom_set.find(h);
    if (hit != _atom_set.end())
        return *hit;
    return Handle::UNDEFINED;
}


/// Return true if the atom is in this atomtable, or in the
/// environment for this atomtable.
bool AtomTable::inEnviron(AtomPtr atom)
{
    AtomTable* atab = atom->getAtomTable();
    AtomTable* env = this;
    while (env) {
        if (atab == env) return true;
        env = env->_environ;
    }
    return false;
}

Handle AtomTable::add(AtomPtr atom, bool async)
{
    // Is the atom already in this table, or one of its environments?
    if (inEnviron(atom))
        return atom->getHandle();

#if LATER
    // XXX FIXME -- technically, this throw is correct, except
    // that SavingLoading gives us atoms with handles preset.
    // So we have to accept that, and hope its correct and consistent.
    // XXX this can also occur if the atom is in some other atomspace;
    // so we need to move this check elsewhere.
    if (atom->_uuid != Handle::UNDEFINED.value())
        throw RuntimeException(TRACE_INFO,
          "AtomTable - Attempting to insert atom with handle already set!");
#endif

    // Lock before checking to see if this kind of atom can already
    // be found in the atomspace.  We need to lock here, to avoid two
    // different threads from trying to add exactly the same atom.
    std::unique_lock<std::recursive_mutex> lck(_mtx);

    // Check again, under the lock this time.
    if (inEnviron(atom))
        return atom->getHandle();

    // Experimental C++ atom types support code
    Type atom_type = atom->getType();
    if (NUMBER_NODE == atom_type) {
        if (NULL == NumberNodeCast(atom))
            atom = createNumberNode(*NodeCast(atom));
    } else if (TYPE_NODE == atom_type) {
        if (NULL == TypeNodeCast(atom))
            atom = createTypeNode(*NodeCast(atom));
    } else if (BIND_LINK == atom_type) {
        if (NULL == BindLinkCast(atom))
            atom = createBindLink(*LinkCast(atom));
    } else if (BETA_REDEX == atom_type) {
        if (NULL == BetaRedexCast(atom))
            atom = createBetaRedex(*LinkCast(atom));
    } else if (DEFINE_LINK == atom_type) {
        if (NULL == DefineLinkCast(atom))
            atom = createDefineLink(*LinkCast(atom));
    } else if (LAMBDA_LINK == atom_type) {
        if (NULL == LambdaLinkCast(atom))
            atom = createLambdaLink(*LinkCast(atom));
    } else if (SATISFACTION_LINK == atom_type) {
        if (NULL == SatisfactionLinkCast(atom))
            atom = createSatisfactionLink(*LinkCast(atom));
    } else if (VARIABLE_LIST == atom_type) {
        if (NULL == VariableListCast(atom))
            atom = createVariableList(*LinkCast(atom));
    }

    // Is the equivalent of this atom already in the table?
    // If so, then return the existing atom.  (Note that this 'existing'
    // atom might be in another atomspace, or might not be in any
    // atomspace yet.)
    Handle hexist(getHandle(atom));
    if (hexist) return hexist;

    // If this atom is in some other atomspace, then we need to clone
    // it. We cannot insert it into this atomtable as-is.  (We already
    // know that its not in this atomspace, or its environ.)
    AtomTable* at = atom->getAtomTable();
    if (at != NULL) {
        NodePtr nnn(NodeCast(atom));
        if (nnn) {
            // Experimental support for atom types
            if (NUMBER_NODE == atom_type) {
                atom = createNumberNode(*nnn);
            } else if (TYPE_NODE == atom_type) {
                atom = createTypeNode(*nnn);
            } else {
                atom = createNode(*nnn);
            }
        } else {
            LinkPtr lll(LinkCast(atom));
            if (BIND_LINK == atom_type) {
                atom = createBindLink(*lll);
            } else if (BETA_REDEX == atom_type) {
                atom = createBetaRedex(*lll);
            } else if (DEFINE_LINK == atom_type) {
                atom = createDefineLink(*lll);
            } else if (LAMBDA_LINK == atom_type) {
                atom = createLambdaLink(*lll);
            } else if (SATISFACTION_LINK == atom_type) {
                atom = createSatisfactionLink(*lll);
            } else if (VARIABLE_LIST == atom_type) {
                atom = createVariableList(*lll);
            } else {
                atom = createLink(*lll);
            }

            // Well, if the link was in some other atomspace, then
            // the outgoing set will be too. So we recursively clone
            // that too.
            const HandleSeq ogset(lll->getOutgoingSet());
            size_t arity = ogset.size();
            for (size_t i = 0; i < arity; i++) {
                add(ogset[i], async);
            }
        }
    }

    // Sometimes one inserts an atom that was previously deleted.
    // In this case, the removal flag might still be set. Clear it.
    atom->unsetRemovalFlag();

    // Check for bad outgoing set members; fix them up if needed.
    // "bad" here means outgoing set members that have UUID's but
    // no pointers to actual atoms.  We want to have the actual atoms,
    // because later steps need the pointers to do stuff, in particular,
    // to make sure the child atoms are in an atomtable, too.
    LinkPtr lll(LinkCast(atom));
    if (lll) {
        const HandleSeq& ogs(lll->getOutgoingSet());
        size_t arity = ogs.size();

        // First, make sure that every member of the outgoing set has
        // a valid atom pointer. We need this, cause we need to call
        // methods on those atoms.
        bool need_copy = false;
        for (size_t i = 0; i < arity; i++) {
            Handle h(ogs[i]);
            // It can happen that the uuid is assigned, but the pointer
            // is NULL. In that case, we should at least know about this
            // uuid.  We explicitly test h._ptr.get() so as not to
            // accidentally call resolve() during the test.
            // XXX ??? How? How can this happen ??? How could we have a
            // UUID but no pointer? Some persistance scenario ???
            // Please explain ...
            if (NULL == h._ptr.get()) {
                if (Handle::UNDEFINED == h) {
                    throw RuntimeException(TRACE_INFO,
                               "AtomTable - Attempting to insert link with "
                               "invalid outgoing members");
                }
                auto it = _atom_set.find(h);
                if (it != _atom_set.end()) {
                    h = *it;

                    // OK, here's the deal. We really need to fixup
                    // link so that it holds a valid atom pointer. We
                    // do that here. Unfortunately, this is not really
                    // thread-safe, and there is no particularly elegant
                    // way to lock. So we punt.  This makes sense,
                    // because it is unlikely that one thread is going to
                    // be wingeing on the outgoing set, while another
                    // thread is performing an atom-table add.  I'm pretty
                    // sure its a user error if the user fails to serialize
                    // atom table adds appropriately for their app.
                    lll->_outgoing[i]->remove_atom(lll);
                    lll->_outgoing[i] = h;
                    lll->_outgoing[i]->insert_atom(lll);
                } else {
                    // XXX FIXME. This can trigger when external code
                    // removes atoms from the atomspace, but retains
                    // copies of the (now defunct, because deleted)
                    // UUID's.  That is, when an atom is removed from
                    // the atomtable, it's UUID is no longer valid, and
                    // So that external code should not have saved the
                    // UUID's.  However, if it did, and then created a
                    // handle out of them, then the handle would have
                    // a null atom pointer and a positive UUID, and we
                    // end up here.  This is a user error.  Note: the
                    // atomspace benchmark has been known to do this.
                    //
                    // Perhaps there are other weird secenarios, an we
                    // should search the environmnet first, before
                    // throwing... (we did not search environmnet,
                    // above ... this may need fixing...)
                    logger().info() << "Failing index i=" << i
                                    << " and arity=" << arity;
                    logger().info() << "Failing outset is this:";
                    for (unsigned int fk=0; fk<arity; fk++)
                        logger().info() << "outset i=" << fk
                                        << " uuid=" << ogs[fk].value();

                    throw RuntimeException(TRACE_INFO,
                        "AtomTable - Atom in outgoing set isn't known!");
                }
            }

            // h has to point to an actual atom, else below will crash.
            // Anyway, the outgoing set must consist entirely of atoms
            // either in this atomtable, or its environment.
            if (not inEnviron(h)) need_copy = true;
        }

        if (need_copy) {
            atom = createLink(*lll);
        }

        // llc not lll, in case a copy was made.
        LinkPtr llc(LinkCast(atom));
        for (size_t i = 0; i < arity; i++) {

            // Make sure all children have correct incoming sets
            Handle ho(llc->_outgoing[i]);
            if (not inEnviron(ho)) {
                ho->remove_atom(llc);
                llc->_outgoing[i] = add(ho, async);
            }
            else if (ho == Handle::UNDEFINED) {
                // If we are here, then the atom is in the atomspace,
                // but the handle has an invalid UUID. This can happen
                // if the atom appears more than once in the outgoing
                // set. Fix the handles' UUID, by forcing a cast.
                llc->_outgoing[i] = ((AtomPtr) llc->_outgoing[i]);
            }
            // Build the incoming set of outgoing atom h.
            llc->_outgoing[i]->insert_atom(llc);
        }

        // OK, so if the above fixed up the outgoing set, and
        // this is an unordered link, then we have to fix it up
        // and put it back into the default sort order. That's
        // because the default sort order uses UUID's, which have
        // now changed.
        if (classserver().isA(llc->getType(), UNORDERED_LINK)) {
            llc->resort();
        }
    }

    // Its possible that the atom already has a UUID assigned,
    // e.g. if it was fetched from persistent storage; this
    // was done to preserve handle consistency. SavingLoading does
    // this too.  XXX Review SavingLoading for correctness...
    if (atom->_uuid == Handle::UNDEFINED.value()) {
       // Atom doesn't yet have a valid uuid assigned to it. Ask the TLB
       // to issue a valid uuid.  And then memorize it.
       TLB::addAtom(atom);
    } else {
       TLB::reserve_upto(atom->_uuid);
    }
    Handle h(atom->getHandle());
    size++;
    _atom_set.insert(h);

    atom->keep_incoming_set();
    atom->setAtomTable(this);

    if (not async)
        put_atom_into_index(atom);

    // We can now unlock, since we are done. In particular, the signals
    // need to run unlocked, since they may result in more atom table
    // additions.
    lck.unlock();

    // Update the indexes asynchronously
    if (async)
        _index_queue.enqueue(atom);

    // Now that we are completely done, emit the added signal.
    _addAtomSignal(h);

    DPRINTF("Atom added: %ld => %s\n", atom->_uuid, atom->toString().c_str());
    return h;
}

void AtomTable::put_atom_into_index(AtomPtr& atom)
{
    std::unique_lock<std::recursive_mutex> lck(_mtx);
    Atom* pat = atom.operator->();
    nodeIndex.insertAtom(pat);
    linkIndex.insertAtom(atom);
    typeIndex.insertAtom(pat);
    importanceIndex.insertAtom(pat);
}

void AtomTable::barrier()
{
    _index_queue.flush_queue();
}

size_t AtomTable::getSize() const
{
    return size;
}

size_t AtomTable::getNumNodes() const
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    return nodeIndex.size();
}

size_t AtomTable::getNumLinks() const
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    return linkIndex.size();
}

size_t AtomTable::getNumAtomsOfType(Type type, bool subclass) const
{ 
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    return typeIndex.getNumAtomsOfType(type, subclass);
}

Handle AtomTable::getRandom(RandGen *rng) const
{
    size_t x = rng->randint(getSize());

    Handle randy(Handle::UNDEFINED);

    // XXX TODO it would be considerably more efficient to go into the
    // the type index, and decrement x by the size of the index for
    // each type.  This would speed up the algo by about 100 (by about
    // the number of types that are in use...).
    foreachHandleByType(
        [&](Handle h)->void {
            if (0 == x) randy = h;
            x--;
        },
        ATOM, true);
    return randy;
}

AtomPtrSet AtomTable::extract(Handle& handle, bool recursive)
{
    AtomPtrSet result;

    // Make sure the atom is fully resolved before we go about
    // deleting it.
    handle = getHandle(handle);
    AtomPtr atom(handle);
    if (!atom || atom->isMarkedForRemoval()) return result;

    // Perhaps the atom is not in any table? Or at least, not in this
    // atom table? Its a user-error if the user is trying to extract
    // atoms that are not in this atomspace, but we're going to be
    // silent about this error -- it seems pointless to throw.
    if (atom->getAtomTable() != this) return result;

    // Lock before fetching the incoming set. Since getting the
    // incoming set also grabs a lock, we need this mutex to be
    // recursive. We need to lock here to avoid confusion if multiple
    // threads are trying to delete the same atom.
    std::unique_lock<std::recursive_mutex> lck(_mtx);

    if (atom->isMarkedForRemoval()) return result;
    atom->markForRemoval();

    // If recursive-flag is set, also extract all the links in the atom's
    // incoming set
    if (recursive) {
        // We need to make a copy of the incoming set because the
        // recursive call will trash the incoming set when the atom
        // is removed.
        IncomingSet is(handle->getIncomingSet());

        IncomingSet::iterator is_it = is.begin();
        IncomingSet::iterator is_end = is.end();
        for (; is_it != is_end; ++is_it)
        {
            Handle his(*is_it);
            DPRINTF("[AtomTable::extract] incoming set: %s",
                 (his) ? his->toString().c_str() : "INVALID HANDLE");

            // Something is seriously screwed up if the incoming set
            // is not in this atomtable, and its not a child of this
            // atom table.  So flag that as an error; it will assert
            // a few dozen lines later, below.
            AtomTable* other = his->getAtomTable();
            if (other and other != this and not other->inEnviron(handle)) {
                logger().warn() << "AtomTable::extract() internal error, "
                                << "non-DAG membership.";
            }
            if (not his->isMarkedForRemoval()) {
                DPRINTF("[AtomTable::extract] marked for removal is false");
                if (other) {
                    AtomPtrSet ex = other->extract(his, true);
                    result.insert(ex.begin(), ex.end());
                }
            }
        }
    }

    // The check is done twice: the call to getIncomingSetSize() can
    // return a non-zero value if the incoming set has weak pointers to
    // deleted atoms. Thus, a second check is made for strong pointers,
    // since getIncomingSet() converts weak to strong.
    if (0 < handle->getIncomingSetSize())
    {
        IncomingSet iset(handle->getIncomingSet());
        if (0 < iset.size())
        {
            if (not recursive)
            {
                // User asked for a non-recursive remove, and the
                // atom is still referenced. So, do nothing.
                handle->unsetRemovalFlag();
                return result;
            }

            // Check for an invalid condition that should not occur. See:
            // https://github.com/opencog/opencog/commit/a08534afb4ef7f7e188e677cb322b72956afbd8f#commitcomment-5842682
            size_t ilen = iset.size();
            for (size_t i=0; i<ilen; i++)
            {
                // Its OK if the atom being extracted is in a link
                // that is not currently in any atom space, or if that
                // link is in a child subspace, in which case, we
                // extract from the child.
                //
                // A bit of a race can happen: when the unlock is
                // done below, to send the removed signal, another
                // thread can sneak in and get to here, if it is
                // deleting a different atom with an overlapping incoming
                // set.  Since the incoming set hasn't yet been updated
                // (that happens after re-acquiring the lock),
                // it will look like the incoming set has not yet been
                // fully cleared.  Well, it hasn't been, but as long as
                // we are marked for removal, things should end up OK.
                //
                // XXX this might not be exactly thread-safe, if
                // other atomspaces are involved...
                if (iset[i]->getAtomTable() != NULL and
                    (not iset[i]->getAtomTable()->inEnviron(handle) or
                     not iset[i]->isMarkedForRemoval()))
                {
                    Logger::Level lev = logger().getBackTraceLevel();
                    logger().setBackTraceLevel(Logger::ERROR);
                    logger().warn() << "AtomTable::extract() internal error";
                    logger().warn() << "Non-empty incoming set of size "
                                    << ilen << " First trouble at " << i;
                    logger().warn() << "This atomtable=" << ((void*) this)
                                    << " other atomtale=" << ((void*) iset[i]->getAtomTable())
                                    << " inEnviron=" << iset[i]->getAtomTable()->inEnviron(handle);
                    logger().warn() << "This atom: " << handle->toString();
                    for (size_t j=0; j<ilen; j++) {
                        logger().warn() << "Atom j=" << j << " " << iset[j]->toString();
                        logger().warn() << "Marked: " << iset[j]->isMarkedForRemoval()
                                        << " Table: " << ((void*) iset[j]->getAtomTable());
                    }
                    logger().setBackTraceLevel(lev);
                    atom->unsetRemovalFlag();
                    throw RuntimeException(TRACE_INFO,
                        "Internal Error: Cannot extract an atom with "
                        "a non-empty incoming set!");
                }
            }
        }
    }

    // Issue the atom removal signal *BEFORE* the atom is actually
    // removed.  This is needed so that certain subsystems, e.g. the
    // Agent system activity table, can correctly manage the atom;
    // it needs info that gets blanked out during removal.
    lck.unlock();
    _removeAtomSignal(atom);
    lck.lock();

    // Decrements the size of the table
    size--;
    _atom_set.erase(handle);

    Atom* pat = atom.operator->();
    nodeIndex.removeAtom(pat);
    linkIndex.removeAtom(atom);
    typeIndex.removeAtom(pat);
    LinkPtr lll(LinkCast(atom));
    if (lll) {
        for (AtomPtr a : lll->_outgoing) {
            a->remove_atom(lll);
        }
    }
    importanceIndex.removeAtom(pat);

    // XXX Setting the atom table causes AVChanged signals to be emitted.
    // We should really do this unlocked, but I'm too lazy to fix, and
    // am hoping no one will notice. This will probably need to be fixed
    // someday.
    atom->setAtomTable(NULL);

    result.insert(atom);
    return result;
}

// This is the resize callback, when a new type is dynamically added.
void AtomTable::typeAdded(Type t)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    //resize all Type-based indexes
    nodeIndex.resize();
    linkIndex.resize();
    typeIndex.resize();
}

