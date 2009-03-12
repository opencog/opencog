/*
 * examples/modules/CustomAtomTypesTester.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "CustomAtomTypesTester.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/HandleEntry.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/types.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

#include "atom_types.h"

using namespace opencog;

void CustomAtomTypesTester::createAtoms()
{
    logger().info("[CustomAtomTypesTester.createAtoms]");

    AtomTable& at = const_cast<AtomTable&>(server().getAtomSpace()->getAtomTable());

    Node* number_node = new Node(NUMBER_NODE, "1");
    Handle number_handle = at.add(number_node);
    logger().info("[CustomAtomTypesTester] new node: %s (%d)", number_node->toShortString().c_str(), number_handle.value());

    Node* foo_node = new Node(FOO_NODE, "foo");
    Handle foo_handle = at.add(foo_node);
    logger().info("[CustomAtomTypesTester] new node: %s (%d)", foo_node->toShortString().c_str(), foo_handle.value());

    Node* bar_node = new Node(BAR_NODE, "bar");
    Handle bar_handle = at.add(bar_node);
    logger().info("[CustomAtomTypesTester] new node: %s (%d)", bar_node->toShortString().c_str(), bar_handle.value());

    std::vector<Handle> v;
    v.push_back(foo_handle);
    v.push_back(bar_handle);
    Link* foobar_link = new Link(FOOBAR_LINK, v);
    Handle foobar_handle = at.add(foobar_link);
    logger().info("[CustomAtomTypesTester] new link: %s (%d)", foobar_link->toShortString().c_str(), foobar_handle.value());

    Link* list_link = new Link(LIST_LINK, v);
    Handle list_handle = at.add(list_link);
    logger().info("[CustomAtomTypesTester] new link: %s (%d)", list_link->toShortString().c_str(), list_handle.value());
}

static void dumpHandleEntry(HandleEntry* he, const char *id)
{
    for (HandleEntry* it = he; it != NULL; it = it->next) {
        Atom* a = TLB::getAtom(it->handle);
        logger().info("[CustomAtomTypesTester] %s: %s", id, a->toShortString().c_str());
    }
    delete he;
}

void CustomAtomTypesTester::dumpAtoms()
{
    logger().info("[CustomAtomTypesTester.dumpAtoms]");
    AtomTable& at = const_cast<AtomTable&>(server().getAtomSpace()->getAtomTable());
    dumpHandleEntry(at.getHandleSet(BAR_NODE            ), "bar node");
    dumpHandleEntry(at.getHandleSet(NODE,           true), "node");
    dumpHandleEntry(at.getHandleSet(FOOBAR_LINK         ), "foobar link");
    dumpHandleEntry(at.getHandleSet(UNORDERED_LINK, true), "unordered link");
    dumpHandleEntry(at.getHandleSet(LINK,           true), "link");
}
