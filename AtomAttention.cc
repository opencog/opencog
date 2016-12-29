/*
 * opencog/atoms/base/Atom.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
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

#include <opencog/atoms/base/Atom.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/attentionbank/ImportanceIndex.h>

namespace opencog {

#define _avmtx _mtx

// ==============================================================

AttentionValuePtr Atom::getAttentionValue() const
{
    if (NULL == _atomTable) return AttentionValue::DEFAULT_AV();
    Atom* a = (Atom*) this;
    return attentionbank(_atomTable->getAtomSpace()).get_av(a->getHandle());
}

// XXX TODO This is insane. All this needs to be moved to the attention bank.
void Atom::setAttentionValue(AttentionValuePtr av)
{
    // If the atom free-floating, we are done.
    if (NULL == _atomTable) return;

    AttentionValuePtr local(getAttentionValue());
    attentionbank(_atomTable->getAtomSpace()).change_av(getHandle(), av);

    // Get old and new bins.
    int oldBin = ImportanceIndex::importanceBin(local->getSTI());
    int newBin = ImportanceIndex::importanceBin(av->getSTI());

    // If the atom importance has changed its bin,
    // update the importance index.
    if (oldBin != newBin) {
        attentionbank(_atomTable->getAtomSpace()).updateImportanceIndex(getHandle(), oldBin);
    }

    // Notify any interested parties that the AV changed.
    AVCHSigl& avch = attentionbank(_atomTable->getAtomSpace()).getAVChangedSignal();
    avch(getHandle(), local, av);
}

void Atom::chgVLTI(int unit)
{
    AttentionValuePtr old_av = getAttentionValue();
    AttentionValuePtr new_av = createAV(
        old_av->getSTI(),
        old_av->getLTI(),
        old_av->getVLTI() + unit);
    setAttentionValue(new_av);
}

// ==============================================================

} // ~namespace opencog
