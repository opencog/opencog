/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
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

#ifndef IATOMTABLEWRAPPER_H
#define IATOMTABLEWRAPPER_H

#include "AtomLookupProvider.h"

namespace reasoning
{

struct iAtomSpaceWrapper : public reasoning::AtomLookupProvider
{
    virtual pHandle addAtom(tree<Vertex>&, const TruthValue& tvn,
                            bool fresh, bool managed = true)=0;
    virtual pHandle addLink(Type T, const pHandleSeq& hs,
                            const TruthValue& tvn, bool fresh=false,
                            bool managed = true)=0;
    virtual pHandle addNode(Type T, const std::string& name,
                            const TruthValue& tvn, bool fresh=false,
                            bool managed = true)=0;

    virtual unsigned int getUniverseSize() const=0;
};

}

#endif
