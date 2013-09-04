/*
 * opencog/xml/NMXmlExporter.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Rodrigo Barra
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

#ifndef _OPENCOG_NMXML_EXPORTER_H
#define _OPENCOG_NMXML_EXPORTER_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/persist/xml/NMXmlParser.h>

namespace opencog
{

/**
 * This class is used to export XML from a subset of Atoms.
 */
class NMXmlExporter
{
    friend class AtomSpace;

private:
    /**
     * Stores the default initial size for the buffer.
     */
    static const int DEFAULT_BUFF_SIZE = 1 << 16;

    /**
     * Find the set of atoms to be exported based on an
     * initial seed.
     */
    UnorderedHandleSet *findExportables(const UnorderedHandleSet&);

    /**
     * Used to explor a subgraph of the AtomTable.
     */
    void findExportables(UnorderedHandleSet *, UnorderedHandleSet *, Handle);

    /**
     * Exports a subset of the AtomSpace to XML.
     * @param hs The set of Handles to be exported.
     *        After the handles are exported
     *        the UnorderedHandleSet is deleted.
     * @return An string describing the subset of Atoms in XML.
     */
    std::string toXML(const UnorderedHandleSet* hs);

    /**
     * Exports an Atom and the Atoms in its outgoingset.
     */
    void exportAtom(Handle, bool [], std::string&, bool = false);

    const AtomSpace* as;
public:

    NMXmlExporter(const AtomSpace* _as);
    ~NMXmlExporter();

    /**
     * Exports a subset of the AtomSpace to XML.
     * @param hs The HandleSeq of Handles to be exported.
     * @return An string describing the subset of Atoms in XML.
     */
    std::string toXML(const HandleSeq& hs);

};

} // namespace opencog

#endif // _OPENCOG_NMXML_EXPORTER_H
