/**
 * NMXmlExporter.h
 *
 *
 * Author: Rodrigo Barra
 * Copyright(c) 2004 Vettatech Technologies
 * All rights reserved.
 */
#ifndef NMXMLEXPORTER_H
#define NMXMLEXPORTER_H

#include <HandleEntry.h>
#include <HandleSet.h>
#include "NMXmlParser.h"

/**
 * This class is used to export XML from a subset of Atoms.
 */
class NMXmlExporter {

private:
	/**
	 * Stores the default initial size for the buffer.
	 */
	static const int DEFAULT_BUFF_SIZE = 1<<16;

	/**
	 * Find the set of atoms to be exported based on an 
	 * initial seed.
	 */
	HandleSet *findExportables(HandleEntry *);

	/**
	 * Used to explor a subgraph of the AtomTable.
	 */
	void findExportables(HandleSet *, HandleSet *, Atom *);


	/**
	 * Exports a subset of the AtomTable to XML.
	 * @param The set of Handles to be exported.
	 *        After the handles are exported
	 *        the HandleSet is deleted.
	 * @return An string describing the subset of Atoms in XML.
	 */
	std::string toXML(HandleSet  *);


	/**
	 * Exports an Atom and the Atoms in its outgoingset.
	 */
	void exportAtom(Handle, bool [], std::string&, bool = false);

public:

	/**
	 * Exports a subset of the AtomTable to XML.
	 * @param The set of Handles from where the subset 
	 *        should be built. After the handles are exported
	 *        the HandleEntry is deleted.
	 * @return A string describing the subset of Atoms in XML.
	 */
	std::string toXML(HandleEntry *);

};

#endif //NMXMLEXPORTER_H
