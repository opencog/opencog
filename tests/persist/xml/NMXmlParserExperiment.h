/**
 * tests/xml/NMXmlParserExperiment.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 *
 * Written by Rodrigo Barra
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

 * Copyright(c) 2003 Rodrigo Barra
 * All rights reserved.
 */

#ifndef _OPENCOG_NM_XML_PARSER_EXPERIMENT_H
#define _OPENCOG_NM_XML_PARSER_EXPERIMENT_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/types.h>

#define NNMXMLXMLEXPERIMENTS 1

class NMXmlParserExperiment {

private:

	static int currentExperiment;
	static const char *expContents[NNMXMLXMLEXPERIMENTS];

	//CHECKING
	static bool checkExp0();
	static bool checkExp1();

	static opencog::Handle one;
	static opencog::Handle two;
	static opencog::Handle link_one_two;
	static opencog::Handle hihger_order_link;

    static opencog::AtomSpace* atomSpace;

public:

    static bool noCheck;
	static char *currentFileName;

	static void initStaticVars();
	static int getNExperiments();
	static void createExperiment(int, opencog::AtomSpace*);
	static bool checkExperiment();
	static opencog::AtomSpace* destroyExperiment(bool = true);
	static opencog::AtomSpace* cleanupAtomSpace();
    static opencog::AtomSpace* getAtomSpace();
};

#endif // _OPENCOG_NM_XML_PARSER_EXPERIMENT_H
